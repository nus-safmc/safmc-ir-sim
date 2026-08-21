"""The 2.5D quadrotor motion model, registered into ir-sim.

State is ``[x, y, theta, z, vx, vy]``:

    x, y     ARENA position, metres
    theta    ARENA yaw, radians, wrapped to (-pi, pi]
    z        altitude, metres above the floor
    vx, vy   ARENA-frame velocity, metres/second -- carried as *state* so that a
             first-order lag between commanded and actual velocity can be modelled

Action is ``[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]``, ARENA frame for the linear channels.
Body-frame commands are rotated into ARENA by the command resolver before they reach here,
so this handler has exactly one convention to honour.

Why a lag at all: the real drone receives a velocity setpoint over MAVLink and PX4 tracks it
with closed-loop dynamics. Modelling that as an instantaneous velocity change would let a
policy reverse direction in one tick, which flatters reactive policies. A single first-order
pole is the cheapest model that removes that artefact. See assumption A-2.

ir-sim integration notes
------------------------
* Registered via ``@register_kinematics``, the supported extension point
  (irsim/lib/handler/kinematics_handler.py:22-48). No fork, no monkey-patch.
* ir-sim's ``KinematicsFactory.create_kinematics`` has a **closed signature**, so extra
  parameters cannot come from YAML -- ``kinematics: {name: quad25d, tau: 0.5}`` raises
  ``TypeError``. Parameters are therefore injected after construction by
  :func:`configure_robots`, which fails loudly if a robot is not running this model.
* Yaw is integrated and wrapped *here*. The reference implementation of arXiv:2607.25195
  integrates yaw outside ``env.step()`` (sdlw/run_simulation.py:311-314), which skips
  wrapping and leaves collision geometry derived from a stale heading. R-DRONE-3 forbids it.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
from irsim.lib import KinematicsHandler, register_kinematics

from .constants import (
    CEILING_M,
    CLIMB_RATE_MAX_MS,
    CRUISE_SPEED_MS,
    VELOCITY_TAU_S,
    YAW_RATE_MAX,
)
from .errors import ConfigError
from .frames import wrap_pi

__all__ = ["QuadParams", "Quad25D", "configure_robots", "KINEMATICS_NAME"]

KINEMATICS_NAME = "quad25d"

# Indices into the state column vector. Named so no code indexes by bare integer.
IX, IY, ITHETA, IZ, IVX, IVY = range(6)
STATE_DIM = 6
ACTION_DIM = 4


@dataclass(frozen=True)
class QuadParams:
    """Dynamics parameters. Frozen: a run's dynamics cannot change under it."""

    tau_s: float = VELOCITY_TAU_S
    """First-order lag time constant on horizontal velocity (A-2)."""

    speed_max_ms: float = CRUISE_SPEED_MS
    """Horizontal speed cap, applied to the commanded velocity *vector*, not per axis."""

    climb_rate_max_ms: float = CLIMB_RATE_MAX_MS
    """Vertical speed cap (A-3). Applied directly; altitude has no modelled lag."""

    yaw_rate_max: float = YAW_RATE_MAX
    """Yaw rate cap, radians/second."""

    ceiling_m: float = CEILING_M
    """Hard altitude clamp. The rules cap flight at 1.4 m."""

    def __post_init__(self) -> None:
        if self.tau_s <= 0.0:
            raise ConfigError(f"tau_s must be > 0, got {self.tau_s}")
        for name in ("speed_max_ms", "climb_rate_max_ms", "yaw_rate_max", "ceiling_m"):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0.0:
                raise ConfigError(f"{name} must be finite and > 0, got {value}")


@register_kinematics(KINEMATICS_NAME)
class Quad25D(KinematicsHandler):
    """Velocity-commanded 2.5D quadrotor with first-order horizontal velocity lag."""

    action_dim = ACTION_DIM
    min_state_dim = 3
    state_dim = STATE_DIM

    # ir-sim clips the action to [vel_min, vel_max] before calling step(). We keep those
    # generous and enforce the real limits ourselves, because ir-sim's per-axis clip cannot
    # express a limit on the speed *vector*. Acceleration limiting is disabled here (inf)
    # for two reasons: the lag already provides it, and a finite `acce` makes ir-sim emit a
    # WARNING on every clipped step (object_base.py:710-718), which floods the log.
    vel_max = [np.inf, np.inf, np.inf, np.inf]
    vel_min = [-np.inf, -np.inf, -np.inf, -np.inf]
    acce = [np.inf, np.inf, np.inf, np.inf]

    color = "royalblue"
    obstacle_color = "k"
    description = None
    show_arrow = True

    def __init__(self, name, noise=False, alpha=None, **_ignored):
        # Signature is fixed by ir-sim's KinematicsFactory; see the module docstring.
        super().__init__(name, noise, alpha)
        self.params = QuadParams()

    # -- the four methods ir-sim requires -------------------------------------------------

    def step(self, state, velocity, step_time):
        """Integrate one tick. ``state`` is (6,1), ``velocity`` is (4,1)."""
        p = self.params
        dt = float(step_time)

        vx_cmd = float(velocity[0, 0])
        vy_cmd = float(velocity[1, 0])
        vz_cmd = float(velocity[2, 0])
        yaw_rate_cmd = float(velocity[3, 0])

        # Cap the commanded horizontal velocity as a vector, preserving direction. A per-axis
        # clip would let a diagonal command exceed the speed limit by sqrt(2).
        speed_cmd = np.hypot(vx_cmd, vy_cmd)
        if speed_cmd > p.speed_max_ms:
            scale = p.speed_max_ms / speed_cmd
            vx_cmd *= scale
            vy_cmd *= scale

        vz_cmd = float(np.clip(vz_cmd, -p.climb_rate_max_ms, p.climb_rate_max_ms))
        yaw_rate_cmd = float(np.clip(yaw_rate_cmd, -p.yaw_rate_max, p.yaw_rate_max))

        # First-order lag, explicit Euler. Guard the gain at 1.0 so a tick longer than tau
        # cannot overshoot into oscillation -- with dt > tau the correct discrete behaviour is
        # "reach the command this tick", not "overshoot past it".
        gain = min(dt / p.tau_s, 1.0)
        vx = float(state[IVX, 0]) + gain * (vx_cmd - float(state[IVX, 0]))
        vy = float(state[IVY, 0]) + gain * (vy_cmd - float(state[IVY, 0]))

        theta = wrap_pi(float(state[ITHETA, 0]) + yaw_rate_cmd * dt)
        z = float(np.clip(float(state[IZ, 0]) + vz_cmd * dt, 0.0, p.ceiling_m))

        return np.array(
            [
                [float(state[IX, 0]) + vx * dt],
                [float(state[IY, 0]) + vy * dt],
                [theta],
                [z],
                [vx],
                [vy],
            ]
        )

    def velocity_to_xy(self, state, velocity):
        """ARENA-frame velocity. Consumed by ir-sim's RVO/social-force neighbours."""
        return np.array([[float(state[IVX, 0])], [float(state[IVY, 0])]])

    def compute_max_speed(self, vel_max):
        return float(self.params.speed_max_ms)

    def compute_heading(self, state, velocity):
        return float(state[ITHETA, 0])


def configure_robots(robot_list, params: QuadParams) -> None:
    """Inject dynamics parameters into every robot after ``irsim.make()``.

    Necessary because ir-sim's kinematics factory has a closed signature and cannot forward
    custom parameters from YAML. Raises if any robot is not running this model, rather than
    silently leaving a differential-drive robot in the fleet -- ir-sim downgrades an unknown
    ``kinematics.name`` to a differential drive with only a warning
    (kinematics_handler.py:462-468), which is exactly the failure this check catches.
    """
    if not robot_list:
        raise ConfigError("no robots in the environment to configure")
    for robot in robot_list:
        handler = getattr(robot, "kf", None)
        if not isinstance(handler, Quad25D):
            raise ConfigError(
                f"robot {getattr(robot, 'name', '?')} is running "
                f"{type(handler).__name__}, not Quad25D. Its YAML almost certainly has a "
                f"misspelled kinematics name -- ir-sim silently falls back to a "
                f"differential drive when it does not recognise one."
            )
        handler.params = replace(params)
