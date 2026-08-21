"""The surface a policy author writes against. This is the stable part of the project.

Everything else here can be rewritten; this file is a promise. A policy written today should
keep working when pose noise, a lossy radio, or a ROS 2 backend appear behind the seams.

Writing a policy
----------------

    from safmc_sim.api import Observation, Policy, VelocityBody, Land, register_policy

    @register_policy("wall_hugger")
    class WallHugger(Policy):
        def reset(self):
            self.turning = False

        def step(self, obs: Observation) -> Command:
            if obs.tof.min_range_m < 0.5:
                return VelocityBody(vx=0.0, yaw_rate=0.8)
            if any(m.kind == "victim" and m.range_m < 0.8 for m in obs.markers):
                return Land()
            self.publish("heading", obs.pose.theta)      # visible to peers NEXT tick
            return VelocityBody(vx=0.45)

Three rules the design enforces rather than requests
----------------------------------------------------

**A policy can only see what a drone could see.** ``Observation`` is frozen and holds plain
data. There is no route from it to the environment, the arena, the mission, or another agent's
object, so "accidentally reading ground truth" is not a mistake that can be made (R-POL-3,
R-POL-4). What it does hold is genuinely public: the drone's own state, its sensors, the
published field dimensions, and whatever peers chose to broadcast.

**Randomness is injected, never ambient.** Each policy is handed its own
``numpy.random.Generator``. Touching ``numpy.random`` directly breaks seeded replay and is
forbidden by R-DET-2.

**A policy that raises, aborts the run.** There is no catch-and-hover. A silently degraded
agent produces a plausible-looking result that is wrong, which is worse than a stack trace
(R-POL-9).
"""

from __future__ import annotations

import warnings
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping, Union

import numpy as np

from .constants import CRUISE_ALT_M, CRUISE_SPEED_MS
from .errors import ConfigError, PolicyError
from .sensors.marker_cam import MarkerDetection
from .sensors.tof_ring import ToFScan

__all__ = [
    "Pose",
    "ArenaInfo",
    "Lifecycle",
    "Observation",
    "Command",
    "Takeoff",
    "VelocityBody",
    "VelocityWorld",
    "PositionWorld",
    "Hold",
    "Land",
    "Policy",
    "register_policy",
    "get_policy",
    "policy_names",
]


# ------------------------------------------------------------------------------------------
# Observation
# ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Pose:
    """ARENA-frame pose. In v0.1 this is ground truth; see ADR-0003 for what that costs."""

    x: float
    y: float
    z: float
    theta: float

    @property
    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y])


class Lifecycle:
    """Where a drone is in its one-way trip from the Start Area to the ground.

    ``LANDED`` is terminal and deliberate: the rules require a rescuing drone to stay "until
    the end of the mission", so landing spends the drone. ``CRASHED`` is terminal too. Both
    make the fleet a depleting resource, which is the actual strategic problem.
    """

    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    FLYING = "FLYING"
    LANDING = "LANDING"
    LANDED = "LANDED"
    CRASHED = "CRASHED"

    ALL = (IDLE, TAKEOFF, FLYING, LANDING, LANDED, CRASHED)
    TERMINAL = (LANDED, CRASHED)
    AIRBORNE = (TAKEOFF, FLYING, LANDING)


@dataclass(frozen=True)
class ArenaInfo:
    """Published field geometry. Public knowledge, so a policy may have it.

    Deliberately not the :class:`ArenaSpec`: obstacle and target positions are exactly what a
    policy is supposed to discover.
    """

    width_m: float
    depth_m: float
    ceiling_m: float
    start_area_depth_m: float
    run_duration_s: float


@dataclass(frozen=True)
class Observation:
    """Everything one drone knows at one tick."""

    agent_id: str
    tick: int
    sim_time_s: float

    pose: Pose
    velocity_xy: tuple[float, float]
    lifecycle: str

    tof: ToFScan
    markers: tuple[MarkerDetection, ...]
    """Markers currently detected. Empty on ticks where the detector did not sample."""

    peers: Mapping[str, Mapping[str, Any]]
    """Blackboard as of the START of this tick, keyed by agent id.

    In v0.1 this is a perfect shared store (ADR-0003). Every agent sees the same snapshot, so
    results cannot depend on the order agents happen to be stepped (R-POL-8).
    """

    arena: ArenaInfo
    tof_stale_ticks: int = 0
    """Ticks since the ToF ring last sampled. Non-zero only if the sensor is decimated."""

    marker_stale_ticks: int = 0

    @property
    def in_start_area(self) -> bool:
        return 0.0 <= self.pose.y <= self.arena.start_area_depth_m

    @property
    def time_remaining_s(self) -> float:
        return max(0.0, self.arena.run_duration_s - self.sim_time_s)


# ------------------------------------------------------------------------------------------
# Commands -- exactly the real firmware's action set, and no more (R-POL-5)
# ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Takeoff:
    """Arm, take off, and climb. Subject to the two-wave rule (R-MISS-6)."""

    altitude_m: float = CRUISE_ALT_M


@dataclass(frozen=True)
class VelocityBody:
    """Body-frame velocity plus yaw rate. Nearest analogue to a reactive controller.

    Maps to ``mavlink_set_velocity_ned`` after rotating into NED.
    """

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class VelocityWorld:
    """ARENA-frame horizontal velocity, held altitude, absolute yaw.

    Maps to ``mavlink_set_velocity_xy_position_z`` -- the workhorse the real nav loop uses for
    cruise (nav_task.c:383).
    """

    vx: float = 0.0
    vy: float = 0.0
    z: float = CRUISE_ALT_M
    yaw: float | None = None
    """``None`` holds the current heading. The firmware warns that commanding yaw 0.0 means
    'face North', which is rarely what a caller means."""


@dataclass(frozen=True)
class PositionWorld:
    """ARENA-frame position setpoint. Maps to ``mavlink_set_position_ned``."""

    x: float
    y: float
    z: float = CRUISE_ALT_M
    yaw: float | None = None
    speed_ms: float = CRUISE_SPEED_MS


@dataclass(frozen=True)
class Hold:
    """Stop and hold position. Maps to ``mavlink_set_hold``."""


@dataclass(frozen=True)
class Land:
    """Descend and land here, permanently. This is how a drone scores -- and is spent."""


Command = Union[Takeoff, VelocityBody, VelocityWorld, PositionWorld, Hold, Land]

COMMAND_TYPES = (Takeoff, VelocityBody, VelocityWorld, PositionWorld, Hold, Land)


# ------------------------------------------------------------------------------------------
# Policy
# ------------------------------------------------------------------------------------------


class Policy(ABC):
    """One instance per drone, constructed once per run.

    Subclasses implement :meth:`step` and optionally :meth:`reset`. Per-drone state lives on
    ``self``; there is no shared mutable state between policy instances except the blackboard,
    which is explicit and snapshot-consistent.
    """

    def __init__(
        self,
        agent_id: str,
        config: Mapping[str, Any],
        rng: np.random.Generator,
        arena: ArenaInfo,
    ) -> None:
        self.agent_id = agent_id
        self.config = MappingProxyType(dict(config))
        self.rng = rng
        self.arena = arena
        self._outbox: dict[str, Any] = {}

    def reset(self) -> None:
        """Clear per-episode state. Called once before the first tick."""

    @abstractmethod
    def step(self, obs: Observation) -> Command:
        """Return this tick's command. Must not mutate ``obs``."""

    # -- peer communication ---------------------------------------------------------------

    def publish(self, key: str, value: Any) -> None:
        """Broadcast a value to peers. Visible to everyone -- including you -- **next** tick.

        The one-tick delay is not an approximation of radio latency; it is what makes the
        result independent of the order agents are stepped in (R-POL-8). A same-tick read
        would make agent 0's publication visible to agent 1 but not the reverse.
        """
        if not isinstance(key, str):
            raise PolicyError(f"blackboard keys must be strings, got {type(key).__name__}")
        self._outbox[key] = value

    def drain_outbox(self) -> dict[str, Any]:
        """Runner-internal: take and clear this tick's publications."""
        out, self._outbox = self._outbox, {}
        return out


# ------------------------------------------------------------------------------------------
# Registry
# ------------------------------------------------------------------------------------------

_REGISTRY: dict[str, type[Policy]] = {}


def register_policy(name: str):
    """Register a policy under a name. Re-registering overwrites, with a warning.

    Deliberately different from ir-sim's behaviour registry in two ways (R-POL-6): the key is
    a name alone rather than ``(kinematics, name)``, so strategy is not welded to the dynamics
    model; and a duplicate warns instead of raising, so re-running a notebook cell works.
    """

    def decorator(cls: type[Policy]) -> type[Policy]:
        if not isinstance(name, str) or not name:
            raise ConfigError("policy name must be a non-empty string")
        if not (isinstance(cls, type) and issubclass(cls, Policy)):
            raise ConfigError(f"{cls!r} is not a Policy subclass")
        if name in _REGISTRY and _REGISTRY[name] is not cls:
            warnings.warn(
                f"policy {name!r} re-registered: {_REGISTRY[name].__name__} -> {cls.__name__}",
                stacklevel=2,
            )
        _REGISTRY[name] = cls
        return cls

    return decorator


def get_policy(name: str) -> type[Policy]:
    if name not in _REGISTRY:
        raise ConfigError(
            f"unknown policy {name!r}. Registered: {sorted(_REGISTRY) or '(none)'}. "
            f"Did you forget to import the module that registers it?"
        )
    return _REGISTRY[name]


def policy_names() -> tuple[str, ...]:
    return tuple(sorted(_REGISTRY))
