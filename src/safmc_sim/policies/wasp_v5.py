"""WASP v5 policy for SAFMC.

Controller and seed helpers below are copied from the user-provided internal WASP
source at revision 6f8b532. SAFMC adapter code follows the copied core.
"""

"""Versioned seed derivation helpers for WASP randomness domains.

The helpers implement ``seed_scheme: v1`` with explicit integer domains and
coordinates. Seed derivation is deterministic and serialises to Python ``int``
after drawing a single 32-bit unsigned integer.
"""


import numpy as np

SEED_SCHEME_VERSION: str = "v1"
TARGET_DOMAIN: int = 1
SIMULATOR_DOMAIN: int = 2
CONTROLLER_DOMAIN: int = 3

_UINT32_MAX = 2**32 - 1


def _validate_seed_component(value: object, name: str) -> None:
    """Validate a seed component for domain-separated derivation.

    Components use Python's strict ``int`` type (rejecting ``bool``), must be
    integer-valued, and must lie in the inclusive ``uint32`` range.
    """
    if type(value) is not int:
        raise ValueError(f"{name} must be an int, got {type(value).__name__}")
    if not (0 <= value <= _UINT32_MAX):
        raise ValueError(f"{name}={value} is outside [0, 2**32 - 1]")


def derive_effective_seed(master_seed: int, domain: int, *coordinates: int) -> int:
    """Derive a stable, pure, uint32-compatible effective seed.

    Uses ``SeedSequence([master_seed, domain, *coordinates])`` and returns a
    single-element ``np.uint32`` sample as a Python ``int`` suitable for
    reproducible controller/simulator storage.
    """
    _validate_seed_component(master_seed, "master_seed")
    _validate_seed_component(domain, "domain")
    for index, coordinate in enumerate(coordinates):
        _validate_seed_component(coordinate, f"coordinates[{index}]")

    seed_seq = np.random.SeedSequence([master_seed, domain, *coordinates])
    return int(seed_seq.generate_state(1, dtype=np.uint32)[0])


def make_controller_rng(master_seed: int, formation_slot: int) -> tuple[int, np.random.Generator]:
    """Derive a controller seed for a formation slot and return its RNG."""
    effective_seed = derive_effective_seed(master_seed, CONTROLLER_DOMAIN, formation_slot)
    return effective_seed, np.random.default_rng(effective_seed)


"""WASP controller core.

Wandering Autonomous Sensor-driven Permeation.

A memoryless reactive coverage algorithm for nano drones operating in unknown
indoor environments. All navigation decisions derive from instantaneous sensor
readings and a small number of scalar state variables. No maps, no SLAM, no
global memory.

This module has no coupling to ir-sim or any simulator. The adapter in
beh_omni_wasp.py maps simulator state to WaspObservation and WaspAction back
to simulator commands.
"""


import math
from dataclasses import dataclass
from enum import Enum
from typing import Mapping

import numpy as np
from scipy.stats import vonmises


def _require_finite(name: str, value: float) -> float:
    """Return value as float, rejecting NaN and infinities."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


V5_CONTROLLER_PARAMETER_NAMES = (
    "KAPPA_MAX",
    "AVOIDANCE_RANGE",
    "AVOIDANCE_GAIN",
    "LEVY_EXPONENT",
    "BACK_SENSOR_WEIGHT",
)
"""Immutable schema of active controller parameter names."""


class V5ControllerMode(str, Enum):
    """Categorical geometry policy used by :class:`WaspV5Controller`."""

    UNIFORM_LEVY = "uniform_levy"
    CIRCULAR_RESULTANT = "circular_resultant"
    PCA_HEADING = "pca_heading"


DEFAULT_V5_CONTROLLER_MODE = V5ControllerMode.PCA_HEADING


def normalize_v5_controller_mode(value: object) -> V5ControllerMode:
    """Return a canonical controller mode, rejecting noncanonical inputs."""
    if isinstance(value, V5ControllerMode):
        return value
    if type(value) is str:
        try:
            return V5ControllerMode(value)
        except ValueError:
            pass
    allowed = ", ".join(mode.value for mode in V5ControllerMode)
    raise ValueError(f"controller_mode must be one of: {allowed}")


def validate_controller_parameters(
    parameters: Mapping[str, object], *, allow_random_levy_exponent: bool = False,
) -> dict[str, float | None]:
    """Validate one complete controller parameter set without mutation.

    ``LEVY_EXPONENT=None`` is accepted only for construction, where the
    controller samples its per-agent exponent. Runtime callers validate the
    already-effective, finite value on each controller instead.
    """
    names = set(parameters)
    expected = set(V5_CONTROLLER_PARAMETER_NAMES)
    if names != expected:
        missing = sorted(expected - names)
        unsupported = sorted(names - expected)
        details = []
        if missing:
            details.append(f"missing: {', '.join(missing)}")
        if unsupported:
            details.append(f"unsupported: {', '.join(unsupported)}")
        raise ValueError(f"invalid controller parameter set ({'; '.join(details)})")

    values: dict[str, float | None] = {}
    for name in V5_CONTROLLER_PARAMETER_NAMES:
        value = parameters[name]
        if name == "LEVY_EXPONENT" and value is None and allow_random_levy_exponent:
            values[name] = None
        else:
            values[name] = _require_finite(name, value)

    if values["KAPPA_MAX"] < 0.0:
        raise ValueError("KAPPA_MAX must be >= 0")
    if values["AVOIDANCE_RANGE"] <= 0.0:
        raise ValueError("AVOIDANCE_RANGE must be > 0")
    if values["AVOIDANCE_GAIN"] < 0.0:
        raise ValueError("AVOIDANCE_GAIN must be >= 0")
    if values["LEVY_EXPONENT"] is not None and values["LEVY_EXPONENT"] <= 1.0:
        raise ValueError("LEVY_EXPONENT must be > 1")
    if not 0.0 <= values["BACK_SENSOR_WEIGHT"] <= 1.0:
        raise ValueError("BACK_SENSOR_WEIGHT must be in [0, 1]")
    return values


def _wrap_to_pi(angle: float) -> float:
    """Wrap an angle to the range [-pi, pi]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


# ---------------------------------------------------------------------------
# I/O types
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class WaspObservation:
    """Sensor observation passed to the controller each tick.

    Attributes:
        bearings:    Body-frame beam angles, shape (N,), radians.
        ranges:      Beam range readings, shape (N,), metres.
        range_max:   Sensor saturation range (metres). Beams at this value
                     are at or beyond sensor maximum.
        delta_dist:  Distance traveled since the previous tick (metres >= 0).
                     Provided by the adapter from odometry or commanded speed.
        step_time:   Duration of this simulation tick (seconds).
        current_yaw: World-frame robot heading (radians). On real hardware
                     this is provided by the IMU/compass.
    """

    bearings: np.ndarray
    ranges: np.ndarray
    range_max: float
    delta_dist: float
    step_time: float
    current_yaw: float


@dataclass(frozen=True)
class WaspAction:
    """Velocity command produced by the controller each tick.

    All velocity components are in the robot body frame (x = forward,
    y = left). The adapter rotates them to the world frame before passing
    to the simulator.

    Attributes:
        velocity_x: Body-frame x velocity (m/s). Normalised to unit forward
                    plus avoidance; the adapter scales to max_vel.
        velocity_y: Body-frame y velocity (m/s).
        target_yaw: World-frame target heading (radians). The adapter runs
                    a P-controller to convert this to a yaw_rate command.
    """

    velocity_x: float
    velocity_y: float
    target_yaw: float


@dataclass(frozen=True)
class WaspDebug:
    """Algorithm state snapshot returned alongside WaspAction each tick.

    new_step is True only on ticks where a new Lévy step was sampled.
    h_body and step_length are 0.0 on ticks where no new step was sampled.
    All angles are in radians.
    """

    new_step: bool
    mu: float  # circular mean open direction (rad, body frame)
    kappa: float  # Von Mises concentration
    anisotropy: float  # normalised active geometry signal [0, 1]
    h_body: float  # sampled heading offset (rad, body frame); valid if new_step
    target_yaw: float  # world-frame target yaw (rad)
    step_length: float  # sampled Lévy step length (m); valid if new_step
    step_remaining: float  # distance remaining in current step (m)
    step_max: float  # maximum allowed step length from the active geometry-signal policy (m)
    forward_obstacle: bool = False


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------


class WaspV5Controller:
    """WASP v5 navigation controller.

    Implements a Lévy walk governed by a Von Mises heading distribution whose
    mean direction μ, concentration κ, and step cap are computed from the
    sensor openness ring each tick according to ``controller_mode``.

    A reactive obstacle avoidance layer computes a linear-falloff repulsive
    velocity contribution and adds it to the navigation velocity every tick.

    All parameters are constructor arguments with defaults matching the values
    in docs/algorithm.md. Override any subset via keyword arguments.

    Args:
        KAPPA_MAX:           Maximum Von Mises concentration when the active geometry signal is one.
        STEP_LENGTH_MIN:     Minimum Lévy step length (m).
        STEP_LENGTH_MAX:     Maximum Lévy step cap: fixed for uniform Lévy and PCA-heading-only modes; upper endpoint of joint active-geometry-signal scaling (m).
        AVOIDANCE_RANGE:     Beams closer than this contribute repulsion (m).
        AVOIDANCE_GAIN:      Scale factor for repulsive velocity contribution.
        LEVY_EXPONENT:       Power-law PDF exponent alpha for step length distribution.
                             P(d = i) proportional to i^{-alpha} (discrete Zipf / truncated
                             zeta), following Clementi et al. (arXiv:2004.01562v6, 2024).
                             If None, sampled once per agent from Uniform(2, 3) for
                             near-optimal parallel search without environment tuning.
        BACK_SENSOR_WEIGHT:  Multiplier for beams near the back (bearing ≈ ±π). Applied as
                             a smooth cosine taper: 1 for front beams, BACK_SENSOR_WEIGHT for
                             rear beams. Range [0, 1].
        controller_mode:     Categorical geometry policy for heading concentration and cap.
    """

    def __init__(
        self,
        KAPPA_MAX: float = 8.0,
        AVOIDANCE_RANGE: float = 0.4,
        AVOIDANCE_GAIN: float = 0.5,
        LEVY_EXPONENT: float | None = None,
        BACK_SENSOR_WEIGHT: float = 0.0,
        *,
        controller_mode: V5ControllerMode | str = DEFAULT_V5_CONTROLLER_MODE,
        rng: np.random.Generator,
    ) -> None:
        if not isinstance(rng, np.random.Generator):
            raise TypeError("rng must be an instance of numpy.random.Generator")
        self._rng = rng
        self.controller_mode = normalize_v5_controller_mode(controller_mode)

        parameters = validate_controller_parameters(
            {
                "KAPPA_MAX": KAPPA_MAX,
                "AVOIDANCE_RANGE": AVOIDANCE_RANGE,
                "AVOIDANCE_GAIN": AVOIDANCE_GAIN,
                "LEVY_EXPONENT": LEVY_EXPONENT,
                "BACK_SENSOR_WEIGHT": BACK_SENSOR_WEIGHT,
            },
            allow_random_levy_exponent=True,
        )
        self.KAPPA_MAX = parameters["KAPPA_MAX"]
        self.AVOIDANCE_RANGE = parameters["AVOIDANCE_RANGE"]
        self.AVOIDANCE_GAIN = parameters["AVOIDANCE_GAIN"]

        # Per-agent Lévy exponent: sample from U(2,3) if not explicitly provided.
        # Clementi et al. (arXiv:2004.01562v6, 2024), Theorem 1.6, shows α ~ U(2,3)
        # is near-optimal for parallel search without knowing target distance.
        if parameters["LEVY_EXPONENT"] is None:
            self.LEVY_EXPONENT = float(self._rng.uniform(2.0, 3.0))
        else:
            self.LEVY_EXPONENT = parameters["LEVY_EXPONENT"]

        self.BACK_SENSOR_WEIGHT = parameters["BACK_SENSOR_WEIGHT"]

        # Mutable state
        # Negative initialisation forces a Von Mises sample on the first tick
        # using the actual μ and κ computed from the first observation.
        self._step_remaining: float = -1.0
        # World-frame target yaw. Initialised to 0; overwritten on first step.
        self._target_yaw: float = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def step(self, obs: WaspObservation) -> tuple[WaspAction, WaspDebug]:
        """Execute one planning tick and return a body-frame velocity command.

        Args:
            obs: Current sensor observation.

        Returns:
            WaspAction with body-frame velocity and yaw_rate = 0.0.
        """
        # Step 1 — openness ring (linear map from 0 to range_max)
        openness = obs.ranges / obs.range_max

        # Step 2 — select the shared geometry policy without consuming RNG state.
        cos_b = np.cos(obs.bearings)
        sin_b = np.sin(obs.bearings)
        mu, anisotropy, kappa, step_max = self._geometry_policy(openness, cos_b, sin_b)

        # Step 5 — Lévy step: sample new heading and length when step is complete
        # or when an obstacle is detected in the forward hemisphere, forcing an
        # immediate resample from the current sensor geometry.
        forward_obstacle = bool(np.any(
            (obs.ranges < self.AVOIDANCE_RANGE) & (np.abs(obs.bearings) < np.pi / 2)
        ))
        if forward_obstacle:
            yaw_error = abs(_wrap_to_pi(self._target_yaw - obs.current_yaw))
            if yaw_error < math.pi / 4:
                self._step_remaining = 0.0

        new_step = False
        h_body = 0.0
        step_length = 0.0
        if self._step_remaining <= 0.0:
            # Sample body-frame heading from Von Mises; convert to world-frame target yaw.
            h_body = self._sample_heading(mu, kappa)
            self._target_yaw = obs.current_yaw + h_body
            step_length = self._sample_step_length(step_max)
            self._step_remaining = step_length
            new_step = True

        # Step 6 — velocity: body-forward + reactive avoidance
        mask = obs.ranges < self.AVOIDANCE_RANGE
        weights = np.where(mask, (self.AVOIDANCE_RANGE - obs.ranges) / self.AVOIDANCE_RANGE, 0.0)
        velocity_x = 1.0 - float(np.sum(weights * cos_b)) * self.AVOIDANCE_GAIN
        velocity_y = 0.0 - float(np.sum(weights * sin_b)) * self.AVOIDANCE_GAIN

        # Step 7 — decrement step counter by distance traveled this tick
        self._step_remaining -= obs.delta_dist
        action = WaspAction(velocity_x=velocity_x, velocity_y=velocity_y, target_yaw=self._target_yaw)
        debug = WaspDebug(
            new_step=new_step,
            mu=mu,
            kappa=kappa,
            anisotropy=anisotropy,
            h_body=h_body,
            target_yaw=self._target_yaw,
            step_length=step_length,
            step_remaining=self._step_remaining,
            step_max=float(step_max),
            forward_obstacle=forward_obstacle,
        )
        return action, debug

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_sensor_mean(
        openness: np.ndarray,
        cos_b: np.ndarray,
        sin_b: np.ndarray,
        back_sensor_weight: float,
    ) -> float:
        """Compute tapered, squared-openness circular mean heading."""
        back_taper = 1.0 - (1.0 - back_sensor_weight) * np.clip(-cos_b, 0.0, 1.0)
        weighted_openness = openness**2 * back_taper
        wx = float(np.sum(weighted_openness * cos_b))
        wy = float(np.sum(weighted_openness * sin_b))
        return math.atan2(wy, wx) if (wx != 0.0 or wy != 0.0) else 0.0

    @staticmethod
    def _compute_circular_signal(
        openness: np.ndarray, cos_b: np.ndarray, sin_b: np.ndarray,
    ) -> float:
        """Compute circular-resultant confinement from linear openness."""
        denominator = float(np.sum(openness))
        if denominator == 0.0:
            resultant = 0.0
        else:
            c = float(np.sum(openness * cos_b))
            s = float(np.sum(openness * sin_b))
            resultant = min(max(math.hypot(c, s) / denominator, 0.0), 1.0)
        return min(max(1.0 - resultant, 0.0), 1.0)

    def _geometry_policy(
        self, openness: np.ndarray, cos_b: np.ndarray, sin_b: np.ndarray,
    ) -> tuple[float, float, float, float]:
        """Return mode-selected μ, signal, κ, and step cap without RNG draws."""
        mode = self.controller_mode
        if mode is V5ControllerMode.UNIFORM_LEVY:
            mu, signal = 0.0, 0.0
        else:
            sensor_mu = self._compute_sensor_mean(
                openness, cos_b, sin_b, self.BACK_SENSOR_WEIGHT,
            )
            if mode is V5ControllerMode.CIRCULAR_RESULTANT:
                signal = self._compute_circular_signal(openness, cos_b, sin_b)
            else:
                signal = self._compute_anisotropy(openness, cos_b, sin_b)
            mu = sensor_mu
        step_max = math.inf
        kappa = signal * self.KAPPA_MAX
        return mu, signal, kappa, step_max

    @staticmethod
    def _compute_anisotropy(
        openness: np.ndarray, cos_b: np.ndarray, sin_b: np.ndarray,
    ) -> float:
        """Rotation-invariant anisotropy via PCA of the openness ring.

        Computes the eigenvalue ratio of the 2x2 covariance matrix of
        (openness * cos(bearing), openness * sin(bearing)).  Returns a
        scalar in [0, 1]: 0 = isotropic (open room), 1 = maximally
        anisotropic (tight corridor).
        """
        ox = openness * cos_b
        oy = openness * sin_b
        n = len(openness)
        inv_n1 = 1.0 / (n - 1)
        mx = float(ox.sum()) / n
        my = float(oy.sum()) / n
        dx = ox - mx
        dy = oy - my
        c00 = float(dx.dot(dx)) * inv_n1
        c01 = float(dx.dot(dy)) * inv_n1
        c11 = float(dy.dot(dy)) * inv_n1
        # Analytical eigenvalues of 2x2 symmetric matrix [[c00,c01],[c01,c11]]
        trace = c00 + c11
        disc = max(0.0, (c00 - c11) * (c00 - c11) + 4.0 * c01 * c01)
        sqrt_disc = math.sqrt(disc)
        lam0 = max(0.0, 0.5 * (trace - sqrt_disc))  # smaller eigenvalue
        lam1 = max(0.0, 0.5 * (trace + sqrt_disc))  # larger eigenvalue
        lam_sum = lam0 + lam1
        raw = lam1 / lam_sum if lam_sum > 1e-9 else 0.5
        return min(max(2.0 * (raw - 0.5), 0.0), 1.0)

    def _sample_heading(self, mu: float, kappa: float) -> float:
        """Sample a heading from Von Mises(mu, kappa).

        At kappa = 0 the distribution is uniform on [-pi, pi]. As kappa
        increases, samples cluster progressively around mu.
        """
        return float(vonmises.rvs(kappa, loc=mu, random_state=self._rng))

    def _sample_step_length(self, step_max: float | None = None) -> float:
        """Sample an intended distance in metres from fixed integer Zipf support."""
        return float(self._rng.zipf(self.LEVY_EXPONENT))


# SAFMC adapter
from ..api import Command, Observation, Policy, Velocity, register_policy
from ..frames import wrap_pi
from ..toolbox import body_to_world
from .mission_wrapper import MissionWrapper


@register_policy("wasp_v5")
class WaspV5Policy(Policy):
    """Adapt WASP v5 exploration to SAFMC observations and commands."""

    def __init__(self, agent_id, config, rng, arena):
        super().__init__(agent_id, config, rng, arena)
        c = self.config
        self.cruise_alt_m = float(c.get("cruise_alt_m", 0.5))
        self.climb_rate_ms = float(c.get("climb_rate_ms", 0.4))
        self.range_max_m = float(c.get("range_max_m", 3.0))
        self.max_speed_ms = float(c.get("max_speed_ms", 0.45))
        self.max_yaw_rate = float(c.get("max_yaw_rate", 1.5))
        self.yaw_p_gain = float(c.get("yaw_p_gain", 3.0))
        self.controller_mode = normalize_v5_controller_mode(
            c.get("controller_mode", DEFAULT_V5_CONTROLLER_MODE)
        )
        self.mission = MissionWrapper(
            land_range_m=float(c.get("land_range_m", 0.75)),
            approach_speed_ms=float(c.get("approach_speed_ms", 0.25)),
            yaw_p_gain=self.yaw_p_gain,
            max_yaw_rate=self.max_yaw_rate,
        ) if bool(c.get("mission_wrapper", False)) else None
        self.reset()

    def reset(self):
        self._controller = WaspV5Controller(rng=self.rng, controller_mode=self.controller_mode)
        self._last_xy = None

    def _wasp_ring(self, scan):
        ranges = scan.ranges_m.reshape(-1)
        return np.where(np.isfinite(ranges), ranges, self.range_max_m), scan.zone_bearings_rad.reshape(-1)

    def _delta_dist(self, obs):
        xy = np.array([obs.pose.x, obs.pose.y])
        distance = 0.0 if self._last_xy is None else float(np.linalg.norm(xy - self._last_xy))
        self._last_xy = xy
        return distance

    def _yaw_rate(self, target, current):
        return float(np.clip(self.yaw_p_gain * wrap_pi(target - current), -self.max_yaw_rate, self.max_yaw_rate))

    def step(self, obs: Observation) -> Command:
        if obs.pose.z < self.cruise_alt_m - 0.02:
            return Velocity(vz=self.climb_rate_ms)
        if self.mission is not None:
            command = self.mission.command(obs)
            if command is not None:
                return command
        ranges, bearings = self._wasp_ring(obs.tof)
        action, _ = self._controller.step(WaspObservation(
            bearings, ranges, self.range_max_m, self._delta_dist(obs), 0.05, obs.pose.theta,
        ))
        vx, vy = body_to_world(action.velocity_x * self.max_speed_ms,
                               action.velocity_y * self.max_speed_ms, obs.pose.theta)
        return Velocity(vx=vx, vy=vy, yaw_rate=self._yaw_rate(action.target_yaw, obs.pose.theta))


class _WaspV5ModePolicy(WaspV5Policy):
    controller_mode_name: V5ControllerMode

    def __init__(self, agent_id, config, rng, arena):
        configured = dict(config)
        configured["controller_mode"] = self.controller_mode_name.value
        super().__init__(agent_id, configured, rng, arena)


@register_policy("wasp_v5_uniform_levy")
class WaspV5UniformLevy(_WaspV5ModePolicy):
    controller_mode_name = V5ControllerMode.UNIFORM_LEVY


@register_policy("wasp_v5_circular_resultant")
class WaspV5CircularResultant(_WaspV5ModePolicy):
    controller_mode_name = V5ControllerMode.CIRCULAR_RESULTANT


@register_policy("wasp_v5_pca_heading")
class WaspV5PcaHeading(_WaspV5ModePolicy):
    controller_mode_name = V5ControllerMode.PCA_HEADING
