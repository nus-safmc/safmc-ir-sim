"""The surface a policy author writes against. This is the stable part of the project.

The design rule is **primitives, not behaviour**. This package wraps ir-sim; it does not fly
the drone for you. There is exactly one motion command -- a velocity -- and one mission
command -- land. Guidance, obstacle avoidance, mapping, search strategy and coordination are
all yours to write, because those are the things the simulator exists to let you compare.

An earlier version shipped a ``SearchPolicy`` base class that took off, chose targets, claimed
them over the blackboard and landed on them, leaving subclasses to supply only a wandering
step. That was a strategy wearing scaffolding's clothes: anyone subclassing it inherited a set
of mission decisions without noticing they had. It is gone. If you want that behaviour, write
it, and then it is yours and it is visible.

Writing a policy
----------------

    from safmc_sim.api import Observation, Policy, Velocity, Land, register_policy

    @register_policy("hug_the_wall")
    class HugTheWall(Policy):
        def reset(self):
            self.airborne = False

        def step(self, obs: Observation) -> Command:
            if obs.pose.z < 0.5:                       # climb -- you decide how
                return Velocity(vz=0.4)
            left = obs.tof.ranges_m[2].min()           # ranger 2 points +90 deg
            if left > 1.0:
                return Velocity(vx=0.3, yaw_rate=0.5)
            return Velocity(vx=0.4)

Three rules the design enforces rather than requests
----------------------------------------------------

**A policy can only see what a drone could see.** ``Observation`` is frozen and holds plain
data. There is no route from it to the environment, the arena, the mission, or another agent's
object, so "accidentally reading ground truth" is not a mistake that can be made.

**Randomness is injected, never ambient.** Each policy is handed its own
``numpy.random.Generator``. Touching ``numpy.random`` directly breaks seeded replay.

**A policy that raises, aborts the run.** There is no catch-and-hover. A silently degraded
agent produces a plausible-looking result that is wrong, which is worse than a stack trace.

Optional building blocks live in :mod:`safmc_sim.toolbox` -- a body-to-world rotation, a
sensor reduction, a log-odds occupancy grid. Nothing here imports them. They are examples to
copy or replace, not part of the contract.
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
    "Velocity",
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
    """Three states, and only the two terminal ones are the simulator's business.

    There is deliberately no ARMED, TAKEOFF or LANDING phase. Climbing and descending are
    things a policy does with a velocity command, not modes the simulator puts a drone into --
    modelling them as states meant the runner was choreographing manoeuvres, which is exactly
    the kind of behaviour that belongs to whoever is writing the strategy.

    ``LANDED`` and ``CRASHED`` are terminal, and both are permanent. Landing is a commitment:
    the competition requires a rescuing drone to stay put until the end of the mission, which
    makes the fleet a depleting resource and turns "how many drones do I spend, and when" into
    the real strategic question.
    """

    ACTIVE = "ACTIVE"
    LANDED = "LANDED"
    CRASHED = "CRASHED"

    ALL = (ACTIVE, LANDED, CRASHED)
    TERMINAL = (LANDED, CRASHED)


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

    sensors: Mapping[str, Any]
    """The latest reading of every sensor this drone carries, keyed by sensor name.

    ``sensors["tof"]`` is a :class:`ToFScan` and ``sensors["markers"]`` a tuple of
    :class:`MarkerDetection`; those two have the shorthands :attr:`tof` and :attr:`markers`
    because the flown airframe carries them. Anything else is reached here under the name its
    config was given -- ``RunConfig(sensors=...)`` decides what a drone carries, and a reading
    is whatever its sensor's author made it, so read that sensor's docstring. Every reading is
    immutable: the runner refuses a sensor whose reading is a list, a dict, a mutable
    dataclass or a writable array (R-SENS-12). What a reading *contains* is its author's
    honesty -- the contract keeps the arena, the mission and other agents out of a sensor's
    reach (R-SENS-15), but it cannot stop a sensor returning more than its physical
    counterpart could measure. That is what review and FIDELITY.md are for.
    """

    peers: Mapping[str, Mapping[str, Any]]
    """Blackboard as of the START of this tick, keyed by agent id.

    In v0.1 this is a perfect shared store (ADR-0003). Every agent sees the same snapshot, so
    results cannot depend on the order agents happen to be stepped (R-POL-8).
    """

    arena: ArenaInfo
    stale_ticks: Mapping[str, int] = field(default_factory=dict)
    """Ticks since each sensor last sampled, keyed like ``sensors``. Zero means fresh.

    Non-zero only for a decimated sensor -- the marker camera at 2 Hz is fresh every tenth
    tick and counts 1..9 in between -- or for a drone that has stopped sensing.
    """

    # -- the flown airframe's two sensors, by name ---------------------------------------------

    @property
    def tof(self) -> ToFScan:
        """The ring's latest scan. Shorthand for ``sensors["tof"]``."""
        return self._reading("tof")

    @property
    def markers(self) -> tuple[MarkerDetection, ...]:
        """Markers in view at the last camera sample. Shorthand for ``sensors["markers"]``."""
        return self._reading("markers")

    def _reading(self, name: str) -> Any:
        try:
            return self.sensors[name]
        except KeyError:
            # AttributeError, not KeyError: this is attribute access, and hasattr(obs, "tof")
            # must answer False rather than raise.
            raise AttributeError(
                f"this run has no sensor named {name!r}; it carries "
                f"{sorted(self.sensors) or 'no sensors'}. RunConfig(sensors=...) decides "
                f"what a drone carries."
            ) from None

    @property
    def in_start_area(self) -> bool:
        return 0.0 <= self.pose.y <= self.arena.start_area_depth_m

    @property
    def time_remaining_s(self) -> float:
        return max(0.0, self.arena.run_duration_s - self.sim_time_s)


# ------------------------------------------------------------------------------------------
# Commands -- one motion primitive, one mission commitment
# ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Velocity:
    """The only motion command. ARENA-frame linear velocity plus a yaw rate.

    World frame rather than body frame for two reasons: it is what the real drone's
    ``mavlink_set_velocity_ned`` actually takes, and it is what our kinematics integrates, so
    nothing is being converted behind your back. If you would rather think in body frame --
    and reasoning from ToF ranges usually means you would -- rotate with
    :func:`safmc_sim.toolbox.body_to_world`, which is four lines you can read.

    Values are clipped by the drone's own limits (speed, climb rate, yaw rate) and tracked
    through a first-order lag. Nothing else happens to them: there is no controller here, no
    setpoint tracking, no altitude hold. If you stop commanding ``vz``, the drone stops
    climbing; it does not hold altitude for you.
    """

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class Land:
    """Commit to the ground here, permanently.

    Not a manoeuvre -- a decision. The drone stops where it is and is spent for the rest of the
    run. This exists as a command rather than as "descend until z is zero" because under the
    rules landing is a *scoring event* with consequences (the drone must stay until the end),
    and because the runner needs an unambiguous moment at which to freeze the drone and latch
    the score. Flying the descent yourself is fine too -- see
    :func:`safmc_sim.toolbox.descend` -- but the commitment is this command.
    """


Command = Union[Velocity, Land]

COMMAND_TYPES = (Velocity, Land)


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
