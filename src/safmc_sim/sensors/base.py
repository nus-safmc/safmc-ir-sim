"""The sensor primitive: what every sensor is, and the one rule it lives under.

A sensor is a function from the drone's **true** state and the world to a frozen reading,
sampled at a rate that divides the tick rate. The runner owns it, steps it after motion, and
hands its latest reading to the policy under the sensor's name. That is the whole contract.
The two sensors that ship -- the ToF ring and the marker camera -- are written against it, so
a third sensor is a third file rather than a third special case.

The rule that keeps a policy honest
-----------------------------------

    Truth enters a sensor. Only readings leave it.

A sensor is handed :class:`TrueState` -- the exact pose and velocity -- because that is what a
physical sensor is bolted to. A policy is never handed it. What a policy knows of its own pose
comes from :class:`~safmc_sim.pose.PoseSource`; what it knows of the world comes from
readings. So there are exactly two paths from ground truth to a policy, and both are seams:
the pose source (R-SEAM-1) and this contract (R-SENS-15). A sensor that stashed the arena on
``self`` and handed it out inside a reading would be a third path, which is why ``sample``
receives a :class:`~safmc_sim.sensors.scene.WorldScene` and never the arena, the mission or
another agent.

Writing a sensor
----------------

Three parts, one file. ``examples/03_custom_sensor.py`` is a complete, runnable one.

1. **A reading** -- a frozen dataclass. It is the *only* thing a policy sees, so make it what
   the real device would report and nothing more. Wrap numpy arrays in :func:`read_only`: a
   frozen dataclass stops rebinding but not in-place writes, and a policy doing
   ``obs.sensors["x"].values[:] = 0`` would otherwise edit the sensor's own memory.

2. **A config** -- a frozen dataclass subclassing :class:`SensorConfig`. Give it a default
   ``name`` (its key under ``obs.sensors``), a default ``rate_hz`` (``None`` means every
   tick), validate in ``__post_init__`` -- calling ``super().__post_init__()`` first -- so a
   bad value fails at construction and never at tick 4 000, and implement
   :meth:`SensorConfig.build`. Field names carry their units: ``_m``, ``_rad``, ``_s``.

3. **The sensor** -- a :class:`Sensor` subclass implementing :meth:`Sensor.sample`. Read the
   world through ``world.sensing_scene(exclude_object_id=truth.object_id)`` for anything a
   ray can hit and ``world.landmarks_of(kind)`` for things placed in the arena. Draw noise
   from ``self.rng``, never from ``numpy.random`` (R-SENS-9). Keep the geometry in a pure
   module-level function and let ``sample`` be the adapter; the pure function is what you
   will unit-test.

Then ``RunConfig(sensors=flown_sensors() + (YourConfig(),))``. Nothing in the runner, the
policy API or the recorder needs to know your sensor exists.

Timing, exactly
---------------

Every sensor is sampled once before the first tick, and then after motion at the end of every
tick ``t`` for which ``(t + 1) % decimation == 0``. So a sensor at 2 Hz on a 20 Hz loop is
fresh in the observations at ticks 0, 10, 20, ... and ``obs.stale_ticks[name]`` counts up in
between. A terminal drone stops sampling; its last reading is held. Rates that do not divide
the tick rate are refused at configuration time (R-TIME-3) rather than rounded.
"""

from __future__ import annotations

import re
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Mapping

import numpy as np

from ..errors import ConfigError

if TYPE_CHECKING:  # pragma: no cover -- annotations only; scene imports this module
    from .scene import WorldScene

__all__ = ["TrueState", "SensorConfig", "Sensor", "read_only", "decimation"]

_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")

# Sensor readings are recorded to ``<name>.npz`` beside these two files.
_RESERVED_NAMES = frozenset({"states", "run"})


@dataclass(frozen=True)
class TrueState:
    """The exact state of the drone carrying a sensor. Sensors see this; policies never do.

    ``object_id`` is ir-sim's id for the carrying body, and exists so a sensor can leave its
    own drone out of the scene it samples -- a ring must not range its own airframe.
    """

    agent_id: str
    object_id: int
    x: float
    y: float
    z: float
    theta: float
    vx: float
    vy: float

    @property
    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y])

    @classmethod
    def from_state(cls, agent_id: str, object_id: int, state: np.ndarray) -> "TrueState":
        """From the 6-row Quad25D state ``[x, y, theta, z, vx, vy]`` (R-DRONE-1)."""
        return cls(
            agent_id=agent_id,
            object_id=int(object_id),
            x=float(state[0, 0]),
            y=float(state[1, 0]),
            theta=float(state[2, 0]),
            z=float(state[3, 0]),
            vx=float(state[4, 0]),
            vy=float(state[5, 0]),
        )


@dataclass(frozen=True)
class SensorConfig(ABC):
    """What a sensor is, independent of which drone carries it. Frozen and logged verbatim.

    Subclasses give ``name`` and ``rate_hz`` defaults and add their own fields. The same
    config is handed to every drone in the run; per-drone state lives on the :class:`Sensor`
    that :meth:`build` returns.
    """

    name: str
    """Key under ``obs.sensors`` and stem of the recorded ``<name>.npz``. Unique per run."""

    rate_hz: float | None
    """Sample rate. ``None`` samples every tick. Must divide the tick rate exactly."""

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not _NAME.match(self.name):
            raise ConfigError(
                f"sensor name must be an identifier (letters, digits, underscores), "
                f"got {self.name!r} -- it is a mapping key and a file stem"
            )
        if self.name in _RESERVED_NAMES:
            raise ConfigError(
                f"sensor name {self.name!r} is reserved: {self.name}.npz would collide "
                f"with the run's own files"
            )
        if self.rate_hz is not None and not self.rate_hz > 0.0:
            raise ConfigError(
                f"sensor {self.name!r}: rate_hz must be > 0 or None, got {self.rate_hz}"
            )

    @property
    def landmark_kinds(self) -> tuple[str, ...]:
        """Landmark kinds this sensor identifies by kind. Empty for a sensor that does not.

        The runner uses this to refuse a point landmark that no configured sensor could ever
        report -- a nav tag nobody can read is a configuration mistake, not a quiet feature.
        Solid landmarks are exempt: the ring sees them as bodies regardless of kind.
        """
        return ()

    @abstractmethod
    def build(self, rng: np.random.Generator) -> "Sensor":
        """One sensor instance for one drone, with that drone's own generator."""


class Sensor(ABC):
    """One sensor on one drone. Constructed by its config; owned and stepped by the runner."""

    def __init__(self, config: SensorConfig, rng: np.random.Generator) -> None:
        self.config = config
        self.rng = rng

    @property
    def name(self) -> str:
        return self.config.name

    @abstractmethod
    def sample(self, truth: TrueState, world: "WorldScene", tick: int) -> Any:
        """Produce this tick's reading from the true state and the world.

        Pure apart from ``self.rng``: the same truth, world and generator state give the same
        reading. ``tick`` is the tick whose post-motion world is being sampled, ``-1`` for the
        sample before the first tick; use it for time-dependent noise, never for anything a
        policy could not know.
        """

    # -- optional: appear in the log ---------------------------------------------------------

    def record(self, reading: Any) -> Mapping[str, np.ndarray] | None:
        """Fixed-shape arrays to log per tick, or ``None`` to leave this sensor out of the log.

        Each key becomes an array in ``<name>.npz`` stacked to ``(ticks, agents, *shape)``.
        The shape must be the same every tick, which is why the default is ``None``: a
        variable-length reading, such as a tuple of detections, does not fit a table.
        """
        return None

    def record_static(self) -> Mapping[str, np.ndarray]:
        """Arrays that are constant for the run and needed to interpret the rows.

        The ring stores its zone bearings here so that a log can be drawn without importing
        the simulator (R-OBS-3).
        """
        return {}


def read_only(array: np.ndarray) -> np.ndarray:
    """A view that cannot be written through.

    A frozen dataclass blocks rebinding but not in-place numpy writes. Without this, a policy
    doing ``obs.tof.zone_bearings_rad[:] += 0.5`` would permanently re-aim its own ring,
    silently and irrecoverably.
    """
    view = array.view()
    view.flags.writeable = False
    return view


def decimation(rate_hz: float | None, tick_hz: float, name: str) -> int:
    """Turn a sample rate into a whole number of ticks, or refuse (R-TIME-3)."""
    if rate_hz is None:
        return 1
    if rate_hz <= 0:
        raise ConfigError(f"{name}: rate_hz must be > 0, got {rate_hz}")
    ratio = tick_hz / rate_hz
    nearest = round(ratio)
    if nearest < 1 or abs(ratio - nearest) > 1e-9:
        raise ConfigError(
            f"{name}: rate_hz={rate_hz} does not divide tick_hz={tick_hz} exactly "
            f"(ratio {ratio:.6f}). Pick a rate that divides the tick rate, or change "
            f"tick_hz -- silently rounding a sensor rate would misreport sensor latency."
        )
    return int(nearest)
