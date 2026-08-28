"""The eight-ranger time-of-flight ring -- the drone's only view of the world.

Mirrors the flown hardware: 8 x ST VL53L5CX mounted counter-clockwise at 45 degree spacing for
gapless 360 degree coverage, on a 40 mm radius, all optical axes horizontal, each contributing
8 zones of 5.625 degrees across its 45 degree field of view.

The runner owns this. It is deliberately **not** plugged into ir-sim's sensor system: doing
that required monkeypatching ``SensorFactory.create_sensor`` (ir-sim has no sensor registry),
and it dragged in a plotting path that never runs, a walk up to the parent object to find the
drone's altitude, and arithmetic to reverse-engineer the tick number from ir-sim's clock. The
runner already has the state, the altitude and the tick. Calling ``step()`` directly is both
smaller and honest about who is in charge.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..constants import (
    TOF_MAX_VALID_M,
    TOF_MOUNT_RADIUS_DIAGONAL_M,
    TOF_MIN_VALID_M,
    TOF_MOUNT_RADIUS_M,
    TOF_SENSOR_COUNT,
    TOF_SENSOR_MAX_RANGE_M,
    TOF_SENSOR_SPACING_RAD,
    TOF_ZONE_WIDTH_RAD,
    TOF_ZONES_PER_SENSOR,
)
from ..errors import ConfigError
from ..frames import wrap_pi
from .raycast import cast_rays
from .scene import WorldScene

__all__ = ["ToFConfig", "ToFScan", "ToFRing"]


@dataclass(frozen=True)
class ToFConfig:
    """Ring geometry and gating. Defaults reproduce the flown hardware exactly."""

    n_rangers: int = TOF_SENSOR_COUNT
    zones_per_ranger: int = TOF_ZONES_PER_SENSOR
    spacing_rad: float = TOF_SENSOR_SPACING_RAD
    zone_width_rad: float = TOF_ZONE_WIDTH_RAD
    mount_radius_m: float = TOF_MOUNT_RADIUS_M
    mount_radius_diagonal_m: float = TOF_MOUNT_RADIUS_DIAGONAL_M
    """The ring is not a circle. In the URDF the four cardinal sensors sit 40 mm out and the
    four diagonals only 34 mm, because the PCB is rectangular. A single radius put the
    diagonals 6 mm too far out -- far too small to change any result at a 3 m gate, but the
    URDF is the source of truth and it is cheaper to match it than to explain why we don't."""
    front_index: int = 0
    """Which ranger points forward. Set per airframe via TOF_FRONT_SENSOR_IDX in menuconfig."""

    sensor_max_range_m: float = TOF_SENSOR_MAX_RANGE_M
    """Physical reach of the VL53L5CX (4 m), above the firmware's own gate."""

    min_valid_m: float = TOF_MIN_VALID_M
    max_valid_m: float = TOF_MAX_VALID_M
    """Acceptance window. A return outside it is reported as no-return -- which is what the
    real firmware does, and what a policy must cope with."""

    noise_std_m: float = 0.0
    """Additive Gaussian range noise. Off by default: the target paper's arenas are noiseless
    and reproducing it is our first regression test. Turn it on for robustness sweeps."""

    def __post_init__(self) -> None:
        if self.n_rangers < 1 or self.zones_per_ranger < 1:
            raise ConfigError("n_rangers and zones_per_ranger must both be >= 1")
        if not 0 <= self.front_index < self.n_rangers:
            raise ConfigError(
                f"front_index {self.front_index} out of range for {self.n_rangers} rangers"
            )
        if self.min_valid_m >= self.max_valid_m:
            raise ConfigError(
                f"min_valid_m ({self.min_valid_m}) must be < max_valid_m ({self.max_valid_m})"
            )
        if self.max_valid_m > self.sensor_max_range_m:
            raise ConfigError(
                f"max_valid_m ({self.max_valid_m}) exceeds the sensor's physical reach "
                f"({self.sensor_max_range_m}) -- the gate cannot see further than the sensor"
            )
        if self.noise_std_m < 0.0:
            raise ConfigError("noise_std_m must be >= 0")


@dataclass(frozen=True)
class ToFScan:
    """One tick of ring data. This is what a policy sees.

    ``inf`` means "no valid return", never a fabricated maximum. The distinction is real: a
    zone that saw nothing and a zone that saw a wall at exactly the gate limit are different
    facts, and a policy that wants to conflate them should do so deliberately.
    """

    ranges_m: np.ndarray
    """``(n_rangers, zones_per_ranger)`` metres, ``inf`` where there was no valid return."""

    zone_bearings_rad: np.ndarray
    """``(n_rangers, zones_per_ranger)`` body-frame bearings, CCW from the nose. Constant."""

    ranger_bearings_rad: np.ndarray
    """``(n_rangers,)`` body-frame ranger axes, CCW from the nose. Constant."""

    @property
    def min_range_m(self) -> float:
        """Nearest valid return anywhere on the ring, or ``inf``."""
        return float(np.min(self.ranges_m))


def _read_only(array: np.ndarray) -> np.ndarray:
    """A view that cannot be written through.

    ``ToFScan`` is a frozen dataclass, which blocks rebinding but not in-place numpy writes.
    Without this, a policy doing ``obs.tof.zone_bearings_rad[:] += 0.5`` would permanently
    re-aim its own ring, silently and irrecoverably.
    """
    view = array.view()
    view.flags.writeable = False
    return view


def _ranger_bearings(cfg: ToFConfig) -> np.ndarray:
    """Body-frame ranger axes, CCW from the nose.

    The firmware stores clockwise angles, ``SENSOR_ANGLES[i] = ((front_idx - i) * 45) mod 360``
    (tof_task.c:183). Negating for our counter-clockwise convention collapses to a simple
    ``(i - front_index) * spacing`` -- which is just the "mounted counter-clockwise" comment
    in tof_task.h:26 stated directly.
    """
    idx = np.arange(cfg.n_rangers)
    return wrap_pi((idx - cfg.front_index) * cfg.spacing_rad)


def _mount_radii(cfg: ToFConfig) -> np.ndarray:
    """How far out each ranger sits. Cardinals and diagonals differ; see the URDF.

    Only meaningful for the real 8-sensor ring, where rangers alternate cardinal, diagonal,
    cardinal... Any other count gets a uniform radius, because there is no hardware to be
    faithful to.
    """
    if cfg.n_rangers != 8:
        return np.full(cfg.n_rangers, cfg.mount_radius_m)
    is_diagonal = (np.arange(8) - cfg.front_index) % 2 == 1
    return np.where(is_diagonal, cfg.mount_radius_diagonal_m, cfg.mount_radius_m)


def _zone_offsets(cfg: ToFConfig) -> np.ndarray:
    """Per-column offsets from a ranger's axis, CCW.

    Firmware: ``angle_deg = SENSOR_ANGLES[s] + (3.5 - col) * 5.625`` clockwise
    (tof_task.c:258). Negated, that is ``(col - 3.5) * 5.625`` counter-clockwise.
    """
    col = np.arange(cfg.zones_per_ranger)
    return (col - (cfg.zones_per_ranger - 1) / 2.0) * cfg.zone_width_rad


class ToFRing:
    """One drone's ring. Constructed and stepped by the runner."""

    def __init__(self, config: ToFConfig, world_scene: WorldScene, rng: np.random.Generator,
                 object_id: int | None = None) -> None:
        self.config = config
        self._world_scene = world_scene
        self._rng = rng
        self._object_id = object_id

        self._ranger_bearings = _ranger_bearings(config)
        self._mount_radii = _mount_radii(config)
        self._zone_offsets = _zone_offsets(config)
        # (n_rangers, zones) body-frame bearing of every zone. Constant for the run.
        self._zone_bearings = wrap_pi(
            self._ranger_bearings[:, None] + self._zone_offsets[None, :]
        )
        # Handed to policies inside every ToFScan. A frozen dataclass blocks rebinding but not
        # in-place writes, and these are the sensor's own arrays -- a policy doing
        # `obs.tof.zone_bearings_rad[:] += 0.5` would permanently re-aim the ring.
        self._zone_bearings_ro = _read_only(self._zone_bearings)
        self._ranger_bearings_ro = _read_only(self._ranger_bearings)

    def step(self, x: float, y: float, theta: float, z: float) -> ToFScan:
        """Sample the ring from a pose. Pure: same pose and scene give the same scan."""
        cfg = self.config
        world_ranger = theta + self._ranger_bearings
        # Each ranger sits on the mount circle, pointing radially outward.
        origins = np.repeat(
            np.stack(
                (x + self._mount_radii * np.cos(world_ranger),
                 y + self._mount_radii * np.sin(world_ranger)),
                axis=1,
            ),
            cfg.zones_per_ranger,
            axis=0,
        )
        world_zone = (theta + self._zone_bearings).reshape(-1)
        directions = np.stack((np.cos(world_zone), np.sin(world_zone)), axis=1)

        scene = self._world_scene.sensing_scene(exclude_object_id=self._object_id)
        raw = cast_rays(scene, origins, directions, z, cfg.sensor_max_range_m)

        if cfg.noise_std_m > 0.0:
            hit = np.isfinite(raw)
            # Noise on a no-return is meaningless -- there is nothing to perturb.
            raw = np.where(hit, raw + self._rng.normal(0.0, cfg.noise_std_m, raw.shape), raw)

        valid = np.isfinite(raw) & (raw >= cfg.min_valid_m) & (raw <= cfg.max_valid_m)
        ranges = np.where(valid, raw, np.inf).reshape(cfg.n_rangers, cfg.zones_per_ranger)

        return ToFScan(
            ranges_m=ranges,
            zone_bearings_rad=self._zone_bearings_ro,
            ranger_bearings_rad=self._ranger_bearings_ro,
        )
