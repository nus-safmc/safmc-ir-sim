"""The eight-ranger time-of-flight ring -- the drone's only view of the world.

Mirrors the flown hardware (nus-safmc/esp-everything @ 99cde05):

    8 x ST VL53L5CX behind a TCA9548A I2C mux, mounted counter-clockwise at 45 degree
    spacing for gapless 360 degree coverage, on a 40 mm radius, all optical axes horizontal.
    Each reports an 8 x 8 zone grid; the firmware collapses it to one horizontal row of 8
    columns at 5.625 degrees each, gated to [50, 3000] mm, and min-pools the 64 resulting
    bearings into a 64-bin polar scan. That 64-bin scan is the only thing the navigation
    stack has ever consumed (tof_task.h:99-103, tof_task.c:243-293).

This is **one ir-sim sensor per drone**, not eight. See
docs/adr/0002-single-vectorised-tof-sensor.md -- the short form is that ir-sim's ``Lidar2D``
casts a contiguous fan (wrong shape for a ring), pays fixed GEOS overhead per instance (so
eight one-beam instances is the worst possible cost), and cannot express height-gated
occlusion at all.

Registration: ir-sim's ``SensorFactory.create_sensor`` is a hardcoded if/elif with no
registry (sensor_factory.py:31-37), unlike its behaviours and grid-map generators. So
:func:`install` patches that one method. It is narrow, idempotent, and delegates every
unrecognised name back to the original.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..constants import (
    TOF_MAX_VALID_M,
    TOF_MIN_VALID_M,
    TOF_MOUNT_RADIUS_M,
    TOF_SENSOR_COUNT,
    TOF_SENSOR_MAX_RANGE_M,
    TOF_SENSOR_SPACING_RAD,
    TOF_STATUS_NO_RETURN,
    TOF_STATUS_VALID,
    TOF_ZONE_WIDTH_RAD,
    TOF_ZONES_PER_SENSOR,
    TOF_COLLAPSED_BINS,
)
from ..errors import ConfigError
from ..frames import wrap_pi
from .raycast import cast_rays
from .scene import WorldScene

__all__ = ["ToFConfig", "ToFScan", "ToFRing", "install", "SENSOR_TYPE"]

SENSOR_TYPE = "tof_ring"


@dataclass(frozen=True)
class ToFConfig:
    """Ring geometry and gating. Defaults reproduce the flown hardware exactly."""

    n_rangers: int = TOF_SENSOR_COUNT
    zones_per_ranger: int = TOF_ZONES_PER_SENSOR
    spacing_rad: float = TOF_SENSOR_SPACING_RAD
    zone_width_rad: float = TOF_ZONE_WIDTH_RAD
    mount_radius_m: float = TOF_MOUNT_RADIUS_M
    front_index: int = 0
    """Which ranger points forward. Set per airframe via TOF_FRONT_SENSOR_IDX in menuconfig."""

    sensor_max_range_m: float = TOF_SENSOR_MAX_RANGE_M
    """Physical reach of the VL53L5CX (4 m), above the firmware's own gate."""

    min_valid_m: float = TOF_MIN_VALID_M
    max_valid_m: float = TOF_MAX_VALID_M
    """Firmware acceptance window. Returns outside it are reported as no-return, exactly as
    the firmware does -- it discards them and treats the zone as free space."""

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

    @property
    def n_zones(self) -> int:
        return self.n_rangers * self.zones_per_ranger


@dataclass(frozen=True)
class ToFScan:
    """One tick of ring data. This is what a policy sees.

    ``inf`` means "no valid return", never a fabricated maximum. The distinction is real: the
    firmware maps target_status 255 to INFINITY and treats it as free space, whereas a genuine
    reading at the gate limit is an obstacle.
    """

    ranges_m: np.ndarray
    """``(n_rangers, zones_per_ranger)`` metres, ``inf`` where invalid."""

    status: np.ndarray
    """``(n_rangers, zones_per_ranger)`` uint8, VL53L5CX encoding: 5 valid, 255 no return."""

    collapsed_m: np.ndarray
    """``(64,)`` min-pooled polar scan. Index 0 straight ahead, clockwise, 5.625 deg per bin.
    The only form the real navigation stack consumes."""

    zone_bearings_rad: np.ndarray
    """``(n_rangers, zones_per_ranger)`` body-frame bearings, CCW from nose. Constant."""

    ranger_bearings_rad: np.ndarray
    """``(n_rangers,)`` body-frame ranger axes, CCW from nose. Constant."""

    tick: int = -1
    sim_time_s: float = -1.0

    def as_firmware_frame(self) -> dict:
        """The exact byte-level product the ESP32 keeps: distance_mm plus target_status.

        The driver offers range_sigma_mm, reflectance, ambient_per_spad and more; the firmware
        copies only these two (tof_task.c:461-466) and discards the rest, so this is the
        complete real interface.
        """
        mm = np.where(
            np.isfinite(self.ranges_m), np.round(self.ranges_m * 1000.0), 0.0
        ).astype(np.uint16)
        return {"distance_mm": mm, "target_status": self.status.astype(np.uint8)}

    @property
    def min_range_m(self) -> float:
        """Nearest valid return anywhere on the ring, or ``inf``."""
        return float(np.min(self.collapsed_m))


def _read_only(array: np.ndarray) -> np.ndarray:
    """A view of ``array`` that cannot be written through."""
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


def _zone_offsets(cfg: ToFConfig) -> np.ndarray:
    """Per-column offsets from a ranger's axis, CCW.

    Firmware: ``angle_deg = SENSOR_ANGLES[s] + (3.5 - col) * 5.625`` clockwise
    (tof_task.c:258). Negated, that is ``(col - 3.5) * 5.625`` counter-clockwise.
    """
    col = np.arange(cfg.zones_per_ranger)
    return (col - (cfg.zones_per_ranger - 1) / 2.0) * cfg.zone_width_rad


class ToFRing:
    """ir-sim sensor object. Duck-typed to ir-sim's contract; there is no base class to inherit.

    ir-sim only ever touches ``.step()``, ``.sensor_type``, ``.parent``, and the three plot
    methods (object_base.py:543, object_plot.py:443-478).
    """

    def __init__(self, state=None, obj_id=None, **kwargs):
        self.sensor_type = SENSOR_TYPE
        self.obj_id = obj_id
        self.parent = None  # assigned by ObjectBase during construction

        cfg = kwargs.pop("config", None)
        if cfg is None:
            # Accept the flat keys ir-sim passes straight from YAML.
            known = {
                k: kwargs.pop(k)
                for k in list(kwargs)
                if k in ToFConfig.__dataclass_fields__
            }
            cfg = ToFConfig(**known)
        self.config: ToFConfig = cfg

        self._world_scene: WorldScene | None = None
        self._rng: np.random.Generator | None = None

        self._ranger_bearings = _ranger_bearings(self.config)
        self._zone_offsets = _zone_offsets(self.config)
        # (n_rangers, zones) body-frame bearing of every zone. Constant for the run.
        self._zone_bearings = wrap_pi(
            self._ranger_bearings[:, None] + self._zone_offsets[None, :]
        )
        self._collapsed_bins = self._compute_collapsed_bins()
        # Handed to policies inside every ToFScan. A frozen dataclass blocks rebinding but not
        # in-place writes, and these two arrays are the sensor's own persistent state -- a
        # policy doing `obs.tof.zone_bearings_rad[:] += 0.5` would permanently re-aim the ring
        # while _collapsed_bins kept the old mapping, silently and irrecoverably (R-POL-2).
        self._zone_bearings_ro = _read_only(self._zone_bearings)
        self._ranger_bearings_ro = _read_only(self._ranger_bearings)

        self._scan: ToFScan | None = None
        self._last_endpoints: np.ndarray | None = None
        self._artist = None

    # -- wiring, done by the runner after irsim.make() ------------------------------------

    def attach(self, world_scene: WorldScene, rng: np.random.Generator) -> None:
        self._world_scene = world_scene
        self._rng = rng

    # -- geometry -------------------------------------------------------------------------

    def _compute_collapsed_bins(self) -> np.ndarray:
        """Map each zone to its bin in the firmware's 64-bin clockwise polar scan."""
        cw_rad = np.mod(-self._zone_bearings, 2.0 * np.pi)
        bin_width = 2.0 * np.pi / TOF_COLLAPSED_BINS
        return np.mod(
            np.floor(cw_rad / bin_width).astype(int), TOF_COLLAPSED_BINS
        )

    @property
    def zone_bearings_rad(self) -> np.ndarray:
        return self._zone_bearings_ro

    # -- the ir-sim contract --------------------------------------------------------------

    def step(self, state) -> ToFScan:
        """Sample the ring. ``state`` is ir-sim's ``(3, 1)`` slice: x, y, theta."""
        if self._world_scene is None or self._rng is None:
            raise ConfigError(
                "ToFRing.attach() was never called -- the sensor has no world to cast into. "
                "The runner does this after irsim.make(); a hand-built env must too."
            )

        x, y, theta = float(state[0, 0]), float(state[1, 0]), float(state[2, 0])
        z = self._altitude()

        parent = self.parent
        env = getattr(parent, "_env", None) if parent is not None else None
        if env is not None:
            self._world_scene.refresh_drones(env.robot_list, getattr(env._world, "count", None))

        cfg = self.config
        world_ranger = theta + self._ranger_bearings                 # (R,)
        # Each ranger sits on the mount circle, pointing radially outward.
        origins_per_ranger = np.stack(
            (
                x + cfg.mount_radius_m * np.cos(world_ranger),
                y + cfg.mount_radius_m * np.sin(world_ranger),
            ),
            axis=1,
        )                                                            # (R, 2)
        origins = np.repeat(origins_per_ranger, cfg.zones_per_ranger, axis=0)

        world_zone = (theta + self._zone_bearings).reshape(-1)       # (R*Z,)
        directions = np.stack((np.cos(world_zone), np.sin(world_zone)), axis=1)

        scene = self._world_scene.sensing_scene(
            exclude_object_id=getattr(parent, "id", None)
        )
        raw = cast_rays(scene, origins, directions, z, cfg.sensor_max_range_m)

        if cfg.noise_std_m > 0.0:
            hit = np.isfinite(raw)
            # Noise on a no-return is meaningless -- there is nothing to perturb.
            raw = np.where(
                hit, raw + self._rng.normal(0.0, cfg.noise_std_m, raw.shape), raw
            )

        # The firmware's acceptance window. Anything outside it is discarded and the zone is
        # reported as free space, which is precisely what target_status 255 means downstream.
        valid = np.isfinite(raw) & (raw >= cfg.min_valid_m) & (raw <= cfg.max_valid_m)
        ranges = np.where(valid, raw, np.inf).reshape(cfg.n_rangers, cfg.zones_per_ranger)
        status = np.where(valid, TOF_STATUS_VALID, TOF_STATUS_NO_RETURN).astype(np.uint8)
        status = status.reshape(cfg.n_rangers, cfg.zones_per_ranger)

        collapsed = np.full(TOF_COLLAPSED_BINS, np.inf)
        np.minimum.at(collapsed, self._collapsed_bins.reshape(-1), ranges.reshape(-1))

        world = getattr(env, "_world", None) if env is not None else None
        self._scan = ToFScan(
            ranges_m=ranges,
            status=status,
            collapsed_m=collapsed,
            zone_bearings_rad=self._zone_bearings_ro,
            ranger_bearings_rad=self._ranger_bearings_ro,
            # ir-sim steps sensors BEFORE incrementing the world clock (env_base.py:328-330),
            # so world.count is still the previous tick here. Report the tick this scan
            # actually belongs to.
            tick=int(getattr(world, "count", -1)) + 1 if world is not None else -1,
            sim_time_s=(
                float(getattr(world, "time", 0.0)) + float(getattr(world, "step_time", 0.0))
                if world is not None
                else -1.0
            ),
        )

        # Endpoints for rendering: a no-return is drawn at the sensor's physical reach so the
        # cone stays visible, but that value never enters the scan.
        draw_len = np.where(np.isfinite(raw), raw, cfg.sensor_max_range_m)
        self._last_endpoints = origins + directions * draw_len[:, None]
        return self._scan

    def get_scan(self) -> ToFScan:
        """The latest scan. Raises if the sensor has never stepped."""
        if self._scan is None:
            raise ConfigError("no scan yet -- step() has not run")
        return self._scan

    @property
    def latest_scan(self) -> ToFScan | None:
        """The latest scan, or ``None`` if the sensor has never stepped.

        Exists because on tick 0 no ``env.step()`` has happened yet, so a caller that wants to
        build a real first observation has to notice the difference between "no scan yet" and
        "a scan that happens to be empty" without catching an exception to find out.
        """
        return self._scan

    def _altitude(self) -> float:
        """Altitude of the parent drone. Row 3 of the Quad25D state."""
        parent = self.parent
        if parent is None:
            raise ConfigError("ToFRing has no parent object")
        state = parent.state
        if state.shape[0] <= 3:
            raise ConfigError(
                f"parent state has {state.shape[0]} rows; the ToF ring needs altitude in "
                f"row 3, which means the robot must use the quad25d kinematics"
            )
        return float(state[3, 0])

    # -- rendering (only called when plotting is enabled) ---------------------------------

    def plot(self, ax, state=None, **kwargs):
        from matplotlib.collections import LineCollection

        if self._last_endpoints is None:
            return
        self._artist = LineCollection(
            self._segments(),
            colors=kwargs.get("color", "tab:orange"),
            linewidths=kwargs.get("linewidth", 0.6),
            alpha=kwargs.get("alpha", 0.5),
            zorder=1,
        )
        ax.add_collection(self._artist)

    def step_plot(self):
        if self._artist is not None and self._last_endpoints is not None:
            self._artist.set_segments(self._segments())

    def plot_clear(self):
        if self._artist is not None:
            self._artist.remove()
            self._artist = None

    def _segments(self):
        cfg = self.config
        x, y, theta = (
            float(self.parent.state[0, 0]),
            float(self.parent.state[1, 0]),
            float(self.parent.state[2, 0]),
        )
        world_ranger = theta + self._ranger_bearings
        origins_per_ranger = np.stack(
            (
                x + cfg.mount_radius_m * np.cos(world_ranger),
                y + cfg.mount_radius_m * np.sin(world_ranger),
            ),
            axis=1,
        )
        origins = np.repeat(origins_per_ranger, cfg.zones_per_ranger, axis=0)
        return list(np.stack((origins, self._last_endpoints), axis=1))


def install() -> None:
    """Teach ir-sim's hardcoded sensor factory about this sensor type. Idempotent."""
    from irsim.world.sensors.sensor_factory import SensorFactory

    if getattr(SensorFactory.create_sensor, "_safmc_patched", False):
        return

    original = SensorFactory.create_sensor

    def create_sensor(self, state, obj_id, **kwargs):
        name = kwargs.get("name", kwargs.get("type", "lidar2d"))
        if name == SENSOR_TYPE:
            kwargs.pop("name", None)
            kwargs.pop("type", None)
            return ToFRing(state, obj_id, **kwargs)
        return original(self, state, obj_id, **kwargs)

    create_sensor._safmc_patched = True
    SensorFactory.create_sensor = create_sensor
