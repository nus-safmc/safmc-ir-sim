"""The UWB ranging tag -- range-only radio localisation, on the sensor contract.

**The airframe does not carry one.** The rules permit ultra-wideband by name (Category Swarm
booklet 6.3, while banning 5.7-5.9 GHz) and say where an anchor may stand (3.3.1 r.14-17:
any number in the Start Area, at most ten in the Known Search Area, none in the Unknown
Search Area, each within 1 m x 1 m, secured and not hung from overhead -- a tripod, in
practice). This module is what a policy would be written against if the team bought a
module, and what a future ``PoseSource`` would consume.
It is a model of the literature on DW1000/DW3000-class hobby modules -- Qorvo DWM1001,
Bitcraze Loco, Pozyx, Makerfabs MaUWB -- not of any device on the bench, and every number in
it is an assumption with an ID. ADR-0006 records the decisions; R-SENS-17 is the contract.

What a tag reports
------------------

A tag in a real-time-location network is configured with its anchors and their surveyed
coordinates, and each ranging cycle reports, per anchor, a distance or nothing (Decawave PANS
``dwm_loc_get``: a distance list with the anchor positions beside it; MaUWB: a range per
anchor slot and a validity mask). So the reading is exactly that and nothing more:

    obs.sensors["uwb"].anchor_ids      # ("start_w0", ...) in arena order, fixed for the run
    obs.sensors["uwb"].anchor_xyz_m    # (N, 3) surveyed positions, the anchors at mount height
    obs.sensors["uwb"].ranges_m        # (N,) reported range, inf where nothing was measured
    obs.sensors["uwb"].heard           # (N,) isfinite(ranges_m): a view, not a fourth channel

No bearing, no quality factor, no line-of-sight flag. The whole difficulty of UWB indoors is
that a biased range looks exactly like a clean one; a reading that told a policy which ranges
were obstructed would delete the problem the sensor exists to pose. Trilateration is the
policy's job (or a ``PoseSource``'s), and the anchor positions are here because they are the
team's own survey -- the same ``ArenaConfig`` that placed the anchors -- not a leaked world
position (R-POL-3, as amended).

The model
---------

Per anchor, from the drone's **true** position ``(x, y, z)`` and the anchor at
``anchor_height_m``:

1. ``d`` is the three-dimensional distance. A 2.0 m anchor 3 m away reads 3.35 m, and a
   policy that trilaterates in the plane must know that.
2. ``d > max_range_m`` reports ``inf``. Datasheets promise 60 m in line of sight; hobby
   firmware delivers 12-20 m indoors, and 20 m is the default (A-10). At 20 m the whole
   field is within reach of three Start Area anchors, only just; at the 12 m floor the far
   third hears none of them. Even in reach, six anchors in a 5 m-deep strip give poor
   along-field geometry beyond about 14 m and the room's walls obstruct -- which is why the
   Known Search Area's ten aids matter.
3. Line of sight is the segment test the mission uses (R-MISS-2), height-gated per R-SENS-6,
   against **walls and pillars only** (``WorldScene.structural_scene``), at the drone's
   altitude. A mission
   marker is a cardboard box and a teammate is a 30 cm airframe; radio goes through both.
4. In line of sight: ``d + N(0, los_noise_std_m)``. DW1000 timestamp noise is 3-4.5 cm and
   calibrated testbeds report 2-8 cm; the noise does not grow with distance (A-9, 0.05 m).
5. Obstructed: dropped (``inf``) with probability ``nlos_drop_probability`` (A-12, 0.10, no
   published basis), otherwise ``d + nlos_bias_m + N(0, nlos_noise_std_m)``. Through one wall
   or a single obstacle, a DW1000 in a flat built of pre-stressed concrete panels measured a
   mean bias of +0.49 ns and a spread of 1.39 ns -- about +0.15 m and 0.42 m, the spread
   rounded to 0.40 m here (Kolakowski & Modelski, TELFOR 2017, Table 1; arXiv 2403.19706).
   The spread admits negative errors, which the DW1000's received-level bias produces (A-11).
   The same table gives +1.92 ns and 2.02 ns, about +0.58 m and 0.61 m, behind several
   walls; the model does not use it (F-24).
6. With probability ``outlier_probability`` any reported range gains a further positive
   error uniform on ``[0, outlier_max_m]``. The heavy positive tail is documented everywhere
   (LOS p99 of 32 cm; 1.5 m behind a body; "several metres" behind concrete) and its rate
   is published nowhere, so it is **off by default** (A-13) the way ToF noise is.
7. A reported range is never negative.

Every draw comes from the sensor's own generator, and the same number of draws is made per
sample whatever the geometry, so the noise stream is a function of the seed alone (R-DET-2).
Every anchor is measured in the same tick (F-23): the real tag ranges to them one at a time
in TDMA slots of 3-4 ms, so a sweep of up to about twelve fits inside one 50 ms tick and the
first-to-last skew -- under 40 ms for the example's ten -- is under 2 cm at cruise speed,
below the noise. PANS itself returns four ranges per 100 ms frame; a ten-anchor sweep at
10 Hz assumes MaUWB-class firmware.

What is not modelled, and matters
---------------------------------

- **Wall count and wall material.** Obstruction is a boolean: the one-wall numbers apply
  behind three walls too, where the same source measured four times the bias. Its walls were
  concrete panels and still ranged; metal is where a link dies. The venue's walls are
  unmeasured (F-24).
- **Reach.** Whether the module does 12 m or 20 m decides whether the far third of the field
  hears the Start Area at all (A-10). Measure this first.
- **Calibration.** ``nlos_bias_m`` is the only bias. An uncalibrated antenna delay adds
  10-30 cm per anchor pair and drifts about 3 cm per metre; the model assumes the team
  calibrates, which the literature says gets it under 1 cm (F-26).
- **Anchors above 2.0 m.** The line-of-sight test is made at the drone's altitude, exact
  while anchors stand no taller than the inner walls and over-reporting obstruction above
  that (F-25).
- **The consumer.** A range-only sensor is half a feature until a ``PoseSource`` fuses it
  (ADR-0003). This is the sensor; that is the next piece of work.

Placing anchors
---------------

An anchor is a :class:`~safmc_sim.world.landmark.Landmark` of the sensor's ``kind``. A point
(no footprint) is invisible to the ring and to collision, which is right for a radio -- but a
point placed at fixed coordinates can end up inside a generated wall, or inside the Unknown
Search Area, where the rules forbid it and ``validate_arena`` refuses it on every run. So
**survey the generated arena, then place**: ``generate_arena`` first, ``in_known_area`` to
choose positions, ``dataclasses.replace(arena, landmarks=...)`` to place them. Give each a
tripod base, ``radius_m=0.25``, and the generator draws around it while the ring and the
collision check still ignore it (a flat mark is not solid).

Two of the aid rules the arena cannot check for itself -- the cap of ten in the Known Search
Area and the 1 m x 1 m footprint -- because it cannot know which landmarks are aids rather
than scenery. Name the kinds and call
:func:`~safmc_sim.world.arena.validate_nav_aids` yourself; the runner never does (R-WORLD-11).
``examples/04_uwb_ranging.py`` does all of this.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from ..constants import (
    UWB_ANCHOR_HEIGHT_M,
    UWB_LOS_NOISE_STD_M,
    UWB_MAX_RANGE_M,
    UWB_NLOS_BIAS_M,
    UWB_NLOS_DROP_PROBABILITY,
    UWB_NLOS_NOISE_STD_M,
    UWB_OUTLIER_MAX_M,
    UWB_OUTLIER_PROBABILITY,
    UWB_RATE_HZ,
)
from ..errors import ConfigError
from ..world.arena import TARGET_KINDS
from ..world.landmark import Landmark
from .base import Sensor, SensorConfig, TrueState, read_only
from .raycast import RayScene, segment_clear
from .scene import WorldScene

__all__ = [
    "UWBConfig",
    "UWBRanges",
    "UWBTag",
    "anchor_positions",
    "true_ranges",
    "line_of_sight",
    "measure",
    "validate_uwb_config",
]


# ------------------------------------------------------------------------------------------
# The config
# ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UWBConfig(SensorConfig):
    """One tag's ranging model. Every default of the reach and noise model is a named constant
    with an assumption ID; the sweep rate and the anchor height are deployment choices."""

    name: str = "uwb"
    rate_hz: float | None = UWB_RATE_HZ
    """One synchronous sweep of every anchor at 10 Hz. PANS returns four ranges per 100 ms
    frame; a sweep of all N at this rate is what MaUWB-class firmware delivers (F-23). Must
    divide the tick rate."""

    kind: str = "uwb_anchor"
    """The landmark kind this tag ranges to. Anything else in the arena is silent to it."""

    anchor_height_m: float = UWB_ANCHOR_HEIGHT_M
    """Antenna height of every anchor. One number: the sim is 2.5D and altitude comes from
    PX4, not from UWB. Above 2.0 m the obstruction test grows pessimistic (F-25)."""

    max_range_m: float = UWB_MAX_RANGE_M          # A-10
    los_noise_std_m: float = UWB_LOS_NOISE_STD_M  # A-9
    nlos_bias_m: float = UWB_NLOS_BIAS_M          # A-11
    nlos_noise_std_m: float = UWB_NLOS_NOISE_STD_M  # A-11
    nlos_drop_probability: float = UWB_NLOS_DROP_PROBABILITY  # A-12
    outlier_probability: float = UWB_OUTLIER_PROBABILITY      # A-13, off by default
    outlier_max_m: float = UWB_OUTLIER_MAX_M                  # A-13

    def __post_init__(self) -> None:
        super().__post_init__()
        validate_uwb_config(self)

    @property
    def landmark_kinds(self) -> tuple[str, ...]:
        # Declared so the runner refuses an arena whose anchors nobody ranges to, and so an
        # arena with anchors cannot be flown without the tag by accident.
        return (self.kind,)

    def build(self, rng: np.random.Generator) -> "UWBTag":
        # Re-checked here, not trusted from construction: a subclass that skipped
        # super().__post_init__() with nlos_drop_probability=5.0 ran a whole mission with
        # every obstructed range dropped and no complaint -- an auditor did. The arena
        # re-validates its landmarks for the same reason.
        validate_uwb_config(self)
        return UWBTag(self, rng)


def validate_uwb_config(cfg: UWBConfig) -> None:
    """Every invariant a tag's config must satisfy. Run at construction and again at build."""
    if not isinstance(cfg.kind, str) or not cfg.kind:
        raise ConfigError(f"kind must be a non-empty landmark kind, got {cfg.kind!r}")
    if cfg.kind in TARGET_KINDS:
        # Ranging to the mission markers would hand every policy the exact position of every
        # victim and fire on the first sweep, as "surveyed anchors" -- the one thing the
        # observation exists to withhold (R-POL-3). The arena refuses a placed landmark under
        # a mission kind for the mirror-image reason (R-WORLD-7).
        raise ConfigError(
            f"kind {cfg.kind!r} is a mission kind. A tag that ranged to the markers would "
            f"report every victim's and fire's true position as an anchor, which is exactly "
            f"the ground truth a policy must not have (R-POL-3). Anchors are things the team "
            f"placed and surveyed; give them a kind of their own."
        )
    if not _finite(cfg.max_range_m) or cfg.max_range_m <= 0.0:
        raise ConfigError(
            f"max_range_m must be a finite number > 0, got {cfg.max_range_m!r}"
        )
    for field_name in ("anchor_height_m", "los_noise_std_m", "nlos_noise_std_m", "outlier_max_m"):
        value = getattr(cfg, field_name)
        if not _finite(value) or value < 0.0:
            raise ConfigError(f"{field_name} must be a finite number >= 0, got {value!r}")
    if not _finite(cfg.nlos_bias_m):
        raise ConfigError(f"nlos_bias_m must be a finite number, got {cfg.nlos_bias_m!r}")
    for field_name in ("nlos_drop_probability", "outlier_probability"):
        value = getattr(cfg, field_name)
        if not _finite(value) or not 0.0 <= value <= 1.0:
            raise ConfigError(
                f"{field_name} must be a probability in [0, 1], got {value!r}"
            )


def _finite(value: object) -> bool:
    """A real number -- Python or numpy, never a bool -- that is finite."""
    return (
        isinstance(value, (int, float, np.integer, np.floating))
        and not isinstance(value, (bool, np.bool_))
        and bool(np.isfinite(value))
    )


# ------------------------------------------------------------------------------------------
# The reading
# ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UWBRanges:
    """One ranging sweep. What the tag reports and nothing it could not know."""

    anchor_ids: tuple[str, ...]
    """The anchors this tag is configured with, in arena order. Fixed for the run."""

    anchor_xyz_m: np.ndarray
    """``(N, 3)`` surveyed anchor positions, each at the configured mount height. Constant."""

    ranges_m: np.ndarray
    """``(N,)`` reported range to each anchor, ``inf`` where no measurement was obtained --
    out of reach, or dropped behind a wall. A finite value may be biased and the reading does
    not say which."""

    @property
    def heard(self) -> np.ndarray:
        """``(N,)`` bool, True where a range was reported this sweep."""
        return np.isfinite(self.ranges_m)


# ------------------------------------------------------------------------------------------
# The geometry and the model, as pure functions
# ------------------------------------------------------------------------------------------


def anchor_positions(anchors: Sequence[Landmark], height_m: float) -> np.ndarray:
    """``(N, 3)`` anchor antennas: each landmark's ``(x, y)`` at the mount height."""
    if not anchors:
        return np.zeros((0, 3), dtype=float)
    xy = np.array([[a.x, a.y] for a in anchors], dtype=float)
    return np.column_stack((xy, np.full(len(anchors), float(height_m))))


def true_ranges(tag_xyz: np.ndarray, anchor_xyz: np.ndarray) -> np.ndarray:
    """``(N,)`` three-dimensional distances from the tag to each anchor."""
    tag = np.asarray(tag_xyz, dtype=float).reshape(3)
    anchors = np.asarray(anchor_xyz, dtype=float).reshape(-1, 3)
    return np.linalg.norm(anchors - tag, axis=1)


def line_of_sight(scene: RayScene, tag_xy: np.ndarray, anchor_xyz: np.ndarray, z: float) -> np.ndarray:
    """``(N,)`` bool: is the straight path from the tag to each anchor clear of ``scene``?

    ``scene`` should be walls and pillars only -- ``WorldScene.structural_scene`` -- because
    that is what obstructs radio. Tested at altitude ``z`` (R-SENS-6), which is exact while
    anchors stand no taller than the inner walls (F-25).
    """
    anchors = np.asarray(anchor_xyz, dtype=float).reshape(-1, 3)
    if not len(anchors):
        return np.zeros(0, dtype=bool)
    origin = np.asarray(tag_xy, dtype=float).reshape(2)
    return segment_clear(scene, np.tile(origin, (len(anchors), 1)), anchors[:, :2], z)


def measure(
    distance_m: np.ndarray,
    los: np.ndarray,
    cfg: UWBConfig,
    gauss: np.ndarray,
    u_drop: np.ndarray,
    u_outlier: np.ndarray,
    u_size: np.ndarray,
) -> np.ndarray:
    """Turn true distances into reported ranges, given the random draws. Pure.

    ``gauss`` is standard normal and the three ``u_*`` are uniform on ``[0, 1)``, one of each
    per anchor. Taking the draws as arguments is what makes the model testable in isolation
    and what lets :class:`UWBTag` draw the same number of values every sweep.
    """
    d = np.asarray(distance_m, dtype=float)
    los = np.asarray(los, dtype=bool)
    in_reach = d <= cfg.max_range_m
    dropped = (~los) & (np.asarray(u_drop) < cfg.nlos_drop_probability)

    std = np.where(los, cfg.los_noise_std_m, cfg.nlos_noise_std_m)
    bias = np.where(los, 0.0, cfg.nlos_bias_m)
    reported = d + bias + std * np.asarray(gauss)
    outlier = np.asarray(u_outlier) < cfg.outlier_probability
    reported = reported + np.where(outlier, np.asarray(u_size) * cfg.outlier_max_m, 0.0)
    reported = np.maximum(reported, 0.0)
    return np.where(in_reach & ~dropped, reported, np.inf)


# ------------------------------------------------------------------------------------------
# The sensor
# ------------------------------------------------------------------------------------------


class UWBTag(Sensor):
    """One drone's tag. Built from a :class:`UWBConfig`; sampled by the runner."""

    config: UWBConfig

    def __init__(self, config: UWBConfig, rng: np.random.Generator) -> None:
        super().__init__(config, rng)
        # The anchor list is fixed for the run, so it is read from the world once and the
        # read-only copy handed out in every reading is shared, as the ring shares its zone
        # bearings. Filled on the first sample; record_static() needs it too.
        self._anchor_ids: tuple[str, ...] | None = None
        self._anchor_xyz: np.ndarray | None = None
        self._anchor_xyz_ro: np.ndarray | None = None

    def _anchors(self, world: WorldScene) -> tuple[tuple[str, ...], np.ndarray, np.ndarray]:
        if self._anchor_xyz is None:
            anchors = world.landmarks_of(self.config.kind)
            self._anchor_ids = tuple(a.id for a in anchors)
            self._anchor_xyz = anchor_positions(anchors, self.config.anchor_height_m)
            self._anchor_xyz_ro = read_only(self._anchor_xyz)
        return self._anchor_ids, self._anchor_xyz, self._anchor_xyz_ro  # type: ignore[return-value]

    def sample(self, truth: TrueState, world: WorldScene, tick: int) -> UWBRanges:
        ids, xyz, xyz_ro = self._anchors(world)
        distance = true_ranges(np.array([truth.x, truth.y, truth.z]), xyz)
        los = line_of_sight(world.structural_scene, truth.xy, xyz, truth.z)
        # The same four draws per anchor every sweep, whether or not they are used: the noise
        # stream then depends on the seed alone, not on which walls happened to be in the way.
        n = len(ids)
        gauss = self.rng.normal(0.0, 1.0, n)
        u_drop = self.rng.random(n)
        u_outlier = self.rng.random(n)
        u_size = self.rng.random(n)
        ranges = measure(distance, los, self.config, gauss, u_drop, u_outlier, u_size)
        return UWBRanges(anchor_ids=ids, anchor_xyz_m=xyz_ro, ranges_m=read_only(ranges))

    # -- the log -------------------------------------------------------------------------------

    def record(self, reading: UWBRanges):
        """One row per sweep: the reported ranges, ``inf`` where nothing was heard."""
        return {"ranges_m": reading.ranges_m}

    def record_static(self):
        """The anchor positions, so ``uwb.npz`` can be graded without the simulator (R-OBS-3).

        Anchor ids are not stored -- the log holds numeric arrays only -- and need not be:
        column ``j`` is the ``j``-th landmark of this tag's kind in the header's landmark
        list, which is the order the tag reads them in.
        """
        if self._anchor_xyz is None:
            raise ConfigError(
                f"sensor {self.name!r}: record_static() before the first sample. The runner "
                f"samples every sensor at build, before the recorder begins; a sensor that is "
                f"recorded without having been sampled has been driven outside the contract."
            )
        return {"anchor_xyz_m": self._anchor_xyz.copy()}
