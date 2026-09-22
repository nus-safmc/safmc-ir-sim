"""The UWB ranging tag -- range-only radio localisation, on the sensor contract.

**The part is the Qorvo DW3000, and the airframe does not carry one yet.** The team has
chosen the chip; nothing here has been measured on it. So this models the DW3000 as its
datasheet and the published measurements of it describe it, and every number is an
assumption with an ID (A-14..A-18) saying which source it came from and whether that source
used a DW3000 or the older DW1000 most of the literature is written about. ADR-0006 records
the decisions; R-SENS-17 is the contract; ``constants.py`` carries the provenance.

**On the rules.** 6.3 bans wireless transmission in 5.7-5.9 GHz on pain of immediate
disqualification and permits ultra-wideband in the same sentence. The DW3000 has exactly two
channels -- 5 at 6489.6 MHz and 9 at 7987.2 MHz, each 499.2 MHz wide -- so **no channel's
occupied band overlaps 5.7-5.9 GHz**: the nearer edge, 6240.0 MHz, sits 340 MHz clear of
5900 MHz, and no configuration moves it. That is an occupied-bandwidth argument, not an
emissions guarantee: a UWB transmitter has skirts, and the datasheet's own channel-5
spectrum reads about -71 dBm/MHz across the banned band, some 30 dB below its in-band
plateau. Whether that satisfies a referee is a question for the referee, and a board with an
always-on power amplifier -- which some hobby modules have and cannot switch off -- raises it
again. 3.3.1 r.14-17 then say where an anchor may stand: any number in the
Start Area, at most ten in the Known Search Area, none in the Unknown Search Area, each
within 1 m x 1 m, secured and not hung from overhead -- a tripod, in practice.

What a tag reports
------------------

A tag in a real-time-location network is configured with its anchors and their surveyed
coordinates, and each ranging cycle reports, per anchor, a distance or nothing (a DW3000 AT
firmware answers ``AT+RANGE`` with a range and a received level per anchor slot; Decawave's
PANS returns a distance list with the anchor positions beside it). So the reading is exactly
that and nothing more:

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

**Peers, opt-in.** ``UWBConfig(peers=True)`` makes the same tag range to every other drone's
tag as well (R-SENS-18, ADR-0007). The reading then also carries:

    obs.sensors["uwb"].peer_ids        # every agent id in the run, in run order, self included
    obs.sensors["uwb"].peer_ranges_m   # (P,) reported range, inf for self and where unheard
    obs.sensors["uwb"].peers_heard     # (P,) isfinite(peer_ranges_m)

and **nothing about the peer**: not its altitude, not whether it is flying, parked or wrecked.
A tag is a radio, not a rotor, so a landed teammate's tag answers exactly as a flying one's
does -- which is the whole reason the relay of R-MISS-4, a chain of *landed* drones one
metre apart, can be spaced by this sensor. The roster ``peer_ids`` is what a real tag is
configured with; it does hand every policy the fleet size, which nothing else in
``Observation`` does. With ``peers`` off both fields are empty and the reading, the draws
and the log are exactly what they were before the option existed.

The model
---------

Per anchor, from the drone's **true** position ``(x, y, z)`` and the anchor at
``anchor_height_m``:

1. ``d`` is the three-dimensional distance. A 2.0 m anchor 3 m away reads 3.35 m, and a
   policy that trilaterates in the plane must know that.
2. ``d > max_range_m`` reports ``inf``. 20 m is the default (A-15) and it is a firmware
   setting more than a chip limit: 20 m is a stock 6.8 Mb/s board in an office, 40-50 m is
   what Qorvo call typical for line of sight, and 850 kb/s with a long preamble has been
   measured past 90 m indoors (F-29). At 20 m the whole
   field is within reach of three Start Area anchors, only just; at the 12 m floor the far
   third hears none of them. Even in reach, six anchors in a 5 m-deep strip give poor
   along-field geometry beyond about 14 m and the room's walls obstruct -- which is why the
   Known Search Area's ten aids matter.
3. Line of sight is the segment test the mission uses (R-MISS-2), height-gated per R-SENS-6,
   against **walls and pillars only** (``WorldScene.structural_scene``), at the drone's
   altitude. A mission
   marker is a cardboard box and a teammate is a 30 cm airframe; radio goes through both.
4. In line of sight: ``d + N(0, los_noise_std_m)``. The DW3000's datasheet claims a 1.5 cm
   ranging standard deviation, calibrated, at -85 dBm; the best independent measurement of
   the part puts the median line-of-sight error at 6 cm. The default is 5 cm, near the
   pessimistic end of that bracket on purpose (A-14). No published DW3000 error-versus-
   distance curve exists, so the noise does not grow with range here.
5. Obstructed: dropped (``inf``) with probability ``nlos_drop_probability`` (A-17, 0.10, a
   guess -- nobody has published a dropout rate for either part), otherwise
   ``d + nlos_bias_m + N(0, nlos_noise_std_m)``. The bias is a DW3000 measurement: median
   distance error by obstacle, from a half wall at +0.08 m and a door at +0.10 m up to a
   concrete pillar at +0.57 m (Flueratoru et al., WiNTECH'22, Table 2). This arena has both
   thin walls and concrete pillars and one boolean cannot hold both, so the default is
   +0.15 m (A-16). The spread is the number still inherited from the DW1000, because that
   paper publishes medians and no per-obstacle deviation (F-28).
6. With probability ``outlier_probability`` any reported range gains a further positive
   error uniform on ``[0, outlier_max_m]``. The heavy positive tail is documented everywhere
   (LOS p99 of 32 cm; 1.5 m behind a body; "several metres" behind concrete) and its rate
   is published nowhere, so it is **off by default** (A-18) the way ToF noise is.
7. A reported range is never negative.

Every draw comes from the sensor's own generator, and the same number of draws is made per
sample whatever the geometry, so the noise stream is a function of the seed alone (R-DET-2).

**A peer range is the same model applied to a second tag.** The true range is the
three-dimensional distance between the two drones' true positions -- a hovering drone 0.8 m
from a landed one reads 0.94 m, and a policy that spaces a chain must know that. Line of
sight is the same structural segment test at the **lower** of the two altitudes. Inner walls
and pillar shafts are 2.0 m and the perimeter 1.5 m, all above the 1.4 m ceiling, so for
them the altitude does not matter; a
pillar's 0.15 m base does -- it obstructs a landed tag's link and not a hovering one's --
and the scorer's own floor-level line of sight counts that same base, so the lower altitude
is what makes the tag agree with the rule (F-34). Airframes and markers are transparent, as
they are to the anchor link. The peer draws come from a **child generator spawned from the
tag's own at build**, so the anchor noise is the same whether ``peers`` is on or off -- not
merely within a sweep but for the whole run -- and there are four draws per fleet member per
sweep whether or not that peer is in reach, flying, or the tag itself (whose slot is always
``inf``). The other side of that coin: the peer stream is a function of the seed **and the
fleet size**, so adding a drone changes every tag's peer noise from the first sweep, which
R-DET-3's per-agent derivation does not prevent. Nothing in any source this repository has
read measures a tag-to-tag link differently from a tag-to-anchor one, so nothing here does
either.

Every anchor is measured in the same tick, which the radio makes reasonable: a DS-TWR
exchange is three frames of about 170 us, so the whole sweep lives inside one tag's TDMA
slot and the first-to-last skew is under 2 cm at cruise speed (F-23). **What the fixed rate
does not carry is the fleet.** Slots are per tag, so the sweep rate is
``1 / (n_tags * slot)`` -- ten drones get 10 Hz each and twenty-five get 4 Hz, on the same
radio. :func:`sweep_rate_hz` computes it and the caller passes the answer to ``rate_hz``;
nothing does it automatically, because a sensor config knows nothing about the fleet
(F-32). **With peers on, the slot itself grows**: a tag now makes ``anchors + tags - 1``
exchanges in its own slot, and a shipping firmware fits eight per 10 ms (A-19), so
:func:`peer_sweep_rate_hz` gives 5 Hz at ten drones and one anchor and **1 Hz at
twenty-five** -- about four times under what a broadcast swarm-ranging protocol measured on
a DW1000 at 13-14 drones, and further under it as the fleet grows (F-33). Pessimistic on
purpose; measure it.

What is not modelled, and matters
---------------------------------

- **Wall count and wall material.** Obstruction is a boolean: the one-wall numbers apply
  behind three walls too, where the same source measured four times the bias. Its walls were
  concrete panels and still ranged; metal is where a link dies. The venue's walls are
  unmeasured (F-24).
- **Reach, and that it is a firmware setting.** 20 m is a stock 6.8 Mb/s configuration in an
  office; 850 kb/s with a long preamble has been measured past 90 m indoors on this part.
  The model has one scalar and cannot express that trade (F-29), so measure A-15 in the
  configuration you will actually fly.
- **Calibration, which is a per-unit constant, not noise.** The DW3000 datasheet puts
  ranging at +/-15 cm uncalibrated and +/-6 cm calibrated, and a DWM3000 module ships with no
  antenna-delay calibration at all while a DWM3001C ships factory-calibrated. That is a fixed
  offset per anchor pair, structurally unlike the zero-mean noise modelled here, and it is
  absent (F-31). Which module the team buys decides how big it is.
- **Anchors above 2.0 m.** The line-of-sight test is made at the drone's altitude, exact
  while anchors stand no taller than the inner walls and over-reporting obstruction above
  that (F-25).
- **Peer ranging's protocol.** The rate model is the naive schedule -- every tag initiates
  to every other, eight exchanges per slot -- and the body between two tags is transparent;
  two uncalibrated tags carry two antenna-delay offsets this model has no term for (F-33,
  F-34). A parked teammate is ranged exactly but is invisible to the ring and cannot be
  collided with (F-35), which a relay built on this sensor rests on.
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

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from ..constants import (
    UWB_ANCHOR_HEIGHT_M,
    UWB_PEER_EXCHANGES_PER_SLOT,
    UWB_SLOT_S,
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
from .scene import Fleet, WorldScene

__all__ = [
    "UWBConfig",
    "UWBRanges",
    "UWBTag",
    "anchor_positions",
    "true_ranges",
    "line_of_sight",
    "peer_line_of_sight",
    "measure",
    "sweep_rate_hz",
    "peer_sweep_rate_hz",
    "validate_uwb_config",
]


# ------------------------------------------------------------------------------------------
# Picking a rate: the one DW3000 number that depends on the fleet
# ------------------------------------------------------------------------------------------


def sweep_rate_hz(n_tags: int, slot_s: float = UWB_SLOT_S) -> float:
    """How often one tag completes a full sweep of its anchors, in a fleet of ``n_tags``.

    A TDMA network gives each **tag** a slot and lets it range to every anchor inside that
    slot, so the sweep rate is ``1 / (n_tags * slot_s)`` -- it falls with the size of the
    swarm and does *not* depend on how many anchors are placed. At the 10 ms slot a shipping
    6.8 Mb/s firmware allows, ten drones get 10 Hz each and twenty-five get 4 Hz.

    This is a helper for choosing :attr:`UWBConfig.rate_hz`, not something the runner applies:
    a sensor config knows nothing about the fleet, and wiring one to the other would make a
    sensor's timing depend on a field of ``RunConfig`` invisibly. Call it, and pass the answer:

        n = 25
        RunConfig(n_drones=n, sensors=flown_sensors() + (UWBConfig(rate_hz=sweep_rate_hz(n)),))

    The rate must still divide the tick rate exactly (R-TIME-3), and not every fleet size
    gives one that does. On the 20 Hz loop at a 10 ms slot the decimation works out to
    ``0.2 * n_tags``, so fleets that are a multiple of five divide exactly (10 drones give
    10 Hz, 15 give 6.67 Hz, 25 give 4 Hz) and the rest do not -- eleven drones want 9.09 Hz,
    which the runner refuses by design rather than rounding it. Pick the next rate down that
    divides, and say in the write-up that you did. F-32.
    """
    if not isinstance(n_tags, int) or isinstance(n_tags, bool) or n_tags < 1:
        raise ConfigError(f"n_tags must be an integer >= 1, got {n_tags!r}")
    if not _finite(slot_s) or slot_s <= 0.0:
        raise ConfigError(f"slot_s must be a finite number > 0, got {slot_s!r}")
    return 1.0 / (n_tags * slot_s)


def peer_sweep_rate_hz(
    n_tags: int,
    n_anchors: int,
    exchanges_per_slot: int = UWB_PEER_EXCHANGES_PER_SLOT,
    slot_s: float = UWB_SLOT_S,
) -> float:
    """How often one tag sweeps its anchors **and every other tag**, in a fleet of ``n_tags``.

    With peers on, a tag makes ``n_anchors + n_tags - 1`` double-sided exchanges inside its
    own slot. A shipping firmware completes eight per 10 ms slot (A-19), so the tag needs
    ``ceil(exchanges / 8)`` slots and the superframe is ``n_tags`` of them:

        10 drones, 1 anchor  ->  10 exchanges  ->  2 slots  ->  1 / (10 * 0.020) = 5 Hz
        25 drones, 1 anchor  ->  25 exchanges  ->  4 slots  ->  1 / (25 * 0.040) = 1 Hz

    Both halves of that are assumptions, and both are pessimistic: every tag initiates to
    every other, so each pair is ranged twice per superframe (a symmetric schedule halves
    it), and the exchange budget is the AT firmware's 1.25 ms rather than the ~0.5 ms of
    airtime. A broadcast swarm-ranging protocol measured 16 Hz per pair at 13-14 drones on a
    DW1000, against 3.6-3.9 Hz from this budget at that fleet size -- about four times, and
    the gap widens with the fleet because this schedule is quadratic in it and a broadcast is
    linear (F-33). The pessimistic figure is the default because it is the firmware the team
    would fly first.

    Like :func:`sweep_rate_hz`, this is a helper for choosing :attr:`UWBConfig.rate_hz`, not
    something the runner applies, and the answer must still divide the tick rate (R-TIME-3).
    On the 20 Hz loop the decimation is ``0.2 * n_tags * slots``, so ten, fifteen, twenty and
    twenty-five drones with one anchor all divide (4, 6, 12 and 20 ticks) while twelve do not
    (4.8) -- pick the next rate down that divides, and say in the write-up that you did.
    """
    if not isinstance(n_tags, int) or isinstance(n_tags, bool) or n_tags < 1:
        raise ConfigError(f"n_tags must be an integer >= 1, got {n_tags!r}")
    if not isinstance(n_anchors, int) or isinstance(n_anchors, bool) or n_anchors < 0:
        raise ConfigError(f"n_anchors must be an integer >= 0, got {n_anchors!r}")
    if not isinstance(exchanges_per_slot, int) or isinstance(exchanges_per_slot, bool) \
            or exchanges_per_slot < 1:
        raise ConfigError(
            f"exchanges_per_slot must be an integer >= 1, got {exchanges_per_slot!r}"
        )
    if not _finite(slot_s) or slot_s <= 0.0:
        raise ConfigError(f"slot_s must be a finite number > 0, got {slot_s!r}")
    exchanges = n_anchors + n_tags - 1
    slots = max(1, math.ceil(exchanges / exchanges_per_slot))
    return 1.0 / (n_tags * slots * slot_s)


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

    max_range_m: float = UWB_MAX_RANGE_M          # A-15
    los_noise_std_m: float = UWB_LOS_NOISE_STD_M  # A-14
    nlos_bias_m: float = UWB_NLOS_BIAS_M          # A-16
    nlos_noise_std_m: float = UWB_NLOS_NOISE_STD_M  # A-16
    nlos_drop_probability: float = UWB_NLOS_DROP_PROBABILITY  # A-17
    outlier_probability: float = UWB_OUTLIER_PROBABILITY      # A-18, off by default
    outlier_max_m: float = UWB_OUTLIER_MAX_M                  # A-18

    peers: bool = False
    """Also range to every other drone's tag (R-SENS-18). Off by default: the reading,
    the draws and the log are then exactly what they were before the option existed. On,
    the same noise model applies to each peer link from a child generator of the tag's own,
    so the anchor noise is unchanged, and the slot budget grows -- pass
    ``rate_hz=peer_sweep_rate_hz(n_drones, n_anchors)`` (A-19, F-33)."""

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
    if not isinstance(cfg.peers, (bool, np.bool_)):
        # A truthy non-bool -- peers=1, peers="yes" -- would switch peer ranging on and log
        # a config that does not say so plainly.
        raise ConfigError(f"peers must be a bool, got {cfg.peers!r}")


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

    peer_ids: tuple[str, ...] = ()
    """Every drone in the run, in run order, this tag's own drone included. Fixed for the
    run. Empty unless ``UWBConfig(peers=True)``."""

    peer_ranges_m: np.ndarray = field(default_factory=lambda: read_only(np.zeros(0)))
    """``(P,)`` reported range to each peer's tag, ``inf`` for this tag's own slot and
    wherever nothing was heard. Three-dimensional: a hovering drone 0.8 m from a landed one
    reads 0.94 m. Says nothing about the peer -- not its altitude, not its lifecycle."""

    @property
    def heard(self) -> np.ndarray:
        """``(N,)`` bool, True where a range was reported this sweep."""
        return np.isfinite(self.ranges_m)

    @property
    def peers_heard(self) -> np.ndarray:
        """``(P,)`` bool, True where a peer range was reported this sweep. Never at self."""
        return np.isfinite(self.peer_ranges_m)


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


def peer_line_of_sight(scene: RayScene, tag_xyz: np.ndarray, peer_xyz: np.ndarray) -> np.ndarray:
    """``(P,)`` bool: is the straight path from the tag to each peer's tag clear of ``scene``?

    Tested at the **lower** of the two altitudes, pair by pair (R-SENS-18). Walls (2.0 m
    inner, 1.5 m perimeter) and pillar shafts stand above the ceiling, so for them the
    choice changes nothing; a pillar's
    0.15 m base obstructs a landed tag and not a hovering one, and the scorer's floor-level
    line of sight counts it too, so the lower altitude is the one that agrees with the rule
    (F-34). Pairs are grouped by their test altitude so a fleet costs one segment cast per
    distinct altitude rather than one per peer.
    """
    tag = np.asarray(tag_xyz, dtype=float).reshape(3)
    peers = np.asarray(peer_xyz, dtype=float).reshape(-1, 3)
    if not len(peers):
        return np.zeros(0, dtype=bool)
    z_test = np.minimum(peers[:, 2], tag[2])
    clear = np.zeros(len(peers), dtype=bool)
    for z in np.unique(z_test):
        rows = np.flatnonzero(z_test == z)
        clear[rows] = segment_clear(
            scene, np.tile(tag[:2], (len(rows), 1)), peers[rows, :2], float(z)
        )
    return clear


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
        # Peer noise comes from a child of the tag's generator, spawned here and never from
        # the parent's stream: with peers off the parent is untouched and an old log
        # reproduces byte for byte; with peers on the anchor noise is still the same run.
        # Drawing peers from the parent *after* the anchors was the first design, and it
        # only held within a sweep -- by the next sweep the parent had advanced (R-DET-3).
        self._peer_rng: np.random.Generator | None = (
            rng.spawn(1)[0] if config.peers else None
        )

    def _anchors(self, world: WorldScene) -> tuple[tuple[str, ...], np.ndarray, np.ndarray]:
        if self._anchor_xyz is None:
            anchors = world.landmarks_of(self.config.kind)
            self._anchor_ids = tuple(a.id for a in anchors)
            self._anchor_xyz = anchor_positions(anchors, self.config.anchor_height_m)
            self._anchor_xyz_ro = read_only(self._anchor_xyz)
        return self._anchor_ids, self._anchor_xyz, self._anchor_xyz_ro  # type: ignore[return-value]

    def sample(self, truth: TrueState, world: WorldScene, tick: int) -> UWBRanges:
        ids, xyz, xyz_ro = self._anchors(world)
        tag_xyz = np.array([truth.x, truth.y, truth.z])
        distance = true_ranges(tag_xyz, xyz)
        los = line_of_sight(world.structural_scene, truth.xy, xyz, truth.z)
        # The same four draws per anchor every sweep, whether or not they are used: the noise
        # stream then depends on the seed alone, not on which walls happened to be in the way.
        ranges = measure(distance, los, self.config, *self._draws(len(ids)))
        if not self.config.peers:
            return UWBRanges(anchor_ids=ids, anchor_xyz_m=xyz_ro, ranges_m=read_only(ranges))

        # Peers draw from their own child generator, so the anchor noise above is the same
        # run with or without them; and there are four draws per peer whatever its lifecycle
        # or reach, including the tag's own slot, which is then overwritten with inf.
        fleet: Fleet = world.fleet
        peer_distance = true_ranges(tag_xyz, fleet.xyz)
        peer_los = peer_line_of_sight(world.structural_scene, tag_xyz, fleet.xyz)
        peer_ranges = measure(
            peer_distance, peer_los, self.config, *self._draws(len(fleet), self._peer_rng)
        )
        me = fleet.index_of(truth.object_id)
        if me >= 0:
            peer_ranges[me] = np.inf
        return UWBRanges(
            anchor_ids=ids, anchor_xyz_m=xyz_ro, ranges_m=read_only(ranges),
            peer_ids=fleet.agent_ids, peer_ranges_m=read_only(peer_ranges),
        )

    def _draws(self, n: int, rng: np.random.Generator | None = None):
        """Four arrays of ``n`` draws in the order :func:`measure` takes them."""
        rng = self.rng if rng is None else rng
        return (
            rng.normal(0.0, 1.0, n),
            rng.random(n),
            rng.random(n),
            rng.random(n),
        )

    # -- the log -------------------------------------------------------------------------------

    def record(self, reading: UWBRanges):
        """One row per sweep: the reported ranges, ``inf`` where nothing was heard.

        With peers on, ``peer_ranges_m`` too, stacked to ``(ticks, agents, agents)``: column
        ``j`` is the ``j``-th entry of the header's ``agents`` list (R-SENS-18). With peers
        off the row is exactly what it was, so an old log and a new one are the same file.
        """
        if not self.config.peers:
            return {"ranges_m": reading.ranges_m}
        return {"ranges_m": reading.ranges_m, "peer_ranges_m": reading.peer_ranges_m}

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
