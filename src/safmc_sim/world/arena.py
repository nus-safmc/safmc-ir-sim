"""Seeded generation and validation of SAFMC Category Swarm arenas.

The rulebook never uses the words "random" or "randomise" -- a full-text sweep of all 27 pages
finds neither. What it does instead is withhold, repeatedly and deliberately, and that is what
makes a *distribution* the honest thing to model. Verbatim from section 3.2 of the 2026
Category Swarm Challenge Booklet:

    "The layout is not drawn to scale."
    "The placements of the inner walls will follow the diagram, but the exact positions and
     dimensions will NOT be given."
    "The layout within the Unknown Search Area is intentionally NOT shown."
    "The placement and number of victims and fires shown are for illustration purposes only."

A policy tuned against one hand-drawn map measures overfitting, so this module produces a
*distribution* of arenas from a seed and refuses to emit one that violates a published
constraint (R-WORLD-1, R-WORLD-3, R-WORLD-4).

Published geometry that is *not* randomised, because it is given: the 20 x 20 m field, the
20 x 6 m Start Area, the 10 x 10 m Unknown Search Area, wall and pillar heights, the minimum
gaps, and the 1.4 m ceiling. See docs/01-competition.md.

A finding that fell out of building this, and that matters strategically
-----------------------------------------------------------------------
Taken literally, the published constraints very nearly determine the arena.

The Known Search Area is 14 m deep (derived: 20 m field minus the 6 m Start Area) and
the Unknown Search Area is 10 m, leaving **4 m of north-south slack in total** for a room that
must clear the north perimeter by the published 2 m gap. The room's south side faces the Start
Area boundary, which is a virtual line rather than a wall, so no gap rule applies there -- its
position has roughly 1.9 m of freedom, no more.

The consequence is what matters. Once the room is placed, the free space in the Known Search
Area is a **corridor ring of about 2 m** around a central 10 x 10 m room, widening on one
side. There is almost no room left for the free-standing maze walls the diagram depicts: any
such wall must clear both the room and the perimeter by 2 m, and those constraints nearly
exhaust the space. Empirically about one fits.

Two things follow:

1. **Search in the Known Search Area is closer to one-dimensional than two.** A ring corridor
   rewards a very different policy from an open field -- coverage is nearly a traversal
   problem, and the interesting decisions are which doorway to enter and when to commit.
2. **The 2 m gap is not an all-pairs constraint.** This was posed here as an open fork, with
   A-6 as the alternative culprit. A-6 has since been retired -- 14 m is arithmetically forced,
   not assumed, so it cannot be the wrong half -- and the diagram settles the rest against the
   all-pairs reading: five of the six wall structures drawn in the Known Search Area are
   *anchored*, at zero gap, to a perimeter wall or to the Unknown Search Area room. Under an
   all-pairs reading the rulebook's own diagram would violate the rulebook's own table.

   The reading that reconciles them is that 2 m is a floor on the **navigable gap** a drone
   flies through, and walls may touch each other and the perimeter. This module still places
   free-standing walls the old way, so the Known Search Area remains sparser than the diagram;
   :mod:`safmc_sim.world.maze` adopts the anchored reading inside the room, and exposes
   ``ArenaConfig.maze_anchor_gap_m`` so both readings stay reachable. Any result quoted from
   this simulator should say which value it used.

(An earlier version of this note claimed the room's north-south position was *forced*. That was
wrong: it required a 2 m gap on the south side, where there is no wall to be 2 m from. The
strategic conclusion survives; the geometry claim did not.)
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.strtree import STRtree

from ..constants import (
    CEILING_M,
    DRONE_RADIUS_M,
    FIELD_DEPTH_M,
    FIELD_WIDTH_M,
    INNER_WALL_HEIGHT_M,
    MARKER_FOOTPRINT_M,
    MARKER_HEIGHT_M,
    MAZE_CORRIDOR_M,
    MIN_GAP_PILLAR_M,
    MIN_GAP_WALL_TO_WALL_M,
    NAV_AID_FOOTPRINT_M,
    NAV_AID_MAX_KNOWN_AREA,
    N_BONUS_VICTIMS,
    N_FIRES,
    N_VICTIMS,
    PERIMETER_WALL_HEIGHT_M,
    PILLAR_BASE_DIAMETER_M,
    PILLAR_BASE_HEIGHT_M,
    PILLAR_DIAMETER_M,
    PILLAR_HEIGHT_M,
    SAFETY_NET_HEIGHT_M,
    SCORE_RADIUS_M,
    START_AREA_DEPTH_M,
    UNKNOWN_AREA_DOORWAY_M,
    UNKNOWN_AREA_DOORWAYS,
    UNKNOWN_AREA_SIZE_M,
    WALL_THICKNESS_M,
)
from ..errors import ArenaError, ConfigError
from ..sensors.raycast import RayScene
from .landmark import Landmark, occluder_scene, validate_landmark_fields
from .maze import MazeGrid, generate_maze, plan_grid

__all__ = [
    "Wall",
    "Pillar",
    "Landmark",
    "Target",
    "ArenaSpec",
    "ArenaConfig",
    "generate_arena",
    "validate_arena",
    "validate_nav_aids",
    "TARGET_KINDS",
]

TARGET_KINDS = ("victim", "bonus_victim", "fire")

# Grid resolution for the connectivity check. Fine enough to find a 1 m doorway, coarse
# enough that a 20 x 20 m field is a 200 x 200 flood fill.
_CONNECTIVITY_RES_M = 0.10


@dataclass(frozen=True)
class Wall:
    """A wall, described by its centre line and thickness.

    Stored as a centre line rather than a polygon because that is how walls are actually laid
    out at the venue, and because it maps directly onto ir-sim's origin-centred rectangle.
    """

    x1: float
    y1: float
    x2: float
    y2: float
    thickness_m: float = WALL_THICKNESS_M
    height_m: float = INNER_WALL_HEIGHT_M
    kind: str = "inner_wall"

    @property
    def length_m(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

    @property
    def centre(self) -> tuple[float, float]:
        return (0.5 * (self.x1 + self.x2), 0.5 * (self.y1 + self.y2))

    @property
    def angle_rad(self) -> float:
        return math.atan2(self.y2 - self.y1, self.x2 - self.x1)

    def corners(self) -> np.ndarray:
        """The four corners, counter-clockwise. Shape (4, 2)."""
        cx, cy = self.centre
        half_l, half_t = self.length_m / 2.0, self.thickness_m / 2.0
        ca, sa = math.cos(self.angle_rad), math.sin(self.angle_rad)
        local = np.array(
            [[-half_l, -half_t], [half_l, -half_t], [half_l, half_t], [-half_l, half_t]]
        )
        rot = np.array([[ca, -sa], [sa, ca]])
        return local @ rot.T + np.array([cx, cy])

    def segments(self) -> np.ndarray:
        """The four edges as ray-castable segments. Shape (4, 4)."""
        c = self.corners()
        return np.stack([np.concatenate([c[i], c[(i + 1) % 4]]) for i in range(4)])

    def polygon(self) -> Polygon:
        return Polygon(self.corners())


@dataclass(frozen=True)
class Pillar:
    """A 2.0 m column on a weighted foot: in profile an upside-down T.

    The rulebook gives the Pillar Obstacle as "0.3m diameter, 2m height (includes a weighted
    circular base of 0.5m diameter and 0.15m height)". The base is staging hardware -- a heavy
    foot so a 2 m column does not topple, the same reason a flagpole has one -- not an obstacle
    feature, and it lives entirely below 0.15 m.

    That split decides where each part belongs:

    - :meth:`polygon` is the **shaft only**, and it is what placement, the published gap checks
      and the occupancy grid consume. Those all answer "can a drone fly through here", and a
      drone cruises at 0.5 m, well above the foot. Folding the base in would tighten every gap
      by 0.10 m per side against a 1.0 m rule and shrink the maze lattice for a body no drone
      at cruise altitude can reach.
    - The base *is* real to a drone that descends to land beside a pillar, so it is emitted as a
      second, low height band in :meth:`ArenaSpec.structural_scene`. The raycaster is already
      banded (``z_min <= z < height``), so this costs one extra circle and nothing else.
    """

    x: float
    y: float
    radius_m: float = PILLAR_DIAMETER_M / 2.0
    height_m: float = PILLAR_HEIGHT_M
    base_radius_m: float = PILLAR_BASE_DIAMETER_M / 2.0
    base_height_m: float = PILLAR_BASE_HEIGHT_M

    def polygon(self) -> Polygon:
        return Polygon(
            [
                (
                    self.x + self.radius_m * math.cos(t),
                    self.y + self.radius_m * math.sin(t),
                )
                for t in np.linspace(0, 2 * math.pi, 24, endpoint=False)
            ]
        )


@dataclass(frozen=True)
class Target(Landmark):
    """A mission marker: a victim, a bonus victim, or a fire. A landmark that scores.

    Solid by default -- a 0.30 m footprint, 1.0 m tall -- so it occludes time-of-flight rays
    at cruise altitude and can be struck (F-11). Markers are *not* ir-sim obstacles, and the
    rules are explicit that they do not block line of sight for scoring, which mentions only
    "walls or pillars" (3.3.4 r.1). The line-of-sight scene is built from structure alone, so
    that stays true by construction.

    A ``Target`` is generated by the arena and tracked by the mission. To place a marker that
    a camera can read but that does not score -- a navigation tag, a start mark -- use a plain
    :class:`Landmark` with a kind of your own.
    """

    radius_m: float = MARKER_FOOTPRINT_M / 2.0
    height_m: float = MARKER_HEIGHT_M

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.kind not in TARGET_KINDS:
            raise ConfigError(f"unknown target kind {self.kind!r}, expected one of {TARGET_KINDS}")


def _reject_decoy(lm: Landmark, error: type) -> None:
    """A placed landmark may not look like a mission target.

    A ``Target``, or a plain ``Landmark`` with a mission kind, would be reported by the camera
    as a victim or a fire and could never score -- a decoy the policy author did not ask for.
    Targets are generated by the arena and tracked by the mission; nothing else may wear
    their kinds.
    """
    if isinstance(lm, Target) or lm.kind in TARGET_KINDS:
        raise error(
            f"landmark {lm.id!r} has the mission kind {lm.kind!r} but is not a generated "
            f"target. The camera would report it as a {lm.kind!r} and it could never score. "
            f"Targets are placed by the generator; give a placed landmark a kind of its own."
        )


@dataclass(frozen=True)
class ArenaConfig:
    """Knobs for generation. Published values are not knobs and live in constants.py."""

    n_inner_walls: int = 1
    n_pillars_known: int = 6
    n_unknown_walls: int = 0
    """Free-standing baffles inside the room. Zero by default because the maze supplies its
    walls; only meaningful alongside ``maze_corridor_m=None``."""
    n_pillars_unknown: int = 2
    n_victims: int = N_VICTIMS
    n_bonus_victims: int = N_BONUS_VICTIMS
    n_fires: int = N_FIRES
    inner_wall_length_range_m: tuple[float, float] = (2.0, 5.0)
    max_placement_attempts: int = 4000
    wall_thickness_m: float = WALL_THICKNESS_M
    min_gap_wall_m: float = MIN_GAP_WALL_TO_WALL_M
    min_gap_pillar_m: float = MIN_GAP_PILLAR_M

    maze_corridor_m: float | None = MAZE_CORRIDOR_M
    """Floor on corridor width inside the Unknown Search Area, or ``None`` for no maze.

    ``None`` restores the pre-maze behaviour exactly: the room's interior is filled by the same
    rejection sampler as the Known Search Area, which yields ``n_unknown_walls`` free-standing
    baffles a drone can simply fly around. Useful only for reproducing old results.
    """

    maze_anchor_gap_m: float = 0.0
    """Clearance retracted from BOTH ends of every maze run.

    ``0.0`` anchors walls to the room, which is what makes dead ends possible. Setting this to
    ``min_gap_wall_m`` turns every maze wall into a free-floating island and degenerates to the
    strict all-pairs reading of the 2 m gap. A metric knob rather than a string enum so it
    passes the units gate in ``test_every_config_field_declares_its_units``.
    """

    n_maze_loops: int = 2
    """Extra walls knocked out after the spanning tree, creating cycles.

    Zero gives a perfect maze: exactly one route between any two cells, which makes the 3.3.7
    relay chain a single fragile path and any collision a total blockage.
    """
    """Both gaps are published values, exposed here because they interact with the Unknown
    Search Area's size in a way that nearly determines the layout -- see the module docstring.
    Lower them only to explore alternative readings of the rulebook, and say so in results."""

    landmarks: tuple[Landmark, ...] = ()
    """Landmarks placed at fixed, known positions in every arena from this config: surveyed
    navigation tags, start-point marks, radio anchors. Fixed rather than seeded because that
    is what a surveyed landmark *is* -- its whole value is that its position is known. A
    solid one counts as structure for generation: walls and pillars keep their published
    gaps from it and the layout is drawn around it. For a placement that depends on the
    generated layout (a tag on each doorway), generate first and then
    ``dataclasses.replace(arena, landmarks=...)``. Mission targets do not go here; the
    generator places them and the mission scores them."""

    def __post_init__(self) -> None:
        seen: set[str] = set()
        for lm in self.landmarks:
            if not isinstance(lm, Landmark):
                raise ConfigError(
                    f"landmarks must be Landmark instances, got {type(lm).__name__}"
                )
            _reject_decoy(lm, ConfigError)
            if lm.id in seen:
                raise ConfigError(f"duplicate landmark id {lm.id!r}")
            seen.add(lm.id)
        if self.n_bonus_victims < 1 or self.n_fires < 1:
            raise ConfigError(
                "the rulebook guarantees the Unknown Search Area contains at least one bonus "
                "victim and at least one fire (3.3.9 r.2), so both counts must be >= 1"
            )
        lo, hi = self.inner_wall_length_range_m
        if not 0 < lo <= hi:
            raise ConfigError(f"inner_wall_length_range_m must be 0 < lo <= hi, got {(lo, hi)}")
        if self.maze_corridor_m is not None and self.n_unknown_walls:
            raise ConfigError(
                "the maze IS the Unknown Search Area's walls, so n_unknown_walls must be 0 when "
                "maze_corridor_m is set. The two passes are mutually exclusive: a free-standing "
                "inner_wall is gap-checked all-pairs and could never sit 2 m from every maze "
                "wall. Set maze_corridor_m=None to go back to free-standing baffles."
            )
        for name in ("n_victims", "n_bonus_victims", "n_fires", "n_inner_walls",
                     "n_pillars_known", "n_unknown_walls", "n_pillars_unknown", "n_maze_loops"):
            if getattr(self, name) < 0:
                raise ConfigError(f"{name} must be >= 0, got {getattr(self, name)}")
        if self.maze_anchor_gap_m < 0.0:
            raise ConfigError(
                f"maze_anchor_gap_m must be >= 0, got {self.maze_anchor_gap_m}"
            )


@dataclass(frozen=True)
class ArenaSpec:
    """A fully resolved, validated arena. Immutable, serialisable, and the unit of a run."""

    seed: int
    width_m: float
    depth_m: float
    ceiling_m: float
    start_area_depth_m: float
    unknown_area: tuple[float, float, float, float]
    walls: tuple[Wall, ...]
    pillars: tuple[Pillar, ...]
    targets: tuple[Target, ...]
    landmarks: tuple[Landmark, ...] = ()
    """Non-mission landmarks. Targets are landmarks too, but they live in ``targets`` because
    the mission owns them; ``all_landmarks`` is the union a sensor sees."""

    layout_seed: int | None = None
    unknown_seed: int | None = None
    mission_seed: int | None = None
    """The per-stream seeds this arena was built with, or ``None`` where the stream was derived
    from :attr:`seed`. Recorded so an arena built with overrides can be *regenerated*, not
    merely replayed: ``seed`` alone would not reproduce it. See :func:`generate_arena`."""
    config: ArenaConfig = field(default_factory=ArenaConfig)

    # -- landmarks ------------------------------------------------------------------------

    @property
    def all_landmarks(self) -> tuple[Landmark, ...]:
        """Everything a sensor may perceive by kind: the mission targets, then the rest."""
        return tuple(self.targets) + tuple(self.landmarks)

    def landmarks_of(self, kind: str) -> tuple[Landmark, ...]:
        return tuple(lm for lm in self.all_landmarks if lm.kind == kind)

    # -- scenes ---------------------------------------------------------------------------

    def structural_scene(self) -> RayScene:
        """Walls and pillars only. Blocks both sensing and line of sight.

        A pillar contributes **two** circles, not one: the 0.30 m shaft over its full height,
        and the 0.50 m weighted base below 0.15 m. So a drone at the 0.5 m cruise altitude
        ranges against the shaft alone -- exact -- while one descending to land beside a pillar
        meets the wider foot it would really clip. The bands are what make that free: a
        primitive occludes only while ``z_min <= z < height_m``.
        """
        segments = (
            np.vstack([w.segments() for w in self.walls])
            if self.walls
            else np.zeros((0, 4))
        )
        seg_h = np.concatenate([np.full(4, w.height_m) for w in self.walls]) if self.walls else np.zeros(0)
        rows: list[list[float]] = []
        heights: list[float] = []
        for p in self.pillars:
            rows.append([p.x, p.y, p.radius_m])
            heights.append(p.height_m)
            # Only when the foot is genuinely wider than the shaft; a degenerate ring would be
            # an invisible primitive the raycaster still pays for on every ray.
            if p.base_radius_m > p.radius_m and p.base_height_m > 0.0:
                rows.append([p.x, p.y, p.base_radius_m])
                heights.append(p.base_height_m)
        circles = np.array(rows, dtype=float) if rows else np.zeros((0, 3))
        circ_h = np.array(heights, dtype=float) if heights else np.zeros(0)
        return RayScene(
            circles=circles, circle_heights=circ_h, segments=segments, segment_heights=seg_h
        )

    def landmark_scene(self) -> RayScene:
        """Solid landmarks only -- markers and any placed body. Blocks sensing; never line of
        sight (R-MISS-2). Point landmarks contribute nothing: a ray cannot hit a tag."""
        return occluder_scene(self.all_landmarks)

    # -- ir-sim bridge --------------------------------------------------------------------

    def to_irsim_world(self, step_time: float) -> dict:
        """The ``world:`` block of an ir-sim YAML config."""
        return {
            "width": self.width_m,
            "height": self.depth_m,
            "step_time": step_time,
            "sample_time": step_time,
            "offset": [0.0, 0.0],
            "control_mode": "auto",
            # Colliding drones freeze. This mirrors the target paper's setup and is the
            # honest model of "a drone that hit a wall is out of the run" -- the rules allow
            # no mid-run repair. The runner records it as CRASHED and stops commanding it.
            "collision_mode": "stop",
        }

    def to_irsim_obstacles(self) -> list[dict]:
        """The ``obstacle:`` block. Walls and pillars only -- markers are handled by us.

        Markers stay out because ir-sim's collision is strictly 2D and would make a 1.0 m
        marker impassable at every altitude, including altitudes at which a drone could
        legally overfly it. The runner does a height-gated marker collision check instead.
        """
        items: list[dict] = []
        for wall in self.walls:
            cx, cy = wall.centre
            items.append(
                {
                    "shape": {
                        "name": "rectangle",
                        "length": wall.length_m,
                        "width": wall.thickness_m,
                    },
                    "state": [cx, cy, wall.angle_rad],
                    "color": "dimgray",
                }
            )
        for pillar in self.pillars:
            items.append(
                {
                    "shape": {"name": "circle", "radius": pillar.radius_m},
                    "state": [pillar.x, pillar.y, 0.0],
                    "color": "slategray",
                }
            )
        return items

    # -- queries --------------------------------------------------------------------------

    @property
    def start_area(self) -> tuple[float, float, float, float]:
        """``(x0, y0, x1, y1)`` of the Start Area: the full-width southern strip."""
        return (0.0, 0.0, self.width_m, self.start_area_depth_m)

    def in_start_area(self, x: float, y: float) -> bool:
        x0, y0, x1, y1 = self.start_area
        return x0 <= x <= x1 and y0 <= y <= y1

    def in_unknown_area(self, x: float, y: float) -> bool:
        x0, y0, x1, y1 = self.unknown_area
        return x0 <= x <= x1 and y0 <= y <= y1

    def in_known_area(self, x: float, y: float) -> bool:
        """The Known Search Area: inside the field, north of the Start Area, outside the room.

        The rulebook names three zones and they carry different *permissions*, not just
        different geometry. Teams may enter this one during setup and place up to ten
        navigation aids in it (3.3.1 r.15-16); they may never enter the Unknown Search Area
        (r.17). So this predicate is the one that says where a team may legally put a tag, and
        it is what :func:`_validate_landmark_zones` enforces.

        Note it depends on the *generated* room position, which is why a nav aid's location
        generally cannot be fixed in ``ArenaConfig``: 33 m^2 at the centre of the field is
        inside the room for every seed. Survey the arena first, then place -- see
        ``docs/01-competition.md``.
        """
        return (
            0.0 <= x <= self.width_m
            and 0.0 <= y <= self.depth_m
            and not self.in_start_area(x, y)
            and not self.in_unknown_area(x, y)
        )

    def targets_of(self, kind: str) -> tuple[Target, ...]:
        return tuple(t for t in self.targets if t.kind == kind)

    def obstacle_polygons(self) -> list[Polygon]:
        return [w.polygon() for w in self.walls] + [p.polygon() for p in self.pillars]

    def occupancy_grid(self, resolution_m: float, inflate_m: float = 0.0) -> np.ndarray:
        """Rasterise everything a drone cannot pass through to a boolean grid indexed
        ``[ix, iy]``. True means blocked: walls, pillars, and every solid landmark -- mission
        markers and placed bodies alike.

        Done analytically rather than with shapely per cell: a rotated rectangle is an
        axis-aligned box in its own frame, and a circle is a distance test. That turns a
        200 x 200 grid from tens of thousands of geometry calls into a handful of array ops.

        ``inflate_m`` dilates every obstacle, which is how the connectivity check asks
        "could a drone of this radius actually fit through here" rather than "is there a
        mathematical gap". Solid landmarks are included because a fence of posts across a
        doorway blocks it exactly as a wall would; an earlier version ignored them and
        validated an arena whose every doorway was plugged.
        """
        if resolution_m <= 0:
            raise ConfigError(f"resolution_m must be > 0, got {resolution_m}")
        nx = int(math.ceil(self.width_m / resolution_m))
        ny = int(math.ceil(self.depth_m / resolution_m))
        xs = (np.arange(nx) + 0.5) * resolution_m
        ys = (np.arange(ny) + 0.5) * resolution_m
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        grid = np.zeros((nx, ny), dtype=bool)

        for wall in self.walls:
            cx, cy = wall.centre
            ca, sa = math.cos(-wall.angle_rad), math.sin(-wall.angle_rad)
            dx, dy = gx - cx, gy - cy
            lx = dx * ca - dy * sa
            ly = dx * sa + dy * ca
            grid |= (np.abs(lx) <= wall.length_m / 2.0 + inflate_m) & (
                np.abs(ly) <= wall.thickness_m / 2.0 + inflate_m
            )

        for pillar in self.pillars:
            grid |= (gx - pillar.x) ** 2 + (gy - pillar.y) ** 2 <= (
                pillar.radius_m + inflate_m
            ) ** 2

        for lm in self.all_landmarks:
            if lm.solid:
                grid |= (gx - lm.x) ** 2 + (gy - lm.y) ** 2 <= (lm.radius_m + inflate_m) ** 2

        return grid


# ------------------------------------------------------------------------------------------
# Generation
# ------------------------------------------------------------------------------------------


def _far_enough(candidate: Polygon, existing: list[Polygon], min_gap: float) -> bool:
    """True when ``candidate`` keeps at least ``min_gap`` clear of everything in ``existing``.

    The prefilter is the candidate's bounding box grown by ``min_gap``, not
    ``candidate.buffer(min_gap)``. Shapely's buffer approximates the offset circle with a
    finite number of segments and so lies strictly *inside* the true offset -- for a pillar's
    24-gon it falls 0.6 mm short. That is enough to make the STRtree miss an obstacle that is
    genuinely within the gap: the placement filter then accepts it and ``_validate_gaps``, which
    measures exactly, rejects the finished arena. It cost about one seed in 300, as a hard
    generation failure rather than a degraded arena, and it presented as a 1.000 m gap being
    "below the minimum of 1.0 m" because the message rounds. A grown bounding box is a strict
    superset of everything within ``min_gap``, so the prefilter can only ever be conservative.
    """
    if not existing:
        return True
    tree = STRtree(existing)
    minx, miny, maxx, maxy = candidate.bounds
    probe = box(minx - min_gap, miny - min_gap, maxx + min_gap, maxy + min_gap)
    for j in tree.query(probe):
        if existing[j].distance(candidate) < min_gap:
            return False
    return True


def _boundary_walls(width: float, depth: float, thickness: float) -> list[Wall]:
    """The field boundary.

    West, north and east are the published 1.5 m perimeter wall. The south edge has no
    perimeter wall -- the rules say netting only -- but it still has to stop drones, because
    ir-sim has no implicit world bounds and a robot silently leaves the world otherwise
    (R-WORLD-6). It is modelled at the safety-net height and tagged ``net`` so that anything
    reasoning about walls can tell the difference.
    """
    half = thickness / 2.0
    return [
        Wall(-half, 0.0, -half, depth, thickness, PERIMETER_WALL_HEIGHT_M, "perimeter_wall"),
        Wall(0.0, depth + half, width, depth + half, thickness, PERIMETER_WALL_HEIGHT_M, "perimeter_wall"),
        Wall(width + half, 0.0, width + half, depth, thickness, PERIMETER_WALL_HEIGHT_M, "perimeter_wall"),
        Wall(0.0, -half, width, -half, thickness, SAFETY_NET_HEIGHT_M, "net"),
    ]


def _room_walls(
    x0: float, y0: float, size: float, thickness: float, rng: np.random.Generator,
    n_doorways: int, doorway_m: float, grid: MazeGrid | None = None,
) -> list[Wall]:
    """The Unknown Search Area: a walled room with open doorways.

    The section 3.2 diagram draws four openings, one roughly centred on each face, and rule
    3.3.9 r.1 says the swarm enters "via the open doorways shown in the diagram", so the default
    is four. Their widths measure 2.40-2.83 m off the diagram, but the diagram is explicitly not
    to scale, so count and width remain assumption A-7. The south face always gets one:
    otherwise the room can be unreachable from the direction every drone actually approaches.

    When a maze ``grid`` is supplied, each doorway is snapped to span exactly one whole cell.
    That is what makes a doorway un-blockable by construction: maze walls run along cell
    *boundaries*, so an opening that covers a cell's full clear width can never be bisected by
    one. Without snapping, a grid line lands inside the opening on about 42% of faces, silently
    halving a doorway that no existing validator would object to.
    """
    x1, y1 = x0 + size, y0 + size
    faces = [
        ((x0, y0), (x1, y0)),  # south -- always gets a doorway
        ((x1, y0), (x1, y1)),  # east
        ((x1, y1), (x0, y1)),  # north
        ((x0, y1), (x0, y0)),  # west
    ]
    chosen = {0}
    while len(chosen) < min(n_doorways, len(faces)):
        chosen.add(int(rng.integers(0, len(faces))))

    walls: list[Wall] = []
    for i, (a, b) in enumerate(faces):
        if i not in chosen:
            walls.append(Wall(a[0], a[1], b[0], b[1], thickness, INNER_WALL_HEIGHT_M, "unknown_wall"))
            continue
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        if grid is not None and grid.n >= 3:
            # Snap to a whole cell. The face runs from corner to corner over `size`, while the
            # grid tiles the clear interior starting thickness/2 in, so the cell's span is
            # offset by that half-thickness before being normalised onto the face.
            #
            # Interior cells only. An end cell leaves a corner stub of exactly thickness/2,
            # which the length guard below then drops -- so the doorway ran flush to the corner
            # at 2.45 m instead of 2.40 m, and when two adjacent faces both chose the cell at
            # the same corner the corner post vanished and the two doorways merged into one
            # L-shaped 4.9 m opening. That happened on 51 of 200 default seeds, including
            # seed 0. Excluding the end cells also matches A-7 as written: the section 3.2
            # diagram draws each doorway roughly centred on its face, not hard against a corner.
            # Interior cells only. An end cell leaves a corner stub of exactly thickness/2,
            # which the length guard below then drops, so the doorway ran flush to the corner
            # and two adjacent faces choosing the same corner deleted the post entirely.
            #
            # A 2x2 lattice has no interior cell -- both are end cells -- so it falls through
            # to the centred placement instead. An earlier cut kept snapping there via an
            # `else` branch that was the pre-fix code verbatim, which left 177 of 200 seeds
            # with an open corner at n=2. Snapping would be wrong there anyway: one cell is
            # 4.90 m, so a "one cell wide" doorway would be half the face.
            cell = int(rng.integers(1, grid.n - 1))
            lo_m, hi_m = grid.cell_span(cell)
            lo = (thickness / 2.0 + lo_m) / size
            hi = (thickness / 2.0 + hi_m) / size
        else:
            centre = float(rng.uniform(doorway_m / size, 1.0 - doorway_m / size))
            half = doorway_m / 2.0 / size
            lo, hi = centre - half, centre + half
        for slo, shi in ((0.0, max(0.0, lo)), (min(1.0, hi), 1.0)):
            if (shi - slo) * size < thickness:
                continue
            walls.append(
                Wall(
                    ax + dx * slo, ay + dy * slo, ax + dx * shi, ay + dy * shi,
                    thickness, INNER_WALL_HEIGHT_M, "unknown_wall",
                )
            )
    return walls


def generate_arena(
    seed: int,
    config: ArenaConfig | None = None,
    validate: bool = True,
    *,
    layout_seed: int | None = None,
    unknown_seed: int | None = None,
    mission_seed: int | None = None,
) -> ArenaSpec:
    """Build one arena from a seed. Deterministic: same seeds, same arena.

    Raises :class:`ArenaError` if a layout satisfying every published constraint could not be
    found, rather than returning a degraded arena. An arena that violates its own constraints
    would invalidate every run performed on it, so there is nothing useful to fall back to.

    Three independent streams, not one
    ----------------------------------
    ``seed`` alone reproduces a whole arena, as before. But an arena is three separable things,
    and holding one fixed while resampling another is how you attribute a result to the thing
    you actually varied:

    ``layout_seed``
        The Known Search Area, and the room's *position*. The room is a 2 m walled box in plain
        sight -- the team walks around it during setup -- so where it sits is public geometry
        that shapes the known area's corridor ring. Only its interior is unknown.
    ``unknown_seed``
        The maze inside the room and the pillars in it. Everything 3.2 calls "intentionally
        NOT shown".
    ``mission_seed``
        Victim, bonus-victim and fire placement. Undisclosed independently of either layout
        (3.3.3 r.1, 3.3.5 r.1).

    Each defaults to a child of ``seed`` via ``SeedSequence.spawn`` (R-DET-3), so omitting all
    three is exactly the old single-stream behaviour with a different, equally arbitrary
    mapping from seed to arena. Passing one pins that stream and leaves the others free::

        # ten known areas, one fixed unknown interior
        [generate_arena(0, layout_seed=k, unknown_seed=7) for k in range(10)]
        # one known area, twenty different mazes
        [generate_arena(0, layout_seed=3, unknown_seed=j) for j in range(20)]

    A fixed set of maps is a *development* set. Policies overfit to it, which is the failure
    the rulebook's withholding exists to punish, so quote final numbers from seeds you have
    never inspected.
    """
    cfg = config or ArenaConfig()
    base = np.random.SeedSequence(seed).spawn(3)
    rng_layout = np.random.default_rng(
        base[0] if layout_seed is None else np.random.SeedSequence(layout_seed)
    )
    rng_unknown = np.random.default_rng(
        base[1] if unknown_seed is None else np.random.SeedSequence(unknown_seed)
    )
    mission_ss = base[2] if mission_seed is None else np.random.SeedSequence(mission_seed)

    width, depth = FIELD_WIDTH_M, FIELD_DEPTH_M
    thickness = cfg.wall_thickness_m
    walls = _boundary_walls(width, depth, thickness)
    structure = [w.polygon() for w in walls]
    # A placed landmark with a footprint -- a post, a start mark on the floor -- is a fixed
    # feature of the venue, so the random layout is generated around it: walls keep the wall
    # gap from a body and pillars and the room the pillar gap, targets are not dropped onto
    # it, and the room is re-drawn if a wall would come too close. A flat mark just may not
    # be built over.
    bodies = [target_polygon(lm) for lm in cfg.landmarks if lm.solid]
    marks = [target_polygon(lm) for lm in cfg.landmarks if lm.radius_m > 0 and not lm.solid]
    structure.extend(bodies)

    def _over_a_mark(poly: Polygon) -> bool:
        return any(poly.intersects(mark) for mark in marks)

    # Gaps are measured between wall FACES, not centre lines. A wall centred on its line
    # extends thickness/2 either side, so a room placed with its centre line min_gap from the
    # perimeter leaves only min_gap - thickness/2 of actual air. Getting this wrong put the
    # room's north face 1.95 m from the perimeter in every seed while validation passed.
    half = thickness / 2.0
    # North, west and east must clear the perimeter by the published gap. The room's SOUTH
    # side faces the Start Area boundary, which is a virtual line rather than a wall
    # (rulebook 3.2), so no wall-to-wall gap applies there -- only that the room stays out of
    # the Start Area.
    y_lo = START_AREA_DEPTH_M + half
    y_hi = depth - cfg.min_gap_wall_m - UNKNOWN_AREA_SIZE_M - half
    x_lo = cfg.min_gap_wall_m + half
    x_hi = width - cfg.min_gap_wall_m - UNKNOWN_AREA_SIZE_M - half
    if y_hi < y_lo - 1e-9 or x_hi < x_lo - 1e-9:
        raise ArenaError(
            f"a {UNKNOWN_AREA_SIZE_M} m Unknown Search Area with {cfg.min_gap_wall_m} m "
            f"perimeter gaps and {thickness} m walls does not fit a "
            f"{width} x {depth} m field with a {START_AREA_DEPTH_M} m Start Area"
        )
    for attempt in range(cfg.max_placement_attempts):
        room_y0 = float(rng_layout.uniform(y_lo, max(y_hi, y_lo)))
        room_x0 = float(rng_layout.uniform(x_lo, max(x_hi, x_lo)))
        # The maze lattice depends on where the room landed, so it is planned per attempt and
        # handed to _room_walls, which snaps each doorway to a whole cell.
        grid = (
            plan_grid(room_x0, room_y0, UNKNOWN_AREA_SIZE_M, thickness, cfg.maze_corridor_m)
            if cfg.maze_corridor_m is not None
            else None
        )
        room = _room_walls(
            room_x0, room_y0, UNKNOWN_AREA_SIZE_M, thickness, rng_layout,
            UNKNOWN_AREA_DOORWAYS, UNKNOWN_AREA_DOORWAY_M, grid,
        )
        polys = [w.polygon() for w in room]
        if all(_far_enough(p, bodies, cfg.min_gap_pillar_m) for p in polys) and not any(
            p.intersects(mark) for p in polys for mark in marks
        ):
            break
    else:
        raise ArenaError(
            f"could not place the Unknown Search Area clear of the placed landmarks in "
            f"{cfg.max_placement_attempts} attempts"
        )
    walls.extend(room)
    # Room walls are one structure; they meet at corners, so they are added together without
    # mutual gap checks. Everything placed after this must clear them.
    structure.extend(w.polygon() for w in room)

    unknown_area = (room_x0, room_y0, room_x0 + UNKNOWN_AREA_SIZE_M, room_y0 + UNKNOWN_AREA_SIZE_M)

    if grid is not None:
        maze = generate_maze(
            rng_unknown, grid,
            height_m=INNER_WALL_HEIGHT_M,
            kind="maze_wall",
            n_loops=cfg.n_maze_loops,
            anchor_gap_m=cfg.maze_anchor_gap_m,
            wall_factory=Wall,
        )
        walls.extend(maze)
        # Maze walls join the room and each other by design, so like the room they are one
        # structure and are appended without mutual gap checks. They still constrain everything
        # placed afterwards, which is what keeps pillars and targets out of the wall faces.
        structure.extend(w.polygon() for w in maze)

    def _sample_wall(bounds, length_range, rng) -> Wall | None:
        bx0, by0, bx1, by1 = bounds
        length = float(rng.uniform(*length_range))
        angle = float(rng.choice([0.0, math.pi / 2.0])) if rng.random() < 0.75 else float(
            rng.uniform(0, math.pi)
        )
        cx = float(rng.uniform(bx0, bx1))
        cy = float(rng.uniform(by0, by1))
        half = length / 2.0
        return Wall(
            cx - half * math.cos(angle), cy - half * math.sin(angle),
            cx + half * math.cos(angle), cy + half * math.sin(angle),
            thickness, INNER_WALL_HEIGHT_M, "inner_wall",
        )

    def _place_walls(n: int, bounds, forbid_room: bool, rng) -> None:
        placed = 0
        for _ in range(cfg.max_placement_attempts):
            if placed >= n:
                return
            wall = _sample_wall(bounds, cfg.inner_wall_length_range_m, rng)
            poly = wall.polygon()
            bx0, by0, bx1, by1 = poly.bounds
            if bx0 < 0 or by0 < 0 or bx1 > width or by1 > depth:
                continue
            if forbid_room and box(*unknown_area).intersects(poly.buffer(cfg.min_gap_wall_m)):
                continue
            if not _far_enough(poly, structure, cfg.min_gap_wall_m) or _over_a_mark(poly):
                continue
            walls.append(wall)
            structure.append(poly)
            placed += 1
        if placed < n:
            raise ArenaError(
                f"could only place {placed} of {n} walls in {bounds} after "
                f"{cfg.max_placement_attempts} attempts with a {cfg.min_gap_wall_m} m "
                f"minimum gap. Reduce n_inner_walls or shorten inner_wall_length_range_m."
            )

    known_bounds = (1.0, START_AREA_DEPTH_M + 1.0, width - 1.0, depth - 1.0)
    _place_walls(cfg.n_inner_walls, known_bounds, forbid_room=True, rng=rng_layout)
    inset = thickness + 0.5
    _place_walls(
        cfg.n_unknown_walls,
        (unknown_area[0] + inset, unknown_area[1] + inset, unknown_area[2] - inset, unknown_area[3] - inset),
        forbid_room=False,
        rng=rng_unknown,
    )

    pillars: list[Pillar] = []

    def _place_pillars(n: int, bounds, rng, forbid_room: bool = False) -> None:
        bx0, by0, bx1, by1 = bounds
        placed = 0
        for _ in range(cfg.max_placement_attempts):
            if placed >= n:
                return
            pillar = Pillar(float(rng.uniform(bx0, bx1)), float(rng.uniform(by0, by1)))
            poly = pillar.polygon()
            if forbid_room and box(*unknown_area).intersects(poly):
                continue
            if not _far_enough(poly, structure, cfg.min_gap_pillar_m) or _over_a_mark(poly):
                continue
            pillars.append(pillar)
            structure.append(poly)
            placed += 1
        if placed < n:
            raise ArenaError(
                f"could only place {placed} of {n} pillars in {bounds} after "
                f"{cfg.max_placement_attempts} attempts with a {cfg.min_gap_pillar_m} m gap"
            )

    # forbid_room mirrors the guard _place_walls already had. Without it the "known area"
    # bounds span the whole field north of the Start Area, room included, and a pillar only had
    # to clear the room *walls* by 1 m -- which the room's 9.9 m interior leaves ample space to
    # do. The result was that 94% of seeds leaked known-area pillars into the Unknown Search
    # Area, 2.27 of them on average, so the room held ~4.3 pillars against a configured 2 and
    # the two areas' obstacle densities were both wrong.
    _place_pillars(cfg.n_pillars_known, known_bounds, rng_layout, forbid_room=True)
    if grid is not None:
        # In a 2.40 m corridor a pillar's centre must sit within a 0.75 m band on the
        # centreline to clear both walls by the published 1 m, so uniform rejection sampling
        # over the room wastes most of its attempts. Cell centres are the exact points that
        # maximise clearance, so they are drawn without replacement instead.
        _place_pillars_at_cells(cfg.n_pillars_unknown, grid, rng_unknown, pillars, structure, cfg)
    else:
        _place_pillars(
            cfg.n_pillars_unknown,
            (unknown_area[0] + inset, unknown_area[1] + inset, unknown_area[2] - inset, unknown_area[3] - inset),
            rng_unknown,
        )

    targets = _place_targets(mission_ss, cfg, structure + marks, unknown_area, width, depth)

    spec = ArenaSpec(
        seed=seed,
        layout_seed=layout_seed,
        unknown_seed=unknown_seed,
        mission_seed=mission_seed,
        width_m=width,
        depth_m=depth,
        ceiling_m=CEILING_M,
        start_area_depth_m=START_AREA_DEPTH_M,
        unknown_area=unknown_area,
        walls=tuple(walls),
        pillars=tuple(pillars),
        targets=tuple(targets),
        landmarks=tuple(cfg.landmarks),
        config=cfg,
    )
    if validate:
        validate_arena(spec)
    return spec


def _place_pillars_at_cells(
    n: int, grid: MazeGrid, rng, pillars: list[Pillar], structure: list[Polygon], cfg
) -> None:
    """Put the room's pillars on maze cell centres, drawn without replacement.

    Raises :class:`ArenaError` rather than quietly placing fewer than ``n``: a room with no
    pillars would violate 3.3.9 r.2, and one with fewer than configured would silently change
    the obstacle density every result depends on.
    """
    if n <= 0:
        return
    radius = PILLAR_DIAMETER_M / 2.0
    if grid.free_radius_m < cfg.min_gap_pillar_m + radius:
        raise ArenaError(
            f"a pillar needs {cfg.min_gap_pillar_m + radius:.2f} m of clearance from a cell "
            f"centre but a {grid.corridor_w_m:.2f} m corridor offers {grid.free_radius_m:.2f} m. "
            f"Raise maze_corridor_m or set n_pillars_unknown to 0."
        )
    cells = grid.cell_centres()
    placed = 0
    for k in rng.permutation(len(cells)):
        if placed >= n:
            return
        cx, cy = cells[int(k)]
        pillar = Pillar(cx, cy)
        poly = pillar.polygon()
        if not _far_enough(poly, structure, cfg.min_gap_pillar_m):
            continue
        pillars.append(pillar)
        structure.append(poly)
        placed += 1
    if placed < n:
        raise ArenaError(
            f"could only place {placed} of {n} pillars on the {grid.n}x{grid.n} maze cell "
            f"centres. Reduce n_pillars_unknown or raise maze_corridor_m."
        )


def _place_targets(mission_ss, cfg, structure, unknown_area, width, depth) -> list[Target]:
    """Place markers, honouring the rulebook's guarantees about where they can be.

    Section 3.3.9 r.2 guarantees the Unknown Search Area contains bonus victim(s) and fire(s),
    and 3.3.3 notes bonus victims "are likely to be placed in regions that are harder to
    rescue" -- so one of each is forced into the room and the rest are sampled from the whole
    Known Search Area. Nothing is placed in the Start Area.

    **One RNG substream per marker**, spawned from the mission seed rather than drawn from a
    single stream. Placement is rejection sampling, so the number of draws a marker consumes
    depends on what it had to avoid -- and the first two markers are forced into the room,
    where what they must avoid is the maze. On one shared stream that made every later marker's
    position a function of ``unknown_seed``: with the Known Search Area bit-identical, changing
    only the maze moved 7 to 12 of the 12 markers and swung the number of markers outside the
    room from 6 to 9. That is up to three markers, and up to 45 points of achievable score,
    migrating between zones because the maze changed -- silently confounding the exact
    experiment R-WORLD-9 exists to make clean. Per-marker streams cost nothing and make each
    marker depend only on its own draws and on the geometry it is actually tested against.
    """
    ux0, uy0, ux1, uy1 = unknown_area
    inset = 0.6
    room_bounds = (ux0 + inset, uy0 + inset, ux1 - inset, uy1 - inset)
    known_bounds = (0.6, START_AREA_DEPTH_M + 0.6, width - 0.6, depth - 0.6)
    # A marker must leave room for a drone to land beside it, in line of sight.
    clearance = DRONE_RADIUS_M + MARKER_FOOTPRINT_M / 2.0 + 0.15

    targets: list[Target] = []
    occupied: list[Polygon] = list(structure)

    # Spawned in a fixed order, so adding a marker kind cannot renumber an existing one.
    n_slots = 2 + max(0, cfg.n_bonus_victims - 1) + max(0, cfg.n_fires - 1) + cfg.n_victims
    streams = iter(np.random.default_rng(c) for c in mission_ss.spawn(n_slots))

    room_box = box(ux0, uy0, ux1, uy1)

    def _place(kind: str, index: int, bounds, forbid_room: bool = False) -> None:
        rng = next(streams)
        bx0, by0, bx1, by1 = bounds
        for _ in range(cfg.max_placement_attempts):
            x = float(rng.uniform(bx0, bx1))
            y = float(rng.uniform(by0, by1))
            target = Target(id=f"{kind}_{index}", kind=kind, x=x, y=y)
            poly = target_polygon(target)
            if forbid_room and room_box.intersects(poly):
                continue
            if not _far_enough(poly, occupied, clearance):
                continue
            targets.append(target)
            occupied.append(poly)
            return
        raise ArenaError(
            f"could not place {kind} #{index} after {cfg.max_placement_attempts} attempts; "
            f"the arena is too cluttered for the requested target count"
        )

    # Forced into the Unknown Search Area by rule 3.3.9 r.2.
    _place("bonus_victim", 0, room_bounds)
    _place("fire", 0, room_bounds)
    # The rest are Known Search Area markers, and `known_bounds` spans the whole field north of
    # the Start Area -- the room included. Without this guard 191 of 200 seeds dropped markers
    # drawn for the known area inside the room instead, up to five in one seed, so the room held
    # a seed-dependent 2 to 7 markers rather than the two 3.3.9 r.2 requires and neither zone's
    # reward density was controllable. It is the same guard _place_walls and _place_pillars use.
    #
    # The rulebook does not forbid a marker in the room -- 3.3.3 r.1 excludes no zone. What it
    # does is refuse to tell you, so the honest model is a distribution you control rather than
    # one that drifts with the maze. Raise n_bonus_victims / n_fires to put more in the room.
    for i in range(1, cfg.n_bonus_victims):
        _place("bonus_victim", i, known_bounds, forbid_room=True)
    for i in range(1, cfg.n_fires):
        _place("fire", i, known_bounds, forbid_room=True)
    for i in range(cfg.n_victims):
        _place("victim", i, known_bounds, forbid_room=True)
    return targets


def landmark_shape(lm: Landmark):
    """What a landmark occupies on the floor: its footprint, or a bare point if it has none.

    ``target_polygon`` collapses a zero-radius landmark to a degenerate 16-gon whose ``area``
    and ``intersects`` are unreliable, which is how a surveyed tag with no footprint -- the
    commonest nav aid there is -- slipped past every zone check.
    """
    return target_polygon(lm) if lm.radius_m > 0.0 else Point(lm.x, lm.y)


def target_polygon(target: Landmark) -> Polygon:
    """The footprint of a solid landmark, for placement and validation."""
    return Polygon(
        [
            (target.x + target.radius_m * math.cos(t), target.y + target.radius_m * math.sin(t))
            for t in np.linspace(0, 2 * math.pi, 16, endpoint=False)
        ]
    )


# ------------------------------------------------------------------------------------------
# Validation
# ------------------------------------------------------------------------------------------


def validate_arena(spec: ArenaSpec, drone_radius_m: float = DRONE_RADIUS_M) -> None:
    """Assert every published constraint. Raises :class:`ArenaError` on the first violation.

    Checks, in order of how badly a violation would corrupt results:

    1. Every landmark is inside the field, has a unique id, and is not a decoy target
       (R-WORLD-7).
    2. Every landmark with a footprint -- every target, every placed body or mark -- is
       outside every obstacle and clear of every other footprint.
    3. Free space is connected from the Start Area to a landing spot for every target, with
       solid landmarks counted as blocking -- an unreachable target silently caps the
       achievable score and would look like a policy failure.
    4. Published minimum gaps hold between independently placed obstacles.
    5. No pocket of the Unknown Search Area is sealed off. The maze's walls are the complement
       of a spanning tree, so connectivity holds by construction -- but a structural argument
       nothing tests is a comment, and this is the one that would fail silently if the doorway
       snapping in ``_room_walls`` ever regressed.
    """
    _validate_landmarks(spec)
    _validate_footprints_clear(spec)
    _validate_landmark_zones(spec)
    _validate_reachability(spec, drone_radius_m)
    _validate_gaps(spec)
    _validate_room_connectivity(spec, drone_radius_m)


def validate_nav_aids(spec: ArenaSpec, kinds: Iterable[str]) -> None:
    """The navigation-aid rules that :func:`validate_arena` deliberately does not enforce.

    Two of the rulebook's three aid rules need no help. r.17 -- nothing in the Unknown Search
    Area, which teams may never enter -- is enforced for every placed landmark by
    :func:`_validate_landmark_zones` on every run, because it is the highest-value cheat the
    rules forbid. r.16 puts no limit on the Start Area, so there is nothing to check.

    What is left is the pair that cannot be checked without knowing which landmarks are
    *navigation aids*: r.15's cap of ten in the Known Search Area, and r.14 f's 1 m x 1 m
    footprint. A :class:`~safmc_sim.world.landmark.Landmark` may equally be scenery, a prop or
    a venue feature, and the primitive carries no field that says which -- so ``kinds`` says,
    and the caller is the one who knows. Every kind listed is counted together, which is what
    the rule does: ten aids in the Known Search Area means ten, whether they are tags or
    anchors or both.

    Opt-in, and called by whoever builds the scenario, never by the runner: an experiment
    that wants twenty anchors in order to ask what dense coverage would be worth is a
    legitimate experiment, and this exists so that nobody quotes one without knowing that is
    what it was. ``examples/04_uwb_ranging.py`` calls it on its anchors.

    Raises:
        ArenaError: naming the offending landmarks and the rule they break.
    """
    if isinstance(kinds, str):
        # set("uwb_anchor") is a set of letters, and then nothing is an aid and every layout
        # passes -- the most natural wrong call approved anything. An auditor made it.
        raise ArenaError(
            f"kinds must be a sequence of landmark kinds, got the string {kinds!r}; "
            f"write ({kinds!r},)"
        )
    wanted = set(kinds)
    if not wanted:
        raise ArenaError("kinds is empty, so nothing would count as a navigation aid")
    aids = [lm for lm in spec.all_landmarks if lm.kind in wanted]

    too_wide = [lm.id for lm in aids if 2.0 * lm.radius_m > NAV_AID_FOOTPRINT_M]
    if too_wide:
        raise ArenaError(
            f"navigation aid(s) {too_wide} exceed the {NAV_AID_FOOTPRINT_M:g} m x "
            f"{NAV_AID_FOOTPRINT_M:g} m footprint the rules allow (sec 3.3.1 r.14 f)"
        )

    in_known = [lm.id for lm in aids if spec.in_known_area(lm.x, lm.y)]
    if len(in_known) > NAV_AID_MAX_KNOWN_AREA:
        raise ArenaError(
            f"{len(in_known)} navigation aids in the Known Search Area, where the rules "
            f"allow at most {NAV_AID_MAX_KNOWN_AREA} (sec 3.3.1 r.15): {in_known}. The "
            f"Start Area takes any number (r.16)."
        )


def _validate_landmarks(spec: ArenaSpec) -> None:
    seen: set[str] = set()
    for lm in spec.all_landmarks:
        # Re-checked here, not trusted from construction: a subclass that skipped
        # super().__post_init__() would otherwise put a NaN or an empty kind in the log.
        validate_landmark_fields(lm, ArenaError)
        if isinstance(lm, Target) and lm.kind not in TARGET_KINDS:
            raise ArenaError(f"target {lm.id!r} has unknown kind {lm.kind!r}")
        if lm.id in seen:
            raise ArenaError(f"duplicate landmark id {lm.id!r}")
        seen.add(lm.id)
        if not (0.0 <= lm.x <= spec.width_m and 0.0 <= lm.y <= spec.depth_m):
            raise ArenaError(
                f"landmark {lm.id} at ({lm.x:.2f}, {lm.y:.2f}) is outside the "
                f"{spec.width_m:g} x {spec.depth_m:g} m field"
            )
    # The ArenaConfig check covers the config path; this covers an ArenaSpec assembled by
    # hand or by dataclasses.replace, which never went through a config.
    for lm in spec.landmarks:
        _reject_decoy(lm, ArenaError)


def _validate_landmark_zones(spec: ArenaSpec) -> None:
    """Where a team is allowed to put things, per rulebook 3.3.1 r.15-17.

    Unlimited in the Start Area, at most ten in the Known Search Area, and **none at all** in
    the Unknown Search Area -- teams are "NOT allowed to enter the Unknown Search Area at all
    times", so they cannot place anything there.

    Only the zone rule is enforced, and only against ``spec.landmarks``: generated mission
    markers live in ``spec.targets``, and 3.3.9 r.2 puts bonus victims and fires *inside* the
    room by design. Placing a tag in there is the highest-value cheat the rules forbid -- a free
    localisation anchor in the one region where localisation is meant to be hard -- so it is
    checked rather than documented.

    The companion rule, at most ten aids in the Known Search Area
    (:data:`~safmc_sim.constants.NAV_AID_MAX_KNOWN_AREA`), is **not** enforced here. It governs
    *navigation aids*, and a ``Landmark`` may equally be scenery, a prop or a venue feature;
    the primitive carries no field distinguishing them, so a blanket cap would reject
    legitimate arenas. Assert it in your own experiment, or give ``Landmark`` a nav-aid flag
    first.
    """
    for lm in spec.landmarks:
        if spec.in_unknown_area(lm.x, lm.y):
            raise ArenaError(
                f"landmark {lm.id!r} at ({lm.x:.2f}, {lm.y:.2f}) is inside the Unknown Search "
                f"Area {tuple(round(v, 2) for v in spec.unknown_area)}. Rulebook 3.3.1 r.17: "
                f"teams may never enter it, so nothing they place can be in it. Nav aids "
                f"generally cannot be fixed in ArenaConfig -- 33 m^2 at the centre of the "
                f"field is inside the room for every seed -- so survey the generated arena "
                f"first: dataclasses.replace(arena, landmarks=...) with in_known_area()."
            )


def _validate_footprints_clear(spec: ArenaSpec) -> None:
    placed = [lm for lm in spec.all_landmarks if lm.radius_m > 0.0]
    polys = [target_polygon(lm) for lm in placed]
    obstacles = spec.obstacle_polygons()
    if obstacles:
        tree = STRtree(obstacles)
        for lm, poly in zip(placed, polys):
            for j in tree.query(poly):
                if obstacles[j].intersects(poly):
                    raise ArenaError(
                        f"landmark {lm.id} at ({lm.x:.2f}, {lm.y:.2f}) overlaps an obstacle"
                    )
    for i, (a, pa) in enumerate(zip(placed, polys)):
        for b, pb in zip(placed[i + 1:], polys[i + 1:]):
            if pa.intersects(pb):
                raise ArenaError(f"landmarks {a.id} and {b.id} overlap")


def _validate_reachability(spec: ArenaSpec, drone_radius_m: float) -> None:
    """Flood fill the drone-inflated free space from the Start Area."""
    res = _CONNECTIVITY_RES_M
    blocked = spec.occupancy_grid(res, inflate_m=drone_radius_m)
    nx, ny = blocked.shape
    reachable = _flood_from_start(spec, blocked, res)

    for target in spec.targets:
        cx, cy = int(target.x / res), int(target.y / res)
        radius_cells = int(math.ceil(SCORE_RADIUS_M / res))
        window = reachable[
            max(0, cx - radius_cells) : min(nx, cx + radius_cells + 1),
            max(0, cy - radius_cells) : min(ny, cy + radius_cells + 1),
        ]
        if not window.any():
            raise ArenaError(
                f"target {target.id} at ({target.x:.2f}, {target.y:.2f}) has no reachable "
                f"landing spot within {SCORE_RADIUS_M} m -- it is walled off from the Start Area"
            )


def _validate_room_connectivity(spec: ArenaSpec, drone_radius_m: float) -> None:
    """Every free cell inside the Unknown Search Area must be reachable from the Start Area.

    ``_validate_reachability`` only asks whether each *target* has a reachable landing spot, so
    it cannot see a sealed pocket that happens to contain nothing. That distinction matters
    once the room has interior walls: an unreachable pocket is dead area a search policy will
    spend the whole run trying to enter, and it would look like a policy failure rather than a
    broken arena.
    """
    res = _CONNECTIVITY_RES_M
    blocked = spec.occupancy_grid(res, inflate_m=drone_radius_m)
    reachable = _flood_from_start(spec, blocked, res)

    x0, y0, x1, y1 = spec.unknown_area
    nx, ny = blocked.shape
    ix0, ix1 = max(0, int(x0 / res)), min(nx, int(math.ceil(x1 / res)))
    iy0, iy1 = max(0, int(y0 / res)), min(ny, int(math.ceil(y1 / res)))

    free = ~blocked[ix0:ix1, iy0:iy1]
    orphans = int(np.count_nonzero(free & ~reachable[ix0:ix1, iy0:iy1]))
    if orphans:
        raise ArenaError(
            f"{orphans} free cells inside the Unknown Search Area ({orphans * res * res:.2f} "
            f"m^2) are walled off from the Start Area. A sealed pocket makes part of the room "
            f"unsearchable and would be misread as a policy failure."
        )


def _flood_from_start(spec: ArenaSpec, blocked: np.ndarray, res: float) -> np.ndarray:
    """Breadth-first flood of the drone-inflated free space, seeded from the Start Area."""
    nx, ny = blocked.shape
    reachable = np.zeros_like(blocked)
    queue: deque[tuple[int, int]] = deque()
    start_rows = int(spec.start_area_depth_m / res)
    for ix in range(nx):
        for iy in range(min(start_rows, ny)):
            if not blocked[ix, iy] and not reachable[ix, iy]:
                reachable[ix, iy] = True
                queue.append((ix, iy))
    if not queue:
        raise ArenaError("the Start Area is entirely blocked")
    while queue:
        ix, iy = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            jx, jy = ix + dx, iy + dy
            if 0 <= jx < nx and 0 <= jy < ny and not blocked[jx, jy] and not reachable[jx, jy]:
                reachable[jx, jy] = True
                queue.append((jx, jy))
    return reachable


def _validate_gaps(spec: ArenaSpec) -> None:
    """Minimum gaps, checked only between obstacles that were placed independently.

    Walls belonging to one structure -- the field boundary, or the Unknown Search Area room --
    meet at corners by design, so their mutual distance is zero and the rule does not apply to
    them. The rule is about navigable gaps between separate obstacles.
    """
    min_gap_wall = spec.config.min_gap_wall_m
    min_gap_pillar = spec.config.min_gap_pillar_m
    groups: dict[str, list[Polygon]] = {}
    for wall in spec.walls:
        groups.setdefault(wall.kind, []).append(wall.polygon())

    independent = [(p, "inner_wall") for p in groups.get("inner_wall", [])]
    structural = [
        (p, kind)
        for kind in ("perimeter_wall", "unknown_wall", "maze_wall", "net")
        for p in groups.get(kind, [])
    ]
    for i, (poly, _) in enumerate(independent):
        for other, kind in independent[i + 1 :] + structural:
            gap = poly.distance(other)
            if gap < min_gap_wall - 1e-6:
                raise ArenaError(
                    f"inner wall to {kind} gap {gap:.3f} m is below the published minimum of "
                    f"{min_gap_wall} m"
                )

    # The Unknown Search Area room against the perimeter. These are two independently placed
    # structures, so the gap rule applies between them -- and it was previously unchecked,
    # which is how a systematic 1.95 m north gap survived in every seed. The room's south face
    # is exempt: it looks at the Start Area boundary, which is a virtual line, not a wall.
    # The room's shell only. Maze walls are a separate kind precisely so this check stays
    # about the two independently placed structures it was written for; an interior maze wall
    # is trivially clear of the perimeter and testing it here proved nothing.
    room = groups.get("unknown_wall", [])
    perimeter = groups.get("perimeter_wall", [])
    for room_poly in room:
        for wall_poly in perimeter:
            gap = room_poly.distance(wall_poly)
            if gap < min_gap_wall - 1e-6:
                raise ArenaError(
                    f"Unknown Search Area to perimeter gap {gap:.3f} m is below the published "
                    f"minimum of {min_gap_wall} m"
                )

    pillar_polys = [p.polygon() for p in spec.pillars]
    all_walls = [p for polys in groups.values() for p in polys]
    for i, poly in enumerate(pillar_polys):
        for other in pillar_polys[i + 1 :] + all_walls:
            gap = poly.distance(other)
            if gap < min_gap_pillar - 1e-6:
                raise ArenaError(
                    f"pillar gap {gap:.3f} m is below the published minimum of "
                    f"{min_gap_pillar} m"
                )
