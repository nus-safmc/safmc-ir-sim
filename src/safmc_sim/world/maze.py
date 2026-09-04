"""Seeded maze generation for the Unknown Search Area.

Why this module exists
----------------------
Section 3.2 of the 2026 Category Swarm Challenge Booklet is deliberately asymmetric about the
two search areas, and the asymmetry is load-bearing:

    "The placements of the inner walls will follow the diagram, but the exact positions and
     dimensions will NOT be given."          <- Known Search Area: topology pinned, metrics free
    "The layout within the Unknown Search Area is intentionally NOT shown."
                                             <- Unknown Search Area: nothing pinned at all

So inside the room a simulator has no layout to reproduce; it has a *distribution* to sample.
Rule 3.3.9 r.2 is the only constraint on contents -- the room "will contain Bonus Victim(s),
wall(s), pillar obstacle(s) and fire(s)" -- and rule 3.3.9 r.1 says the swarm enters "via the
open doorways shown in the diagram in Section 3.2".

Before this module the room was filled by the same rejection sampler as the Known Search Area:
two free-standing walls and two pillars, each forced 2 m from everything else. That is sparse
scatter, not a maze. A drone can fly around every baffle, so the room stayed topologically an
open box and search in it was never harder than search outside it.

The grid is not a taste choice
------------------------------
The room's clear interior span is ``UNKNOWN_AREA_SIZE_M - thickness`` = 9.90 m. Fitting ``n``
corridors of clear width ``w`` separated by ``n - 1`` walls of thickness ``t`` requires
``n * w + (n - 1) * t = 9.90``, so:

    n = 2  ->  w = 4.900 m   <- one wall only: (n-1)^2, so the braid must not remove it
    n = 3  ->  w = 3.233 m
    n = 4  ->  w = 2.400 m   <- the densest grid that still clears the published 2 m gap
    n = 5  ->  w = 1.900 m   <- below the published minimum

The published >= 2 m wall-to-wall gap therefore caps the maze at **4 x 4 cells with 2.40 m
corridors**. That is a happy accident: 2.40 m also sits inside the 2.40-2.83 m doorway widths
measured off the section 3.2 diagram, so one cell is exactly one doorway.

How the 2 m gap is read, and why it matters
-------------------------------------------
``arena.py`` reads the 2 m gap as an all-pairs exclusion: every wall must clear every other
wall by 2 m. Under that reading a maze is impossible -- at most three free-floating baffles fit,
each with both ends open -- and, more damningly, the rulebook's own diagram violates its own
table, because five of the six wall structures drawn in the Known Search Area are anchored at
zero gap to a perimeter wall or to the room.

This module adopts the reading that reconciles the table with the diagram: **2 m is a floor on
the navigable gap a drone flies through, and walls may anchor to each other and to the room.**
That is what makes dead ends and forced backtracking possible.

Both readings stay reachable, and through a metric knob rather than a string enum (which would
fail ``test_every_config_field_declares_its_units``). ``ArenaConfig.maze_anchor_gap_m`` is the
clearance retracted from **both ends** of every maze run: ``0.0`` leaves them anchored, and
setting it to ``min_gap_wall_m`` pulls every run clear of the room *and* of the other runs it
met, degenerating to the all-pairs reading -- measured, 52 touching wall pairs at ``0.0`` and 0
at ``2.0``. An earlier cut retracted only the ends touching a room face, which left every
interior T- and L-junction at exactly zero distance and so changed nothing about the property it
claimed to control. Any result quoted from this simulator should say which value it used.

Connectivity is structural, not checked-and-hoped
-------------------------------------------------
The walls are the complement of a spanning tree over the cells, so every cell reaches every
other cell by construction. ``arena._room_walls`` snaps each doorway to a whole *interior* cell,
so a doorway opens into a cell rather than being bisected by a wall end, and never runs flush to
a corner. Together those facts mean a generated maze cannot seal a doorway or strand a target.
``validate_arena`` still flood fills and asserts it, because a structural argument that is never
tested is a comment -- and the test that should have guarded the snapping asserted only that the
room had *some* free space, which is true of an empty box.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..errors import ArenaError

__all__ = ["MazeGrid", "plan_grid", "generate_maze"]


@dataclass(frozen=True)
class MazeGrid:
    """The cell lattice a maze is cut from.

    Consumers place things at cell centres: a cell centre is the point furthest from every wall
    that could bound the cell, which is the only placement that survives rejection sampling in a
    2.40 m corridor without thousands of retries.
    """

    n: int
    """Cells per axis."""

    corridor_w_m: float
    """Clear corridor width. Always >= the requested ``corridor_m``: slack is spread, not left
    at one edge."""

    pitch_m: float
    """Cell centre to cell centre, ``corridor_w_m + thickness_m``."""

    x0_m: float
    """South-west corner of the room's clear interior box (inside the room walls)."""

    y0_m: float

    thickness_m: float

    @property
    def span_m(self) -> float:
        """The clear interior span the grid tiles, on either axis."""
        return self.n * self.corridor_w_m + (self.n - 1) * self.thickness_m

    @property
    def free_radius_m(self) -> float:
        """Clearance from a cell centre to the nearest possible wall face."""
        return self.corridor_w_m / 2.0

    def cell_origin(self, i: int, j: int) -> tuple[float, float]:
        """South-west corner of cell ``(i, j)``."""
        return (self.x0_m + i * self.pitch_m, self.y0_m + j * self.pitch_m)

    def cell_centre(self, i: int, j: int) -> tuple[float, float]:
        ox, oy = self.cell_origin(i, j)
        half = self.corridor_w_m / 2.0
        return (ox + half, oy + half)

    def cell_centres(self) -> list[tuple[float, float]]:
        return [self.cell_centre(i, j) for i in range(self.n) for j in range(self.n)]

    def cell_span(self, k: int) -> tuple[float, float]:
        """The clear extent of cell index ``k`` along one axis, as an offset from that axis'
        origin. Used to snap a doorway to a whole cell."""
        lo = k * self.pitch_m
        return (lo, lo + self.corridor_w_m)


def plan_grid(
    room_x0: float,
    room_y0: float,
    size_m: float,
    thickness_m: float,
    corridor_m: float,
) -> MazeGrid:
    """Choose the densest cell grid whose corridors are at least ``corridor_m`` wide.

    Raises :class:`ArenaError` rather than returning a degenerate one-cell grid: a "maze" with
    no internal walls is an empty room wearing the wrong name, and silently returning one would
    make every result quoted from it wrong in a way nothing downstream could detect.
    """
    if corridor_m <= 0.0:
        raise ArenaError(f"maze_corridor_m must be positive, got {corridor_m}")
    span = size_m - thickness_m
    n = int(math.floor((span + thickness_m) / (corridor_m + thickness_m)))
    if n < 2:
        raise ArenaError(
            f"a {corridor_m} m corridor does not fit twice in the {span:.2f} m clear interior "
            f"of a {size_m} m Unknown Search Area. Reduce maze_corridor_m."
        )
    width = (span - (n - 1) * thickness_m) / n
    return MazeGrid(
        n=n,
        corridor_w_m=width,
        pitch_m=width + thickness_m,
        x0_m=room_x0 + thickness_m / 2.0,
        y0_m=room_y0 + thickness_m / 2.0,
        thickness_m=thickness_m,
    )


def _spanning_tree_passages(rng: np.random.Generator, n: int) -> set[frozenset[tuple[int, int]]]:
    """Randomised depth-first search over the cells; returns the edges it carves through.

    Depth-first rather than Kruskal or Prim on purpose. DFS produces long winding corridors with
    genuine dead ends -- the case that punishes a greedy frontier policy. Kruskal and Prim give
    bushier, junction-heavy mazes that are markedly easier to cover, so they would flatter a
    search policy rather than test it.
    """
    seen = np.zeros((n, n), dtype=bool)
    passages: set[frozenset[tuple[int, int]]] = set()
    stack: list[tuple[int, int]] = [(0, 0)]
    seen[0, 0] = True
    while stack:
        i, j = stack[-1]
        options = [
            (a, b)
            for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1))
            if 0 <= a < n and 0 <= b < n and not seen[a, b]
        ]
        if not options:
            stack.pop()
            continue
        a, b = options[int(rng.integers(0, len(options)))]
        passages.add(frozenset({(i, j), (a, b)}))
        seen[a, b] = True
        stack.append((a, b))
    return passages


def _braid(
    rng: np.random.Generator,
    n: int,
    passages: set[frozenset[tuple[int, int]]],
    n_loops: int,
) -> None:
    """Knock out ``n_loops`` further walls, in place, creating cycles.

    A pure spanning tree has exactly one route between any two cells, which makes the 3.3.7
    relay chain a single fragile path and turns any collision into a total blockage. Real
    buildings have loops; so should this.
    """
    if n_loops < 0:
        raise ArenaError(f"n_maze_loops must be >= 0, got {n_loops}")
    if n_loops == 0:
        return
    standing = [
        frozenset({(i, j), (a, b)})
        for i in range(n)
        for j in range(n)
        for a, b in ((i + 1, j), (i, j + 1))
        if a < n and b < n and frozenset({(i, j), (a, b)}) not in passages
    ]
    # Leave at least one wall standing. An n x n lattice has only (n-1)^2 walls after the
    # spanning tree, which is exactly ONE at n = 2 -- so the default n_maze_loops of 2 emptied
    # the room completely and validate_arena passed, because it checks connectivity rather than
    # the existence of walls. That is the empty room plan_grid raises to prevent at n < 2, and
    # 3.3.9 r.2 guarantees the Unknown Search Area contains wall(s).
    budget = min(n_loops, max(0, len(standing) - 1))
    if budget <= 0:
        return
    for k in rng.permutation(len(standing))[:budget]:
        passages.add(standing[int(k)])


def generate_maze(
    rng: np.random.Generator,
    grid: MazeGrid,
    *,
    height_m: float,
    kind: str,
    n_loops: int = 2,
    anchor_gap_m: float = 0.0,
    wall_factory=None,
) -> list:
    """Cut a maze into the Unknown Search Area and return its walls.

    ``wall_factory`` builds one wall from ``(x1, y1, x2, y2, thickness_m, height_m, kind)``;
    :mod:`safmc_sim.world.arena` passes its own ``Wall``. Injecting it keeps this module free of
    a circular import back into ``arena``.

    The returned walls are tagged ``kind="maze_wall"``. They must not be ``inner_wall``:
    ``_validate_gaps`` applies the 2 m rule all-pairs to that kind, and a maze T-junction has a
    mutual distance of exactly zero, so it would raise on the first seed. ``maze_wall`` joins
    the *structural* group alongside the room shell, which is the honest reading -- 3.3.9 r.2
    guarantees walls in this room, and a maze is one connected structure whose parts meet by
    design.

    An earlier cut reused ``unknown_wall``, the room shell's kind, to inherit its checks. That
    made the shell and its interior indistinguishable, which matters once arenas are durable
    artefacts you save, view and analyse -- telling them apart needed a geometric predicate
    rather than a field read. It also silently widened the room-versus-perimeter gap check to
    interior walls that are trivially clear of the perimeter. Two kinds, both structural.
    """
    if wall_factory is None:  # pragma: no cover - arena always supplies one
        raise ValueError("generate_maze requires a wall_factory")

    n, p, w, t = grid.n, grid.pitch_m, grid.corridor_w_m, grid.thickness_m
    passages = _spanning_tree_passages(rng, n)
    _braid(rng, n, passages, n_loops)

    # Every internal edge that the spanning tree did not carve through becomes a wall. Ends are
    # overhung by t/2 wherever a neighbouring cell continues, which is exactly the width of the
    # gap between two cells' clear extents, so junctions close flush instead of leaving a
    # needle-thin slit that the raycaster would happily shine through.
    verticals: dict[float, list[tuple[float, float]]] = {}
    horizontals: dict[float, list[tuple[float, float]]] = {}

    for i in range(n):
        for j in range(n):
            if i + 1 < n and frozenset({(i, j), (i + 1, j)}) not in passages:
                x = grid.x0_m + (i + 1) * p - t / 2.0
                lo = grid.y0_m + j * p - (t / 2.0 if j > 0 else 0.0)
                hi = grid.y0_m + j * p + w + (t / 2.0 if j + 1 < n else 0.0)
                verticals.setdefault(round(x, 9), []).append((lo, hi))
            if j + 1 < n and frozenset({(i, j), (i, j + 1)}) not in passages:
                y = grid.y0_m + (j + 1) * p - t / 2.0
                lo = grid.x0_m + i * p - (t / 2.0 if i > 0 else 0.0)
                hi = grid.x0_m + i * p + w + (t / 2.0 if i + 1 < n else 0.0)
                horizontals.setdefault(round(y, 9), []).append((lo, hi))

    walls = []
    for axis, buckets in (("v", verticals), ("h", horizontals)):
        for coord, spans in sorted(buckets.items()):
            for lo, hi in _merge_spans(spans):
                if anchor_gap_m > 0.0:
                    # Both ends, not only the ones touching a room face. Retracting just the
                    # face ends left every interior T- and L-junction at exactly zero distance
                    # -- 75 touching pairs over 40 seeds at anchor_gap 0.0, and still 75 at 2.0
                    # -- so the knob changed nothing about the property it claimed to control,
                    # and an arena labelled "generated under the all-pairs reading" was still an
                    # anchored maze.
                    lo, hi = lo + anchor_gap_m, hi - anchor_gap_m
                if hi - lo < t:
                    # A run shorter than its own thickness is not a wall. It would also be a
                    # degenerate ray-cast segment, which the raycaster classes as parallel and
                    # never intersects -- an invisible obstacle is worse than no obstacle.
                    continue
                if axis == "v":
                    walls.append(wall_factory(coord, lo, coord, hi, t, height_m, kind))
                else:
                    walls.append(wall_factory(lo, coord, hi, coord, t, height_m, kind))
    return walls


def _merge_spans(spans: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Merge overlapping or touching spans, so a column of stacked unit walls becomes one Wall.

    This is not cosmetic. Every wall costs the raycaster four segments on every ray of every
    drone on every tick, and the raycaster is dense and index-free, so cost is exactly linear in
    segment count. Merging is what keeps the maze's sensing overhead near 1.1x instead of 5x.
    """
    out: list[tuple[float, float]] = []
    for lo, hi in sorted(spans):
        if out and lo <= out[-1][1] + 1e-9:
            out[-1] = (out[-1][0], max(out[-1][1], hi))
        else:
            out.append((lo, hi))
    return out


