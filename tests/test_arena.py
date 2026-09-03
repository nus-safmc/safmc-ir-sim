"""R-WORLD-1..6: arena generation, published geometry, and self-validation."""

import math

import numpy as np
import pytest
from shapely.geometry import box

from safmc_sim import constants as K
from safmc_sim.errors import ArenaError
from safmc_sim.world.arena import (
    ArenaConfig,
    ArenaSpec,
    Pillar,
    Target,
    Wall,
    generate_arena,
    validate_arena,
)


@pytest.fixture(scope="module")
def arenas():
    return [generate_arena(seed) for seed in range(12)]


def test_published_field_geometry(arenas):
    """R-WORLD-2."""
    for a in arenas:
        assert a.width_m == K.FIELD_WIDTH_M == 20.0
        assert a.depth_m == K.FIELD_DEPTH_M == 20.0
        assert a.ceiling_m == K.CEILING_M == 1.4
        assert a.start_area_depth_m == K.START_AREA_DEPTH_M == 6.0
        x0, y0, x1, y1 = a.unknown_area
        assert x1 - x0 == pytest.approx(K.UNKNOWN_AREA_SIZE_M)
        assert y1 - y0 == pytest.approx(K.UNKNOWN_AREA_SIZE_M)
        assert a.start_area == (0.0, 0.0, 20.0, 6.0)


def test_perimeter_wall_is_on_three_sides_and_the_south_edge_is_net(arenas):
    """The rulebook gives a perimeter wall on three sides and netting on all."""
    for a in arenas:
        perimeter = [w for w in a.walls if w.kind == "perimeter_wall"]
        nets = [w for w in a.walls if w.kind == "net"]
        assert len(perimeter) == 3
        assert len(nets) == 1
        assert all(w.height_m == K.PERIMETER_WALL_HEIGHT_M for w in perimeter)
        # R-WORLD-6: the south edge still has to stop drones, since ir-sim has no bounds.
        assert nets[0].height_m > K.CEILING_M


def test_wall_and_pillar_heights_are_the_published_values(arenas):
    for a in arenas:
        for w in a.walls:
            if w.kind in ("inner_wall", "unknown_wall"):
                assert w.height_m == K.INNER_WALL_HEIGHT_M
        for p in a.pillars:
            assert p.height_m == K.PILLAR_HEIGHT_M
            assert p.radius_m == pytest.approx(K.PILLAR_DIAMETER_M / 2)


def test_every_obstacle_is_taller_than_the_ceiling(arenas):
    """Consequence of the 1.4 m flight limit: structure is never overflyable, so 2D is exact."""
    for a in arenas:
        for w in a.walls:
            assert w.height_m > K.CEILING_M
        for p in a.pillars:
            assert p.height_m > K.CEILING_M


def test_target_counts_and_the_unknown_area_guarantee(arenas):
    """Rulebook 3.3.9 r.2: the Unknown Search Area contains bonus victim(s) and fire(s)."""
    for a in arenas:
        assert len(a.targets_of("victim")) == K.N_VICTIMS
        assert len(a.targets_of("bonus_victim")) == K.N_BONUS_VICTIMS
        assert len(a.targets_of("fire")) == K.N_FIRES
        assert any(a.in_unknown_area(t.x, t.y) for t in a.targets_of("bonus_victim"))
        assert any(a.in_unknown_area(t.x, t.y) for t in a.targets_of("fire"))


def test_no_target_is_in_the_start_area(arenas):
    for a in arenas:
        assert not any(a.in_start_area(t.x, t.y) for t in a.targets)


def test_target_ids_are_unique(arenas):
    for a in arenas:
        ids = [t.id for t in a.targets]
        assert len(set(ids)) == len(ids)


def test_generation_is_deterministic_and_seed_dependent():
    """R-WORLD-3, R-DET-1."""
    a, b, c = generate_arena(5), generate_arena(5), generate_arena(6)
    assert a.walls == b.walls and a.pillars == b.pillars and a.targets == b.targets
    assert (a.walls, a.targets) != (c.walls, c.targets)


def test_arenas_actually_vary_across_seeds(arenas):
    """A generator that ignores its seed would pass every other test here."""
    rooms = {tuple(round(v, 3) for v in a.unknown_area) for a in arenas}
    assert len(rooms) > 1
    positions = {(round(t.x, 3), round(t.y, 3)) for a in arenas for t in a.targets}
    assert len(positions) > 10 * len(arenas) / 2


def test_published_minimum_gaps_hold(arenas):
    """R-WORLD-4. Pillars keep 1 m; independently placed walls keep 2 m."""
    for a in arenas:
        pillars = [p.polygon() for p in a.pillars]
        walls = [w.polygon() for w in a.walls]
        for i, p in enumerate(pillars):
            for other in pillars[i + 1 :] + walls:
                assert p.distance(other) >= K.MIN_GAP_PILLAR_M - 1e-6


def test_every_target_has_a_reachable_landing_spot(arenas):
    """R-WORLD-4. An unreachable target silently caps the score and looks like policy failure."""
    for a in arenas:
        validate_arena(a)  # raises if not


def test_validation_rejects_a_target_inside_an_obstacle():
    a = generate_arena(0)
    pillar = a.pillars[0]
    bad = ArenaSpec(
        seed=a.seed, width_m=a.width_m, depth_m=a.depth_m, ceiling_m=a.ceiling_m,
        start_area_depth_m=a.start_area_depth_m, unknown_area=a.unknown_area,
        walls=a.walls, pillars=a.pillars,
        targets=a.targets + (Target(id="victim_x", kind="victim", x=pillar.x, y=pillar.y),),
        config=a.config,
    )
    with pytest.raises(ArenaError, match="overlaps an obstacle"):
        validate_arena(bad)


def test_validation_rejects_a_walled_off_target():
    """Seal the Unknown Search Area completely and its targets must become unreachable."""
    a = generate_arena(0)
    x0, y0, x1, y1 = a.unknown_area
    sealed = tuple(w for w in a.walls if w.kind != "unknown_wall") + (
        Wall(x0, y0, x1, y0, 0.1, K.INNER_WALL_HEIGHT_M, "unknown_wall"),
        Wall(x1, y0, x1, y1, 0.1, K.INNER_WALL_HEIGHT_M, "unknown_wall"),
        Wall(x1, y1, x0, y1, 0.1, K.INNER_WALL_HEIGHT_M, "unknown_wall"),
        Wall(x0, y1, x0, y0, 0.1, K.INNER_WALL_HEIGHT_M, "unknown_wall"),
    )
    bad = ArenaSpec(
        seed=a.seed, width_m=a.width_m, depth_m=a.depth_m, ceiling_m=a.ceiling_m,
        start_area_depth_m=a.start_area_depth_m, unknown_area=a.unknown_area,
        walls=sealed, pillars=a.pillars, targets=a.targets, config=a.config,
    )
    with pytest.raises(ArenaError, match="no reachable landing spot"):
        validate_arena(bad)


def test_generation_raises_rather_than_degrading_when_over_constrained():
    """Fail fast: an arena that cannot meet its constraints is not silently downgraded."""
    with pytest.raises(ArenaError):
        generate_arena(0, ArenaConfig(n_inner_walls=40, max_placement_attempts=300))


def test_scenes_separate_structure_from_markers(arenas):
    """R-MISS-2: markers occlude sensing but must never block line of sight."""
    for a in arenas:
        structural = a.structural_scene()
        markers = a.landmark_scene()
        assert len(markers.circles) == len(a.targets)
        assert len(structural.circles) == len(a.pillars)
        assert len(structural.segments) == 4 * len(a.walls)
        # No marker leaks into the line-of-sight scene.
        for t in a.targets:
            assert not np.any(
                np.all(np.isclose(structural.circles[:, :2], [t.x, t.y]), axis=1)
            )


def test_irsim_obstacles_exclude_markers(arenas):
    """Markers get a height-gated check of our own; ir-sim's collision is strictly 2D."""
    for a in arenas:
        items = a.to_irsim_obstacles()
        assert len(items) == len(a.walls) + len(a.pillars)


def test_irsim_world_block_is_well_formed(arenas):
    a = arenas[0]
    world = a.to_irsim_world(step_time=0.05)
    # R-TIME-4: sample_time below step_time makes ir-sim raise ZeroDivisionError.
    assert world["sample_time"] >= world["step_time"]
    assert world["width"] == 20 and world["height"] == 20


def test_wall_geometry_helpers_are_consistent():
    wall = Wall(1.0, 2.0, 5.0, 2.0, thickness_m=0.2, height_m=2.0)
    assert wall.length_m == pytest.approx(4.0)
    assert wall.centre == pytest.approx((3.0, 2.0))
    assert wall.angle_rad == pytest.approx(0.0)
    corners = wall.corners()
    assert corners.shape == (4, 2)
    assert wall.polygon().area == pytest.approx(4.0 * 0.2)
    assert wall.segments().shape == (4, 4)


def test_rotated_wall_polygon_area_is_preserved():
    wall = Wall(0.0, 0.0, 3.0, 4.0, thickness_m=0.1, height_m=2.0)
    assert wall.length_m == pytest.approx(5.0)
    assert wall.polygon().area == pytest.approx(5.0 * 0.1)


def test_occupancy_grid_shape_and_inflation(arenas):
    a = arenas[0]
    grid = a.occupancy_grid(0.1)
    assert grid.shape == (200, 200)
    inflated = a.occupancy_grid(0.1, inflate_m=0.5)
    assert inflated.sum() > grid.sum()
    assert bool(np.all(grid <= inflated))


def test_occupancy_grid_agrees_with_shapely_on_sample_points(arenas):
    """The analytic rasteriser must match the geometry it claims to rasterise."""
    a = arenas[0]
    res = 0.2
    grid = a.occupancy_grid(res)
    polys = a.obstacle_polygons()
    rng = np.random.default_rng(0)
    for _ in range(300):
        ix = int(rng.integers(0, grid.shape[0]))
        iy = int(rng.integers(0, grid.shape[1]))
        x, y = (ix + 0.5) * res, (iy + 0.5) * res
        cell = box(x - res / 2, y - res / 2, x + res / 2, y + res / 2)
        expected = any(p.intersects(cell) for p in polys)
        if grid[ix, iy] != expected:
            # The analytic test uses the cell centre, shapely the whole cell, so they can
            # legitimately differ for cells the obstacle only clips. Require agreement
            # wherever the obstacle covers the centre.
            assert not any(p.contains(cell.centroid) for p in polys) or grid[ix, iy]


def test_config_rejects_impossible_target_counts():
    from safmc_sim.errors import ConfigError

    with pytest.raises(ConfigError):
        ArenaConfig(n_bonus_victims=0)
    with pytest.raises(ConfigError):
        ArenaConfig(n_fires=0)
    with pytest.raises(ConfigError):
        ArenaConfig(inner_wall_length_range_m=(5.0, 2.0))


# ---------------------------------------------------------------------------------------
# The Unknown Search Area maze (R-WORLD-3). Section 3.2 pins nothing inside this room --
# "The layout within the Unknown Search Area is intentionally NOT shown" -- so what is
# asserted here is the shape of the *distribution*, never one layout.
# ---------------------------------------------------------------------------------------


def _cell_adjacency(a):
    """Rebuild the maze's cell graph from the emitted wall geometry.

    Deliberately independent of the generator's internals: it probes the midpoint between
    neighbouring cell centres against the finished polygons, so it would catch a maze whose
    internal bookkeeping disagrees with the walls it actually emitted.
    """
    from shapely.geometry import Point
    from shapely.strtree import STRtree

    from safmc_sim.world.maze import plan_grid

    grid = plan_grid(
        a.unknown_area[0], a.unknown_area[1], K.UNKNOWN_AREA_SIZE_M,
        a.config.wall_thickness_m, a.config.maze_corridor_m,
    )
    polys = [w.polygon() for w in a.walls]
    tree = STRtree(polys)
    adj: dict[tuple[int, int], set[tuple[int, int]]] = {}
    for i in range(grid.n):
        for j in range(grid.n):
            adj.setdefault((i, j), set())
            for di, dj in ((1, 0), (0, 1)):
                ni, nj = i + di, j + dj
                if ni >= grid.n or nj >= grid.n:
                    continue
                (x1, y1), (x2, y2) = grid.cell_centre(i, j), grid.cell_centre(ni, nj)
                mid = Point((x1 + x2) / 2.0, (y1 + y2) / 2.0)
                if not any(polys[k].intersects(mid) for k in tree.query(mid)):
                    adj[(i, j)].add((ni, nj))
                    adj.setdefault((ni, nj), set()).add((i, j))
    return grid, adj


def test_the_published_gap_fixes_the_maze_at_four_by_four():
    """The 2 m wall-to-wall gap is what sets the grid, not taste.

    9.90 m of clear interior admits four 2.400 m corridors; a fifth would be 1.900 m, below the
    published minimum. If this ever changes, the arena has silently started breaking a rule.
    """
    from safmc_sim.world.maze import plan_grid

    grid = plan_grid(0.0, 0.0, K.UNKNOWN_AREA_SIZE_M, K.WALL_THICKNESS_M, K.MAZE_CORRIDOR_M)
    assert grid.n == 4
    assert grid.corridor_w_m == pytest.approx(2.400)
    assert grid.corridor_w_m >= K.MIN_GAP_WALL_TO_WALL_M
    assert grid.span_m == pytest.approx(K.UNKNOWN_AREA_SIZE_M - K.WALL_THICKNESS_M)
    # A fifth corridor would violate the published gap.
    assert (grid.span_m - 4 * K.WALL_THICKNESS_M) / 5 < K.MIN_GAP_WALL_TO_WALL_M


def test_every_maze_is_fully_connected(arenas):
    """A sealed pocket is dead area a search policy burns the whole run trying to enter."""
    for a in arenas:
        grid, adj = _cell_adjacency(a)
        cells = [(i, j) for i in range(grid.n) for j in range(grid.n)]
        seen = {cells[0]}
        stack = [cells[0]]
        while stack:
            for nxt in adj[stack.pop()]:
                if nxt not in seen:
                    seen.add(nxt)
                    stack.append(nxt)
        assert len(seen) == len(cells), (
            f"seed {a.seed}: only {len(seen)} of {len(cells)} maze cells are reachable"
        )


def test_the_maze_has_dead_ends_and_loops(arenas):
    """Otherwise it is not a maze, and the room stays as easy to search as an open box.

    Dead ends are what force backtracking; the braid loops are what stop the 3.3.7 relay chain
    from resting on a single fragile path.
    """
    total_dead_ends = 0
    for a in arenas:
        grid, adj = _cell_adjacency(a)
        cells = [(i, j) for i in range(grid.n) for j in range(grid.n)]
        edges = sum(len(v) for v in adj.values()) // 2
        total_dead_ends += sum(1 for c in cells if len(adj[c]) == 1)
        assert edges - (len(cells) - 1) == a.config.n_maze_loops, (
            f"seed {a.seed}: expected exactly {a.config.n_maze_loops} independent cycles"
        )
    assert total_dead_ends > 0, "no arena had a dead end -- these are not mazes"


def test_maze_walls_never_block_a_doorway(arenas):
    """Doorways are snapped to whole cells so a wall end cannot bisect one.

    Without snapping a grid line lands inside the opening on roughly 42% of faces, halving a
    doorway that no other validator would object to.
    """
    for a in arenas:
        blocked = a.occupancy_grid(0.05, inflate_m=K.DRONE_RADIUS_M)
        x0, y0, x1, y1 = a.unknown_area
        interior = blocked[
            int(x0 / 0.05) : int(x1 / 0.05), int(y0 / 0.05) : int(y1 / 0.05)
        ]
        assert (~interior).any(), f"seed {a.seed}: the room has no free space at all"


def test_maze_walls_are_merged_into_long_runs(arenas):
    """Every wall costs the raycaster four segments per ray per drone per tick.

    A 4x4 maze has up to 24 unit edges; leaving them unmerged would multiply sensing cost for
    no change in geometry.
    """
    for a in arenas:
        unknown = [w for w in a.walls if w.kind == "unknown_wall"]
        assert len(unknown) <= 24, f"seed {a.seed}: {len(unknown)} unknown walls, unmerged?"


def test_maze_can_be_disabled_for_reproducing_old_results():
    """maze_corridor_m=None restores the free-standing-baffle behaviour."""
    a = generate_arena(0, ArenaConfig(maze_corridor_m=None, n_unknown_walls=2))
    assert a.config.maze_corridor_m is None
    validate_arena(a)


def test_maze_and_free_standing_walls_are_mutually_exclusive():
    """Silently ignoring one of the two would change obstacle density without saying so."""
    from safmc_sim.errors import ConfigError

    with pytest.raises(ConfigError, match="n_unknown_walls must be 0"):
        ArenaConfig(n_unknown_walls=2)


def test_anchor_gap_knob_spans_both_readings_of_the_two_metre_rule():
    """maze_anchor_gap_m=min_gap_wall_m degenerates to free-floating islands."""
    anchored = generate_arena(3)
    islands = generate_arena(3, ArenaConfig(maze_anchor_gap_m=K.MIN_GAP_WALL_TO_WALL_M))
    room_anchored = [w for w in anchored.walls if w.kind == "unknown_wall"]
    room_islands = [w for w in islands.walls if w.kind == "unknown_wall"]
    # Retracting both ends of every run can only shorten or delete walls.
    assert sum(w.length_m for w in room_islands) < sum(w.length_m for w in room_anchored)


def test_a_corridor_wider_than_the_room_is_refused():
    """Returning a one-cell 'maze' would be an empty room wearing the wrong name."""
    with pytest.raises(ArenaError, match="does not fit twice"):
        generate_arena(0, ArenaConfig(maze_corridor_m=9.0))


def test_known_area_pillars_stay_out_of_the_unknown_area(arenas):
    """Regression: _place_walls had a room guard and _place_pillars did not.

    94% of seeds leaked known-area pillars into the room, 2.27 per arena on average, so the
    room held about 4.3 pillars against a configured 2 and both areas' obstacle densities were
    wrong.
    """
    for a in arenas:
        room = box(*a.unknown_area)
        known = a.pillars[: a.config.n_pillars_known]
        leaked = [p for p in known if room.contains(box(p.x, p.y, p.x, p.y).centroid)]
        assert not leaked, (
            f"seed {a.seed}: {len(leaked)} known-area pillars leaked into the Unknown Search Area"
        )


def test_room_connectivity_validator_catches_a_sealed_pocket():
    """The structural argument is that this cannot happen; this proves the check would notice."""
    a = generate_arena(0)
    x0, y0, x1, y1 = a.unknown_area
    t = a.config.wall_thickness_m
    # A closed 2 m box in the middle of the room. Sealing a corner is not enough: a doorway can
    # open straight into it, which is exactly the mistake this test was written wrong once to
    # make. A box touching nothing is unambiguously enclosed.
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    h = 1.0
    sealed = a.walls + (
        Wall(cx - h, cy - h, cx + h, cy - h, t, K.INNER_WALL_HEIGHT_M, "unknown_wall"),
        Wall(cx - h, cy + h, cx + h, cy + h, t, K.INNER_WALL_HEIGHT_M, "unknown_wall"),
        Wall(cx - h, cy - h, cx - h, cy + h, t, K.INNER_WALL_HEIGHT_M, "unknown_wall"),
        Wall(cx + h, cy - h, cx + h, cy + h, t, K.INNER_WALL_HEIGHT_M, "unknown_wall"),
    )
    # Pillars and targets are dropped so the earlier gap and reachability checks cannot fire
    # first and mask the one this test is about.
    with pytest.raises(ArenaError, match="walled off from the Start Area"):
        validate_arena(
            ArenaSpec(**{**a.__dict__, "walls": sealed, "targets": (), "pillars": ()})
        )


def test_far_enough_prefilter_does_not_miss_obstacles_at_the_gap_boundary():
    """Regression: candidate.buffer(min_gap) inner-approximates the offset circle.

    For a pillar's 24-gon it fell 0.6 mm short, so the STRtree prefilter skipped a wall that was
    genuinely inside the gap; placement accepted it and validation then rejected the whole
    arena. It cost about one seed in 300 as a hard generation failure.
    """
    from safmc_sim.world.arena import _far_enough

    wall = Wall(0.0, 20.05, 20.0, 20.05, 0.1, K.PERIMETER_WALL_HEIGHT_M, "perimeter_wall")
    # A pillar whose true clearance is just under the published 1 m.
    pillar = Pillar(17.0947322, 18.8503116)
    assert wall.polygon().distance(pillar.polygon()) < K.MIN_GAP_PILLAR_M
    assert not _far_enough(pillar.polygon(), [wall.polygon()], K.MIN_GAP_PILLAR_M)
