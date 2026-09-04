"""R-WORLD-3 / R-DET-1: independent generation streams, and arenas that outlive the process.

An arena is three separable things -- the Known Search Area, the interior of the Unknown Search
Area, and where the mission markers went. Holding one fixed while resampling another is how a
result gets attributed to the thing that actually varied, and it is what a map library needs:
ten known areas crossed with N unknown interiors.
"""

import dataclasses
import json

import numpy as np
import pytest

from safmc_sim import constants as K
from safmc_sim.errors import ArenaError, LogFormatError
from safmc_sim.world.arena import ArenaConfig, generate_arena, validate_arena
from safmc_sim.world.landmark import Landmark
from safmc_sim.world.store import (
    SCHEMA_VERSION,
    arena_from_dict,
    arena_to_dict,
    load_arena,
    save_arena,
)


def _known(spec):
    """The Known Search Area's signature: its walls, its pillars, and where the room sits."""
    return (
        tuple(sorted((round(w.x1, 4), round(w.y1, 4), round(w.x2, 4), round(w.y2, 4))
                     for w in spec.walls if w.kind == "inner_wall")),
        tuple(sorted((round(p.x, 4), round(p.y, 4)) for p in spec.pillars
                     if not spec.in_unknown_area(p.x, p.y))),
        tuple(round(v, 4) for v in spec.unknown_area),
    )


def _maze(spec):
    """The maze pattern, in room-relative coordinates so a translated maze compares equal."""
    x0, y0, _, _ = spec.unknown_area
    return tuple(sorted((round(w.x1 - x0, 4), round(w.y1 - y0, 4),
                         round(w.x2 - x0, 4), round(w.y2 - y0, 4))
                        for w in spec.walls if w.kind == "maze_wall"))


def _mission(spec):
    return tuple(sorted((t.kind, round(t.x, 4), round(t.y, 4)) for t in spec.targets))


def test_layout_seed_pins_the_known_area_while_the_unknown_one_varies():
    """The library case: ten known areas, each crossed with many unknown interiors."""
    base = generate_arena(0, layout_seed=3, unknown_seed=0)
    for j in range(1, 8):
        other = generate_arena(0, layout_seed=3, unknown_seed=j)
        assert _known(other) == _known(base), f"unknown_seed={j} disturbed the known area"
    mazes = {_maze(generate_arena(0, layout_seed=3, unknown_seed=j)) for j in range(8)}
    assert len(mazes) > 1, "unknown_seed changed nothing"


def test_unknown_seed_pins_the_maze_while_the_known_area_varies():
    """The same maze, translated with the room. Pattern is room-relative, so it compares equal."""
    base = generate_arena(0, layout_seed=0, unknown_seed=5)
    rooms = {base.unknown_area}
    for k in range(1, 8):
        other = generate_arena(0, layout_seed=k, unknown_seed=5)
        assert _maze(other) == _maze(base), f"layout_seed={k} changed the maze pattern"
        rooms.add(other.unknown_area)
    assert len(rooms) > 1, "layout_seed did not move the room"


def test_mission_seed_moves_only_the_markers():
    a = generate_arena(0, layout_seed=1, unknown_seed=1, mission_seed=1)
    b = generate_arena(0, layout_seed=1, unknown_seed=1, mission_seed=2)
    assert _known(a) == _known(b) and _maze(a) == _maze(b)
    assert _mission(a) != _mission(b)


def test_streams_are_reproducible_and_recorded():
    """R-DET-1. An arena built with overrides must be regenerable, not merely replayable."""
    a = generate_arena(4, layout_seed=3, unknown_seed=7, mission_seed=11)
    b = generate_arena(4, layout_seed=3, unknown_seed=7, mission_seed=11)
    assert a == b
    assert (a.seed, a.layout_seed, a.unknown_seed, a.mission_seed) == (4, 3, 7, 11)
    # Plain seeding still reproduces, and leaves the per-stream seeds unset.
    c = generate_arena(4)
    assert c == generate_arena(4)
    assert (c.layout_seed, c.unknown_seed, c.mission_seed) == (None, None, None)


def test_streams_are_derived_by_seedsequence_spawn():
    """R-DET-3 mandates SeedSequence.spawn, not seed arithmetic like seed+1."""
    plain = generate_arena(9)
    spawned = np.random.SeedSequence(9).spawn(3)
    explicit = generate_arena(
        9,
        layout_seed=None, unknown_seed=None, mission_seed=None,
    )
    assert plain == explicit
    # An arithmetic scheme would make these collide; spawned children must not.
    assert len({s.generate_state(2).tobytes() for s in spawned}) == 3


def test_an_arena_round_trips_through_a_file(tmp_path):
    a = generate_arena(0, layout_seed=3, unknown_seed=7)
    a = dataclasses.replace(a, landmarks=(Landmark("aid_0", "nav_tag", 1.0, 19.0),))
    path = save_arena(a, tmp_path / "maps" / "map_03.json")
    assert path.exists()
    b = load_arena(path)
    assert b == a
    assert (b.seed, b.layout_seed, b.unknown_seed, b.mission_seed) == (0, 3, 7, None)


def test_a_map_file_carries_its_config_so_revalidation_uses_the_right_gaps():
    """Without the config, a map generated at a 1.4 m gap would be re-checked against 2.0 m."""
    a = generate_arena(1, ArenaConfig(min_gap_wall_m=1.4, n_pillars_known=2, n_maze_loops=0))
    b = arena_from_dict(arena_to_dict(a))
    assert b.config == a.config
    assert b.config.min_gap_wall_m == 1.4 and b.config.n_maze_loops == 0
    validate_arena(b)


def test_config_landmarks_and_spec_landmarks_stay_distinguishable(tmp_path):
    """dataclasses.replace rewrites the spec's copy and leaves the config's alone."""
    surveyed = Landmark("survey", "prop", 1.0, 1.0, radius_m=0.1, height_m=0.5)
    a = generate_arena(2, ArenaConfig(landmarks=(surveyed,)))
    placed = dataclasses.replace(a, landmarks=a.landmarks + (Landmark("late", "nav_tag", 2.0, 2.0),))
    b = load_arena(save_arena(placed, tmp_path / "m.json"))
    assert b.config.landmarks == (surveyed,)
    assert len(b.landmarks) == 2
    assert b == placed


def test_every_config_field_survives_a_round_trip():
    """Serialised from dataclasses.fields, so a knob added later cannot silently vanish."""
    a = generate_arena(0)
    written = set(arena_to_dict(a)["config"])
    declared = {f.name for f in dataclasses.fields(ArenaConfig)}
    assert written == declared, f"missing from the map file: {declared - written}"


def test_a_file_from_another_schema_is_refused(tmp_path):
    a = generate_arena(0)
    data = arena_to_dict(a)
    data["schema"] = "safmc-sim/arena/999"
    with pytest.raises(LogFormatError, match="schema"):
        arena_from_dict(data)
    assert data["schema"] != SCHEMA_VERSION


def test_a_file_from_a_newer_build_is_refused_not_silently_dropped():
    """An unknown config knob means the map was written by a build that knew more than this one."""
    data = arena_to_dict(generate_arena(0))
    data["config"]["maze_fractal_depth"] = 3
    with pytest.raises(LogFormatError, match="newer build"):
        arena_from_dict(data)


def test_loading_validates_by_default(tmp_path):
    """A map on disk gets hand-edited; a bad one would invalidate every run that used it."""
    a = generate_arena(0)
    path = save_arena(a, tmp_path / "m.json")
    data = json.loads(path.read_text())
    # Drag a nav tag into the Unknown Search Area, the one place a team may never reach.
    x0, y0, x1, y1 = a.unknown_area
    data["landmarks"] = [{"id": "cheat", "kind": "nav_tag", "x": (x0 + x1) / 2,
                          "y": (y0 + y1) / 2, "radius_m": 0.0, "height_m": 0.0}]
    path.write_text(json.dumps(data))
    with pytest.raises(ArenaError, match="Unknown Search Area"):
        load_arena(path)
    load_arena(path, validate=False)  # opt out explicitly and it loads


def test_a_saved_map_is_flyable(tmp_path):
    """The point of the whole thing: generate once, keep, reload, run."""
    from safmc_sim import policies  # noqa: F401  registers 'sdlw'
    from safmc_sim.runner import RunConfig, run

    a = generate_arena(3, layout_seed=3, unknown_seed=7)
    reloaded = load_arena(save_arena(a, tmp_path / "map_03.json"))
    result = run(RunConfig(seed=99, n_drones=K.FLEET_MIN, duration_s=10.0, record=False),
                 arena=reloaded)
    assert result.config.arena_config == reloaded.config


def test_a_missing_config_field_is_refused_not_defaulted():
    """The failure carrying the config exists to prevent, reachable by deleting one key.

    Taking this build's default for an absent field means a map generated at
    min_gap_wall_m=1.4 revalidates against 2.0 and is rejected for a violation it never
    committed -- or passes, and is then recorded in the run log as flown at 2.0.
    """
    data = arena_to_dict(generate_arena(0, ArenaConfig(min_gap_wall_m=1.4)))
    del data["config"]["min_gap_wall_m"]
    with pytest.raises(LogFormatError, match="missing ArenaConfig fields"):
        arena_from_dict(data)
    data.pop("config")
    with pytest.raises(LogFormatError, match="missing ArenaConfig fields"):
        arena_from_dict(data)


def test_a_landmark_subclass_is_refused_rather_than_silently_flattened():
    """A run log may drop subclass state; a map file may not -- preservation is its purpose."""

    @dataclasses.dataclass(frozen=True)
    class UwbAnchor(Landmark):
        channel: int = 3

    arena = generate_arena(0)
    placed = dataclasses.replace(arena, landmarks=(UwbAnchor("a0", "uwb", 1.0, 19.0),))
    with pytest.raises(LogFormatError, match="cannot represent"):
        arena_to_dict(placed)


def test_the_three_streams_stay_independent_of_each_other():
    """R-WORLD-9. Rejection sampling made the mission stream a function of the maze.

    The first two markers are forced into the room, so how many draws they burn depends on the
    maze; on one shared stream that shifted every later marker. With the Known Search Area
    bit-identical, changing only unknown_seed moved up to 12 of 12 markers and swung the number
    outside the room from 6 to 9 -- up to 45 points of achievable score migrating between zones
    because the maze changed.
    """
    ref = generate_arena(0, layout_seed=3, unknown_seed=0, mission_seed=11)
    outside_ref = {t.id: (round(t.x, 6), round(t.y, 6))
                   for t in ref.targets if not ref.in_unknown_area(t.x, t.y)}
    counts = {len(outside_ref)}
    for j in range(1, 12):
        a = generate_arena(0, layout_seed=3, unknown_seed=j, mission_seed=11)
        outside = {t.id: (round(t.x, 6), round(t.y, 6))
                   for t in a.targets if not a.in_unknown_area(t.x, t.y)}
        counts.add(len(outside))
        assert outside == outside_ref, (
            f"unknown_seed={j} moved a Known Search Area marker"
        )
    assert len(counts) == 1, f"the marker count outside the room varied: {sorted(counts)}"


def test_known_area_markers_do_not_leak_into_the_room():
    """The room holds exactly the two 3.3.9 r.2 requires, not a seed-dependent 2 to 7."""
    for seed in range(30):
        a = generate_arena(seed)
        inside = [t.id for t in a.targets if a.in_unknown_area(t.x, t.y)]
        assert sorted(inside) == ["bonus_victim_0", "fire_0"], (
            f"seed {seed}: room holds {sorted(inside)}"
        )


def test_anchor_gap_actually_produces_free_islands():
    """maze_anchor_gap_m = min_gap_wall_m must degenerate to the all-pairs reading.

    Retracting only the ends touching a room face left every interior T- and L-junction at
    exactly zero distance, so the knob changed nothing about the property it documented.
    """
    def touching(cfg):
        n = 0
        for seed in range(6):
            a = generate_arena(seed, cfg)
            mw = [w.polygon() for w in a.walls if w.kind == "maze_wall"]
            n += sum(1 for i in range(len(mw)) for j in range(i + 1, len(mw))
                     if mw[i].distance(mw[j]) < 1e-6)
        return n
    assert touching(ArenaConfig()) > 0, "an anchored maze should have touching walls"
    assert touching(ArenaConfig(maze_anchor_gap_m=K.MIN_GAP_WALL_TO_WALL_M)) == 0
