"""R-WORLD-7 / R-WORLD-8: landmarks -- what they are, what they block, how they are placed."""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from safmc_sim import constants as K
from safmc_sim.errors import ArenaError, ConfigError
from safmc_sim.sensors.raycast import cast_rays
from safmc_sim.sensors.scene import WorldScene
from safmc_sim.world.arena import ArenaConfig, Target, generate_arena, validate_arena
from safmc_sim.world.landmark import Landmark, occluder_scene


def test_a_target_is_a_landmark_with_the_marker_body():
    t = Target("victim_0", "victim", 3.0, 4.0)
    assert isinstance(t, Landmark)
    assert t.solid
    assert t.radius_m == K.MARKER_FOOTPRINT_M / 2 and t.height_m == K.MARKER_HEIGHT_M
    with pytest.raises(ConfigError, match="unknown target kind"):
        Target("x", "nav_tag", 1.0, 1.0)


def test_a_point_landmark_is_not_solid_and_casts_no_shadow():
    tag = Landmark("tag_12", "nav_tag", 10.0, 10.0)
    assert not tag.solid
    scene = occluder_scene([tag])
    assert scene.n_primitives == 0
    # A flat mark with an extent is still a point to a ray.
    mark = Landmark("start_00", "start_mark", 2.0, 2.0, radius_m=0.2)
    assert not mark.solid and occluder_scene([mark]).n_primitives == 0


def test_a_solid_landmark_occludes_with_a_height_gate():
    post = Landmark("anchor_0", "uwb_anchor", 12.0, 10.0, radius_m=0.05, height_m=0.8)
    assert post.solid
    scene = occluder_scene([post])
    origins, directions = np.array([[10.0, 10.0]]), np.array([[1.0, 0.0]])
    assert np.isfinite(cast_rays(scene, origins, directions, 0.5, 10.0)[0])
    assert np.isinf(cast_rays(scene, origins, directions, 0.9, 10.0)[0])


@pytest.mark.parametrize("kw", [
    dict(id=""), dict(kind=""), dict(radius_m=-1.0), dict(height_m=-1.0),
    dict(height_m=1.0),           # a height with no footprint is not a body
])
def test_impossible_landmarks_are_rejected(kw):
    base = dict(id="lm", kind="thing", x=1.0, y=1.0)
    with pytest.raises(ConfigError):
        Landmark(**{**base, **kw})


def test_landmarks_placed_by_config_reach_the_arena_and_the_scene():
    tag = Landmark("tag_12", "nav_tag", 3.0, 9.0)
    post = Landmark("anchor_0", "uwb_anchor", 1.0, 19.0, radius_m=0.05, height_m=0.8)
    arena = generate_arena(0, ArenaConfig(landmarks=(tag, post)))
    assert arena.landmarks == (tag, post)
    assert set(arena.all_landmarks) >= {tag, post} and len(arena.all_landmarks) == len(arena.targets) + 2
    assert arena.landmarks_of("nav_tag") == (tag,)
    # Only the body joins the ray-castable scene.
    assert len(arena.landmark_scene().circles) == len(arena.targets) + 1

    world = WorldScene.from_arena(arena)
    assert world.landmarks == arena.all_landmarks
    assert world.landmarks_of("nav_tag", "uwb_anchor") == (tag, post)
    # 2 circles per pillar (shaft + weighted base), plus the targets, plus the one
    # solid placed landmark.
    assert len(world.static_sensing_scene.circles) == 2 * len(arena.pillars) + len(arena.targets) + 1


def test_derived_placement_is_a_dataclass_replace():
    """A tag on every doorway: generate first, then place from the generated layout."""
    arena = generate_arena(1)
    x0, y0, x1, y1 = arena.unknown_area
    tags = (Landmark("tag_sw", "nav_tag", x0 - 1.0, y0 - 1.0),
            Landmark("tag_ne", "nav_tag", x1 + 1.0, y1 + 1.0))
    placed = dataclasses.replace(arena, landmarks=tags)
    validate_arena(placed)
    assert placed.landmarks == tags and placed.targets == arena.targets


def test_generation_keeps_targets_off_a_placed_body():
    """A body placed by config is occupied space to the target placer."""
    # (3.0, 18.0): in the Known Search Area for every seed, and close enough to where the
    # generator actually puts things that the guard has bite. An earlier relocation parked it
    # at (1.2, 18.5), the extreme corner of the feasible band, where nothing generated ever
    # came near -- the assertion then held even with the post absent from the config entirely.
    post = Landmark("anchor_0", "uwb_anchor", 3.0, 18.0, radius_m=0.3, height_m=1.0)
    for seed in range(8):
        arena = generate_arena(seed, ArenaConfig(landmarks=(post,)))
        for t in arena.targets:
            assert np.hypot(t.x - post.x, t.y - post.y) > t.radius_m + post.radius_m


def test_validation_catches_bad_landmarks():
    arena = generate_arena(0)

    def with_landmarks(*lms):
        return dataclasses.replace(arena, landmarks=lms)

    with pytest.raises(ArenaError, match="outside"):
        validate_arena(with_landmarks(Landmark("far", "nav_tag", 25.0, 1.0)))
    with pytest.raises(ArenaError, match="duplicate"):
        validate_arena(with_landmarks(Landmark(arena.targets[0].id, "nav_tag", 1.0, 1.0)))
    pillar = arena.pillars[0]
    with pytest.raises(ArenaError, match="overlaps an obstacle"):
        validate_arena(with_landmarks(
            Landmark("in_pillar", "prop", pillar.x, pillar.y, radius_m=0.1, height_m=1.0)))
    t = arena.targets[0]
    with pytest.raises(ArenaError, match="overlap"):
        validate_arena(with_landmarks(
            Landmark("on_target", "prop", t.x + 0.05, t.y, radius_m=0.1, height_m=1.0)))
    # A point may sit anywhere inside the field, including on a wall -- a tag on a wall.
    wall = next(w for w in arena.walls if w.kind == "inner_wall")
    validate_arena(with_landmarks(Landmark("wall_tag", "nav_tag", *wall.centre)))


def test_config_refuses_a_target_placed_as_a_landmark():
    with pytest.raises(ConfigError, match="Target"):
        ArenaConfig(landmarks=(Target("victim_9", "victim", 5.0, 9.0),))
    with pytest.raises(ConfigError, match="duplicate"):
        ArenaConfig(landmarks=(Landmark("a", "k", 1.0, 1.0), Landmark("a", "k", 2.0, 2.0)))


def test_a_solid_landmark_of_any_kind_is_lethal_below_its_height_and_not_above():
    """What can kill you is what you can see, for placed bodies as for markers."""
    from safmc_sim.api import Policy, Velocity, register_policy
    from safmc_sim.runner import RunConfig, run
    from safmc_sim.sensors.tof_ring import ToFConfig

    def fly_north_at(z_cruise):
        @register_policy(f"_north_at_{int(z_cruise * 100)}")
        class North(Policy):
            def step(self, obs):
                if obs.pose.z < z_cruise - 0.02:
                    return Velocity(vz=0.5)
                return Velocity(vy=0.45)
        return f"_north_at_{int(z_cruise * 100)}"

    # A fence of low posts across the Start Area, 0.1 m apart, north of the take-off row.
    # Inside the Start Area so it cannot collide with the room, which the generator draws
    # around placed bodies anyway.
    posts = tuple(
        Landmark(f"post_{i}", "prop", 1.0 + i * 0.4, 4.5, radius_m=0.15, height_m=0.6)
        for i in range(33)
    )
    cfg = ArenaConfig(landmarks=posts, n_pillars_known=0, n_inner_walls=0)

    low = run(RunConfig(seed=0, n_drones=10, duration_s=30.0, record=False,
                        policy=fly_north_at(0.4), arena_config=cfg, sensors=(ToFConfig(),)))
    high = run(RunConfig(seed=0, n_drones=10, duration_s=30.0, record=False,
                         policy=fly_north_at(0.9), arena_config=cfg, sensors=(ToFConfig(),)))
    def struck_posts(result):
        return [e for e in result.events
                if e.kind == "crashed" and "struck landmark post" in e.detail["reason"]]

    assert struck_posts(low), "the low fleet should hit the posts"
    # The high fleet clears the 0.6 m posts. It may still hit a 1.0 m mission marker further
    # north -- that is the same rule, applied to a taller body.
    assert not struck_posts(high)


# -- what the audit got through, and the world now refuses ---------------------------------------
#
# The first cut validated an arena whose every doorway was plugged with posts, accepted a
# landmark of kind "victim" as a decoy, let a drone land on top of a marker and score, and
# documented a placement path that could not be run. Each has a test now.

import tempfile
from dataclasses import dataclass

from safmc_sim import policies  # noqa: F401 -- registers sdlw
from safmc_sim.api import Land, Policy, Velocity, register_policy
from safmc_sim.recorder import Recorder, load_run, score_from_log
from safmc_sim.runner import RunConfig, Runner, run
from safmc_sim.sensors.marker_cam import MarkerCamConfig
from safmc_sim.sensors.tof_ring import ToFConfig
from safmc_sim.world.arena import TARGET_KINDS, target_polygon


def test_a_supplied_arena_runs_and_the_log_rescores_it():
    """The documented placement path -- generate, replace, run -- end to end."""
    from safmc_sim.metrics import compute_metrics

    arena = generate_arena(1)
    x0, y0, x1, y1 = arena.unknown_area
    tags = (Landmark("tag_sw", "nav_tag", x0 - 1.0, y0 - 1.0),
            Landmark("tag_ne", "nav_tag", x1 + 1.0, y1 + 1.0))
    placed = dataclasses.replace(arena, landmarks=tags)
    cfg = RunConfig(seed=1, n_drones=10, duration_s=3.0, policy="sdlw",
                    sensors=(ToFConfig(), MarkerCamConfig(kinds=TARGET_KINDS + ("nav_tag",))))
    with tempfile.TemporaryDirectory() as tmp:
        result = run(cfg, recorder=Recorder(tmp), arena=placed)
        log = load_run(tmp)
        assert log["header"]["arena_source"] == "supplied"
        assert [lm["id"] for lm in log["header"]["arena"]["landmarks"]] == ["tag_sw", "tag_ne"]
        assert score_from_log(tmp).total == result.score.total
        compute_metrics(tmp)
    assert result.arena is placed

    with pytest.raises(ConfigError, match="ArenaSpec"):
        Runner(cfg, arena=arena.config)


def test_a_landmark_subclass_round_trips_through_the_log_as_a_landmark():
    @dataclass(frozen=True)
    class NavTag(Landmark):
        tag_id: int = 0

    tag = NavTag("tag_12", "nav_tag", 3.0, 9.0, tag_id=12)
    cfg = RunConfig(seed=0, n_drones=10, duration_s=2.0, policy="sdlw",
                    arena_config=ArenaConfig(landmarks=(tag,)),
                    sensors=(ToFConfig(), MarkerCamConfig(kinds=TARGET_KINDS + ("nav_tag",))))
    with tempfile.TemporaryDirectory() as tmp:
        result = run(cfg, recorder=Recorder(tmp))
        row = load_run(tmp)["header"]["arena"]["landmarks"][0]
        assert set(row) == {"id", "kind", "x", "y", "radius_m", "height_m"}
        assert score_from_log(tmp).total == result.score.total


def test_decoy_targets_are_refused_on_every_path():
    with pytest.raises(ConfigError, match="mission kind"):
        ArenaConfig(landmarks=(Landmark("d", "victim", 5.0, 9.0),))
    with pytest.raises(ConfigError, match="mission kind"):
        ArenaConfig(landmarks=(Target("victim_9", "victim", 5.0, 9.0),))
    arena = generate_arena(0)
    for decoy in (Landmark("d", "fire", 5.0, 9.0), Target("t", "bonus_victim", 5.0, 9.0)):
        with pytest.raises(ArenaError, match="mission kind"):
            validate_arena(dataclasses.replace(arena, landmarks=(decoy,)))


@pytest.mark.parametrize("kw", [
    dict(x=float("nan")), dict(y=float("inf")), dict(radius_m=float("nan")),
    dict(radius_m=0.1, height_m=float("inf")), dict(radius_m=float("inf"), height_m=1.0),
])
def test_non_finite_landmarks_are_rejected(kw):
    with pytest.raises(ConfigError, match="finite"):
        Landmark(**{**dict(id="lm", kind="thing", x=1.0, y=1.0), **kw})


def test_solid_landmarks_block_the_grid_and_a_fence_walls_a_target_off():
    post = Landmark("post", "prop", 3.0, 18.0, radius_m=0.3, height_m=1.0)
    grid = generate_arena(0, ArenaConfig(landmarks=(post,))).occupancy_grid(0.1)
    # Derived from the post, not hard-coded: the index has to follow if the post ever moves.
    assert grid[int(post.x / 0.1), int(post.y / 0.1)], (
        "a solid landmark must be blocked in the occupancy grid"
    )

    # A ring of tall posts around a target, tight enough that a 0.18 m drone cannot pass:
    # nothing inside the ring is reachable and nothing within the 1 m landing radius is
    # outside it. Validation must say so rather than pass an unscorable arena.
    # The landing-spot check looks in a square window of +/-1 m around the target, so the
    # fence has to seal it out to the window's corners at 1.41 m: twelve 0.28 m posts on a
    # 1.4 m ring, inflated by the drone radius, close every gap and reach 1.6 m.
    ring_m, post_m, clear_m = 1.4, 0.28, 2.0

    def far_from_everything(arena, t):
        obstacles = arena.obstacle_polygons()
        return (
            min(o.distance(target_polygon(t)) for o in obstacles) > clear_m
            and all(np.hypot(t.x - o.x, t.y - o.y) > clear_m for o in arena.targets if o is not t)
            and clear_m < t.x < arena.width_m - clear_m
            # North of the Start Area by the whole fence: every free Start Area cell is
            # reachable by definition, so a fence overlapping it would not enclose anything.
            and arena.start_area_depth_m + clear_m < t.y < arena.depth_m - clear_m
            # In the Known Search Area: a fence is a team-placed thing, and 3.3.1 r.17 bars
            # teams from the Unknown Search Area, so a target in the room cannot be fenced.
            and arena.in_known_area(t.x, t.y)
        )

    # Scanned rather than fixed, because which seed offers a target with 2 m of clear space
    # is an accident of the generator. The range is wide enough to survive a reseeding: the
    # maze and the three-stream split both reshuffled every arena, and a range of 12 did not.
    for seed in range(64):
        arena = generate_arena(seed)
        target = next((t for t in arena.targets if far_from_everything(arena, t)), None)
        if target is not None:
            break
    assert target is not None, "no seed in 0..63 has a target with 2 m of clear space"
    angles = np.linspace(0.0, 2.0 * np.pi, 12, endpoint=False)
    fence = tuple(
        Landmark(f"fence_{i}", "prop", target.x + ring_m * np.cos(a), target.y + ring_m * np.sin(a),
                 radius_m=post_m, height_m=2.0)
        for i, a in enumerate(angles)
    )
    with pytest.raises(ArenaError, match="reachable"):
        validate_arena(dataclasses.replace(arena, landmarks=fence))


def test_a_body_under_a_take_off_position_is_refused():
    probe = Runner(RunConfig(seed=0, n_drones=10, duration_s=1.0, record=False, policy="sdlw"))
    x, y = probe._start_positions()[3][:2]
    post = Landmark("post", "prop", float(x), float(y), radius_m=0.1, height_m=0.5)
    runner = Runner(RunConfig(seed=0, n_drones=10, duration_s=1.0, record=False, policy="sdlw",
                              arena_config=ArenaConfig(landmarks=(post,))))
    try:
        with pytest.raises(ConfigError, match="drone_03 would take off inside"):
            runner.build()
    finally:
        runner._teardown()


def test_landing_onto_a_body_is_a_crash_not_a_score():
    """Overfly a 0.6 m post at 0.9 m, then land on it. At ground level the post is in the way."""
    probe = Runner(RunConfig(seed=0, n_drones=10, duration_s=1.0, record=False, policy="sdlw"))
    x0, y0 = probe._start_positions()[0][:2]
    post = Landmark("post", "prop", float(x0), float(y0) + 1.5, radius_m=0.15, height_m=0.6)

    @register_policy("_land_on_post")
    class LandOnPost(Policy):
        def step(self, obs):
            if self.agent_id != "drone_00":
                return Velocity()
            if obs.pose.z < 0.88:
                return Velocity(vz=0.5)
            dx, dy = post.x - obs.pose.x, post.y - obs.pose.y
            d = float(np.hypot(dx, dy))
            if d < 0.05:
                return Land()
            speed = min(0.3, d)
            return Velocity(vx=speed * dx / d, vy=speed * dy / d)

    result = run(RunConfig(seed=0, n_drones=10, duration_s=30.0, record=False,
                           policy="_land_on_post", arena_config=ArenaConfig(landmarks=(post,)),
                           sensors=(ToFConfig(),)))
    crashes = [e for e in result.events if e.kind == "crashed" and e.agent_id == "drone_00"]
    assert crashes, "drone_00 never reached the post"
    assert crashes[0].detail["reason"] == "struck landmark post while landing"
    assert "drone_00" not in result.landed

    # It is recorded where it stopped -- on top of the post -- not hovering at cruise.
    with tempfile.TemporaryDirectory() as tmp:
        run(RunConfig(seed=0, n_drones=10, duration_s=30.0, policy="_land_on_post",
                      arena_config=ArenaConfig(landmarks=(post,)), sensors=(ToFConfig(),)),
            recorder=Recorder(tmp))
        pose = load_run(tmp)["states"]["pose"]
    assert pose[-1, 0, 2] == pytest.approx(post.height_m)


def test_a_flat_mark_is_kept_clear_of_generated_structure():
    mark = Landmark("start_00", "start_mark", 3.0, 18.0, radius_m=0.5)
    footprint = target_polygon(mark)
    for seed in range(20):
        arena = generate_arena(seed, ArenaConfig(landmarks=(mark,)))
        assert not any(o.intersects(footprint) for o in arena.obstacle_polygons())
        assert all(
            np.hypot(t.x - mark.x, t.y - mark.y) > t.radius_m + mark.radius_m for t in arena.targets
        )


def test_the_generated_arena_s_landmarks_are_in_the_header_too():
    tag = Landmark("tag_1", "nav_tag", 3.0, 9.0)
    with tempfile.TemporaryDirectory() as tmp:
        run(RunConfig(seed=0, n_drones=10, duration_s=1.0, policy="sdlw",
                      arena_config=ArenaConfig(landmarks=(tag,)),
                      sensors=(ToFConfig(), MarkerCamConfig(kinds=TARGET_KINDS + ("nav_tag",)))),
            recorder=Recorder(tmp))
        header = load_run(tmp)["header"]
    assert header["arena_source"] == "generated"
    assert header["arena"]["landmarks"] == [
        {"id": "tag_1", "kind": "nav_tag", "x": 3.0, "y": 9.0, "radius_m": 0.0, "height_m": 0.0}
    ]


# -- second audit pass ---------------------------------------------------------------------------


def test_landmark_invariants_are_re_checked_on_a_resolved_arena():
    """A subclass that skipped super().__post_init__() must not reach the log."""

    @dataclass(frozen=True)
    class Sloppy(Landmark):
        def __post_init__(self):
            pass

    arena = generate_arena(0)
    for bad, match in (
        (Sloppy("e", "", 5.0, 9.0), "kind"),
        (Sloppy("n", "prop", 5.0, 9.0, radius_m=float("nan")), "finite"),
        (Sloppy("neg", "prop", 5.0, 9.0, radius_m=-0.5, height_m=1.0), ">= 0"),
        (Sloppy("", "prop", 5.0, 9.0), "id"),
    ):
        with pytest.raises(ArenaError, match=match):
            validate_arena(dataclasses.replace(arena, landmarks=(bad,)))

    @dataclass(frozen=True)
    class SloppyTarget(Target):
        def __post_init__(self):
            pass

    with pytest.raises(ArenaError, match="unknown kind"):
        validate_arena(dataclasses.replace(
            arena, targets=arena.targets + (SloppyTarget("t", "prop", 5.0, 9.0),)))


def test_a_supplied_arena_s_config_replaces_the_dead_one_in_the_log():
    arena = generate_arena(5, ArenaConfig(n_pillars_known=3))
    cfg = RunConfig(seed=1, n_drones=10, duration_s=1.0, policy="sdlw",
                    arena_config=ArenaConfig(n_pillars_known=0, n_inner_walls=0))
    with tempfile.TemporaryDirectory() as tmp:
        result = run(cfg, recorder=Recorder(tmp), arena=arena)
        header = load_run(tmp)["header"]
    assert result.config.arena_config == arena.config
    assert header["config"]["arena_config"]["n_pillars_known"] == 3
    assert header["seed"] == 1 and header["arena"]["seed"] == 5


# ---------------------------------------------------------------------------------------
# Zone permissions (rulebook 3.3.1 r.15-17). The three zones differ in what a team may DO in
# them, not just in where they are: unlimited aids in the Start Area, at most ten in the Known
# Search Area, and none at all in the Unknown Search Area, which teams may never enter.
# ---------------------------------------------------------------------------------------


def test_the_three_zones_partition_the_field():
    """Every point in the field is in exactly one zone, and the room is not the known area."""
    arena = generate_arena(0)
    for x in np.linspace(0.05, 19.95, 40):
        for y in np.linspace(0.05, 19.95, 40):
            zones = [
                arena.in_start_area(x, y),
                arena.in_known_area(x, y),
                arena.in_unknown_area(x, y),
            ]
            assert sum(zones) == 1, f"({x:.2f}, {y:.2f}) is in {sum(zones)} zones, not 1"


def test_a_landmark_inside_the_unknown_area_is_rejected():
    """The highest-value cheat the rules forbid: a surveyed anchor where teams may never go.

    Regression: point landmarks have radius 0, so they were in neither the `bodies` nor the
    `marks` list that room placement consults, and were never considered at all. Fifteen nav
    tags scattered over the field put a mean of 11.3 of them inside the room, in 200 seeds
    out of 200.
    """
    arena = generate_arena(0)
    x0, y0, x1, y1 = arena.unknown_area
    inside = Landmark("cheat", "nav_tag", (x0 + x1) / 2.0, (y0 + y1) / 2.0)
    with pytest.raises(ArenaError, match="Unknown Search Area"):
        validate_arena(dataclasses.replace(arena, landmarks=(inside,)))


def test_a_plain_landmark_smuggled_into_the_targets_list_is_still_caught():
    """The zone rule reads what a *sensor* reads, not just ``spec.landmarks``.

    An auditor put a plain Landmark of an anchor kind into ``spec.targets`` with
    dataclasses.replace. It is not a Target, so the mission ignores it; but ``all_landmarks``
    includes it, so a UWB tag ranged to a free anchor in the middle of the room while the
    zone rule and the nav-aid cap both looked at the other list."""
    arena = generate_arena(3)
    x0, y0, x1, y1 = arena.unknown_area
    smuggled = Landmark("cheat", "uwb_anchor", (x0 + x1) / 2.0, (y0 + y1) / 2.0)
    with pytest.raises(ArenaError, match="Unknown Search Area"):
        validate_arena(dataclasses.replace(arena, targets=arena.targets + (smuggled,)))
    # Outside the room the same smuggling is legal, so the check is about the zone, not the list.
    validate_arena(dataclasses.replace(
        arena, targets=arena.targets + (Landmark("ok", "uwb_anchor", 0.5, 0.5),)))


def test_generated_targets_in_the_room_are_not_caught_by_the_zone_rule():
    """3.3.9 r.2 puts bonus victims and fires inside the room by design.

    The zone rule reads `spec.landmarks`, never `all_landmarks`, so the mission markers the
    generator is *required* to put in there are untouched by it.
    """
    for seed in range(6):
        arena = generate_arena(seed)
        assert any(arena.in_unknown_area(t.x, t.y) for t in arena.targets_of("bonus_victim"))
        validate_arena(arena)  # raises if the zone rule caught a generated target


def test_nav_aids_are_placed_after_survey_not_fixed_in_config():
    """The faithful ordering: the room exists, the team surveys it, then places aids.

    A fixed ArenaConfig position cannot work in general -- 33 m^2 at the centre of the field is
    inside the room for every seed -- so this is the pattern the docs point to.
    """
    rng = np.random.default_rng(0)
    for seed in range(8):
        arena = generate_arena(seed)
        aids = []
        while len(aids) < K.NAV_AID_MAX_KNOWN_AREA:
            x, y = float(rng.uniform(0.5, 19.5)), float(rng.uniform(0.5, 19.5))
            if arena.in_known_area(x, y):
                aids.append(Landmark(f"aid_{len(aids)}", "nav_tag", x, y))
        placed = dataclasses.replace(arena, landmarks=tuple(aids))
        validate_arena(placed)
        assert len(placed.landmarks) == K.NAV_AID_MAX_KNOWN_AREA
        assert not any(placed.in_unknown_area(lm.x, lm.y) for lm in placed.landmarks)

# ---------------------------------------------------------------------------------------
# R-WORLD-11: the aid rules validate_arena deliberately leaves alone.
#
# The zone rule above (r.17) is enforced on every run. The cap of ten in the Known Search
# Area (r.15) and the 1 m x 1 m footprint (r.14 f) cannot be, because a Landmark may be
# scenery or a prop and the primitive does not say which -- so the caller names the kinds
# that are aids, and calls this itself. The runner never does.
# ---------------------------------------------------------------------------------------

from safmc_sim.world.arena import validate_nav_aids  # noqa: E402


def _aids_in_known_area(arena, n, kind="uwb_anchor", prefix="a"):
    """``n`` aids at surveyed positions inside the Known Search Area of THIS arena.

    Surveyed, not fixed: the room moves with the seed, so a hard-coded coordinate is inside
    it for some seeds. This is the ordering the docs prescribe -- generate, survey, place.
    """
    rng = np.random.default_rng(abs(hash(prefix)) % (2**32))
    aids = []
    while len(aids) < n:
        x, y = float(rng.uniform(0.5, 19.5)), float(rng.uniform(0.5, 19.5))
        if arena.in_known_area(x, y):
            aids.append(Landmark(f"{prefix}{len(aids)}", kind, x, y))
    return tuple(aids)


def test_any_number_of_aids_in_the_start_area_and_ten_in_the_known_area():
    arena = generate_arena(0)
    start = tuple(Landmark(f"s{i}", "uwb_anchor", 0.5 + i * 1.5, 0.5) for i in range(13))
    known = _aids_in_known_area(arena, K.NAV_AID_MAX_KNOWN_AREA)
    placed = dataclasses.replace(arena, landmarks=start + known)
    validate_arena(placed)
    validate_nav_aids(placed, ("uwb_anchor",))          # r.15 and r.16: allowed


def test_an_eleventh_aid_in_the_known_area_is_refused_and_kinds_count_together():
    arena = generate_arena(0)
    eleven = _aids_in_known_area(arena, K.NAV_AID_MAX_KNOWN_AREA + 1)
    with pytest.raises(ArenaError, match="at most 10"):
        validate_nav_aids(dataclasses.replace(arena, landmarks=eleven), ("uwb_anchor",))
    # Ten pass -- the check bites on the count, not on the placement.
    validate_nav_aids(dataclasses.replace(arena, landmarks=eleven[:-1]), ("uwb_anchor",))
    # Six anchors and five tags are eleven aids (r.15 counts aids, not kinds)...
    mixed = _aids_in_known_area(arena, 6) + _aids_in_known_area(arena, 5, "nav_tag", "t")
    with pytest.raises(ArenaError, match="11 navigation aids"):
        validate_nav_aids(dataclasses.replace(arena, landmarks=mixed), ("uwb_anchor", "nav_tag"))
    # ...but a kind the caller did not name is not an aid.
    validate_nav_aids(dataclasses.replace(arena, landmarks=mixed), ("uwb_anchor",))
    # Nor is an aid in the Start Area counted against the Known Area's ten (r.16).
    start = tuple(Landmark(f"s{i}", "uwb_anchor", 0.5 + i * 1.4, 0.5) for i in range(13))
    validate_nav_aids(dataclasses.replace(arena, landmarks=eleven[:-1] + start), ("uwb_anchor",))


def test_the_room_rule_and_the_cap_are_enforced_in_different_places():
    """Division of labour: validate_arena refuses the room on every run (r.17), and only the
    caller can apply the cap (r.15), because only the caller knows which kinds are aids."""
    arena = generate_arena(3)
    x0, y0, x1, y1 = arena.unknown_area
    in_room = dataclasses.replace(
        arena, landmarks=(Landmark("cheat", "uwb_anchor", (x0 + x1) / 2, (y0 + y1) / 2),))
    with pytest.raises(ArenaError, match="Unknown Search Area"):
        validate_arena(in_room)                        # the run itself refuses it...
    validate_nav_aids(in_room, ("uwb_anchor",))        # ...so this need not, and does not

    over_cap = dataclasses.replace(arena, landmarks=_aids_in_known_area(arena, 11))
    validate_arena(over_cap)                           # a legal placement, so the run allows it
    with pytest.raises(ArenaError, match="at most 10"):
        validate_nav_aids(over_cap, ("uwb_anchor",))   # only this says it breaks r.15


def test_an_aid_wider_than_a_metre_is_refused():
    arena = generate_arena(0)
    tripod = Landmark("tripod", "uwb_anchor", 3.0, 1.0, radius_m=0.5)      # exactly 1 m: fine
    stand = Landmark("stand", "uwb_anchor", 8.0, 1.0, radius_m=0.51)       # 1.02 m: not
    validate_nav_aids(dataclasses.replace(arena, landmarks=(tripod,)), ("uwb_anchor",))
    with pytest.raises(ArenaError, match="r.14"):
        validate_nav_aids(dataclasses.replace(arena, landmarks=(stand,)), ("uwb_anchor",))


def test_a_string_or_empty_kinds_is_refused_not_silently_passed():
    """validate_nav_aids(arena, "uwb_anchor") made a set of letters, counted nothing, and
    approved eleven aids -- the most natural wrong call passed everything."""
    arena = generate_arena(3)
    over_cap = dataclasses.replace(arena, landmarks=_aids_in_known_area(arena, 11))
    with pytest.raises(ArenaError, match="string"):
        validate_nav_aids(over_cap, "uwb_anchor")
    with pytest.raises(ArenaError, match="empty"):
        validate_nav_aids(over_cap, ())
    for kinds in (["uwb_anchor"], {"uwb_anchor"}, (k for k in ("uwb_anchor",))):
        with pytest.raises(ArenaError, match="at most 10"):
            validate_nav_aids(over_cap, kinds)
