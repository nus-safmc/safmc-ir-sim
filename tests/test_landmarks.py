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
    assert len(world.static_sensing_scene.circles) == len(arena.pillars) + len(arena.targets) + 1


def test_derived_placement_is_a_dataclass_replace():
    """A tag on every doorway: generate first, then place from the generated layout."""
    arena = generate_arena(1)
    x0, y0, x1, y1 = arena.unknown_area
    tags = (Landmark("tag_sw", "nav_tag", x0, y0), Landmark("tag_ne", "nav_tag", x1, y1))
    placed = dataclasses.replace(arena, landmarks=tags)
    validate_arena(placed)
    assert placed.landmarks == tags and placed.targets == arena.targets


def test_generation_keeps_targets_off_a_placed_body():
    """A body placed by config is occupied space to the target placer."""
    post = Landmark("anchor_0", "uwb_anchor", 10.0, 12.0, radius_m=0.3, height_m=1.0)
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
