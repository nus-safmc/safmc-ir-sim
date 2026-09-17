"""R-SENS-17: the UWB ranging tag -- what it reports, what obstructs it, and how it lies.

The geometry and the noise model are pure functions and are tested as such; the runner path,
the log and the determinism guarantees are tested end to end, the way the contract tests do.
"""

from __future__ import annotations

import dataclasses
import functools
import importlib.util
import tempfile
from pathlib import Path

import numpy as np
import pytest

from safmc_sim import constants as K
from safmc_sim import policies  # noqa: F401 -- registers sdlw
from safmc_sim.api import Land, Policy, Velocity, register_policy
from safmc_sim.errors import ConfigError
from safmc_sim.recorder import Recorder, arena_from_log, load_run
from safmc_sim.runner import RunConfig, Runner, flown_sensors, run
from safmc_sim.sensors.base import TrueState, check_reading_is_immutable
from safmc_sim.sensors.marker_cam import MarkerCamConfig
from safmc_sim.sensors.raycast import RayScene, cast_rays, segment_clear
from safmc_sim.sensors.scene import WorldScene
from safmc_sim.sensors.tof_ring import ToFConfig
from safmc_sim.sensors.uwb import (
    UWBConfig,
    UWBRanges,
    UWBTag,
    anchor_positions,
    line_of_sight,
    measure,
    peer_line_of_sight,
    peer_sweep_rate_hz,
    sweep_rate_hz,
    true_ranges,
)
from safmc_sim.world.arena import (
    ArenaConfig,
    generate_arena,
    validate_arena,
    validate_nav_aids,
)
from safmc_sim.world.landmark import Landmark

SHORT = dict(n_drones=10, duration_s=3.0, record=False)

# Every stochastic part off: the geometry alone.
EXACT = dict(los_noise_std_m=0.0, nlos_noise_std_m=0.0, nlos_bias_m=0.0,
             nlos_drop_probability=0.0, outlier_probability=0.0)

# Anchors in the Start Area, where nothing is ever generated, so a drone at take-off has
# every one of them in line of sight.
START_ANCHORS = (
    Landmark("a_sw", "uwb_anchor", 0.5, 0.5),
    Landmark("a_se", "uwb_anchor", 19.5, 0.5),
    Landmark("a_n", "uwb_anchor", 10.0, 5.5),
)


def box_scene(size=20.0, height=2.0, walls=(), landmarks=()):
    """A walled box plus optional thin walls given as ``(x1, y1, x2, y2)``, all ``height`` tall."""
    segments = [[0, 0, size, 0], [size, 0, size, size], [size, size, 0, size], [0, size, 0, 0]]
    segments += [list(w) for w in walls]
    segments = np.array(segments, dtype=float)
    return WorldScene(
        RayScene(segments=segments, segment_heights=np.full(len(segments), float(height))),
        landmarks=landmarks,
    )


def truth_at(x, y, z=0.5):
    return TrueState("drone_00", -1, x=x, y=y, z=z, theta=0.0, vx=0.0, vy=0.0)


def make_tag(seed=0, **cfg):
    return UWBTag(UWBConfig(**cfg), np.random.default_rng(seed))


# -- the config ---------------------------------------------------------------------------------


def test_defaults_are_the_registered_assumptions():
    cfg = UWBConfig()
    assert cfg.name == "uwb" and cfg.rate_hz == K.UWB_RATE_HZ
    assert cfg.landmark_kinds == ("uwb_anchor",)
    assert (cfg.max_range_m, cfg.los_noise_std_m) == (K.UWB_MAX_RANGE_M, K.UWB_LOS_NOISE_STD_M)
    assert (cfg.nlos_bias_m, cfg.nlos_noise_std_m) == (K.UWB_NLOS_BIAS_M, K.UWB_NLOS_NOISE_STD_M)
    assert cfg.nlos_drop_probability == K.UWB_NLOS_DROP_PROBABILITY
    assert cfg.outlier_probability == K.UWB_OUTLIER_PROBABILITY == 0.0, "outliers are off until measured (A-18)"
    assert cfg.outlier_max_m == K.UWB_OUTLIER_MAX_M
    assert cfg.anchor_height_m == K.UWB_ANCHOR_HEIGHT_M
    # 10 Hz divides the 20 Hz loop; the runner would refuse otherwise.
    RunConfig(sensors=(cfg,))


def test_the_modelled_part_is_the_dw3000_and_its_channels_clear_the_banned_band():
    """Rule 6.3 bans 5.7-5.9 GHz outright and permits ultra-wideband in the same sentence.
    The DW3000 has exactly two channels and neither reaches the banned band -- the nearer
    edge is 340 MHz clear. This is the compliance argument, as arithmetic."""
    assert K.UWB_MODULE_PART == "DW3000"
    banned_lo, banned_hi = 5.7e9, 5.9e9
    for centre in (K.UWB_CHANNEL_5_HZ, K.UWB_CHANNEL_9_HZ):
        lo = centre - K.UWB_CHANNEL_BANDWIDTH_HZ / 2
        hi = centre + K.UWB_CHANNEL_BANDWIDTH_HZ / 2
        assert lo > banned_hi or hi < banned_lo, f"channel at {centre/1e9:.4f} GHz overlaps"
    assert K.UWB_CHANNEL_5_HZ - K.UWB_CHANNEL_BANDWIDTH_HZ / 2 - banned_hi == pytest.approx(340e6)


def test_the_sweep_rate_falls_with_the_fleet_not_the_anchor_count():
    """Slots are per tag and every anchor is swept inside one, so a bigger swarm ranges less
    often on the same radio. The model's fixed rate does not know that (F-32); this helper
    is how a scenario author finds the right one."""
    assert sweep_rate_hz(10) == pytest.approx(10.0)      # ten drones, 10 ms slots
    assert sweep_rate_hz(25) == pytest.approx(4.0)       # the rulebook's maximum fleet
    assert sweep_rate_hz(20) == pytest.approx(5.0)
    # It is 1 / (n_tags * slot): the anchor count does not appear.
    assert sweep_rate_hz(10, slot_s=0.015) == pytest.approx(1.0 / 0.15)
    # A fleet that is a multiple of five divides the 20 Hz tick; eleven drones does not, and
    # the runner refuses it rather than rounding a sensor's latency.
    for n in (10, 15, 20, 25):
        RunConfig(n_drones=n, sensors=(UWBConfig(rate_hz=sweep_rate_hz(n)),))
    with pytest.raises(ConfigError, match="does not divide"):
        RunConfig(n_drones=11, sensors=(UWBConfig(rate_hz=sweep_rate_hz(11)),))
    for bad in (0, -1, 2.5, True):
        with pytest.raises(ConfigError, match="n_tags"):
            sweep_rate_hz(bad)
    with pytest.raises(ConfigError, match="slot_s"):
        sweep_rate_hz(10, slot_s=0.0)


def test_a_full_fleet_ranges_at_four_hertz_end_to_end():
    """The 25-drone case the fixed default hides: same radio, same anchors, 4 Hz not 10."""
    seen = []

    @register_policy("_reads_uwb_at_4hz")
    class Reads(Policy):
        def step(self, obs):
            seen.append(obs.stale_ticks["uwb"])
            return Velocity(vz=0.4)

    run(RunConfig(seed=0, policy="_reads_uwb_at_4hz", n_drones=25, duration_s=3.0, record=False,
                  arena_config=ArenaConfig(landmarks=START_ANCHORS),
                  sensors=(ToFConfig(), UWBConfig(rate_hz=sweep_rate_hz(25)))))
    assert max(seen) == 4, "4 Hz on a 20 Hz loop: a reading ages to four ticks between sweeps"


def test_the_tag_is_not_part_of_the_flown_suite():
    """The airframe carries no UWB module; a run opts in by appending the config."""
    assert UWBConfig not in {type(s) for s in flown_sensors()}
    RunConfig(sensors=flown_sensors() + (UWBConfig(),))


@pytest.mark.parametrize("kw, match", [
    (dict(kind=""), "kind"),
    (dict(max_range_m=0.0), "max_range_m"),
    (dict(max_range_m=float("inf")), "max_range_m"),
    (dict(los_noise_std_m=-0.1), "los_noise_std_m"),
    (dict(nlos_noise_std_m=float("nan")), "nlos_noise_std_m"),
    (dict(nlos_bias_m=float("inf")), "nlos_bias_m"),
    (dict(nlos_drop_probability=1.5), "nlos_drop_probability"),
    (dict(outlier_probability=-0.01), "outlier_probability"),
    (dict(outlier_max_m=-1.0), "outlier_max_m"),
    (dict(anchor_height_m=-0.5), "anchor_height_m"),
    (dict(rate_hz=3.0), "does not divide"),
    (dict(kind="victim"), "mission kind"),
    (dict(kind="fire"), "R-POL-3"),
])
def test_impossible_configs_are_refused_at_construction(kw, match):
    with pytest.raises(ConfigError, match=match):
        cfg = UWBConfig(**kw)
        RunConfig(sensors=(cfg,))          # the rate is checked against the tick rate here


def test_a_tag_cannot_range_to_the_mission_markers():
    """Both auditors: UWBConfig(kind="victim") handed every policy the victims' true positions
    as "surveyed anchors" on the first sweep, and the R-POL-4 walk could not see it -- a
    position array is exactly what the reading is allowed to carry. Refused by kind."""
    for kind in ("victim", "bonus_victim", "fire"):
        with pytest.raises(ConfigError, match="true position"):
            UWBConfig(kind=kind)


def test_a_config_that_skipped_super_post_init_is_caught_at_build():
    """A sloppy subclass with nlos_drop_probability=5.0 ran a whole mission with every
    obstructed range dropped, silently. The config is re-validated when the tag is built."""
    @dataclasses.dataclass(frozen=True)
    class Sloppy(UWBConfig):
        def __post_init__(self):
            pass                                          # forgot super().__post_init__()

    # No anchors in the arena: with some, the runner's orphan-kind check fires first for the
    # config whose kind is "victim", which is a different refusal.
    for kw, match in ((dict(nlos_drop_probability=5.0), "probability"),
                      (dict(max_range_m=float("nan")), "max_range_m"),
                      (dict(kind="victim"), "mission kind")):
        with pytest.raises(ConfigError, match=match):
            Runner(RunConfig(seed=0, policy="sdlw", sensors=(ToFConfig(), Sloppy(**kw)), **SHORT)).build()


def test_numpy_scalars_are_accepted_and_a_bool_is_not():
    cfg = UWBConfig(max_range_m=np.float32(12.0), los_noise_std_m=np.float64(0.04),
                    nlos_drop_probability=np.int64(0))
    assert cfg.max_range_m == np.float32(12.0)
    with pytest.raises(ConfigError, match="finite number"):
        UWBConfig(max_range_m=True)


# -- the geometry -------------------------------------------------------------------------------


def test_range_is_three_dimensional_to_the_anchor_at_mount_height():
    anchor = Landmark("a", "uwb_anchor", 13.0, 10.0)
    tag = make_tag(anchor_height_m=2.0, **EXACT)
    reading = tag.sample(truth_at(10.0, 10.0, z=0.5), box_scene(landmarks=(anchor,)), 0)
    assert reading.anchor_ids == ("a",)
    assert np.allclose(reading.anchor_xyz_m, [[13.0, 10.0, 2.0]])
    # 3 m across and 1.5 m up: a planar trilateration would be 35 cm out.
    assert reading.ranges_m[0] == pytest.approx(np.hypot(3.0, 1.5), abs=1e-9)
    assert np.allclose(true_ranges([10.0, 10.0, 0.5], anchor_positions([anchor], 2.0)), [np.hypot(3.0, 1.5)])


def test_anchors_are_reported_in_arena_order_and_only_the_tag_s_kind():
    placed = (
        Landmark("tag_12", "nav_tag", 3.0, 9.0),
        Landmark("b", "uwb_anchor", 2.0, 2.0),
        Landmark("post", "prop", 8.0, 8.0, radius_m=0.1, height_m=1.0),
        Landmark("a", "uwb_anchor", 18.0, 18.0),
    )
    reading = make_tag(**EXACT).sample(truth_at(10.0, 10.0), box_scene(landmarks=placed), 0)
    assert reading.anchor_ids == ("b", "a"), "arena order, not sorted, not by distance"
    assert reading.ranges_m.shape == (2,) and reading.anchor_xyz_m.shape == (2, 3)
    assert reading.heard.all()


def test_a_tag_with_nothing_to_hear_reports_an_empty_sweep():
    reading = make_tag().sample(truth_at(10.0, 10.0), box_scene(), 0)
    assert reading.anchor_ids == () and reading.ranges_m.shape == (0,)
    assert reading.anchor_xyz_m.shape == (0, 3)
    check_reading_is_immutable("uwb", reading)


def test_beyond_max_range_is_inf_not_a_number():
    anchor = Landmark("far", "uwb_anchor", 19.0, 19.0)
    tag = make_tag(max_range_m=5.0, **EXACT)
    reading = tag.sample(truth_at(1.0, 1.0), box_scene(landmarks=(anchor,)), 0)
    assert np.isinf(reading.ranges_m[0]) and not reading.heard[0]


# -- obstruction: walls and pillars, not markers, not teammates ---------------------------------


class _FakeRobot:
    """Enough of an ir-sim robot for WorldScene.refresh_drones: an id and a state column."""

    def __init__(self, id_, x, y):
        self.id = id_
        self.state = np.array([[x], [y], [0.0], [0.5], [0.0], [0.0]])


def test_a_wall_obstructs_and_a_marker_or_a_teammate_does_not():
    behind_wall = Landmark("behind", "uwb_anchor", 14.0, 10.0)
    past_marker = Landmark("past", "uwb_anchor", 10.0, 14.0)
    marker = Landmark("victim_like", "prop", 10.0, 12.0, radius_m=0.15, height_m=1.0)
    # A thin wall at x = 12 from y = 5 to 15 sits between the tag and `behind_wall`.
    world = box_scene(walls=[(12.0, 5.0, 12.0, 15.0)], landmarks=(behind_wall, past_marker, marker))
    world.refresh_drones([_FakeRobot(7, 10.0, 12.5)], cache_key=0)      # a teammate in the way

    tag = make_tag(nlos_bias_m=0.5, los_noise_std_m=0.0, nlos_noise_std_m=0.0,
                   nlos_drop_probability=0.0)
    truth = truth_at(10.0, 10.0)
    reading = tag.sample(truth, world, 0)
    d = true_ranges([10.0, 10.0, 0.5], reading.anchor_xyz_m)
    assert reading.ranges_m[0] == pytest.approx(d[0] + 0.5), "behind the wall: biased"
    assert reading.ranges_m[1] == pytest.approx(d[1]), "behind a marker and a teammate: clean"

    # The ring, by contrast, is stopped by the marker -- radio and light disagree, on purpose.
    # A ray north from the tag hits the marker's face at 1.85 m in the sensing scene, and
    # nothing before the box's far wall at 10 m in the structural one.
    origins = np.array([[10.0, 10.0]])
    up = np.array([[0.0, 1.0]])
    assert cast_rays(world.sensing_scene(), origins, up, 0.5, 20.0)[0] == pytest.approx(1.85)
    assert cast_rays(world.structural_scene, origins, up, 0.5, 20.0)[0] == pytest.approx(10.0)
    assert line_of_sight(world.structural_scene, [10.0, 10.0], reading.anchor_xyz_m, 0.5).tolist() == [False, True]


def test_obstruction_is_decided_at_the_drone_s_altitude():
    """A low wall blocks a low drone and not a high one -- the R-SENS-6 gate, and F-25."""
    anchor = Landmark("a", "uwb_anchor", 14.0, 10.0)
    low_wall = box_scene(walls=[(12.0, 5.0, 12.0, 15.0)], landmarks=(anchor,))
    # The box is 2.0 m; make only the inner wall 1.0 m tall.
    scene = low_wall.structural_scene
    heights = scene.segment_heights.copy()
    heights[-1] = 1.0
    low_wall = WorldScene(RayScene(segments=scene.segments, segment_heights=heights), landmarks=(anchor,))
    tag = make_tag(nlos_bias_m=0.5, los_noise_std_m=0.0, nlos_noise_std_m=0.0, nlos_drop_probability=0.0)
    d = np.hypot(4.0, 2.0 - 0.5)
    assert tag.sample(truth_at(10.0, 10.0, z=0.5), low_wall, 0).ranges_m[0] == pytest.approx(d + 0.5)
    d_high = np.hypot(4.0, 2.0 - 1.2)
    assert tag.sample(truth_at(10.0, 10.0, z=1.2), low_wall, 1).ranges_m[0] == pytest.approx(d_high)


def test_an_anchor_on_the_field_s_edge_is_never_obstructed_by_the_perimeter():
    """An anchor at x = 0.0 sits on the perimeter's inner face; the strict line-of-sight test
    lost to rounding one time in a hundred and applied the through-wall bias. Tolerance."""
    arena = generate_arena(0)
    edge = Landmark("edge", "uwb_anchor", 0.0, 3.0)
    world = WorldScene.from_arena(dataclasses.replace(arena, landmarks=(edge,)))
    tag = make_tag(nlos_bias_m=1.0, los_noise_std_m=0.0, nlos_noise_std_m=0.0, nlos_drop_probability=0.0)
    rng = np.random.default_rng(0)
    for _ in range(500):
        x, y = rng.uniform(0.3, 5.5, size=2)                 # the Start Area: nothing in the way
        reading = tag.sample(truth_at(x, y), world, 0)
        assert reading.ranges_m[0] == pytest.approx(true_ranges([x, y, 0.5], reading.anchor_xyz_m)[0])


def test_the_generated_arena_s_walls_are_what_obstruct():
    """End to end on a real arena: the tag's obstruction agrees with the mission's LOS scene."""
    arena = generate_arena(4)
    corners = tuple(Landmark(f"c{i}", "uwb_anchor", x, y)
                    for i, (x, y) in enumerate([(0.5, 19.5), (19.5, 19.5), (0.5, 0.5), (19.5, 0.5)]))
    world = WorldScene.from_arena(dataclasses.replace(arena, landmarks=corners))
    tag = make_tag(nlos_bias_m=1.0, los_noise_std_m=0.0, nlos_noise_std_m=0.0,
                   nlos_drop_probability=0.0, max_range_m=100.0)
    rng = np.random.default_rng(1)
    for _ in range(40):
        x, y = rng.uniform(0.3, 19.7, size=2)
        reading = tag.sample(truth_at(x, y), world, 0)
        d = true_ranges([x, y, 0.5], reading.anchor_xyz_m)
        clear = segment_clear(arena.structural_scene(), np.tile([x, y], (4, 1)), reading.anchor_xyz_m[:, :2], 0.5)
        assert np.allclose(reading.ranges_m, d + np.where(clear, 0.0, 1.0))


# -- the noise model, as a pure function -----------------------------------------------------------


def _draws(n, seed=0):
    rng = np.random.default_rng(seed)
    return rng.normal(0.0, 1.0, n), rng.random(n), rng.random(n), rng.random(n)


def test_line_of_sight_noise_is_zero_mean_gaussian_at_the_registered_std():
    n = 40_000
    cfg = UWBConfig()
    r = measure(np.full(n, 5.0), np.ones(n, bool), cfg, *_draws(n))
    assert np.isfinite(r).all(), "line of sight never drops"
    assert np.mean(r) == pytest.approx(5.0, abs=0.002)
    assert np.std(r) == pytest.approx(cfg.los_noise_std_m, abs=0.002)


def test_obstructed_ranges_are_biased_wider_and_sometimes_dropped():
    n = 40_000
    cfg = UWBConfig()
    r = measure(np.full(n, 5.0), np.zeros(n, bool), cfg, *_draws(n))
    heard = np.isfinite(r)
    assert np.mean(heard) == pytest.approx(1.0 - cfg.nlos_drop_probability, abs=0.01)
    assert np.mean(r[heard]) == pytest.approx(5.0 + cfg.nlos_bias_m, abs=0.02)
    assert np.std(r[heard]) == pytest.approx(cfg.nlos_noise_std_m, abs=0.02)
    # A dropped anchor is inf, never a number: "nothing heard" and "far away" stay distinct.
    assert np.isinf(r[~heard]).all()


def test_the_model_is_optimistic_against_the_measured_dw3000_by_a_known_margin():
    """F-30, pinned so it cannot drift silently.

    The only independent measurement of the chosen part (Ember et al., IFIP 2024: a DW3000 in
    a 60x40 m office, channel 9) reports mean absolute errors of 5.7 cm in line of sight and
    46.7 cm behind an obstruction, with 90th percentiles of 13.7 cm and 129.5 cm. This model
    is optimistic against all four, and most so in the tail -- which is the outlier term
    (A-18) being off. If a change makes the model *worse* than measured, this fails."""
    cfg, n, d = UWBConfig(), 400_000, 8.0
    rng = np.random.default_rng(0)

    def error(los):
        draws = (rng.normal(0, 1, n), rng.random(n), rng.random(n), rng.random(n))
        r = measure(np.full(n, d), np.full(n, los), cfg, *draws)
        return np.abs(r[np.isfinite(r)] - d)

    los, nlos = error(True), error(False)
    measured = {"los_mae": 0.057, "los_p90": 0.137, "nlos_mae": 0.467, "nlos_p90": 1.295}
    model = {
        "los_mae": los.mean(), "los_p90": np.percentile(los, 90),
        "nlos_mae": nlos.mean(), "nlos_p90": np.percentile(nlos, 90),
    }
    for key, measured_value in measured.items():
        assert model[key] < measured_value, f"{key}: model {model[key]:.3f} is no longer optimistic"
    # ...but not wildly so in the body of the distribution: within a factor of two.
    assert model["los_mae"] > measured["los_mae"] / 2
    assert model["nlos_mae"] > measured["nlos_mae"] / 2
    # The tail is where the gap is worst, and switching the outlier term on narrows it.
    assert model["nlos_p90"] < measured["nlos_p90"] / 1.5
    heavier = UWBConfig(outlier_probability=0.15)
    draws = (rng.normal(0, 1, n), rng.random(n), rng.random(n), rng.random(n))
    r = measure(np.full(n, d), np.zeros(n, bool), heavier, *draws)
    tail = np.percentile(np.abs(r[np.isfinite(r)] - d), 90)
    assert tail > model["nlos_p90"], "the outlier term is what fattens the tail"


def test_out_of_reach_is_inf_and_a_reported_range_is_never_negative():
    cfg = UWBConfig(max_range_m=10.0, los_noise_std_m=1.0)
    n = 10_000
    r = measure(np.full(n, 0.01), np.ones(n, bool), cfg, *_draws(n))
    assert (r >= 0.0).all() and (r == 0.0).any(), "clipped at zero, not reflected or negative"
    r = measure(np.array([9.99, 10.0, 10.01]), np.ones(3, bool), cfg, *_draws(3))
    assert np.isfinite(r[:2]).all() and np.isinf(r[2])


def test_outliers_are_off_by_default_and_positive_when_on():
    n = 1000
    gauss, u_drop, u_out, u_size = np.zeros(n), np.ones(n), np.zeros(n), np.ones(n)
    off = measure(np.full(n, 5.0), np.ones(n, bool), UWBConfig(), gauss, u_drop, u_out, u_size)
    assert np.allclose(off, 5.0), "outlier_probability defaults to zero (A-18)"
    on = measure(np.full(n, 5.0), np.ones(n, bool), UWBConfig(outlier_probability=1.0, outlier_max_m=1.5),
                 gauss, u_drop, u_out, u_size)
    assert np.allclose(on, 6.5), "an outlier is a positive error of up to outlier_max_m"
    half = measure(np.full(n, 5.0), np.ones(n, bool), UWBConfig(outlier_probability=0.5),
                   gauss, u_drop, np.linspace(0, 1, n, endpoint=False), u_size)
    assert np.mean(half > 5.0) == pytest.approx(0.5, abs=0.01)


# -- determinism ---------------------------------------------------------------------------------


def test_the_noise_stream_does_not_depend_on_the_geometry():
    """Same seed, a wall in one world and not the other, an anchor in reach or not: the
    generators stay in lockstep. A mutant that drew the Gaussian only for anchors in reach
    survived the first version of this test, which only varied the walls."""
    anchors = (Landmark("a", "uwb_anchor", 14.0, 10.0), Landmark("b", "uwb_anchor", 10.0, 14.0))
    walled = box_scene(walls=[(12.0, 5.0, 12.0, 15.0)], landmarks=anchors)
    open_ = box_scene(landmarks=anchors)
    tag_w, tag_o = make_tag(seed=9), make_tag(seed=9)
    for tick in range(5):
        tag_w.sample(truth_at(10.0, 10.0), walled, tick)
        tag_o.sample(truth_at(10.0, 10.0), open_, tick)
    assert tag_w.rng.random() == tag_o.rng.random()

    near, far = make_tag(seed=9, max_range_m=5.0), make_tag(seed=9, max_range_m=5.0)
    for tick in range(5):
        near.sample(truth_at(10.0, 10.0), open_, tick)      # both anchors within 5 m
        reading = far.sample(truth_at(1.0, 1.0), open_, tick)   # neither is
        assert not reading.heard.any()
    assert near.rng.random() == far.rng.random()


def test_identical_runs_with_the_tag_are_identical():
    seen: dict[str, list] = {}

    @register_policy("_records_uwb")
    class Records(Policy):
        def step(self, obs):
            seen.setdefault(self.config["run"], []).append(obs.sensors["uwb"].ranges_m.tolist())
            return Velocity(vz=0.3)

    def config(label):
        return RunConfig(seed=3, policy="_records_uwb", policy_config={"run": label},
                         arena_config=ArenaConfig(landmarks=START_ANCHORS),
                         sensors=(ToFConfig(), UWBConfig()), **SHORT)

    run(config("a"))
    run(config("b"))
    assert seen["a"] == seen["b"]


def test_appending_the_tag_leaves_the_flown_sensors_streams_untouched():
    """R-DET-3 for the tag: generators are spawned in config order, so the ring's noise is
    the same with and without a UWB tag after it."""
    def first_draw(sensors, arena_config=ArenaConfig()):
        runner = Runner(RunConfig(seed=5, policy="sdlw", sensors=sensors, arena_config=arena_config, **SHORT)).build()
        try:
            return [a.sensors[0].rng.random() for a in runner.agents]
        finally:
            runner._teardown()

    alone = first_draw((ToFConfig(noise_std_m=0.01), MarkerCamConfig()))
    with_tag = first_draw((ToFConfig(noise_std_m=0.01), MarkerCamConfig(), UWBConfig()),
                          ArenaConfig(landmarks=START_ANCHORS))
    assert alone == with_tag


# -- the runner path -----------------------------------------------------------------------------


def test_the_reading_reaches_the_policy_by_name_fresh_every_other_tick():
    seen = []

    @register_policy("_reads_uwb")
    class ReadsUWB(Policy):
        def step(self, obs):
            reading = obs.sensors["uwb"]
            assert isinstance(reading, UWBRanges)
            assert reading.anchor_ids == ("a_sw", "a_se", "a_n")
            check_reading_is_immutable("uwb", reading)
            assert not any(isinstance(v, Landmark) for v in reading.anchor_ids)
            with pytest.raises(ValueError, match="read-only"):
                reading.ranges_m[:] = 0.0
            with pytest.raises(ValueError, match="read-only"):
                reading.anchor_xyz_m[:] = 0.0
            seen.append((obs.agent_id, obs.tick, obs.stale_ticks["uwb"], reading))
            return Velocity(vz=0.4)

    run(RunConfig(seed=0, policy="_reads_uwb", arena_config=ArenaConfig(landmarks=START_ANCHORS),
                  sensors=flown_sensors() + (UWBConfig(),), **SHORT))
    assert seen
    for agent, tick, stale, reading in seen:
        assert stale == tick % 2, "10 Hz on a 20 Hz loop: fresh at even ticks, one tick old at odd"
        # Every Start Area anchor is in line of sight and in reach of a drone at take-off.
        assert reading.heard.all()
    # And it is sampling the true state: drone_00 climbs 1.2 m towards a 2.0 m anchor about
    # 2 m away, so its 3D range to that anchor shrinks by far more than the 5 cm noise.
    a_sw = [r.ranges_m[0] for agent, t, s, r in seen if agent == "drone_00" and s == 0]
    assert a_sw[-1] < a_sw[0] - 0.3


def test_an_arena_with_anchors_cannot_be_flown_without_the_tag():
    """A point nobody ranges to is refused (R-WORLD-8); a base makes no difference."""
    with_base = (Landmark("a", "uwb_anchor", 10.0, 3.0, radius_m=0.25),)
    for anchors in (START_ANCHORS, with_base):
        with pytest.raises(ConfigError, match="uwb_anchor"):
            Runner(RunConfig(seed=0, policy="sdlw", arena_config=ArenaConfig(landmarks=anchors),
                             sensors=flown_sensors(), **SHORT))
    Runner(RunConfig(seed=0, policy="sdlw", arena_config=ArenaConfig(landmarks=with_base),
                     sensors=flown_sensors() + (UWBConfig(),), **SHORT))


def test_the_log_holds_ranges_and_anchor_positions_in_the_header_s_landmark_order():
    """R-SENS-17 / R-OBS-3: uwb.npz plus the header is enough to grade the sensor."""
    anchors = START_ANCHORS + (Landmark("a_far", "uwb_anchor", 10.0, 19.0, radius_m=0.25),)
    cfg = RunConfig(seed=1, policy="sdlw", n_drones=10, duration_s=3.0,
                    arena_config=ArenaConfig(landmarks=anchors),
                    sensors=flown_sensors() + (UWBConfig(),))
    with tempfile.TemporaryDirectory() as tmp:
        result = run(cfg, recorder=Recorder(tmp))
        log = load_run(tmp)

    entry = log["header"]["sensors"][2]
    assert entry["name"] == "uwb" and entry["recorded"] is True
    assert entry["type"].endswith("UWBConfig") and entry["rate_hz"] == 10.0

    uwb = log["sensors"]["uwb"]
    assert uwb["ranges_m"].shape == (result.ticks, 10, 4)
    assert uwb["anchor_xyz_m"].shape == (4, 3)
    assert set(np.diff(uwb["sample_tick"][:, 0])) <= {0, 2}

    # Column j is the j-th landmark of the tag's kind in the header, in the header's order.
    in_header = [lm for lm in log["header"]["arena"]["landmarks"] if lm["kind"] == "uwb_anchor"]
    assert [lm["id"] for lm in in_header] == ["a_sw", "a_se", "a_n", "a_far"]
    assert np.allclose(uwb["anchor_xyz_m"][:, :2], [[lm["x"], lm["y"]] for lm in in_header])
    assert np.allclose(uwb["anchor_xyz_m"][:, 2], K.UWB_ANCHOR_HEIGHT_M)

    # Grade it from the log alone: the fresh rows in line of sight are within the LOS noise.
    fresh = uwb["sample_tick"] == uwb["ticks"][:, None]                  # (T, N)
    pose = log["states"]["pose"]                                         # (T, N, 4): x, y, z, theta
    tag_xyz = pose[:, :, :3]
    d = np.linalg.norm(tag_xyz[:, :, None, :] - uwb["anchor_xyz_m"][None, None, :, :], axis=-1)
    error = uwb["ranges_m"] - d                                          # (T, N, A)
    los_scene = arena_from_log(log["header"]).structural_scene()
    t_idx, n_idx = np.nonzero(fresh)
    a = np.repeat(pose[t_idx, n_idx, :2], 4, axis=0)
    b = np.tile(uwb["anchor_xyz_m"][:, :2], (len(t_idx), 1))
    # At cruise altitude: every crossable structure is 2.0 m, so any flyable altitude agrees.
    clear = segment_clear(los_scene, a, b, 0.5).reshape(len(t_idx), 4)
    heard = np.isfinite(uwb["ranges_m"][t_idx, n_idx])
    los_errors = error[t_idx, n_idx][clear & heard]
    assert len(los_errors) > 100
    assert np.abs(los_errors).max() < 6 * K.UWB_LOS_NOISE_STD_M
    assert abs(np.mean(los_errors)) < 0.02


def test_recording_the_tag_does_not_change_the_run():
    """R-OBS-4 with the mixture model's draws in the suite."""
    captured: dict[str, list] = {}

    @register_policy("_captures_uwb")
    class Captures(Policy):
        def step(self, obs):
            captured.setdefault(self.config["run"], []).append(obs.sensors["uwb"].ranges_m.tolist())
            return Velocity(vx=0.3, vz=0.2)

    def config(label, record):
        return RunConfig(seed=2, policy="_captures_uwb", n_drones=10, duration_s=3.0, record=record,
                         policy_config={"run": label}, arena_config=ArenaConfig(landmarks=START_ANCHORS),
                         sensors=flown_sensors() + (UWBConfig(),))

    with tempfile.TemporaryDirectory() as tmp:
        run(config("loud", True), recorder=Recorder(tmp))
    run(config("quiet", False))
    assert captured["loud"] == captured["quiet"]


def test_record_static_before_the_first_sample_is_a_contract_violation():
    with pytest.raises(ConfigError, match="before the first sample"):
        make_tag().record_static()


# -- peers: the same tag, ranging to the fleet (R-SENS-18) ---------------------------------------


def _fleet(world, *drones, key=0):
    """``drones`` are ``(agent_id, object_id, x, y, z)``; fills the scene's fleet."""
    world.refresh_fleet(
        [(a, oid, np.array([[x], [y], [0.0], [z], [0.0], [0.0]])) for a, oid, x, y, z in drones],
        cache_key=key,
    )
    return world


def test_peers_are_off_by_default_and_the_reading_is_then_what_it_was():
    cfg = UWBConfig()
    assert cfg.peers is False
    reading = make_tag(**EXACT).sample(truth_at(10.0, 10.0), _fleet(box_scene(), ("drone_00", -1, 10, 10, 0.5)), 0)
    assert reading.peer_ids == () and reading.peer_ranges_m.shape == (0,)
    assert reading.peers_heard.shape == (0,)
    # The positional constructor the R-POL-4 walk uses still works: the new fields default.
    UWBRanges(("a",), read_only_xyz := np.zeros((1, 3)), np.zeros(1))
    assert read_only_xyz.shape == (1, 3)


def test_peers_must_be_a_bool():
    for bad in (1, "yes", None):
        with pytest.raises(ConfigError, match="peers must be a bool"):
            UWBConfig(peers=bad)
    UWBConfig(peers=np.bool_(True))


def test_a_peer_range_is_three_dimensional_between_the_two_tags_and_self_is_inf():
    world = _fleet(box_scene(),
                   ("drone_00", 1, 10.0, 10.0, 0.5),      # me
                   ("drone_01", 2, 10.8, 10.0, 0.0),      # a landed teammate 0.8 m away
                   ("drone_02", 3, 10.0, 13.0, 0.5))      # a flying one 3 m away
    tag = make_tag(peers=True, **EXACT)
    reading = tag.sample(TrueState("drone_00", 1, 10.0, 10.0, 0.5, 0.0, 0.0, 0.0), world, 0)
    assert reading.peer_ids == ("drone_00", "drone_01", "drone_02"), "run order, self included"
    assert np.isinf(reading.peer_ranges_m[0]) and not reading.peers_heard[0], "a tag cannot range to itself"
    assert reading.peer_ranges_m[1] == pytest.approx(np.hypot(0.8, 0.5)), "0.8 m across, 0.5 m down: 0.94 m"
    assert reading.peer_ranges_m[2] == pytest.approx(3.0)
    assert reading.peers_heard.tolist() == [False, True, True]
    check_reading_is_immutable("uwb", reading)
    with pytest.raises(ValueError, match="read-only"):
        reading.peer_ranges_m[:] = 0.0


def test_a_tag_the_fleet_has_never_heard_of_has_no_self_slot():
    """A hand-built scene whose fleet omits the sampling drone: every slot is a real peer."""
    world = _fleet(box_scene(), ("drone_07", 7, 12.0, 10.0, 0.5))
    reading = make_tag(peers=True, **EXACT).sample(truth_at(10.0, 10.0), world, 0)   # object_id -1
    assert reading.peer_ids == ("drone_07",) and reading.peer_ranges_m[0] == pytest.approx(2.0)


def test_a_wall_obstructs_a_peer_and_an_airframe_does_not():
    behind = ("drone_01", 2, 14.0, 10.0, 0.5)
    beyond_a_body = ("drone_02", 3, 10.0, 14.0, 0.5)
    body = ("drone_03", 4, 10.0, 12.0, 0.5)                      # right on the line to drone_02
    world = _fleet(box_scene(walls=[(12.0, 5.0, 12.0, 15.0)]), ("drone_00", 1, 10, 10, 0.5),
                   behind, beyond_a_body, body)
    world.refresh_drones([_FakeRobot(4, 10.0, 12.0)], cache_key=0)  # and it is a ring body too
    tag = make_tag(peers=True, nlos_bias_m=0.5, los_noise_std_m=0.0, nlos_noise_std_m=0.0,
                   nlos_drop_probability=0.0)
    reading = tag.sample(TrueState("drone_00", 1, 10.0, 10.0, 0.5, 0.0, 0.0, 0.0), world, 0)
    assert reading.peer_ranges_m[1] == pytest.approx(4.0 + 0.5), "behind the wall: biased"
    assert reading.peer_ranges_m[2] == pytest.approx(4.0), "behind a teammate's body: clean"
    assert reading.peer_ranges_m[3] == pytest.approx(2.0)


def test_peer_line_of_sight_is_tested_at_the_lower_of_the_two_altitudes():
    """A 1.0 m wall between a drone at 1.2 m and one on the floor: obstructed, because the
    landed tag is below the wall's top. Between two drones at 1.2 m: clear. F-34."""
    scene = box_scene(walls=[(12.0, 5.0, 12.0, 15.0)]).structural_scene
    heights = scene.segment_heights.copy()
    heights[-1] = 1.0
    low = RayScene(segments=scene.segments, segment_heights=heights)
    me_high = np.array([10.0, 10.0, 1.2])
    peers = np.array([[14.0, 10.0, 0.0], [14.0, 10.0, 1.2], [14.0, 10.0, 0.5]])
    assert peer_line_of_sight(low, me_high, peers).tolist() == [False, True, False]
    me_low = np.array([10.0, 10.0, 0.0])
    assert peer_line_of_sight(low, me_low, peers).tolist() == [False, False, False]
    assert peer_line_of_sight(low, me_low, np.zeros((0, 3))).shape == (0,)


def test_a_peer_beyond_reach_is_inf_and_a_dropped_one_is_inf():
    world = _fleet(box_scene(), ("drone_00", 1, 1.0, 1.0, 0.5), ("drone_01", 2, 19.0, 19.0, 0.5))
    reading = make_tag(peers=True, max_range_m=5.0, **EXACT).sample(
        TrueState("drone_00", 1, 1.0, 1.0, 0.5, 0.0, 0.0, 0.0), world, 0)
    assert np.isinf(reading.peer_ranges_m).all()
    walled = _fleet(box_scene(walls=[(12.0, 5.0, 12.0, 15.0)]),
                    ("drone_00", 1, 10, 10, 0.5), ("drone_01", 2, 14, 10, 0.5))
    always_drops = make_tag(peers=True, nlos_drop_probability=1.0, los_noise_std_m=0.0)
    reading = always_drops.sample(TrueState("drone_00", 1, 10.0, 10.0, 0.5, 0.0, 0.0, 0.0), walled, 0)
    assert np.isinf(reading.peer_ranges_m[1])


def test_switching_peers_on_leaves_the_anchor_stream_untouched_and_draws_four_per_peer():
    """R-SENS-18: peers draw from a child generator spawned at build, so the anchor noise is
    the same run with peers on or off -- for every sweep, not just the first -- and there are
    four draws per peer per sweep whatever the peer's reach, lifecycle or identity."""
    anchors = (Landmark("a", "uwb_anchor", 14.0, 10.0),)
    off = _fleet(box_scene(landmarks=anchors), ("drone_00", 1, 10, 10, 0.5), ("drone_01", 2, 11, 10, 0.5))
    on = _fleet(box_scene(landmarks=anchors), ("drone_00", 1, 10, 10, 0.5), ("drone_01", 2, 11, 10, 0.5))
    me = TrueState("drone_00", 1, 10.0, 10.0, 0.5, 0.0, 0.0, 0.0)
    tag_off, tag_on = make_tag(seed=4), make_tag(seed=4, peers=True)
    for tick in range(5):
        a = tag_off.sample(me, off, tick)
        b = tag_on.sample(me, on, tick)
        assert a.ranges_m.tolist() == b.ranges_m.tolist(), "anchor noise does not depend on peers"
    assert tag_off.rng.random() == tag_on.rng.random(), "the parent stream is untouched by peers"
    # Exactly four draws per fleet member per sweep from the child: a fresh child of the
    # same parent, advanced by 5 sweeps * 4 draws * 2 drones, is in lockstep.
    child = np.random.default_rng(4).spawn(1)[0]
    child.random(5 * 4 * 2)
    assert child.random() == tag_on._peer_rng.random()

    # And the count does not depend on geometry or lifecycle: one peer in reach and flying,
    # the other out of reach and on the floor, same number of draws.
    near = _fleet(box_scene(), ("drone_00", 1, 10, 10, 0.5), ("drone_01", 2, 11, 10, 0.5), ("drone_02", 3, 12, 10, 0.5))
    far = _fleet(box_scene(), ("drone_00", 1, 10, 10, 0.5), ("drone_01", 2, 19, 19, 0.0), ("drone_02", 3, 0.5, 19, 0.0))
    t_near, t_far = make_tag(seed=8, peers=True, max_range_m=5.0), make_tag(seed=8, peers=True, max_range_m=5.0)
    for tick in range(3):
        assert t_near.sample(me, near, tick).peers_heard.sum() == 2
        assert t_far.sample(me, far, tick).peers_heard.sum() == 0
    # The CHILD generators, which is where the peer draws come from: an auditor showed a
    # mutant drawing extra child values per out-of-reach peer passed a parent comparison.
    assert t_near._peer_rng.random() == t_far._peer_rng.random()
    assert t_near.rng.random() == t_far.rng.random()


def test_the_peer_stream_depends_on_the_fleet_size_and_the_anchor_stream_does_not():
    """R-SENS-18 says so out loud: four draws per fleet member per sweep, so adding a drone
    moves every tag's peer noise from the first sweep on. Not a defect -- a fact a comparison
    across fleet sizes must not mistake for behaviour."""
    def first_draws(n):
        runner = Runner(RunConfig(seed=5, policy="sdlw", n_drones=n, duration_s=2.0, record=False,
                                  arena_config=ArenaConfig(landmarks=START_ANCHORS),
                                  sensors=(ToFConfig(), UWBConfig(peers=True)))).build()
        try:
            tag = runner.agents[0].sensors[1]
            return tag.rng.random(), tag._peer_rng.random()
        finally:
            runner._teardown()
    ten, eleven = first_draws(10), first_draws(11)
    assert ten[0] == eleven[0], "the anchor stream of drone_00 is untouched by an eleventh drone"
    assert ten[1] != eleven[1], "its peer stream is not"


def test_the_peer_sweep_rate_grows_the_slot_with_the_fleet_and_the_anchors():
    """A-19: eight exchanges per 10 ms slot, every tag initiating to every anchor and every
    other tag. Ten drones and one anchor need two slots; twenty-five need four."""
    assert peer_sweep_rate_hz(10, 1) == pytest.approx(5.0)
    assert peer_sweep_rate_hz(25, 1) == pytest.approx(1.0)
    assert peer_sweep_rate_hz(20, 1) == pytest.approx(1.0 / (20 * 0.030))
    assert peer_sweep_rate_hz(10, 0) == pytest.approx(1.0 / (10 * 0.020))          # 9 peers: 2 slots
    assert peer_sweep_rate_hz(8, 1) == pytest.approx(sweep_rate_hz(8)), "eight exchanges fit one slot"
    assert peer_sweep_rate_hz(1, 3) == pytest.approx(sweep_rate_hz(1)), "alone, it is the anchor rate"
    # A better protocol is one argument away, and so is a slower radio.
    assert peer_sweep_rate_hz(25, 1, exchanges_per_slot=25) == pytest.approx(4.0)
    assert peer_sweep_rate_hz(10, 1, slot_s=0.015) == pytest.approx(1.0 / (10 * 0.030))
    # On the 20 Hz loop the decimation is 0.2 * n_tags * slots: ten, fifteen, twenty and
    # twenty-five drones with one anchor divide; twelve (4.8) do not, and the runner refuses
    # rather than rounds. (A first draft of the docstring said fifteen do not. They do.)
    for n in (10, 15, 20, 25):
        RunConfig(n_drones=n, sensors=(UWBConfig(peers=True, rate_hz=peer_sweep_rate_hz(n, 1)),))
    with pytest.raises(ConfigError, match="does not divide"):
        RunConfig(n_drones=12, sensors=(UWBConfig(peers=True, rate_hz=peer_sweep_rate_hz(12, 1)),))
    for bad in ((0, 1), (10, -1), (2.5, 1), (True, 1)):
        with pytest.raises(ConfigError):
            peer_sweep_rate_hz(*bad)
    with pytest.raises(ConfigError, match="exchanges_per_slot"):
        peer_sweep_rate_hz(10, 1, exchanges_per_slot=0)


def test_peer_ranges_reach_the_policy_and_see_a_landed_teammate_at_floor_level():
    """End to end: drone_00 lands at tick 10; every other tag then ranges to it at z = 0 and
    the range shrinks by the altitude difference, not to inf."""
    seen: dict[int, list] = {}

    @register_policy("_ranges_to_peers")
    class RangesToPeers(Policy):
        def step(self, obs):
            r = obs.sensors["uwb"]
            assert r.peer_ids == tuple(f"drone_{i:02d}" for i in range(10))
            assert np.isinf(r.peer_ranges_m[int(obs.agent_id[-2:])]), "self is inf"
            if obs.agent_id == "drone_01" and obs.stale_ticks["uwb"] == 0:
                seen[obs.tick] = (obs.pose.z, float(r.peer_ranges_m[0]))
            if obs.agent_id == "drone_00" and obs.tick == 10:
                return Land()
            return Velocity(vz=0.4)

    run(RunConfig(seed=0, policy="_ranges_to_peers", arena_config=ArenaConfig(landmarks=START_ANCHORS),
                  sensors=(ToFConfig(), UWBConfig(peers=True, los_noise_std_m=0.0)), **SHORT))
    # drone_00 and drone_01 are grid neighbours 1.25 m apart, both climbing in step, so the
    # range between them is 1.25 m until drone_00 lands; then it grows by the height gap.
    before = [r for t, (z, r) in seen.items() if t <= 10]
    after = [(z, r) for t, (z, r) in seen.items() if t >= 14]
    assert before and after
    assert all(r == pytest.approx(K.START_SPACING_M, abs=1e-6) for r in before)
    for z, r in after:
        assert np.isfinite(r), "a landed teammate's tag still answers"
        assert r == pytest.approx(np.hypot(K.START_SPACING_M, z), abs=1e-6)


def test_the_log_holds_peer_ranges_square_in_the_header_s_agent_order():
    cfg = RunConfig(seed=1, policy="sdlw", n_drones=10, duration_s=3.0,
                    arena_config=ArenaConfig(landmarks=START_ANCHORS),
                    sensors=flown_sensors() + (UWBConfig(peers=True),))
    with tempfile.TemporaryDirectory() as tmp:
        result = run(cfg, recorder=Recorder(tmp))
        log = load_run(tmp)
    uwb = log["sensors"]["uwb"]
    assert uwb["ranges_m"].shape == (result.ticks, 10, 3)
    assert uwb["peer_ranges_m"].shape == (result.ticks, 10, 10)
    assert log["header"]["agents"] == [f"drone_{i:02d}" for i in range(10)]
    # The diagonal is self and is always inf; the rest is graded against the true geometry.
    diag = np.einsum("tii->ti", uwb["peer_ranges_m"])
    assert np.isinf(diag).all()
    fresh = uwb["sample_tick"] == uwb["ticks"][:, None]
    pose = log["states"]["pose"][:, :, :3]                                   # (T, N, 3)
    d = np.linalg.norm(pose[:, :, None, :] - pose[:, None, :, :], axis=-1)   # (T, N, N)
    err = uwb["peer_ranges_m"] - d
    t_idx, n_idx = np.nonzero(fresh)
    rows = err[t_idx, n_idx]                                                 # (K, N)
    heard = np.isfinite(uwb["peer_ranges_m"][t_idx, n_idx])
    # At take-off nothing stands between grid neighbours, so every heard range is a LOS one.
    assert heard.sum() > 500
    assert np.abs(rows[heard]).max() < 6 * K.UWB_LOS_NOISE_STD_M
    assert abs(rows[heard].mean()) < 0.02


def test_a_run_without_peers_writes_the_log_it_always_did():
    """R-SENS-18: with peers off, byte-for-byte what a run before the option produced. The
    file cannot be compared against history here, so this pins the schema and the draws:
    the same anchor ranges, and no peer array."""
    def log_of(peers):
        cfg = RunConfig(seed=2, policy="sdlw", n_drones=10, duration_s=2.0,
                        arena_config=ArenaConfig(landmarks=START_ANCHORS),
                        sensors=flown_sensors() + (UWBConfig(peers=peers),))
        with tempfile.TemporaryDirectory() as tmp:
            run(cfg, recorder=Recorder(tmp))
            return load_run(tmp)["sensors"]["uwb"]
    off, on = log_of(False), log_of(True)
    assert set(off) == {"ticks", "sample_tick", "ranges_m", "anchor_xyz_m"}
    assert "peer_ranges_m" in on
    assert np.array_equal(off["ranges_m"], on["ranges_m"]), "anchor noise does not depend on peers"


# -- the example and the rulebook ----------------------------------------------------------------


@functools.lru_cache(maxsize=1)
def _load_example():
    # Cached: executing the module twice re-registers its policy, which warns by design.
    path = Path(__file__).resolve().parent.parent / "examples" / "04_uwb_ranging.py"
    spec = importlib.util.spec_from_file_location("example_uwb", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_example_places_a_legal_anchor_set_and_runs():
    example = _load_example()
    cfg = example.make_config(seed=0, duration_s=2.0)
    arena = generate_arena(cfg.seed, cfg.arena_config)
    validate_nav_aids(arena, ("uwb_anchor",))                   # R-WORLD-11: the rules allow it
    n_known = sum(1 for lm in arena.landmarks if arena.in_known_area(lm.x, lm.y))
    assert 0 < n_known <= K.NAV_AID_MAX_KNOWN_AREA
    result = run(dataclasses.replace(cfg, record=False))
    assert result.ticks == 40


def test_the_example_s_fixed_anchors_clear_the_room_on_every_seed():
    """The room moves with the seed, so a fixed coordinate is a trap, and the maze on main
    made it a sharper one. The example hugs the field edges to stay legal on every seed;
    this is what says so."""
    example = _load_example()
    arena_config = example.make_config().arena_config
    for seed in range(30):
        arena = generate_arena(seed, arena_config)
        validate_arena(arena)                    # refuses anything inside the room (r.17)
        validate_nav_aids(arena, ("uwb_anchor",))
        assert not any(arena.in_unknown_area(lm.x, lm.y) for lm in arena.landmarks)
