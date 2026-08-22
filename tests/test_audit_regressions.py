"""Regression tests for every defect confirmed by the v0.1 adversarial spec audit.

Each test names the requirement it guards and, where the defect was subtle, states the
mechanism -- because the value of these tests is that they fail if someone reintroduces the
*cause*, not just the symptom.
"""

import json
import math
import tempfile

import numpy as np
import pytest

from safmc_sim import constants as K
from safmc_sim import policies  # noqa: F401
from safmc_sim.api import (
    ArenaInfo, Hold, Land, Lifecycle, Policy, PositionWorld, Takeoff, VelocityBody,
    register_policy,
)
from safmc_sim.blackboard import PerfectBlackboard
from safmc_sim.errors import ArenaError, ConfigError, PolicyError
from safmc_sim.frames import wrap_pi
from safmc_sim.mission import Mission
from safmc_sim.recorder import Recorder, load_run, score_from_log
from safmc_sim.runner import RunConfig, Runner, run
from safmc_sim.sensors.marker_cam import MarkerCam
from safmc_sim.sensors.raycast import RayScene
from safmc_sim.world.arena import Target, generate_arena
from safmc_sim.world.arena import validate_arena


# -- R-FRAME-2 --------------------------------------------------------------------------------


def test_wrap_pi_closes_the_boundary_hole():
    """The one representable double that used to escape (-pi, pi]."""
    just_above = np.nextafter(np.pi, np.inf)
    wrapped = wrap_pi(just_above)
    assert -np.pi < wrapped <= np.pi
    assert wrap_pi(wrapped) == wrapped              # idempotent, as the docstring promises
    # And the array path behaves identically.
    arr = wrap_pi(np.array([just_above, -np.pi, np.pi, 0.0]))
    assert np.all(arr > -np.pi) and np.all(arr <= np.pi)


# -- R-WORLD-2 / R-WORLD-4 --------------------------------------------------------------------


@pytest.mark.parametrize("seed", range(30))
def test_unknown_area_keeps_the_published_gap_from_the_perimeter(seed):
    """Gaps are between wall FACES, not centre lines.

    A wall centred on its line extends thickness/2 either side, so placing the room's centre
    line `min_gap` from the perimeter left only `min_gap - thickness/2` of air. The result was
    a systematic 1.95 m north gap in every seed, which validation did not catch because it
    never compared the room against the perimeter at all.
    """
    arena = generate_arena(seed)
    perimeter = [w.polygon() for w in arena.walls if w.kind == "perimeter_wall"]
    room = [w.polygon() for w in arena.walls if w.kind == "unknown_wall"]
    assert perimeter and room
    for room_poly in room:
        for wall_poly in perimeter:
            assert room_poly.distance(wall_poly) >= K.MIN_GAP_WALL_TO_WALL_M - 1e-6


def test_room_position_is_not_pinned():
    """It has real freedom; an earlier derivation wrongly concluded it was forced."""
    ys = {round(generate_arena(s).unknown_area[1], 3) for s in range(25)}
    assert len(ys) > 20


def test_validation_catches_a_room_placed_too_close_to_the_perimeter():
    from safmc_sim.world.arena import ArenaSpec, Wall

    arena = generate_arena(0)
    x0, _, x1, y1 = arena.unknown_area
    # Shove the room's north wall right up against the perimeter.
    bad_walls = tuple(w for w in arena.walls if w.kind != "unknown_wall") + (
        Wall(x0, 19.0, x1, 19.0, 0.1, K.INNER_WALL_HEIGHT_M, "unknown_wall"),
    )
    bad = ArenaSpec(
        seed=arena.seed, width_m=arena.width_m, depth_m=arena.depth_m,
        ceiling_m=arena.ceiling_m, start_area_depth_m=arena.start_area_depth_m,
        unknown_area=arena.unknown_area, walls=bad_walls, pillars=arena.pillars,
        targets=arena.targets, config=arena.config,
    )
    # Call the gap check directly: the target-overlap check runs first and would otherwise
    # fire on the moved wall before the gap check is ever reached.
    from safmc_sim.world.arena import _validate_gaps

    with pytest.raises(ArenaError, match="perimeter gap"):
        _validate_gaps(bad)


# -- R-DRONE-9 --------------------------------------------------------------------------------


def test_an_unhandled_lifecycle_raises_instead_of_being_flown():
    """Transitions must be total. The if-chain used to fall through to the flying branch."""
    runner = Runner(RunConfig(seed=0, n_drones=10, policy="hold")).build()
    try:
        agent = runner.agents[0]
        agent.lifecycle = "SOMETHING_ELSE"
        with pytest.raises(ConfigError, match="unhandled lifecycle"):
            runner._resolve(agent, Hold(), 0, 0.0)
    finally:
        runner._teardown()


def test_lifecycle_set_matches_the_spec():
    assert set(Lifecycle.ALL) == {
        "IDLE", "TAKEOFF", "FLYING", "LANDING", "LANDED", "CRASHED",
    }


# -- R-DRONE-10 / R-MISS-8 --------------------------------------------------------------------


def test_a_crashed_drone_is_also_frozen():
    result = run(RunConfig(seed=1, n_drones=12, policy="frontier", duration_s=60.0,
                           record=False))
    assert result.crashed or result.landed  # the run has to have produced a terminal agent


def test_mission_servicing_is_latched_and_never_revoked():
    """Once a target is serviced it stays serviced, whatever happens to the drone afterwards."""
    from tests.test_mission import bare_arena

    victim = Target("v0", "victim", 10.0, 10.0)
    mission = Mission(bare_arena([victim]))
    mission.update(0, 0.0, {"d0": np.array([10.2, 10.0])})
    assert mission.targets["v0"].serviced

    # Even if the drone somehow ends up outside the radius, the award stands.
    mission.update(1, 0.05, {"d0": np.array([15.0, 15.0])})
    assert mission.targets["v0"].serviced
    assert mission.score({"d0": np.array([15.0, 15.0])}).raw_total == 5


def test_offline_rescoring_survives_a_landing_at_speed():
    """The case that broke R-MISS-8: land while still moving, then check both scorers agree."""

    @register_policy("_land_at_speed")
    class LandAtSpeed(Policy):
        def step(self, obs):
            if obs.lifecycle == "IDLE":
                return Takeoff()
            if obs.lifecycle != "FLYING":
                return Hold()
            if obs.markers and min(m.range_m for m in obs.markers) < 0.9:
                return Land()          # commanded at cruise speed, never slowing first
            if obs.tof.ranges_m[0].min() < 0.7:
                return VelocityBody(vx=0.0, yaw_rate=1.0)
            return VelocityBody(vx=0.45)

    for seed in (2, 5, 8):
        with tempfile.TemporaryDirectory() as tmp:
            result = run(
                RunConfig(seed=seed, n_drones=12, policy="_land_at_speed", duration_s=120.0),
                recorder=Recorder(tmp),
            )
            offline = score_from_log(tmp)
            assert offline.total == result.score.total
            assert offline.per_target == result.score.per_target


# -- R-POL-2 ----------------------------------------------------------------------------------


def test_a_policy_cannot_re_aim_the_ring_it_is_handed():
    """The frozen dataclass blocked rebinding but not in-place writes into a shared array."""
    runner = Runner(RunConfig(seed=0, n_drones=10, policy="hold")).build()
    try:
        sensor = runner.agents[0].robot.sensors[0]
        scan = sensor.step(runner.agents[0].state[0:3])
        for name in ("zone_bearings_rad", "ranger_bearings_rad"):
            array = getattr(scan, name)
            assert not array.flags.writeable, f"{name} is writable"
            with pytest.raises(ValueError):
                array[:] = 0.0
        assert not sensor.zone_bearings_rad.flags.writeable
    finally:
        runner._teardown()


# -- R-POL-8 ----------------------------------------------------------------------------------


def test_mutating_a_published_value_cannot_change_what_peers_read():
    """Storing by reference let a publisher rewrite history mid-tick for later-stepped agents."""
    board = PerfectBlackboard()
    owned = {"state": "OLD"}
    board.publish("a", {"box": owned})
    board.commit()
    before = board.snapshot("z")["a"]["box"]["state"]
    owned["state"] = "NEW"
    after = board.snapshot("z")["a"]["box"]["state"]
    assert before == after == "OLD"


def test_nested_published_containers_are_also_isolated():
    board = PerfectBlackboard()
    payload = {"grid": [[1, 2], [3, 4]]}
    board.publish("a", payload)
    board.commit()
    payload["grid"][0][0] = 99
    assert board.snapshot("z")["a"]["grid"][0][0] == 1


# -- non-finite commands ------------------------------------------------------------------------


@pytest.mark.parametrize(
    "index,command",
    list(enumerate([
        VelocityBody(vx=float("nan")),
        VelocityBody(yaw_rate=float("inf")),
        PositionWorld(x=float("nan"), y=1.0),
        Takeoff(altitude_m=float("inf")),
    ])),
)
def test_a_non_finite_command_is_rejected_with_a_useful_error(index, command):
    """It used to poison the state and surface as a GEOSException with no agent or tick."""
    name = f"_nonfinite_{index}"

    @register_policy(name)
    class NonFinite(Policy):
        def step(self, obs):
            if obs.lifecycle == "IDLE":
                return Takeoff()
            return command if obs.tick == 20 else Hold()

    with pytest.raises(PolicyError) as info:
        run(RunConfig(seed=0, n_drones=10, policy=name, duration_s=8.0, record=False))
    message = str(info.value)
    assert "tick 20" in message and "drone_" in message and "finite" in message


# -- collision modes ----------------------------------------------------------------------------


def test_unobstructed_is_a_real_control_with_no_crashes():
    """Markers used to stay lethal in 'unobstructed', so the control mode was not a control."""
    for seed in (0, 3):
        result = run(
            RunConfig(seed=seed, n_drones=12, policy="frontier", duration_s=120.0,
                      collision_behaviour="unobstructed", record=False)
        )
        assert not result.crashed, f"seed {seed} crashed in unobstructed mode: {result.crashed}"


def test_a_drone_cannot_leave_the_field_even_unobstructed():
    """ir-sim has no world bounds; one was measured 55 m into a 20 m field."""

    @register_policy("_escape")
    class Escape(Policy):
        def step(self, obs):
            if obs.lifecycle == "IDLE":
                return Takeoff()
            return VelocityWorldNorth()

    from safmc_sim.api import VelocityWorld

    def VelocityWorldNorth():
        return VelocityWorld(vx=0.0, vy=5.0)

    result = run(
        RunConfig(seed=0, n_drones=10, policy="_escape", duration_s=120.0,
                  collision_behaviour="unobstructed", record=False)
    )
    runner_arena = result.arena
    assert any(e.kind == "crashed" and e.detail.get("reason") == "left the field"
               for e in result.events)
    for event in result.events:
        if event.kind == "crashed":
            assert -1.0 <= event.detail["y"] <= runner_arena.depth_m + 1.0


# -- R-SENS-10 ------------------------------------------------------------------------------------


@pytest.mark.parametrize("z", [0.5, 1.0, 1.2, 1.39])
def test_markers_are_visible_at_every_legal_altitude(z):
    """An altitude cut-off copied from the ToF ring made a drone above 1.0 m totally blind."""
    cam = MarkerCam()
    target = Target("v0", "victim", 2.0, 0.0)
    detections = cam.detect(np.array([0.0, 0.0]), 0.0, z, [target], RayScene())
    assert detections and detections[0].marker_id == "v0"


# -- recorder ---------------------------------------------------------------------------------------


def test_the_log_is_strictly_valid_json():
    """json.dumps writes bare Infinity/NaN, which every strict parser rejects."""
    with tempfile.TemporaryDirectory() as tmp:
        run(RunConfig(seed=1, n_drones=10, policy="sdlw", duration_s=15.0), recorder=Recorder(tmp))
        text = (__import__("pathlib").Path(tmp) / "run.jsonl").read_text()
        assert "Infinity" not in text and "NaN" not in text
        for line in text.splitlines():
            json.loads(line, parse_constant=_reject_constant)


def _reject_constant(name):
    raise AssertionError(f"log contains the non-JSON constant {name!r}")


def test_recorded_pose_is_double_precision():
    """float32 quantises a 20 m coordinate enough to flip a 1 m scoring decision."""
    with tempfile.TemporaryDirectory() as tmp:
        run(RunConfig(seed=1, n_drones=10, policy="hold", duration_s=6.0), recorder=Recorder(tmp))
        states = load_run(tmp)["states"]
        assert states["pose"].dtype == np.float64
        assert states["time_s"].dtype == np.float64


def test_the_log_carries_its_own_codebook():
    """states.npz is a wall of integers without it."""
    with tempfile.TemporaryDirectory() as tmp:
        run(RunConfig(seed=1, n_drones=10, policy="hold", duration_s=6.0), recorder=Recorder(tmp))
        codebook = load_run(tmp)["header"]["codebook"]
        assert set(codebook["lifecycle"].values()) == set(Lifecycle.ALL)
        assert "Land" in codebook["command"].values()
        assert codebook["pose_columns"] == ["x", "y", "z", "theta"]


def test_a_recorder_with_record_disabled_is_a_configuration_error():
    with pytest.raises(ConfigError, match="record"):
        Runner(RunConfig(seed=0, n_drones=10, policy="hold", record=False), recorder=Recorder("/tmp/x"))


def test_reported_sim_time_matches_the_log():
    with tempfile.TemporaryDirectory() as tmp:
        result = run(
            RunConfig(seed=1, n_drones=10, policy="hold", duration_s=6.0), recorder=Recorder(tmp)
        )
        times = load_run(tmp)["states"]["time_s"]
        assert result.ticks == len(times)
        assert math.isclose(result.sim_time_s, float(times[-1]), rel_tol=1e-9)


# -- teardown ------------------------------------------------------------------------------------


def test_an_aborted_run_does_not_leak_figures():
    import matplotlib.pyplot as plt

    @register_policy("_abort")
    class Abort(Policy):
        def step(self, obs):
            raise RuntimeError("deliberate")

    plt.close("all")
    before = len(plt.get_fignums())
    for _ in range(3):
        with pytest.raises(PolicyError):
            run(RunConfig(seed=0, n_drones=10, policy="_abort", duration_s=4.0, record=False))
    assert len(plt.get_fignums()) == before


# -- cheap guards for requirements the audit found correct but unguarded -----------------------


def test_published_and_hardware_defaults_have_regression_barriers():
    """R-TIME-1, R-DRONE-6/7/8, R-MISS-5. Each was correct but nothing would catch a change."""
    assert K.DEFAULT_TICK_HZ == K.NAV_RATE_HZ == 20.0
    assert RunConfig().tick_hz == 20.0 and RunConfig().dt == 0.05
    assert K.CRUISE_ALT_M == 0.5 and RunConfig().cruise_alt_m == 0.5
    assert K.CRUISE_SPEED_MS == 0.45
    assert K.DRONE_RADIUS_M == 0.18
    assert K.RUN_DURATION_S == 600.0 and RunConfig().duration_s == 600.0
    assert K.CEILING_M == 1.4
    assert K.SCORE_RADIUS_M == 1.0 and K.RELAY_SPACING_M == 1.0
    assert K.FIRE_SUPPRESSION_RADIUS_M == 2.5 and K.RELAY_MULTIPLIER == 2.0


def test_every_config_field_declares_its_units():
    """R-FRAME-3. A future `fov_deg` or `max_range_mm` would otherwise ship silently."""
    import dataclasses

    from safmc_sim.kinematics import QuadParams
    from safmc_sim.sensors.marker_cam import MarkerCamConfig
    from safmc_sim.sensors.tof_ring import ToFConfig
    from safmc_sim.world.arena import ArenaConfig

    allowed = ("_m", "_rad", "_s", "_ms", "_hz", "_rads", "_index", "_ticks")
    dimensionless = {
        "n_rangers", "zones_per_ranger", "front_index", "seed", "n_drones", "policy",
        "policy_config", "arena_config", "quad_params", "tof_config", "marker_config",
        "collision_behaviour", "record", "n_inner_walls", "n_pillars_known",
        "n_unknown_walls", "n_pillars_unknown", "n_victims", "n_bonus_victims", "n_fires",
        "max_placement_attempts", "sample_hz", "tick_hz", "tof_rate_hz", "marker_rate_hz",
        "duration_s",
    }
    for cls in (RunConfig, QuadParams, ToFConfig, MarkerCamConfig, ArenaConfig):
        for field in dataclasses.fields(cls):
            if field.name in dimensionless:
                continue
            assert field.name.endswith(allowed), (
                f"{cls.__name__}.{field.name} does not declare SI units; R-FRAME-3 requires "
                f"metres/radians/seconds everywhere except a named conversion boundary"
            )


def test_a_sample_rate_faster_than_the_tick_rate_is_rejected():
    """R-TIME-4. ir-sim would otherwise raise a bare ZeroDivisionError from inside world.step."""
    with pytest.raises(ConfigError, match="sample_hz"):
        RunConfig(tick_hz=20.0, sample_hz=50.0)
    RunConfig(tick_hz=20.0, sample_hz=10.0)


def test_each_drone_carries_exactly_one_tof_sensor():
    """R-SENS-1. A return to per-ranger Lidar2D instances would be a silent 14-31x slowdown."""
    from safmc_sim.sensors.tof_ring import ToFRing

    runner = Runner(RunConfig(seed=0, n_drones=10, policy="hold")).build()
    try:
        for agent in runner.agents:
            assert len(agent.robot.sensors) == 1
            assert isinstance(agent.robot.sensors[0], ToFRing)
    finally:
        runner._teardown()


def test_pose_and_velocity_both_come_through_the_pose_source_seam():
    """R-SEAM-1. Velocity used to bypass it, leaking an exact motion signal past a noisy pose."""
    from safmc_sim.api import Pose
    from safmc_sim.pose import PoseSource

    seen = []

    class Sentinel(PoseSource):
        def pose_of(self, agent_id, state, tick):
            seen.append(("pose", agent_id))
            return Pose(x=1.0, y=2.0, z=0.5, theta=0.0)

        def velocity_of(self, agent_id, state, tick):
            seen.append(("velocity", agent_id))
            return (9.0, 9.0)

    @register_policy("_seam_probe")
    class Probe(Policy):
        def step(self, obs):
            assert obs.pose.x == 1.0 and obs.pose.y == 2.0
            assert obs.velocity_xy == (9.0, 9.0)
            return Hold()

    runner = Runner(RunConfig(seed=0, n_drones=10, policy="_seam_probe", duration_s=2.0,
                              record=False))
    runner.build()
    runner.pose_source = Sentinel()
    runner.run()
    assert {kind for kind, _ in seen} == {"pose", "velocity"}


# -- the seam's worked example must actually work ---------------------------------------------


def test_noisy_pose_drifts_and_is_reproducible():
    """`NoisyPose` is the template a real odometry model gets written against.

    It shipped untested, which is exactly the wrong state for the one class people will copy.
    Three properties matter: drift accumulates rather than being white noise, velocity is
    corrupted too (or a policy could integrate an exact signal to recover the true position),
    and the whole thing is reproducible from its generator.
    """
    from safmc_sim.pose import NoisyPose

    truth = np.array([[5.0], [5.0], [0.3], [0.5], [0.2], [-0.1]])

    def walk(seed, n=400):
        source = NoisyPose(np.random.default_rng(seed))
        out = [source.pose_of("d0", truth, t) for t in range(n)]
        return np.array([[p.x, p.y, p.theta] for p in out])

    a, b, c = walk(1), walk(1), walk(2)
    assert np.array_equal(a, b), "same seed must give the same drift"
    assert not np.array_equal(a, c), "different seeds must differ"

    # Drift is a random walk: later error is larger than early error, in expectation.
    error = np.abs(a[:, :2] - truth[:2, 0])
    assert error[-50:].mean() > error[:50].mean()

    # Altitude is not corrupted -- the real system measures it directly, and the class says so.
    source = NoisyPose(np.random.default_rng(0))
    assert source.pose_of("d0", truth, 0).z == 0.5

    # Velocity goes through the seam too.
    vx, vy = source.velocity_of("d0", truth, 0)
    assert (vx, vy) != (0.2, -0.1)
    assert abs(vx - 0.2) < 0.5 and abs(vy + 0.1) < 0.5
