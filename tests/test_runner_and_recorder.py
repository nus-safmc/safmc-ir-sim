"""R-TIME-3, R-DET-1, R-MISS-6..8, R-OBS-1..4, R-POL-9: the run loop and its log."""

import filecmp
import json
import tempfile
from pathlib import Path

import numpy as np
import pytest

from safmc_sim import constants as K
from safmc_sim import policies  # noqa: F401 -- registers built-ins
from safmc_sim.api import Land, Policy, Velocity, register_policy
from safmc_sim.errors import ConfigError, LogFormatError, PolicyError
from safmc_sim.recorder import Recorder, load_run, score_from_log
from safmc_sim.runner import RunConfig, Runner, run

SHORT = dict(n_drones=10, duration_s=8.0, record=False)

CRUISE_M = 0.5


def _climb_then(fly):
    """Build a policy that climbs to cruise, then defers to ``fly(self, obs)``.

    Every test that needs a flying drone now has to get it airborne itself, because the
    platform has no take-off command. That is the point of the change, but it is repetitive in
    tests, so it lives here once.
    """

    class Built(Policy):
        def step(self, obs):
            if obs.pose.z < CRUISE_M - 0.02:
                return Velocity(vz=0.4)
            return fly(self, obs)

    return Built


# -- configuration validation ---------------------------------------------------------------


@pytest.mark.parametrize("n", [0, 9, 26, 100])
def test_fleet_size_outside_the_published_range_is_rejected(n):
    """R-MISS-7: fewer than 10 drones forfeits the run."""
    with pytest.raises(ConfigError, match="n_drones"):
        RunConfig(n_drones=n)


def test_a_sensor_rate_that_does_not_divide_the_tick_rate_is_rejected():
    """R-TIME-3: no silent rounding of a sensor rate."""
    with pytest.raises(ConfigError, match="does not divide"):
        RunConfig(tick_hz=20.0, marker_rate_hz=3.0)
    RunConfig(tick_hz=20.0, marker_rate_hz=2.0)


def test_unknown_collision_behaviour_is_rejected():
    with pytest.raises(ConfigError, match="collision_behaviour"):
        RunConfig(collision_behaviour="bounce")


# -- determinism -----------------------------------------------------------------------------


def test_identical_inputs_give_identical_results():
    """R-DET-1."""
    a = run(RunConfig(seed=7, policy="sdlw", **SHORT))
    b = run(RunConfig(seed=7, policy="sdlw", **SHORT))
    assert a.score.total == b.score.total
    assert a.ticks == b.ticks
    assert [(e.tick, e.kind, e.agent_id) for e in a.events] == [
        (e.tick, e.kind, e.agent_id) for e in b.events
    ]
    assert a.lifecycles == b.lifecycles


def test_different_seeds_give_different_runs():
    a = run(RunConfig(seed=1, policy="sdlw", **SHORT))
    b = run(RunConfig(seed=2, policy="sdlw", **SHORT))
    assert (a.score.total, a.mission_summary) != (b.score.total, b.mission_summary)


def test_logs_of_identical_runs_differ_only_in_the_meta_block(tmp_path):
    """R-DET-1: wall-clock values are confined to one block."""
    for name in ("a", "b"):
        run(
            RunConfig(seed=11, policy="sdlw", n_drones=10, duration_s=8.0),
            recorder=Recorder(tmp_path / name),
        )
    assert filecmp.cmp(tmp_path / "a" / "states.npz", tmp_path / "b" / "states.npz", shallow=False)

    def strip_meta(path):
        lines = []
        for line in (path / "run.jsonl").read_text().splitlines():
            record = json.loads(line)
            record.pop("meta", None)
            lines.append(json.dumps(record, sort_keys=True))
        return lines

    assert strip_meta(tmp_path / "a") == strip_meta(tmp_path / "b")


def test_adding_an_agent_does_not_perturb_the_earlier_agents_streams():
    """R-DET-3: SeedSequence.spawn semantics, checked at the policy level."""
    small = Runner(RunConfig(seed=3, n_drones=10, policy="sdlw", **{})).build()
    large = Runner(RunConfig(seed=3, n_drones=12, policy="sdlw")).build()
    try:
        # SDLW draws alpha once at construction from its own generator.
        small_alphas = [a.policy.alpha for a in small.agents]
        large_alphas = [a.policy.alpha for a in large.agents]
        assert small_alphas == large_alphas[: len(small_alphas)]
    finally:
        small._teardown()
        large._teardown()


# -- policy contract --------------------------------------------------------------------------


def test_a_raising_policy_aborts_the_run_with_context():
    """R-POL-9: never caught and converted into a hover."""

    @register_policy("_boom")
    class Boom(Policy):
        def step(self, obs):
            if obs.tick == 3:
                raise ZeroDivisionError("deliberate")
            return Velocity()

    with pytest.raises(PolicyError) as info:
        run(RunConfig(seed=0, policy="_boom", **SHORT))
    assert "tick 3" in str(info.value)
    assert isinstance(info.value.__cause__, ZeroDivisionError)


def test_a_policy_returning_the_wrong_type_is_rejected():
    @register_policy("_wrongtype")
    class WrongType(Policy):
        def step(self, obs):
            return "go forwards"

    with pytest.raises(PolicyError, match="expected one of"):
        run(RunConfig(seed=0, policy="_wrongtype", **SHORT))


def test_observations_are_consistent_within_a_tick():
    """R-POL-8: every agent sees the same blackboard snapshot."""
    seen: list[set] = []

    @register_policy("_snapshot_probe")
    class Probe(Policy):
        def step(self, obs):
            if obs.tick == 5:
                seen.append(frozenset((a, tuple(sorted(v))) for a, v in obs.peers.items()))
            self.publish("mark", obs.tick)
            return Velocity(vz=0.4)

    run(RunConfig(seed=0, policy="_snapshot_probe", **SHORT))
    assert len(seen) == 10
    assert len(set(seen)) == 1


# -- lifecycle and rules -------------------------------------------------------------------------


def test_landing_is_irreversible():
    """R-DRONE-10: a landed drone stays landed AND stops moving, for the rest of the run.

    The original version of this test was vacuous, which a mutation caught: every drone landed
    on the same tick, the run ended, and the "try to fly again" branch never executed. This
    version keeps one drone flying so the run outlives the landings, lands the rest *at cruise
    speed* (landing from a hover slides ~0 mm and proves nothing), and asserts bit-identical
    positions for every subsequent tick.
    """

    @register_policy("_land_then_fly")
    class LandThenFly(Policy):
        def step(self, obs):
            if obs.pose.z < CRUISE_M - 0.02:
                return Velocity(vz=0.4)
            # drone_00 flies for the whole run so the others' landings are followed by many
            # more ticks in which they could misbehave.
            if self.agent_id == "drone_00":
                return Velocity(vx=0.1)
            if obs.tick < 40:
                return Velocity(vx=0.45)      # still moving when it commits
            if obs.lifecycle == "LANDED":
                return Velocity(vx=2.0)       # try hard to move a landed drone
            return Land()

    from safmc_sim.recorder import LIFECYCLE_NAMES

    with tempfile.TemporaryDirectory() as tmp:
        result = run(
            RunConfig(seed=0, n_drones=10, duration_s=40.0, policy="_land_then_fly"),
            recorder=Recorder(tmp),
        )
        states = load_run(tmp)["states"]

    landed_ids = [i for i, a in enumerate(result.lifecycles) if result.lifecycles[a] == "LANDED"]
    assert landed_ids, "no drone landed, so the test proves nothing"

    names = {code: name for code, name in LIFECYCLE_NAMES.items()}
    for i in landed_ids:
        series = [names[int(c)] for c in states["lifecycle"][:, i]]
        first = series.index("LANDED")
        assert first < len(series) - 20, "landing happened too late to prove anything"
        assert set(series[first:]) == {"LANDED"}
        frozen = states["pose"][first:, i, :2]
        assert np.array_equal(frozen, np.broadcast_to(frozen[0], frozen.shape))


def test_departures_are_recorded_not_refused():
    """The two-wave take-off rule is scored after the fact, not enforced mid-flight.

    The platform used to refuse a third take-off. That made the simulator a referee, which is
    the wrong job: a run that breaks a competition rule is a run whose *score* should reflect
    it, and a policy that never learns it broke the rule cannot be debugged. Departures are
    emitted as events and the wave analysis reads them.
    """

    @register_policy("_staggered")
    class Staggered(Policy):
        def step(self, obs):
            index = int(self.agent_id.split("_")[1])
            due = 0.0 if index < 3 else (12.0 if index < 6 else 25.0)
            if obs.sim_time_s < due:
                return Velocity()
            if obs.pose.z < CRUISE_M - 0.02:
                return Velocity(vz=0.4)
            return Velocity(vy=0.45)      # +y is North, out of the Start Area

    result = run(RunConfig(seed=0, policy="_staggered", n_drones=10, duration_s=40.0,
                           record=False))
    departures = [e for e in result.events if e.kind == "departed"]
    assert departures, "no drone left the Start Area"
    # Nothing was refused: every drone that tried to leave, left.
    assert not [e for e in result.events if e.kind == "rule_violation"]

    from safmc_sim.mission import takeoff_waves

    waves = takeoff_waves([e.sim_time_s for e in departures])
    assert len(waves) >= 1


def test_mission_start_is_the_first_exit_from_the_start_area():
    result = run(RunConfig(seed=1, policy="sdlw", n_drones=10, duration_s=30.0,
                           record=False))
    started = [e for e in result.events if e.kind == "mission_started"]
    assert len(started) == 1
    assert result.mission_started_tick == started[0].tick


def test_a_velocity_command_moves_the_drone_with_nothing_in_between():
    """The runner is a pass-through: what a policy commands is what the kinematics receives.

    Guards against a controller creeping back in. If anything sat between the command and the
    drone -- a path follower, an altitude hold, a yaw servo -- the drone would not track a
    constant velocity to within its own first-order lag.
    """

    @register_policy("_constant_velocity")
    class ConstantVelocity(Policy):
        def step(self, obs):
            return Velocity(vx=0.3, vz=0.2)

    runner = Runner(RunConfig(seed=0, n_drones=10, duration_s=6.0, policy="_constant_velocity",
                              record=False))
    runner.build()
    start = np.array([a.xy for a in runner.agents])
    runner.run()
    moved = np.array([a.xy for a in runner.agents]) - start
    # Straight along +x, nothing along +y, and airborne.
    assert np.all(moved[:, 0] > 1.0)
    assert np.allclose(moved[:, 1], 0.0, atol=1e-6)
    assert all(a.z > 0.5 for a in runner.agents)


# -- recording ---------------------------------------------------------------------------------


def test_recording_does_not_change_the_simulation(tmp_path):
    """R-OBS-4."""
    quiet = run(RunConfig(seed=5, policy="sdlw", n_drones=10, duration_s=15.0, record=False))
    loud = run(
        RunConfig(seed=5, policy="sdlw", n_drones=10, duration_s=15.0),
        recorder=Recorder(tmp_path / "loud"),
    )
    assert quiet.score.total == loud.score.total
    assert quiet.ticks == loud.ticks
    assert quiet.lifecycles == loud.lifecycles


# Offline-vs-online re-scoring (R-MISS-8) is covered by
# test_audit_regressions.py::test_offline_rescoring_survives_a_landing_at_speed, which exercises
# the same guarantee on the case that actually broke it. A second, weaker copy here cost 7
# seconds a run and proved strictly less.


def test_log_contains_everything_the_spec_requires(tmp_path):
    """R-OBS-2."""
    result = run(
        RunConfig(seed=1, policy="sdlw", n_drones=10, duration_s=20.0),
        recorder=Recorder(tmp_path / "r"),
    )
    log = load_run(tmp_path / "r")
    header = log["header"]
    assert set(header) >= {"schema", "seed", "config", "agents", "arena", "meta"}
    assert header["arena"]["walls"] and header["arena"]["targets"]
    assert header["meta"]["versions"]["ir-sim"] == "2.10.2"

    states = log["states"]
    n_ticks, n_agents = states["pose"].shape[:2]
    assert n_agents == 10
    assert states["pose"].shape == (n_ticks, 10, 4)      # x, y, z, theta
    assert states["lifecycle"].shape == (n_ticks, 10)
    assert states["command_kind"].shape == (n_ticks, 10)
    assert log["tof"]["ranges_m"].shape == (n_ticks, 10, 64)

    kinds = {e["kind"] for e in log["events"]}
    assert {"departed", "mission_started"} <= kinds
    assert log["footer"]["score"]["total"] == result.score.total


def test_reading_a_log_with_the_wrong_schema_fails_loudly(tmp_path):
    directory = tmp_path / "bad"
    run(RunConfig(seed=1, policy="sdlw", n_drones=10, duration_s=4.0),
        recorder=Recorder(directory))
    path = directory / "run.jsonl"
    lines = path.read_text().splitlines()
    header = json.loads(lines[0])
    header["schema"] = "something/else/9"
    lines[0] = json.dumps(header)
    path.write_text("\n".join(lines) + "\n")
    with pytest.raises(LogFormatError, match="schema"):
        load_run(directory)


def test_reading_a_missing_log_fails_loudly(tmp_path):
    with pytest.raises(LogFormatError, match="no run.jsonl"):
        load_run(tmp_path / "nothing_here")


def test_visualiser_reads_only_the_log(tmp_path):
    """R-OBS-3: replay must not require re-simulating."""
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
    import viz

    directory = tmp_path / "r"
    run(RunConfig(seed=2, policy="sdlw", n_drones=10, duration_s=15.0),
        recorder=Recorder(directory))
    payload = viz.build_payload(directory)
    assert len(payload["pose"]) == len(payload["times"]) > 0
    assert len(payload["agents"]) == 10
    assert payload["arena"]["targets"]
    html = viz.TEMPLATE.replace("__PAYLOAD__", json.dumps(payload))
    assert "__PAYLOAD__" not in html and len(html) > 10_000


# -- start formation ------------------------------------------------------------------------


def test_the_fleet_actually_leaves_the_start_area():
    """Regression: the take-off grid must not trip the fleet's own avoidance thresholds.

    Found the hard way. At 0.72 m spacing a drone's neighbours sat 0.36 m away, and at 0.66 m
    from the southern boundary its rear ranger read 0.62 m before it had moved. Any policy with
    an omnidirectional avoidance threshold therefore turned on the spot for the entire run and
    never left the Start Area -- which looks exactly like a bad strategy and is in fact a
    simulator artefact. Both distances are now named constants with margins.
    """
    for policy in ("sdlw",):
        result = run(
            RunConfig(seed=0, n_drones=12, policy=policy, duration_s=45.0, record=False)
        )
        departures = [e for e in result.events if e.kind == "mission_started"]
        assert departures, f"{policy}: no drone left the Start Area in 45 s"


def test_start_formation_keeps_clear_of_walls_and_of_itself():
    from safmc_sim import constants as K

    for n in (K.FLEET_MIN, 18, K.FLEET_MAX):
        runner = Runner(RunConfig(n_drones=n, policy="sdlw"))
        states = runner._start_positions()
        xy = states[:, :2]
        assert len(xy) == n

        # No drone within a body diameter of another.
        deltas = np.linalg.norm(xy[:, None, :] - xy[None, :, :], axis=2)
        np.fill_diagonal(deltas, np.inf)
        assert deltas.min() > 2 * K.DRONE_RADIUS_M

        # Clear of the field boundary, and inside the Start Area.
        assert xy[:, 0].min() >= K.START_WALL_MARGIN_M
        assert xy[:, 1].min() >= K.START_WALL_MARGIN_M
        assert xy[:, 0].max() <= K.FIELD_WIDTH_M - K.START_WALL_MARGIN_M
        assert xy[:, 1].max() <= K.START_AREA_DEPTH_M


def test_an_impossible_start_spacing_is_rejected():
    from safmc_sim import constants as K

    with pytest.raises(ConfigError, match="start_spacing_m"):
        RunConfig(start_spacing_m=2 * K.DRONE_RADIUS_M)
    with pytest.raises(ConfigError, match="Start Area depth"):
        Runner(RunConfig(n_drones=25, start_spacing_m=3.0, policy="sdlw"))._start_positions()
