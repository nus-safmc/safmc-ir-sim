"""R-TIME-3, R-DET-1, R-MISS-6..8, R-OBS-1..4, R-POL-9: the run loop and its log."""

import filecmp
import json
from pathlib import Path

import numpy as np
import pytest

from safmc_sim import constants as K
from safmc_sim import policies  # noqa: F401 -- registers built-ins
from safmc_sim.api import Hold, Land, Policy, Takeoff, register_policy
from safmc_sim.errors import ConfigError, LogFormatError, PolicyError
from safmc_sim.recorder import Recorder, load_run, score_from_log
from safmc_sim.runner import RunConfig, Runner, run

SHORT = dict(n_drones=10, duration_s=8.0, record=False)


# -- configuration validation ---------------------------------------------------------------


@pytest.mark.parametrize("n", [0, 9, 26, 100])
def test_fleet_size_outside_the_published_range_is_rejected(n):
    """R-MISS-7: fewer than 10 drones forfeits the run."""
    with pytest.raises(ConfigError, match="n_drones"):
        RunConfig(n_drones=n)


def test_a_sensor_rate_that_does_not_divide_the_tick_rate_is_rejected():
    """R-TIME-3: no silent rounding of a sensor rate."""
    with pytest.raises(ConfigError, match="does not divide"):
        RunConfig(tick_hz=20.0, tof_rate_hz=15.0)
    with pytest.raises(ConfigError, match="does not divide"):
        RunConfig(tick_hz=20.0, marker_rate_hz=3.0)
    # 20 / 2 and 20 / 4 are exact, so these are fine.
    RunConfig(tick_hz=20.0, marker_rate_hz=2.0)
    RunConfig(tick_hz=20.0, tof_rate_hz=4.0)


def test_cruise_altitude_above_the_ceiling_is_rejected():
    with pytest.raises(ConfigError, match="cruise_alt_m"):
        RunConfig(cruise_alt_m=K.CEILING_M + 0.1)


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
            RunConfig(seed=11, policy="frontier", n_drones=10, duration_s=8.0),
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
            return Hold()

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
            return Takeoff() if obs.lifecycle == "IDLE" else Hold()

    run(RunConfig(seed=0, policy="_snapshot_probe", **SHORT))
    assert len(seen) == 10
    assert len(set(seen)) == 1


# -- lifecycle and rules -------------------------------------------------------------------------


def test_landing_is_irreversible():
    """R-DRONE-10: a landed drone stays landed and stops moving."""

    @register_policy("_land_then_fly")
    class LandThenFly(Policy):
        def step(self, obs):
            if obs.lifecycle == "IDLE":
                return Takeoff()
            if obs.tick < 40:
                return Hold()
            return Land() if obs.tick < 100 else Takeoff()

    runner = Runner(RunConfig(seed=0, n_drones=10, duration_s=12.0, policy="_land_then_fly"))
    result = runner.build().run()
    assert set(result.lifecycles.values()) <= {"LANDED", "CRASHED"}
    assert result.landed


def test_only_two_takeoff_waves_are_admitted():
    """R-MISS-6: a third wave is refused and recorded, not raised."""

    @register_policy("_three_waves")
    class ThreeWaves(Policy):
        def step(self, obs):
            index = int(self.agent_id.split("_")[1])
            # A wave stays open for 10 s from its first departure, so these three groups are
            # three distinct waves: t=0, t=12 (wave 1 has closed), t=25 (wave 2 has closed).
            due = 0.0 if index < 3 else (12.0 if index < 6 else 25.0)
            if obs.lifecycle == "IDLE" and obs.sim_time_s >= due:
                return Takeoff()
            return Hold()

    result = run(RunConfig(seed=0, policy="_three_waves", n_drones=10, duration_s=30.0,
                           record=False))
    waves = [e for e in result.events if e.kind == "wave_opened"]
    violations = [e for e in result.events if e.kind == "rule_violation"]
    assert len(waves) == K.MAX_TAKEOFF_WAVES
    assert violations and violations[0].detail["rule"] == "max_takeoff_waves"
    # The refused drones never left the ground.
    refused = {e.agent_id for e in violations}
    assert all(result.lifecycles[a] == "IDLE" for a in refused)


def test_mission_start_is_the_first_exit_from_the_start_area():
    result = run(RunConfig(seed=1, policy="frontier", n_drones=10, duration_s=30.0,
                           record=False))
    started = [e for e in result.events if e.kind == "mission_started"]
    assert len(started) == 1
    assert result.mission_started_tick == started[0].tick


def test_a_grounded_drone_cannot_be_moved_by_a_velocity_command():
    from safmc_sim.api import VelocityBody

    @register_policy("_ground_dash")
    class GroundDash(Policy):
        def step(self, obs):
            return VelocityBody(vx=2.0)

    runner = Runner(RunConfig(seed=0, n_drones=10, duration_s=4.0, policy="_ground_dash"))
    runner.build()
    before = np.array([a.xy for a in runner.agents])
    runner.run()
    assert np.allclose(before, np.array([a.xy for a in runner.agents]))


# -- recording ---------------------------------------------------------------------------------


def test_recording_does_not_change_the_simulation(tmp_path):
    """R-OBS-4."""
    quiet = run(RunConfig(seed=5, policy="frontier", n_drones=10, duration_s=15.0, record=False))
    loud = run(
        RunConfig(seed=5, policy="frontier", n_drones=10, duration_s=15.0),
        recorder=Recorder(tmp_path / "loud"),
    )
    assert quiet.score.total == loud.score.total
    assert quiet.ticks == loud.ticks
    assert quiet.lifecycles == loud.lifecycles


def test_offline_rescoring_matches_the_online_score_exactly(tmp_path):
    """R-MISS-8."""
    for seed in (1, 2, 3):
        directory = tmp_path / f"s{seed}"
        result = run(
            RunConfig(seed=seed, policy="frontier", n_drones=10, duration_s=40.0),
            recorder=Recorder(directory),
        )
        offline = score_from_log(directory)
        assert offline.total == result.score.total
        assert offline.raw_total == result.score.raw_total
        assert offline.relay_formed == result.score.relay_formed
        assert offline.per_target == result.score.per_target


def test_log_contains_everything_the_spec_requires(tmp_path):
    """R-OBS-2."""
    result = run(
        RunConfig(seed=1, policy="frontier", n_drones=10, duration_s=20.0),
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
    assert states["velocity"].shape == (n_ticks, 10, 2)
    assert states["lifecycle"].shape == (n_ticks, 10)
    assert states["command_kind"].shape == (n_ticks, 10)
    assert log["tof"]["collapsed_m"].shape == (n_ticks, 10, 64)

    kinds = {e["kind"] for e in log["events"]}
    assert {"wave_opened", "airborne"} <= kinds
    assert log["footer"]["score"]["total"] == result.score.total


def test_reading_a_log_with_the_wrong_schema_fails_loudly(tmp_path):
    directory = tmp_path / "bad"
    run(RunConfig(seed=1, policy="hold", n_drones=10, duration_s=4.0),
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
    for policy in ("random_walk", "sdlw", "wall_follow", "frontier"):
        result = run(
            RunConfig(seed=0, n_drones=12, policy=policy, duration_s=45.0, record=False)
        )
        departures = [e for e in result.events if e.kind == "mission_started"]
        assert departures, f"{policy}: no drone left the Start Area in 45 s"


def test_start_formation_keeps_clear_of_walls_and_of_itself():
    from safmc_sim import constants as K

    for n in (K.FLEET_MIN, 18, K.FLEET_MAX):
        runner = Runner(RunConfig(n_drones=n, policy="hold"))
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
        Runner(RunConfig(n_drones=25, start_spacing_m=3.0, policy="hold"))._start_positions()
