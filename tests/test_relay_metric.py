"""The relay timeline: *when* a relay formed, replayed from the log by the mission's own rule.

Until ADR-0007 ``relay_formed`` was one bit in the footer. These tests plant a bonus victim in
the west corridor of the Known Search Area -- clear of the room on every seed -- and script
three drones to land in a chain from it into the Start Area, then read the moment back from
``states.npz`` alone.
"""

from __future__ import annotations

import dataclasses
import tempfile

import numpy as np
import pytest

from safmc_sim import constants as K
from safmc_sim import policies  # noqa: F401
from safmc_sim.api import Land, Policy, Velocity, register_policy
from safmc_sim.metrics import RelayMoment, compute_metrics, relay_timeline
from safmc_sim.recorder import Recorder, load_run, score_from_log
from safmc_sim.runner import RunConfig, run
from safmc_sim.world.arena import Target, generate_arena, validate_arena

# A bonus victim at x = 1.0 is in the Known Area's west corridor whatever the seed: the room's
# west face is at x0 >= 2.05, and the perimeter wall is the only structure west of it.
BONUS = Target("b_west", "bonus_victim", 1.0, 7.6)

# Landing spots, head first: within 1 m of the victim with floor-level LOS, then 0.8 m steps
# south until the tail is inside the Start Area (y <= 6.0).
CHAIN = ((1.0, 6.9), (1.0, 6.1), (1.0, 5.3))


def relay_arena(seed=0):
    arena = generate_arena(seed)
    arena = dataclasses.replace(arena, targets=(BONUS,))
    validate_arena(arena)
    return arena


@register_policy("_scripted_chain")
class ScriptedChain(Policy):
    """Drone k flies to CHAIN[k] and lands, one drone at a time; everyone else hovers."""

    def step(self, obs):
        k = int(self.agent_id[-2:])
        if k >= len(CHAIN):
            return Velocity(vz=0.3) if obs.pose.z < 0.5 else Velocity()
        if obs.sim_time_s < 15.0 * k:
            return Velocity(vz=0.3) if obs.pose.z < 0.5 else Velocity()
        if obs.pose.z < 0.45:
            return Velocity(vz=0.4)
        goal = np.array(CHAIN[k])
        delta = goal - obs.pose.xy
        distance = float(np.linalg.norm(delta))
        if distance < 0.05:
            return Land()
        speed = min(0.45, 2.0 * distance)
        v = delta / distance * speed
        return Velocity(vx=float(v[0]), vy=float(v[1]))


def _run(tmp, seed=0):
    cfg = RunConfig(seed=seed, policy="_scripted_chain", n_drones=10, duration_s=50.0)
    return run(cfg, recorder=Recorder(tmp), arena=relay_arena(seed))


def test_the_timeline_reports_the_relay_at_the_landing_that_completed_it():
    with tempfile.TemporaryDirectory() as tmp:
        result = _run(tmp)
        log = load_run(tmp)
        offline = score_from_log(tmp)
        metrics = compute_metrics(tmp)

    assert result.score.relay_formed, result.score.explain()
    assert offline.relay_chain == result.score.relay_chain == ("drone_00", "drone_01", "drone_02")

    moments = relay_timeline(log)
    assert [m.landed for m in moments] == [1, 2, 3], "one moment per landing, in order"
    assert all(isinstance(m, RelayMoment) for m in moments)
    assert moments[0].chain == () and moments[1].chain == (), "no relay until the tail is in the Start Area"
    assert moments[2].chain == ("drone_00", "drone_01", "drone_02")
    # The moment's tick is the tick on which the third drone became LANDED in the log.
    landed_code = [c for c, n in log["header"]["codebook"]["lifecycle"].items() if n == "LANDED"][0]
    third = np.flatnonzero((log["states"]["lifecycle"] == int(landed_code)).sum(axis=1) == 3)[0]
    assert moments[2].tick == int(third)
    assert moments[2].sim_time_s == pytest.approx(float(log["states"]["time_s"][third]))

    assert metrics.relay_formed and metrics.relay_chain_drones == 3
    assert metrics.time_to_relay_s == pytest.approx(moments[2].sim_time_s)
    assert 30.0 < metrics.time_to_relay_s < 50.0, "the third drone launched at t = 30 s"
    assert metrics.score_total == 2 * K.POINTS_BONUS_VICTIM


def test_a_chain_with_a_gap_never_forms_and_the_metric_says_none():
    """Pull the middle drone 1.1 m south: two links now exceed 1.0 m and no replay finds a relay."""
    global CHAIN
    original = CHAIN
    CHAIN = ((1.0, 6.9), (1.0, 4.9), (1.0, 4.1))
    try:
        with tempfile.TemporaryDirectory() as tmp:
            result = _run(tmp)
            log = load_run(tmp)
            metrics = compute_metrics(tmp)
    finally:
        CHAIN = original
    assert not result.score.relay_formed
    assert all(m.chain == () for m in relay_timeline(log))
    assert metrics.time_to_relay_s is None and metrics.relay_chain_drones == 0
    assert metrics.score_total == K.POINTS_BONUS_VICTIM, "the rescue still scores, unmultiplied"


def test_a_run_with_no_landings_has_an_empty_timeline():
    with tempfile.TemporaryDirectory() as tmp:
        run(RunConfig(seed=0, policy="sdlw", n_drones=10, duration_s=2.0), recorder=Recorder(tmp))
        log = load_run(tmp)
        metrics = compute_metrics(tmp)
    assert relay_timeline(log) == ()
    assert metrics.time_to_relay_s is None and metrics.relay_chain_drones == 0
