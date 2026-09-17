"""ADR-0007's relay trial, T-1..T-4: the example forms the relay and is graded from its log.

The trail geometry and the crumb network are pure and tested as such; T-1..T-3 are one planted
scenario run end to end in both deployment modes; T-4 is a refusal.
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
from safmc_sim.errors import PolicyError
from safmc_sim.recorder import Recorder, load_run
from safmc_sim.runner import RunConfig, flown_sensors, run
from safmc_sim.sensors.uwb import UWBConfig
from safmc_sim.world.arena import ArenaConfig, Target, generate_arena, validate_arena
from safmc_sim.world.landmark import Landmark


@functools.lru_cache(maxsize=1)
def example():
    path = Path(__file__).resolve().parent.parent / "examples" / "06_uwb_relay.py"
    spec = importlib.util.spec_from_file_location("example_relay", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# -- the trail --------------------------------------------------------------------------------------


def test_a_trail_accumulates_arclength_and_interpolates():
    ex = example()
    t = ex.Trail()
    for x in (0.0, 1.0, 2.0):
        t.append(x, 0.0, 1.0)
    assert t.length == pytest.approx(2.0)
    assert np.allclose(t.point_at(0.5), [0.5, 0.0])
    assert np.allclose(t.point_at(-1.0), [0.0, 0.0]) and np.allclose(t.point_at(9.0), [2.0, 0.0])
    t.append(2.0, 0.0, 1.0)                                   # a repeated point is ignored
    assert len(t) == 3


def test_a_revisit_inside_a_seen_free_disc_cuts_the_loop_and_one_outside_does_not():
    """The cutter's evidence is the ring: a chord fits inside a disc the ring saw empty, with room
    for a body. Without that evidence the same revisit is not cut."""
    ex = example()
    loop = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (0.5, 0.5), (0.5, 0.05)]
    seen = ex.Trail()
    for x, y in loop:
        seen.append(x, y, 0.8)                                # clearance 0.8 everywhere
    # (0.5, 0.05) is 0.5 m from (0, 0): chord + body 0.68 <= 0.8, so everything between is cut.
    assert len(seen) == 2 and seen.length == pytest.approx(np.hypot(0.5, 0.05))
    blind = ex.Trail()
    for x, y in loop:
        blind.append(x, y, 0.3)                               # hugging a wall: no evidence
    assert len(blind) == 7
    assert blind.length > 4.0


def test_relays_needed_is_the_spacing_arithmetic():
    ex = example()
    assert ex.relays_needed(0.5) == 1
    assert ex.relays_needed(1.8) == 1          # two links of 0.9
    assert ex.relays_needed(1.81) == 2
    assert ex.relays_needed(9.0) == 9
    assert ex.horizontal(np.hypot(0.8, 0.5), 0.5) == pytest.approx(0.8)
    assert ex.horizontal(0.3, 0.5) == 0.0, "never imaginary"


def test_the_network_links_an_anchor_to_crumbs_anywhere_inside_its_reach():
    """The auditor's reproduction: a trail beginning 0.70-0.87 m from the anchor was 'infeasible'
    because the anchor edge was searched with a 0.6 m cell hash. Every crumb within 0.9 m links."""
    ex = example()
    anchors = np.array([[1.75, 5.0]])
    for x0 in (2.35, 2.45, 2.55, 2.62):
        head = [(x0, y, 0.8) for y in np.arange(5.0, 7.0, 0.25)]
        routed = ex.CrumbNetwork(anchors).route({"h": head}, "h")
        assert routed is not None, x0
        # Dijkstra takes the longest anchor hop it may (the farthest crumb inside 0.9 m), so the
        # route is shorter than "across, then north" and no shorter than the straight line.
        straight = np.hypot(x0 - 1.75, 1.75)
        assert straight <= routed[0].length <= abs(x0 - 1.75) + 1.75 + 1e-9
    beyond = [(2.70, y, 0.8) for y in np.arange(5.0, 7.0, 0.25)]           # 0.95 m: out of reach
    assert ex.CrumbNetwork(anchors).route({"h": beyond}, "h") is None


def test_an_anchor_with_structure_in_its_disc_is_dropped_for_that_seed():
    """22 of 200 generated arenas have an inner wall reaching below the start line, and the
    network treats the 0.9 m disc around an anchor as free. Seeds 16 and 144 put a row anchor
    within 0.35 m of such a wall; those anchors are left out. The first anchor never is."""
    ex = example()
    from safmc_sim.world.arena import generate_arena
    for seed, missing in ((16, "start_anchor_8"), (144, "start_anchor_1")):
        row = ex.anchor_row(10, generate_arena(seed))
        ids = {lm.id for lm in row}
        assert missing not in ids and "start_anchor_0" in ids and len(row) == 9
    assert len(ex.anchor_row(10)) == 10, "without an arena nothing is dropped"
    cfg = ex.make_config(seed=16, n_anchors=10)
    assert len(cfg.arena_config.landmarks) == 9
    assert cfg.sensors[-1].rate_hz == pytest.approx(ex.peer_sweep_rate_hz(10, 9))


def test_the_network_routes_through_another_searcher_s_crumbs():
    """A head that wandered east before landing; a second searcher crossed the strip straight
    north. The route uses the second trail and is far shorter than the head's own."""
    ex = example()
    anchors = np.array([[1.75, 5.0], [7.45, 5.0]])
    head = [(1.75, y, 0.8) for y in np.arange(1.75, 6.5, 0.25)]           # north along x = 1.75
    head += [(x, 6.5, 0.8) for x in np.arange(2.0, 7.5, 0.25)]           # then 5.5 m east
    head += [(7.5, 6.6, 0.8)]                                             # landed here
    other = [(7.4, y, 0.8) for y in np.arange(1.75, 6.75, 0.25)]          # straight north at x = 7.4
    net = ex.CrumbNetwork(anchors)
    routed = net.route({"drone_00": head, "drone_01": other}, "drone_00")
    assert routed is not None
    trail, anchor_index = routed
    assert anchor_index == 1, "the anchor nearest the short route"
    assert trail.length < 2.5, f"{trail.length:.2f} m via the second trail, not ~11 m via the head's own"
    own = ex.compose_trail(anchors, head).trail
    assert own.length > 6.0
    # No crumbs of the head at all: nothing to route to.
    assert net.route({"drone_01": other}, "drone_00") is None


# -- T-1..T-3: the planted scenario, both modes -------------------------------------------------------


BONUS = Target("b_north", "bonus_victim", 1.75, 8.0)


def planted_arena(cfg: RunConfig):
    arena = generate_arena(cfg.seed, cfg.arena_config)
    arena = dataclasses.replace(arena, targets=(BONUS,))
    validate_arena(arena)
    return arena


@pytest.mark.parametrize("mode", ["dispatch", "train"])
def test_the_trial_forms_a_relay_on_a_planted_bonus_victim_and_grades_from_the_log(mode):
    """T-1: the relay forms. T-2: every link of the chain at formation is within 1.0 m with
    floor-level line of sight and the tail is in the Start Area, by the mission's own rule from
    states.npz. T-3: every relay's last fresh sweep before landing measured both chain
    neighbours inside the gate, from uwb.npz."""
    ex = example()
    cfg = ex.make_config(seed=0, n_drones=10, n_relay=3, mode=mode, duration_s=120.0)
    with tempfile.TemporaryDirectory() as tmp:
        result = run(cfg, recorder=Recorder(tmp), arena=planted_arena(cfg))
        report = ex.grade(tmp)
        log = load_run(tmp)

    assert result.score.relay_formed, result.score.explain()
    assert report["relay_formed"] and report["time_to_relay_s"] is not None
    assert report["time_to_relay_s"] < 120.0
    assert result.score.total == 2 * K.POINTS_BONUS_VICTIM

    # T-2. The chain comes from the mission's own rule, so "links <= 1.0 and clear" cannot fail
    # once T-1 holds (an auditor pointed that out); what can fail is whether the relays that
    # landed are all *in* it -- nobody landed uselessly -- and how tight the band left it.
    assert report["chain"][0] == "drone_00", "the lead, on the bonus victim, is the head"
    assert len(report["chain"]) == 3, "a ~2.3 m trail needs two relays; the third stays home"
    landed_relays = {a for a, life in result.lifecycles.items() if life == "LANDED" and a >= "drone_07"}
    assert landed_relays == set(report["chain"][1:]), "every relay that landed is a link"
    assert np.mean(report["links_m"]) <= ex.MAX_LINK_M, report["links_m"]
    assert all(report["links_clear"]) and report["tail_in_start_area"]

    # T-3
    gate = report["gate"]
    assert len(gate) == 2
    for entry in gate:
        assert entry["ok"], entry
        assert entry["succ"] <= ex.MAX_LINK_M and entry["pred"] <= ex.MAX_LINK_M
        assert entry["t_sweep"] < entry["t_land"]

    # The relays landed on the same tick: same predicate, same snapshot (R-POL-8).
    landed = [e for e in log["events"] if e["kind"] == "landed" and e["agent_id"] in report["chain"][1:]]
    assert len({e["tick"] for e in landed}) == 1
    # And the third relay never left the ground.
    assert result.lifecycles["drone_09"] == "ACTIVE"
    assert not any(e["agent_id"] == "drone_09" for e in log["events"] if e["kind"] == "departed")


def test_too_few_relays_for_the_trail_never_land_because_the_gate_does_not_pass():
    """T-3 the other way round, and the test a mutant gate cannot pass: one relay on the same
    ~2.3 m trail. In train mode it joins (there is room at d*) and the band parks it at the
    middle, 1.15 m from each end -- outside MAX_LINK_M -- so it must hover to the end of the
    run. A gate that ignored the ranges would land it and score a 1.15 m 'relay' the mission
    rejects. In dispatch the same trail is infeasible for one relay and it never launches."""
    ex = example()
    cfg = ex.make_config(seed=0, n_drones=10, n_relay=1, mode="train", duration_s=90.0)
    with tempfile.TemporaryDirectory() as tmp:
        result = run(cfg, recorder=Recorder(tmp, record_sensors=False), arena=planted_arena(cfg))
        log = load_run(tmp)
    assert not result.score.relay_formed
    assert result.lifecycles["drone_09"] == "ACTIVE", "airborne to the end: the gate never passed"
    assert any(e["agent_id"] == "drone_09" for e in log["events"] if e["kind"] == "departed"), "it did join"
    z = log["states"]["pose"][-1, 9, 2]
    assert z > 0.3, "and it is hovering, not parked"

    cfg = ex.make_config(seed=0, n_drones=10, n_relay=1, mode="dispatch", duration_s=60.0)
    with tempfile.TemporaryDirectory() as tmp:
        result = run(cfg, recorder=Recorder(tmp, record_sensors=False), arena=planted_arena(cfg))
        log = load_run(tmp)
    assert not result.score.relay_formed
    assert not any(e["agent_id"] == "drone_09" for e in log["events"] if e["kind"] == "departed"), \
        "infeasible: it never launched"


def test_the_trial_refuses_to_run_without_peer_ranging_or_without_an_anchor():
    """T-4: the controller must not fall back to pose for spacing silently."""
    ex = example()
    cfg = ex.make_config(seed=0, n_drones=10, n_relay=3, duration_s=2.0)
    no_peers = dataclasses.replace(
        cfg, record=False,
        sensors=flown_sensors() + (UWBConfig(peers=False, anchor_height_m=ex.ANCHOR_HEIGHT_M),),
    )
    # The runner re-raises a policy's exception with the agent and tick attached (R-POL-9);
    # the refusal itself is the cause.
    def refused(config, match):
        with pytest.raises(PolicyError) as info:
            run(config)
        cause = info.value.__cause__
        assert isinstance(cause, PolicyError) and match in str(cause), (match, str(cause))

    refused(no_peers, "peers=True")
    no_anchor = dataclasses.replace(
        cfg, record=False, arena_config=ArenaConfig(landmarks=()),
        sensors=flown_sensors() + (UWBConfig(peers=True),),
    )
    refused(no_anchor, "at least one anchor")
    with pytest.raises(PolicyError, match="mode"):
        run(dataclasses.replace(cfg, record=False, policy_config={"n_relay": 3, "mode": "convoy"}))
    refused(dataclasses.replace(cfg, record=False, policy_config={"n_relay": 10, "mode": "dispatch"}),
            "no searcher")


def test_the_anchor_row_is_legal_and_the_rates_divide_the_tick():
    ex = example()
    for n in (1, 5, 10):
        row = ex.anchor_row(n)
        assert all(lm.y == ex.ANCHOR_Y and lm.kind == "uwb_anchor" for lm in row)
        assert all(0.0 < lm.x < 20.0 for lm in row)
    with pytest.raises(ValueError):
        ex.anchor_row(0)
    with pytest.raises(ValueError):
        ex.anchor_row(12)
    for n_drones, n_anchors in ((10, 1), (10, 10), (25, 1), (25, 10), (15, 1)):
        ex.make_config(n_drones=n_drones, n_anchors=n_anchors)


def test_the_sweep_summary_reports_per_cell():
    ex = example()
    reports = [
        dict(mode="dispatch", n_anchors=1, n_relay=3, n_drones=10, seed=s, relay_formed=s == 0,
             time_to_relay_s=50.0 if s == 0 else None, score=40 if s == 0 else 20, targets=2, crashed=0, waves=2)
        for s in range(2)
    ] + [dict(mode="none", n_anchors=1, n_relay=0, n_drones=10, seed=0, relay_formed=False,
              time_to_relay_s=None, score=25, targets=3, crashed=1, waves=1)]
    text = ex.summarise_sweep(reports)
    assert "dispatch" in text and "none" in text
    assert "0.50" in text, "P(relay) over the two dispatch seeds"
