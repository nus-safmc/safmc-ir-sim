"""R-MISS-1..5: the competition's scoring rules."""

import numpy as np
import pytest

from safmc_sim import constants as K
from safmc_sim.mission import POINTS, Mission
from safmc_sim.world.arena import ArenaConfig, ArenaSpec, Target, Wall


def bare_arena(targets, walls=()):
    """A 20x20 arena with only the walls a test asks for, so geometry is unambiguous."""
    boundary = (
        Wall(-0.05, 0, -0.05, 20, 0.1, K.PERIMETER_WALL_HEIGHT_M, "perimeter_wall"),
        Wall(0, 20.05, 20, 20.05, 0.1, K.PERIMETER_WALL_HEIGHT_M, "perimeter_wall"),
        Wall(20.05, 0, 20.05, 20, 0.1, K.PERIMETER_WALL_HEIGHT_M, "perimeter_wall"),
        Wall(0, -0.05, 20, -0.05, 0.1, K.SAFETY_NET_HEIGHT_M, "net"),
    )
    return ArenaSpec(
        seed=0, width_m=20.0, depth_m=20.0, ceiling_m=K.CEILING_M,
        start_area_depth_m=K.START_AREA_DEPTH_M, unknown_area=(5.0, 8.0, 15.0, 18.0),
        walls=boundary + tuple(walls), pillars=(), targets=tuple(targets),
        config=ArenaConfig(),
    )


def test_points_are_the_published_values():
    assert POINTS == {"victim": 5, "bonus_victim": 15, "fire": 10}


def test_target_scores_only_when_a_landed_drone_is_inside_the_radius():
    """R-MISS-1: within 1.0 m."""
    victim = Target("v0", "victim", 10.0, 10.0)
    mission = Mission(bare_arena([victim]))

    mission.update(0, 0.0, {"d0": np.array([10.0 + K.SCORE_RADIUS_M + 0.05, 10.0])})
    assert mission.score({}).raw_total == 0

    mission.update(1, 0.05, {"d0": np.array([10.0 + K.SCORE_RADIUS_M - 0.05, 10.0])})
    assert mission.score({}).raw_total == POINTS["victim"]


def test_line_of_sight_is_required_and_only_structure_blocks_it():
    """R-MISS-2: 'no walls or pillars on the line'. Markers must not block."""
    victim = Target("v0", "victim", 12.0, 10.0)
    # A wall between the drone and the victim, with the drone still only 0.6 m away -- well
    # inside the scoring radius. Range alone is not enough.
    wall = Wall(11.7, 8.0, 11.7, 12.0, 0.1, K.INNER_WALL_HEIGHT_M, "inner_wall")

    blocked = Mission(bare_arena([victim], [wall]))
    blocked.update(0, 0.0, {"d0": np.array([11.4, 10.0])})
    assert blocked.score({}).raw_total == 0

    clear = Mission(bare_arena([victim]))
    clear.update(0, 0.0, {"d0": np.array([11.4, 10.0])})
    assert clear.score({}).raw_total == POINTS["victim"]


def test_another_marker_does_not_block_line_of_sight():
    victim = Target("v0", "victim", 12.0, 10.0)
    decoy = Target("f0", "fire", 11.5, 10.0)
    mission = Mission(bare_arena([victim, decoy]))
    mission.update(0, 0.0, {"d0": np.array([11.1, 10.0])})
    assert mission.targets["v0"].serviced


def test_each_target_scores_once_however_many_drones_land():
    victim = Target("v0", "victim", 10.0, 10.0)
    mission = Mission(bare_arena([victim]))
    mission.update(
        0, 0.0,
        {"d0": np.array([10.3, 10.0]), "d1": np.array([9.7, 10.0]), "d2": np.array([10.0, 10.4])},
    )
    assert mission.score({}).raw_total == POINTS["victim"]
    assert len(mission.targets["v0"].serviced_by) == 3


def test_airborne_drones_never_score():
    """Only landed drones count -- the runner passes landed positions only."""
    victim = Target("v0", "victim", 10.0, 10.0)
    mission = Mission(bare_arena([victim]))
    mission.update(0, 0.0, {})
    assert mission.score({}).raw_total == 0


def test_unextinguished_fire_zeroes_victims_within_2p5_metres():
    """R-MISS-3."""
    victim = Target("v0", "victim", 10.0, 10.0)
    fire = Target("f0", "fire", 10.0, 10.0 + K.FIRE_SUPPRESSION_RADIUS_M - 0.1)
    mission = Mission(bare_arena([victim, fire]))

    mission.update(0, 0.0, {"d0": np.array([10.2, 10.0])})
    breakdown = mission.score({"d0": np.array([10.2, 10.0])})
    assert breakdown.raw_total == 0
    assert breakdown.suppressed == ("v0",)

    # Extinguish the fire and the victim's points come back, plus the fire's own.
    mission.update(1, 0.05, {"d0": np.array([10.2, 10.0]), "d1": np.array([10.0, 12.3])})
    landed = {"d0": np.array([10.2, 10.0]), "d1": np.array([10.0, 12.3])}
    breakdown = mission.score(landed)
    assert breakdown.suppressed == ()
    assert breakdown.raw_total == POINTS["victim"] + POINTS["fire"]


def test_a_distant_fire_does_not_suppress():
    victim = Target("v0", "victim", 10.0, 10.0)
    fire = Target("f0", "fire", 10.0, 10.0 + K.FIRE_SUPPRESSION_RADIUS_M + 0.2)
    mission = Mission(bare_arena([victim, fire]))
    mission.update(0, 0.0, {"d0": np.array([10.2, 10.0])})
    assert mission.score({"d0": np.array([10.2, 10.0])}).raw_total == POINTS["victim"]


def _relay_chain(y_from, y_to, x, spacing):
    """Landed drones stepping from ``y_from`` down to ``y_to`` at ``spacing`` apart."""
    ys = np.arange(y_from, y_to - 1e-9, -spacing)
    return {f"r{i}": np.array([x, y]) for i, y in enumerate(ys)}


def test_relay_doubles_the_total_score():
    """R-MISS-4."""
    bonus = Target("b0", "bonus_victim", 10.0, 10.0)
    arena = bare_arena([bonus])
    mission = Mission(arena)

    chain = _relay_chain(9.6, 5.0, 10.0, K.RELAY_SPACING_M - 0.05)
    chain["r0"] = np.array([10.0, 9.6])  # within 1 m of the bonus victim
    mission.update(0, 0.0, chain)
    breakdown = mission.score(chain)

    assert mission.targets["b0"].serviced
    assert breakdown.relay_formed
    assert breakdown.multiplier == K.RELAY_MULTIPLIER
    assert breakdown.total == POINTS["bonus_victim"] * 2
    # The chain must start at the rescuer and end inside the Start Area.
    assert breakdown.relay_chain[0] == "r0"
    assert arena.in_start_area(*chain[breakdown.relay_chain[-1]])


def test_relay_needs_a_bonus_victim_rescuer_at_its_head():
    """A chain of landed drones to the Start Area is not a relay without the bonus rescue."""
    victim = Target("v0", "victim", 10.0, 10.0)   # regular, not bonus
    mission = Mission(bare_arena([victim]))
    chain = _relay_chain(9.6, 5.0, 10.0, K.RELAY_SPACING_M - 0.05)
    mission.update(0, 0.0, chain)
    breakdown = mission.score(chain)
    assert mission.targets["v0"].serviced
    assert not breakdown.relay_formed
    assert breakdown.multiplier == 1.0


def test_relay_breaks_when_a_gap_exceeds_one_metre():
    bonus = Target("b0", "bonus_victim", 10.0, 10.0)
    mission = Mission(bare_arena([bonus]))
    chain = _relay_chain(9.6, 5.0, 10.0, K.RELAY_SPACING_M - 0.05)
    # Remove one link, opening a ~1.9 m gap.
    key = sorted(chain)[3]
    del chain[key]
    mission.update(0, 0.0, chain)
    assert not mission.score(chain).relay_formed


def test_relay_requires_line_of_sight_between_adjacent_drones():
    bonus = Target("b0", "bonus_victim", 10.0, 10.0)
    # A wall between two adjacent links, close enough that they are within 1 m of each other.
    wall = Wall(9.0, 7.5, 11.0, 7.5, 0.1, K.INNER_WALL_HEIGHT_M, "inner_wall")
    mission = Mission(bare_arena([bonus], [wall]))
    chain = _relay_chain(9.6, 5.0, 10.0, K.RELAY_SPACING_M - 0.05)
    mission.update(0, 0.0, chain)
    assert not mission.score(chain).relay_formed


def test_two_relays_score_the_same_as_one():
    bonus = Target("b0", "bonus_victim", 10.0, 10.0)
    mission = Mission(bare_arena([bonus]))
    first = _relay_chain(9.6, 5.0, 10.0, K.RELAY_SPACING_M - 0.05)
    second = {f"s{k}": v + np.array([3.0, 0.0]) for k, v in first.items()}
    second["s0"] = np.array([10.0, 9.6]) + np.array([0.3, 0.0])
    both = {**first, **second}
    mission.update(0, 0.0, both)
    assert mission.score(both).multiplier == K.RELAY_MULTIPLIER


def test_score_arithmetic_is_explainable():
    bonus = Target("b0", "bonus_victim", 10.0, 10.0)
    mission = Mission(bare_arena([bonus]))
    chain = _relay_chain(9.6, 5.0, 10.0, K.RELAY_SPACING_M - 0.05)
    mission.update(0, 0.0, chain)
    text = mission.score(chain).explain()
    assert "raw 15" in text and "x2" in text and "30" in text


def test_servicing_emits_exactly_one_event_per_target():
    victim = Target("v0", "victim", 10.0, 10.0)
    mission = Mission(bare_arena([victim]))
    events = mission.update(0, 0.0, {"d0": np.array([10.2, 10.0])})
    assert [e.kind for e in events] == ["target_serviced"]
    assert events[0].detail["points"] == POINTS["victim"]
    # A second drone joining changes serviced_by but must not re-award.
    again = mission.update(1, 0.05, {"d0": np.array([10.2, 10.0]), "d1": np.array([9.8, 10.0])})
    assert [e.kind for e in again] == []


def test_theoretical_maximum_is_240():
    """(4x5 + 4x15 + 4x10) x 2, per the rulebook."""
    targets = []
    landed = {}
    for i in range(4):
        for kind in ("victim", "bonus_victim", "fire"):
            # Spread far apart so no fire suppresses any victim.
            x, y = 2.0 + 4.0 * i, {"victim": 10.0, "bonus_victim": 14.0, "fire": 18.0}[kind]
            targets.append(Target(f"{kind}_{i}", kind, x, y))
            landed[f"{kind}_{i}_d"] = np.array([x + 0.3, y])
    arena = bare_arena(targets)
    mission = Mission(arena)
    # A relay from one bonus rescuer down to the Start Area.
    chain = _relay_chain(13.6, 5.0, 2.3, K.RELAY_SPACING_M - 0.05)
    landed.update(chain)
    landed["bonus_victim_0_d"] = np.array([2.3, 14.0])
    mission.update(0, 0.0, landed)
    breakdown = mission.score(landed)
    assert breakdown.raw_total == 4 * 5 + 4 * 15 + 4 * 10 == 120
    assert breakdown.total == 240
