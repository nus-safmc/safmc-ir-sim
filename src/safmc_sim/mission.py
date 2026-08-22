"""The competition's task and scoring layer.

Implements the SAFMC 2026 Category Swarm rules (booklet v2.0, sections 3.3 and 3.4). Kept
data-driven and separate from the simulator because the *mechanic* layer is the volatile part:
between 2025 and 2026, Danger Zones were deleted, fires were added with a coupling rule, and
the relay with its 2x multiplier appeared. The stable core -- the field, the ceiling, the fleet
size, land-within-1-m-with-line-of-sight -- did not move. See ADR-0004.

The scoring rules, verbatim in effect:

* Regular victim +5, bonus victim +15, fire +10; each target scores **once**, no matter how
  many drones land on it.
* Scoring requires a **landed** drone within **1.0 m** of the organiser-set target position
  **and** in line of sight -- "no walls or pillars on the line" (3.3.4 r.1). Markers do not
  block; only structure does.
* A victim within **2.5 m** of a fire that is still burning at the end scores **zero** (3.4).
* One relay doubles the **total** mission score. A relay is a chain of landed drones from the
  drone that rescued a bonus victim to a drone inside the Start Area, every adjacent pair at
  most 1.0 m apart and in mutual line of sight (3.3.7). Two relays score the same as one.

The fire coupling and the relay are what make this more than a coverage benchmark. The fire
rule makes target *ordering* matter; the relay makes ~8-15 of your drones a committed
structure rather than searchers. Both are pure 2D geometry, which is precisely why a simulator
this cheap can answer real strategic questions about them.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping

import numpy as np

from .constants import (
    FIRE_SUPPRESSION_RADIUS_M,
    TAKEOFF_WAVE_WINDOW_S,
    POINTS_BONUS_VICTIM,
    POINTS_FIRE,
    POINTS_VICTIM,
    RELAY_MULTIPLIER,
    RELAY_SPACING_M,
    SCORE_RADIUS_M,
)
from .errors import ConfigError
from .sensors.raycast import segment_clear
from .world.arena import ArenaSpec, Target

__all__ = ["Event", "TargetState", "ScoreBreakdown", "Mission", "POINTS", "takeoff_waves"]

POINTS: Mapping[str, int] = {
    "victim": POINTS_VICTIM,
    "bonus_victim": POINTS_BONUS_VICTIM,
    "fire": POINTS_FIRE,
}

# Line of sight for scoring is evaluated at floor level: both the landed drone and the marker
# are on the ground, and every wall and pillar extends from the floor upward.
_SCORING_Z = 0.0


@dataclass(frozen=True)
class Event:
    """A discrete, typed thing that happened. R-OBS-6 requires all of these in the log."""

    tick: int
    sim_time_s: float
    kind: str
    agent_id: str | None = None
    detail: Mapping[str, Any] = field(default_factory=dict)


@dataclass
class TargetState:
    target: Target
    serviced_by: tuple[str, ...] = ()
    """Landed drones satisfying the range-and-line-of-sight condition."""

    serviced_tick: int | None = None

    @property
    def serviced(self) -> bool:
        return bool(self.serviced_by)


@dataclass(frozen=True)
class ScoreBreakdown:
    """The full arithmetic, so a number can always be explained."""

    per_target: Mapping[str, int]
    raw_total: int
    suppressed: tuple[str, ...]
    """Victims zeroed because they sit within 2.5 m of a still-burning fire."""

    relay_formed: bool
    relay_chain: tuple[str, ...]
    multiplier: float
    total: int

    def explain(self) -> str:
        lines = [f"raw {self.raw_total}"]
        if self.suppressed:
            lines.append(f"suppressed by fire: {', '.join(self.suppressed)}")
        lines.append(
            f"relay {'formed via ' + ' -> '.join(self.relay_chain) if self.relay_formed else 'not formed'}"
        )
        lines.append(f"x{self.multiplier:g} = {self.total}")
        return " | ".join(lines)


class Mission:
    """Tracks target servicing and computes the score. Pure function of landed-drone positions."""

    def __init__(self, arena: ArenaSpec) -> None:
        if not arena.targets:
            raise ConfigError("mission needs at least one target")
        self.arena = arena
        self._los_scene = arena.structural_scene()
        self.targets: dict[str, TargetState] = {
            t.id: TargetState(target=t) for t in arena.targets
        }
        self._target_xy = np.array([[t.x, t.y] for t in arena.targets])
        self._target_ids = [t.id for t in arena.targets]

    # -- per-tick update -------------------------------------------------------------------

    def update(
        self, tick: int, sim_time_s: float, landed: Mapping[str, np.ndarray]
    ) -> list[Event]:
        """Recompute which targets are serviced. ``landed`` maps agent id to its (x, y).

        Servicing is **latched**: once a target is serviced it stays serviced, and the set of
        drones credited only grows. Two reasons, and the second is the load-bearing one.

        A landed drone does not move (the runner freezes its velocity state on touchdown), so
        in normal operation recomputation and latching agree. But relying on that made scoring
        silently sensitive to a physics detail in a completely different module. Latching makes
        the rule match the competition's own semantics -- a rescue that has happened has
        happened -- and keeps the online result reproducible offline from the log (R-MISS-8).
        """
        events: list[Event] = []
        if not landed:
            return events

        agent_ids = list(landed)
        positions = np.array([landed[a] for a in agent_ids], dtype=float)

        for idx, target_id in enumerate(self._target_ids):
            state = self.targets[target_id]
            target_xy = self._target_xy[idx]

            distances = np.linalg.norm(positions - target_xy, axis=1)
            near = np.flatnonzero(distances <= SCORE_RADIUS_M)
            if not len(near):
                continue

            visible = segment_clear(
                self._los_scene,
                positions[near],
                np.tile(target_xy, (len(near), 1)),
                _SCORING_Z,
            )
            found = {agent_ids[i] for i, ok in zip(near, visible) if ok}
            servicing = tuple(sorted(set(state.serviced_by) | found))
            if servicing == state.serviced_by:
                continue

            was_serviced = state.serviced
            state.serviced_by = servicing
            if servicing and not was_serviced:
                state.serviced_tick = tick
                events.append(
                    Event(
                        tick=tick,
                        sim_time_s=sim_time_s,
                        kind="target_serviced",
                        agent_id=servicing[0],
                        detail={
                            "target_id": target_id,
                            "kind": state.target.kind,
                            "points": POINTS[state.target.kind],
                            "by": list(servicing),
                        },
                    )
                )
        return events

    # -- scoring ---------------------------------------------------------------------------

    def score(self, landed: Mapping[str, np.ndarray]) -> ScoreBreakdown:
        """Final score. Call at end of run; the fire coupling is only defined at the end."""
        per_target: dict[str, int] = {}
        suppressed: list[str] = []

        burning = [
            self.targets[t].target
            for t in self.targets
            if self.targets[t].target.kind == "fire" and not self.targets[t].serviced
        ]

        for target_id, state in self.targets.items():
            if not state.serviced:
                per_target[target_id] = 0
                continue
            points = POINTS[state.target.kind]
            if state.target.kind in ("victim", "bonus_victim") and self._near_burning_fire(
                state.target, burning
            ):
                suppressed.append(target_id)
                points = 0
            per_target[target_id] = points

        raw_total = sum(per_target.values())
        chain = self._find_relay(landed)
        multiplier = RELAY_MULTIPLIER if chain else 1.0

        return ScoreBreakdown(
            per_target=per_target,
            raw_total=raw_total,
            suppressed=tuple(sorted(suppressed)),
            relay_formed=bool(chain),
            relay_chain=tuple(chain),
            multiplier=multiplier,
            total=int(round(raw_total * multiplier)),
        )

    @staticmethod
    def _near_burning_fire(target: Target, burning: Iterable[Target]) -> bool:
        return any(
            np.hypot(target.x - fire.x, target.y - fire.y) <= FIRE_SUPPRESSION_RADIUS_M
            for fire in burning
        )

    # -- the relay ---------------------------------------------------------------------------

    def _find_relay(self, landed: Mapping[str, np.ndarray]) -> list[str]:
        """Shortest landed-drone chain from a bonus-victim rescuer to the Start Area.

        Breadth-first over the graph whose edges are "at most 1 m apart and in mutual line of
        sight". Returns the chain, or an empty list if no relay exists. One relay is worth as
        much as the entire rest of the mission, so this is worth getting exactly right.
        """
        if not landed:
            return []

        agent_ids = list(landed)
        positions = np.array([landed[a] for a in agent_ids], dtype=float)
        index = {a: i for i, a in enumerate(agent_ids)}

        heads = sorted(
            {
                agent
                for state in self.targets.values()
                if state.target.kind == "bonus_victim" and state.serviced
                for agent in state.serviced_by
                if agent in index
            }
        )
        if not heads:
            return []

        tails = {
            a
            for a in agent_ids
            if self.arena.in_start_area(float(landed[a][0]), float(landed[a][1]))
        }
        if not tails:
            return []

        adjacency = self._relay_adjacency(positions)

        # BFS from every head at once; the first tail reached gives the shortest chain.
        previous: dict[int, int | None] = {index[h]: None for h in heads}
        queue: deque[int] = deque(previous)
        while queue:
            node = queue.popleft()
            if agent_ids[node] in tails:
                chain: list[str] = []
                cursor: int | None = node
                while cursor is not None:
                    chain.append(agent_ids[cursor])
                    cursor = previous[cursor]
                return list(reversed(chain))
            for neighbour in np.flatnonzero(adjacency[node]):
                neighbour = int(neighbour)
                if neighbour not in previous:
                    previous[neighbour] = node
                    queue.append(neighbour)
        return []

    def _relay_adjacency(self, positions: np.ndarray) -> np.ndarray:
        """Boolean adjacency: within the spacing limit and in mutual line of sight."""
        n = len(positions)
        deltas = positions[:, None, :] - positions[None, :, :]
        close = np.linalg.norm(deltas, axis=2) <= RELAY_SPACING_M
        np.fill_diagonal(close, False)
        if not close.any():
            return close

        i_idx, j_idx = np.nonzero(np.triu(close))
        visible = segment_clear(
            self._los_scene, positions[i_idx], positions[j_idx], _SCORING_Z
        )
        adjacency = np.zeros((n, n), dtype=bool)
        adjacency[i_idx[visible], j_idx[visible]] = True
        adjacency |= adjacency.T
        return adjacency

    # -- queries ----------------------------------------------------------------------------

    def summary(self) -> dict[str, Any]:
        return {
            target_id: {
                "kind": state.target.kind,
                "x": state.target.x,
                "y": state.target.y,
                "serviced": state.serviced,
                "serviced_by": list(state.serviced_by),
                "serviced_tick": state.serviced_tick,
            }
            for target_id, state in self.targets.items()
        }


def takeoff_waves(
    departure_times_s, window_s: float = TAKEOFF_WAVE_WINDOW_S
) -> list[list[float]]:
    """Group departure times into take-off waves.

    The rules define a "simultaneous take-off" as a group whose last drone leaves within 10 s
    of its first, and allow at most two such groups (booklet 3.3.2). This is deliberately a
    *pure function over recorded times* rather than something the runner enforces: the
    simulator's job is to report what the fleet did, and whether that broke a rule is a
    question about the run, answerable afterwards from the log. Enforcing it mid-flight made
    the platform a referee and hid the violation from the policy that caused it.

    Returns one list of times per wave. ``len(result) > MAX_TAKEOFF_WAVES`` is a rule
    violation; the caller decides what that means for the score.
    """
    times = sorted(float(t) for t in departure_times_s)
    if not times:
        return []
    waves: list[list[float]] = [[times[0]]]
    for t in times[1:]:
        if t - waves[-1][0] <= window_s:
            waves[-1].append(t)
        else:
            waves.append([t])
    return waves
