"""Metrics computed from a recorded run.

Two design choices here are deliberate corrections to problems recon found in the target
paper's evaluation, and they matter more than the metric list itself.

**Coverage is reported two ways, because they measure different things.**
``path_coverage`` counts grid cells the drone physically passed near -- the metric
arXiv:2607.25195 uses, with a radius equal to ir-sim's ``goal_threshold`` default of 0.1 m.
That is a *path-proximity* measure, not "area sensed", and it produces very small absolute
numbers. ``sensed_coverage`` counts cells actually swept by the ToF ring's line of sight. For
a search mission the second is the honest one; the first exists so results stay comparable to
the paper.

**Coverage is also normalised by live-agent-seconds.** Under ``collision_behaviour="stop"`` a
crashed drone contributes nothing for the rest of the episode, so raw coverage silently
rewards *not crashing* as much as it rewards *searching well*. Recon measured half a
four-drone team dead 35 s into a 900 s episode in the reference implementation, which means
its headline "better heading policy gives more coverage" is entangled with "fewer deaths give
more live agent-seconds". Dividing by live-agent-seconds separates the two.

**Collisions are counted as events and as distinct agents**, because the paper's metric counts
only distinct agents and therefore saturates at the fleet size -- its k=12 figures are "how
many of the 12 died", not a collision rate. Both are reported here so neither can be mistaken
for the other.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .api import Lifecycle
from .constants import DRONE_RADIUS_M
from .recorder import LIFECYCLE_NAMES, load_run
from .sensors.raycast import RayScene, cast_rays
from .world.arena import ArenaConfig, ArenaSpec, Pillar, Target, Wall

__all__ = ["RunMetrics", "compute_metrics", "arena_from_log", "summarise"]

_DEFAULT_GRID_M = 0.25
_PATH_COVERAGE_RADIUS_M = 0.1   # ir-sim's goal_threshold default; the paper's implicit radius


@dataclass(frozen=True)
class RunMetrics:
    """Everything worth comparing between two policies."""

    policy: str
    seed: int
    n_drones: int
    collision_behaviour: str

    score_total: int
    score_raw: int
    relay_formed: bool
    targets_serviced: int
    targets_total: int

    sim_time_s: float
    live_agent_seconds: float
    """Sum over drones of the time each spent airborne. The denominator that matters."""

    path_coverage: float
    sensed_coverage: float
    sensed_coverage_per_live_minute: float

    time_to_first_target_s: float | None
    time_to_last_target_s: float | None
    time_to_half_sensed_coverage_s: float | None

    crashed_agents: int
    crash_events: int
    landed_agents: int
    idle_agents: int
    rule_violations: int

    coverage_curve: tuple[tuple[float, float], ...] = field(repr=False, default=())
    """``(sim_time_s, sensed_coverage)`` samples, for plotting."""


def arena_from_log(header: Mapping[str, Any]) -> ArenaSpec:
    """Rebuild the arena from the recorded geometry, not from the seed.

    Regenerating from the seed would test the generator's determinism rather than reading what
    was actually simulated, and would hide a divergence between the two.
    """
    spec = header["arena"]
    return ArenaSpec(
        seed=spec["seed"],
        width_m=spec["width_m"],
        depth_m=spec["depth_m"],
        ceiling_m=spec["ceiling_m"],
        start_area_depth_m=spec["start_area_depth_m"],
        unknown_area=tuple(spec["unknown_area"]),
        walls=tuple(Wall(**w) for w in spec["walls"]),
        pillars=tuple(Pillar(**p) for p in spec["pillars"]),
        targets=tuple(Target(**t) for t in spec["targets"]),
        config=ArenaConfig(),
    )


def compute_metrics(
    directory: str | Path,
    grid_m: float = _DEFAULT_GRID_M,
    coverage_every: int = 20,
    sensor_range_m: float = 3.0,
) -> RunMetrics:
    """Compute every metric from a recorded run. Reads the log only."""
    run = load_run(directory)
    header, footer, events = run["header"], run["footer"], run["events"]
    states = run["states"]
    arena = arena_from_log(header)

    times = states["time_s"]
    pose = states["pose"]                     # (T, N, 4) -> x, y, z, theta
    lifecycle = states["lifecycle"]           # (T, N)
    n_ticks, n_agents = pose.shape[:2]
    dt = float(times[1] - times[0]) if n_ticks > 1 else 0.0

    free = ~arena.occupancy_grid(grid_m)
    n_free = int(free.sum())
    nx, ny = free.shape

    airborne_codes = {i for name, i in _codes().items() if name in Lifecycle.AIRBORNE}
    airborne = np.isin(lifecycle, list(airborne_codes))
    live_agent_seconds = float(airborne.sum() * dt)

    path_visited = np.zeros_like(free)
    sensed = np.zeros_like(free)
    curve: list[tuple[float, float]] = []

    radius_cells = max(int(round(_PATH_COVERAGE_RADIUS_M / grid_m)), 0)
    for t in range(n_ticks):
        active = np.flatnonzero(airborne[t])
        for i in active:
            ix = int(np.clip(pose[t, i, 0] / grid_m, 0, nx - 1))
            iy = int(np.clip(pose[t, i, 1] / grid_m, 0, ny - 1))
            lo_x, hi_x = max(0, ix - radius_cells), min(nx, ix + radius_cells + 1)
            lo_y, hi_y = max(0, iy - radius_cells), min(ny, iy + radius_cells + 1)
            path_visited[lo_x:hi_x, lo_y:hi_y] = True

        if t % coverage_every == 0 and len(active):
            _mark_sensed(sensed, arena, pose[t, active], free, grid_m, sensor_range_m)
            curve.append((float(times[t]), float((sensed & free).sum() / max(n_free, 1))))

    path_coverage = float((path_visited & free).sum() / max(n_free, 1))
    sensed_coverage = float((sensed & free).sum() / max(n_free, 1))
    live_minutes = live_agent_seconds / 60.0

    serviced = [e for e in events if e["kind"] == "target_serviced"]
    crash_events = [e for e in events if e["kind"] == "crashed"]
    violations = [e for e in events if e["kind"] == "rule_violation"]

    half = next((t for t, c in curve if c >= 0.5 * sensed_coverage and sensed_coverage > 0), None)
    lifecycles = footer["lifecycles"]

    return RunMetrics(
        policy=header["config"]["policy"],
        seed=header["seed"],
        n_drones=n_agents,
        collision_behaviour=header["config"]["collision_behaviour"],
        score_total=footer["score"]["total"],
        score_raw=footer["score"]["raw_total"],
        relay_formed=bool(footer["score"]["relay_formed"]),
        targets_serviced=sum(1 for v in footer["mission_summary"].values() if v["serviced"]),
        targets_total=len(footer["mission_summary"]),
        sim_time_s=float(times[-1]) if n_ticks else 0.0,
        live_agent_seconds=live_agent_seconds,
        path_coverage=path_coverage,
        sensed_coverage=sensed_coverage,
        sensed_coverage_per_live_minute=(
            sensed_coverage / live_minutes if live_minutes > 0 else 0.0
        ),
        time_to_first_target_s=serviced[0]["sim_time_s"] if serviced else None,
        time_to_last_target_s=serviced[-1]["sim_time_s"] if serviced else None,
        time_to_half_sensed_coverage_s=half,
        crashed_agents=sum(1 for s in lifecycles.values() if s == Lifecycle.CRASHED),
        crash_events=len(crash_events),
        landed_agents=sum(1 for s in lifecycles.values() if s == Lifecycle.LANDED),
        idle_agents=sum(1 for s in lifecycles.values() if s == Lifecycle.IDLE),
        rule_violations=len(violations),
        coverage_curve=tuple(curve),
    )


def _codes() -> dict[str, int]:
    return {name: code for code, name in LIFECYCLE_NAMES.items()}


def _mark_sensed(sensed, arena, poses, free, grid_m, sensor_range_m) -> None:
    """Mark cells within line of sight of any drone, out to the sensor range.

    Uses the same ray-casting engine the sensor itself uses, so "sensed" here means the same
    thing it means on board -- rather than a disc that quietly ignores walls.
    """
    scene = arena.structural_scene()
    nx, ny = free.shape
    n_rays = 64
    angles = np.arange(n_rays) * (2 * np.pi / n_rays)
    directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)

    for x, y, z, _theta in poses:
        origins = np.tile([x, y], (n_rays, 1))
        reach = cast_rays(scene, origins, directions, float(z), sensor_range_m)
        reach = np.where(np.isfinite(reach), reach, sensor_range_m)
        step = grid_m * 0.5
        for angle, distance in zip(angles, reach):
            n_samples = max(int(distance / step), 1)
            travel = np.arange(n_samples) * step
            ix = np.clip(((x + travel * np.cos(angle)) / grid_m).astype(int), 0, nx - 1)
            iy = np.clip(((y + travel * np.sin(angle)) / grid_m).astype(int), 0, ny - 1)
            sensed[ix, iy] = True


def summarise(metrics: list[RunMetrics]) -> str:
    """A compact comparison table, grouped by policy.

    Deliberately prints n, mean and spread rather than a single number: every arena is a draw
    from a distribution (the rulebook guarantees the layout is not given), so a single-seed
    result is not a result.
    """
    if not metrics:
        return "(no runs)"
    by_policy: dict[str, list[RunMetrics]] = {}
    for m in metrics:
        by_policy.setdefault(m.policy, []).append(m)

    header = (
        f"{'policy':<14}{'n':>3}{'score':>8}{'+/-':>7}{'sensed':>9}"
        f"{'/livemin':>10}{'crashed':>9}{'relay':>7}"
    )
    lines = [header, "-" * len(header)]
    for policy, runs in sorted(by_policy.items()):
        scores = np.array([r.score_total for r in runs], dtype=float)
        sensed = np.array([r.sensed_coverage for r in runs])
        per_min = np.array([r.sensed_coverage_per_live_minute for r in runs])
        crashed = np.array([r.crashed_agents for r in runs], dtype=float)
        relays = sum(r.relay_formed for r in runs)
        lines.append(
            f"{policy:<14}{len(runs):>3}{scores.mean():>8.1f}{scores.std():>7.1f}"
            f"{sensed.mean():>9.3f}{per_min.mean():>10.3f}"
            f"{crashed.mean():>9.1f}{relays:>7}"
        )
    return "\n".join(lines)
