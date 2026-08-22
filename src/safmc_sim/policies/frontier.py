"""Map-based frontier exploration -- the counterpart to the mapless strategies.

Mapless versus map-based is the comparison this simulator exists to make, so this policy is
built to be a fair opponent for SDLW rather than a strawman: it builds a real occupancy grid
from the same sensor, plans on **its own** map (never the simulator's ground truth), and flies
with the same VFH-style avoidance the real firmware uses.

The mapping is standard log-odds occupancy:

* Each valid ToF zone marks the cells along its ray as free and the cell at the hit as
  occupied. A ray with no return marks its whole length free out to the gate.
* Log-odds are clamped so the map can be revised when the world disagrees with it -- an
  unclamped grid saturates and stops learning, which shows up as a drone convinced a doorway
  is a wall.
* Cells never observed stay unknown, and unknown is what a frontier is made of.

Frontier selection is nearest-first with two adjustments that matter with 25 drones: a
frontier another drone has already claimed is skipped, and a chosen frontier is held until it
is reached or becomes known, because re-deciding every tick makes drones oscillate between two
equidistant frontiers and cover nothing.

With ``share_map: true`` (the default) drones publish their frontier claims through the
blackboard. Under v0.1's perfect-communication assumption that is free; under a real radio it
would not be. See ADR-0003.
"""

from __future__ import annotations

import numpy as np

from ..api import Command, Hold, Observation, VelocityBody, register_policy
from ..frames import wrap_pi
from .base import SearchPolicy, vfh_steer

__all__ = ["FrontierExplorer", "OccupancyMap"]

_LOG_ODDS_FREE = -0.4
_LOG_ODDS_OCCUPIED = 0.85
_LOG_ODDS_CLAMP = 4.0
_OCCUPIED_THRESHOLD = 0.5
_FREE_THRESHOLD = -0.5


class OccupancyMap:
    """A log-odds occupancy grid built from ToF rays. Owned by one policy instance."""

    def __init__(self, width_m: float, depth_m: float, resolution_m: float = 0.25) -> None:
        self.resolution_m = resolution_m
        self.nx = int(np.ceil(width_m / resolution_m))
        self.ny = int(np.ceil(depth_m / resolution_m))
        self.log_odds = np.zeros((self.nx, self.ny), dtype=float)
        self.observed = np.zeros((self.nx, self.ny), dtype=bool)

    def to_cell(self, x, y):
        return (
            np.clip((np.asarray(x) / self.resolution_m).astype(int), 0, self.nx - 1),
            np.clip((np.asarray(y) / self.resolution_m).astype(int), 0, self.ny - 1),
        )

    def integrate(self, obs: Observation, max_range_m: float) -> None:
        """Fold one ToF scan into the map."""
        origin = obs.pose.xy
        theta = obs.pose.theta
        ranges = obs.tof.ranges_m.reshape(-1)
        bearings = (obs.tof.zone_bearings_rad.reshape(-1) + theta).astype(float)

        hit = np.isfinite(ranges)
        # A no-return means free space all the way to the gate, which is information -- and
        # most of the information, since most zones see nothing most of the time.
        reach = np.where(hit, ranges, max_range_m)

        step = self.resolution_m * 0.5
        for bearing, distance, is_hit in zip(bearings, reach, hit):
            n_samples = max(int(distance / step), 1)
            travel = np.arange(n_samples) * step
            xs = origin[0] + travel * np.cos(bearing)
            ys = origin[1] + travel * np.sin(bearing)
            ix, iy = self.to_cell(xs, ys)
            self.log_odds[ix, iy] += _LOG_ODDS_FREE
            self.observed[ix, iy] = True
            if is_hit:
                hx, hy = self.to_cell(
                    origin[0] + distance * np.cos(bearing),
                    origin[1] + distance * np.sin(bearing),
                )
                # Undo the free update on the hit cell before marking it occupied, or a cell
                # at the end of many rays oscillates.
                self.log_odds[hx, hy] += _LOG_ODDS_OCCUPIED - _LOG_ODDS_FREE
                self.observed[hx, hy] = True

        np.clip(self.log_odds, -_LOG_ODDS_CLAMP, _LOG_ODDS_CLAMP, out=self.log_odds)

    @property
    def free(self) -> np.ndarray:
        return self.observed & (self.log_odds < _FREE_THRESHOLD)

    @property
    def occupied(self) -> np.ndarray:
        return self.observed & (self.log_odds > _OCCUPIED_THRESHOLD)

    @property
    def unknown(self) -> np.ndarray:
        return ~self.observed

    def frontiers(self) -> np.ndarray:
        """Free cells with at least one unknown 4-neighbour. Shape ``(n, 2)`` of indices."""
        free, unknown = self.free, self.unknown
        neighbour_unknown = np.zeros_like(unknown)
        neighbour_unknown[:-1, :] |= unknown[1:, :]
        neighbour_unknown[1:, :] |= unknown[:-1, :]
        neighbour_unknown[:, :-1] |= unknown[:, 1:]
        neighbour_unknown[:, 1:] |= unknown[:, :-1]
        return np.argwhere(free & neighbour_unknown)


@register_policy("frontier")
class FrontierExplorer(SearchPolicy):
    """Build a map, fly at the nearest unclaimed frontier, avoid what the ring sees."""

    def __init__(self, agent_id, config, rng, arena):
        super().__init__(agent_id, config, rng, arena)
        c = self.config
        self.map = OccupancyMap(
            arena.width_m, arena.depth_m, float(c.get("map_resolution_m", 0.25))
        )
        self.max_range_m = float(c.get("sensor_max_range_m", 3.0))
        self.clearance_m = float(c.get("clearance_m", 0.6))
        self.arrive_m = float(c.get("arrive_m", 0.5))
        self.share_map = bool(c.get("share_map", True))
        self.replan_ticks = int(c.get("replan_ticks", 20))
        self._goal: np.ndarray | None = None
        self._goal_tick = -(10**9)
        self._stuck_ticks = 0

    def reset(self) -> None:
        super().reset()
        self.map.log_odds[:] = 0.0
        self.map.observed[:] = False
        self._goal = None
        self._goal_tick = -(10**9)
        self._stuck_ticks = 0

    def choose_motion(self, obs: Observation) -> Command:
        self.map.integrate(obs, self.max_range_m)

        if self._goal is not None and np.linalg.norm(obs.pose.xy - self._goal) <= self.arrive_m:
            self._goal = None
        if self._goal is None or obs.tick - self._goal_tick >= self.replan_ticks:
            self._goal = self._pick_frontier(obs)
            self._goal_tick = obs.tick
            if self.share_map:
                self.publish(
                    "frontier_goal", None if self._goal is None else self._goal.tolist()
                )

        if self._goal is None:
            # Nothing left to explore from this drone's map. Sweep rather than stop: another
            # drone's unexplored region may still come into range.
            return VelocityBody(vx=self.cruise_speed_ms * 0.5, yaw_rate=0.3)

        delta = self._goal - obs.pose.xy
        desired = wrap_pi(float(np.arctan2(delta[1], delta[0])) - obs.pose.theta)
        bearing = vfh_steer(obs, desired, self.clearance_m)

        if bearing is None:
            self._stuck_ticks += 1
            if self._stuck_ticks > 40:
                # Boxed in for two seconds: abandon this goal, it is not reachable from here.
                self._goal = None
                self._stuck_ticks = 0
            return VelocityBody(vx=0.0, yaw_rate=1.0)

        self._stuck_ticks = 0
        return VelocityBody(
            vx=self.cruise_speed_ms * float(np.cos(bearing)),
            vy=self.cruise_speed_ms * float(np.sin(bearing)),
            yaw_rate=float(np.clip(2.0 * bearing, -1.5, 1.5)),
        )

    def _pick_frontier(self, obs: Observation) -> np.ndarray | None:
        cells = self.map.frontiers()
        if not len(cells):
            return None
        world = (cells + 0.5) * self.map.resolution_m
        distances = np.linalg.norm(world - obs.pose.xy, axis=1)

        if self.share_map:
            claimed = [
                np.asarray(values["frontier_goal"], dtype=float)
                for agent, values in obs.peers.items()
                if agent != self.agent_id and values.get("frontier_goal") is not None
            ]
            if claimed:
                claimed_xy = np.array(claimed)
                # Push away from frontiers someone else is already flying at, rather than
                # forbidding them outright -- a hard exclusion deadlocks when goals cluster.
                to_claimed = np.linalg.norm(
                    world[:, None, :] - claimed_xy[None, :, :], axis=2
                ).min(axis=1)
                distances = distances + np.clip(3.0 - to_claimed, 0.0, None) * 4.0

        return world[int(np.argmin(distances))]
