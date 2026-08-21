"""Minimal reference policies. Read these first if you are writing your own."""

from __future__ import annotations

import numpy as np

from ..api import Command, Hold, Observation, Policy, Takeoff, VelocityBody, register_policy
from ..frames import wrap_pi
from .base import SearchPolicy, vfh_steer

__all__ = ["HoldPolicy", "RandomWalk", "WallFollow"]


@register_policy("hold")
class HoldPolicy(Policy):
    """Take off and sit there. The null baseline -- any strategy must beat this."""

    def step(self, obs: Observation) -> Command:
        return Takeoff() if obs.lifecycle == "IDLE" else Hold()


@register_policy("random_walk")
class RandomWalk(SearchPolicy):
    """Fly straight until something is close, then turn to a random free bearing.

    The cheapest honest baseline for a mapless strategy. Anything more sophisticated should be
    compared against this before it is compared against nothing.
    """

    def __init__(self, agent_id, config, rng, arena):
        super().__init__(agent_id, config, rng, arena)
        self.clearance_m = float(self.config.get("clearance_m", 0.6))

    def choose_motion(self, obs: Observation) -> Command:
        bearing = vfh_steer(obs, 0.0, self.clearance_m)
        if bearing is None:
            return VelocityBody(vx=0.0, yaw_rate=1.0)
        if abs(bearing) < 1e-6:
            return VelocityBody(vx=self.cruise_speed_ms)
        # Something ahead: commit to a random free direction rather than always the nearest,
        # which would make every drone hug the same side of every obstacle.
        jitter = float(self.rng.normal(0.0, 0.4))
        return VelocityBody(
            vx=self.cruise_speed_ms * 0.5,
            yaw_rate=float(np.clip(2.0 * wrap_pi(bearing + jitter), -1.5, 1.5)),
        )


@register_policy("wall_follow")
class WallFollow(SearchPolicy):
    """Keep a wall at a fixed distance on one side.

    Included because the Known Search Area, once the Unknown room is placed, is close to a
    2 m corridor ring -- and in a corridor, wall following is very hard to beat. It is the
    strategy the arena's own geometry suggests. See ``world/arena.py``.
    """

    def __init__(self, agent_id, config, rng, arena):
        super().__init__(agent_id, config, rng, arena)
        self.standoff_m = float(self.config.get("standoff_m", 0.8))
        self.side = 1.0 if self.config.get("side", "left") == "left" else -1.0
        self.gain = float(self.config.get("gain", 1.5))

    def choose_motion(self, obs: Observation) -> Command:
        ranges = obs.tof.ranges_m
        n = ranges.shape[0]
        side_index = int(round(n / 4)) if self.side > 0 else int(round(3 * n / 4))
        side_range = float(np.min(ranges[side_index]))
        front = float(np.min(ranges[0]))

        if np.isfinite(front) and front < self.standoff_m:
            return VelocityBody(vx=0.05, yaw_rate=-self.side * 1.2)
        if not np.isfinite(side_range):
            # Lost the wall: arc gently toward the side we are following to find it again.
            return VelocityBody(vx=self.cruise_speed_ms, yaw_rate=self.side * 0.4)
        error = side_range - self.standoff_m
        return VelocityBody(
            vx=self.cruise_speed_ms,
            yaw_rate=float(np.clip(-self.side * self.gain * error, -1.0, 1.0)),
        )
