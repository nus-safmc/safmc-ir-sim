"""Shared reactive marker pursuit and landing behavior for search policies."""

from __future__ import annotations

import numpy as np

from ..api import Land, Observation, Velocity
from ..frames import wrap_pi
from ..toolbox import body_to_world


class MissionWrapper:
    """Pursue observed mission markers and land within the scoring margin."""

    def __init__(
        self,
        *,
        land_range_m: float = 0.75,
        approach_speed_ms: float = 0.25,
        yaw_p_gain: float = 3.0,
        max_yaw_rate: float = 1.5,
    ) -> None:
        self.land_range_m = land_range_m
        self.approach_speed_ms = approach_speed_ms
        self.yaw_p_gain = yaw_p_gain
        self.max_yaw_rate = max_yaw_rate

    def command(self, obs: Observation) -> Velocity | Land | None:
        rank = {"fire": 0, "bonus_victim": 1, "victim": 2}
        markers = [marker for marker in obs.markers if marker.kind in rank]
        if not markers:
            return None
        marker = min(markers, key=lambda item: (rank[item.kind], item.range_m))
        if marker.range_m <= self.land_range_m:
            return Land()
        vx, vy = body_to_world(self.approach_speed_ms, 0.0, obs.pose.theta)
        yaw_rate = float(np.clip(
            self.yaw_p_gain * wrap_pi(marker.bearing_rad),
            -self.max_yaw_rate,
            self.max_yaw_rate,
        ))
        return Velocity(vx=vx, vy=vy, yaw_rate=yaw_rate)
