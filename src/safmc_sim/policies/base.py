"""Shared scaffolding for search policies.

Every competition policy has the same skeleton -- take off, search, land on what you find --
and differs only in *where to go next*. :class:`SearchPolicy` owns the skeleton so a new
strategy is one method, and so that two strategies being compared differ in exactly the thing
under test rather than in how carefully each remembered to handle take-off.

Subclass and implement :meth:`choose_motion`.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from ..api import Command, Hold, Land, Observation, Policy, Takeoff, VelocityBody
from ..constants import CRUISE_SPEED_MS, SCORE_RADIUS_M
from ..frames import wrap_pi

__all__ = ["SearchPolicy", "ring_quadrants", "vfh_steer"]


def ring_quadrants(obs: Observation) -> dict[str, float]:
    """Reduce the 8-ranger ring to front/left/right/back minima.

    Several published controllers -- including arXiv:2607.25195 -- assume a Crazyflie
    Multi-ranger deck: four single beams at 0, +90, -90 and 180 degrees. Our ring is strictly
    richer, so reducing it is lossless in the directions those controllers care about and is
    the honest way to run their algorithm unmodified.

    Missing returns come back as ``inf``, which is what "nothing within range" means. A caller
    that needs a finite number should saturate at its own assumed maximum, not here.
    """
    ranges = obs.tof.ranges_m
    n = ranges.shape[0]
    quarter = n // 4
    return {
        "front": float(np.min(ranges[0])),
        "left": float(np.min(ranges[quarter])),
        "back": float(np.min(ranges[2 * quarter])),
        "right": float(np.min(ranges[3 * quarter])),
    }


def vfh_steer(
    obs: Observation,
    desired_bearing: float,
    clearance_m: float,
    drone_radius_m: float = 0.18,
) -> float | None:
    """Pick a body-frame heading close to ``desired_bearing`` that is not blocked.

    A compact vector-field-histogram, deliberately shaped like the one the real firmware flies
    (vfh.c): build a polar density from the collapsed scan, widen each blocked bin by the
    drone's own radius so the drone does not clip a corner it can see past, then take the free
    bearing nearest the goal.

    Returns ``None`` when every direction is blocked, which the caller should treat as "stop
    and turn", not as "go straight".
    """
    collapsed = obs.tof.collapsed_m
    n_bins = len(collapsed)
    bin_width = 2.0 * np.pi / n_bins
    # Collapsed bin i covers clockwise angle [i, i+1) * bin_width, so its body-frame CCW
    # bearing is negative. Recovering it here keeps the policy in one angular convention.
    bearings = wrap_pi(-(np.arange(n_bins) + 0.5) * bin_width)

    blocked = np.isfinite(collapsed) & (collapsed < clearance_m)
    if blocked.any():
        # Widen each blocked bin by the angular half-width the drone actually needs.
        widened = np.zeros_like(blocked)
        for idx in np.flatnonzero(blocked):
            reach = max(collapsed[idx], 1e-3)
            half = np.arctan2(drone_radius_m, reach)
            span = int(np.ceil(half / bin_width))
            for offset in range(-span, span + 1):
                widened[(idx + offset) % n_bins] = True
        blocked = widened

    free = np.flatnonzero(~blocked)
    if not len(free):
        return None
    error = np.abs(wrap_pi(bearings[free] - desired_bearing))
    return float(bearings[free[int(np.argmin(error))]])


class SearchPolicy(Policy):
    """Take off, search, land on a target. Subclasses supply the search.

    Configuration keys understood here:

    ``land_range_m``
        Land when a detected marker is at most this far away. Default 0.6 m, comfortably
        inside the 1.0 m scoring radius so the drone still scores after drifting during
        descent.
    ``target_kinds``
        Which marker kinds are worth landing on. Default all three.
    ``cruise_speed_ms``
        Forward speed while searching. Default 0.45 m/s, the firmware's value.
    ``claim_targets``
        When true (default), a drone announces the target it is committing to on the
        blackboard and others ignore it. Landing is irreversible and each target scores once,
        so two drones landing on the same victim wastes one of them permanently.
    """

    def __init__(self, agent_id, config, rng, arena):
        super().__init__(agent_id, config, rng, arena)
        self.land_range_m = float(self.config.get("land_range_m", 0.6))
        self.cruise_speed_ms = float(self.config.get("cruise_speed_ms", CRUISE_SPEED_MS))
        self.target_kinds = tuple(
            self.config.get("target_kinds", ("victim", "bonus_victim", "fire"))
        )
        self.claim_targets = bool(self.config.get("claim_targets", True))
        self._claim: str | None = None

    def reset(self) -> None:
        self._claim = None

    # -- the skeleton ----------------------------------------------------------------------

    def step(self, obs: Observation) -> Command:
        if obs.lifecycle == "IDLE":
            return Takeoff(altitude_m=self.config.get("cruise_alt_m", 0.5))
        if obs.lifecycle != "FLYING":
            return Hold()

        target = self._pick_target(obs)
        if target is not None:
            if self.claim_targets and self._claim != target.marker_id:
                self._claim = target.marker_id
                self.publish("claimed_target", target.marker_id)
            if target.range_m <= self.land_range_m:
                return Land()
            return self._approach(obs, target)

        if self._claim is not None:
            self._claim = None
            self.publish("claimed_target", None)
        return self.choose_motion(obs)

    def choose_motion(self, obs: Observation) -> Command:
        """Where to go when nothing is in sight. This is the strategy."""
        raise NotImplementedError

    # -- helpers ----------------------------------------------------------------------------

    def _claimed_by_others(self, obs: Observation) -> set[str]:
        return {
            values["claimed_target"]
            for agent, values in obs.peers.items()
            if agent != self.agent_id and values.get("claimed_target")
        }

    def _pick_target(self, obs: Observation):
        """Nearest detected, unclaimed marker of an interesting kind."""
        if not obs.markers:
            return None
        taken = self._claimed_by_others(obs) if self.claim_targets else set()
        candidates = [
            m
            for m in obs.markers
            if m.kind in self.target_kinds
            and (m.marker_id not in taken or m.marker_id == self._claim)
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda m: m.range_m)

    def _approach(self, obs: Observation, target) -> Command:
        """Close on a marker without driving through anything on the way."""
        bearing = vfh_steer(obs, target.bearing_rad, clearance_m=0.5)
        if bearing is None:
            return VelocityBody(vx=0.0, yaw_rate=self.rng.choice([-1.0, 1.0]) * 0.8)
        # Slow down inside the scoring radius so the landing lands where it was aimed.
        speed = self.cruise_speed_ms * (0.4 if target.range_m < SCORE_RADIUS_M else 1.0)
        return VelocityBody(
            vx=speed * float(np.cos(bearing)),
            vy=speed * float(np.sin(bearing)),
            yaw_rate=float(np.clip(2.0 * bearing, -1.5, 1.5)),
        )
