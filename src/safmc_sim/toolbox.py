"""Optional building blocks. **Nothing in the framework imports this module.**

These are examples to copy, adapt or ignore. They are deliberately kept out of the platform
because each one encodes a choice that the simulator exists to let you compare: how to reduce
the sensor, how to represent a map, how fast to climb. If any of them were in the runner or in
a base class, every policy would silently inherit that choice and two policies would no longer
be measuring their own ideas.

Read them, then write your own. That is the intended use.

    from safmc_sim.toolbox import body_to_world, ring_quadrants, OccupancyMap
"""

from __future__ import annotations

import numpy as np

from .api import Observation, Velocity

__all__ = [
    "body_to_world",
    "ring_quadrants",
    "climb",
    "descend",
    "OccupancyMap",
]


# ------------------------------------------------------------------------------------------
# Frames
# ------------------------------------------------------------------------------------------


def body_to_world(vx: float, vy: float, theta: float) -> tuple[float, float]:
    """Rotate a body-frame velocity into the ARENA frame ``Velocity`` expects.

    Reasoning from ToF ranges almost always means thinking in body frame -- "forward",
    "to my left" -- while the command primitive is world frame, because that is what the real
    ``mavlink_set_velocity_ned`` takes. This is the four lines in between, in the open rather
    than hidden in the runner.

        cmd = Velocity(*body_to_world(0.4, 0.0, obs.pose.theta))
    """
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    return (vx * cos_t - vy * sin_t, vx * sin_t + vy * cos_t)


# ------------------------------------------------------------------------------------------
# Sensor reduction
# ------------------------------------------------------------------------------------------


def ring_quadrants(obs: Observation) -> dict[str, float]:
    """Reduce the 8-ranger ring to front/left/right/back minima.

    Several published controllers -- including arXiv:2607.25195 -- assume a Crazyflie
    Multi-ranger deck: four single beams at 0, +90, -90 and 180 degrees. Our ring is strictly
    richer, so this projection is lossless in the directions those controllers care about, and
    it is the honest way to run their algorithm unmodified against our sensor.

    Missing returns come back as ``inf``. A caller that needs a finite number should saturate
    at its own assumed maximum -- that is an assumption about the sensor, so it belongs to the
    caller, not here.
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


# ------------------------------------------------------------------------------------------
# Trivial vertical manoeuvres
# ------------------------------------------------------------------------------------------


def climb(obs: Observation, target_m: float, rate_ms: float = 0.4) -> Velocity | None:
    """A ``Velocity`` that climbs toward ``target_m``, or ``None`` once there.

    Deliberately not a controller: it is a sign test. ``None`` means "you have arrived, decide
    what to do next" rather than silently holding altitude, because altitude hold is a policy
    decision and the platform does not make it for you.
    """
    gap = target_m - obs.pose.z
    if abs(gap) < 0.02:
        return None
    return Velocity(vz=rate_ms if gap > 0 else -rate_ms)


def descend(obs: Observation, rate_ms: float = 0.4) -> Velocity:
    """A ``Velocity`` that descends. Pair with ``Land`` once you are near the floor."""
    return Velocity(vz=-abs(rate_ms))


# ------------------------------------------------------------------------------------------
# Mapping
# ------------------------------------------------------------------------------------------

_LOG_ODDS_FREE = -0.4
_LOG_ODDS_OCCUPIED = 0.85
_LOG_ODDS_CLAMP = 4.0
_OCCUPIED_THRESHOLD = 0.5
_FREE_THRESHOLD = -0.5


class OccupancyMap:
    """A log-odds occupancy grid built from ToF rays. One per policy instance.

    Textbook, and included as a starting point rather than as *the* mapper -- mapping quality
    is one of the things this simulator exists to let you compare, so treat this as the
    baseline you are trying to beat.

    Each valid zone marks the cells along its ray free and the cell at the hit occupied. A ray
    with no return marks its whole length free out to the gate, which is where most of the
    information actually comes from. Log-odds are clamped so the map can be revised when the
    world disagrees with it; an unclamped grid saturates and stops learning, which shows up as
    a drone convinced a doorway is a wall.
    """

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
        ranges = obs.tof.ranges_m.reshape(-1)
        bearings = (obs.tof.zone_bearings_rad.reshape(-1) + obs.pose.theta).astype(float)

        hit = np.isfinite(ranges)
        reach = np.where(hit, ranges, max_range_m)
        step = self.resolution_m * 0.5

        for bearing, distance, is_hit in zip(bearings, reach, hit):
            n_samples = max(int(distance / step), 1)
            travel = np.arange(n_samples) * step
            ix, iy = self.to_cell(
                origin[0] + travel * np.cos(bearing), origin[1] + travel * np.sin(bearing)
            )
            self.log_odds[ix, iy] += _LOG_ODDS_FREE
            self.observed[ix, iy] = True
            if is_hit:
                hx, hy = self.to_cell(
                    origin[0] + distance * np.cos(bearing),
                    origin[1] + distance * np.sin(bearing),
                )
                # Undo the free update on the hit cell first, or a cell at the end of many
                # rays oscillates between free and occupied.
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
