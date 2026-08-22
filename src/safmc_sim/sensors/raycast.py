"""Vectorised, closed-form ray casting with altitude gating.

This replaces ir-sim's ``Lidar2D`` for all of our sensing. Three reasons, in the order they
matter (see docs/adr/0002-single-vectorised-tof-sensor.md):

1. **Shape.** ``Lidar2D`` casts a *contiguous* fan. The real drone carries eight rangers
   separated by 45 degrees. That is not a fan, and the only native workaround is eight sensor
   instances per drone.
2. **Cost.** ``Lidar2D`` is a shapely boolean difference of the beam fan against the obstacle
   union. Its cost is dominated by fixed per-sensor GEOS overhead rather than per-beam work,
   so eight one-beam instances is the most expensive possible way to buy sparse ranging.
   Profiling attributes 68% of ir-sim's total step time to one GEOS ``difference`` per lidar.
3. **Altitude.** Shapely is strictly 2D and ``ObjectBase.z`` is dead code that always returns
   zero. Height-gated occlusion -- a 1.0 m mission marker blocking a ray at 0.5 m cruise but
   not at 1.2 m -- is unobtainable from ``Lidar2D`` at any price. Here it is one comparison.

Everything is closed-form: ray-circle and ray-segment intersection, no polygonisation, so
results are exact to floating point rather than to a circle's tessellation.

Convention: a ray that starts *inside* a primitive returns the distance to where it exits.
That is the geometrically correct answer to "how far along this ray is the nearest surface",
and it is specifically the case where ``Lidar2D`` silently freezes its entire scan at the
previous tick's values (irsim/world/sensors/lidar2d.py:313-327). R-SENS-8.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..errors import ConfigError

__all__ = ["RayScene", "cast_rays", "segment_clear"]

# Rays whose direction is within this of parallel to a segment are treated as missing it.
# Chosen well above float64 noise but far below any angle a sensor can resolve.
_PARALLEL_EPS = 1e-12


@dataclass(frozen=True)
class RayScene:
    """A batch of ray-castable primitives, each carrying a height.

    Heights are what make this 2.5D. A primitive occludes a ray at altitude ``z`` only if
    ``height_m > z``. Structural obstacles (walls 1.5-2.0 m, pillars 2.0 m) are always taller
    than the 1.4 m ceiling and so always occlude; mission markers (1.0 m) occlude at the 0.5 m
    cruise altitude but not above 1.0 m.

    Circles are ``[cx, cy, r]``; segments are ``[x1, y1, x2, y2]``. Rectangles are decomposed
    into four segments by the arena builder -- there is no rectangle primitive here, because a
    segment is the only shape a wall needs to be.

    ``*_z_min`` defaults to zero, i.e. the primitive stands on the floor. Airborne bodies pass
    an explicit lower bound so that a drone at 0.8 m does not occlude a ray at 0.5 m.
    """

    circles: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), dtype=float))
    circle_heights: np.ndarray = field(default_factory=lambda: np.zeros((0,), dtype=float))
    segments: np.ndarray = field(default_factory=lambda: np.zeros((0, 4), dtype=float))
    segment_heights: np.ndarray = field(default_factory=lambda: np.zeros((0,), dtype=float))
    circle_z_min: np.ndarray | None = None
    segment_z_min: np.ndarray | None = None

    def __post_init__(self) -> None:
        for name, arr, cols in (
            ("circles", self.circles, 3),
            ("segments", self.segments, 4),
        ):
            if arr.ndim != 2 or (arr.size and arr.shape[1] != cols):
                raise ConfigError(f"{name} must have shape (N, {cols}), got {arr.shape}")
        if len(self.circles) != len(self.circle_heights):
            raise ConfigError(
                f"circles ({len(self.circles)}) and circle_heights "
                f"({len(self.circle_heights)}) length mismatch"
            )
        if len(self.segments) != len(self.segment_heights):
            raise ConfigError(
                f"segments ({len(self.segments)}) and segment_heights "
                f"({len(self.segment_heights)}) length mismatch"
            )
        if self.circles.size and np.any(self.circles[:, 2] <= 0.0):
            raise ConfigError("circle radii must be > 0")
        # z_min defaults to the floor, which is right for everything that stands on it: walls,
        # pillars, mission markers. Airborne bodies (other drones) pass an explicit band.
        object.__setattr__(
            self,
            "circle_z_min",
            np.zeros(len(self.circles)) if self.circle_z_min is None else np.asarray(self.circle_z_min, dtype=float),
        )
        object.__setattr__(
            self,
            "segment_z_min",
            np.zeros(len(self.segments)) if self.segment_z_min is None else np.asarray(self.segment_z_min, dtype=float),
        )
        if len(self.circle_z_min) != len(self.circles):
            raise ConfigError("circle_z_min length must match circles")
        if len(self.segment_z_min) != len(self.segments):
            raise ConfigError("segment_z_min length must match segments")

    @property
    def n_primitives(self) -> int:
        return len(self.circles) + len(self.segments)

    def merged_with(self, other: "RayScene") -> "RayScene":
        """Concatenate two scenes. Used to add mission markers or live drones to a static scene."""
        return RayScene(
            circles=np.vstack((self.circles, other.circles))
            if (self.circles.size or other.circles.size)
            else self.circles,
            circle_heights=np.concatenate((self.circle_heights, other.circle_heights)),
            segments=np.vstack((self.segments, other.segments))
            if (self.segments.size or other.segments.size)
            else self.segments,
            segment_heights=np.concatenate((self.segment_heights, other.segment_heights)),
            circle_z_min=np.concatenate((self.circle_z_min, other.circle_z_min)),
            segment_z_min=np.concatenate((self.segment_z_min, other.segment_z_min)),
        )


def _circle_hits(origins, directions, circles, active) -> np.ndarray:
    """Nearest non-negative ray-circle intersection distance. Returns (M, Nc), inf for misses."""
    m = len(origins)
    if not len(circles):
        return np.full((m, 0), np.inf)

    # oc points from the circle centre to the ray origin, so |oc| < r means "starting inside".
    oc = origins[:, None, :] - circles[None, :, :2]          # (M, Nc, 2)
    b = np.einsum("mk,mnk->mn", directions, oc)               # (M, Nc)
    c = np.einsum("mnk,mnk->mn", oc, oc) - circles[None, :, 2] ** 2

    disc = b * b - c
    real = disc >= 0.0
    root = np.sqrt(np.where(real, disc, 0.0))

    t_near = -b - root
    t_far = -b + root
    # Starting inside a primitive gives t_near < 0 <= t_far; the exit point is the answer.
    t = np.where(t_near >= 0.0, t_near, t_far)

    hit = real & (t >= 0.0) & active[None, :]
    return np.where(hit, t, np.inf)


def _segment_hits(origins, directions, segments, active) -> np.ndarray:
    """Ray-segment intersection distance. Returns (M, Ns), inf for misses."""
    m = len(origins)
    if not len(segments):
        return np.full((m, 0), np.inf)

    p1 = segments[:, :2]                                      # (Ns, 2)
    edge = segments[:, 2:] - p1                               # (Ns, 2)

    # Solve o + t*d = p1 + u*e for t (along the ray) and u (along the segment).
    #   t = (w x e) / (d x e),  u = (w x d) / (d x e),  with w = p1 - o
    denom = (
        directions[:, None, 0] * edge[None, :, 1]
        - directions[:, None, 1] * edge[None, :, 0]
    )                                                         # (M, Ns)
    w = p1[None, :, :] - origins[:, None, :]                  # (M, Ns, 2)

    parallel = np.abs(denom) < _PARALLEL_EPS
    safe_denom = np.where(parallel, 1.0, denom)

    t = (w[..., 0] * edge[None, :, 1] - w[..., 1] * edge[None, :, 0]) / safe_denom
    u = (
        w[..., 0] * directions[:, None, 1] - w[..., 1] * directions[:, None, 0]
    ) / safe_denom

    hit = (~parallel) & (t >= 0.0) & (u >= 0.0) & (u <= 1.0) & active[None, :]
    return np.where(hit, t, np.inf)


def _normalise(origins, directions):
    """Validate shapes and return unit-length directions."""
    origins = np.asarray(origins, dtype=float)
    directions = np.asarray(directions, dtype=float)
    if origins.ndim != 2 or origins.shape[1] != 2:
        raise ConfigError(f"origins must have shape (M, 2), got {origins.shape}")
    if directions.shape != origins.shape:
        raise ConfigError(f"directions {directions.shape} must match origins {origins.shape}")
    norms = np.linalg.norm(directions, axis=1, keepdims=True)
    if np.any(norms == 0.0):
        raise ConfigError("zero-length ray direction")
    return origins, directions / norms


def _cast_untruncated(scene: RayScene, origins, directions, z: float) -> np.ndarray:
    """Nearest hit distance per ray with no range limit. ``inf`` where nothing is hit.

    Both public entry points funnel through here so that range gating and line-of-sight
    testing can never disagree about what "hit" means.
    """
    # A primitive occludes a ray at altitude z only if the ray passes through its vertical
    # extent [z_min, height). This is the whole of the 2.5D model: mission markers are 1.0 m
    # tall so they block at the 0.5 m cruise altitude but not above; walls are 1.5-2.0 m so
    # they always block below the 1.4 m ceiling; a landed drone blocks nothing at cruise.
    circle_active = (scene.circle_heights > z) & (scene.circle_z_min <= z)
    segment_active = (scene.segment_heights > z) & (scene.segment_z_min <= z)

    best = np.full(len(origins), np.inf)
    if len(scene.circles):
        best = np.minimum(
            best, _circle_hits(origins, directions, scene.circles, circle_active).min(axis=1)
        )
    if len(scene.segments):
        best = np.minimum(
            best,
            _segment_hits(origins, directions, scene.segments, segment_active).min(axis=1),
        )
    return best


def cast_rays(
    scene: RayScene,
    origins: np.ndarray,
    directions: np.ndarray,
    z: float,
    max_range: float,
) -> np.ndarray:
    """Cast M rays and return the distance to the nearest occluding surface.

    Args:
        scene: primitives to test against.
        origins: ``(M, 2)`` ray origins in ARENA metres.
        directions: ``(M, 2)`` ray directions. Need not be normalised; a caller building
            directions from angles should not have to care.
        z: altitude of the rays. A primitive occludes only if its height exceeds this.
        max_range: rays are truncated here.

    Returns:
        ``(M,)`` distances, ``inf`` where nothing was hit inside ``max_range``. ``inf`` is
        used deliberately rather than ``max_range``: "no return" and "a surface at exactly
        max_range" are different facts, and the real sensor distinguishes them too via
        target_status 255.
    """
    if not np.isfinite(max_range) or max_range <= 0.0:
        raise ConfigError(f"max_range must be finite and > 0, got {max_range}")
    origins, directions = _normalise(origins, directions)
    best = _cast_untruncated(scene, origins, directions, z)
    return np.where(best <= max_range, best, np.inf)


def segment_clear(scene: RayScene, a: np.ndarray, b: np.ndarray, z: float) -> np.ndarray:
    """Line-of-sight test: is the straight segment from ``a`` to ``b`` unobstructed?

    This is the competition's scoring predicate, which requires "no walls or pillars on the
    line" (rulebook 3.3.4 r.1). The caller is responsible for passing a scene containing only
    the primitives allowed to block -- mission markers must not (R-MISS-2).

    Args:
        a: ``(M, 2)`` or ``(2,)`` start points.
        b: ``(M, 2)`` or ``(2,)`` end points.
        z: altitude at which to test.

    Returns:
        ``(M,)`` boolean array, True where the path is clear.
    """
    a = np.atleast_2d(np.asarray(a, dtype=float))
    b = np.atleast_2d(np.asarray(b, dtype=float))
    if a.shape != b.shape:
        raise ConfigError(f"a {a.shape} and b {b.shape} must have the same shape")

    delta = b - a
    distance = np.linalg.norm(delta, axis=1)
    # Coincident endpoints are trivially in line of sight, and normalising a zero vector is
    # undefined, so they are answered directly rather than cast.
    degenerate = distance <= 0.0
    safe_delta = np.where(degenerate[:, None], np.array([[1.0, 0.0]]), delta)

    origins, directions = _normalise(a, safe_delta)
    hits = _cast_untruncated(scene, origins, directions, z)

    # A target standing against a wall must still be visible, so a blocker has to be strictly
    # nearer than the endpoint, not merely at or beyond it.
    return degenerate | (hits >= distance)
