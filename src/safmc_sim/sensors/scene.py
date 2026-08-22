"""The ray-castable view of the world, and who is allowed to block what.

There are two distinct notions of "blocking" in this simulator and conflating them would
silently break scoring:

**Sensing** -- what a ToF ray or a camera can see through. Walls, pillars, mission markers
and other airborne drones all occlude.

**Line of sight, for scoring** -- the rulebook is specific: a rescue counts when "drawing a
straight line from the drone to the victim, there must be no walls or pillars on the line"
(SAFMC 2026 Cat Swarm 3.3.4 r.1). Markers do not block. Drones do not block. Only structure.

So this module keeps them as separate scenes rather than one scene with flags, because the
type system then makes it impossible to pass the wrong one (R-MISS-2).

Live drone bodies are pulled from ir-sim at the moment a sensor asks, not pushed in by the
runner. That ordering is forced: ir-sim moves every object, rebuilds its tree, and only then
steps every sensor (env_base.py:316-331). Anything the runner injected before ``env.step()``
would be one tick stale. The result is cached per tick so N sensors cost one rebuild.
"""

from __future__ import annotations

import numpy as np

from ..constants import DRONE_RADIUS_M
from ..errors import ConfigError
from .raycast import RayScene

__all__ = ["WorldScene", "DRONE_BODY_HALF_HEIGHT_M"]

# Vertical half-extent of a drone body for occlusion purposes. A quadrotor is a thin disc;
# this is what stops a drone at 0.8 m from occluding a ray cast at 0.5 m.
DRONE_BODY_HALF_HEIGHT_M = 0.05


class WorldScene:
    """Owns the static geometry and lazily assembles the per-tick dynamic geometry."""

    def __init__(
        self,
        structural: RayScene,
        markers: RayScene | None = None,
        drone_radius_m: float = DRONE_RADIUS_M,
    ) -> None:
        if structural.n_primitives == 0:
            raise ConfigError(
                "structural scene is empty -- an arena with no walls would let drones leave "
                "the world silently, because ir-sim has no implicit boundaries"
            )
        self._structural = structural
        self._markers = markers if markers is not None else RayScene()
        self._drone_radius_m = float(drone_radius_m)

        self._static_sensing = self._structural.merged_with(self._markers)
        self._cache_key: object = None
        self._drone_ids: np.ndarray = np.zeros((0,), dtype=int)
        self._drone_scene: RayScene = RayScene()

    # -- the two scenes ------------------------------------------------------------------

    @property
    def static_sensing_scene(self) -> RayScene:
        """Structure plus mission markers, without live drone bodies."""
        return self._static_sensing

    def sensing_scene(self, exclude_object_id: int | None = None) -> RayScene:
        """Everything a ray can hit, optionally omitting one drone's own body.

        A sensor must not see the drone carrying it. ir-sim's own lidar does the same thing by
        skipping ``obj._id == self.obj_id``.
        """
        if not len(self._drone_ids):
            return self._static_sensing
        if exclude_object_id is None:
            return self._static_sensing.merged_with(self._drone_scene)

        keep = self._drone_ids != exclude_object_id
        if keep.all():
            return self._static_sensing.merged_with(self._drone_scene)
        subset = RayScene(
            circles=self._drone_scene.circles[keep],
            circle_heights=self._drone_scene.circle_heights[keep],
            circle_z_min=self._drone_scene.circle_z_min[keep],
        )
        return self._static_sensing.merged_with(subset)

    # -- per-tick dynamic bodies ---------------------------------------------------------

    def refresh_drones(self, robots, cache_key: object) -> None:
        """Rebuild the live drone bodies, at most once per ``cache_key``.

        ``cache_key`` is the world tick count. Sensors all call this; the first one in a tick
        does the work and the rest are a no-op comparison.
        """
        if cache_key is not None and cache_key == self._cache_key:
            return
        self._cache_key = cache_key

        n = len(robots)
        if n == 0:
            self._drone_ids = np.zeros((0,), dtype=int)
            self._drone_scene = RayScene()
            return

        ids = np.empty(n, dtype=int)
        circles = np.empty((n, 3), dtype=float)
        z_centre = np.empty(n, dtype=float)
        for i, robot in enumerate(robots):
            state = robot.state
            ids[i] = robot.id
            circles[i, 0] = float(state[0, 0])
            circles[i, 1] = float(state[1, 0])
            circles[i, 2] = self._drone_radius_m
            # Row 3 is altitude in our Quad25D state. A robot that is not running Quad25D has
            # no altitude, which configure_robots() has already made impossible.
            z_centre[i] = float(state[3, 0]) if state.shape[0] > 3 else 0.0

        self._drone_ids = ids
        self._drone_scene = RayScene(
            circles=circles,
            circle_heights=z_centre + DRONE_BODY_HALF_HEIGHT_M,
            circle_z_min=z_centre - DRONE_BODY_HALF_HEIGHT_M,
        )
