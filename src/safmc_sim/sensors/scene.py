"""The world as a sensor sees it, and who is allowed to block what.

This is the only view of the world a sensor is ever handed (R-SENS-15). It carries two things:
geometry a ray can hit, and the landmarks placed in the arena. It does not carry the arena,
the mission, or any agent.

There are two distinct notions of "blocking" in this simulator and conflating them would
silently break scoring:

**Sensing** -- what a ToF ray or a camera can see through. Walls, pillars, solid landmarks
(mission markers) and other airborne drones all occlude.

**Line of sight, for scoring** -- the rulebook is specific: a rescue counts when "drawing a
straight line from the drone to the victim, there must be no walls or pillars on the line"
(SAFMC 2026 Cat Swarm 3.3.4 r.1). Markers do not block. Drones do not block. Only structure.

So the scoring scene lives with the mission (``ArenaSpec.structural_scene``) and never here,
because the type system then makes it impossible to pass the wrong one (R-MISS-2).

Live drone bodies are rebuilt from ir-sim's robot list once per tick, by the runner, after
``env.step()`` and before any sensor samples. That ordering is forced: ir-sim moves every
object and rebuilds its tree during the step, so anything read before it would be one tick
stale. The rebuild is cached by tick so N sensors on N drones cost one pass.
"""

from __future__ import annotations

from typing import Iterable

import numpy as np

from ..constants import DRONE_RADIUS_M
from ..errors import ConfigError
from ..world.landmark import Landmark, occluder_scene
from .raycast import RayScene

__all__ = ["WorldScene"]


class WorldScene:
    """Owns the static geometry and the landmarks; lazily assembles the per-tick bodies."""

    def __init__(
        self,
        structural: RayScene,
        landmarks: Iterable[Landmark] = (),
        drone_radius_m: float = DRONE_RADIUS_M,
    ) -> None:
        if structural.n_primitives == 0:
            raise ConfigError(
                "structural scene is empty -- an arena with no walls would let drones leave "
                "the world silently, because ir-sim has no implicit boundaries"
            )
        self._structural = structural
        self._landmarks = tuple(landmarks)
        self._drone_radius_m = float(drone_radius_m)

        # Solid landmarks join the static sensing scene as anonymous circles. Points do not
        # exist to a ray at all -- only to a sensor that asks for them by kind.
        self._static_sensing = self._structural.merged_with(occluder_scene(self._landmarks))
        self._cache_key: object = None
        self._drone_ids: np.ndarray = np.zeros((0,), dtype=int)
        self._drone_scene: RayScene = RayScene()

    @classmethod
    def from_arena(cls, arena) -> "WorldScene":
        """The sensing view of a resolved arena: its structure plus every landmark."""
        return cls(arena.structural_scene(), arena.all_landmarks)

    # -- landmarks -----------------------------------------------------------------------

    @property
    def landmarks(self) -> tuple[Landmark, ...]:
        """Every landmark in the arena, solid or not, targets included."""
        return self._landmarks

    def landmarks_of(self, *kinds: str) -> tuple[Landmark, ...]:
        """The landmarks whose kind is one of ``kinds``, in arena order."""
        wanted = set(kinds)
        return tuple(lm for lm in self._landmarks if lm.kind in wanted)

    # -- the ray-castable scenes ----------------------------------------------------------

    @property
    def static_sensing_scene(self) -> RayScene:
        """Structure plus solid landmarks, without live drone bodies."""
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

        ``cache_key`` is the world tick count. The runner calls this once per tick before any
        sensor samples; a second call with the same key is a no-op comparison.
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
        for i, robot in enumerate(robots):
            state = robot.state
            ids[i] = robot.id
            circles[i, 0] = float(state[0, 0])
            circles[i, 1] = float(state[1, 0])
            circles[i, 2] = self._drone_radius_m

        self._drone_ids = ids
        # Drones occlude each other at EVERY altitude, deliberately.
        #
        # An earlier version gave each drone a +/-5 cm vertical band, so a drone at 0.8 m did
        # not occlude a ray cast at 0.5 m. That was internally consistent and externally a
        # trap: collision is ir-sim's and strictly 2D, so two drones 0.7 m apart vertically
        # were mutually invisible on the ring and still collided. A new user reasonably read
        # altitude as a deconfliction axis and lost most of a fleet to it.
        #
        # What can kill you must be what you can see. Since collision is 2D, sensing of other
        # drones is 2D too. Landmarks keep their height gate because their collision is ours
        # and is height-gated to match.
        self._drone_scene = RayScene(
            circles=circles, circle_heights=np.full(n, np.inf)
        )
