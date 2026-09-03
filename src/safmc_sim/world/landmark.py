"""Landmarks: things placed in the world for sensors to find.

Structure -- walls and pillars -- is what a drone must not hit. A *landmark* is everything
else that is deliberately in the arena: a mission marker with an AprilTag on its face, a
surveyed navigation tag on a wall, a start-point mark painted on the floor, a radio anchor in
a corner. A landmark has a position and a kind. Whether it has a body is up to it.

Two rules, and the second is the one that keeps policies honest.

**A landmark is solid if it has both a footprint and a height.** Solid landmarks occlude
ranging -- height-gated, so a 1.0 m marker hides the wall behind it from a drone at 0.5 m but
not from one at 1.2 m (R-SENS-6) -- and they can be struck. Everything else is a point: a tag
painted on a wall does not block a time-of-flight ray, and a drone cannot crash into a radio
anchor's signal. The same predicate, :attr:`Landmark.solid`, decides both, so what can kill
you is exactly what the ring can see.

**A landmark reaches a policy only through a sensor's geometric query.** The runner never
hands the landmark list to a policy. The ToF ring sees solid landmarks as anonymous circles.
The marker camera reports id, kind, range and bearing for the kinds it is configured to detect
and nothing for the rest. A UWB anchor is invisible to a camera because a camera does not
detect radio -- not because someone remembered to hide it. This is R-SENS-11 restated from the
world's side.

Mission targets are landmarks with scoring semantics; see
:class:`~safmc_sim.world.arena.Target`. Everything else is placed with
``ArenaConfig(landmarks=...)`` or ``dataclasses.replace(arena, landmarks=...)``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from ..errors import ConfigError
from ..sensors.raycast import RayScene

__all__ = ["Landmark", "occluder_scene"]


@dataclass(frozen=True)
class Landmark:
    """Something placed in the arena that a sensor may perceive. Not structure.

    ``radius_m`` is the footprint and ``height_m`` the top of the body above the floor. Both
    default to zero, which makes the landmark a point: a tag on a wall, an anchor. A footprint
    with zero height is a flat mark on the floor -- it has an extent but no body. Give it both
    to make it a body that occludes rays and can be hit.
    """

    id: str
    kind: str
    x: float
    y: float
    radius_m: float = 0.0
    height_m: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.id, str) or not self.id:
            raise ConfigError(f"landmark id must be a non-empty string, got {self.id!r}")
        if not isinstance(self.kind, str) or not self.kind:
            raise ConfigError(f"landmark {self.id!r}: kind must be a non-empty string")
        if self.radius_m < 0.0 or self.height_m < 0.0:
            raise ConfigError(
                f"landmark {self.id!r}: radius_m and height_m must be >= 0, "
                f"got {self.radius_m} and {self.height_m}"
            )
        if self.height_m > 0.0 and self.radius_m <= 0.0:
            raise ConfigError(
                f"landmark {self.id!r} has height_m={self.height_m} but no footprint. "
                f"height_m is the top of a body, and a point has no body -- give it a "
                f"radius_m, or drop the height."
            )

    @property
    def solid(self) -> bool:
        """True if this landmark occludes rays and can be collided with."""
        return self.radius_m > 0.0 and self.height_m > 0.0

    @property
    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y])


def occluder_scene(landmarks: Iterable[Landmark]) -> RayScene:
    """The ray-castable view of the solid landmarks. Points contribute nothing.

    Every circle stands on the floor, so ``circle_z_min`` keeps its default of zero and the
    height gate alone decides whether a ray at altitude ``z`` is blocked.
    """
    solid = [lm for lm in landmarks if lm.solid]
    if not solid:
        return RayScene()
    return RayScene(
        circles=np.array([[lm.x, lm.y, lm.radius_m] for lm in solid], dtype=float),
        circle_heights=np.array([lm.height_m for lm in solid], dtype=float),
    )
