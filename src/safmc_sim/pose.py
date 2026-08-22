"""Where a policy's idea of "where am I" comes from -- and the seam for making it wrong.

v0.1 ships ground truth by team decision (ADR-0003). R-SEAM-1 requires that this is the
*only* producer of the pose in an ``Observation``, so that a drifting implementation is one
new class and no policy changes.

Why this seam is the important one. The competition's highest-value targets sit in the Unknown
Search Area, where the rules forbid placing any navigation aid and forbid teams from ever
entering the room. Localisation there is dead reckoning plus onboard sensing. Meanwhile the
real firmware has no filter of any kind: seeing a surveyed tag **hard-overwrites** the frame
offset with no gain, no covariance and no outlier rejection (esp-everything/main/odom.c:118-148).

So a result obtained on ground-truth pose has not been tested in the regime that decides the
score. That is a fine place to start and a bad place to stop.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from .api import Pose
from .frames import wrap_pi

__all__ = ["PoseSource", "GroundTruthPose", "POSE_SOURCES"]


class PoseSource(ABC):
    @abstractmethod
    def pose_of(self, agent_id: str, state: np.ndarray, tick: int) -> Pose:
        """Produce the pose a policy will see. ``state`` is the true 6-row Quad25D state."""

    def velocity_of(self, agent_id: str, state: np.ndarray, tick: int) -> tuple[float, float]:
        """Produce the velocity a policy will see.

        Part of the same seam as :meth:`pose_of`, and for the same reason: a drifting pose
        source that left velocity as ground truth would leak an exact motion signal a real
        drone does not have, and a policy could integrate it to recover the true position.
        The default is ground truth; override it alongside ``pose_of``.
        """
        return (float(state[4, 0]), float(state[5, 0]))

    def reset(self) -> None: ...


class GroundTruthPose(PoseSource):
    """Exact pose. The only implementation today."""

    def pose_of(self, agent_id: str, state: np.ndarray, tick: int) -> Pose:
        return Pose(
            x=float(state[0, 0]),
            y=float(state[1, 0]),
            z=float(state[3, 0]),
            theta=wrap_pi(float(state[2, 0])),
        )


# Selected by name from RunConfig, so the choice lands in the recorded log header. Adding a
# drifting implementation means writing one class and adding one line here -- that is the
# whole of the seam ADR-0003 promises.
POSE_SOURCES = {"ground_truth": lambda rng: GroundTruthPose()}
