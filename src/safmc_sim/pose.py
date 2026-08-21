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

__all__ = ["PoseSource", "GroundTruthPose", "NoisyPose"]


class PoseSource(ABC):
    @abstractmethod
    def pose_of(self, agent_id: str, state: np.ndarray, tick: int) -> Pose:
        """Produce the pose a policy will see. ``state`` is the true 6-row Quad25D state."""

    def reset(self) -> None: ...


class GroundTruthPose(PoseSource):
    """Exact pose. The v0.1 default."""

    def pose_of(self, agent_id: str, state: np.ndarray, tick: int) -> Pose:
        return Pose(
            x=float(state[0, 0]),
            y=float(state[1, 0]),
            z=float(state[3, 0]),
            theta=wrap_pi(float(state[2, 0])),
        )


class NoisyPose(PoseSource):
    """A random-walk drift model, provided as the worked example of the seam.

    Deliberately simple and deliberately **not** the default: it is a placeholder for a real
    odometry model, not a claim about the airframe. Drift accumulates per agent as a random
    walk in position and heading, which is the qualitative behaviour of integrating optical
    flow without loop closure -- unbounded, and worse the longer you fly.

    Do not quote numbers from this without measuring the real drift first (see A-2, A-4).
    """

    def __init__(
        self,
        rng: np.random.Generator,
        position_drift_ms: float = 0.01,
        heading_drift_rads: float = 0.002,
        tick_s: float = 0.05,
    ) -> None:
        self._rng = rng
        self._pos_sigma = position_drift_ms * np.sqrt(tick_s)
        self._yaw_sigma = heading_drift_rads * np.sqrt(tick_s)
        self._drift: dict[str, np.ndarray] = {}

    def reset(self) -> None:
        self._drift.clear()

    def pose_of(self, agent_id: str, state: np.ndarray, tick: int) -> Pose:
        drift = self._drift.get(agent_id)
        if drift is None:
            drift = np.zeros(3)
            self._drift[agent_id] = drift
        drift += self._rng.normal(
            0.0, [self._pos_sigma, self._pos_sigma, self._yaw_sigma]
        )
        return Pose(
            x=float(state[0, 0]) + drift[0],
            y=float(state[1, 0]) + drift[1],
            z=float(state[3, 0]),
            theta=wrap_pi(float(state[2, 0]) + drift[2]),
        )
