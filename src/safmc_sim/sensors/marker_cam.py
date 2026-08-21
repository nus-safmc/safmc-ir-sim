"""Marker detection -- the geometric stand-in for the AprilTag pipeline.

The real drone runs an AprilTag detector (tag16h5, 0.12 m tags) on QVGA grayscale frames from
an OV2640 pitched 45 degrees nose-down, gated by ``hamming <= 1``, ``decision_margin > 55``
and a pose-fit error below 0.5. Nominal loop rate is 10 Hz; the firmware's own notes measure
**~2 Hz** in practice, because the detector task runs at the lowest priority in the system.

This models the *geometry* of that: range, field of view, and occlusion. It does not model
image formation, so there are no false positives, no missed detections at oblique angles, and
no motion blur. Divergences F-5 and F-6 in docs/FIDELITY.md.

Reporting the marker's ``kind`` alongside its id is realistic rather than cheating: the
markers are team-supplied and the team assigns tag ids by role -- the flown configuration
reserves ids 0-11 for landing targets with 8-11 as bonus victims (laptop/setup.yaml:86-103).
A drone that reads a tag id therefore does know what it is looking at.

**Assumptions A-4 and A-5 live here and they matter more than they look.** Detection range
sets how much area a drone sweeps per metre flown, which is the dominant term in any
search-policy comparison. Nothing in any repository measures it. Measure it before believing
a headline number from this simulator.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..constants import MARKER_DETECT_FOV_RAD, MARKER_DETECT_RANGE_M
from ..errors import ConfigError
from ..frames import wrap_pi
from .raycast import RayScene, segment_clear

__all__ = ["MarkerCamConfig", "MarkerDetection", "MarkerCam"]


@dataclass(frozen=True)
class MarkerCamConfig:
    max_range_m: float = MARKER_DETECT_RANGE_M   # A-4
    fov_rad: float = MARKER_DETECT_FOV_RAD       # A-5
    bearing_offset_rad: float = 0.0
    """Where the camera points in the body frame. Zero is straight ahead."""

    def __post_init__(self) -> None:
        if self.max_range_m <= 0:
            raise ConfigError(f"max_range_m must be > 0, got {self.max_range_m}")
        if not 0 < self.fov_rad <= 2 * np.pi:
            raise ConfigError(f"fov_rad must be in (0, 2pi], got {self.fov_rad}")


@dataclass(frozen=True)
class MarkerDetection:
    """One marker seen this tick, in the frame a real detector would report it."""

    marker_id: str
    kind: str
    range_m: float
    bearing_rad: float
    """Body-frame bearing, CCW from the nose."""


class MarkerCam:
    """Geometric marker detector. Not an ir-sim sensor -- the runner drives it directly.

    It is kept out of ir-sim's sensor list deliberately: it needs the mission's target list,
    which is not world geometry, and wiring mission state into an ir-sim sensor would be the
    first crack in the rule that policies cannot reach ground truth.
    """

    def __init__(self, config: MarkerCamConfig | None = None) -> None:
        self.config = config or MarkerCamConfig()

    def detect(
        self,
        pose_xy: np.ndarray,
        theta: float,
        z: float,
        targets,
        occlusion_scene: RayScene,
    ) -> tuple[MarkerDetection, ...]:
        """Return every marker currently visible from ``pose_xy``.

        ``occlusion_scene`` should contain everything that can hide a marker: walls, pillars,
        and the other markers. A marker occluding itself is handled by testing line of sight
        to its near surface rather than its centre.
        """
        if not targets:
            return ()
        cfg = self.config
        origin = np.asarray(pose_xy, dtype=float).reshape(2)

        centres = np.array([[t.x, t.y] for t in targets])
        radii = np.array([t.radius_m for t in targets])
        delta = centres - origin
        distances = np.linalg.norm(delta, axis=1)

        # Range is measured to the near surface, which is what a detector locking onto the
        # tag face would report, and keeps a marker the drone is touching from reading as
        # "distance 0.15 m to the centre of a solid object".
        surface = np.maximum(distances - radii, 0.0)
        bearings = wrap_pi(np.arctan2(delta[:, 1], delta[:, 0]) - theta - cfg.bearing_offset_rad)

        candidate = (surface <= cfg.max_range_m) & (np.abs(bearings) <= cfg.fov_rad / 2.0)
        # A marker only exists above the floor up to its own height; a drone above it sees
        # nothing. Consistent with the ToF ring's height gating.
        candidate &= np.array([z < t.height_m for t in targets])
        if not candidate.any():
            return ()

        idx = np.flatnonzero(candidate)
        # Stop just short of the surface so the target itself is not counted as its occluder.
        margin = 1e-3
        reach = np.maximum(surface[idx] - margin, 0.0)
        unit = delta[idx] / np.maximum(distances[idx], 1e-12)[:, None]
        endpoints = origin + unit * reach[:, None]
        clear = segment_clear(
            occlusion_scene, np.tile(origin, (len(idx), 1)), endpoints, z
        )

        return tuple(
            MarkerDetection(
                marker_id=targets[i].id,
                kind=targets[i].kind,
                range_m=float(surface[i]),
                bearing_rad=float(bearings[i]),
            )
            for i, ok in zip(idx, clear)
            if ok
        )
