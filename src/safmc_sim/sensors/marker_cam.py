"""Marker detection -- the geometric stand-in for the AprilTag pipeline.

The real drone runs an AprilTag detector (tag16h5, 0.12 m tags) on QVGA grayscale frames from
an OV2640 pitched 45 degrees nose-down, gated by ``hamming <= 1``, ``decision_margin > 55``
and a pose-fit error below 0.5. Nominal loop rate is 10 Hz; the firmware's own notes measure
**~2 Hz** in practice, because the detector task runs at the lowest priority in the system.

This models the *geometry* of that: range, field of view, and occlusion. It does not model
image formation, so there are no false positives, no missed detections at oblique angles, and
no motion blur. Divergences F-5 and F-6 in docs/FIDELITY.md. Other drones do not occlude it
(F-21): occlusion is tested against structure and solid landmarks only.

What it detects is a **landmark** -- anything placed in the arena with a kind this camera is
configured for. By default that is the three mission-marker kinds. The same detector on the
real drone also reads the surveyed navigation tags (ids 12-29 in ``laptop/setup.yaml``), and
those are one line of configuration away: place ``Landmark(kind="nav_tag", ...)`` in the arena
and add ``"nav_tag"`` to ``kinds``. Nothing else changes. See docs/06-sensors.md.

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
from typing import Sequence

import numpy as np

from ..constants import MARKER_DETECT_FOV_RAD, MARKER_DETECT_RANGE_M, MARKER_RATE_HZ
from ..errors import ConfigError
from ..frames import wrap_pi
from ..world.arena import TARGET_KINDS
from ..world.landmark import Landmark
from .base import Sensor, SensorConfig, TrueState
from .raycast import RayScene, segment_clear
from .scene import WorldScene

__all__ = ["MarkerCamConfig", "MarkerDetection", "MarkerCam", "detect_markers"]


@dataclass(frozen=True)
class MarkerCamConfig(SensorConfig):
    name: str = "markers"
    rate_hz: float | None = MARKER_RATE_HZ
    """Default 2 Hz, the measured AprilTag rate on the real hardware (R-SENS-10)."""

    max_range_m: float = MARKER_DETECT_RANGE_M   # A-4
    fov_rad: float = MARKER_DETECT_FOV_RAD       # A-5
    bearing_offset_rad: float = 0.0
    """Where the camera points in the body frame. Zero is straight ahead."""

    kinds: tuple[str, ...] = TARGET_KINDS
    """Landmark kinds this camera reports. Everything else in the arena is invisible to it."""

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.max_range_m <= 0:
            raise ConfigError(f"max_range_m must be > 0, got {self.max_range_m}")
        if not 0 < self.fov_rad <= 2 * np.pi:
            raise ConfigError(f"fov_rad must be in (0, 2pi], got {self.fov_rad}")
        if not self.kinds or not all(isinstance(k, str) and k for k in self.kinds):
            raise ConfigError(
                f"kinds must be a non-empty tuple of landmark kinds, got {self.kinds!r}"
            )

    @property
    def landmark_kinds(self) -> tuple[str, ...]:
        return tuple(self.kinds)

    def build(self, rng: np.random.Generator) -> "MarkerCam":
        return MarkerCam(self, rng)


@dataclass(frozen=True)
class MarkerDetection:
    """One marker seen this tick, in the frame a real detector would report it."""

    marker_id: str
    kind: str
    range_m: float
    bearing_rad: float
    """Body-frame bearing, CCW from the nose."""


def detect_markers(
    cfg: MarkerCamConfig,
    origin_xy: np.ndarray,
    theta: float,
    z: float,
    landmarks: Sequence[Landmark],
    occlusion_scene: RayScene,
) -> tuple[MarkerDetection, ...]:
    """Every landmark in ``landmarks`` visible from ``origin_xy``. Pure geometry.

    ``occlusion_scene`` should contain everything that can hide a marker: walls, pillars, and
    the other solid landmarks. A marker occluding itself is handled by testing line of sight
    to its near surface rather than its centre. Kind filtering is the caller's job -- this
    function reports whatever it is given.
    """
    if not landmarks:
        return ()
    origin = np.asarray(origin_xy, dtype=float).reshape(2)

    centres = np.array([[lm.x, lm.y] for lm in landmarks])
    radii = np.array([lm.radius_m for lm in landmarks])
    delta = centres - origin
    distances = np.linalg.norm(delta, axis=1)

    # Range is measured to the near surface, which is what a detector locking onto the tag
    # face would report, and keeps a marker the drone is touching from reading as "distance
    # 0.15 m to the centre of a solid object". A point landmark has no surface, so its range
    # is to the point.
    surface = np.maximum(distances - radii, 0.0)
    bearings = wrap_pi(np.arctan2(delta[:, 1], delta[:, 0]) - theta - cfg.bearing_offset_rad)

    candidate = (surface <= cfg.max_range_m) & (np.abs(bearings) <= cfg.fov_rad / 2.0)
    # No altitude cut-off. An earlier version copied the ToF ring's height gate, which made a
    # drone above 1.0 m completely blind to markers even though the ceiling is 1.4 m. That
    # reasoning is inverted for a camera: the real OV2640 is pitched 45 degrees NOSE-DOWN, so
    # it preferentially sees what is *below* it. Occlusion is still evaluated at the drone's
    # altitude, which is the part that genuinely depends on z.
    if not candidate.any():
        return ()

    idx = np.flatnonzero(candidate)
    # Stop just short of the surface so the target itself is not counted as its occluder.
    margin = 1e-3
    reach = np.maximum(surface[idx] - margin, 0.0)
    unit = delta[idx] / np.maximum(distances[idx], 1e-12)[:, None]
    endpoints = origin + unit * reach[:, None]
    clear = segment_clear(occlusion_scene, np.tile(origin, (len(idx), 1)), endpoints, z)

    return tuple(
        MarkerDetection(
            marker_id=landmarks[i].id,
            kind=landmarks[i].kind,
            range_m=float(surface[i]),
            bearing_rad=float(bearings[i]),
        )
        for i, ok in zip(idx, clear)
        if ok
    )


class MarkerCam(Sensor):
    """The detector as a sensor: filters the world's landmarks by kind, then asks the geometry.

    Its reading is a plain tuple of :class:`MarkerDetection`, empty when nothing is in view.
    A tuple has no fixed length, so this sensor is not recorded to the log (``record`` keeps
    the base class's ``None``); detections that mattered show up as ``landed`` events instead.
    """

    config: MarkerCamConfig

    def sample(self, truth: TrueState, world: WorldScene, tick: int) -> tuple[MarkerDetection, ...]:
        return detect_markers(
            self.config,
            truth.xy,
            truth.theta,
            truth.z,
            world.landmarks_of(*self.config.kinds),
            world.static_sensing_scene,
        )
