"""R-SENS-6, R-SENS-7, R-SENS-8: the raycaster's correctness and its 2.5D gating.

R-SENS-7 requires agreement with an independent analytic ground truth to 1e-9 m. The
references below are deliberately derived differently from the implementation -- a
law-of-cosines form for circles and a 2x2 linear solve for segments -- so that a shared
algebraic slip would not cancel out. Shapely is deliberately *not* used as the reference: its
circles are polygonised, which costs about 1.5 mm.
"""

import numpy as np
import pytest

from safmc_sim.errors import ConfigError
from safmc_sim.sensors.raycast import RayScene, cast_rays, segment_clear


def analytic_ray_circle(origin, direction, centre, radius):
    """Nearest non-negative hit, via the law of cosines. Independent of the implementation."""
    d = np.asarray(direction, float) / np.linalg.norm(direction)
    to_centre = np.asarray(centre, float) - np.asarray(origin, float)
    dist = np.linalg.norm(to_centre)
    if dist == 0.0:
        return radius
    along = float(np.dot(to_centre, d))
    perp_sq = dist**2 - along**2
    if perp_sq > radius**2:
        return np.inf
    half_chord = np.sqrt(max(radius**2 - perp_sq, 0.0))
    near, far = along - half_chord, along + half_chord
    if near >= 0:
        return near
    return far if far >= 0 else np.inf


def analytic_ray_segment(origin, direction, p1, p2):
    """Nearest hit, via a 2x2 linear solve. Independent of the implementation."""
    d = np.asarray(direction, float) / np.linalg.norm(direction)
    p1, p2 = np.asarray(p1, float), np.asarray(p2, float)
    edge = p2 - p1
    matrix = np.array([[d[0], -edge[0]], [d[1], -edge[1]]])
    if abs(np.linalg.det(matrix)) < 1e-14:
        return np.inf
    t, u = np.linalg.solve(matrix, p1 - np.asarray(origin, float))
    return t if (t >= 0 and 0.0 <= u <= 1.0) else np.inf


def test_circle_matches_analytic_to_1e9():
    rng = np.random.default_rng(0)
    circles = rng.uniform([1, 1, 0.2], [9, 9, 1.5], size=(12, 3))
    scene = RayScene(circles=circles, circle_heights=np.full(12, 3.0))
    origins = rng.uniform(0, 10, size=(400, 2))
    angles = rng.uniform(-np.pi, np.pi, 400)
    directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)

    got = cast_rays(scene, origins, directions, 0.5, 50.0)
    for i in range(len(origins)):
        expect = min(
            analytic_ray_circle(origins[i], directions[i], c[:2], c[2]) for c in circles
        )
        expect = expect if expect <= 50.0 else np.inf
        if np.isinf(expect):
            assert np.isinf(got[i])
        else:
            assert got[i] == pytest.approx(expect, abs=1e-9)


def test_segment_matches_analytic_to_1e9():
    rng = np.random.default_rng(1)
    segments = rng.uniform(0, 10, size=(15, 4))
    scene = RayScene(segments=segments, segment_heights=np.full(15, 3.0))
    origins = rng.uniform(0, 10, size=(400, 2))
    angles = rng.uniform(-np.pi, np.pi, 400)
    directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)

    got = cast_rays(scene, origins, directions, 0.5, 50.0)
    for i in range(len(origins)):
        expect = min(
            analytic_ray_segment(origins[i], directions[i], s[:2], s[2:]) for s in segments
        )
        expect = expect if expect <= 50.0 else np.inf
        if np.isinf(expect):
            assert np.isinf(got[i])
        else:
            assert got[i] == pytest.approx(expect, abs=1e-9)


def test_origin_inside_obstacle_returns_the_exit_distance_not_stale_data():
    """R-SENS-8. This is the exact case where ir-sim's Lidar2D silently freezes its scan."""
    scene = RayScene(circles=np.array([[10.0, 10.0, 2.0]]), circle_heights=np.array([2.0]))
    angles = np.linspace(-np.pi, np.pi, 24, endpoint=False)
    directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)

    # Walk the sensor from outside the circle, through it, and out the far side. Every
    # position must produce a fresh, correct answer -- never the previous position's.
    seen = []
    for x in np.linspace(7.0, 13.0, 25):
        origins = np.tile([x, 10.0], (24, 1))
        got = cast_rays(scene, origins, directions, 0.5, 20.0)
        seen.append(got.copy())
        if abs(x - 10.0) < 1e-9:  # dead centre: every ray exits at exactly the radius
            assert np.allclose(got, 2.0, atol=1e-9)

    # No two consecutive positions may produce byte-identical scans, which is the signature
    # of the stale-data defect.
    for a, b in zip(seen, seen[1:]):
        assert not np.array_equal(a, b)


def test_height_gating_is_the_whole_of_the_2p5d_model():
    """R-SENS-6. A 1.0 m marker blocks at 0.5 m cruise and not above it."""
    marker = RayScene(circles=np.array([[10.0, 10.0, 0.15]]), circle_heights=np.array([1.0]))
    origins, directions = np.array([[7.0, 10.0]]), np.array([[1.0, 0.0]])
    assert np.isfinite(cast_rays(marker, origins, directions, 0.5, 10.0)[0])
    assert np.isinf(cast_rays(marker, origins, directions, 1.2, 10.0)[0])
    # Exactly at the top is "above": the interval is [z_min, height).
    assert np.isinf(cast_rays(marker, origins, directions, 1.0, 10.0)[0])


def test_airborne_body_has_a_vertical_band_not_a_column():
    """A drone at 0.8 m must not occlude a ray cast at 0.5 m."""
    drone = RayScene(
        circles=np.array([[10.0, 10.0, 0.18]]),
        circle_heights=np.array([0.85]),
        circle_z_min=np.array([0.75]),
    )
    origins, directions = np.array([[7.0, 10.0]]), np.array([[1.0, 0.0]])
    assert np.isinf(cast_rays(drone, origins, directions, 0.5, 10.0)[0])
    assert np.isfinite(cast_rays(drone, origins, directions, 0.8, 10.0)[0])


def test_no_return_is_inf_not_max_range():
    """R-SENS-4: 'nothing there' and 'a surface at exactly max_range' are different facts."""
    scene = RayScene(circles=np.array([[50.0, 50.0, 1.0]]), circle_heights=np.array([2.0]))
    got = cast_rays(scene, np.array([[0.0, 0.0]]), np.array([[1.0, 0.0]]), 0.5, 4.0)
    assert np.isinf(got[0])


def test_line_of_sight_endpoint_semantics():
    """A target standing against a wall is still visible; a target behind it is not."""
    wall = RayScene(segments=np.array([[12.0, 8.0, 12.0, 12.0]]), segment_heights=np.array([2.0]))
    a = np.array([[7.0, 10.0]] * 3)
    b = np.array([[15.0, 10.0], [11.0, 10.0], [12.0, 10.0]])
    assert list(segment_clear(wall, a, b, 0.5)) == [False, True, True]


def test_coincident_endpoints_are_trivially_visible():
    wall = RayScene(segments=np.array([[12.0, 8.0, 12.0, 12.0]]), segment_heights=np.array([2.0]))
    assert segment_clear(wall, np.array([[7.0, 10.0]]), np.array([[7.0, 10.0]]), 0.5)[0]


@pytest.mark.parametrize(
    "kwargs",
    [
        {"circles": np.zeros((2, 3)), "circle_heights": np.zeros(1)},
        {"segments": np.zeros((2, 4)), "segment_heights": np.zeros(1)},
        {"circles": np.zeros((1, 2)), "circle_heights": np.zeros(1)},
    ],
)
def test_malformed_scenes_raise(kwargs):
    with pytest.raises(ConfigError):
        RayScene(**kwargs)


def test_zero_direction_and_bad_range_raise():
    scene = RayScene(circles=np.array([[1.0, 1.0, 1.0]]), circle_heights=np.array([1.0]))
    with pytest.raises(ConfigError):
        cast_rays(scene, np.array([[0.0, 0.0]]), np.array([[0.0, 0.0]]), 0.5, 5.0)
    with pytest.raises(ConfigError):
        cast_rays(scene, np.array([[0.0, 0.0]]), np.array([[1.0, 0.0]]), 0.5, 0.0)
    with pytest.raises(ConfigError):
        cast_rays(scene, np.array([[0.0, 0.0]]), np.array([[1.0, 0.0]]), 0.5, np.inf)
