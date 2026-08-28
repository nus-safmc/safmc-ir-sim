"""The ToF ring: geometry, gating, height, noise, and what a policy may do to it."""

import numpy as np
import pytest

from safmc_sim.constants import (
    TOF_MAX_VALID_M,
    TOF_MIN_VALID_M,
    TOF_MOUNT_RADIUS_M,
)
from safmc_sim.errors import ConfigError
from safmc_sim.sensors.raycast import RayScene
from safmc_sim.sensors.scene import WorldScene
from safmc_sim.sensors.tof_ring import ToFConfig, ToFRing, _ranger_bearings, _zone_offsets


def box_scene(size=20.0, height=2.0, markers=None):
    segments = np.array(
        [[0, 0, size, 0], [size, 0, size, size], [size, size, 0, size], [0, size, 0, 0]],
        dtype=float,
    )
    return WorldScene(
        RayScene(segments=segments, segment_heights=np.full(4, height)), markers=markers
    )


def make_ring(scene, seed=0, **cfg):
    return ToFRing(ToFConfig(**cfg), scene, np.random.default_rng(seed))


def test_ring_geometry_matches_the_flown_hardware():
    """8 rangers 45 degrees apart, 8 zones of 5.625 degrees each, 40 mm mount radius."""
    cfg = ToFConfig()
    assert cfg.n_rangers == 8 and cfg.zones_per_ranger == 8
    bearings = np.rad2deg(_ranger_bearings(cfg))
    # Mounted counter-clockwise: ranger i sits at i * 45 degrees CCW from the nose.
    assert np.allclose(np.sort(np.mod(bearings, 360)), np.arange(0, 360, 45))
    offsets = np.rad2deg(_zone_offsets(cfg))
    assert np.allclose(np.diff(offsets), 45.0 / 8.0)
    # Symmetric about the ranger axis, so no zone points exactly along it.
    assert offsets[0] == pytest.approx(-offsets[-1])
    assert cfg.mount_radius_m == TOF_MOUNT_RADIUS_M


def test_front_index_rotates_the_whole_ring():
    a = np.rad2deg(_ranger_bearings(ToFConfig(front_index=0)))
    b = np.rad2deg(_ranger_bearings(ToFConfig(front_index=1)))
    assert np.allclose(np.sort(np.mod(a, 360)), np.sort(np.mod(b, 360)))
    assert not np.allclose(a, b)


def test_config_is_applied_not_ignored():
    """The config reaches the derived geometry.

    It used to be assigned onto the sensor *after* construction, so the bearings kept their
    defaults while `sensor.config` reported the requested values -- meaning the recorded log
    described a ring that was never simulated.
    """
    ring = make_ring(box_scene(), front_index=4)
    assert np.allclose(ring._ranger_bearings, _ranger_bearings(ToFConfig(front_index=4)))
    assert not np.allclose(ring._ranger_bearings, _ranger_bearings(ToFConfig(front_index=0)))


def test_ranges_are_correct_against_hand_computed_geometry():
    # Drone at the centre of a 20 m box, facing +x. The forward ranger's origin is one mount
    # radius ahead, so its nearest zone reads slightly less than 10 m.
    ring = make_ring(box_scene(20.0), sensor_max_range_m=25.0, max_valid_m=20.0)
    fwd = ring.step(10.0, 10.0, 0.0, 0.5).ranges_m[0]
    axis = 10.0 - TOF_MOUNT_RADIUS_M
    # The innermost zones are 2.8125 deg off-axis, so range = axis / cos(offset).
    off = np.deg2rad(2.8125)
    assert fwd[3] == pytest.approx(axis / np.cos(off), abs=1e-6)
    assert fwd[4] == pytest.approx(axis / np.cos(off), abs=1e-6)


def test_gate_rejects_returns_outside_the_acceptance_window():
    """Outside [50, 3000] mm is reported as no-return, never as a fabricated number."""
    scene = box_scene(20.0)
    far = make_ring(scene).step(10.0, 10.0, 0.0, 0.5)      # walls ~10 m away, gate is 3 m
    assert np.all(np.isinf(far.ranges_m))
    assert np.isinf(far.min_range_m)

    near = make_ring(scene).step(18.0, 10.0, 0.0, 0.5)
    assert np.isfinite(near.ranges_m[0]).all()
    axis = 2.0 - TOF_MOUNT_RADIUS_M
    assert near.min_range_m == pytest.approx(axis / np.cos(np.deg2rad(2.8125)), abs=1e-6)


def test_gate_bounds_are_the_firmware_values():
    cfg = ToFConfig()
    assert cfg.min_valid_m == TOF_MIN_VALID_M == 0.050
    assert cfg.max_valid_m == TOF_MAX_VALID_M == 3.000
    assert cfg.sensor_max_range_m == 4.0


def test_height_gating_reaches_the_scan():
    """A 1.0 m marker is seen at cruise altitude and not above it."""
    markers = RayScene(circles=np.array([[12.0, 10.0, 0.15]]), circle_heights=np.array([1.0]))
    scene = box_scene(20.0, markers=markers)
    assert np.isfinite(make_ring(scene).step(10.0, 10.0, 0.0, 0.5).ranges_m).any()
    assert np.all(np.isinf(make_ring(scene).step(10.0, 10.0, 0.0, 1.2).ranges_m))


def test_noise_is_drawn_from_the_supplied_generator_and_is_reproducible():
    scene = box_scene(20.0)

    def sample(seed):
        return make_ring(scene, seed=seed, noise_std_m=0.02).step(18.0, 10.0, 0.0, 0.5).ranges_m

    assert np.array_equal(sample(3), sample(3))
    assert not np.array_equal(sample(3), sample(4))


def test_noise_never_invents_a_return_where_there_was_none():
    """Perturbing 'nothing is there' is meaningless."""
    scan = make_ring(box_scene(20.0), noise_std_m=0.5).step(10.0, 10.0, 0.0, 0.5)
    assert np.all(np.isinf(scan.ranges_m))


def test_bearing_arrays_handed_to_a_policy_are_read_only():
    """A frozen dataclass blocks rebinding but not in-place numpy writes."""
    scan = make_ring(box_scene()).step(10.0, 10.0, 0.0, 0.5)
    for name in ("zone_bearings_rad", "ranger_bearings_rad"):
        array = getattr(scan, name)
        assert not array.flags.writeable, f"{name} is writable"
        with pytest.raises(ValueError):
            array[:] = 0.0


@pytest.mark.parametrize(
    "kw",
    [
        {"n_rangers": 0},
        {"front_index": 9},
        {"min_valid_m": 5.0, "max_valid_m": 3.0},
        {"max_valid_m": 9.0},
        {"noise_std_m": -1.0},
    ],
)
def test_impossible_config_raises(kw):
    with pytest.raises(ConfigError):
        ToFConfig(**kw)


# -- fidelity to the real hardware ------------------------------------------------------------
#
# These pin the ring to its two sources of truth: the firmware's own angle arithmetic, and the
# URDF's mount geometry. If someone changes the ring, one of these should be what tells them.

# esp-everything/main/tof_task.c:183 and :258, transcribed literally. Firmware angles are
# CLOCKWISE from body-forward; ours are counter-clockwise, hence the negation.
def _firmware_bearings_ccw(front_index=0, n=8, zones=8):
    sensor_cw = np.array([((front_index - i) * 45 % 360 + 360) % 360 for i in range(n)], float)
    zone_cw = np.array([[sensor_cw[s] + (3.5 - c) * (45.0 / 8.0) for c in range(zones)]
                        for s in range(n)])
    return -sensor_cw, -zone_cw


# safmc-ros/safmc_mapping/urdf/robot.urdf, in the index order of tof_pub.py's TOF_ID_MAP:
# tof_n, tof_nw, tof_w, tof_sw, tof_s, tof_se, tof_e, tof_ne. ROS body frame: x forward, y left.
_URDF_MOUNTS = [
    (0.040, 0.0), (0.0240416306, 0.0240416306), (0.0, 0.040), (-0.0240416306, 0.0240416306),
    (-0.040, 0.0), (-0.0240416306, -0.0240416306), (0.0, -0.040), (0.0240416306, -0.0240416306),
]


def _same_angle(a, b):
    return np.allclose(np.mod(np.asarray(a) - np.asarray(b) + 180, 360) - 180, 0, atol=1e-9)


@pytest.mark.parametrize("front_index", [0, 1, 4])
def test_every_bearing_matches_the_firmware_formula(front_index):
    cfg = ToFConfig(front_index=front_index)
    sensor_ccw, zone_ccw = _firmware_bearings_ccw(front_index)
    assert _same_angle(np.rad2deg(_ranger_bearings(cfg)), sensor_ccw)
    sim_zone = np.rad2deg(_ranger_bearings(cfg)[:, None] + _zone_offsets(cfg)[None, :])
    assert _same_angle(sim_zone, zone_ccw)


def test_mount_geometry_matches_the_urdf():
    """The ring is not a circle: cardinals at 40 mm, diagonals at 34 mm."""
    from safmc_sim.sensors.tof_ring import _mount_radii

    cfg = ToFConfig()
    bearings = _ranger_bearings(cfg)
    radii = _mount_radii(cfg)
    placed = np.stack((radii * np.cos(bearings), radii * np.sin(bearings)), axis=1)
    assert np.allclose(placed, np.array(_URDF_MOUNTS), atol=1e-6)
    # The diagonals really are closer in -- a uniform radius would put them 6 mm too far out.
    assert radii[1] < radii[0]


def test_rangers_point_where_their_names_say():
    """End-to-end handedness: a wall on the left must be seen by the left ranger.

    A formula comparison cannot catch a sign or handedness error that is consistent between
    the two formulas. Putting a real wall in a real scene can.
    """
    scene_for = lambda seg: WorldScene(
        RayScene(segments=np.array([seg], float), segment_heights=np.array([2.0]))
    )
    # ARENA frame: +x East, +y North, theta CCW. Drone at the origin facing +x.
    cases = {
        0: [2, -1, 2, 1],     # ahead  (+x)
        2: [-1, 2, 1, 2],     # left   (+y)
        4: [-2, -1, -2, 1],   # behind (-x)
        6: [-1, -2, 1, -2],   # right  (-y)
    }
    for expected_ranger, wall in cases.items():
        ring = ToFRing(ToFConfig(), scene_for(wall), np.random.default_rng(0))
        per_ranger = np.min(ring.step(0.0, 0.0, 0.0, 0.5).ranges_m, axis=1)
        assert int(np.argmin(np.where(np.isfinite(per_ranger), per_ranger, np.inf))) == expected_ranger

    # Yaw the drone 90 degrees anticlockwise and the wall that was ahead is now on its right.
    ring = ToFRing(ToFConfig(), scene_for(cases[0]), np.random.default_rng(0))
    per_ranger = np.min(ring.step(0.0, 0.0, np.pi / 2, 0.5).ranges_m, axis=1)
    assert int(np.argmin(np.where(np.isfinite(per_ranger), per_ranger, np.inf))) == 6
