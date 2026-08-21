"""R-SENS-1..5: the ToF ring reproduces the flown hardware's geometry and data product."""

import numpy as np
import pytest

from safmc_sim.constants import (
    TOF_COLLAPSED_BINS,
    TOF_MAX_VALID_M,
    TOF_MIN_VALID_M,
    TOF_MOUNT_RADIUS_M,
    TOF_STATUS_NO_RETURN,
    TOF_STATUS_VALID,
)
from safmc_sim.errors import ConfigError
from safmc_sim.sensors.raycast import RayScene
from safmc_sim.sensors.scene import WorldScene
from safmc_sim.sensors.tof_ring import ToFConfig, ToFRing, _ranger_bearings, _zone_offsets


class FakeParent:
    """Minimal stand-in for an ir-sim ObjectBase."""

    def __init__(self, x, y, theta, z):
        self.state = np.array([[x], [y], [theta], [z], [0.0], [0.0]])
        self.id = 0
        self._env = None


def make_ring(parent, scene, **cfg):
    ring = ToFRing(config=ToFConfig(**cfg))
    ring.parent = parent
    ring.attach(scene, np.random.default_rng(0))
    return ring


def box_scene(size=20.0, height=2.0):
    segments = np.array(
        [[0, 0, size, 0], [size, 0, size, size], [size, size, 0, size], [0, size, 0, 0]],
        dtype=float,
    )
    return WorldScene(RayScene(segments=segments, segment_heights=np.full(4, height)))


def test_ring_geometry_matches_the_firmware():
    """R-SENS-2: eight rangers, 45 deg apart, eight 5.625 deg zones each."""
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


def test_collapsed_scan_covers_all_64_bins_exactly_once():
    """R-SENS-5: 64 bins of 5.625 deg, index 0 straight ahead, clockwise."""
    ring = make_ring(FakeParent(10, 10, 0.0, 0.5), box_scene())
    bins = ring._collapsed_bins.reshape(-1)
    assert len(bins) == TOF_COLLAPSED_BINS
    assert sorted(bins) == list(range(TOF_COLLAPSED_BINS))
    # The two zones straddling the nose land in bins 0 and 63.
    assert {0, 63} <= set(ring._collapsed_bins[0])


def test_ranges_are_correct_against_hand_computed_geometry():
    # Drone at the centre of a 20 m box, facing +x. The forward ranger's origin is one mount
    # radius ahead, so its nearest zone reads slightly less than 10 m.
    ring = make_ring(
        FakeParent(10, 10, 0.0, 0.5),
        box_scene(20.0),
        sensor_max_range_m=25.0,
        max_valid_m=20.0,
    )
    scan = ring.step(np.array([[10.0], [10.0], [0.0]]))
    fwd = scan.ranges_m[0]
    expected_axis = 10.0 - TOF_MOUNT_RADIUS_M
    # The innermost zones are 2.8125 deg off-axis, so range = axis / cos(offset).
    off = np.deg2rad(2.8125)
    assert fwd[3] == pytest.approx(expected_axis / np.cos(off), abs=1e-6)
    assert fwd[4] == pytest.approx(expected_axis / np.cos(off), abs=1e-6)


def test_gate_rejects_returns_outside_the_firmware_window():
    """R-SENS-3, R-SENS-4: outside [50, 3000] mm is reported as no-return, not a number."""
    scene = box_scene(20.0)
    ring = make_ring(FakeParent(10, 10, 0.0, 0.5), scene)  # walls ~10 m away, gate is 3 m
    scan = ring.step(np.array([[10.0], [10.0], [0.0]]))
    assert np.all(np.isinf(scan.ranges_m))
    assert np.all(scan.status == TOF_STATUS_NO_RETURN)
    assert np.all(np.isinf(scan.collapsed_m))

    # Move within the gate and the forward ranger reports.
    near = make_ring(FakeParent(18.0, 10, 0.0, 0.5), scene)
    scan = near.step(np.array([[18.0], [10.0], [0.0]]))
    assert np.isfinite(scan.ranges_m[0]).all()
    assert np.all(scan.status[0] == TOF_STATUS_VALID)
    # The nearest zone is 2.8125 deg off the ranger axis, so it reads axis/cos(offset),
    # not the axis distance itself.
    axis = 2.0 - TOF_MOUNT_RADIUS_M
    assert scan.min_range_m == pytest.approx(axis / np.cos(np.deg2rad(2.8125)), abs=1e-6)


def test_gate_bounds_are_the_firmware_values():
    cfg = ToFConfig()
    assert cfg.min_valid_m == TOF_MIN_VALID_M == 0.050
    assert cfg.max_valid_m == TOF_MAX_VALID_M == 3.000
    assert cfg.sensor_max_range_m == 4.0


def test_height_gating_reaches_the_scan():
    """R-SENS-6 end to end: a 1.0 m marker is seen at cruise and not above it."""
    marker = RayScene(circles=np.array([[12.0, 10.0, 0.15]]), circle_heights=np.array([1.0]))
    walls = np.array([[0, 0, 20, 0], [20, 0, 20, 20], [20, 20, 0, 20], [0, 20, 0, 0]], float)
    scene = WorldScene(
        RayScene(segments=walls, segment_heights=np.full(4, 2.0)), markers=marker
    )
    low = make_ring(FakeParent(10, 10, 0.0, 0.5), scene)
    assert np.isfinite(low.step(np.array([[10.0], [10.0], [0.0]])).ranges_m).any()

    high = make_ring(FakeParent(10, 10, 0.0, 1.2), scene)
    assert np.all(np.isinf(high.step(np.array([[10.0], [10.0], [0.0]])).ranges_m))


def test_firmware_frame_carries_only_distance_and_status():
    ring = make_ring(FakeParent(18.0, 10, 0.0, 0.5), box_scene(20.0))
    frame = ring.step(np.array([[18.0], [10.0], [0.0]])).as_firmware_frame()
    assert set(frame) == {"distance_mm", "target_status"}
    assert frame["distance_mm"].dtype == np.uint16
    assert frame["target_status"].dtype == np.uint8
    # A no-return is millimetre zero, which is what the driver reports for an invalid zone.
    assert frame["distance_mm"][np.isinf(ring.get_scan().ranges_m)].tolist() == [0] * int(
        np.isinf(ring.get_scan().ranges_m).sum()
    )


def test_noise_is_drawn_from_the_supplied_generator_and_is_reproducible():
    """R-DET-2: no global RNG anywhere."""
    scene = box_scene(20.0)

    def sample(seed):
        ring = ToFRing(config=ToFConfig(noise_std_m=0.02))
        ring.parent = FakeParent(18.0, 10, 0.0, 0.5)
        ring.attach(scene, np.random.default_rng(seed))
        return ring.step(np.array([[18.0], [10.0], [0.0]])).ranges_m.copy()

    assert np.array_equal(sample(3), sample(3))
    assert not np.array_equal(sample(3), sample(4))


def test_noise_never_invents_a_return_where_there_was_none():
    """R-SENS-9: perturbing 'nothing is there' is meaningless."""
    ring = ToFRing(config=ToFConfig(noise_std_m=0.5))
    ring.parent = FakeParent(10, 10, 0.0, 0.5)
    ring.attach(box_scene(20.0), np.random.default_rng(0))
    scan = ring.step(np.array([[10.0], [10.0], [0.0]]))
    assert np.all(np.isinf(scan.ranges_m))


def test_missing_attach_fails_loudly():
    ring = ToFRing()
    ring.parent = FakeParent(0, 0, 0, 0.5)
    with pytest.raises(ConfigError, match="attach"):
        ring.step(np.array([[0.0], [0.0], [0.0]]))


def test_parent_without_altitude_fails_loudly():
    class Flat:
        state = np.zeros((3, 1))
        id = 0
        _env = None

    ring = ToFRing()
    ring.parent = Flat()
    ring.attach(box_scene(), np.random.default_rng(0))
    with pytest.raises(ConfigError, match="altitude"):
        ring.step(np.zeros((3, 1)))


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
