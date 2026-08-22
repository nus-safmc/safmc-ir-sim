"""R-FRAME-1..5: frames, wrapping, and the NED bijection."""

import numpy as np
import pytest

from safmc_sim.frames import (
    arena_to_ned,
    arena_yaw_to_ned_heading,
    ned_to_arena,
    wrap_2pi,
    wrap_pi,
)


def test_wrap_pi_uses_half_open_interval():
    # R-FRAME-2: (-pi, pi]. -pi must map to +pi so that wrapping is idempotent.
    assert wrap_pi(-np.pi) == pytest.approx(np.pi)
    assert wrap_pi(np.pi) == pytest.approx(np.pi)
    assert wrap_pi(wrap_pi(-np.pi)) == pytest.approx(wrap_pi(-np.pi))


def test_wrap_pi_in_range_and_congruent():
    """One vectorised assertion over the same 401 angles.

    This was a `parametrize` producing 401 separate test cases, which inflated the suite's
    headline count by 68% and told you nothing a single array assertion does not. The function
    under test is vectorised; the test should be too.
    """
    angles = np.linspace(-20.0, 20.0, 401)
    wrapped = wrap_pi(angles)
    assert np.all(wrapped > -np.pi) and np.all(wrapped <= np.pi + 1e-12)
    residual = (wrapped - angles) % (2 * np.pi)
    assert np.all(np.isclose(residual, 0.0, atol=1e-9) | np.isclose(residual, 2 * np.pi, atol=1e-9))


def test_cardinal_headings_match_the_firmware_convention():
    # NED heading is 0 at North and clockwise positive [esp-everything wifi_task.h:36-38].
    assert arena_yaw_to_ned_heading(np.pi / 2) == pytest.approx(0.0)          # +y is North
    assert arena_yaw_to_ned_heading(0.0) == pytest.approx(np.pi / 2)          # +x is East
    assert arena_yaw_to_ned_heading(-np.pi / 2) == pytest.approx(np.pi)       # -y is South
    assert arena_yaw_to_ned_heading(np.pi) == pytest.approx(3 * np.pi / 2)    # -x is West


def test_ned_axes_are_swapped_and_z_is_inverted():
    # R-FRAME-4.
    ned_x, ned_y, ned_z, _ = arena_to_ned(3.0, 7.0, 0.5, 0.0)
    assert (ned_x, ned_y, ned_z) == (7.0, 3.0, -0.5)


def test_ned_roundtrip_is_exact_across_the_wrap_discontinuity():
    # R-FRAME-5: 1e-9 position, 1e-9 rad heading, including at +/-pi.
    rng = np.random.default_rng(0)
    thetas = np.concatenate(
        [rng.uniform(-np.pi, np.pi, 500), [np.pi, -np.pi, 0.0, np.pi / 2, -np.pi / 2]]
    )
    for theta in thetas:
        x, y, z = rng.uniform(-50, 50, 3)
        back = ned_to_arena(*arena_to_ned(x, y, z, theta))
        assert back[0] == pytest.approx(x, abs=1e-9)
        assert back[1] == pytest.approx(y, abs=1e-9)
        assert back[2] == pytest.approx(z, abs=1e-9)
        assert wrap_pi(back[3] - theta) == pytest.approx(0.0, abs=1e-9)


def test_wrap_2pi_range():
    for a in np.linspace(-20, 20, 200):
        assert 0.0 <= wrap_2pi(a) < 2 * np.pi
