"""R-DRONE-1..8: the 2.5D quadrotor model."""

import numpy as np
import pytest

from safmc_sim.constants import CEILING_M
from safmc_sim.errors import ConfigError
from safmc_sim.kinematics import IVX, IVY, IZ, ITHETA, Quad25D, QuadParams


def make(**kw):
    handler = Quad25D("quad25d", False, None)
    handler.params = QuadParams(**kw)
    return handler


def run(handler, state, action, n, dt=0.05):
    for _ in range(n):
        state = handler.step(state, np.asarray(action, float).reshape(-1, 1), dt)
    return state


def test_state_is_six_dimensional_with_velocity_carried():
    # R-DRONE-1.
    assert Quad25D.state_dim == 6
    assert Quad25D.action_dim == 4


def test_first_order_lag_reaches_63_percent_at_one_tau():
    # R-DRONE-4. The defining property of a first-order pole.
    tau, target = 0.35, 0.45
    handler = make(tau_s=tau, speed_max_ms=1.0)
    state = run(handler, np.zeros((6, 1)), [target, 0, 0, 0], int(round(tau / 0.05)))
    assert state[IVX, 0] == pytest.approx(0.632 * target, rel=0.06)


def test_lag_settles_at_the_command():
    handler = make(tau_s=0.35, speed_max_ms=1.0)
    state = run(handler, np.zeros((6, 1)), [0.45, 0, 0, 0], 200)
    assert state[IVX, 0] == pytest.approx(0.45, abs=1e-6)


def test_a_tick_longer_than_tau_does_not_overshoot():
    # Gain is clamped at 1.0; without that clamp dt/tau > 1 oscillates.
    handler = make(tau_s=0.05, speed_max_ms=10.0)
    state = handler.step(np.zeros((6, 1)), np.array([[5.0], [0], [0], [0]]), 0.5)
    assert state[IVX, 0] == pytest.approx(5.0)


def test_speed_cap_applies_to_the_vector_not_per_axis():
    # A per-axis clip would let a diagonal command reach speed_max * sqrt(2).
    handler = make(tau_s=0.001, speed_max_ms=1.0)
    state = run(handler, np.zeros((6, 1)), [10.0, 10.0, 0, 0], 50)
    speed = float(np.hypot(state[IVX, 0], state[IVY, 0]))
    assert speed == pytest.approx(1.0, abs=1e-6)
    # Direction is preserved by the scaling.
    assert state[IVX, 0] == pytest.approx(state[IVY, 0], abs=1e-9)


def test_yaw_is_integrated_and_wrapped_inside_the_handler():
    # R-DRONE-3. The reference implementation of arXiv:2607.25195 integrates yaw outside
    # env.step() and never wraps it, which is what this forbids.
    handler = make(yaw_rate_max=10.0)
    state = run(handler, np.zeros((6, 1)), [0, 0, 0, 2.0], 100)  # 10 rad of yaw
    assert -np.pi < state[ITHETA, 0] <= np.pi


def test_altitude_is_rate_limited_and_clamped_to_the_ceiling():
    # R-DRONE-5. The rules cap flight at 1.4 m.
    handler = make(climb_rate_max_ms=0.5)
    one_tick = handler.step(np.zeros((6, 1)), np.array([[0], [0], [99.0], [0]]), 0.05)
    assert one_tick[IZ, 0] == pytest.approx(0.5 * 0.05)
    settled = run(handler, np.zeros((6, 1)), [0, 0, 99.0, 0], 500)
    assert settled[IZ, 0] == pytest.approx(CEILING_M)
    floor = run(handler, settled, [0, 0, -99.0, 0], 500)
    assert floor[IZ, 0] == pytest.approx(0.0)


def test_yaw_rate_is_capped():
    handler = make(yaw_rate_max=0.5)
    state = handler.step(np.zeros((6, 1)), np.array([[0], [0], [0], [99.0]]), 0.1)
    assert state[ITHETA, 0] == pytest.approx(0.05)


def test_velocity_to_xy_reports_arena_frame_state_velocity():
    handler = make()
    state = np.zeros((6, 1))
    state[IVX, 0], state[IVY, 0] = 0.3, -0.2
    assert handler.velocity_to_xy(state, None).ravel() == pytest.approx([0.3, -0.2])


@pytest.mark.parametrize(
    "kw",
    [{"tau_s": 0.0}, {"tau_s": -1.0}, {"speed_max_ms": 0.0}, {"ceiling_m": -1.0},
     {"climb_rate_max_ms": np.inf}],
)
def test_impossible_parameters_raise(kw):
    with pytest.raises(ConfigError):
        QuadParams(**kw)


def test_configure_robots_rejects_a_non_quad_robot():
    from safmc_sim.kinematics import configure_robots

    class Fake:
        name = "robot_0"
        kf = object()

    with pytest.raises(ConfigError, match="not Quad25D"):
        configure_robots([Fake()], QuadParams())
    with pytest.raises(ConfigError):
        configure_robots([], QuadParams())
