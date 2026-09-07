import numpy as np


def test_policy_module_exposes_wasp_controller():
    from safmc_sim.policies.wasp_v5 import WaspObservation, WaspV5Controller

    controller = WaspV5Controller(LEVY_EXPONENT=2.5, rng=np.random.default_rng(7))
    action, debug = controller.step(
        WaspObservation(
            bearings=np.array([0.0, np.pi / 2, np.pi, -np.pi / 2]),
            ranges=np.full(4, 3.0),
            range_max=3.0,
            delta_dist=0.0,
            step_time=0.05,
            current_yaw=0.0,
        )
    )
    assert np.isfinite(action.velocity_x)
    assert debug.new_step is True


def test_wasp_mode_aliases_register_distinct_controller_modes():
    from safmc_sim.api import get_policy
    from safmc_sim.policies.wasp_v5 import V5ControllerMode

    expected = {
        "wasp_v5_uniform_levy": V5ControllerMode.UNIFORM_LEVY,
        "wasp_v5_circular_resultant": V5ControllerMode.CIRCULAR_RESULTANT,
        "wasp_v5_pca_heading": V5ControllerMode.PCA_HEADING,
    }
    for name, mode in expected.items():
        policy = get_policy(name)("drone_00", {}, np.random.default_rng(0), None)
        assert policy._controller.controller_mode is mode
