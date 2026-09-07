import numpy as np

from safmc_sim.api import Land, Velocity
from safmc_sim.sensors.marker_cam import MarkerDetection
from safmc_sim.testing import make_observation


def test_mission_wrapper_prioritises_fire_and_lands_inside_threshold():
    from safmc_sim.policies.mission_wrapper import MissionWrapper

    wrapper = MissionWrapper()
    markers = (
        MarkerDetection("victim", "victim", 0.4, 0.0),
        MarkerDetection("fire", "fire", 0.6, 0.0),
    )
    assert isinstance(wrapper.command(make_observation(markers=markers)), Land)


def test_mission_wrapper_approaches_visible_target_before_landing():
    from safmc_sim.policies.mission_wrapper import MissionWrapper

    command = MissionWrapper().command(make_observation(markers=(
        MarkerDetection("fire", "fire", 1.0, np.pi / 2),
    )))
    assert isinstance(command, Velocity)
    assert command.vx > 0.0
    assert command.yaw_rate > 0.0
