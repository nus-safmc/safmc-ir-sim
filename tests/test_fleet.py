"""R-SENS-15 as amended by ADR-0007: the fleet -- every drone, every lifecycle -- as a sensor's view.

The bodies a ray can hit are active drones only (what can kill you is what you can see). The
fleet is a different question, asked by a device every airframe carries and every other
airframe answers: where is every tag, flying or parked? These tests pin the difference.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from safmc_sim import policies  # noqa: F401 -- registers sdlw
from safmc_sim.api import Land, Lifecycle, Policy, Velocity, register_policy
from safmc_sim.errors import ConfigError
from safmc_sim.runner import RunConfig, Runner
from safmc_sim.sensors.base import Sensor, SensorConfig, TrueState, read_only
from safmc_sim.sensors.raycast import RayScene
from safmc_sim.sensors.scene import Fleet, WorldScene
from safmc_sim.sensors.tof_ring import ToFConfig

SHORT = dict(n_drones=10, duration_s=2.0, record=False)


def box_scene(size=20.0, height=2.0):
    segments = np.array(
        [[0, 0, size, 0], [size, 0, size, size], [size, size, 0, size], [0, size, 0, 0]], float
    )
    return WorldScene(RayScene(segments=segments, segment_heights=np.full(4, float(height))))


def state(x, y, z=0.5):
    return np.array([[x], [y], [0.0], [z], [0.0], [0.0]])


# -- the query --------------------------------------------------------------------------------------


def test_a_scene_built_by_hand_has_an_empty_fleet():
    fleet = box_scene().fleet
    assert isinstance(fleet, Fleet) and len(fleet) == 0
    assert fleet.agent_ids == () and fleet.xyz.shape == (0, 3) and fleet.object_ids.shape == (0,)
    assert fleet.index_of(0) == -1


def test_the_fleet_names_every_drone_in_order_with_its_true_position():
    world = box_scene()
    world.refresh_fleet(
        [("drone_00", 3, state(1.0, 2.0, 0.5)), ("drone_01", 9, state(4.0, 5.0, 0.0)),
         ("drone_02", 5, state(7.0, 8.0, 1.2))],
        cache_key=0,
    )
    fleet = world.fleet
    assert fleet.agent_ids == ("drone_00", "drone_01", "drone_02"), "run order, not sorted by id"
    assert fleet.object_ids.tolist() == [3, 9, 5]
    assert np.allclose(fleet.xyz, [[1.0, 2.0, 0.5], [4.0, 5.0, 0.0], [7.0, 8.0, 1.2]])
    assert fleet.index_of(9) == 1 and fleet.index_of(42) == -1
    assert len(fleet) == 3


def test_the_fleet_is_read_only_and_shared():
    """Every sensor on every drone reads the same fleet this tick; one that wrote into it would
    move a teammate for every sensor sampled after it."""
    world = box_scene()
    world.refresh_fleet([("drone_00", 0, state(1.0, 1.0))], cache_key=0)
    fleet = world.fleet
    with pytest.raises(ValueError, match="read-only"):
        fleet.xyz[0, 0] = 99.0
    with pytest.raises(ValueError, match="read-only"):
        fleet.object_ids[0] = 99
    with pytest.raises(ValueError):
        fleet.xyz.flags.writeable = True
    assert world.fleet is fleet


def test_the_fleet_is_rebuilt_once_per_key_and_independently_of_the_bodies():
    world = box_scene()
    world.refresh_fleet([("drone_00", 0, state(1.0, 1.0))], cache_key=7)
    first = world.fleet
    world.refresh_fleet([("drone_00", 0, state(5.0, 5.0))], cache_key=7)   # same key: no-op
    assert world.fleet is first and world.fleet.xyz[0, 0] == 1.0
    world.refresh_fleet([("drone_00", 0, state(5.0, 5.0))], cache_key=8)
    assert world.fleet.xyz[0, 0] == 5.0
    # The bodies have their own key: refreshing the fleet did not touch them, and vice versa.
    assert len(world._drone_ids) == 0
    world.refresh_drones([], cache_key=8)
    assert world.fleet.xyz[0, 0] == 5.0


def test_a_fleet_with_a_repeated_or_empty_agent_id_is_refused():
    world = box_scene()
    with pytest.raises(ConfigError, match="repeats"):
        world.refresh_fleet([("d", 0, state(1, 1)), ("d", 1, state(2, 2))], cache_key=0)
    with pytest.raises(ConfigError, match="no agent id"):
        world.refresh_fleet([("", 0, state(1, 1))], cache_key=1)


def test_an_empty_fleet_after_a_full_one_is_empty():
    world = box_scene()
    world.refresh_fleet([("drone_00", 0, state(1.0, 1.0))], cache_key=0)
    world.refresh_fleet([], cache_key=1)
    assert len(world.fleet) == 0 and world.fleet.xyz.shape == (0, 3)


# -- the runner path: every lifecycle, same tick as the bodies --------------------------------------


@dataclass(frozen=True)
class Roster:
    """A reading that reports only how many drones the fleet holds and where drone_00 is."""

    n: int
    n_bodies: int
    self_index: int
    first_xyz: tuple[float, float, float]


@dataclass(frozen=True)
class RosterConfig(SensorConfig):
    name: str = "roster"
    rate_hz: float | None = None

    def build(self, rng):
        return RosterSensor(self, rng)


class RosterSensor(Sensor):
    def sample(self, truth: TrueState, world: WorldScene, tick: int) -> Roster:
        fleet = world.fleet
        return Roster(
            n=len(fleet),
            n_bodies=int(
                world.sensing_scene(exclude_object_id=truth.object_id).n_primitives
                - world.static_sensing_scene.n_primitives
            ),
            self_index=fleet.index_of(truth.object_id),
            first_xyz=tuple(float(v) for v in fleet.xyz[0]),
        )


def test_the_runner_feeds_every_drone_to_the_fleet_and_only_active_ones_to_the_bodies():
    """Land half the fleet at tick 5 and crash nothing: the bodies shrink, the fleet does not."""
    seen: list[tuple[int, str, Roster]] = []

    @register_policy("_lands_half")
    class LandsHalf(Policy):
        def step(self, obs):
            seen.append((obs.tick, obs.agent_id, obs.sensors["roster"]))
            if obs.tick == 5 and int(obs.agent_id[-2:]) % 2 == 0:
                return Land()
            return Velocity(vz=0.3)

    runner = Runner(RunConfig(seed=0, policy="_lands_half", sensors=(ToFConfig(), RosterConfig()), **SHORT))
    result = runner.build().run()
    assert set(result.lifecycles.values()) == {Lifecycle.LANDED, Lifecycle.ACTIVE}

    before = [r for t, a, r in seen if t == 5]
    after = [r for t, a, r in seen if t == 8]
    assert before and after
    assert {r.n for r in before} == {10} and {r.n for r in after} == {10}, "the fleet is the whole run"
    assert {r.n_bodies for r in before} == {9}, "every other drone is a body, minus my own"
    assert {r.n_bodies for r in after} == {4}, "five landed drones are no longer bodies"
    # Every sensor found its own slot, and the slot is the run-order index.
    for t, a, r in seen:
        assert r.self_index == int(a[-2:])


def test_the_fleet_reflects_the_post_step_state_the_bodies_are_built_from():
    """R-SENS-13's ordering, extended: drone_00's fleet entry is its own truth this tick."""
    seen = []

    @register_policy("_checks_own_slot")
    class ChecksOwnSlot(Policy):
        def step(self, obs):
            if obs.agent_id == "drone_00":
                seen.append((obs.pose.z, obs.sensors["roster"].first_xyz[2]))
            return Velocity(vz=0.4)

    Runner(RunConfig(seed=0, policy="_checks_own_slot", sensors=(RosterConfig(),), **SHORT)).build().run()
    assert len(seen) > 10
    # The reading was sampled after the previous tick's motion; the pose is that same state.
    for z_pose, z_fleet in seen:
        assert z_pose == pytest.approx(z_fleet)
    assert seen[-1][1] > seen[0][1], "and it is climbing, so the fleet is being refreshed"


def test_a_landed_drone_sits_in_the_fleet_at_floor_level():
    seen = {}

    @register_policy("_one_lands")
    class OneLands(Policy):
        def step(self, obs):
            if obs.agent_id == "drone_01":
                seen[obs.tick] = obs.sensors["roster"]
            if obs.agent_id == "drone_00" and obs.tick == 20:
                return Land()
            return Velocity(vz=0.4)

    Runner(RunConfig(seed=0, policy="_one_lands", sensors=(RosterConfig(),), **SHORT)).build().run()
    assert seen[20].first_xyz[2] > 0.2, "drone_00 was climbing (0.4 m/s through a 0.35 s lag)"
    assert seen[22].first_xyz[2] == 0.0, "and is on the floor once landed, still in the fleet"
    assert seen[22].n == 10
