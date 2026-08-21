"""R-POL-2..8 and R-SEAM-2: the policy surface and the communication seam."""

import dataclasses

import numpy as np
import pytest

from safmc_sim import api
from safmc_sim.api import (
    ArenaInfo,
    COMMAND_TYPES,
    Hold,
    Land,
    Observation,
    Policy,
    PositionWorld,
    Pose,
    Takeoff,
    VelocityBody,
    VelocityWorld,
    get_policy,
    register_policy,
)
from safmc_sim.blackboard import PerfectBlackboard
from safmc_sim.errors import ConfigError


def test_command_set_is_exactly_the_firmware_action_set():
    """R-POL-5. The real drone can do these six things (mavlink_task.h:73-89) and no others."""
    assert {c.__name__ for c in COMMAND_TYPES} == {
        "Takeoff", "VelocityBody", "VelocityWorld", "PositionWorld", "Hold", "Land",
    }


@pytest.mark.parametrize("cls", COMMAND_TYPES)
def test_commands_are_frozen(cls):
    """R-POL-2: a policy must not be able to mutate the command after returning it."""
    assert dataclasses.is_dataclass(cls)
    assert cls.__dataclass_params__.frozen


def test_observation_and_pose_are_frozen():
    assert Observation.__dataclass_params__.frozen
    assert Pose.__dataclass_params__.frozen
    assert ArenaInfo.__dataclass_params__.frozen


def test_observation_exposes_no_route_to_ground_truth():
    """R-POL-4. Walk everything reachable from an Observation and find no world state.

    This is the structural guarantee behind R-POL-3: a policy cannot cheat, not because it is
    asked not to, but because there is nothing to cheat with.
    """
    from safmc_sim.sensors.tof_ring import ToFScan

    scan = ToFScan(
        ranges_m=np.full((8, 8), np.inf), status=np.full((8, 8), 255, np.uint8),
        collapsed_m=np.full(64, np.inf), zone_bearings_rad=np.zeros((8, 8)),
        ranger_bearings_rad=np.zeros(8),
    )
    obs = Observation(
        agent_id="drone_00", tick=0, sim_time_s=0.0,
        pose=Pose(1.0, 2.0, 0.5, 0.0), velocity_xy=(0.0, 0.0), lifecycle="FLYING",
        tof=scan, markers=(), peers={},
        arena=ArenaInfo(20.0, 20.0, 1.4, 6.0, 600.0),
    )

    banned = ("ArenaSpec", "Mission", "Runner", "AgentView", "EnvBase", "ObjectBase", "World")
    seen: set[int] = set()

    def walk(obj, path, depth=0):
        if id(obj) in seen or depth > 4:
            return
        seen.add(id(obj))
        assert type(obj).__name__ not in banned, f"{path} reaches {type(obj).__name__}"
        if isinstance(obj, (str, bytes, int, float, bool, type(None), np.ndarray)):
            return
        if isinstance(obj, dict):
            for k, v in obj.items():
                walk(v, f"{path}[{k!r}]", depth + 1)
            return
        if isinstance(obj, (list, tuple, set)):
            for i, v in enumerate(obj):
                walk(v, f"{path}[{i}]", depth + 1)
            return
        for name in dir(obj):
            if name.startswith("_"):
                continue
            try:
                walk(getattr(obj, name), f"{path}.{name}", depth + 1)
            except Exception:  # noqa: BLE001 -- a property that raises exposes nothing
                pass

    walk(obs, "obs")
    # And the obvious names are simply absent.
    for attribute in ("env", "arena_spec", "mission", "targets", "obstacles", "world"):
        assert not hasattr(obs, attribute)


def test_registry_overwrites_with_a_warning_rather_than_raising():
    """R-POL-6: re-running a notebook cell must not crash."""

    class A(Policy):
        def step(self, obs):
            return Hold()

    class B(Policy):
        def step(self, obs):
            return Hold()

    register_policy("_dupe_test")(A)
    with pytest.warns(UserWarning, match="re-registered"):
        register_policy("_dupe_test")(B)
    assert get_policy("_dupe_test") is B
    # Registering the identical class again is a no-op, not a warning.
    register_policy("_dupe_test")(B)
    del api._REGISTRY["_dupe_test"]


def test_registry_rejects_non_policies_and_unknown_names():
    with pytest.raises(ConfigError):
        register_policy("_bad")(object)
    with pytest.raises(ConfigError, match="unknown policy"):
        get_policy("_definitely_not_registered")


def test_blackboard_publications_are_invisible_until_committed():
    """R-POL-8: the one-tick delay is what makes agent order irrelevant."""
    board = PerfectBlackboard()
    board.publish("a", {"k": 1})
    assert dict(board.snapshot("b")) == {}
    board.commit()
    assert dict(board.snapshot("b")["a"]) == {"k": 1}


def test_every_reader_sees_the_same_snapshot_regardless_of_order():
    board = PerfectBlackboard()
    board.publish("a", {"v": 1})
    board.publish("b", {"v": 2})
    board.commit()
    views = [board.snapshot(agent) for agent in ("a", "b", "c")]
    assert all(v is views[0] for v in views)


def test_snapshot_is_read_only():
    board = PerfectBlackboard()
    board.publish("a", {"v": 1})
    board.commit()
    with pytest.raises(TypeError):
        board.snapshot("b")["a"] = {}
    with pytest.raises(TypeError):
        board.snapshot("b")["a"]["v"] = 99


def test_publications_accumulate_across_ticks():
    board = PerfectBlackboard()
    board.publish("a", {"x": 1})
    board.commit()
    board.publish("a", {"y": 2})
    board.commit()
    assert dict(board.snapshot("b")["a"]) == {"x": 1, "y": 2}


def test_reset_clears_everything():
    board = PerfectBlackboard()
    board.publish("a", {"x": 1})
    board.commit()
    board.reset()
    assert dict(board.snapshot("a")) == {}


def test_policy_outbox_drains_once():
    class P(Policy):
        def step(self, obs):
            return Hold()

    policy = P("d0", {}, np.random.default_rng(0), ArenaInfo(20, 20, 1.4, 6, 600))
    policy.publish("k", 1)
    assert policy.drain_outbox() == {"k": 1}
    assert policy.drain_outbox() == {}


def test_policy_config_is_read_only():
    class P(Policy):
        def step(self, obs):
            return Hold()

    policy = P("d0", {"a": 1}, np.random.default_rng(0), ArenaInfo(20, 20, 1.4, 6, 600))
    with pytest.raises(TypeError):
        policy.config["a"] = 2
