"""The tick loop: builds a run, drives it, and returns a result.

This is a **thin wrapper over ir-sim**, and thin is a requirement rather than an aspiration.
The runner resolves a policy's command into an ir-sim action, steps the world, observes what
happened, and records it. It contains no guidance, no controllers, no manoeuvre choreography
and no altitude hold. An earlier version had all four -- a path follower behind a "go to (x,y)"
command, proportional controllers on yaw and altitude, and canned take-off and landing
sequences the runner flew itself. All of it was strategy living in the platform, and all of it
is gone.

A run is fully specified by ``(scenario, seed, policy, policy_config)`` and nothing else
(R-DET-1). Given the same four, the recorded log is byte-identical.

One tick, in order, and the order is load-bearing:

1. **Freeze the blackboard.** Every agent this tick reads the same immutable view. Without
   this, agent 0's publication would reach agent 1 but not the reverse, and results would
   depend on the order agents happen to be indexed (R-POL-8).
2. **Build each Observation** from the pose source, the sensors' latest samples, and that
   frozen view.
3. **Call every policy.** An exception propagates with agent id and tick attached. There is no
   catch-and-hover: a silently degraded agent produces a plausible wrong answer (R-POL-9).
4. **Resolve commands** through the lifecycle state machine into ir-sim actions.
5. **``env.step()``** -- ir-sim integrates every object from the pre-step state, rebuilds its
   collision tree, then steps every sensor. That ordering is ir-sim's own guarantee and is why
   there is no temporal skew between agents.
6. **Update collisions and the mission.**
7. **Record, then commit the blackboard** so publications become visible next tick.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field, replace
from typing import Any, Mapping

import numpy as np

from . import constants as K
from .api import (
    ArenaInfo,
    Command,
    Land,
    Lifecycle,
    Observation,
    Pose,
    Velocity,
    get_policy,
)
from .blackboard import Blackboard, PerfectBlackboard
from .errors import ConfigError, PolicyError
from .frames import wrap_pi
from .kinematics import KINEMATICS_NAME, QuadParams, configure_robots
from .mission import Event, Mission, ScoreBreakdown
from .pose import GroundTruthPose, PoseSource
from .sensors.marker_cam import MarkerCam, MarkerCamConfig
from .sensors.scene import WorldScene
from .sensors.tof_ring import ToFConfig, ToFScan
from .sensors import tof_ring as tof_ring_module
from .world.arena import ArenaConfig, ArenaSpec, generate_arena

__all__ = ["RunConfig", "RunResult", "AgentView", "Runner", "run"]

# The action for a drone that is not going anywhere.
_STOP = [0.0, 0.0, 0.0, 0.0]



@dataclass(frozen=True)
class RunConfig:
    """Everything that defines a run. Frozen, serialisable, and logged verbatim."""

    seed: int = K.DEFAULT_SEED
    n_drones: int = K.FLEET_MIN
    policy: str = "hold"
    policy_config: Mapping[str, Any] = field(default_factory=dict)

    tick_hz: float = K.DEFAULT_TICK_HZ
    duration_s: float = K.RUN_DURATION_S

    arena_config: ArenaConfig = field(default_factory=ArenaConfig)
    quad_params: QuadParams = field(default_factory=QuadParams)
    tof_config: ToFConfig = field(default_factory=ToFConfig)
    marker_config: MarkerCamConfig = field(default_factory=MarkerCamConfig)

    tof_rate_hz: float | None = None
    """``None`` samples every tick. Otherwise must divide ``tick_hz`` exactly (R-TIME-3).

    The real ring runs at 15 Hz round-robin with up to 64 ms of skew across sensors
    (assumption A-8); v0.1 samples it synchronously."""

    marker_rate_hz: float = K.MARKER_RATE_HZ
    """Default 2 Hz, the measured AprilTag rate on the real hardware."""

    sample_hz: float | None = None
    """Render sampling rate. ``None`` means every tick. Must not exceed ``tick_hz``: ir-sim
    computes ``int(sample_time / step_time)`` and divides by it, so a faster sample rate
    produces a bare ZeroDivisionError from inside its world step (R-TIME-4)."""

    start_spacing_m: float = K.START_SPACING_M
    """Take-off grid spacing. See constants.START_SPACING_M -- too small deadlocks reactive
    policies against their own neighbours before anyone leaves the Start Area."""

    collision_behaviour: str = "stop"
    """``stop`` freezes a colliding drone permanently, which is faithful -- the rules allow no
    mid-run repair. ``unobstructed`` disables collision so drones fly through obstacles.

    Both exist because this choice is a **load-bearing confound**, not a detail. Recon of
    arXiv:2607.25195 measured half a four-drone team dead 35 s into a 900 s episode under
    ``stop``; its headline coverage gain is entangled with "fewer deaths means more live
    agent-seconds". Any comparison from this simulator should report which mode it used, and
    coverage should be normalised by live-agent-seconds."""

    record: bool = True

    def __post_init__(self) -> None:
        if not K.FLEET_MIN <= self.n_drones <= K.FLEET_MAX:
            raise ConfigError(
                f"n_drones must be in [{K.FLEET_MIN}, {K.FLEET_MAX}], got {self.n_drones}. "
                f"Fewer than {K.FLEET_MIN} drones forfeits the run (rulebook 3.3.1, Penalty #2)."
            )
        if self.tick_hz <= 0:
            raise ConfigError(f"tick_hz must be > 0, got {self.tick_hz}")
        if self.duration_s <= 0:
            raise ConfigError(f"duration_s must be > 0, got {self.duration_s}")
        if self.start_spacing_m <= 2.0 * K.DRONE_RADIUS_M:
            raise ConfigError(
                f"start_spacing_m {self.start_spacing_m} must exceed the drone diameter "
                f"{2 * K.DRONE_RADIUS_M}; drones would start overlapping"
            )
        if self.collision_behaviour not in ("stop", "unobstructed"):
            raise ConfigError(
                f"collision_behaviour must be 'stop' or 'unobstructed', "
                f"got {self.collision_behaviour!r}"
            )
        if self.sample_hz is not None and self.sample_hz > self.tick_hz:
            raise ConfigError(
                f"sample_hz {self.sample_hz} exceeds tick_hz {self.tick_hz}. ir-sim computes "
                f"int(sample_time / step_time) and would raise an opaque ZeroDivisionError "
                f"deep inside world.step() (world.py:171)."
            )
        _decimation(self.tof_rate_hz, self.tick_hz, "tof_rate_hz")
        _decimation(self.marker_rate_hz, self.tick_hz, "marker_rate_hz")

    @property
    def dt(self) -> float:
        return 1.0 / self.tick_hz

    @property
    def max_ticks(self) -> int:
        return int(round(self.duration_s * self.tick_hz))


def _decimation(rate_hz: float | None, tick_hz: float, name: str) -> int:
    """Convert a rate into an integer tick decimation, or refuse (R-TIME-3)."""
    if rate_hz is None:
        return 1
    if rate_hz <= 0:
        raise ConfigError(f"{name} must be > 0, got {rate_hz}")
    ratio = tick_hz / rate_hz
    nearest = round(ratio)
    if nearest < 1 or abs(ratio - nearest) > 1e-9:
        raise ConfigError(
            f"{name}={rate_hz} Hz does not divide tick_hz={tick_hz} Hz exactly "
            f"(ratio {ratio:.6f}). Pick a rate that divides the tick rate, or change "
            f"tick_hz -- silently rounding a sensor rate would misreport sensor latency."
        )
    return int(nearest)


@dataclass
class AgentView:
    """Per-drone bookkeeping owned by the runner. Never handed to a policy."""

    agent_id: str
    robot: Any
    policy: Any
    lifecycle: str = Lifecycle.ACTIVE
    last_command: Command = field(default_factory=Velocity)
    last_scan: ToFScan | None = None
    last_scan_tick: int = -1
    last_markers: tuple = ()
    last_marker_tick: int = -1
    crash_reason: str | None = None
    left_start_area: bool = False

    @property
    def state(self) -> np.ndarray:
        return self.robot.state

    @property
    def xy(self) -> np.ndarray:
        return np.array([float(self.state[0, 0]), float(self.state[1, 0])])

    @property
    def z(self) -> float:
        return float(self.state[3, 0])

    @property
    def terminal(self) -> bool:
        return self.lifecycle in Lifecycle.TERMINAL


@dataclass(frozen=True)
class RunResult:
    config: RunConfig
    arena: ArenaSpec
    score: ScoreBreakdown
    events: tuple[Event, ...]
    ticks: int
    sim_time_s: float
    wall_time_s: float
    mission_started_tick: int | None
    lifecycles: Mapping[str, str]
    mission_summary: Mapping[str, Any]
    log_path: str | None = None

    @property
    def crashed(self) -> tuple[str, ...]:
        return tuple(sorted(a for a, s in self.lifecycles.items() if s == Lifecycle.CRASHED))

    @property
    def landed(self) -> tuple[str, ...]:
        return tuple(sorted(a for a, s in self.lifecycles.items() if s == Lifecycle.LANDED))


class Runner:
    """Builds and drives one run."""

    def __init__(self, config: RunConfig, recorder=None) -> None:
        self.config = config
        if recorder is not None and not config.record:
            raise ConfigError(
                "a recorder was supplied but RunConfig.record is False. Set record=True, or "
                "pass recorder=None -- silently ignoring one of the two would make the log's "
                "absence look like a simulator decision rather than a configuration mistake."
            )
        self._recorder = recorder
        self._seeds = np.random.SeedSequence(config.seed).spawn(4)
        self.arena = generate_arena(config.seed, config.arena_config)
        self.mission = Mission(self.arena)
        self.blackboard: Blackboard = PerfectBlackboard()
        self.pose_source: PoseSource = GroundTruthPose()
        self.world_scene = WorldScene(self.arena.structural_scene(), self.arena.marker_scene())
        self.marker_cam = MarkerCam(config.marker_config)

        self._tof_decimation = _decimation(config.tof_rate_hz, config.tick_hz, "tof_rate_hz")
        self._marker_decimation = _decimation(
            config.marker_rate_hz, config.tick_hz, "marker_rate_hz"
        )

        self.arena_info = ArenaInfo(
            width_m=self.arena.width_m,
            depth_m=self.arena.depth_m,
            ceiling_m=self.arena.ceiling_m,
            start_area_depth_m=self.arena.start_area_depth_m,
            run_duration_s=config.duration_s,
        )

        self.env = None
        self.agents: list[AgentView] = []
        self.events: list[Event] = []
        self._mission_started_tick: int | None = None
        self._tick = 0

    # -- construction ------------------------------------------------------------------------

    def _start_positions(self) -> np.ndarray:
        """A grid formation inside the Start Area, seeded and non-overlapping.

        The rules require simultaneous take-off from the Start Area, so drones begin packed in
        the southern strip rather than scattered. Spacing is ``START_SPACING_M``, which is
        chosen to keep neighbours outside a typical avoidance threshold -- see the constant's
        docstring for why packing tighter deadlocks the whole fleet at t=0.
        """
        rng = np.random.default_rng(self._seeds[0])
        n = self.config.n_drones
        spacing = self.config.start_spacing_m
        per_row = max(1, int((self.arena.width_m - 2.0 * K.START_WALL_MARGIN_M) // spacing))
        rows = int(np.ceil(n / per_row))
        margin = K.START_WALL_MARGIN_M
        depth_needed = rows * spacing
        usable = self.arena.start_area_depth_m - margin
        if depth_needed > usable:
            raise ConfigError(
                f"{n} drones at {spacing:.2f} m spacing need {depth_needed:.1f} m of Start "
                f"Area depth but only {usable:.1f} m is usable after the "
                f"{margin} m wall margin"
            )
        x0 = margin + float(rng.uniform(0.0, 0.5))
        y0 = margin + float(rng.uniform(0.0, 0.5))
        states = []
        for i in range(n):
            row, col = divmod(i, per_row)
            states.append(
                [x0 + col * spacing, y0 + row * spacing, np.pi / 2.0, 0.0, 0.0, 0.0]
            )
        return np.array(states)

    def build(self) -> "Runner":
        import irsim

        tof_ring_module.install()

        world = self.arena.to_irsim_world(self.config.dt)
        if self.config.sample_hz is not None:
            world["sample_time"] = 1.0 / self.config.sample_hz
        world["collision_mode"] = (
            "stop" if self.config.collision_behaviour == "stop" else "unobstructed"
        )
        cfg = {
            "world": world,
            "robot": [
                {
                    "number": self.config.n_drones,
                    "kinematics": {"name": KINEMATICS_NAME},
                    "shape": {"name": "circle", "radius": K.DRONE_RADIUS_M},
                    "state": self._start_positions().tolist(),
                    "sensors": [{"name": tof_ring_module.SENSOR_TYPE}],
                }
            ],
            "obstacle": self.arena.to_irsim_obstacles(),
        }

        import tempfile, os
        import yaml

        handle = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        yaml.safe_dump(cfg, handle)
        handle.close()
        try:
            self.env = irsim.make(
                handle.name, display=False, disable_all_plot=True,
                seed=self.config.seed, log_level="ERROR",
            )
        finally:
            os.unlink(handle.name)

        configure_robots(self.env.robot_list, self.config.quad_params)

        policy_cls = get_policy(self.config.policy)
        policy_seeds = np.random.SeedSequence(self._seeds[1].entropy, spawn_key=self._seeds[1].spawn_key).spawn(
            self.config.n_drones
        )
        sensor_seeds = np.random.SeedSequence(self._seeds[2].entropy, spawn_key=self._seeds[2].spawn_key).spawn(
            self.config.n_drones
        )

        for i, robot in enumerate(self.env.robot_list):
            agent_id = f"drone_{i:02d}"
            for sensor in robot.sensors:
                sensor.config = replace(self.config.tof_config)
                sensor.attach(self.world_scene, np.random.default_rng(sensor_seeds[i]))
            policy = policy_cls(
                agent_id=agent_id,
                config=self.config.policy_config,
                rng=np.random.default_rng(policy_seeds[i]),
                arena=self.arena_info,
            )
            policy.reset()
            self.agents.append(
                AgentView(
                    agent_id=agent_id,
                    robot=robot,
                    policy=policy,
                )
            )
        return self

    # -- the loop ------------------------------------------------------------------------------

    def run(self) -> RunResult:
        if self.env is None:
            self.build()
        started = time.perf_counter()

        if self._recorder is not None:
            self._recorder.begin(self.config, self.arena, [a.agent_id for a in self.agents])

        try:
            for tick in range(self.config.max_ticks):
                self._tick = tick
                if self._tick_once(tick):
                    break
        except BaseException:
            # A policy that raises still aborts the run (R-POL-9), but it must not also leak
            # an ir-sim environment and a matplotlib figure per attempt.
            self._teardown()
            raise

        sim_time = (self._tick + 1) * self.config.dt
        landed = self._landed_positions()
        score = self.mission.score(landed)
        result = RunResult(
            config=self.config,
            arena=self.arena,
            score=score,
            events=tuple(self.events),
            ticks=self._tick + 1,
            sim_time_s=sim_time,
            wall_time_s=time.perf_counter() - started,
            mission_started_tick=self._mission_started_tick,
            lifecycles={a.agent_id: a.lifecycle for a in self.agents},
            mission_summary=self.mission.summary(),
            log_path=None,
        )
        if self._recorder is not None:
            path = self._recorder.finish(result)
            result = replace(result, log_path=path)
        self._teardown()
        return result

    def _tick_once(self, tick: int) -> bool:
        """Advance one tick. Returns True when the run should stop."""
        dt = self.config.dt
        sim_time = tick * dt

        snapshot_by_agent = {a.agent_id: self.blackboard.snapshot(a.agent_id) for a in self.agents}

        commands: list[Command] = []
        for agent in self.agents:
            obs = self._observe(agent, tick, sim_time, snapshot_by_agent[agent.agent_id])
            if agent.terminal:
                commands.append(Velocity())
                continue
            try:
                command = agent.policy.step(obs)
            except Exception as exc:  # noqa: BLE001 -- re-raised with context, never swallowed
                raise PolicyError(
                    f"policy {self.config.policy!r} for {agent.agent_id} raised at tick "
                    f"{tick} (t={sim_time:.2f}s)"
                ) from exc
            if not isinstance(command, tuple(_COMMAND_TYPES)):
                raise PolicyError(
                    f"policy {self.config.policy!r} for {agent.agent_id} returned "
                    f"{type(command).__name__}; expected one of "
                    f"{[c.__name__ for c in _COMMAND_TYPES]}"
                )
            _check_finite(command, agent.agent_id, tick, self.config.policy)
            commands.append(command)
            self.blackboard.publish(agent.agent_id, agent.policy.drain_outbox())

        actions = [self._resolve(agent, command, tick, sim_time)
                   for agent, command in zip(self.agents, commands)]

        self.env.step(actions, action_id=[a.robot.id for a in self.agents])

        self._post_step(tick, sim_time + dt)

        if self._recorder is not None:
            self._recorder.tick(tick, sim_time + dt, self.agents, commands)

        self.blackboard.commit()

        return self._should_stop(tick)

    # -- observation ---------------------------------------------------------------------------

    def _observe(self, agent: AgentView, tick: int, sim_time: float, peers) -> Observation:
        sensor = agent.robot.sensors[0]
        if tick % self._tof_decimation == 0 and not agent.terminal:
            # On tick 0 no env.step() has happened, so no sensor has sampled yet. Sample it
            # directly rather than handing the policy a placeholder for its first decision.
            scan = sensor.latest_scan
            agent.last_scan = scan if scan is not None else sensor.step(agent.state[0:3])
            agent.last_scan_tick = tick
        elif agent.last_scan is None:
            agent.last_scan = sensor.step(agent.state[0:3])
            agent.last_scan_tick = tick

        if tick % self._marker_decimation == 0 and not agent.terminal:
            agent.last_markers = self.marker_cam.detect(
                agent.xy, float(agent.state[2, 0]), agent.z,
                self.arena.targets, self.world_scene.static_sensing_scene,
            )
            agent.last_marker_tick = tick

        pose = self.pose_source.pose_of(agent.agent_id, agent.state, tick)
        return Observation(
            agent_id=agent.agent_id,
            tick=tick,
            sim_time_s=sim_time,
            pose=pose,
            velocity_xy=self.pose_source.velocity_of(agent.agent_id, agent.state, tick),
            lifecycle=agent.lifecycle,
            tof=agent.last_scan,
            markers=agent.last_markers,
            peers=peers,
            arena=self.arena_info,
            tof_stale_ticks=tick - agent.last_scan_tick,
            marker_stale_ticks=tick - agent.last_marker_tick,
        )

    # -- command resolution ---------------------------------------------------------------------

    def _resolve(self, agent: AgentView, command: Command, tick: int, sim_time: float):
        """Turn a command into an ir-sim action. That is the whole job.

        A velocity passes straight through; the drone's own limits and first-order lag are
        applied by the kinematics handler, which is where they belong. ``Land`` is a state
        transition, not a manoeuvre.
        """
        agent.last_command = command
        if agent.terminal:
            return _STOP

        if isinstance(command, Velocity):
            return [command.vx, command.vy, command.vz, command.yaw_rate]

        if isinstance(command, Land):
            self._land(agent, tick, sim_time)
            return _STOP

        raise PolicyError(f"unhandled command type {type(command).__name__}")

    def _land(self, agent: AgentView, tick: int, sim_time: float) -> None:
        """Put a drone on the ground here, permanently.

        The descent is not modelled -- the drone settles to the floor in the tick it commits
        (divergence F-13). A policy that wants to fly its own approach can descend with
        ``Velocity(vz=...)`` first and issue ``Land`` at the bottom; the commitment is this
        call either way.
        """
        self._freeze(agent)
        agent.robot._state[3, 0] = 0.0
        agent.lifecycle = Lifecycle.LANDED
        self._emit(
            tick, sim_time, "landed", agent.agent_id,
            {"x": float(agent.xy[0]), "y": float(agent.xy[1])},
        )

    # -- observation ------------------------------------------------------------------------------

    # -- post-step --------------------------------------------------------------------------------

    def _post_step(self, tick: int, sim_time: float) -> None:
        for agent in self.agents:
            if agent.terminal:
                continue

            reason = self._collision_reason(agent)
            if reason is not None:
                self._freeze(agent)
                agent.lifecycle = Lifecycle.CRASHED
                agent.crash_reason = reason
                self._emit(
                    tick, sim_time, "crashed", agent.agent_id,
                    {"reason": reason, "x": float(agent.xy[0]), "y": float(agent.xy[1])},
                )
                continue

            if self._out_of_bounds(agent):
                self._freeze(agent)
                agent.lifecycle = Lifecycle.CRASHED
                agent.crash_reason = "left the field"
                self._emit(
                    tick, sim_time, "crashed", agent.agent_id,
                    {"reason": "left the field", "x": float(agent.xy[0]),
                     "y": float(agent.xy[1])},
                )
                continue

            # Departures are *recorded*, never refused. The two-wave take-off rule is a
            # competition rule, so it is scored from these events after the fact rather than
            # enforced by the platform -- the simulator's job is to report what the fleet did,
            # not to referee it mid-flight.
            if not agent.left_start_area and not self.arena.in_start_area(*agent.xy):
                agent.left_start_area = True
                self._emit(tick, sim_time, "departed", agent.agent_id, {})
                if self._mission_started_tick is None:
                    self._mission_started_tick = tick
                    self._emit(
                        tick, sim_time, "mission_started", agent.agent_id,
                        {"note": "first drone left the Start Area; the run clock starts here"},
                    )

        self.events.extend(self.mission.update(tick, sim_time, self._landed_positions()))

    @staticmethod
    def _freeze(agent: AgentView) -> None:
        """Bring a drone to a dead stop, in state, at the moment it becomes terminal.

        Zeroing the *command* is not enough. Velocity is carried in the state (rows 4 and 5)
        and decays through a first-order lag, so a drone that lands while still moving keeps
        sliding for a second or more afterwards. Measured at up to 116 mm -- 12% of the 1 m
        scoring radius -- which was enough to score a target the drone had not actually reached
        at the moment it touched down, and to make offline re-scoring disagree with the online
        result (R-DRONE-10, R-MISS-8). Mission.update's whole recompute-from-scratch design
        rests on "a landed drone never moves"; this is what makes that true.
        """
        agent.robot._state[4, 0] = 0.0
        agent.robot._state[5, 0] = 0.0
        agent.robot._velocity = np.zeros_like(agent.robot._velocity)

    def _collision_reason(self, agent: AgentView) -> str | None:
        if self.config.collision_behaviour == "stop" and bool(getattr(agent.robot, "collision", False)):
            other = getattr(agent.robot, "collision_obj", None)
            name = getattr(other[0], "name", "obstacle") if other else "obstacle"
            return f"collided with {name}"

        if self.config.collision_behaviour != "stop":
            # 'unobstructed' must disable ALL collision, not just ir-sim's. Leaving markers
            # lethal here made the control mode not a control: a policy could still lose
            # drones in the run that was supposed to isolate search strategy from crashes.
            return None

        # Mission markers are not ir-sim obstacles, because ir-sim's collision is strictly 2D
        # and would make a 1.0 m marker impassable at every altitude. Height-gated here instead.
        for target in self.arena.targets:
            if agent.z >= target.height_m:
                continue
            reach = K.DRONE_RADIUS_M + target.radius_m
            if float(np.hypot(agent.xy[0] - target.x, agent.xy[1] - target.y)) < reach:
                return f"struck marker {target.id}"
        return None

    def _out_of_bounds(self, agent: AgentView) -> bool:
        """Has the drone left the field?

        Needed because ``collision_behaviour="unobstructed"`` disables ir-sim's collision
        entirely, and ir-sim has no implicit world bounds -- a drone was measured 55 m into a
        20 m field. Leaving the arena is not a search strategy, and letting it happen silently
        corrupts every coverage number computed against the field's free area.
        """
        margin = K.DRONE_RADIUS_M
        x, y = agent.xy
        return not (
            -margin <= x <= self.arena.width_m + margin
            and -margin <= y <= self.arena.depth_m + margin
        )

    def _landed_positions(self) -> dict[str, np.ndarray]:
        return {a.agent_id: a.xy for a in self.agents if a.lifecycle == Lifecycle.LANDED}

    def _should_stop(self, tick: int) -> bool:
        if all(a.terminal for a in self.agents):
            self._emit(
                tick, (tick + 1) * self.config.dt, "all_agents_terminal", None,
                {"landed": len(self._landed_positions())},
            )
            return True
        return False

    def _emit(self, tick, sim_time, kind, agent_id, detail) -> None:
        self.events.append(
            Event(tick=tick, sim_time_s=sim_time, kind=kind, agent_id=agent_id, detail=detail)
        )

    def _teardown(self) -> None:
        if self.env is not None:
            self.env.end(0)
            # ir-sim creates a matplotlib figure unconditionally, even fully headless, and
            # frees it in neither `del env` nor `env.end()`. An episodic loop leaks without
            # this. See docs/03-irsim.md landmine 14.
            import matplotlib.pyplot as plt

            plt.close("all")
            self.env = None


_COMMAND_TYPES = (Velocity, Land)

_COMMAND_FLOAT_FIELDS = {
    Velocity: ("vx", "vy", "vz", "yaw_rate"),
    Land: (),
}


def _check_finite(command, agent_id: str, tick: int, policy: str) -> None:
    """Reject NaN and infinity in a command, naming the agent and tick.

    The commonest policy bug is a 0/0 normalisation. Without this, the NaN propagates into the
    drone's position, ir-sim tries to build collision geometry from it, and the run dies inside
    shapely with `GEOSException: Points of LinearRing do not form a closed linestring` -- no
    agent, no tick, no line of policy code. The type of the command was already validated with
    a clear error; its values deserve the same.
    """
    for field_name in _COMMAND_FLOAT_FIELDS[type(command)]:
        value = getattr(command, field_name)
        if value is None:
            continue
        if not np.isfinite(value):
            raise PolicyError(
                f"policy {policy!r} for {agent_id} returned "
                f"{type(command).__name__}.{field_name}={value!r} at tick {tick}. "
                f"Commands must be finite -- a NaN here would propagate into the drone's "
                f"position and fail later inside shapely with no reference to your code."
            )


def run(config: RunConfig, recorder=None) -> RunResult:
    """Build and drive one run. The one-liner entry point."""
    return Runner(config, recorder=recorder).build().run()
