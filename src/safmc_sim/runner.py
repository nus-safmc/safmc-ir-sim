"""The tick loop: builds a run, drives it, and returns a result.

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
    Hold,
    Land,
    Lifecycle,
    Observation,
    PositionWorld,
    Pose,
    Takeoff,
    VelocityBody,
    VelocityWorld,
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

# Proportional gain on yaw error, in 1/s. High enough to be effectively bang-bang against the
# rate limit for large errors, low enough not to chatter near the setpoint.
_YAW_GAIN = 4.0
# Proportional gain on altitude error, in 1/s.
_ALT_GAIN = 3.0
_TAKEOFF_TOLERANCE_M = 0.03
_LANDED_TOLERANCE_M = 0.02


@dataclass(frozen=True)
class RunConfig:
    """Everything that defines a run. Frozen, serialisable, and logged verbatim."""

    seed: int = K.DEFAULT_SEED
    n_drones: int = K.FLEET_MIN
    policy: str = "hold"
    policy_config: Mapping[str, Any] = field(default_factory=dict)

    tick_hz: float = K.DEFAULT_TICK_HZ
    duration_s: float = K.RUN_DURATION_S
    cruise_alt_m: float = K.CRUISE_ALT_M

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
        if self.collision_behaviour not in ("stop", "unobstructed"):
            raise ConfigError(
                f"collision_behaviour must be 'stop' or 'unobstructed', "
                f"got {self.collision_behaviour!r}"
            )
        if self.cruise_alt_m <= 0 or self.cruise_alt_m > self.quad_params.ceiling_m:
            raise ConfigError(
                f"cruise_alt_m {self.cruise_alt_m} must be in (0, {self.quad_params.ceiling_m}]"
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
    lifecycle: str = Lifecycle.IDLE
    wave: int | None = None
    takeoff_tick: int | None = None
    target_alt_m: float = K.CRUISE_ALT_M
    last_command: Command = field(default_factory=Hold)
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
        self._waves: list[float] = []
        self._mission_started_tick: int | None = None
        self._tick = 0

    # -- construction ------------------------------------------------------------------------

    def _start_positions(self) -> np.ndarray:
        """A grid formation inside the Start Area, seeded and non-overlapping.

        The rules require simultaneous take-off from the Start Area, so drones begin packed in
        the southern strip rather than scattered. Spacing is four drone radii, which is tight
        but keeps a 25-drone fleet inside the 20 x 6 m strip.
        """
        rng = np.random.default_rng(self._seeds[0])
        n = self.config.n_drones
        spacing = 4.0 * K.DRONE_RADIUS_M
        per_row = max(1, int((self.arena.width_m - 2.0) // spacing))
        rows = int(np.ceil(n / per_row))
        depth_needed = rows * spacing
        if depth_needed > self.arena.start_area_depth_m - 1.0:
            raise ConfigError(
                f"{n} drones at {spacing:.2f} m spacing need {depth_needed:.1f} m of Start "
                f"Area depth but only {self.arena.start_area_depth_m - 1.0:.1f} m is usable"
            )
        x0 = 1.0 + float(rng.uniform(0.0, 0.5))
        y0 = 0.5 + float(rng.uniform(0.0, 0.5))
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
                    target_alt_m=self.config.cruise_alt_m,
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

        for tick in range(self.config.max_ticks):
            self._tick = tick
            if self._tick_once(tick):
                break

        sim_time = self._tick * self.config.dt
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
                commands.append(Hold())
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
            velocity_xy=(float(agent.state[4, 0]), float(agent.state[5, 0])),
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
        agent.last_command = command
        if agent.terminal:
            return [0.0, 0.0, 0.0, 0.0]

        params = self.config.quad_params

        if isinstance(command, Takeoff):
            if agent.lifecycle == Lifecycle.IDLE:
                if self._admit_takeoff(agent, tick, sim_time):
                    agent.lifecycle = Lifecycle.TAKEOFF
                    agent.target_alt_m = float(
                        np.clip(command.altitude_m, 0.05, params.ceiling_m)
                    )
                    agent.takeoff_tick = tick
                else:
                    return [0.0, 0.0, 0.0, 0.0]
            # fall through so a TAKEOFF drone keeps climbing

        if agent.lifecycle == Lifecycle.IDLE:
            # Not airborne, and this was not a Takeoff. Nothing else can move a grounded drone.
            return [0.0, 0.0, 0.0, 0.0]

        if isinstance(command, Land) and agent.lifecycle in (Lifecycle.FLYING, Lifecycle.TAKEOFF):
            agent.lifecycle = Lifecycle.LANDING
            self._emit(tick, sim_time, "land_commanded", agent.agent_id, {})

        if agent.lifecycle == Lifecycle.TAKEOFF:
            return [0.0, 0.0, params.climb_rate_max_ms, 0.0]

        if agent.lifecycle == Lifecycle.LANDING:
            return [0.0, 0.0, -params.climb_rate_max_ms, 0.0]

        return self._flying_action(agent, command)

    def _flying_action(self, agent: AgentView, command: Command) -> list[float]:
        theta = float(agent.state[2, 0])
        z = agent.z

        if isinstance(command, (Hold, Takeoff)):
            return [0.0, 0.0, self._alt_rate(z, agent.target_alt_m), 0.0]

        if isinstance(command, VelocityBody):
            cos_t, sin_t = np.cos(theta), np.sin(theta)
            return [
                command.vx * cos_t - command.vy * sin_t,
                command.vx * sin_t + command.vy * cos_t,
                command.vz,
                command.yaw_rate,
            ]

        if isinstance(command, VelocityWorld):
            return [
                command.vx,
                command.vy,
                self._alt_rate(z, command.z),
                self._yaw_rate(theta, command.yaw),
            ]

        if isinstance(command, PositionWorld):
            delta = np.array([command.x, command.y]) - agent.xy
            distance = float(np.linalg.norm(delta))
            if distance < 1e-9:
                vx = vy = 0.0
                heading = command.yaw
            else:
                # Do not overshoot: never command more than the remaining distance per tick.
                speed = min(command.speed_ms, distance / self.config.dt)
                vx, vy = (delta / distance * speed).tolist()
                heading = command.yaw if command.yaw is not None else float(
                    np.arctan2(delta[1], delta[0])
                )
            return [vx, vy, self._alt_rate(z, command.z), self._yaw_rate(theta, heading)]

        raise PolicyError(f"unhandled command type {type(command).__name__}")

    def _alt_rate(self, z: float, target: float) -> float:
        params = self.config.quad_params
        return float(
            np.clip(_ALT_GAIN * (target - z), -params.climb_rate_max_ms, params.climb_rate_max_ms)
        )

    def _yaw_rate(self, theta: float, target: float | None) -> float:
        if target is None:
            return 0.0
        params = self.config.quad_params
        return float(
            np.clip(_YAW_GAIN * wrap_pi(target - theta), -params.yaw_rate_max, params.yaw_rate_max)
        )

    # -- rules ------------------------------------------------------------------------------------

    def _admit_takeoff(self, agent: AgentView, tick: int, sim_time: float) -> bool:
        """The two-wave rule (R-MISS-6, rulebook 3.3.2).

        A wave is a group whose last departure falls within 10 s of its first. At most two
        waves exist; a third is refused and recorded, not raised -- the run continues and the
        violation shows up in the log, which is what an evaluator needs to see.
        """
        if self._waves and sim_time - self._waves[-1] <= K.TAKEOFF_WAVE_WINDOW_S:
            agent.wave = len(self._waves)
            return True
        if len(self._waves) >= K.MAX_TAKEOFF_WAVES:
            self._emit(
                tick, sim_time, "rule_violation", agent.agent_id,
                {
                    "rule": "max_takeoff_waves",
                    "detail": (
                        f"take-off refused: {K.MAX_TAKEOFF_WAVES} waves already used and this "
                        f"request is {sim_time - self._waves[-1]:.1f}s after wave "
                        f"{len(self._waves)} opened (window is {K.TAKEOFF_WAVE_WINDOW_S}s)"
                    ),
                },
            )
            return False
        self._waves.append(sim_time)
        agent.wave = len(self._waves)
        self._emit(tick, sim_time, "wave_opened", agent.agent_id, {"wave": len(self._waves)})
        return True

    # -- post-step --------------------------------------------------------------------------------

    def _post_step(self, tick: int, sim_time: float) -> None:
        for agent in self.agents:
            if agent.terminal:
                continue

            if agent.lifecycle == Lifecycle.TAKEOFF and agent.z >= agent.target_alt_m - _TAKEOFF_TOLERANCE_M:
                agent.lifecycle = Lifecycle.FLYING
                self._emit(tick, sim_time, "airborne", agent.agent_id, {"z": agent.z})

            if agent.lifecycle == Lifecycle.LANDING and agent.z <= _LANDED_TOLERANCE_M:
                agent.lifecycle = Lifecycle.LANDED
                self._emit(
                    tick, sim_time, "landed", agent.agent_id,
                    {"x": float(agent.xy[0]), "y": float(agent.xy[1])},
                )
                continue

            reason = self._collision_reason(agent)
            if reason is not None:
                agent.lifecycle = Lifecycle.CRASHED
                agent.crash_reason = reason
                self._emit(
                    tick, sim_time, "crashed", agent.agent_id,
                    {"reason": reason, "x": float(agent.xy[0]), "y": float(agent.xy[1])},
                )
                continue

            if not agent.left_start_area and not self.arena.in_start_area(*agent.xy):
                agent.left_start_area = True
                if self._mission_started_tick is None:
                    self._mission_started_tick = tick
                    self._emit(
                        tick, sim_time, "mission_started", agent.agent_id,
                        {"note": "first drone left the Start Area; the run clock starts here"},
                    )

        self.events.extend(self.mission.update(tick, sim_time, self._landed_positions()))

    def _collision_reason(self, agent: AgentView) -> str | None:
        if self.config.collision_behaviour == "stop" and bool(getattr(agent.robot, "collision", False)):
            other = getattr(agent.robot, "collision_obj", None)
            name = getattr(other[0], "name", "obstacle") if other else "obstacle"
            return f"collided with {name}"

        # Mission markers are not ir-sim obstacles, because ir-sim's collision is strictly 2D
        # and would make a 1.0 m marker impassable at every altitude. Height-gated here instead.
        if agent.lifecycle in Lifecycle.AIRBORNE or agent.lifecycle == Lifecycle.LANDED:
            for target in self.arena.targets:
                if agent.z >= target.height_m:
                    continue
                reach = K.DRONE_RADIUS_M + target.radius_m
                if float(np.hypot(agent.xy[0] - target.x, agent.xy[1] - target.y)) < reach:
                    return f"struck marker {target.id}"
        return None

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


_COMMAND_TYPES = (Takeoff, VelocityBody, VelocityWorld, PositionWorld, Hold, Land)


def run(config: RunConfig, recorder=None) -> RunResult:
    """Build and drive one run. The one-liner entry point."""
    return Runner(config, recorder=recorder).build().run()
