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
2. **Build each Observation** from the pose source, every sensor's latest reading, and that
   frozen view.
3. **Call every policy.** An exception propagates with agent id and tick attached. There is no
   catch-and-hover: a silently degraded agent produces a plausible wrong answer (R-POL-9).
4. **Resolve commands** through the lifecycle state machine into ir-sim actions.
5. **``env.step()``** -- ir-sim integrates every object from the pre-step state and rebuilds
   its collision tree.
6. **Sense.** Every due sensor on every active drone samples the post-motion world, so no
   agent is measured against a staler picture than another. The runner drives every sensor
   through one contract (``sensors/base.py``); it does not know what any of them are.
7. **Update collisions and the mission.**
8. **Record, then commit the blackboard** so publications become visible next tick.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field, replace
from types import MappingProxyType
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
from .pose import POSE_SOURCES, PoseSource
from .sensors.base import Sensor, SensorConfig, TrueState, decimation
from .sensors.marker_cam import MarkerCamConfig
from .sensors.scene import WorldScene
from .sensors.tof_ring import ToFConfig
from .world.arena import ArenaConfig, ArenaSpec, generate_arena

__all__ = ["RunConfig", "RunResult", "AgentView", "Runner", "run", "flown_sensors"]

# The action for a drone that is not going anywhere.
_STOP = [0.0, 0.0, 0.0, 0.0]


def flown_sensors() -> tuple[SensorConfig, ...]:
    """The sensors the real airframe carries, with the flown geometry and rates.

    The default for :attr:`RunConfig.sensors`. Extend it rather than replacing it when adding
    a sensor -- ``RunConfig(sensors=flown_sensors() + (BeaconConfig(),))`` -- so a run still
    carries the hardware that exists. See ``sensors/base.py`` for how to write one.
    """
    return (ToFConfig(), MarkerCamConfig())


@dataclass(frozen=True)
class RunConfig:
    """Everything that defines a run. Frozen, serialisable, and logged verbatim."""

    seed: int = K.DEFAULT_SEED
    n_drones: int = K.FLEET_MIN
    policy: str = "sdlw"
    policy_config: Mapping[str, Any] = field(default_factory=dict)

    tick_hz: float = K.DEFAULT_TICK_HZ
    duration_s: float = K.RUN_DURATION_S

    arena_config: ArenaConfig = field(default_factory=ArenaConfig)
    quad_params: QuadParams = field(default_factory=QuadParams)

    sensors: tuple[SensorConfig, ...] = field(default_factory=flown_sensors)
    """What every drone carries. Each config is built once per drone, with that drone's own
    generator, and driven by the runner through the contract in ``sensors/base.py``. Names
    must be unique -- they are the keys under ``obs.sensors`` -- and every rate must divide
    ``tick_hz`` exactly. The default is :func:`flown_sensors`."""

    start_spacing_m: float = K.START_SPACING_M
    """Take-off grid spacing. See constants.START_SPACING_M -- too small deadlocks reactive
    policies against their own neighbours before anyone leaves the Start Area."""

    pose_source: str = "ground_truth"
    """Which :mod:`safmc_sim.pose` implementation to use. Recorded in the log header, because
    a run flown on a noisy pose source would otherwise be indistinguishable from a
    ground-truth one -- and the whole point of the seam is that the difference matters."""

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
        if round(self.duration_s * self.tick_hz) < 1:
            raise ConfigError(
                f"duration_s={self.duration_s} at tick_hz={self.tick_hz} is less than one "
                f"tick. The loop would never run and the result would still look complete."
            )
        if self.start_spacing_m <= 2.0 * K.DRONE_RADIUS_M:
            raise ConfigError(
                f"start_spacing_m {self.start_spacing_m} must exceed the drone diameter "
                f"{2 * K.DRONE_RADIUS_M}; drones would start overlapping"
            )
        if self.pose_source not in POSE_SOURCES:
            raise ConfigError(
                f"pose_source must be one of {sorted(POSE_SOURCES)}, got {self.pose_source!r}"
            )
        if self.collision_behaviour not in ("stop", "unobstructed"):
            raise ConfigError(
                f"collision_behaviour must be 'stop' or 'unobstructed', "
                f"got {self.collision_behaviour!r}"
            )
        object.__setattr__(self, "sensors", tuple(self.sensors))
        names: list[str] = []
        for cfg in self.sensors:
            if not isinstance(cfg, SensorConfig):
                raise ConfigError(
                    f"sensors must be SensorConfig instances, got {type(cfg).__name__}. "
                    f"Pass the config (ToFConfig()), not the sensor (ToFRing)."
                )
            if cfg.name in names:
                raise ConfigError(
                    f"two sensors are named {cfg.name!r}. Names are the keys under "
                    f"obs.sensors and must be unique -- give the second one a name."
                )
            names.append(cfg.name)
            decimation(cfg.rate_hz, self.tick_hz, f"sensor {cfg.name!r}")

    @property
    def dt(self) -> float:
        return 1.0 / self.tick_hz

    @property
    def max_ticks(self) -> int:
        return int(round(self.duration_s * self.tick_hz))


@dataclass
class AgentView:
    """Per-drone bookkeeping owned by the runner. Never handed to a policy."""

    agent_id: str
    robot: Any
    policy: Any
    sensors: list[Sensor] = field(default_factory=list)
    """This drone's sensors, in ``RunConfig.sensors`` order."""
    readings: dict[str, Any] = field(default_factory=dict)
    """Latest reading per sensor name. Held, not cleared, between samples."""
    sample_tick: dict[str, int] = field(default_factory=dict)
    """The tick whose post-motion world each reading reflects; -1 before the first tick."""
    lifecycle: str = Lifecycle.ACTIVE
    last_command: Command = field(default_factory=Velocity)
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
        self.pose_source: PoseSource = POSE_SOURCES[config.pose_source](
            np.random.default_rng(self._seeds[3])
        )
        self.world_scene = WorldScene.from_arena(self.arena)
        self._decimations = [
            decimation(cfg.rate_hz, config.tick_hz, f"sensor {cfg.name!r}")
            for cfg in config.sensors
        ]
        # What can kill you is what you can see: the same predicate that puts a landmark into
        # the ring's scene makes it collidable here.
        self._solid_landmarks = [lm for lm in self.arena.all_landmarks if lm.solid]
        self._check_point_landmarks_are_perceivable()

        self.arena_info = ArenaInfo(
            width_m=self.arena.width_m,
            depth_m=self.arena.depth_m,
            ceiling_m=self.arena.ceiling_m,
            start_area_depth_m=self.arena.start_area_depth_m,
            run_duration_s=config.duration_s,
        )

        self.env = None
        self._spent = False
        self.agents: list[AgentView] = []
        self.events: list[Event] = []
        self._mission_started_tick: int | None = None
        self._tick = 0

    def _check_point_landmarks_are_perceivable(self) -> None:
        """A point landmark that no sensor can report is a configuration mistake.

        A solid landmark is a body: the ring sees it whatever its kind. A point exists *only*
        to be reported by a sensor that knows its kind, so one nobody is configured to detect
        would sit in the arena doing nothing, and the policy author would spend an afternoon
        wondering why the camera never sees their nav tags.
        """
        perceived = {kind for cfg in self.config.sensors for kind in cfg.landmark_kinds}
        orphans = sorted(
            {lm.kind for lm in self.arena.all_landmarks if not lm.solid and lm.kind not in perceived}
        )
        if orphans:
            raise ConfigError(
                f"landmark kind(s) {orphans} are points that no configured sensor detects. "
                f"Add the kind to a sensor that reports by kind -- e.g. "
                f"MarkerCamConfig(kinds=TARGET_KINDS + ({orphans[0]!r},)) -- or give the "
                f"landmark a footprint and a height so the ring can see it as a body."
            )

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
            # One generator per (drone, sensor), spawned in config order, so adding a sensor
            # at the end of the list does not perturb the streams of the ones before it, and
            # adding a drone does not perturb the earlier drones (R-DET-3).
            per_sensor = sensor_seeds[i].spawn(max(len(self.config.sensors), 1))
            sensors = [
                sensor_cfg.build(np.random.default_rng(per_sensor[j]))
                for j, sensor_cfg in enumerate(self.config.sensors)
            ]
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
                    sensors=sensors,
                )
            )
        # Every sensor samples the initial world once, so the observation at tick 0 has a
        # reading for each of them rather than a hole.
        self._sense(-1)
        return self

    # -- the loop ------------------------------------------------------------------------------

    def run(self) -> RunResult:
        if self._spent:
            raise ConfigError(
                "this Runner has already been used. Build a new one -- reusing it appended a "
                "second set of agents holding references to the destroyed environment, and "
                "silently dropped half the fleet from the result."
            )
        if self.env is None:
            self.build()
        self._spent = True
        started = time.perf_counter()

        if self._recorder is not None:
            self._recorder.begin(
                self.config, self.arena, [a.agent_id for a in self.agents],
                sensors=self.agents[0].sensors if self.agents else (),
            )

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

        self._sense(tick)

        self._post_step(tick, sim_time + dt)

        if self._recorder is not None:
            self._recorder.tick(tick, sim_time + dt, self.agents, commands)

        self.blackboard.commit()

        return self._should_stop(tick)

    # -- sensing ---------------------------------------------------------------------------------

    def _sense(self, tick: int) -> None:
        """Sample every due sensor on every active drone from the post-motion world.

        ``tick`` is the tick ir-sim has just integrated, or -1 before the first. Sensing
        happens AFTER motion so every scan sees the same post-move world and no agent is
        measured against a staler snapshot than another; ir-sim guarantees the same ordering
        for its own sensors, and we do it explicitly because the sensors are ours. A sensor
        with decimation ``d`` samples when ``(tick + 1) % d == 0``, which makes it fresh in
        the observations at ticks 0, d, 2d, ... A terminal drone stops sensing: its last
        reading is held, and ``sample_tick`` says so.
        """
        self.world_scene.refresh_drones(self.env.robot_list, tick)
        for agent in self.agents:
            if agent.terminal:
                continue
            truth = TrueState.from_state(agent.agent_id, agent.robot.id, agent.state)
            for sensor, every in zip(agent.sensors, self._decimations):
                if (tick + 1) % every == 0:
                    agent.readings[sensor.name] = sensor.sample(truth, self.world_scene, tick)
                    agent.sample_tick[sensor.name] = tick

    # -- observation ---------------------------------------------------------------------------

    def _observe(self, agent: AgentView, tick: int, sim_time: float, peers) -> Observation:
        pose = self.pose_source.pose_of(agent.agent_id, agent.state, tick)
        # A reading sampled at the end of tick s is first current at tick s + 1.
        return Observation(
            agent_id=agent.agent_id,
            tick=tick,
            sim_time_s=sim_time,
            pose=pose,
            velocity_xy=self.pose_source.velocity_of(agent.agent_id, agent.state, tick),
            lifecycle=agent.lifecycle,
            sensors=MappingProxyType(dict(agent.readings)),
            stale_ticks=MappingProxyType(
                {name: tick - 1 - sampled for name, sampled in agent.sample_tick.items()}
            ),
            peers=peers,
            arena=self.arena_info,
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
                if self.config.collision_behaviour == "stop":
                    self._freeze(agent)
                    agent.lifecycle = Lifecycle.CRASHED
                    agent.crash_reason = "left the field"
                    self._emit(
                        tick, sim_time, "crashed", agent.agent_id,
                        {"reason": "left the field", "x": float(agent.xy[0]),
                         "y": float(agent.xy[1])},
                    )
                    continue
                # 'unobstructed' means no crashes, full stop -- otherwise whether the control
                # mode is actually a control depends on whether the policy under test happens
                # to stay inside the field, which is not a property you can assume of a policy
                # you are evaluating. Clamp to the boundary instead and keep flying.
                self._clamp_to_field(agent)

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
                        {"note": "first drone left the Start Area"},
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
        # Plain attribute access: ir-sim is pinned, `collision` is a documented property, and
        # a getattr default here would silently turn every collision off -- which is the single
        # most consequential confound in any comparison this simulator produces.
        if self.config.collision_behaviour == "stop" and bool(agent.robot.collision):
            other = agent.robot.collision_obj
            name = getattr(other[0], "name", "obstacle") if other else "obstacle"
            return f"collided with {name}"

        if self.config.collision_behaviour != "stop":
            # 'unobstructed' must disable ALL collision, not just ir-sim's. Leaving markers
            # lethal here made the control mode not a control: a policy could still lose
            # drones in the run that was supposed to isolate search strategy from crashes.
            return None

        # Solid landmarks -- mission markers, any placed body -- are not ir-sim obstacles,
        # because ir-sim's collision is strictly 2D and would make a 1.0 m marker impassable at
        # every altitude. Height-gated here instead, with the same predicate the ring uses.
        for landmark in self._solid_landmarks:
            if agent.z >= landmark.height_m:
                continue
            reach = K.DRONE_RADIUS_M + landmark.radius_m
            if float(np.hypot(agent.xy[0] - landmark.x, agent.xy[1] - landmark.y)) < reach:
                return f"struck landmark {landmark.id}"
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

    def _clamp_to_field(self, agent: AgentView) -> None:
        margin = K.DRONE_RADIUS_M
        agent.robot._state[0, 0] = float(
            np.clip(agent.xy[0], -margin, self.arena.width_m + margin)
        )
        agent.robot._state[1, 0] = float(
            np.clip(agent.xy[1], -margin, self.arena.depth_m + margin)
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
