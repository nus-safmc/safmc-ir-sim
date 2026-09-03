"""R-SENS-12..16: the sensor contract, checked with a sensor written the way the docs say to.

The sensor below is the pattern from ``sensors/base.py`` reduced to its skeleton: a reading,
a config, a sensor. If it stops working, so does every sensor anyone writes.
"""

from __future__ import annotations

import tempfile
from dataclasses import dataclass

import numpy as np
import pytest

from safmc_sim import policies  # noqa: F401 -- registers sdlw
from safmc_sim.api import Land, Policy, Velocity, register_policy
from safmc_sim.errors import ConfigError
from safmc_sim.recorder import Recorder, load_run
from safmc_sim.runner import RunConfig, Runner, flown_sensors, run
from safmc_sim.sensors.base import Sensor, SensorConfig, TrueState, read_only
from safmc_sim.sensors.marker_cam import MarkerCamConfig
from safmc_sim.sensors.tof_ring import ToFConfig
from safmc_sim.world.arena import ArenaConfig, TARGET_KINDS
from safmc_sim.world.landmark import Landmark


# -- the smallest sensor that exercises every part of the contract ------------------------------


@dataclass(frozen=True)
class Altitude:
    z_m: float
    tick: int


@dataclass(frozen=True)
class AltimeterConfig(SensorConfig):
    name: str = "altimeter"
    rate_hz: float | None = 4.0
    noise_std_m: float = 0.0

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.noise_std_m < 0:
            raise ConfigError("noise_std_m must be >= 0")

    def build(self, rng):
        return Altimeter(self, rng)


class Altimeter(Sensor):
    config: AltimeterConfig

    def sample(self, truth: TrueState, world, tick: int) -> Altitude:
        z = truth.z + (self.rng.normal(0.0, self.config.noise_std_m) if self.config.noise_std_m else 0.0)
        return Altitude(z_m=float(z), tick=tick)

    def record(self, reading: Altitude):
        return {"z_m": np.array([reading.z_m])}


SHORT = dict(n_drones=10, duration_s=3.0, record=False)


def with_altimeter(**kw):
    return AltimeterConfig(**kw)


# -- configuration -------------------------------------------------------------------------------


def test_default_sensors_are_the_flown_hardware():
    cfg = RunConfig()
    assert [type(s) for s in cfg.sensors] == [ToFConfig, MarkerCamConfig]
    assert [s.name for s in cfg.sensors] == ["tof", "markers"]
    assert cfg.sensors == flown_sensors()


def test_a_sensor_is_configured_not_instantiated():
    """Pass the config; the runner builds one sensor per drone from it."""
    with pytest.raises(ConfigError, match="SensorConfig"):
        RunConfig(sensors=(ToFConfig(), Altimeter(AltimeterConfig(), np.random.default_rng(0))))


def test_duplicate_sensor_names_are_rejected():
    with pytest.raises(ConfigError, match="named 'tof'"):
        RunConfig(sensors=(ToFConfig(), ToFConfig()))
    # The same sensor twice is fine under two names.
    RunConfig(sensors=(ToFConfig(), ToFConfig(name="tof_upper")))


@pytest.mark.parametrize("name", ["", "8ball", "has-dash", "has space", "states", "run"])
def test_a_sensor_name_must_be_an_identifier_and_not_reserved(name):
    with pytest.raises(ConfigError):
        AltimeterConfig(name=name)


def test_a_rate_that_does_not_divide_the_tick_rate_is_refused():
    with pytest.raises(ConfigError, match="does not divide"):
        RunConfig(sensors=(with_altimeter(rate_hz=3.0),))


def test_config_validation_runs_at_construction():
    with pytest.raises(ConfigError, match="noise_std_m"):
        AltimeterConfig(noise_std_m=-1.0)


# -- the runner drives it through the same path as the flown sensors -----------------------------


def test_a_custom_sensor_reaches_the_policy_under_its_name():
    seen = []

    @register_policy("_reads_altimeter")
    class ReadsAltimeter(Policy):
        def step(self, obs):
            reading = obs.sensors["altimeter"]
            assert isinstance(reading, Altitude)
            seen.append((obs.tick, obs.stale_ticks["altimeter"], reading))
            return Velocity(vz=0.4)

    run(RunConfig(seed=0, policy="_reads_altimeter",
                  sensors=flown_sensors() + (with_altimeter(),), **SHORT))
    assert seen
    # At 4 Hz on a 20 Hz loop the reading is fresh at ticks 0, 5, 10, ... and ages in between.
    for tick, stale, reading in seen:
        assert stale == tick % 5
        assert reading.tick == tick - 1 - stale
    # And it is sampling the true state: the drones are climbing, so altitude rises.
    rising = [r.z_m for t, _, r in seen if t % 5 == 0]
    assert rising[-1] > rising[0]


def test_readings_are_sampled_after_motion_from_the_same_world_for_every_drone():
    """Every drone's fresh reading reflects the post-move state of the tick just integrated."""
    runner = Runner(RunConfig(seed=0, policy="sdlw", sensors=(with_altimeter(rate_hz=None),), **SHORT))
    runner.build()
    try:
        for agent in runner.agents:
            assert agent.sample_tick == {"altimeter": -1}
            assert agent.readings["altimeter"].z_m == agent.z
    finally:
        runner._teardown()


def test_a_run_with_only_the_ring_has_no_markers_and_says_so():
    @register_policy("_no_camera")
    class NoCamera(Policy):
        def step(self, obs):
            assert set(obs.sensors) == {"tof"}
            with pytest.raises(AttributeError, match="no sensor named 'markers'"):
                obs.markers
            assert not hasattr(obs, "markers") and hasattr(obs, "tof")
            return Velocity()

    run(RunConfig(seed=0, policy="_no_camera", sensors=(ToFConfig(),), **SHORT))


def test_flown_sensors_can_be_removed_entirely():
    """A blind run is a legitimate experiment, not an error."""
    @register_policy("_blind")
    class Blind(Policy):
        def step(self, obs):
            assert dict(obs.sensors) == {}
            return Velocity()

    run(RunConfig(seed=0, policy="_blind", sensors=(), **SHORT))


def test_each_drone_gets_its_own_sensor_instance_and_generator():
    runner = Runner(RunConfig(seed=0, policy="sdlw",
                              sensors=(with_altimeter(noise_std_m=0.1),), **SHORT))
    runner.build()
    try:
        instances = [a.sensors[0] for a in runner.agents]
        assert len({id(s) for s in instances}) == len(instances)
        assert len({id(s.rng) for s in instances}) == len(instances)
        # Different drones, different noise streams.
        draws = {s.rng.random() for s in instances}
        assert len(draws) == len(instances)
    finally:
        runner._teardown()


def test_adding_a_sensor_does_not_perturb_the_streams_of_the_sensors_before_it():
    """R-DET-3 applied to sensors: generators are spawned in config order."""
    def first_draws(sensors):
        runner = Runner(RunConfig(seed=5, policy="sdlw", sensors=sensors, **SHORT)).build()
        try:
            return [a.sensors[0].rng.random() for a in runner.agents]
        finally:
            runner._teardown()

    alone = first_draws((with_altimeter(),))
    with_more = first_draws((with_altimeter(), ToFConfig(), MarkerCamConfig()))
    assert alone == with_more


def test_identical_runs_with_a_noisy_custom_sensor_are_identical():
    """R-DET-1 survives a sensor that draws noise."""
    seen: dict[int, list] = {}

    @register_policy("_records_altitude")
    class Records(Policy):
        def step(self, obs):
            seen.setdefault(id(seen), []).append(obs.sensors["altimeter"].z_m)
            return Velocity(vz=0.3)

    cfg = RunConfig(seed=3, policy="_records_altitude",
                    sensors=(with_altimeter(noise_std_m=0.05),), **SHORT)
    run(cfg); a = seen.pop(id(seen))
    run(cfg); b = seen.pop(id(seen))
    assert a == b


# -- the log -----------------------------------------------------------------------------------------


def test_a_recordable_sensor_appears_in_the_log_by_name():
    with tempfile.TemporaryDirectory() as tmp:
        result = run(RunConfig(seed=1, policy="sdlw", n_drones=10, duration_s=3.0,
                               sensors=flown_sensors() + (with_altimeter(),)),
                     recorder=Recorder(tmp))
        log = load_run(tmp)
    names = [s["name"] for s in log["header"]["sensors"]]
    assert names == ["tof", "markers", "altimeter"]
    assert log["header"]["sensors"][2]["type"].endswith("AltimeterConfig")
    alt = log["sensors"]["altimeter"]
    assert alt["z_m"].shape == (result.ticks, 10, 1)
    assert alt["ticks"].shape == (result.ticks,)
    # A 4 Hz sensor is held between samples, and the log says which ticks were fresh.
    held = alt["sample_tick"][:, 0]
    assert set(np.diff(held)) <= {0, 5}
    # The camera has no fixed-shape reading, so it is listed and not recorded.
    assert "markers" not in log["sensors"]
    assert log["header"]["sensors"][1]["recorded"] is False


def test_record_keys_cannot_shadow_the_log_s_own_columns():
    class Bad(Altimeter):
        def record(self, reading):
            return {"ticks": np.array([0.0])}

    @dataclass(frozen=True)
    class BadConfig(AltimeterConfig):
        name: str = "bad"

        def build(self, rng):
            return Bad(self, rng)

    from safmc_sim.errors import LogFormatError

    with tempfile.TemporaryDirectory() as tmp:
        with pytest.raises(LogFormatError, match="reserved key"):
            run(RunConfig(seed=1, policy="sdlw", n_drones=10, duration_s=1.0,
                          sensors=(BadConfig(),)), recorder=Recorder(tmp))


# -- the boundary --------------------------------------------------------------------------------------


def test_a_sensor_is_handed_the_world_scene_and_nothing_else():
    """R-SENS-15: no arena, no mission, no agent reaches a sensor."""
    from safmc_sim.sensors.scene import WorldScene

    captured = []

    class Spy(Altimeter):
        def sample(self, truth, world, tick):
            captured.append((truth, world))
            return super().sample(truth, world, tick)

    @dataclass(frozen=True)
    class SpyConfig(AltimeterConfig):
        name: str = "spy"

        def build(self, rng):
            return Spy(self, rng)

    run(RunConfig(seed=0, policy="sdlw", sensors=(ToFConfig(), SpyConfig(rate_hz=None)), **SHORT))
    assert captured
    for truth, world in captured:
        assert isinstance(truth, TrueState)
        assert isinstance(world, WorldScene)
        for banned in ("arena", "mission", "agents", "env", "policy"):
            assert not hasattr(world, banned)


def test_read_only_blocks_in_place_writes():
    arr = read_only(np.zeros(3))
    with pytest.raises(ValueError):
        arr[0] = 1.0


def test_true_state_reads_the_six_row_state_in_the_documented_order():
    state = np.array([[1.0], [2.0], [0.3], [0.5], [0.1], [-0.2]])
    truth = TrueState.from_state("d0", 7, state)
    assert (truth.x, truth.y, truth.theta, truth.z, truth.vx, truth.vy) == (1.0, 2.0, 0.3, 0.5, 0.1, -0.2)
    assert truth.object_id == 7 and truth.agent_id == "d0"


# -- the marker camera is a landmark detector -------------------------------------------------------


def test_the_camera_reports_only_the_kinds_it_is_configured_for():
    # A tag on the floor of the Start Area, a couple of metres north of the take-off row,
    # straight ahead of the fleet, which starts facing north. In view from tick 0.
    tag = Landmark("tag_12", "nav_tag", 1.75, 4.0)
    seen: dict[str, set[str]] = {"with": set(), "without": set()}

    def reader(name):
        @register_policy(f"_tag_reader_{name}")
        class TagReader(Policy):
            def step(self, obs):
                seen[name].update(m.kind for m in obs.markers)
                return Velocity()
        return f"_tag_reader_{name}"

    # Visible to a camera configured for its kind...
    run(RunConfig(seed=0, policy=reader("with"), arena_config=ArenaConfig(landmarks=(tag,)),
                  sensors=(ToFConfig(), MarkerCamConfig(kinds=TARGET_KINDS + ("nav_tag",))),
                  **SHORT))
    assert "nav_tag" in seen["with"]

    # ...invisible to one that is not, when something else can still perceive it...
    @dataclass(frozen=True)
    class TagRadio(AltimeterConfig):
        name: str = "tag_radio"

        @property
        def landmark_kinds(self):
            return ("nav_tag",)

    run(RunConfig(seed=0, policy=reader("without"), arena_config=ArenaConfig(landmarks=(tag,)),
                  sensors=(ToFConfig(), MarkerCamConfig(), TagRadio()), **SHORT))
    assert "nav_tag" not in seen["without"]

    # ...and a point landmark nobody is configured to detect is refused, not silently ignored.
    with pytest.raises(ConfigError, match="nav_tag"):
        Runner(RunConfig(seed=0, policy="_tag_reader", n_drones=10, duration_s=1.0, record=False,
                         arena_config=ArenaConfig(landmarks=(tag,))))


def test_the_camera_can_be_re_aimed_by_name_and_carried_twice():
    """Two cameras, two names, two readings -- the contract does not assume one of anything."""
    fwd = MarkerCamConfig(name="cam_front")
    aft = MarkerCamConfig(name="cam_rear", bearing_offset_rad=np.pi)
    names = set()

    @register_policy("_two_cams")
    class TwoCams(Policy):
        def step(self, obs):
            names.update(obs.sensors)
            return Velocity()

    run(RunConfig(seed=0, policy="_two_cams", sensors=(ToFConfig(), fwd, aft), **SHORT))
    assert names == {"tof", "cam_front", "cam_rear"}


# -- what the audit got through, and the contract now refuses -----------------------------------
#
# An adversarial audit of the first cut found that a policy could write into its held ToF
# scan, that a sensor could return a list, that a config which forgot super().__post_init__()
# could call itself "states", and that a sensor whose rows changed shape produced a
# misaligned file. Each has a test now, so the next audit can spend its time elsewhere.


@dataclass
class UnfrozenReading:
    z_m: float


class Probe(Altimeter):
    """Returns whatever shape of reading the config asks for, right or wrong."""

    def sample(self, truth, world, tick):
        variant = self.config.variant
        if variant == "list":
            return [truth.z]
        if variant == "writable":
            return np.array([truth.z])
        if variant == "unfrozen":
            return UnfrozenReading(truth.z)
        if variant == "nested":
            return (Altitude(truth.z, tick), {"z": truth.z})
        return Altitude(truth.z, tick)


@dataclass(frozen=True)
class ProbeConfig(AltimeterConfig):
    name: str = "probe"
    variant: str = "ok"

    def build(self, rng):
        return Probe(self, rng)


@pytest.mark.parametrize("variant, match", [
    ("list", "mutable"),
    ("writable", "writable array"),
    ("unfrozen", "frozen dataclass"),
    ("nested", r"reading\[1\]"),
])
def test_a_reading_a_policy_could_write_into_is_refused_at_build(variant, match):
    runner = Runner(RunConfig(seed=0, policy="sdlw",
                              sensors=(ToFConfig(), ProbeConfig(variant=variant)), **SHORT))
    try:
        with pytest.raises(ConfigError, match=match):
            runner.build()
    finally:
        runner._teardown()


def test_the_ring_s_scan_is_read_only_through_the_observation():
    """A policy that writes into ranges_m would edit its own next observation and the log."""

    @register_policy("_writes_scan")
    class Writes(Policy):
        def step(self, obs):
            with pytest.raises(ValueError, match="read-only"):
                obs.tof.ranges_m[:] = -7.0
            return Velocity()

    run(RunConfig(seed=0, policy="_writes_scan", **SHORT))


def test_sensing_happens_after_motion():
    """The fresh reading at tick t is the state at the end of tick t-1: not earlier, not later."""
    seen = []

    @register_policy("_when_sampled")
    class When(Policy):
        def step(self, obs):
            seen.append((obs.tick, obs.pose.z, obs.sensors["altimeter"].z_m))
            return Velocity(vz=0.5)

    run(RunConfig(seed=0, policy="_when_sampled", sensors=(with_altimeter(rate_hz=None),),
                  n_drones=10, duration_s=2.0, record=False))
    by_tick: dict[int, list] = {}
    for tick, z, read in seen:
        by_tick.setdefault(tick, []).append((z, read))
    # The reading equals the pose the policy sees in the same observation...
    for rows in by_tick.values():
        for z, read in rows:
            assert read == z
    # ...and the fleet is climbing, so that pose is strictly below the next tick's. A reading
    # taken before this tick's motion would match the previous tick's pose instead.
    for tick in range(1, 30):
        for (z0, _), (z1, _) in zip(by_tick[tick], by_tick[tick + 1]):
            assert z0 < z1


def test_a_terminal_drone_stops_sampling_and_holds():
    @register_policy("_lands_at_20")
    class Lands(Policy):
        def step(self, obs):
            return Velocity(vz=0.4) if obs.tick < 20 else Land()

    with tempfile.TemporaryDirectory() as tmp:
        run(RunConfig(seed=0, policy="_lands_at_20", n_drones=10, duration_s=3.0),
            recorder=Recorder(tmp))
        tof = load_run(tmp)["sensors"]["tof"]
    sample = tof["sample_tick"]
    assert sample.shape[0] == 21, "every drone landed at tick 20, so the run stops there"
    assert (sample[:20] == np.arange(20)[:, None]).all()      # fresh every tick while flying
    assert (sample[20] == 19).all()                            # landed: the last scan is held


def test_recording_a_custom_sensor_does_not_change_the_run():
    """R-OBS-4 with a noisy custom sensor in the suite."""
    captured: dict[str, list] = {}

    @register_policy("_captures_altitude")
    class Captures(Policy):
        def step(self, obs):
            captured.setdefault(self.config["run"], []).append(obs.sensors["altimeter"].z_m)
            return Velocity(vz=0.3)

    def config(label, record):
        return RunConfig(seed=2, policy="_captures_altitude", n_drones=10, duration_s=3.0,
                         record=record, policy_config={"run": label},
                         sensors=flown_sensors() + (with_altimeter(noise_std_m=0.05),))

    with tempfile.TemporaryDirectory() as tmp:
        run(config("loud", True), recorder=Recorder(tmp))
    run(config("quiet", False))
    assert captured["loud"] == captured["quiet"]


def test_a_config_that_skipped_super_post_init_is_still_validated():
    @dataclass(frozen=True)
    class Sloppy(AltimeterConfig):
        def __post_init__(self):
            pass                                      # forgot super().__post_init__()

    with pytest.raises(ConfigError, match="reserved"):
        RunConfig(sensors=(Sloppy(name="states"),))
    with pytest.raises(ConfigError, match="identifier"):
        RunConfig(sensors=(Sloppy(name="../escaped"),))
    with pytest.raises(ConfigError, match="case-insensitively"):
        RunConfig(sensors=(ToFConfig(), ToFConfig(name="TOF")))


def test_a_sensor_whose_rows_change_shape_is_refused_and_writes_nothing():
    from pathlib import Path

    from safmc_sim.errors import LogFormatError

    class Shifty(Altimeter):
        def record(self, reading):
            return {"z_m": np.zeros(1 if reading.tick < 5 else 2)}

    @dataclass(frozen=True)
    class ShiftyConfig(AltimeterConfig):
        name: str = "shifty"
        rate_hz: float | None = None

        def build(self, rng):
            return Shifty(self, rng)

    with tempfile.TemporaryDirectory() as tmp:
        with pytest.raises(LogFormatError, match="shape"):
            run(RunConfig(seed=0, policy="sdlw", n_drones=10, duration_s=1.0,
                          sensors=(ToFConfig(), ShiftyConfig())), recorder=Recorder(tmp))
        assert not list(Path(tmp).iterdir()), "a refused run must not leave a partial log"


def test_record_and_record_static_cannot_share_a_key():
    from safmc_sim.errors import LogFormatError

    class Overlap(Altimeter):
        def record_static(self):
            return {"z_m": np.zeros(1)}

    @dataclass(frozen=True)
    class OverlapConfig(AltimeterConfig):
        name: str = "overlap"

        def build(self, rng):
            return Overlap(self, rng)

    with tempfile.TemporaryDirectory() as tmp:
        with pytest.raises(LogFormatError, match="both record"):
            run(RunConfig(seed=0, policy="sdlw", n_drones=10, duration_s=1.0,
                          sensors=(OverlapConfig(),)), recorder=Recorder(tmp))


def test_the_header_types_every_sensor_config():
    with tempfile.TemporaryDirectory() as tmp:
        run(RunConfig(seed=1, policy="sdlw", n_drones=10, duration_s=1.0,
                      sensors=flown_sensors() + (with_altimeter(),)), recorder=Recorder(tmp))
        header = load_run(tmp)["header"]
    assert header["config"]["sensors"][2]["type"].endswith("AltimeterConfig")
    assert header["config"]["sensors"][2]["name"] == "altimeter"
    assert header["arena_source"] == "generated"
