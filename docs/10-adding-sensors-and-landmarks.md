# Adding a sensor, or something for it to sense

**Start here if you are adding a sensor** — a mocked camera, optical flow, a UWB tag — **or
placing something in the arena for one to find** — a start mark, a navigation tag, a radio
anchor. This is the extension guide; it sits beside [Writing a policy](05-policy-api.md) and
is meant to be the only page you need for either job.

Two primitives, one rule. A **sensor** is a function from the drone's true state and the
world to a frozen reading. A **landmark** is anything deliberately in the arena that is not
structure. And *truth enters a sensor; only readings leave it* — a policy sees what a device
could measure, never the world itself. [ADR-0005](adr/0005-sensor-and-landmark-primitives.md)
records why it is built this way; [06-sensors.md](06-sensors.md) is what the two flown
sensors, and the opt-in UWB tag, report.

## What a sensor is

Every sensor in this simulator is the same shape. It is a function from the drone's **true**
state and the world to a frozen reading, sampled at a rate that divides the tick rate. The
runner owns it, samples it after motion, and hands the latest reading to your policy as
`obs.sensors[name]`. That contract lives in `sensors/base.py`, both flown sensors are written
against it, and [adding one of your own](#adding-a-sensor) is one file.

The rule the contract enforces: **truth enters a sensor; only readings leave it.** A sensor is
handed the exact pose because that is what a physical sensor is bolted to. A policy is never
handed it. There are exactly two paths from ground truth to a policy — the pose source
(`obs.pose`, R-SEAM-1) and sensor readings (R-SENS-15) — and both are seams. See
[ADR-0005](adr/0005-sensor-and-landmark-primitives.md).

```python
obs.sensors            # {"tof": ToFScan, "markers": (MarkerDetection, ...), ...}
obs.stale_ticks        # {"tof": 0, "markers": 7, ...}  ticks since each last sampled
obs.tof                # shorthand for obs.sensors["tof"]
obs.markers            # shorthand for obs.sensors["markers"]
```

`RunConfig(sensors=...)` decides what a drone carries. The default is `flown_sensors()`: the
ring and the camera of [06-sensors.md](06-sensors.md), with the flown geometry and rates.

## Adding a sensor

Three parts, one file, then one tuple entry. `examples/03_custom_sensor.py` is the complete
runnable version; `sensors/base.py` has the contract's docstring; `sensors/uwb.py` is the
same shape with real physics and a fidelity entry behind every number, if you want to see
what a finished one looks like. Nothing in the runner, the policy API or the recorder needs
to know your sensor exists.

**1. The reading** — a frozen dataclass. It is the only thing a policy will ever see of your
sensor, so make it what the real device reports and nothing more. Wrap arrays in
`read_only()`: a frozen dataclass stops rebinding but not in-place writes.

```python
@dataclass(frozen=True)
class BeaconRanges:
    anchor_ids: tuple[str, ...]
    ranges_m: np.ndarray        # inf where out of range -- same convention as the ring
```

**2. The config** — a frozen `SensorConfig` subclass. Default `name` (its key under
`obs.sensors`), default `rate_hz` (`None` means every tick; anything else must divide the tick
rate or the run refuses to start), validated in `__post_init__` after
`super().__post_init__()`, and a `build`. Field names carry their units. If the sensor
reports landmarks by kind, say which kinds in `landmark_kinds` so the runner can check that
your anchors are perceivable.

```python
@dataclass(frozen=True)
class BeaconConfig(SensorConfig):
    name: str = "beacons"
    rate_hz: float | None = 10.0
    kind: str = "uwb_anchor"
    max_range_m: float = 15.0
    noise_std_m: float = 0.10

    def __post_init__(self):
        super().__post_init__()
        if self.max_range_m <= 0:
            raise ConfigError(f"max_range_m must be > 0, got {self.max_range_m}")

    @property
    def landmark_kinds(self):
        return (self.kind,)

    def build(self, rng):
        return BeaconRanger(self, rng)
```

**3. The sensor** — a `Sensor` subclass with `sample(truth, world, tick)`. `truth` is the
carrying drone's exact `TrueState`; `world` is a `WorldScene`, which offers
`sensing_scene(exclude_object_id=truth.object_id)` for anything a ray can hit and
`landmarks_of(kind)` for placed things. Noise comes from `self.rng`, never `numpy.random`.
Keep the geometry in a pure function and let `sample` be the adapter — the pure function is
what you unit-test. Return fixed-shape arrays from `record()` and the reading appears in the
log as `<name>.npz`.

```python
class BeaconRanger(Sensor):
    def sample(self, truth, world, tick):
        anchors = world.landmarks_of(self.config.kind)
        ranges = range_to_anchors(anchors, truth.xy, self.config.max_range_m)
        hit = np.isfinite(ranges)
        ranges = np.where(hit, ranges + self.rng.normal(0, self.config.noise_std_m, ranges.shape), ranges)
        return BeaconRanges(tuple(a.id for a in anchors), read_only(ranges))

    def record(self, reading):
        return {"ranges_m": reading.ranges_m}
```

**4. Wire it in.** Extend the flown suite rather than replacing it, put something in the
arena for it to sense, and read it by name.

```python
RunConfig(sensors=flown_sensors() + (BeaconConfig(),),
          arena_config=ArenaConfig(landmarks=ANCHORS))

def step(self, obs):
    beacons = obs.sensors["beacons"]
    ...
```

**Testing a policy that reads it** needs no simulator:
`make_observation(sensors={"beacons": BeaconRanges(...)})`.

Things the contract refuses, and when — always before tick 4 000:

- **At construction:** a name that is not an identifier or collides with `states`, `run` or
  `replay`; two sensors with one name, case-insensitively; a rate that does not divide the
  tick rate. `RunConfig` re-checks the names itself, so a config that forgot
  `super().__post_init__()` gets caught too.
- **At build:** a first reading a policy could write into — a list, a dict, an unfrozen
  dataclass, a writable array, an array with a writable array underneath it, an object
  array; a point landmark whose kind no sensor lists; a take-off position inside a body. Only
  the *first* reading is checked: a sensor that returns a frozen reading once and a list
  afterwards is its author's bug, and the runner does not pay to re-check every tick.
- **When the run starts recording:** a `record()` key named `ticks` or `sample_tick`, or
  shared with `record_static()`; a non-numeric value in either. Every sensor's row keys and
  shapes are fixed from its first reading, and a later row that differs — or a sensor that
  returned `None` first and rows later — stops the run with the sensor, key, drone and tick
  named. Everything is assembled before anything is written, so a refused run leaves no log.

Things it cannot check and you must: that the reading is what the real device would report —
the contract keeps the arena, the mission and other agents out of a sensor's reach, but a
sensor that returned every landmark's true position is within reach and outside the rule —
and that the physics is defensible. Say so in the module docstring and in
[FIDELITY.md](FIDELITY.md), the way the two flown sensors do.

Two things worth knowing about determinism: `record()` must be pure, because it runs only
when recording is on and R-OBS-4 requires a recorded run to match an unrecorded one; and each
sensor's generator is spawned in `RunConfig.sensors` order, so appending a sensor leaves the
earlier sensors' noise streams untouched while inserting one in front re-seeds everything
after it.

### Timing, exactly

Every sensor samples once before the first tick, then at the end of each tick `t` for which
`(t + 1) % decimation == 0` — after motion, and after the collision pass, so a drone that hit
something this tick is already terminal and does not record a scan from inside the wall. A
2 Hz sensor on the 20 Hz loop is therefore fresh in the observations at ticks 0, 10, 20, … and
`obs.stale_ticks[name]` counts up between. All drones sample the same post-move world, so no
drone is measured against a staler picture than another. A terminal drone stops sampling and
its last reading is held; the log's `sample_tick` column says which rows are held.

## Landmarks

Structure — walls and pillars — is what a drone must not hit. A **landmark** is everything
else deliberately in the arena: a mission marker with a tag on its face, a surveyed tag on a
wall, a start-point mark on the floor, a radio anchor in a corner. `world/landmark.py`.

```python
from safmc_sim.world.landmark import Landmark

Landmark("tag_12", "nav_tag", x=3.0, y=9.0)                      # a point: a tag on a wall
Landmark("start_00", "start_mark", 2.0, 2.0, radius_m=0.2)       # a flat mark with an extent
Landmark("anchor_0", "uwb_anchor", 1.0, 19.0, radius_m=0.05, height_m=0.8)   # a body
```

Two rules:

- **Solid means footprint *and* height.** A solid landmark occludes rays through the same
  height gate as a marker and can be struck below its height. Everything else is a point:
  the ring cannot see it and a drone cannot hit it. One predicate — `Landmark.solid` —
  decides both, so what can kill you is exactly what the ring can see. Mission targets are
  solid landmarks (0.30 m footprint, 1.0 m tall) with scoring semantics: `Target` subclasses
  `Landmark`.
- **A landmark reaches a policy only through a sensor's geometric query.** The ring sees
  solid ones as anonymous circles. The camera reports id, kind, range and bearing for the
  kinds it is configured for and nothing for the rest. A UWB anchor is invisible to a camera
  because a camera does not detect radio, not because someone remembered to hide it.

**Placing them.** Fixed, surveyed positions go in the scenario:

```python
RunConfig(arena_config=ArenaConfig(landmarks=(Landmark("tag_12", "nav_tag", 3.0, 9.0),)),
          sensors=(ToFConfig(), MarkerCamConfig(kinds=TARGET_KINDS + ("nav_tag",))))
```

A placement that depends on the generated layout — a tag on each doorway of the Unknown
Search Area — is generated first, placed with `dataclasses.replace`, and handed to the run:

```python
arena = generate_arena(seed)
placed = dataclasses.replace(arena, landmarks=tags_on_the_doorways(arena))
run(config, arena=placed)          # validated; header records arena_source: "supplied"
```

**Where the rules let you put one.** A navigation aid — a UWB anchor, a surveyed tag — may
stand anywhere in the Start Area, at most ten may stand in the Known Search Area, none may
stand in the Unknown Search Area, and each must fit within 1 m x 1 m on its own tripod
(booklet §3.3.1 r.14–17). Those rules are enforced in two places, because they are not all
checkable from the arena alone (R-WORLD-11).

The room rule needs no notion of an aid — *nothing* a team places may be in there — so
`validate_arena` refuses it on every run. The other two do: a `Landmark` may equally be
scenery or a prop, and the primitive has no field that says which. So
`validate_nav_aids(arena, kinds)` takes the kinds that count as aids from you, counts every
kind you name together, and names the offending landmarks and the rule. **The runner never
calls it**: twenty anchors is a legitimate experiment when you asked for it, and the check
exists so nobody quotes such a run without knowing. Call it where you build the scenario, as
`examples/04_uwb_ranging.py` does.

A point at a fixed coordinate is also a trap: the room moves with the seed, so **survey the
generated arena and then place** — `in_known_area` picks the positions, `dataclasses.replace`
places them. A small footprint (`radius_m=0.25`, a tripod base) makes each a flat mark the
generator keeps clear, still invisible to the ring and to collision.

A placed landmark with a footprint is fixed structure to the generator: walls and pillars
keep their published gaps from a body, nothing is built over a flat mark, targets are not
dropped on either, and the room is drawn clear of them. Validation rejects a landmark outside
the field, a duplicate id, non-finite geometry, a footprint overlapping structure or another
footprint, a target walled off by solid landmarks, and — on either path — a landmark wearing a
mission kind that is not a generated target, because the camera would report it as a victim
that can never score (R-WORLD-7). The run itself refuses a take-off position inside a body.

**Solid means lethal at every altitude below its height, including zero.** Under the default
`collision_behaviour="stop"`, a drone that lands inside a solid landmark crashes and is
recorded where it stopped, on top of the body; it does not settle and score. Under
`unobstructed` nothing collides -- that mode switches every crash off, this one included --
so the landing stands.

**The runner refuses a point landmark nobody can perceive.** A point exists only to be
reported by a sensor that knows its kind, so if no configured sensor lists it, that is a
configuration mistake and you get a `ConfigError` naming the kind when the run is constructed,
rather than an afternoon wondering why the camera never sees your tags. Solid landmarks are
exempt — the ring sees them whatever their kind — which also means a camera configured
without the mission kinds is accepted: a run that cannot read victims is a legitimate
experiment.

Every landmark is written to the log header with the arena, and the replay draws them: a
hollow diamond for a point or a flat mark, an orange-ringed disc for a body, id and kind on
hover.

## Sensors we expect to add

One sensor from the first version of this table now exists: the **UWB ranging tag**,
`sensors/uwb.py`, described in [06-sensors.md](06-sensors.md#the-uwb-ranging-tag--a-dw3000-not-flown)
and decided in [ADR-0006](adr/0006-uwb-ranging-sensor.md). It is the worked answer to the
last column — every number it needs is an assumption with an ID, and the airframe still does
not carry one. None of the rest exist. Each is a config and a `sample()` away, and each needs
a number nobody has measured. Listed so the shape of the work is visible and so the fidelity
questions are asked before the code is written.

| Sensor | Reading | Perceives | Needs before it is a model rather than a template |
|---|---|---|---|
| **Nav-tag camera** | already the marker detector: `Landmark(kind="nav_tag")` + `MarkerCamConfig(kinds=...)` | `nav_tag` landmarks | The same A-4/A-5 measurements; a pose-fit error model if the detection is to feed a `PoseSource` |
| **Vision cues** | a `(cue_id, kind, bearing, apparent size)` tuple from a nose-down camera | `vision_cue` / `start_mark` landmarks | The 45° pitch (F-6) actually modelled, so a floor mark enters view at a range set by altitude |
| **Optical flow** | body-frame velocity plus a quality flag; reads `truth.vx, truth.vy, truth.z` | nothing placed — the floor | Flow-quality vs altitude and texture; this is what PX4 already fuses for the real height, so it belongs with a `PoseSource`, not just a sensor |
| **Altimeter / downward ranger** | `z` with noise | the floor | Whether it sees a marker top as the floor; the real airframe gets height from PX4, not from a sensor the ESP reads |

A sensor whose reading feeds *localisation* rather than *search* — flow, nav tags used for
re-localisation, and now the UWB tag — is only half a feature until there is a `PoseSource`
that consumes it.
That seam is [ADR-0003](adr/0003-ground-truth-pose-and-perfect-comms.md), and it is the
highest-value single class in the project.

## In the log

Every sensor is listed in the header's `sensors` block with its config type, its rate and
whether it was recorded. A recorded sensor gets `<name>.npz` with `ticks`, a per-agent
`sample_tick`, one array per `record()` key stacked to `(ticks, agents, ...)`, and any
constants from `record_static()`. The ring is recorded whenever it is carried; the camera is
not, because a tuple of detections has no fixed shape. See
[07-logging-and-viz.md](07-logging-and-viz.md).
