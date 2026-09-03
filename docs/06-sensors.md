# Sensor models

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
ring and the camera below, with the flown geometry and rates.

## The ToF ring

Eight **ST VL53L5CX** multizone sensors. One raycast per drone computes all of them.

> **The part number matters.** The VL53L5CX has a 65° *diagonal* FoV, which ST specifies as a
> 45° × 45° square. Its pin- and driver-compatible near-twin the **VL53L7CX** is 90° diagonal
> (60° × 60° square) with 3.5 m of range instead of 4.0 m. Swapping one for the other changes
> the zone width from 5.625° to 7.5° and turns a gapless ring into one with 120° of overlap.
>
> `nus-safmc/gazebo-slam-prototype`'s repo description says "8 × VL53L7CX". That is stale — the
> flown firmware links ST's `vl53l5cx` driver and the mount geometry only tiles with the L5CX. Why one and
not eight is [ADR-0002](adr/0002-single-vectorised-tof-sensor.md).

### Geometry — reproduced from the flown firmware

| Property | Value | Source |
|---|---|---|
| Rangers | 8, mounted counter-clockwise, 45° apart, gapless 360° | `tof_task.h:26-36`, `tof_task.c:183` |
| Zones per ranger | 8 columns of 5.625° across the 45° FoV | `tof_task.c:258` |
| Mount radius | 0.040 m cardinal, 0.034 m diagonal (the PCB is rectangular) | `safmc-ros/.../robot.urdf` |
| Optical axes | all horizontal, no pitch or roll | `safmc-ros/.../robot.urdf` |
| Square FoV per sensor | 45° × 45° (65° diagonal) | datasheet |
| Zone width | 5.625° — **derived** as FoV ÷ 8, never written down twice | |
| Ring coverage | 8 × 45° = exactly 360°, no gap, no overlap | |
| Physical reach | 4.0 m | datasheet |
| Max rate at 8×8 | 15 Hz (the sensor only does 60 Hz at 4×4) | datasheet |
| Firmware gate | `[0.05, 3.0]` m | `tof_task.h:16-18` |

Ranger `i` sits at `i * 45°` counter-clockwise from the nose, and zone `j` at
`(j - 3.5) * 5.625°` from its ranger's axis. Both are checked against the firmware's own
arithmetic (`tof_task.c:183` and `:258`) by a test, and the mount positions against the URDF,
so if anyone changes the ring a test tells them rather than a flight.

`front_index` says which ranger points forward. It defaults to 0; the flown `sdkconfig` had it
at 1 for that airframe, so set it per drone if you care. Note that **no zone points exactly along the
ranger axis** — the eight zones straddle it. A drone facing a wall 2 m away reads
`2.0 / cos(2.8125°) = 2.00236 m` on its two innermost forward zones, not 2.0 m.

### What you get

`ToFScan` has exactly four members. That is the whole sensor API:

```python
scan = obs.tof
scan.ranges_m             # (8, 8) float, inf where there was no valid return
scan.zone_bearings_rad    # (8, 8) body-frame bearings, CCW from nose. Constant.
scan.ranger_bearings_rad  # (8,)   body-frame ranger axes, CCW from nose. Constant.
scan.min_range_m          # nearest valid return anywhere on the ring
```

**`inf` means no valid return. It never means "maximum range".** Those are different facts and
the real sensor distinguishes them too, via `target_status` 255. A policy that does
`np.minimum(ranges, 4.0)` is choosing to conflate them — which is often the right choice for a
potential-field controller, but should be a choice.

There is **no `status` array and no `collapsed_m`**. Both existed in an earlier version and were
withdrawn in `50e2643`; see [SPEC](SPEC.md) R-SENS-4 and R-SENS-5 for why. Short version: the
simulator only ever emitted status `5` or `255`, which `isfinite(range)` already tells you, and
the collapsed scan was `ranges_m` reordered.

#### If you need firmware index order

The firmware's `tof_scan_collapsed_t` holds the same 64 values indexed by **absolute clockwise
bearing** (`tof_task.c:243-293`). Ours are in `(ranger, zone)` order, **anticlockwise from the
nose**. They are a permutation of each other — same numbers, different slots — so converting is
pure reindexing:

```python
import numpy as np

def collapsed_m(scan, n_bins=64):
    """ranges_m -> firmware bin order: index 0 straight ahead, clockwise, 5.625 deg per bin."""
    cw  = (-scan.zone_bearings_rad) % (2 * np.pi)          # CCW body frame -> CW, as firmware
    idx = np.minimum((cw * n_bins / (2 * np.pi)).astype(int), n_bins - 1).reshape(-1)
    out = np.full(n_bins, np.inf)
    np.minimum.at(out, idx, scan.ranges_m.reshape(-1))     # min-pool, as firmware
    return out
```

With the flown geometry the 64 zones land in 64 distinct bins, so the min-pool pools nothing —
but it is kept because a non-default `ToFConfig` (more rangers, more zones) can genuinely
collide, and then pooling is what the firmware would do.

> **Do not assume `tof.npz` columns are firmware bin indices.** They are `(ranger, zone)` order.
> Always map through `zone_bearings_rad`, which is stored in the same file.

### Height gating — the whole of the 2.5D model

A ray at altitude `z` is occluded by a primitive only if `z_min <= z < height`. That is one
comparison, and it is the reason the ring is our own sensor rather than ir-sim's: shapely is
strictly 2D and `ObjectBase.z` is dead code.

| Object | Height | Blocks a ray at 0.5 m cruise? |
|---|---|---|
| Perimeter wall | 1.5 m | yes |
| Inner wall, Unknown-area wall | 2.0 m | yes |
| Pillar | 2.0 m | yes |
| **Mission marker** | **1.0 m** | **yes at 0.5 m, no at or above 1.0 m** |
| Any other solid landmark | its own `height_m` | below its height only |
| Point landmark (a tag, a mark, an anchor) | none | never — a ray cannot hit a point |
| Another drone | **every altitude** | always for the ring, matching the 2D collision model; **never for the camera** (F-21) |

Since the rules cap flight at 1.4 m and every wall is at least 1.5 m, structure is never
overflyable — 2D is exact for navigation. Altitude earns its keep for markers and other placed
bodies. Drones occlude each other at every altitude deliberately: collision is ir-sim's and
strictly 2D, so what can kill you is what you can see.

### Accuracy

Closed-form ray-circle and ray-segment intersection, verified against two independently
derived analytic references to **1e-9 m** over 800 random rays. No polygonisation error: for
comparison, ir-sim's `Lidar2D` carries about 1.5 mm of error from tessellating circles.

A ray whose origin is *inside* an obstacle returns the distance to where it exits. This is the
exact case where `Lidar2D` silently freezes its whole scan at the previous tick's values, and
there is a test that walks a sensor through an obstacle asserting no two consecutive scans are
identical.

### Noise

`ToFConfig(noise_std_m=...)` adds Gaussian range noise from the sensor's own generator, applied
before gating and **never** to a no-return — perturbing "nothing is there" is meaningless.

Off by default, because the target paper's arenas are noiseless and reproducing it is our
first regression test. Turn it on for robustness sweeps; producing exactly those sweeps is
something the paper lists as future work.

### What is not modelled

The ring is sampled **synchronously at the tick rate**. The real ring is round-robin, one
sensor per 8 ms, so each refreshes at ~15 Hz and the ring is **skewed by up to 64 ms** across
sensors — it is never a synchronous snapshot. That matters for scan-matching and for a drone
turning fast, and not much for reactive avoidance. Assumption A-8. `ToFConfig(rate_hz=...)`
will at least decimate it.

Also absent: `range_sigma_mm`, `reflectance`, `ambient_per_spad` (the firmware discards them
too); the firmware's conservative "substitute 0.40 m for an unreliable return"; and any
vertical structure within a zone, which a 2.5D sim cannot have.

## The marker detector

Stands in for the AprilTag pipeline. Models the **geometry** — range, field of view, occlusion
— and nothing about image formation.

```python
for m in obs.markers:
    m.marker_id, m.kind, m.range_m, m.bearing_rad
```

Range is measured to the marker's **near surface**, not its centre. Occlusion is tested to
just short of that surface, so a marker never occludes itself but another marker can occlude
it. Other drones do not occlude it (F-21). Default rate is 2 Hz, the measured AprilTag rate on
the real hardware — nominally 10 Hz, but the detector task runs at the lowest priority in the
system — so `obs.markers` is fresh every tenth tick and `obs.stale_ticks["markers"]` counts
1–9 in between.

**It detects landmarks by kind.** `MarkerCamConfig.kinds` defaults to the three mission-marker
kinds. The real detector also reads the surveyed navigation tags (ids 12–29 in
`laptop/setup.yaml`); in the simulator that is a [landmark](#landmarks) of kind `"nav_tag"`
plus one entry in `kinds`, and the detection comes back with `kind == "nav_tag"` so a policy
can tell it from a victim — exactly as the real one can from the id range.

Reporting `kind` is not cheating: the markers are team-supplied and the flown configuration
assigns tag ids by role (0-11 landing targets, 8-11 bonus). A drone that reads a tag id does
know what it is looking at.

> **Assumption A-4 — detection range, default 3.0 m — is the single most important unmeasured
> number in this simulator.** It sets how much area a drone sweeps per metre flown, which is
> the dominant term in every search-policy comparison. Nothing in any repository measures it;
> the only claim is an inherited comment about "about 1 meter (tested with tag on screen, not
> on paper)" describing *different* detector parameters than the ones actually set. Getting it
> wrong by a factor of two could change which policy wins.
>
> Measuring it is one afternoon: fly at a tag, log detections against distance. Do it before
> quoting any headline number.

A-5 (camera FOV, 1.0 rad) is derived from the QVGA intrinsics rather than measured, and the
real camera is pitched 45° nose-down while ours is modelled horizontal (F-6) — so it sees the
floor ahead rather than the horizon, and ground markers enter view differently.

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

## Adding a sensor

Three parts, one file, then one tuple entry. `examples/03_custom_sensor.py` is the complete
runnable version; `sensors/base.py` has the contract's docstring. Nothing in the runner, the
policy API or the recorder needs to know your sensor exists.

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

## Sensors we expect to add

None of these exist. Each is a config and a `sample()` away, and each needs a number nobody
has measured. Listed so the shape of the work is visible and so the fidelity questions are
asked before the code is written.

| Sensor | Reading | Perceives | Needs before it is a model rather than a template |
|---|---|---|---|
| **Nav-tag camera** | already the marker detector: `Landmark(kind="nav_tag")` + `MarkerCamConfig(kinds=...)` | `nav_tag` landmarks | The same A-4/A-5 measurements; a pose-fit error model if the detection is to feed a `PoseSource` |
| **Vision cues** | a `(cue_id, kind, bearing, apparent size)` tuple from a nose-down camera | `vision_cue` / `start_mark` landmarks | The 45° pitch (F-6) actually modelled, so a floor mark enters view at a range set by altitude |
| **Optical flow** | body-frame velocity plus a quality flag; reads `truth.vx, truth.vy, truth.z` | nothing placed — the floor | Flow-quality vs altitude and texture; this is what PX4 already fuses for the real height, so it belongs with a `PoseSource`, not just a sensor |
| **UWB ranging** | range-only to each anchor, `inf` out of range | `uwb_anchor` landmarks | Ranging noise, NLOS bias behind walls, anchor placement the venue would allow. `examples/03_custom_sensor.py` is the template |
| **Altimeter / downward ranger** | `z` with noise | the floor | Whether it sees a marker top as the floor; the real airframe gets height from PX4, not from a sensor the ESP reads |

A sensor whose reading feeds *localisation* rather than *search* — flow, UWB, nav tags used
for re-localisation — is only half a feature until there is a `PoseSource` that consumes it.
That seam is [ADR-0003](adr/0003-ground-truth-pose-and-perfect-comms.md), and it is the
highest-value single class in the project.

## In the log

Every sensor is listed in the header's `sensors` block with its config type, its rate and
whether it was recorded. A recorded sensor gets `<name>.npz` with `ticks`, a per-agent
`sample_tick`, one array per `record()` key stacked to `(ticks, agents, ...)`, and any
constants from `record_static()`. The ring is recorded whenever it is carried; the camera is
not, because a tuple of detections has no fixed shape. See
[07-logging-and-viz.md](07-logging-and-viz.md).
