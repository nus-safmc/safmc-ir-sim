# ADR-0005: Sensors and landmarks are primitives, not special cases

**Status:** Accepted · **Date:** 2026-09-03

## Context

v0.1 shipped two sensors with two unrelated interfaces. The ring was
`ToFRing(config, world_scene, rng, object_id).step(x, y, theta, z)`; the camera was
`MarkerCam(config).detect(pose_xy, theta, z, targets, occlusion_scene)`. The runner carried a
field per sensor on `AgentView`, a config field and a rate field per sensor on `RunConfig`, a
field and a staleness counter per sensor on `Observation`, and a recording path that knew the
ring's array shape. Adding a third sensor meant editing five files and inventing a third
interface. Meanwhile `docs/06-sensors.md` still told a reader to copy `tof_ring.install()` and
a patch of ir-sim's `SensorFactory` — neither of which had existed since R-SENS-1 was amended.

The team wants to add mocked sensors — a camera for vision cues, optical flow, UWB ranging —
and things in the arena for them to perceive: start-point marks, surveyed AprilTags, vision
cues. The only perceivable non-structural object was the mission `Target`, and the camera was
hard-wired to `arena.targets`. There was no way to put a nav tag in the world.

## Decision

Two primitives, one rule.

**`Sensor` / `SensorConfig`** (`sensors/base.py`). A sensor is a function from the carrying
drone's *true* state and the world to a frozen reading, sampled at a rate that divides the
tick rate. A config is a frozen dataclass with a `name`, a `rate_hz`, its own validated
fields, and `build(rng)`; a sensor implements `sample(truth, world, tick)` and may implement
`record(reading)` to appear in the log. `RunConfig.sensors` is a tuple of configs, defaulting
to the flown pair. The runner builds one instance per drone per config, samples every due
sensor after motion, and hands the latest readings to the policy as `obs.sensors[name]` with
`obs.stale_ticks[name]`. `obs.tof` and `obs.markers` survive as shorthands for the two flown
sensors. The runner does not know what any sensor is.

**`Landmark`** (`world/landmark.py`). Anything deliberately in the arena that is not structure:
id, kind, position, footprint, height. A landmark with both a footprint and a height is
*solid*: it occludes rays through the same height gate as everything else and it can be
struck. A landmark without both is a point — a tag on a wall, a mark on the floor, a radio
anchor — and neither blocks nor kills. `Target` is a `Landmark` with scoring semantics. The
marker camera detects landmarks by *kind*, configured on the camera, so a nav tag is a
landmark with kind `"nav_tag"` plus one entry in `MarkerCamConfig.kinds`. Landmarks are
placed by `ArenaConfig(landmarks=...)` for fixed, surveyed positions, or by
`dataclasses.replace(arena, landmarks=...)` when placement depends on the generated layout.

**The rule.** *Truth enters a sensor; only readings leave it.* A sensor is handed
`TrueState` and a `WorldScene` — geometry a ray can hit, plus the landmark list — and never
the arena, the mission, or an agent. A policy is never handed either. So there are exactly
two paths from ground truth to a policy, both seams: the pose source (R-SEAM-1) and the
sensor contract (R-SENS-15). A landmark reaches a policy only through a sensor's geometric
query (R-SENS-11, restated from the world's side).

## Rationale

1. **The shared thing is the sampling contract, not the physics.** A ring, a camera, a flow
   sensor and a radio have nothing in common except that each is a pure function of true
   state and world, sampled at a rate. Abstracting anything more specific — a `Camera` base
   class with image-formation hooks — would be guessing at models nobody has written.
2. **Configs, not instances, in `RunConfig`.** A config is frozen, serialisable, and logged
   verbatim, so a run's sensor suite is reproducible from its header. An instance holds a
   generator and per-drone state and belongs to one drone.
3. **Readings by name.** Every sensor added as a named field on `Observation` would edit the
   policy contract. A mapping does not, and the two flown sensors keep typed shorthands so
   no existing policy changes.
4. **Kind on the sensor, not `detectable_by` on the landmark.** A camera knows what it can
   read; an anchor does not know who can hear it. The direction matters for the failure
   mode: a point landmark whose kind no configured sensor declares is refused at build
   time, because it could never do anything and the author would spend an afternoon
   wondering why the camera never sees their tags.
5. **One predicate for "blocks" and "kills".** `Landmark.solid` puts a body into the ring's
   scene and into the collision check. What can kill you is what you can see — the rule
   that already governs drone-drone occlusion — now holds for placed objects by
   construction.

## Consequences

- **Adding a sensor is one file and one tuple entry.** No edits to the runner, the API, the
  recorder or the replay. `examples/03_custom_sensor.py` is the template and a test runs it.
- **The stale documentation is fixed at the root**: `docs/06-sensors.md` now describes the
  contract, and `tests/test_sensor_primitive.py` builds a sensor the way the docs say to.
- **`Observation.sensors` is `Mapping[str, Any]`.** The type of a reading is its author's.
  The R-POL-4 walk test now descends into mappings — it previously did not, which means it
  had never actually inspected `peers` either — and additionally bans `Landmark`, `Target`,
  `WorldScene`, `TrueState` and `Sensor` from anything reachable from an observation.
- **Timing is now one rule for every sensor**: sampled once before tick 0 and then after
  motion whenever `(tick + 1) % decimation == 0`, so a decimated sensor is fresh at ticks
  0, d, 2d. The camera used to sample at the top of the tick from the same world state;
  nothing a policy observes changed.
- **Cost:** `obs.tof` / `obs.markers` are properties, not fields. `make_observation` accepts
  `sensors={...}` and is otherwise unchanged. `Recorder(record_tof=, tof_every=)` became
  `Recorder(record_sensors=, sensor_every=)`. `load_run()["tof"]` became
  `load_run()["sensors"]["tof"]`, and every sensor file gains a `sample_tick` array so a
  held reading is distinguishable from a fresh one.
- **Cost:** a sensor whose reading has no fixed shape (a tuple of detections) cannot use the
  row recorder and is listed in the header as not recorded. The camera is such a sensor.
- **Cost:** a solid placed landmark counts as structure for generation, so walls and pillars
  keep their published gaps from it and an over-cluttered config can fail to generate. That
  is the same fail-fast the generator already applies to everything else.

## Rejected alternatives

| Option | Why not |
|---|---|
| Register sensors with ir-sim's `SensorFactory` | Already rejected in the R-SENS-1 amendment: it needs a monkeypatch, a dead plotting path, and a walk to the parent for altitude the runner already has |
| A `Camera` base class with `TagCamera`, `FlowCamera` subclasses | Premature. The nav-tag case turned out to be one config field on the existing camera; the flow case has no shared image model to inherit |
| Named `Observation` fields per sensor | Every sensor edits the policy contract and the walk test; the mapping costs one string lookup |
| `Landmark.detectable_by` | Wrong direction; and it cannot express "the ring sees any body regardless of kind" |
| Landmarks as ir-sim obstacles | Same reason markers never were: shapely is strictly 2D and would make a 1.0 m marker impassable at every altitude |
