# Checkpoints

Each checkpoint is a commit. For each: what was built, what was verified, and what was still open
at that point. Written so an auditor can start at any checkpoint and check the claims against the
tree at that commit.

Verification vocabulary:
- **TESTED** — an automated test exists that fails if the claim is false.
- **MEASURED** — a number produced by running something, quoted with its conditions.
- **ASSERTED** — believed correct, no test yet. Every ASSERTED item is a debt.

---

## C0 — research digest, spec, ADRs

Commit `fd67425`.

**Built.** `docs/SPEC.md` (the audit contract), `docs/01-04` and `09` (competition, hardware,
ir-sim, architecture, related work), `docs/FIDELITY.md`, four ADRs, package scaffold with
`ir-sim==2.10.2` pinned.

**Verified.** Nothing executable yet. Every factual claim carries a `file:line` or URL, and
everything recon could not confirm is marked UNVERIFIED in place.

**Open at this point.** All code.

---

## C1 + C2 — core sim layer and arena generator

**Built.**

- `frames.py` — ARENA frame, angle wrapping, the NED bijection to the flight stack.
- `constants.py` — every published, hardware-derived and assumed value, each tagged with
  provenance. No bare numeric literals elsewhere.
- `kinematics.py` — `Quad25D`, a 6-state 2.5D quadrotor registered with ir-sim via
  `@register_kinematics`. First-order velocity lag, vector speed cap, yaw integrated and
  wrapped in-handler, altitude rate-limited and clamped to the 1.4 m ceiling.
- `sensors/raycast.py` — vectorised closed-form ray casting with per-primitive vertical bands.
- `sensors/scene.py` — separates the sensing scene from the line-of-sight scene, and pulls
  live drone bodies from ir-sim at sensor-step time with a per-tick cache.
- `sensors/tof_ring.py` — the 8-ranger ring as **one** ir-sim sensor, emitting per-zone
  `(range, status)` and the firmware's 64-bin collapsed scan.
- `sensors/marker_cam.py` — geometric marker detection with range, FOV and occlusion.
- `world/arena.py` — seeded arena generation, ir-sim YAML emission, and self-validation.

**Verified — TESTED.** 472 tests, all passing.

- Raycaster agrees with two *independently derived* analytic references (law-of-cosines for
  circles, 2x2 linear solve for segments) to **1e-9 m** over 800 random rays. R-SENS-7.
- The stale-scan defect that afflicts ir-sim's `Lidar2D` is tested for directly by walking a
  sensor through an obstacle and asserting no two consecutive scans are identical. R-SENS-8.
- Ring geometry, gating window and the 64-bin collapse are asserted against the firmware
  constants; all 64 bins are covered exactly once. R-SENS-2, R-SENS-3, R-SENS-5.
- 2.5D height gating tested at the sensor level and end-to-end. R-SENS-6.
- NED round-trip exact to 1e-9 across the wrap discontinuity. R-FRAME-5.
- Arena validation rejects a target inside an obstacle and a walled-off target, and
  generation raises rather than degrading when over-constrained. R-WORLD-4.

**Verified — MEASURED.**

100/100 arenas generate and validate in 3.0 s; generation is bit-deterministic per seed.

Headless throughput, macOS arm64, Python 3.12.10, full SAFMC arena, 200 steps after warm-up:

| Config | N=4 | N=10 | N=25 |
|---|---|---|---|
| `ToFRing`, 8 rangers x 8 zones = **64 rays/drone** | 924 steps/s | 397 | **151** |
| ir-sim `Lidar2D`, a single **8-beam** fan | 831 steps/s | 273 | 100 |

The ring casts **eight times as many rays** and is still faster at every fleet size — 1.5x at
N=25. And this understates the gap: the honest `Lidar2D` equivalent of a ring is *eight*
one-beam instances per drone, which pays the fixed GEOS overhead eight times over. This is the
evidence behind [ADR-0002](adr/0002-single-vectorised-tof-sensor.md).

At N=25 a full 600 s competition run is 12 000 ticks, about **79 s wall-clock**. A 50-seed
sweep is roughly an hour single-threaded and is embarrassingly parallel across processes.

**Found while building.** The published constraints nearly determine the arena: a 10 m room
plus two 2 m gaps exactly fills the 14 m Known Search Area, so the room's north-south position
is forced and the surrounding free space is a ~2 m corridor ring. Empirically only about one
free-standing inner wall fits. Either assumption A-6 is wrong or the 2 m gap is not an
all-pairs constraint — both are exposed as config. This has strategic consequences and is
written up in `world/arena.py`'s module docstring.

**Open at this point.** Policy API, runner, blackboard, mission scoring, recorder, visualiser,
reference policies.

---

## C3 + C4 + C5 — policy API, runner, mission, recorder, visualiser, reference policies

**Built.**

- `api.py` — `Observation`, six `Command` types matching the firmware's action set exactly,
  the `Policy` base class, and a name-keyed registry that overwrites with a warning.
- `blackboard.py` / `pose.py` — the two deferred-work seams, each with a working v0.1
  implementation and a worked example of the replacement (`NoisyPose`).
- `mission.py` — target servicing, line of sight, the fire-suppression coupling, and the
  relay evaluated as a breadth-first search over the "within 1 m and mutually visible"
  graph.
- `runner.py` — the tick loop, lifecycle state machine, two-wave rule, collision handling.
- `recorder.py` — versioned structured log (`run.jsonl` + `states.npz` + `tof.npz`) and
  `score_from_log`, which re-scores offline from the log alone.
- `tools/viz.py` — self-contained HTML replay: arena, tracks, live ToF rays, scoring radii,
  agent table, event timeline, scrubbing.
- `cli.py` — `safmc-run run | sweep | replay | policies`. Sweeps run one process per run,
  because ir-sim's RNG is process-global.
- Five reference policies: `hold`, `random_walk`, `wall_follow`, `frontier` (log-odds
  occupancy map + frontier selection + VFH avoidance), and `sdlw` (a faithful port of
  arXiv:2607.25195, with its `uhlw` baseline).

**Verified — TESTED.** 530 tests, all passing.

- Nothing reachable from an `Observation` is world state: a test walks every public attribute
  four levels deep and asserts no `ArenaSpec`, `Mission`, `Runner` or ir-sim object appears.
  R-POL-4.
- A raising policy aborts the run with agent id, tick and the original exception chained.
  R-POL-9.
- All ten agents observe an identical blackboard snapshot within a tick. R-POL-8.
- Two identical runs produce logs that differ **only** in the `meta` block; `states.npz` is
  byte-identical. R-DET-1.
- Adding a twelfth drone leaves the first ten agents' RNG streams unchanged. R-DET-3.
- A sensor rate that does not divide the tick rate raises rather than rounding. R-TIME-3.
- Offline re-scoring equals the online score exactly across three seeds. R-MISS-8.
- Recording on versus off produces identical simulation results. R-OBS-4.
- Scoring: the 1 m radius, the line-of-sight requirement, markers *not* blocking line of
  sight, one-award-per-target, the 2.5 m fire coupling in both directions, relay formation
  and its three failure modes, and the 240-point theoretical maximum. R-MISS-1..4.
- The two-wave rule admits exactly two waves and records a refusal for the third. R-MISS-6.

**Verified — MEASURED.** A 12-run comparative sweep, 12 drones, 120 s, seeds 0-3:

| policy | mean score | min | max |
|---|---|---|---|
| `frontier` (map-based) | 22.5 | 15 | 40 |
| `sdlw` (mapless, IROS 2026) | 18.8 | 10 | 35 |
| `random_walk` | 8.8 | 0 | 20 |

The ordering is what the project set out to be able to measure, and the null baseline
(`hold`) scores zero as it must.

**A result that needs stating carefully.** In that sweep `frontier` crashed 7-10 of 12 drones
while `sdlw` crashed 0-4. Its higher score is achieved *despite* losing most of its fleet, and
under `collision_behaviour="stop"` a crashed drone stops contributing for the rest of the
episode. This is exactly the confound recon found in the target paper, so the comparison is
not yet trustworthy: it needs the `unobstructed` mode as a control and coverage normalised by
live-agent-seconds. `frontier`'s avoidance clearance also wants tuning before anyone quotes
these numbers. Recorded as an open item rather than a headline.

**Open at this point.** Metrics module; the collision-mode control study; the adversarial
audit.

---

## C5b — metrics, docs, and a fleet-deadlock fix

**Built.** `metrics.py`; `docs/05-policy-api.md`, `06-sensors.md`, `07-logging-and-viz.md`,
`08-porting-to-ros.md`; top-level `README.md`; two runnable examples.

**Found while writing the examples — a real simulator artefact, not a strategy failure.**

The take-off grid was spaced at 4 drone radii (0.72 m) and placed 0.66 m from the southern
boundary. Both are inside a typical reactive avoidance threshold: a drone's neighbours sat
0.36 m away and its rear-facing ranger read 0.62 m *before it had moved*. Any policy using an
omnidirectional threshold therefore turned on the spot for the entire run and never left the
Start Area — scoring zero, and looking exactly like a bad search strategy.

Fixed with two named constants carrying the reasoning: `START_SPACING_M = 1.25` (leaving
~0.89 m of clear air between neighbours) and `START_WALL_MARGIN_M = 1.5`. A 25-drone fleet
still fits comfortably: 15 per row, two rows, 2.5 m of the 6 m Start Area. Three regression
tests now cover it, including one that asserts every reference policy actually gets a drone
out of the Start Area within 45 s.

**Verified — MEASURED.** The fix changes the ranking, which is the point of catching it:

| policy | mean score before | mean score after |
|---|---|---|
| `wall_follow` | (deadlocked) | **42.5** |
| `random_walk` | 8.8 | 27.5 |
| `frontier` | 22.5 | 22.5 |
| `sdlw` | 18.8 | 20.0 |

`wall_follow` now leads by a wide margin, which is what the arena's own geometry predicted:
once the 10 x 10 m Unknown Search Area is placed, the Known Search Area is close to a 2 m
corridor ring, and in a corridor, wall following is very hard to beat. That is a genuine
strategic finding for the team, and it fell out of taking the published constraints literally.

**Verified — TESTED.** 533 tests.

**Open at this point.** The adversarial audit against `docs/SPEC.md`.

---

## C6 — adversarial spec audit, and the fixes it forced

Full report: [AUDIT-v0.1.md](AUDIT-v0.1.md).

**Done.** Seven independent auditors, one per area of `SPEC.md`, each required to locate the
code, locate the regression test, and *attempt falsification* with its own scripts. Every
reported violation was then handed to a separate skeptic instructed to refute it. 57 agents.

**Result.** 71 requirements: 39 SATISFIED, 18 UNTESTED, 14 VIOLATED. Plus 36 extra bugs. Of 50
claims sent to skeptics, **24 survived**.

**The one that mattered most.** A `LANDED` drone was not immobile. The runner zeroed its
command, but velocity lives in the state and decays through a lag, so a drone landing at speed
slid up to 116 mm — 12% of the scoring radius. An auditor produced a case where a drone touched
down 1.021 m from a bonus victim (outside the radius, not serviced) and slid to 0.906 m,
**scoring 15 points it had not earned**, and making offline re-scoring disagree with the online
result. `mission.py` documented the false invariant as fact.

**The test that should have caught it was vacuous.** A full drone-resurrection mutation passed
all 533 tests. The replacement was *itself* vacuous on the first attempt — drones landing from
a hover slide ~0 mm — and now lands them at cruise speed and asserts bit-identical positions
afterwards. Mutation-verified.

Nine other confirmed defects fixed, including: a policy could permanently re-aim its own ToF
ring through an in-place numpy write; blackboard publications were stored by reference, giving
index-order-dependent reads; the `unobstructed` control mode still killed drones on markers and
let them fly 55 m out of a 20 m field; `NaN` in a command surfaced as a `GEOSException` with no
agent or tick; and the log contained bare `Infinity`/`NaN`, which is invalid JSON and rendered
the replay page blank.

**Two corrections to the spec itself.** `R-DRONE-9` specified an `ARMED` lifecycle state that
nothing implemented and no policy could distinguish — removed. `FIDELITY.md` F-3 claimed the
firmware's 0.40 m unreliable-return substitution was modelled; it is not modelled at all.

**And a correction to my own earlier finding.** I had concluded the Unknown Search Area's
north-south position was *forced*. It is not: that derivation demanded a 2 m gap on the room's
south side, which faces the Start Area's virtual boundary line rather than a wall. The room has
~1.9 m of freedom. Related real defect: gaps were measured centre-line to centre-line, so the
room sat 1.95 m from the perimeter in **every** seed while validation passed — because the
validator never compared the room against the perimeter at all. Both fixed.

**Verified — TESTED.** 595 tests. `tests/test_audit_regressions.py` carries one named test per
confirmed defect.

**Verified — MEASURED, and this is the headline.** 12 drones, 180 s, seeds 0-4, both collision
modes:

| policy | `unobstructed` (control) | `stop` (survivability) |
|---|---|---|
| `frontier` (map-based) | **67.0** | 13.0 |
| `wall_follow` | 50.0 | 15.0 |
| `sdlw` (mapless, IROS 2026) | 19.0 | 17.0 |
| `random_walk` | — | 21.0 |

**The two modes rank the policies in opposite orders, and both rankings are true.** Isolating
search strategy from crashes, the map-based policy is decisively better — 67 against 19 for the
published mapless baseline. Include crashes and it is the *worst*, because it flies close to
obstacles and dies. This is precisely the confound the target-paper recon warned about, now
measured on our own policies, and it is only visible because `unobstructed` was fixed into a
real control.

Do not read this as "frontier wins". Read it as: **the map-based policy has the better search
strategy and unusable obstacle-avoidance tuning**, and the honest next step is to fix its
clearance and re-run, not to pick a winner. Five seeds is also too few to publish.

**Open.** Five UNTESTED requirements with no cheap guard, listed in the audit report — chiefly
that nothing prevents a policy calling `numpy.random` directly (`R-POL-7`), and that
`R-DRONE-7`'s "cruise speed" is bound to a speed *cap*, which is a modelling decision rather
than a test gap.

---

## C7 — strip the platform back to primitives

**Why.** Review found policy baked into the framework. The old repos were reference for what
the drone and the world *are* — ring geometry, zone layout, MAVLink action set, arena, frames —
not for how the drone flies. I over-read them and ported the firmware's *navigation behaviour*
into the platform, where every policy inherited it invisibly.

**Five places it had leaked, all removed.**

1. **`SearchPolicy`** — presented as scaffolding but actually a strategy: take off, land if a
   marker is within 0.6 m, else claim it over the blackboard and approach, else defer to the
   subclass. Every "policy" in the repo supplied only a wandering step; the mission decisions
   were mine, in a base class.
2. **`vfh_steer`** — a direct port of the firmware's `vfh.c`. Literally the old codebase's
   obstacle-avoidance policy.
3. **`PositionWorld`** — the runner computed bearings, set speed, pointed yaw along travel and
   prevented overshoot. A path follower living in the simulator.
4. **`VelocityWorld` / `Hold`** — proportional controllers on yaw and altitude, plus a
   remembered target altitude.
5. **`Takeoff` / `Land` as phases** — the runner flew the climb and descent itself.

**What the action space is now.** `Velocity(vx, vy, vz, yaw_rate)` in the ARENA frame, and
`Land()`. That is all. World frame because it is what `mavlink_set_velocity_ned` actually
takes and what the kinematics already integrates, so nothing is converted behind the caller's
back; body-frame thinking gets `toolbox.body_to_world`, four readable lines.

**Lifecycle** went from six states to three: `ACTIVE`, `LANDED`, `CRASHED`. There are no flight
phases because climbing is a velocity and when to stop climbing is a policy's decision.

**The two-wave take-off rule** is no longer enforced. The runner emits `departed` events and
`mission.takeoff_waves()` computes compliance from them. Enforcing it mid-flight made the
platform a referee, hid the violation from the policy that caused it, and welded the runner to
one year's rulebook.

**`policies/` holds exactly one policy**: the arXiv:2607.25195 port, rewritten on primitives.
A strategy written by whoever wrote the simulator is not a baseline — it is the simulator's own
assumptions wearing a policy's clothes. SDLW is externally authored and citable. It never lands
(the paper's task is pure coverage), so it scores zero on the mission by design; it is a
*search* baseline and a regression test.

**`toolbox.py`** holds the building blocks worth keeping — `body_to_world`, `ring_quadrants`,
`climb`/`descend`, `OccupancyMap` — explicitly outside the framework. Two new requirements make
the boundary auditable: **R-POL-10** (no guidance, control or strategy in the simulator) and
**R-POL-11** (the framework may not import `policies` or `toolbox`), both with structural tests.

**Verified — TESTED.** 595 tests. Two new guards: one walks every framework module's imports
and fails if either opt-in package appears; one greps `runner.py` for the controller names that
were removed.

**Superseded.** The v0.1 comparative numbers (`frontier` 67.0 vs `sdlw` 19.0 in the control,
reversed with crashes) were produced by policies that no longer exist. They stay recorded above
as history and as the origin of the live-agent-seconds finding, which is about *metrics* and
survives independently. They are **not** a current claim about anything.

**Open.** Rebuild a comparison once the team has written its own policies — that is now the
intended shape.

---

## C8 — sensors and landmarks become primitives

**Why.** Two sensors, two unrelated interfaces, five files to edit for a third, and an "Adding
a sensor" section in `docs/06-sensors.md` that described an ir-sim factory patch removed at C3.
The team wants mocked cameras, optical flow and UWB, and things in the arena for them to
perceive — start marks, surveyed AprilTags, vision cues. [ADR-0005](adr/0005-sensor-and-landmark-primitives.md).

**Built.**

- `sensors/base.py` — the contract: `SensorConfig` (frozen, named, rated, `build(rng)`),
  `Sensor` (`sample(truth, world, tick)`, optional `record`), `TrueState`, `read_only`,
  `decimation`. One timing rule for every sensor: sampled before tick 0, then after motion when
  `(t + 1) % d == 0`.
- `world/landmark.py` — `Landmark(id, kind, x, y, radius_m, height_m)`; solid ⇔ footprint and
  height; solid ones occlude and collide, points do neither. `Target` is now a `Landmark`.
- `sensors/scene.py` — `WorldScene` carries the landmark list and is the only world a sensor sees.
- The ring and the camera ported onto the contract. The camera detects landmarks **by kind**
  (`MarkerCamConfig.kinds`), so a nav tag is a landmark plus one config entry.
- `runner.py` — `RunConfig.sensors` (default `flown_sensors()`), one `_sense` path for every
  sensor, per-(drone, sensor) generators, solid-landmark collision, and a refusal of any point
  landmark no configured sensor can report.
- `api.py` — `Observation.sensors` and `stale_ticks`; `tof` and `markers` become shorthands
  that raise by name when the run does not carry that sensor.
- `recorder.py` — `<name>.npz` per recorded sensor with `sample_tick`; header `sensors` block;
  landmarks in the arena block. `load_run()["sensors"][name]`.
- `world/arena.py` — `ArenaConfig.landmarks`, `ArenaSpec.landmarks` / `all_landmarks` /
  `landmark_scene()`; solid placed landmarks are structure to the generator; validation.
- `examples/03_custom_sensor.py` — a range-only beacon sensor, four anchors, a policy that
  reads it by name. A template, not a model (F-22).
- Spec: R-SENS-12..16, R-WORLD-7..8; R-POL-3 and R-OBS-2 amended. A new guide,
  `docs/10-adding-sensors-and-landmarks.md`, in the reading order beside "Writing a policy";
  `docs/06` keeps the flown models. Docs 04–08, ARCHITECTURE, FIDELITY (F-21, F-22), README.

**Verified — TESTED.** 239 tests. `tests/test_sensor_primitive.py` builds a sensor the way the
docs say to and checks: names and rates refused at construction; a custom reading reaches the
policy by name with the documented staleness; per-drone instances and generators; adding a
sensor does not perturb earlier sensors' streams (R-DET-3); byte-identical runs with a noisy
custom sensor (R-DET-1); the log by name with `sample_tick`; reserved keys refused; a spy
sensor is handed `TrueState` and `WorldScene` and nothing with `arena`/`mission`/`agents`; the
camera reports only its configured kinds; two cameras under two names. `tests/test_landmarks.py`
covers solid vs point, the height gate, config and `replace` placement, generation around a
placed body, validation, and a fence of 0.6 m posts that kills a fleet at 0.4 m and not at
0.9 m. The R-POL-4 walk now descends into mappings and bans `Landmark`, `Target`, `WorldScene`,
`TrueState` and `Sensor`.

**Verified — MEASURED.** Full suite 45 s (38 s at C7 with 200 tests). The example runs 60 s of
10 drones with three sensors and writes `beacons.npz` shaped `(1200, 10, 4)`.

Against the pre-change tree (`3a24398`, run in a separate process from a `git archive` of its
`src/`): the default arena is **identical for every seed 0–29** — walls, pillars, targets and
room; and a default `sdlw` run (seed 3, 10 drones, 30 s) is **byte-identical** in
`states.npz`, in every recorded ToF row including row 0, and in every event. The tick-0
change below is to the *observation* a policy is handed before the first step, which the log
never held; the reference policy climbs before it reads the ring, so nothing it did changed.

**Found while building.** The R-POL-4 walk did not descend into a `MappingProxyType` (it is
not a `dict`), so the new `sensors` mapping would have been skipped. Other drones never
occluded the camera (now F-21). A solid landmark placed by config had to become structure for
the generator — on seed 7 a random inner wall landed on one, caught by a test.

**Behaviour that changed.** The camera samples at the end of tick t−1 instead of the top of
tick t: same world state, its readings are identical. The **tick-0 ring observation** is
different: the old runner sampled it lazily before the drone bodies had been added to the
scene, so at tick 0 no drone saw its neighbours; now every drone does. The recorded rows are
unaffected (row 0 was always the post-step scan) and later observations are identical; the
byte-identical comparison above is the measurement. A marker
strike is reported as `struck landmark <id>`, not `struck marker`.
`Recorder(record_tof=, tof_every=)` is `Recorder(record_sensors=, sensor_every=)`;
`load_run()["tof"]` is `load_run()["sensors"]["tof"]`, which also gains `sample_tick`.

**Audited.** Two independent adversarial audits against R-SENS-12..16 and R-WORLD-7..8, each
told to falsify rather than confirm. Findings that changed code, all now under test:

- The R-POL-4 walk **could not fail**: its try/except wrapped the recursive call and
  swallowed every assertion below the root, in this version and in every earlier one. An
  auditor smuggled a `Landmark` and a `TrueState` through it. The walk now guards only the
  attribute access, bans by `isinstance`, and the test proves it can fail.
- `ToFScan.ranges_m` was **writable**, and the scan is held between samples: one write in a
  policy put `-7` into 18 of 20 recorded rows. Read-only now, as the bearings were; the
  runner refuses any sensor whose first reading a policy could write into.
- Reachability validation **ignored solid landmarks**: an arena with every doorway plugged
  by posts validated. Solid landmarks are now in the occupancy grid, placed footprints are
  fixed structure to the generator, and a take-off position inside a body is refused.
- A drone could **land on top of a 1.0 m marker** from 1.2 m and score 15. Landing inside a
  solid landmark is a crash.
- A `Landmark` with kind `"victim"` was accepted as a **decoy** the camera would report and
  the mission would never score. Refused on the config path and the replace path.
- The documented `dataclasses.replace` placement **could not be run**. `run(config,
  arena=placed)` now exists; the header records `arena_source`.
- A config that skipped `super().__post_init__()` could name itself `states` and
  **overwrite `states.npz`**; `TOF` and `tof` collided on a case-insensitive filesystem.
  Names are re-validated by `RunConfig`, case-insensitively.
- A `record()` whose keys or shapes changed between ticks wrote a **misaligned file** that
  loaded without complaint, and a stacking failure left `run.jsonl` beside a missing sensor
  file. The row schema is fixed at run start and enforced every tick; the log is written all
  or nothing.
- A `Landmark` subclass with extra fields, or one with NaN geometry, **broke offline
  re-scoring**. The header records base fields only; geometry must be finite.
- No test guarded "sensed after motion" or "a terminal drone stops sampling" — both mutants
  survived the suite. Both have tests.

Doc claims the audits falsified and that were corrected: "the ring is always recorded"; "the
walk never inspected any reading" (it walked `obs.tof`; what it never did was fail); the
reserved-key check happens at run start, not construction; `docs/04`'s diagram placed the
sensors inside ir-sim; `docs/08` still described the six-command API removed at C7;
FIDELITY F-9, F-13 and F-14 described the removed descent and controllers; `docs/07`'s log
sizes were ten times stale.

**Audited again.** A second pass on the fixes, told to re-create each hole and its
neighbours. What it found, all now fixed and under test:

- `read_only()` returned a *view*: the writable original was one `.base` away, and
  `obs.tof.zone_bearings_rad.base[:] += 0.5` re-aimed the ring for the run. It copies now,
  `RayScene` owns copies, and the build-time check follows the `.base` chain and refuses
  object arrays.
- A `Landmark` subclass that skipped `super().__post_init__()` reached the log with an empty
  kind or a NaN and broke offline re-scoring. Every invariant is re-checked on the resolved
  arena, and a `Target` subclass with an unknown kind is refused too.
- Published blackboard values were mutable by *readers*: an agent that appended to a peer's
  list changed what the agents stepped after it saw in the same tick (pre-existing).
  Published containers are frozen at commit.
- Pre-C8 logs lost their `tof.npz` in `load_run`. They fall back to it.
- The walk test's depth cap ran before its bans, and it treated `frozenset` and object
  arrays as leaves. Bans first, deeper cap, both containers walked; eight smuggling cases.
- A sensor returning `None` at its first sample and rows later was silently unrecorded; a
  `record_static()` returning a string saved and then failed to load; a `record()` that
  raised lost its sensor and tick. All refused by name.
- A supplied arena left the config's `arena_config` in the header describing an arena that
  was never flown; the arena's own config replaces it. A drone that crashed while landing
  was recorded hovering at cruise; it is recorded on top of the body.
- A sloppy config's `rate_hz="4"` and `sensors=None` raised bare `TypeError`s; `ConfigError`.

Design choices the pass questioned, kept and now stated: under `unobstructed` a landing
inside a body stands, because that mode switches every crash off; only a sensor's *first*
reading is checked for immutability; `sensed_coverage` on old logs shifts ~0.1 % because
markers now occupy the grid. **274 tests.**

**Open.** No sensor beyond the flown two is a model of anything; the example is a template.
A sensor that feeds localisation — flow, UWB, nav tags — is half a feature until a
`PoseSource` consumes it (ADR-0003). The CLI has no `--sensors`; custom sensors are configured
in Python, as `examples/03_custom_sensor.py` shows. The contract bounds a sensor's *reach*
(R-SENS-15) but cannot bound its *use*: a sensor that returned every landmark's true position
would pass every check. That is R-SENS-11's review obligation, not a property of the code.

---

## C9 — a UWB ranging tag on the sensor contract

Commits `4b273e7` (spec) and the build commit that follows it. ADR-0006, R-SENS-17,
R-WORLD-9, A-9..A-13, F-23..F-27.

**Built.**

- `sensors/uwb.py` — `UWBConfig` / `UWBRanges` / `UWBTag`. Range-only to every landmark of
  the configured kind, in arena order: anchor ids, surveyed positions at one mount height,
  one range per anchor with `inf` for nothing heard. Three-dimensional range; obstruction by
  walls and pillars only, at the drone's altitude, through the same segment test the
  mission uses; line-of-sight Gaussian noise, through-wall bias plus wider noise plus a
  dropout probability, and a positive-outlier component that is off by default. Four draws
  per anchor per sweep whatever the geometry. Recorded as `uwb.npz` with the anchor
  positions as a static array. Not in `flown_sensors()`: the airframe carries no UWB.
- `sensors/scene.py` — `WorldScene.structural_scene`, walls and pillars only, for a sensor
  whose signal passes through a marker and a teammate. Not the scoring scene, which stays
  with the mission.
- `world/arena.py` — `validate_nav_aids(arena, kinds)`: the booklet's placement rules
  (§3.3.1 r.14–17) as an opt-in check the runner never applies.
- `constants.py` — `NAV_AID_*` from the booklet; `UWB_*` as A-9..A-13 with their sources;
  `UWB_RATE_HZ` and `UWB_ANCHOR_HEIGHT_M` as deployment defaults.
- `examples/04_uwb_ranging.py` — six anchors across both rows of the Start Area and four on
  tripods in the Known Search Area, checked against the rules; a policy that reads the tag
  by name; and a grade of the sensor from the log alone.
- Docs: ADR-0006; SPEC R-SENS-17, R-WORLD-9, an R-POL-3 note, §12; FIDELITY A-9..A-13 and
  F-23..F-27; a UWB section in `docs/06`; the nav-aid rules and the table update in
  `docs/10`; `docs/07`, `docs/04`, ARCHITECTURE, README.

**Verified — TESTED.** 312 tests. `tests/test_uwb.py` (33 items) checks: the defaults are the
registered constants and the tag is not flown; eleven impossible configs refused at
construction; the range is three-dimensional to the anchor at mount height; anchors come
back in arena order and only the tag's kind; an empty sweep; beyond reach is `inf`; a wall
biases while a marker and a teammate do not, and the ring disagrees on purpose;
obstruction follows the drone's altitude; on a generated arena the tag's obstruction equals
the mission's line-of-sight scene at 40 random poses; the noise model as a pure function —
zero-mean Gaussian at A-9 in line of sight, biased and wider and dropped at A-11/A-12
behind a wall, `inf` beyond reach, never negative, outliers off by default and positive
when on; the noise stream is independent of the geometry; identical runs identical;
appending the tag leaves the ring's stream untouched (R-DET-3); the reading reaches the
policy by name, fresh every other tick, immutable, with no `Landmark` in it; an arena with
anchors cannot be flown without the tag; the log holds `ranges_m` shaped `(ticks, agents,
anchors)` and the anchor positions in the header's landmark order, and grading it from the
log alone puts every line-of-sight error inside six sigma with a zero mean; recording does
not change the run; `record_static()` before the first sample is refused; the example's
layout passes the rules and runs. `tests/test_landmarks.py` (+5): any number of aids in the
Start Area and ten in the Known Search Area pass; an eleventh is refused and kinds count
together; an aid inside the room, or on its wall, is refused and one just outside is not;
an aid wider than a metre is refused; the runner does not referee. The R-POL-4 walk now
carries a UWB reading and still bans the `Landmark`; the units-suffix test covers
`UWBConfig`.

**Verified — MEASURED.** `examples/04_uwb_ranging.py`, seed 0, 10 drones, 60 s, ten anchors:
6 000 fresh sweeps; 70.0 % of tag–anchor paths in line of sight, 93.2 % within 20 m; heard
on 100 % of in-reach line-of-sight paths and 90.1 % of in-reach paths behind a wall (A-12
is 0.10); line-of-sight error 0.000 ± 0.050 m (A-9 is 0.05); behind a wall +0.157 ± 0.398 m
(A-11 is +0.15 and 0.40). `uwb.npz` is 204 kB for that run. Full suite 96 s.

**Found while building.** `WorldScene` exposed no walls-and-pillars scene, so the first cut
would have let a 1.0 m marker obstruct radio at cruise altitude and not above it. A UWB
tag's `record_static()` cannot know its anchors until the first sample; the runner's order
(sample at build, then begin recording) guarantees it, and the tag refuses to be recorded
outside that order rather than writing an empty array. The log refuses string arrays, so
anchor ids are recovered from the header's landmark order instead of stored. A point
anchor at fixed coordinates in the Known Search Area can end up inside a generated wall;
the example gives its Known-Area anchors a 0.25 m base so the generator draws around them.

**Behaviour that changed.** None for existing runs: the tag is opt-in, `flown_sensors()` is
unchanged, and a default run's log is unaffected. `WorldScene` gains a read-only property.

**Open.** Every number is an assumption: A-10 (reach) first, then A-11/A-12 against the
venue's actual walls. Obstruction is boolean — one wall's numbers behind any number of
walls (F-24) — and calibration is assumed (F-26). No `PoseSource` consumes the tag yet;
that is the next piece of work (ADR-0003), and until it exists a policy that wants a
position from these ranges trilaterates for itself. The CLI has no `--sensors`, so the tag
is configured in Python, as the example shows. The replay does not draw `uwb.npz`.

