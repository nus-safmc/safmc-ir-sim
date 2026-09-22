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

Commits `4b273e7` (spec), `dc8689a` (build) and the audit commit after it. ADR-0006,
R-SENS-17, R-WORLD-11, A-14..A-18, F-23..F-27. Closes C8's first open item: a sensor beyond
the flown two is now a model of something, with every number it needs registered.

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
- `constants.py` — `NAV_AID_*` from the booklet; `UWB_*` as A-14..A-18 with their sources;
  `UWB_RATE_HZ` and `UWB_ANCHOR_HEIGHT_M` as deployment defaults.
- `examples/04_uwb_ranging.py` — six anchors across both rows of the Start Area and four on
  tripods in the Known Search Area, checked against the rules; a policy that reads the tag
  by name; and a grade of the sensor from the log alone.
- Docs: ADR-0006; SPEC R-SENS-17, R-WORLD-11, an R-POL-3 note, §12; FIDELITY A-14..A-18 and
  F-23..F-27; a UWB section in `docs/06`; the nav-aid rules and the table update in
  `docs/10`; `docs/07`, `docs/04`, ARCHITECTURE, README.

**Verified — TESTED**, at this checkpoint's commit: 321 tests, `tests/test_uwb.py` at 39
items (44 after C10). It checks: the defaults are the
registered constants and the tag is not flown; twelve impossible configs refused by
`UWBConfig` and a rate that does not divide the tick by `RunConfig`; the range is three-dimensional to the anchor at mount height; anchors come
back in arena order and only the tag's kind; an empty sweep; beyond reach is `inf`; a wall
biases while a marker and a teammate do not, and the ring disagrees on purpose;
obstruction follows the drone's altitude; on a generated arena the tag's obstruction equals
the mission's line-of-sight scene at 40 random poses; the noise model as a pure function —
zero-mean Gaussian at A-14 in line of sight, biased and wider and dropped at A-16/A-17
behind a wall, `inf` beyond reach, never negative, outliers off by default and positive
when on; the noise stream is independent of the geometry; identical runs identical;
appending the tag leaves the ring's stream untouched (R-DET-3); the reading reaches the
policy by name, fresh every other tick, immutable, with no `Landmark` in it; an arena with
anchors cannot be flown without the tag; the log holds `ranges_m` shaped `(ticks, agents,
anchors)` and the anchor positions in the header's landmark order, and grading it from the
log alone puts every line-of-sight error inside six sigma with a mean within 2 cm of zero
(measured 0.000 m); recording does not change the run; `record_static()` before the first
sample is refused; the example's layout passes the rules and runs. After the audit: a
mission kind is refused; a subclass that skipped `super().__post_init__()` is caught at
build; numpy scalars are accepted and a bool is not; an anchor on the field's edge is never
obstructed by the perimeter; the noise stream is independent of reach as well as of walls.
`tests/test_landmarks.py` (+6): any number of aids in the Start Area and ten in the Known
Search Area pass; an eleventh is refused and kinds count together; an aid inside the room,
or on its wall, is refused and one just outside is not; an aid wider than a metre is
refused; the runner does not referee; a string or an empty `kinds` is refused rather than
silently passed. `tests/test_sensor_primitive.py` (+2): a reading's array cannot be made
writable again, on its own or through the ring's scan. The R-POL-4 walk now carries a UWB
reading and still bans the `Landmark`; the units-suffix test covers `UWBConfig`.

**Verified — MEASURED**, on the pre-maze arena this checkpoint was built against; the merge
below re-measured them and C10 carries the current figures. `examples/04_uwb_ranging.py`,
seed 0, 10 drones, 60 s, ten anchors:
6 000 fresh sweeps; 70.0 % of tag–anchor paths in line of sight, 93.2 % within 20 m; heard
on 100 % of in-reach line-of-sight paths and 90.1 % of in-reach paths behind a wall (A-17
is 0.10); line-of-sight error 0.000 ± 0.050 m (A-14 is 0.05); behind a wall +0.157 ± 0.398 m
(A-16 is +0.15 and 0.40). `uwb.npz` is 207 kB (206,636 bytes) for that run; the numbers are
identical before and after the audit fixes. The full suite takes about a minute on a laptop
(54–96 s across three machines).

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

**Audited.** Two independent adversarial audits — one against the code and the spec, one
against every claim in the docs — and a skeptic pass on the second, which upheld 19 of its
36 findings, upheld 12 in part and refuted 5. What changed code, all now under test:

- **A tag could range to the mission markers.** `UWBConfig(kind="victim")` was accepted and
  reported every victim's true position as a surveyed anchor on the first sweep, and the
  R-POL-4 walk could not see it, because a position array is exactly what the reading may
  carry. Both auditors found it. Mission kinds are refused, at construction and at build.
- **A read-only array could be made writable again.** numpy allows `flags.writeable = True`
  on an array that owns its memory, which `read_only()`'s copy did; an auditor moved a UWB
  anchor for the rest of a run and wrote `-123` into a held ToF scan and so into the log.
  Every sensor was exposed. `read_only()` now lays the copy over a `bytes` object, which
  refuses the flip, and the build-time check walks the base chain down to it.
- **`validate_nav_aids(arena, "uwb_anchor")` approved anything.** A string is a sequence of
  letters, so nothing counted as an aid. A string, or an empty `kinds`, is refused.
- **An anchor on the field's edge was obstructed by the perimeter** one time in a hundred,
  by rounding in the strict line-of-sight comparison. `segment_clear` tolerates a nanometre.
- **A sloppy subclass ran silently wrong.** A config that skipped `super().__post_init__()`
  with `nlos_drop_probability=5.0` flew a whole mission with every obstructed range dropped.
  The config is re-validated at build, as the arena re-validates its landmarks.
- **A surviving mutant.** Drawing the Gaussian only for anchors in reach passed all 33
  tests; the geometry-independence test now varies reach as well as walls, and kills it.
- numpy scalars are accepted; a bool is refused with a reason that is true.

Doc claims the audits falsified and that were corrected: "20 m does not reach the far end
of the field" (three Start Area anchors reach every point at 20 m; at 12 m the far third
hears none); "give them a footprint for a blind control" (a footprint alone is still
refused); "through concrete the link would die" (the measurement A-16 rests on was made
through pre-stressed concrete panels, which ranged with a +0.15 m bias); "nobody has
measured a second wall" (the same table gives +0.58 m and 0.61 m); "a full sweep at 10 Hz is
the PANS ceiling" (PANS returns four ranges per frame); the skew arithmetic; the F-number
range; the 1.39 ns → 0.40 m rounding; an R-SENS-7 citation that should have been R-MISS-2;
"eleven configs refused at construction"; "a zero mean"; 204 kB; 96 s; and several lists of
"the two sensors". Refuted and kept: §6.3 does name ultra-wideband as acceptable; r.15 does
say teams enter the Known Search Area only during setup; F-25's "no taller than the 2.0 m
inner walls" is the exact condition for this arena.

**Open.** Every number is an assumption: A-15 (reach) first, then A-16/A-17 against the
venue's actual walls. Obstruction is boolean — one wall's numbers behind any number of
walls (F-24) — and calibration is assumed (F-26). No `PoseSource` consumes the tag yet;
that is the next piece of work (ADR-0003), and until it exists a policy that wants a
position from these ranges trilaterates for itself. The CLI has no `--sensors`, so the tag
is configured in Python, as the example shows. The replay does not draw `uwb.npz`.

**Merged with `main` at `adb78f3` (PR #5, the maze and the arena's three streams).** Two
requirements and one check collided, and the reconciliation is the interesting part.

- **R-WORLD-9 was taken.** `main` uses it for the three independent arena RNG streams and
  R-WORLD-10 for the arena store, so the nav-aid requirement is now **R-WORLD-11**, rewritten
  to describe a division of labour rather than one check.
- **`main` already enforces the room rule.** `_validate_landmark_zones` refuses any placed
  landmark inside the Unknown Search Area on every run, calling it "the highest-value cheat
  the rules forbid". That is stronger than this branch's opt-in check and it wins: a run with
  an anchor in the room is no longer possible, and ADR-0006's argument that such a run is a
  legitimate experiment is withdrawn for the room specifically. It still stands for the cap.
- **`main` explicitly left the cap of ten undone**, because "a `Landmark` may equally be
  scenery, a prop or a venue feature; the primitive carries no field distinguishing them, so
  a blanket cap would reject legitimate arenas. Assert it in your own experiment, or give
  `Landmark` a nav-aid flag first." `validate_nav_aids(arena, kinds)` is the other answer to
  that question: the caller names the kinds, so no flag on the primitive is needed. The
  function now checks only what `validate_arena` cannot -- the cap (r.15) and the 1 m x 1 m
  footprint (r.14 f) -- and its room check is gone as redundant.
- `NAV_AID_LIMIT_KNOWN_AREA` became `main`'s `NAV_AID_MAX_KNOWN_AREA`; `NAV_AID_FOOTPRINT_M`
  is new and stays. The nav-aid tests now survey the generated arena with `in_known_area`
  rather than assuming a column of fixed coordinates clears the room, which the maze made a
  trap: the room moves with the seed.

**Verified — TESTED.** 372 tests pass on the merge. The example's ten anchors, which sit near
the field edges, are checked to generate and validate on every seed 0-59.

---

## C10 — the UWB tag becomes a DW3000

The team chose the part: **Qorvo DW3000**. ADR-0006 addendum, R-SENS-17 amended, A-14..A-18
re-sourced, F-28..F-31.

**Built.**

- `constants.py` — `UWB_MODULE_PART = "DW3000"` and a part-number block in the style of the
  VL53L5CX one, because the same trap exists: a number from a DW1000 paper is a number about
  a different radio. Every assumption's docstring now says which part its source measured.
  New: the two channel centres and the bandwidth, the 10 ms TDMA slot, and the 8-anchor cap a
  shipping firmware imposes.
- `sensors/uwb.py` — `sweep_rate_hz(n_tags, slot_s)`. The model's fixed rate hid the fact
  that TDMA slots belong to tags, so the sweep rate falls with the *fleet*: 10 Hz at ten
  drones, 4 Hz at twenty-five, on the same radio and the same anchors. It is a helper the
  scenario author calls, not something the runner applies, because a sensor config knows
  nothing about `RunConfig.n_drones`.
- `examples/04_uwb_ranging.py` — takes `n_drones` and derives its rate from it.
- Docs: the ADR addendum; `docs/06` gains a part-number callout and the RF-compliance
  argument; `docs/02` records the choice in the hardware table; SPEC R-SENS-17 and §12;
  FIDELITY F-28..F-31.

**Verified — TESTED.** 378 tests. New in `tests/test_uwb.py`: the modelled part is the
DW3000 and both its channels clear the banned 5.7-5.9 GHz band by arithmetic on the
constants; the sweep rate is `1/(n_tags * slot)` and does not mention anchors, with the
fleets that divide the 20 Hz tick and one that does not; a 25-drone run really does age its
reading four ticks between sweeps; and **F-30 is pinned** -- the model must stay optimistic
against the measured DW3000 in all four statistics, but within a factor of two in the body,
and switching the outlier term on must fatten the tail.

**Verified — MEASURED.** The model against the one independent measurement of the part
(Ember et al., IFIP 2024: a DW3000 on channel 9, 125 positions in a 60x40 m office):

| | measured | this model | |
|---|---|---|---|
| line of sight, mean absolute error | 5.7 cm | 4.0 cm | optimistic by 1.4x |
| line of sight, 90th percentile | 13.7 cm | 8.2 cm | optimistic by 1.7x |
| obstructed, mean absolute error | 46.7 cm | 34.1 cm | optimistic by 1.4x |
| obstructed, 90th percentile | 129.5 cm | 70.3 cm | **optimistic by 1.8x** |

The body is within a factor of two; the tail is not, because a Gaussian tail is not a UWB
tail. `outlier_probability` (A-18) is the term for that and is off for want of a published
rate. `los_noise_std_m=0.07` matches the measured line-of-sight mean absolute error but not its
90th percentile: no Gaussian matches both. The example, rerun after the
maze merge, reports 6 000 sweeps, 64.2% of paths in line of sight (was 70.0% before the
maze), 94.2% in reach, line-of-sight error 0.000 +/- 0.050 m and 0.155 +/- 0.397 m behind a
wall, and 89.9% heard behind a wall in reach.

**Found while building.** Three priors were wrong and are corrected in the ADR: the
datasheet's "10 cm" is marketing copy from page one, not the specification (Table 14 says
+/-6 cm calibrated, +/-15 cm uncalibrated, 1.5 cm standard deviation); the DW3000 is **not
faster** than the DW1000, per-frame airtime being essentially identical; and it is
**shorter**-ranged, having dropped the DW1000's 110 kb/s long-range mode. Its gains are power
and 802.15.4z security. Also: a first draft of `sweep_rate_hz`'s docstring claimed fifteen
drones give a rate the runner refuses. It does not -- 6.67 Hz divides 20 Hz exactly. The rule
is that the decimation is `0.2 * n_tags`, so multiples of five divide and the rest do not.

**Behaviour that changed.** Defaults are unchanged, so an existing UWB run reproduces
exactly. What changed is what the numbers are *about*, and the example now derives its rate
from its fleet rather than taking the 10 Hz default.

**Open, and this is the honest part.** The DW3000 evidence is thinner than the DW1000
evidence it replaces. Two independent literature searches disagreed on whether a DW3000
through-wall measurement exists at all; what is citable is an aggregate error, and A-16's
**spread is still a DW1000 number** from the one paper this repository has read in full
(F-28). Antenna-delay calibration is a per-unit constant offset of up to 15 cm that this
model has no term for, and which module the team buys decides its size (F-31). Range varies
3-5x with data rate, preamble and PAC, which one scalar cannot express (F-29). Nothing has
been measured on the team's own kit in the hall, which is what A-14 through A-18 exist to ask
for. And the tag still feeds no `PoseSource` (ADR-0003).

**Audited.** One adversarial pass over both commits, told to falsify. It confirmed the
arithmetic it could check independently — the channel edges, the F-30 model figures to two
decimals, `sweep_rate_hz` for every fleet size 10-25 through actual `RunConfig` acceptance,
`in_known_area` against 80 000 maze-corridor samples, and every changed markdown anchor — and
found nine defects, all now fixed:

- **The RF claim overclaimed, on the one matter that carries disqualification.** "It cannot
  break the RF rule", "no configuration reaches it", "provably satisfied". The channel plan
  proves no channel's *occupied band* overlaps 5.7-5.9 GHz; it does not prove zero emission
  there, and `constants.py` already said the datasheet shows about -71 dBm/MHz inside the
  band. All three statements now claim the occupied-bandwidth argument and name its limit.
- **The merge collided two assumption ids.** PR #5 took A-9 and A-10 for the maze corridor
  and the marker census while this branch had already taken A-9..A-13, and the merge simply
  concatenated the tables. The UWB block is now **A-14..A-18**; A-11..A-13 are retired unused
  and both registers say so. SPEC §12 was also missing the marker-census row entirely, which
  made FIDELITY's "mirrors SPEC §12" false, and both tables were out of numeric order.
- **A landmark could reach a run inside the room after all.** `_validate_landmark_zones` read
  `spec.landmarks` while a sensor reads `all_landmarks`, so a plain `Landmark` smuggled into
  `spec.targets` with `dataclasses.replace` gave a UWB tag a free anchor in the middle of the
  Unknown Search Area, past both the zone rule and the cap. The check now walks
  `all_landmarks` minus actual `Target`s, which keeps 3.3.9 r.2's generated markers exempt.
- **F-28 named two different divergences.** Eight citations meant "the fixed rate does not
  carry the fleet", which had no ledger row at all; that is now **F-32**.
- **F-23 was stale by about eight times.** Its 3-4 ms per-anchor slot premise was overturned
  by this very change: slots are per tag at 10 ms and an exchange is ~0.51 ms per anchor.
- **The withdrawn 60 m reach story survived in two live places**, contradicting A-15 fifty
  lines below it in the same files.
- **The ADR still called the room rule opt-in.** The withdrawal existed only in a commit
  message and this file; the record itself now carries the amendment.
- Smaller: the calibration material cited F-26 where it belongs to F-31; the obstructed mean
  absolute error is 34.1 cm, not 34.2, which was Monte-Carlo noise recorded as a figure; and
  "`los_noise_std_m=0.07` matches the measured line of sight" is true of the mean absolute
  error only — its 90th percentile stays about 16% optimistic, because no Gaussian matches
  both. That last one is F-30 restating itself.

**379 tests.**


---

## C11 — the spec for tag-to-tag ranging and a relay trial

The team's brief, 2026-09-17: *use UWB to form a relay, with tags on the drones and one
anchor at the start line, via a potential-based method that converges towards a relay.*
Research first, then a spec, then code — this checkpoint is the spec.

**Built.** `docs/adr/0007-tag-to-tag-ranging-and-the-relay.md`; SPEC R-SENS-15 amended,
**R-SENS-18** added, **A-19** registered; FIDELITY A-19 mirror and **F-33..F-36**;
`constants.UWB_PEER_EXCHANGES_PER_SLOT` with its source line. No code behaviour changed.

**Found while researching, and it reframes the brief.** In this rulebook a *relay* is the
scoring chain of R-MISS-4 — landed drones ≤ 1.0 m apart with floor-level line of sight from
a bonus-victim rescuer into the Start Area, worth ×2 — not a communications link. The docs
say the real fleet has no drone-to-drone radio at all (`docs/02-hardware.md`). The team
confirmed the scoring relay is the goal. It has never fired in any recorded run.

**Found while auditing the platform for readiness.** Three blockers, none of them in the
tag itself: `WorldScene` gives a sensor no sanctioned view of another drone's position
(anonymous circles, `x, y` only, private arrays); `Runner._sense` drops every non-`ACTIVE`
drone from the scene, and a relay is made of landed drones; and identity, draw-count and
TDMA conventions all need restating for a reading whose targets are the fleet.
`WorldScene.fleet` is the answer to the first two and is deliberately a second query
beside the bodies, because a radio and a ray disagree about a parked airframe.

**Found in the literature** (three threads, ~60 sources; the ADR's rejected-alternatives
table is the digest). Range-only formation control almost always means *position* vectors
with distance targets; a truly scalar-range chain is not rigid in the plane and the published
fixes — dither, stop-and-go trilateration, extremum seeking — are slow and fragile under
dropout. The design that survives is a one-dimensional spring chain along a *trail* a
teammate flew, which fixes the geometry the ranges cannot, converges to equal spacing with
both ends fixed, and needs the anchor for exactly one thing: certifying the tail is in the
Start Area from a range rather than a pose. The closest validated system is Varadharajan et
al.'s *Swarm Relays* (ICRA 2020, six Crazyflies and seven Kheperas), which is the same idea
with spacing zones instead of a potential.

**Verified.** Nothing executable yet. ASSERTED: A-19's arithmetic (5 Hz at ten drones and
one anchor, 1 Hz at twenty-five) — a test pins it in C13. The trial's own requirements are
T-1..T-5 in the ADR, written to be falsified in C15.

**Open at this point.** All code: C12 the fleet query, C13 peer ranging, C14 the metric,
C15 the example and its sweep, then the audit.

---

## C12 — `WorldScene.fleet`: every drone, every lifecycle, as a sensor's view

**Built.** `sensors/scene.py`: a frozen `Fleet(agent_ids, object_ids, xyz)` with `index_of`,
`WorldScene.fleet`, and `refresh_fleet(entries, cache_key)` — cached by tick and independent
of `refresh_drones`, because the two are built from different subsets: bodies from active
drones (what a ray can hit, what a drone can strike), the fleet from all of them (what a
radio can range to). `runner._sense` feeds every agent's `(agent_id, robot.id, state)` from
the same post-step state as the bodies. `Fleet` joins the R-POL-4 banned types and the
smuggle list. `docs/10` and `sensors/base.py` say what the fleet is for and what it is not.

**Verified — TESTED.** 396 tests (+9 in `tests/test_fleet.py`). The fleet is run-ordered,
read-only through `.base` and the writeable flag, rebuilt once per key, refused on a repeated
or empty id, and empty in a hand-built scene. On the runner path: land half of ten drones at
tick 5 and the bodies a sensor sees fall from nine to four while the fleet stays at ten;
every sensor's `index_of(truth.object_id)` is its run index; drone_00's fleet `z` equals its
own pose each tick and hits `0.0` the tick after it lands. R-POL-4: a `Fleet` inside a
reading is caught by the walk.

**Behaviour that changed.** None visible to a policy or a log: no existing sensor reads the
fleet. One more per-tick pass over the agent list in `_sense`.

**Open.** No sensor consumes it yet — that is C13.

---

## C13 — the tag ranges to its peers

**Built.** `sensors/uwb.py`: `UWBConfig(peers=True)`; `UWBRanges.peer_ids` /
`peer_ranges_m` / `peers_heard`; `peer_line_of_sight` (the structural segment test at the
lower of the two altitudes, grouped by altitude so a fleet costs one cast per distinct `z`);
`peer_sweep_rate_hz(n_tags, n_anchors)` under the A-19 slot budget; `record()` adds
`peer_ranges_m` only when peers are on. Peer noise is drawn from a **child generator spawned
from the tag's own at build**. `docs/06` gains a peers section; `tests/test_audit_regressions`
whitelists the dimensionless `peers` field.

**Verified — TESTED.** 408 tests (+12 in `tests/test_uwb.py`). Self is `inf` and in the
list; a peer range is the 3-D distance (0.8 m across and 0.5 m down reads 0.94 m); a wall
biases a peer link and a teammate's body does not; line of sight is at the lower altitude
(a 1.0 m wall blocks a 1.2 m drone from a landed one and not from another at 1.2 m); a peer
beyond reach or dropped is `inf`; the anchor noise is identical with peers on and off for
five consecutive sweeps **and the parent generator's next draw is identical**; the child
consumes exactly four draws per fleet member per sweep whatever the reach or lifecycle;
`peer_sweep_rate_hz` gives 5 Hz / 1 Hz / 1.67 Hz for 10 / 25 / 20 drones with one anchor,
collapses to `sweep_rate_hz` when everything fits one slot, and twelve drones are refused by
the runner while ten, fifteen, twenty and twenty-five are accepted; end to end, a landed
teammate is still ranged and the range grows by exactly the height gap; the log holds
`peer_ranges_m` square in the header's agent order with an `inf` diagonal and grades within
6σ against `states.npz`; a peers-off log has exactly the old keys and the same anchor
ranges as a peers-on one.

**Found while building — two of the spec's claims were wrong and the tests caught both.**
(1) R-SENS-18 first said the peer draws come "after the anchor draws, so that the anchor
stream does not depend on `peers`". That holds within a sweep and fails by the next one:
the parent generator has advanced by the peer draws. Fixed by spawning a child generator
for peers at build, which also makes the byte-identity claim for peers-off true by
construction; the spec, the ADR and the code now say so. (2) `peer_sweep_rate_hz`'s
docstring said fifteen drones with one anchor give a rate the runner refuses. They do not:
the decimation is `0.2 × n_tags × slots` = 6. Twelve is the fleet that does not divide.
The same trap C10's audit found in `sweep_rate_hz`.

**Behaviour that changed.** None for any existing run: `peers` defaults off, the reading's
new fields default empty, `record()` is unchanged when off, and the parent generator is not
touched.

**Open.** No metric says *when* a relay forms (C14); no policy reads the peer ranges (C15).

---

## C14 — the relay as a moment: `relay_timeline`, `time_to_relay_s`, `relay_chain_drones`

**Built.** `metrics.py`: `RelayMoment` and `relay_timeline(run)`, which replays a fresh
`Mission` built from the recorded arena over `states.npz` at every tick on which the landed
count grew — the mission's own latched `update` and `_find_relay`, not a second
implementation of the rule. `RunMetrics` gains `time_to_relay_s` (first moment with a
chain, or `None`) and `relay_chain_drones` (length of the footer's final chain). `docs/07`
says so.

**Verified — TESTED.** 411 tests (+3 in `tests/test_relay_metric.py`). A bonus victim
planted at `(1.0, 7.6)` — the Known Area's west corridor, clear of the room on every seed
— and three scripted drones landing at 0.8 m steps from `y = 6.9` to `y = 5.3`: the online
score, the offline re-score and the timeline agree on the chain `drone_00 → drone_01 →
drone_02`; the timeline has one moment per landing, empty chains for the first two and the
chain on the third, at exactly the tick the log's lifecycle array shows the third drone
LANDED; `time_to_relay_s` is that moment; the score is `2 × 15`. Pull the middle drone
1.1 m south and no replay finds a relay, `time_to_relay_s` is `None`, the rescue still
scores 15 unmultiplied. A run with no landings has an empty timeline.

**This is also the first recorded run in which the relay fired**, scripted rather than
searched, in a test. REVIEW.md's "has never fired in any recorded run" is now false in the
narrow sense; the open question it was pointing at — can a *policy* reach it — is C15.

**Behaviour that changed.** `RunMetrics` has two more fields; nothing else.

**Open.** The example and its sweep.

---

## C15 — the relay trial: an elastic band of drones along a crumb trail, landed on UWB alone

Commits `d48347c` (the example), `f8959fc`, `8e97118`, `64af741`, `be8c10d`, `687f85c`,
`c86a48b` (two audits' and a skeptic's fixes), and this one. ADR-0007 "The trial" and
"Amendments from the build"; F-36.

**Built.** `examples/06_uwb_relay.py`, one registered policy `uwb_relay` that assigns a role
per drone from the tag's roster: `drone_00` the *lead* (wasp_v5 that lands only on a bonus
victim), `n_relay` *relays* from the southern take-off row (`relay_roles`), the rest wasp_v5
searchers with the mission wrapper. Searchers leave the Start Area north along their column
(staggered 0 / 1.5 / 3 s by index mod 3, handing over to wasp_v5 when the ring sees anything
1.4 m ahead past the anchor row), drop a breadcrumb with their ring clearance every 0.25 m,
and announce a head when they land on a bonus victim. A relay assembles a **trail**: in
dispatch, the shortest route through every searcher's crumbs to the nearest anchor
(`CrumbNetwork`: flown segments, ring-evidenced cross-links, anchor edges inside 0.9 m;
Dijkstra once per head); in train, the lead's own trail grown incrementally
(`TrailBuilder`), with map-free loop cuts (`Trail`). A relay joins the trail at the
projection of its position onto the trail's first 1.5 m. The controller is the ADR's band:
a one-dimensional spring potential on **measured** ranges to the two chain neighbours,
stepped per fresh sweep only while the relay is on its carrot, equal spacing its
equilibrium; a per-link UWB gate (both neighbour ranges ≤ 0.9 m horizontal for three
consecutive sweeps), the tail's predecessor an anchor at tag height on the row `y = 5.0`;
a same-tick consensus landing from the snapshot. `--anchors N` places a row (one is the
brief), dropping any anchor with structure inside its 0.95 m disc for that seed; `grade()`
checks T-2 and T-3 from the log alone; `--sweep` compares dispatch, train and no relay over
seeds. `tests/test_relay_trial.py`, 14 tests.

**Verified — TESTED.** 428 tests. The trail accumulates arclength, interpolates and
projects; a revisit inside a seen free disc is cut and one without evidence is not;
`relays_needed` is the spacing arithmetic; the network links an anchor to crumbs anywhere
inside 0.9 m (the audit's reproduction) and routes through another searcher's crumbs at a
fifth of the head's own trail; anchors within 0.95 m of structure are dropped for that seed
(seeds 16, 144 and, for the lead's column, 99), the legal room face drops none over seeds
0–59, and a lone anchor is kept; relays come from the southern row and launch northern-row
first. **T-1** the relay forms on a planted bonus victim in both modes inside 120 s and
scores 2 × 15; **T-2** every relay that landed is a link of the chain at formation, mean
link ≤ 0.9 m, every link clear, tail in the Start Area; **T-3** every landed relay's last
fresh sweep measured two links inside the gate and one certified against the anchor, and —
the clause a gate that ignored the ranges would fail — one relay on a trail that needs two
joins in train mode, parks between 0.9 and 1.35 m from the head and from the anchor on the
floor (asserted), and hovers to the end of the run, while in dispatch it never launches;
**T-4** the trial refuses to run without `peers=True`, without an anchor, with an unknown
mode or with no searcher left; the relays landed on the same tick; the third relay never
left the ground.

**Verified — MEASURED (T-5).** `python examples/06_uwb_relay.py --sweep`, 600 s runs,
`collision_behaviour="stop"`, `sensor_every=4`, at commit `c86a48b`; every figure below is
recomputed from the run directories by a script, not typed. Ten drones, five seeds a cell:

| mode | anchors | relays | P(relay) | score | ± | t_relay (s) | targets | crashed | waves |
|---|---|---|---|---|---|---|---|---|---|
| none | — | 0 | — | 38.0 | 8.1 | — | 3.4 | 1.2 | 1.0 |
| dispatch | 1 | 3 | 0.00 | 34.0 | 9.2 | — | 2.8 | 1.2 | 1.0 |
| dispatch | 1 | 5 | 0.40 | 41.0 | 17.4 | 88 | 2.6 | 0.4 | 2.2 |
| dispatch | 1 | 8 | 0.20 | 25.0 | 13.0 | 58 | 1.6 | 0.2 | 1.4 |
| dispatch | 10 | 3 | 0.40 | 39.0 | 16.6 | 43 | 2.4 | 1.2 | 1.2 |
| **dispatch** | **10** | **5** | **0.80** | **47.0** | 12.5 | 74 | 2.4 | 0.6 | 1.8 |
| dispatch | 10 | 8 | 0.60 | 35.0 | 8.9 | 80 | 1.8 | 0.2 | 2.4 |
| train | 1 | 3 | 0.00 | 31.0 | 4.9 | — | 2.6 | 0.8 | 2.4 |
| train | 1 | 5 | 0.20 | 37.0 | 6.8 | 66 | 2.8 | 0.2 | 3.0 |
| train | 1 | 8 | 0.20 | 23.0 | 13.6 | 48 | 1.4 | 0.0 | 4.0 |
| train | 10 | 3 | 0.00 | 33.0 | 7.5 | — | 2.8 | 0.6 | 2.8 |
| train | 10 | 5 | 0.20 | 34.0 | 9.7 | 64 | 2.8 | 0.6 | 3.2 |
| train | 10 | 8 | 0.20 | 26.0 | 12.4 | 50 | 1.8 | 0.0 | 4.2 |

Twenty-five drones, three seeds a cell:

| mode | anchors | relays | P(relay) | score | ± | t_relay (s) | targets | crashed | waves |
|---|---|---|---|---|---|---|---|---|---|
| none | — | 0 | — | 48.3 | 6.2 | — | 4.3 | 1.0 | 4.3 |
| dispatch | 1 | 8 | 0.00 | 38.3 | 2.4 | — | 3.3 | 0.7 | 4.0 |
| dispatch | 1 | 15 | 0.33 | 50.0 | 21.2 | 75 | 3.0 | 0.0 | 1.7 |
| **dispatch** | **10** | **8** | **1.00** | **76.7** | 4.7 | 40 | 3.3 | 1.0 | 2.7 |
| **dispatch** | **10** | **15** | **1.00** | **73.3** | 4.7 | 77 | 3.0 | 0.0 | 2.0 |
| train | 1 | 8 | 0.00 | 38.3 | 2.4 | — | 3.3 | 0.7 | 4.7 |
| train | 1 | 15 | 0.00 | 36.7 | 2.4 | — | 3.0 | 0.0 | 3.3 |
| train | 10 | 8 | 0.00 | 38.3 | 2.4 | — | 3.3 | 0.7 | 5.3 |
| train | 10 | 15 | 0.00 | 36.7 | 2.4 | — | 3.0 | 0.0 | 3.3 |

Across every formed relay (16 + 7 runs, 50 + 14 links): mean link 0.76 m; longest 0.90 m
at ten drones and 0.97 m at twenty-five — the latter the scorer starting its chain from a
*second* bonus rescuer 0.97 m from a relay whose own head was 0.89 m away; every link in
floor-level line of sight; every tail in the Start Area; every landed relay with at least
two links inside the gate on its last sweep (0 exceptions in 23 runs); every formed relay
with an anchor-certified relay; no relay landed in any run in which the mission did not
score a relay. Formation times, raw: 26–141 s at ten drones (median 60), 36–120 s at
twenty-five (median 49). Chain lengths: one relay in nine runs (heads 0.2–0.5 m north of
the line), two in one, four or five in thirteen (heads up to 2.8 m north). In 4 of 23 runs
a relay that landed is not in the scored chain — the mission's BFS returns the *shortest*
chain and skips the anchor-certified tail when another relay sits inside the Start Area,
1–31 cm past the line; those relays each had two gated links of their own. Every "10"
anchor cell in both sweeps ran with all ten anchors placed.

**What the numbers say, under the conditions below.**

1. **The relay pays for itself when it forms.** Ten drones, five relays, an anchor row:
   formed in four seeds of five, mean score 47 against the no-relay fleet's 38. Twenty-five
   drones, the row: eight relays formed it three times in three for 77 against 48, fifteen
   relays three in three for 73. Eight relays out of ten leave too few searchers (1.8
   targets, 35); three cannot span most trails.
2. **One anchor costs most of the relays it forms.** The same five relays with the single
   anchor of the brief: two seeds of five and a mean of 41, a wash against 38; at
   twenty-five drones, none of three with eight relays and one of three with fifteen. The
   chain has to come back to the one point that can certify its tail. The fix is the row,
   and rule 3.3.1 r.16 permits it.
3. **Train is worse than not relaying.** Every one of the twelve train cells scores below
   its baseline. The relays follow the lead's own wandering trail, which is usually too long
   for them, and hover to the end; and reeling relays out one at a time crosses the line in
   2.4–5.3 waves against a limit of two. Dispatch, which waits and routes, is the deployment
   to keep.
4. **Time.** From the start of the run to the relay: a median of 60 s at ten drones and
   49 s at twenty-five, the tail of the distribution (up to 141 s) being relays flying the
   anchor row from the far end of the grid and long chains settling.
5. **The two-wave rule binds** on five of ten dispatch cells (2.2–4.0 waves; the other five
   1.0–2.0) and on every train cell; the twenty-five-drone *baseline* itself crosses in 4.3
   waves, because searchers turned back by the room's face cross late. It is reported, not
   enforced, and the trial makes no attempt to cross abreast.

**Statistical weight, before anyone quotes a cell.** The cells hold five seeds (ten drones)
or three (twenty-five), and the `±` is a population standard deviation. Per cell, the
contrasts above are not significant: row vs one anchor at five relays, 4/5 vs 2/5, Fisher
exact p = 0.52; at twenty-five drones and eight relays, 3/3 vs 0/3, p = 0.10; the ten-drone
47 vs 38, Welch p = 0.27. What survives is the pooled contrast and the one clean cell:
**dispatch forms the relay more often than train** (12/30 vs 4/30 at ten drones, p = 0.039;
7/12 vs 0/12 at twenty-five, p = 0.005); **a row forms it more often than one anchor** across
every dispatch cell (15/21 vs 4/21, p = 0.002); and the twenty-five-drone row cell's score,
77 vs 48, Welch p = 0.008 (Mann–Whitney 0.077). Say those; do not say "the row doubles
P(relay)" from a five-seed cell. (Computed from the run footers with `scipy.stats`; a
lesson's completeness critic raised it, and the arithmetic was re-run before this was
written.)

**Conditions (F-36).** Trail following, the carrot, the approach, the crumbs and the
publications that pace launches run on ground-truth pose, and every horizontal correction
uses the relay's own true altitude; crumbs, head announcements, chain order and the landing
consensus run on the perfect blackboard (ADR-0003). Roles come from the tag's roster. The
0.1 m gate margin covers A-14 and not the per-unit antenna-delay offsets (F-34). The gate
certifies distance; line of sight comes from the trail's construction and is checked
offline. The searchers are wasp_v5 with a north leg and a mission wrapper, on the census
assumption A-10 and the detection range A-4. Nothing is measured on the team's kit.

**Found while building — OBSERVED, the runs overwritten by later ones.** The record carries
nine amendments; the audits and the skeptic forced most of them. In order of consequence:
one certification point makes the relay expensive (a head 12 m east of the single anchor
needed sixteen relays); the head's own trail is what it *flew*, and a Lévy walker's is long
— routing through every searcher's crumbs cut a 9-relay trail to a 2-relay one; a relay
aiming at the anchor point through a hovering teammate deadlocked for minutes; the band
must not step while the airframe is still answering the last step; the gate must be per
link, because a "balanced" gate is defeated by A-14; the anchor must stand at tag height,
because a range is three-dimensional and the horizontal reduction dilutes it by `r / h`;
launch order must be arrival order; the north leg must hand over before a wall and columns
must not reach it together; a 1.2 m anchor clearance thinned the row on 46 of 200 seeds;
and `main`'s generator lets an inner wall reach into the Start Area on 22 of 200 seeds,
which the docs deny (a task is flagged). Recomposing a trail from scratch each tick made a
600 s run take six minutes.

**Audited.** Two adversarial passes, told to falsify, one on the platform pieces (C12–C14)
and one on the trial; then every claim of this section's first draft was sent to a skeptic
with the run directories. Findings and what changed:

- A sensor returning `world.fleet` passed the build-time contract check; the ban was
  test-time only. `check_reading_is_immutable` now refuses `Fleet`, `WorldScene`,
  `Landmark`, `TrueState` and `Sensor` inside any reading, private fields included.
- F-34's "the altitude changes no answer" was false at a pillar's 0.15 m base — where the
  lower-altitude rule in fact agrees with the scorer's floor-level line of sight. Reworded in
  four places.
- "Ten times at 13–14 drones" was 4.2–4.5× by the code's own arithmetic; the
  child-draw-count test compared the parent generator; adding a drone perturbs every tag's
  peer stream (now stated in R-SENS-18 and pinned); `relay_chain_drones` is the footer's
  chain (now said so).
- `CrumbNetwork` dropped every anchor edge between 0.6 and 0.9 m (cell hash vs reach);
  every dispatch result before the fix was biased toward infeasible.
- Searchers parked against the room's south face, and neighbours reaching it together
  turned into each other; the handover, the stagger and the slide followed.
- T-3 could not fail on a gate that ignored the ranges; the one-relay-on-a-two-relay-trail
  test can, and its parking distance is asserted. T-2's "links ≤ 1.0 and clear" is
  tautological once T-1 holds and is now stated as such.
- The grader judged the mission's *shortest* chain, which skips the anchor-certified tail
  when a relay lands inside the Start Area; T-3 is graded over every landed relay.
- With two take-off rows, "the last `n` ids" parked the relays north of the searchers;
  `relay_roles` takes the southern row first.
- **The skeptic:** the first draft's "174–183 s to a relay at twenty-five drones, mostly
  flight" was the range of two cell means over raw values of 40–466 s, and the cause was a
  deadlock, not flight — the second relay aiming at the anchor point through the first
  relay's body; the anchor row was silently thinned on 46 of 200 seeds by a 1.2 m clearance
  that also caught the room's legal face, two sweep seeds among them, and the first anchor
  was exempt from a check three seeds needed; "every landed relay is a link" fails whenever
  the scorer shortcuts; "1.8–2.7 waves" was the two bold cells; "one-relay chains, heads a
  metre north" was three two-relay chains and heads 0.2–0.5 m north; "1.15 m" was a
  docstring, not an assertion; the ADR said five amendments and listed eight. Each is fixed
  above or in the ADR, both sweeps were rerun at the fixed commit, and the numbers here come
  from a script over the logs.
- Stated as overclaims and left as caveats: the gate certifies distance, not line of sight
  (0 of 13 293 ~1 m chords along real trails were floor-blocked in the auditor's scan); the
  margin does not cover calibration offsets; a relay that crashes mid-chain keeps its last
  blackboard tuple for ever and the chain hovers.

**Open.** A lossy `Blackboard` and a noisy `PoseSource` before any of this is flown
(ADR-0003) — the two seams that turn "given perfect pose and free comms" into a claim. An
abreast crossing so five or more relays make one wave. **The train wedges, and the lead's
trail is only half its failure.** Replaying the lead's crumbs through `TrailBuilder` for all
42 train runs: 4 formed, 29 were infeasible by length or had no bonus landing by the lead,
and **9 had enough relays and hovered to the end**. An instrumented re-run of one
(`relay_train_r15_n25_a1_s1`, L = 4.5 m, five launched, five needed) shows two relays frozen
in BAND with carrots 0.19 m apart along the trail, bodies held 0.48 m apart by the 0.3 m
chain repulsion, each 0.14–0.15 m from its carrot — beyond `ON_CARROT_M` = 0.12 — so the band
step that would separate them never fires and their commands are exactly zero for 500 s; a
third relay's carrot sits 0.015 m from one of those bodies, so `toward()` slides for ever. A
wedge of three constants (`ON_CARROT_M`, `CHAIN_AVOID_RANGE_M`, the FOLLOW targets
`m·L/(n_active+1)` that narrow to 0.23 m apart as relays join), not a long trail. Dispatch
runs the same code and escaped because its southern-row relays arrive ~75 ticks apart and the
band moves the earlier joiner outward before the next arrives — timing, not design. Found by
the lesson's completeness critic after this checkpoint's first commit; not fixed here. The
fix is either to let the band step off-carrot when a chain neighbour is inside `d*`, or to
space FOLLOW targets by `d*` rather than `L/(n_active+1)` while the head is still moving. The crumb network's
free-disc chords rest on ring clearance capped at 0.8 m; the venue's walls are unmeasured
(F-24). A-19 — the peer-ranging rate on stock firmware — is the number that decides whether
1 Hz at twenty-five drones is what the team gets, and it is a bench afternoon.
