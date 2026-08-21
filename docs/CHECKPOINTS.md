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
