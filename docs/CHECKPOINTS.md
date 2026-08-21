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
