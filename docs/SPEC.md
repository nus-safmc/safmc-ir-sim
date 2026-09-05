# SAFMC-SIM — Normative Specification

**Status:** v0.1 · **Applies to:** `safmc_sim` package · **Audience:** implementers and auditors.

This document is the contract. Every requirement has an ID (`R-<area>-<n>`) and is written to be
**falsifiable**: an auditor must be able to write a test that fails if the requirement is unmet.

Keywords: **MUST** = auditable failure if absent. **MUST NOT** = auditable failure if present.
**SHOULD** = deviation requires a note in `docs/FIDELITY.md`. **MAY** = optional.

Where a requirement encodes a fact about the real world (competition rules, hardware), the
source is cited as `[src: file:line]` or `[src: URL]`. Requirements marked **[ASSUMPTION]**
are choices we made in the absence of published data and are listed together in §11.

---

## 1. Scope and non-goals

**R-SCOPE-1** The system MUST simulate: rigid-body 2.5D drone motion, sparse time-of-flight
ranging, marker detection, occupancy mapping inputs, multi-agent search, and the SAFMC
Category Swarm scoring rules.

**R-SCOPE-2** The system MUST NOT simulate: aerodynamics, motor/ESC dynamics, attitude control,
battery, or any rigid-body rotation beyond yaw. Rationale: the project's premise is that the
Gazebo+PX4 stack is unnecessary for validating sensing and search policy. The target paper
(arXiv:2607.25195, IROS 2026) reached an accepted result on a strictly weaker model.

**R-SCOPE-3** Pose supplied to policies is **ground truth**. Odometry noise and drift are
explicitly out of scope for v0.1 and MUST be reachable later by adding a layer at the
`PoseSource` seam (R-SEAM-1) without changing any policy's code.

**R-SCOPE-4** Inter-agent communication is a **perfect shared blackboard**. Radio range, loss,
latency and bandwidth are out of scope for v0.1 and MUST be reachable later by adding a layer at
the `Blackboard` seam (R-SEAM-2) without changing any policy's code.

---

## 2. Frames, units, conventions

**R-FRAME-1** The canonical simulation frame is **ARENA**: `x` = East (metres), `y` = North
(metres), `z` = up (metres), `theta` = yaw, radians, **counter-clockwise positive from +x**.
Origin at the arena's south-west floor corner. This is ir-sim's native frame, so no conversion
occurs anywhere inside the simulator.

**R-FRAME-2** All angles held in state or returned to policies MUST be wrapped to `(-pi, pi]`.

**R-FRAME-3** Units are SI throughout: metres, metres/second, radians, radians/second, seconds.
No configuration value anywhere may be expressed in millimetres or degrees except at an
explicitly named conversion boundary.

**R-FRAME-4** The package MUST provide a bijection between ARENA and the flight stack's **NED map
frame** (`ned_x` = North, `ned_y` = East, `ned_z` down, `heading` = 0 at North, **clockwise
positive**) as used by the real firmware [src: esp-everything/main/wifi_task.h:32-48].
The mapping MUST be `ned_x = y`, `ned_y = x`, `ned_z = -z`, `heading = wrap_2pi(pi/2 - theta)`.

**R-FRAME-5** The NED bijection MUST be round-trip exact to within 1e-9 for position and 1e-9 rad
for heading, verified over a randomised sweep including the wrap discontinuity.

---

## 3. Time

**R-TIME-1** The simulator MUST be fixed-step. Default tick `dt = 0.05 s` (20 Hz), matching the
firmware's navigation loop rate [src: esp-everything/main/nav_task.c:510].

**R-TIME-2** There MUST be no coupling between simulated time and wall-clock time in headless
mode.

**R-TIME-3** Any component with its own rate (sensor, policy) MUST declare it as an integer
**decimation** in ticks. Supplying a rate that does not divide the tick rate exactly MUST raise
`ConfigError` at construction. No silent rounding.

**R-TIME-4** `world.sample_time < world.step_time` MUST be rejected by our config loader with a
clear error, because ir-sim raises an opaque `ZeroDivisionError` in that case
[src: irsim/world/world.py:171].

---

## 4. Determinism

**R-DET-1** A run is fully specified by `(scenario, seed, policy_id, policy_config)`. Two runs
with identical inputs MUST produce byte-identical recorded logs, excluding wall-clock timestamps
which MUST be confined to a single `meta` block.

**R-DET-2** No simulator or policy code may call the global `numpy.random` module-level API.
Every stochastic component MUST receive an explicit `numpy.random.Generator`.

**R-DET-3** Per-agent RNGs MUST be derived by `numpy.random.SeedSequence(seed).spawn(n)` so that
adding an agent does not perturb the streams of existing agents with lower indices.

**R-DET-4** ir-sim's process-global RNG MUST be seeded from the same run seed, and the code MUST
document that in-process concurrent environments with independent seeds are unsupported
[src: irsim/util/random.py:14-24]. Parallel sweeps MUST use separate processes.

---

## 5. World and arena

**R-WORLD-1** The arena model MUST be data-driven from a scenario descriptor. Hard-coding a
single fixed map is a specification violation, because the rulebook states the inner-wall
positions "will NOT be given" and the Unknown Search Area layout is "intentionally NOT shown"
[src: SAFMC 2026 Cat Swarm Challenge Booklet v2.0 §3.2].

**R-WORLD-2** The default arena MUST encode the published 2026 Category Swarm geometry:
field 20.0 m x 20.0 m; Start Area 20.0 m x 6.0 m on the south edge; Unknown Search Area
10.0 m x 10.0 m walled with four doorways, interior maze sampled per seed; perimeter wall height 1.5 m on west/north/east only;
inner wall height 2.0 m; pillars 0.30 m diameter, 2.0 m tall on a 0.50 m x 0.15 m base; minimum
wall-to-wall gap 2.0 m; minimum pillar gap 1.0 m; flight ceiling 1.4 m
[src: SAFMC 2026 Cat Swarm Challenge Booklet v2.0 §3.2, §3.3.1 r.13].

**R-WORLD-3** Inner-wall placement and the entire contents of the Unknown Search Area MUST be
randomised as a function of the run seed, subject to the published gap constraints.

**R-WORLD-4** Generated arenas MUST be validated before use: every declared minimum gap holds,
the free space is connected from the Start Area to every target, and no target lies inside an
obstacle. Violations MUST raise, not warn.

**R-WORLD-5** Every obstacle MUST carry a `height_m`. The arena MUST NOT rely on ir-sim's
`obj.z`, which is dead code that always returns 0 [src: irsim/world/object_base.py:1499-1508].

**R-WORLD-6** The arena MUST include explicit boundary walls. ir-sim has no implicit world
bounds and a robot will silently leave the world otherwise [src: verified in recon].

**R-WORLD-7** *(added with ADR-0005)* Every non-structural object a sensor may perceive MUST be
a `Landmark`: an id, a kind, a finite position, a finite footprint radius and a finite height.
A landmark with both a footprint and a height is **solid** and MUST occlude ranging through the
height gate of R-SENS-6 and MUST be collidable at altitudes below its height -- including
altitude zero, so under `collision_behaviour="stop"` a landing inside a solid landmark is a
collision, not a score (under `unobstructed` nothing collides, by definition of that mode); a
landmark without both MUST do neither. Mission targets MUST be landmarks. A placed landmark that would
be reported under a mission kind but is not a generated target MUST be refused. Validation
MUST reject a landmark outside the field, a duplicate id, and a landmark with a footprint
overlapping structure or another footprint.

**R-WORLD-8** *(added with ADR-0005)* Landmarks MUST be placeable at fixed positions through
the scenario descriptor (`ArenaConfig.landmarks`), and a resolved `ArenaSpec` carrying
landmarks MUST be usable in place of the generated one (`run(config, arena=...)`), validated
first -- every landmark invariant re-checked, whatever constructed it -- and recorded in full
with its provenance and its own arena config. The generator MUST treat a placed landmark
with a footprint as fixed: walls and pillars keep their published gaps from a body, nothing
is built over a flat mark, and targets are not placed on either. Reachability validation
(R-WORLD-4) MUST count solid landmarks as blocking, and a run MUST refuse a take-off position
inside one. A point landmark whose kind no configured sensor reports MUST be refused at run
construction.

---

## 6. Drone model

**R-WORLD-9** Arena generation MUST expose three independent RNG streams — `layout_seed` (the
Known Search Area and the room's position), `unknown_seed` (the maze and the pillars inside the
room) and `mission_seed` (victim, bonus-victim and fire placement) — each defaulting to a child
of the arena seed via `SeedSequence.spawn` (R-DET-3). Pinning one stream MUST leave the others
free, so that a result can be attributed to the factor that was varied. The maze pattern MUST be
invariant, in room-relative coordinates, to `layout_seed`: moving the room translates the maze
rather than resampling it. The seeds actually used MUST be recorded on the `ArenaSpec`, so an
arena built with overrides is regenerable and not merely replayable.

> A fixed set of maps is a *development* set. Policies overfit to it, which is the failure the
> rulebook's withholding exists to punish, so final numbers MUST be quoted from seeds that were
> never inspected.

**R-WORLD-10** An arena MUST be serialisable to, and restorable from, a standalone file
independent of any run log. The file MUST carry the geometry, the per-stream seeds and the full
`ArenaConfig` — a standalone map has no run header to fall back on, so a dropped config would let
revalidation re-check a map against gaps it was never generated under. Config fields MUST be
serialised from `dataclasses.fields`, so a knob added later cannot silently vanish. Loading MUST
validate by default, and MUST refuse a file whose schema differs or whose config carries fields
this build does not know, rather than dropping them silently.

**R-WORLD-11** *(added with ADR-0006)* The rulebook's navigation-aid rules
[src: SAFMC 2026 Cat Swarm Challenge Booklet v2.0 §3.3.1 r.14-17] MUST be enforced in two
places, because they are not all checkable from the arena alone.

Arena validation MUST refuse any placed landmark inside the Unknown Search Area, on every
run (r.17: teams may never enter it, so nothing they place can be there). That rule needs no
notion of what an aid is — nothing at all may be in the room — so it is not the caller's to
opt into.

The remaining two do need it: at most **ten** aids in the Known Search Area (r.15), and each
aid within **1 m x 1 m** (r.14 f). A `Landmark` may equally be scenery, a prop or a venue
feature, and the primitive carries no field that distinguishes them, so the package MUST
provide a check that takes the landmark kinds counting as aids **from its caller**, counts
every listed kind together, and names the offending landmarks and the rule. The runner MUST
NOT apply that check: an experiment may deliberately exceed the cap to ask what denser
coverage would be worth, and the check exists so that nobody quotes such a run without
knowing that is what it was.

**R-DRONE-1** Drone state MUST be `[x, y, theta, z, vx, vy]` where `vx, vy` are ARENA-frame
velocities carried as state so that a first-order velocity lag can be modelled.

**R-DRONE-2** The kinematics MUST be registered with ir-sim via `@register_kinematics`
[src: irsim/lib/handler/kinematics_handler.py:22-48] rather than forked or monkey-patched.

**R-DRONE-3** Yaw MUST be integrated by the kinematics handler itself and wrapped. It MUST NOT be
integrated outside `env.step()`; the reference implementation of the target paper does exactly
that and thereby skips angle wrapping and geometry refresh
[src: sdlw/run_simulation.py:311-314].

**R-DRONE-4** Commanded horizontal velocity MUST be tracked through a first-order lag with time
constant `tau`, default 0.35 s. **[ASSUMPTION]**

**R-DRONE-5** Altitude MUST be a real state. Commanded altitude changes MUST be rate-limited by
`climb_rate_max`, default 0.5 m/s **[ASSUMPTION]**, and clamped to `[0, ceiling_m]` where
`ceiling_m` defaults to 1.4 m [src: rulebook §3.3.1 r.13].

**R-DRONE-7** Default cruise speed MUST be 0.45 m/s [src: esp-everything/main/nav_task.h:24].

**R-DRONE-8** The drone collision radius MUST default to 0.18 m
[src: esp-everything/main/vfh.h:43].

**R-DRONE-9** Drone lifecycle MUST be exactly `{ACTIVE, LANDED, CRASHED}`, with both terminal
states permanent. The simulator MUST NOT model flight phases.

> Amended twice. The original set included `ARMED`, which nothing implemented. The set then
> still carried `TAKEOFF` and `LANDING`, which turned out to be worse: they were *modes the
> runner flew*, choreographing a climb and a descent at a fixed rate and deciding when each
> was complete. Climbing is a velocity command; when to stop climbing is a policy's decision.
> Removing the phases removed the choreography.

**R-DRONE-10** A `LANDED` drone MUST be permanently immobile for the remainder of the run. The
rules require a rescuing drone to remain "until the end of the mission"
[src: rulebook §3.1]. Landing is therefore an irreversible resource commitment.

---

## 7. Sensors

**R-SENS-1** *(amended in `50e2643`)* The ToF ring MUST compute all rangers in one vectorised
numpy raycast, and MUST NOT be implemented as N instances of ir-sim's `Lidar2D`. Justification:
`Lidar2D` cost is dominated by fixed per-sensor GEOS overhead, measured at 14-31x a vectorised
numpy raycaster for sparse beams [src: recon benchmark], and its `angle_list` is a contiguous fan
that cannot express a ring of separated rangers.

> The original wording required the ring to be *an ir-sim sensor object*. It is deliberately no
> longer one: registering it needed a monkeypatch of `SensorFactory.create_sensor` (ir-sim has no
> sensor registry) and dragged in a dead plotting path, a walk to the parent for altitude, and
> arithmetic to recover the tick from ir-sim's clock — all of which the runner already has. The
> performance requirement, which is the part that mattered, is unchanged and still met.
> `tests/test_audit_regressions.py::test_each_drone_carries_exactly_one_ring_owned_by_the_runner`
> asserts the current design.

**R-SENS-2** The default ring MUST reproduce the flown hardware: **8 rangers at 45 degree spacing**
covering 360 degrees, each with a 45 degree horizontal field of view sampled at **8 zones**
(5.625 degrees per zone), mount radius 0.040 m, all optical axes horizontal
[src: esp-everything/main/tof_task.h:26-36, tof_task.c:183, safmc-ros/safmc_mapping/urdf/robot.urdf].

**R-SENS-3** Range gating MUST default to `[0.05, 3.0]` m, matching `TOF_MIN_VALID_MM = 50` and
`TOF_MAX_VALID_MM = 3000` [src: esp-everything/main/tof_task.h:16-18]. The physical sensor
maximum of 4.0 m MUST be configurable separately from the firmware's gate.

**R-SENS-4** *(withdrawn in `50e2643`)* A zone with no valid return MUST report `inf`, never a
fabricated number such as the gate limit. "Nothing there" and "a surface at exactly max range"
are different facts and MUST stay distinguishable.

> The original wording also required a per-zone `status` field carrying the VL53L5CX encoding
> (`5` valid, `9` valid-weak, `255` no return)
> [src: esp-everything/main/tof_task.c:266-277]. It was dropped: the simulator only ever emitted
> `5` or `255`, and both are recoverable from the range alone (`isfinite` ⇔ `5`). It never
> modelled the firmware's *third* case — an unreliable return substituted with a hard-coded
> 0.40 m — so the field carried no information a policy could not derive. That remaining gap is
> tracked as divergence **F-3** in [FIDELITY.md](FIDELITY.md), not as a spec requirement.

**R-SENS-5** *(withdrawn in `50e2643`)* No requirement. The ring exposes `ranges_m` shaped
`(n_rangers, zones_per_ranger)` plus the matching `zone_bearings_rad`.

> The original wording required a second, "collapsed" 64-bin view indexed by absolute clockwise
> bearing, mirroring the firmware's `tof_scan_collapsed_t`
> [src: esp-everything/main/tof_task.h:99-103, tof_task.c:243-293]. It was withdrawn because the
> two are a **permutation of each other, carrying identical information**: with the flown
> geometry the 64 zones map onto the 64 bins one-to-one, so the firmware's min-pool pools
> nothing and the collapsed scan is `ranges_m` reordered. Verified numerically — the simulator's
> zone bearings match the firmware's `tof_task.c:258` formula to **0.0e+00 degrees**, and both
> tile bins 0-63 with no collisions and no gaps.
>
> A policy that wants firmware index order can reconstruct it in four lines from
> `zone_bearings_rad`; see [docs/06-sensors.md](06-sensors.md). Do **not** assume `tof.npz`
> columns are firmware bin indices — they are not.

**R-SENS-6** Raycasting MUST be **height-gated**: a ray at drone altitude `z` is occluded by an
obstacle only if `obstacle.height_m > z`. This is the operative 2.5D effect — mission markers are
at most 1.0 m tall while walls are at least 1.5 m [src: rulebook §3.3.3 r.3, §3.2].

**R-SENS-7** Ranges MUST be computed by closed-form ray-versus-primitive intersection, and MUST
agree with an independent analytic ground truth to within 1e-9 m for circles and segments.

**R-SENS-8** The sensor MUST NOT exhibit ir-sim's `Lidar2D` stale-data defect: when the sensor
origin is inside an obstacle, every zone MUST report a correct value for that tick, never the
previous tick's value [src: irsim/world/sensors/lidar2d.py:313-327, reproduced in recon].

**R-SENS-9** Optional additive Gaussian range noise MUST be drawn from the agent's own
`Generator`, applied before gating, and MUST NOT be applied to no-return zones.

**R-SENS-10** The marker detector MUST report `(marker_id, kind, range_m, bearing_rad)` for
markers within `max_range_m` (default 3.0 m **[ASSUMPTION]**), inside the camera horizontal FOV
(default 1.0 rad **[ASSUMPTION]**), and in unobstructed line of sight. Its default decimation
MUST correspond to 2 Hz, the measured AprilTag rate on the real hardware
[src: esp-everything/CLAUDE.md:54].

**R-SENS-11** No sensor may read another agent's private state or any mission ground truth other
than through its own geometric query.

> Clarified with ADR-0005. The contract enforces the *reach*: a sensor is handed geometry and
> the landmark list and nothing else (R-SENS-15). It cannot enforce the *use*: a sensor that
> returned every landmark's exact position would be within reach and outside the rule. That
> part is a review obligation -- the auditor checklist, and a FIDELITY entry for every sensor
> -- not a property the code has.

**R-SENS-12** *(added with ADR-0005)* Every sensor MUST implement the contract in
`sensors/base.py`: a frozen `SensorConfig` subclass carrying a unique `name`, a `rate_hz`
(`None` for every tick) and `build(rng)`; and a `Sensor` subclass whose
`sample(truth, world, tick)` returns an immutable reading. Readings MUST be immutable — a
frozen dataclass or a tuple — and any numpy array in one MUST be read-only with no writable
array underneath it. The runner MUST check the first reading of every sensor at build and
refuse a mutable one; a sensor that changes the shape of its reading afterwards is its
author's defect, which the runner does not re-check every tick.

**R-SENS-13** *(added with ADR-0005)* The runner MUST drive every sensor through the same path:
one instance per drone per config, built with that drone's own generator; sampled once before
the first tick and then after motion and after the collision pass whenever
`(tick + 1) % decimation == 0`, so a decimated sensor is fresh in the observations at ticks
`0, d, 2d, ...`; held, not cleared, between samples; not sampled on a drone that is terminal,
including one that became terminal this tick. Sensor names MUST be unique within a run,
case-insensitively, whether or not the config validated them; rates MUST satisfy R-TIME-3.

**R-SENS-14** *(added with ADR-0005)* `Observation.sensors` MUST hold the latest reading of
every configured sensor under its name and nothing else, and `Observation.stale_ticks` MUST
hold each reading's age in ticks. `obs.tof` and `obs.markers` are shorthands for the flown
sensors and MUST raise a descriptive error when that sensor is not configured.

**R-SENS-15** *(added with ADR-0005)* A sensor's only view of the world MUST be the
`WorldScene` handed to `sample`: the sensing scene (structure, solid landmarks, other drones'
bodies) and the landmark list. A sensor MUST NOT be handed the arena, the mission, an agent, or
the environment. Sensors sample from ground truth (`TrueState`); the pose a policy sees comes
from `PoseSource` alone (R-SEAM-1). Together those are the only two paths from truth to a
policy.

**R-SENS-16** *(added with ADR-0005)* A sensor MAY be recorded by returning fixed-shape arrays
from `record()`. The recorder MUST fix each sensor's row keys and shapes from its first
reading before any tick is recorded, MUST refuse a later row that differs -- including a
sensor that returned `None` first and rows later -- MUST accept only numeric arrays, and MUST
assemble every array before writing any file, so a failure while assembling leaves nothing on
disk. (A failure of the disk itself mid-write can still leave a partial directory.) It writes `<name>.npz` holding `ticks`, a per-agent
`sample_tick`, one stacked array per key, and any constants from `record_static()`; the header
MUST list every sensor with its config type and whether it was recorded. `record()` MUST be
pure: recording MUST NOT affect results (R-OBS-4).

**R-SENS-17** *(added with ADR-0006, amended with its DW3000 addendum)* The package MUST
provide a UWB ranging tag on the R-SENS-12 contract (`sensors/uwb.py`) that is NOT part of
`flown_sensors()`. It MUST model a **named part** -- the Qorvo DW3000
[src: DW3000 Datasheet v1.3] -- and every default MUST record whether its source measured
that part or the older DW1000, because most of the published UWB literature is DW1000 work
and the two differ [src: DW3000 Datasheet §1.2]. Its configured
kind MUST NOT be a mission kind: a tag that ranged to the markers would report every
target's true position (R-POL-3). Its reading MUST be, for every landmark of its kind in
arena order and fixed for the run: the anchor ids, the surveyed anchor positions (each at
the configured mount height), and one reported range per anchor, `inf` where no measurement
was obtained -- and nothing carrying more information than those: no bearing, no
line-of-sight or quality flag (a derived convenience such as `heard`, `isfinite(ranges_m)`,
is not a fourth channel). The true range MUST be the three-dimensional distance from the
drone's true position to the anchor. An anchor beyond `max_range_m` MUST report `inf`.
Obstruction MUST be decided against walls and pillars only, by the line-of-sight segment
test of R-MISS-2, height-gated per R-SENS-6, at the tag's altitude; solid landmarks and
other drones MUST NOT obstruct. An unobstructed range MUST be the true range plus zero-mean Gaussian noise of
`los_noise_std_m`; an obstructed range MUST be `inf` with probability
`nlos_drop_probability` and otherwise the true range plus `nlos_bias_m` plus zero-mean
Gaussian noise of `nlos_noise_std_m`; with probability `outlier_probability` a reported range
MUST gain a further positive error uniform on `[0, outlier_max_m]`. A reported range MUST NOT
be negative. Every draw MUST come from the sensor's own generator (R-DET-2), and the number
of draws per sample MUST NOT depend on the geometry, so the noise stream is a function of the
seed alone. Every default of the reach and noise model MUST be a named constant registered
in §12 (A-14..A-18); the sweep rate and the anchor height are deployment choices, named
constants outside the register. The sensor MUST record `ranges_m` shaped `(ticks, agents, anchors)` and the anchor positions as a
static array (R-SENS-16); column `j` MUST be the `j`-th landmark of the sensor's kind in the
header's landmark list, so anchor identity is recoverable from the log alone (R-OBS-3).

---

## 8. Policy interface

**R-POL-1** The authoring surface MUST be a per-tick callback: a class with
`step(self, obs: Observation) -> Command`. This is the shape the team chose.

**R-POL-2** `Observation` and `Command` MUST be frozen dataclasses. A policy MUST NOT be able to
mutate simulator state by mutating the objects it is handed.

**R-POL-3** `Observation` MUST contain only what a real drone could obtain: own pose and velocity,
lifecycle state, tick index and time, the latest reading of each configured sensor, and the
blackboard snapshot. Ground-truth world data (full obstacle list, other drones' true poses,
unfound target or landmark positions) MUST NOT be reachable from `Observation`.

> Amended with ADR-0005. The original wording named "the ToF product, marker detections",
> which was the sensor list of the day rather than a rule. Readings now arrive under
> `sensors`; the flown two keep their shorthands.
>
> Amended with ADR-0006. "Landmark positions" means positions the drone has neither measured
> nor been given. A navigation aid the team placed and surveyed is the team's own
> configuration, and the sensor that reads it MAY report the surveyed position beside the
> measurement, as a UWB tag configured with its anchor list does (R-SENS-17). What stays
> unreachable is everything the team did not put there: targets, structure, the room.

**R-POL-4** Violating R-POL-3 MUST be detectable: an auditor MUST be able to enumerate every
public attribute reachable from an `Observation` and find no reference to the environment, the
arena, the mission, a landmark, a sensor, or another agent's object. The test that does this
MUST itself be shown able to fail.

**R-POL-5** `Command` MUST be exactly two types: `Velocity` (ARENA-frame linear velocity plus a
yaw rate) and `Land`. No others.

> Amended after v0.1. The original requirement mirrored the firmware's six MAVLink helpers,
> which was the wrong thing to copy: those helpers are *guidance*, and reproducing them put a
> path follower and two proportional controllers inside the simulator, where every policy
> inherited them invisibly. The firmware remains the reference for what the drone **is** — its
> sensors, geometry, limits and frames — not for how it flies.

**R-POL-10** The simulator MUST NOT contain guidance, control or strategy. Specifically it MUST
NOT implement path following, setpoint tracking, altitude or heading hold, obstacle avoidance,
mapping, target selection, or inter-agent coordination. A commanded velocity MUST reach the
kinematics handler unmodified apart from the drone's own limits and lag.

**R-POL-11** No module under `safmc_sim/` outside `policies/` and `toolbox/` may import from
either. `toolbox` is opt-in example code and MUST NOT be imported by the framework.

**R-POL-6** Policies MUST be registered by string name only, decoupled from the kinematics model.
Re-registering a name MUST overwrite with a warning, not raise, so that notebook reloads work.

**R-POL-7** Every policy instance MUST receive its own `numpy.random.Generator` at construction
and MUST NOT source randomness elsewhere.

**R-POL-8** All agents MUST observe the same blackboard snapshot within a tick: agent `i`'s
publication MUST NOT be visible to agent `j` until the following tick, regardless of index order.

**R-POL-9** A policy raising an exception MUST abort the run with the agent id, tick and original
traceback. It MUST NOT be caught and converted into a hover command.

---

## 9. Mission and scoring

**R-MISS-1** Scoring MUST implement the 2026 Category Swarm rules
[src: SAFMC 2026 Cat Swarm Challenge Booklet v2.0 §3.4]:
regular victim +5, bonus victim +15, fire +10, each awarded at most once, requiring a **landed**
drone within **1.0 m** of the organiser-set target position **and** in line of sight.

**R-MISS-2** Line of sight MUST mean no wall or pillar intersecting the straight segment between
drone and target [src: rulebook §3.3.4 r.1]. Mission markers MUST NOT themselves block LOS.

**R-MISS-3** A victim within **2.5 m** of a fire that is not extinguished at end of run MUST score
zero [src: rulebook §3.4].

**R-MISS-4** A successful relay MUST multiply the **total** mission score by **2.0**. A relay
requires a chain of landed drones from the drone that rescued a bonus victim to a drone inside the
Start Area, where every adjacent pair is at most **1.0 m** apart and has mutual line of sight
[src: rulebook §3.3.7]. More than one relay MUST score the same as one.

**R-MISS-5** Run duration MUST default to **600 s** [src: rulebook §3.3.1 r.1].

**R-MISS-6** Take-off MUST be constrained to at most **two** waves, each wave defined as a group
whose last departure is within **10 s** of its first [src: rulebook §3.3.2]. Attempting a third
wave MUST be rejected and recorded as a rule violation.

**R-MISS-7** Fleet size MUST be configurable in `[10, 25]` and a configuration outside that range
MUST be rejected, because fewer than 10 drones forfeits the run [src: rulebook §3.3.1, Penalty #2].

**R-MISS-8** Scoring MUST be recomputable offline from the recorded log alone, and the offline
result MUST equal the online result exactly.

---

## 10. Observability

**R-OBS-1** Every run MUST emit a structured log with a declared, versioned schema. Free-text
logging is not a substitute; ir-sim provides only a loguru text wrapper and no state history
[src: irsim/env/env_logger.py].

**R-OBS-2** The log MUST contain, per tick: simulation time, and per agent the pose, velocity,
lifecycle state, issued command, and every recorded sensor's rows (R-SENS-16; the ring is
recorded whenever it is carried). It MUST contain, once per run: the resolved scenario
including every obstacle, target and landmark and whether it was generated or supplied, the
seed, the full config including the sensor suite, and package versions.

**R-OBS-3** The log MUST be sufficient to reconstruct the run without re-simulating: the visualiser
MUST read only the log.

**R-OBS-4** Recording MUST be able to be disabled for throughput, and the disabled path MUST NOT
change simulation results (verified by comparing final state with recording on and off).

**R-OBS-5** A log visualiser MUST exist that renders the arena, agent tracks, ToF returns, target
states, scoring events and per-agent timelines, and allows scrubbing through time.

**R-OBS-6** Every rule violation, collision, and scoring event MUST appear in the log as a
discrete, typed event with its tick index.

---

## 11. Seams for deferred work

**R-SEAM-1** `PoseSource` MUST be the single point at which a policy's pose is produced. v0.1 ships
`GroundTruthPose`. Adding a noisy/drifting implementation MUST require no change to any policy.

**R-SEAM-2** `Blackboard` MUST be the single point of inter-agent data exchange. v0.1 ships
`PerfectBlackboard`. Adding a lossy/range-limited implementation MUST require no change to any
policy.

**R-SEAM-3** The NED adapter (R-FRAME-4) plus the `Command` set (R-POL-5) together MUST constitute
a complete description of what a future ROS 2 / MAVLink node needs to implement. `docs/08-porting-to-ros.md`
MUST state the mapping from each `Command` type to its MAVLink message and type mask.

---

## 12. Assumptions register

Every value below is a choice made without published data. Each MUST be a named constant in one
place, not a literal scattered through the code.

| ID | Assumption | Default | Why unresolved |
|----|-----------|---------|----------------|
| A-1 | Wall thickness | 0.10 m | Not published by SAFMC in any 2026 document. |
| A-2 | Velocity lag time constant `tau` | 0.35 s | No step-response data from the airframe. |
| A-3 | Climb rate limit | 0.5 m/s | Not in firmware; PX4 default territory. |
| A-4 | Marker detection range | 3.0 m | Firmware comment claims ~1 m for a screen-displayed tag; no measured data. |
| A-5 | Camera horizontal FOV for detection | 1.0 rad | Derived from QVGA intrinsics `fx=163.5`, not measured. |
| A-7 | Unknown Search Area doorway count/width | 4 doorways, 2.4 m | The sec 3.2 diagram draws one opening per face at 2.40-2.83 m, and 3.3.9 r.1 routes entry through "the open doorways shown in the diagram". Previously 2 x 1.0 m, which had no source and serialised entry. |
| A-8 | ToF ring sampled synchronously at tick rate | 20 Hz, no skew | Hardware is 15 Hz round-robin with up to 64 ms skew across the ring. |
| A-9 | Maze corridor width in the Unknown Search Area | 2.0 m floor, giving a 4x4 grid at 2.40 m | Sec 3.2 states the layout there is "intentionally NOT shown", so a distribution is sampled. The published 2 m gap caps the grid at 4x4. |
| A-10 | Marker census 4/4/4 treated as known | 4 victims, 4 bonus, 4 fires every seed | The rulebook publishes the counts but says the placements are unknown; the generator reproduces the counts exactly, which a real team could not rely on. |
| A-14 | DW3000 line-of-sight ranging noise | 0.05 m std | Bracketed by the datasheet's 1.5 cm (Table 14, calibrated, at -85 dBm) and two independent measurements of the part at 5.7-6 cm; ~30% optimistic against the latter (F-30). Not measured on the team's kit. |
| A-15 | DW3000 maximum range | 20 m | A firmware setting more than a chip limit: 20 m stock at 6.8 Mb/s, 40-50 m typical, past 90 m indoors at 850 kb/s with a long preamble (F-29). At 20 m three Start Area anchors reach every point of the field; at 12 m the far third hears none. |
| A-16 | DW3000 through-wall bias and spread | +0.15 m, 0.40 m std | No DW3000 study publishes a bias in this form; the aggregate is 46.7 cm mean absolute error (Ember et al. 2024). The spread is still a DW1000 number (TELFOR 2017, Table 1) — the model's weakest joint, F-28. |
| A-17 | DW3000 through-wall dropout probability | 0.10 | No published rate for either part. Secure 802.15.4z ranging drops far more. |
| A-18 | DW3000 outlier probability and size | 0 (off), up to 1.5 m | The heavy positive tail is documented, its frequency is not; off until measured, and it is the gap in F-30. |
| ~~A-6~~ | ~~Known Search Area depth~~ | 14.0 m | **Retired — not an assumption.** The booklet v1 and v2 Play Field Element tables are character-identical and neither contains a Known Search Area row, so the figure was never published and never withdrawn. It is forced by 20 - 6. |

*A-11 to A-13 are unused: they were briefly held by the UWB assumptions on an unmerged branch, which renumbered to A-14..A-18 when PR #5 took A-9 and A-10 first. An id is never reused.*

**R-ASSUME-1** `docs/FIDELITY.md` MUST list every entry in this table together with what would be
needed to resolve it, and MUST list every known divergence between the simulator and the real
system.

---

## 13. Auditor checklist

An auditor SHOULD, for each requirement: locate the implementing code, locate the test that would
fail if it were violated, and attempt one falsification. Report per requirement one of
`SATISFIED` (code + passing test found), `UNTESTED` (code found, no test that would catch a
regression), `VIOLATED` (falsified, with the reproduction), `N/A`.
