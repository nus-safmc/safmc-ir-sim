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
10.0 m x 10.0 m walled with doorways; perimeter wall height 1.5 m on west/north/east only;
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

---

## 6. Drone model

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

**R-DRONE-6** Default cruise altitude MUST be 0.5 m, matching firmware `CRUISE_ALT_M`
[src: esp-everything/main/wifi_task.c:29].

**R-DRONE-7** Default cruise speed MUST be 0.45 m/s [src: esp-everything/main/nav_task.h:24].

**R-DRONE-8** The drone collision radius MUST default to 0.18 m
[src: esp-everything/main/vfh.h:43].

**R-DRONE-9** Drone lifecycle MUST be an explicit state machine over
`{IDLE, TAKEOFF, FLYING, LANDING, LANDED, CRASHED}`. Transitions MUST be total: an unhandled
lifecycle value raises rather than falling through to a default.

> Amended after the v0.1 audit. The original set included `ARMED`. There is no command that
> arms without taking off -- the firmware's arm, offboard-mode and climb sequence is atomic
> from a policy's point of view -- so `ARMED` was a state nothing could observe or act on. It
> was specified and never implemented, which the audit correctly flagged. Removed rather than
> added, because adding it would have created a state with no transitions in or out that a
> policy could distinguish.

**R-DRONE-10** A `LANDED` drone MUST be permanently immobile for the remainder of the run. The
rules require a rescuing drone to remain "until the end of the mission"
[src: rulebook §3.1]. Landing is therefore an irreversible resource commitment.

---

## 7. Sensors

**R-SENS-1** The ToF ring MUST be a **single** ir-sim sensor object per drone that computes all
rangers in one vectorised numpy raycast. It MUST NOT be implemented as N instances of ir-sim's
`Lidar2D`. Justification: `Lidar2D` cost is dominated by fixed per-sensor GEOS overhead, measured
at 14-31x a vectorised numpy raycaster for sparse beams [src: recon benchmark], and its
`angle_list` is a contiguous fan that cannot express a ring of separated rangers.

**R-SENS-2** The default ring MUST reproduce the flown hardware: **8 rangers at 45 degree spacing**
covering 360 degrees, each with a 45 degree horizontal field of view sampled at **8 zones**
(5.625 degrees per zone), mount radius 0.040 m, all optical axes horizontal
[src: esp-everything/main/tof_task.h:26-36, tof_task.c:183, safmc-ros/safmc_mapping/urdf/robot.urdf].

**R-SENS-3** Range gating MUST default to `[0.05, 3.0]` m, matching `TOF_MIN_VALID_MM = 50` and
`TOF_MAX_VALID_MM = 3000` [src: esp-everything/main/tof_task.h:16-18]. The physical sensor
maximum of 4.0 m MUST be configurable separately from the firmware's gate.

**R-SENS-4** Each zone MUST report `(range_m, status)` where status uses the VL53L5CX encoding
the firmware relies on: `5` = valid, `9` = valid-weak, `255` = no return
[src: esp-everything/main/tof_task.c:266-277]. A zone beyond the gate MUST report status `255`
and range `inf`, never a fabricated number.

**R-SENS-5** The sensor MUST also expose the firmware's derived product: a **64-bin collapsed
scan**, index 0 straight ahead, clockwise, 5.625 degrees per bin, min-pooled, `inf` for empty
bins. This is the only form the real navigation stack consumes
[src: esp-everything/main/tof_task.h:99-103, tof_task.c:243-293].

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

---

## 8. Policy interface

**R-POL-1** The authoring surface MUST be a per-tick callback: a class with
`step(self, obs: Observation) -> Command`. This is the shape the team chose.

**R-POL-2** `Observation` and `Command` MUST be frozen dataclasses. A policy MUST NOT be able to
mutate simulator state by mutating the objects it is handed.

**R-POL-3** `Observation` MUST contain only what a real drone could obtain: own pose and velocity,
lifecycle state, tick index and time, the ToF product, marker detections, and the blackboard
snapshot. Ground-truth world data (full obstacle list, other drones' true poses, unfound target
positions) MUST NOT be reachable from `Observation`.

**R-POL-4** Violating R-POL-3 MUST be detectable: an auditor MUST be able to enumerate every
public attribute reachable from an `Observation` and find no reference to the environment, the
arena, the mission, or another agent's object.

**R-POL-5** `Command` MUST cover exactly the real firmware's action set
[src: esp-everything/main/mavlink_task.h:73-89]: `VelocityBody`, `VelocityWorld`, `PositionWorld`,
`Hold`, `Land`, and `Takeoff`. No other command types.

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
lifecycle state, issued command, and the collapsed ToF scan. It MUST contain, once per run: the
resolved scenario including every obstacle and target, the seed, the full config, and package
versions.

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
| A-6 | Known Search Area depth | 14.0 m | Published in 2025, withdrawn from the 2026 table; derived as 20 - 6. |
| A-7 | Unknown Search Area doorway count/width | 2 doorways, 1.0 m | Rulebook shows gaps in a not-to-scale diagram. |
| A-8 | ToF ring sampled synchronously at tick rate | 20 Hz, no skew | Hardware is 15 Hz round-robin with up to 64 ms skew across the ring. |

**R-ASSUME-1** `docs/FIDELITY.md` MUST list every entry in this table together with what would be
needed to resolve it, and MUST list every known divergence between the simulator and the real
system.

---

## 13. Auditor checklist

An auditor SHOULD, for each requirement: locate the implementing code, locate the test that would
fail if it were violated, and attempt one falsification. Report per requirement one of
`SATISFIED` (code + passing test found), `UNTESTED` (code found, no test that would catch a
regression), `VIOLATED` (falsified, with the reproduction), `N/A`.
