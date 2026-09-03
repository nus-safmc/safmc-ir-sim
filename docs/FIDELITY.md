# Fidelity ledger

Every known divergence between this simulator and the real system, and every value we picked
without published data. Required by [R-ASSUME-1](SPEC.md).

**Read this before quoting a simulator result at anyone.**

## 1. Deliberately not simulated

| Thing | Status | Why | Cost to add |
|---|---|---|---|
| Rigid-body dynamics, motors, attitude | Never | The project premise. arXiv:2607.25195 reached IROS on strictly less | Out of scope |
| Pose error, drift, estimator | **Deferred** | Team decision; ground truth in v0.1 | One class behind `PoseSource` |
| Radio range, loss, latency, bandwidth | **Deferred** | Team decision; perfect blackboard | One class behind `Blackboard` |
| Battery / flight-time limit | Not modelled | No discharge data. Real limit likely binds inside 600 s | Needs bench data |
| Wind, ground effect, prop wash | Never | Indoor, and irrelevant to search strategy | Out of scope |
| Camera imagery | Never | Marker detection is modelled geometrically, not visually | Out of scope |

The first two are the ones that matter. A v0.1 result is *"policy A beats policy B given perfect
state and free communication"* — nothing stronger. See
[ADR-0003](adr/0003-ground-truth-pose-and-perfect-comms.md).

## 2. Simulated, but simplified

| # | Real system | Simulator | Impact |
|---|---|---|---|
| F-1 | ToF ring is round-robin: one sensor per 8 ms, ~15 Hz each, **up to 64 ms skew across the ring** | All 8 rangers sampled synchronously at tick rate | Understates motion blur and inter-ranger inconsistency while turning. Matters for scan-matching, not for reactive avoidance |
| F-2 | Firmware discards ToF rows 0-3, keeping only rows 4-7 (~22.5° of the 45° vertical FoV) | Single horizontal slice per zone | We model the horizontal geometry the firmware actually uses; vertical structure is absent by construction in a 2.5D sim |
| F-3 | Unreliable ToF returns are substituted with a hard-coded 0.40 m "assume obstacle" (`tof_task.c`) | **Not modelled at all.** Our sensor reports only a range — finite (a valid return) or `inf` (no return). It has no notion of the firmware's third, "unreliable" case, so the substitution never fires | The real drone treats a bad reading as an obstacle 0.40 m ahead. A policy tuned here will meet phantom obstacles near glass and dark surfaces that this simulator never produces. `TOF_UNRELIABLE_SUBSTITUTE_M` exists as a named constant but is currently unused |
| F-4 | VL53L5CX has per-zone `range_sigma_mm`, `reflectance`, `ambient_per_spad` | Only `(distance_mm, target_status)` | Matches the firmware, which discards the rest anyway |
| F-5 | Marker detection is a real AprilTag detector at ~2 Hz with `hamming<=1`, `decision_margin>55` | Geometric range + FOV + LOS test | No false positives, no missed detections at oblique angles, no motion blur |
| F-6 | Camera is pitched 45° nose-down | Detector modelled as horizontal | Real camera sees the floor ahead, not the horizon; ground markers enter view differently |
| F-7 | PX4 tracks setpoints with real closed-loop dynamics | First-order velocity lag, `tau` = 0.35 s | No overshoot, no attitude-induced translation, no tracking error under aggressive commands |
| F-8 | Drones are 3D bodies with props | Circles of radius 0.18 m | Matches the radius the real VFH planner uses |
| F-9 | Landing takes time and can fail | Instantaneous and always succeeds (F-16); a landing inside a solid landmark is a crash | Overstates landing reliability, which directly inflates score |
| F-10 | Link loss disarms motors after 3 s | Not modelled (no comms model) | Removes a real failure mode |

## 3. Assumptions — values chosen without published data

Mirrors [SPEC §12](SPEC.md#12-assumptions-register). Each is a named constant in one place.

| ID | Assumption | Default | How to resolve |
|---|---|---|---|
| A-1 | Wall thickness | 0.10 m | Measure at the venue, or ask SAFMC. Affects LOS through gaps and corridor width |
| A-2 | Velocity lag `tau` | 0.35 s | One step-response flight log: command 0 → 0.45 m/s, fit the rise |
| A-3 | Climb rate limit | 0.5 m/s | Read from the PX4 parameter set actually flown |
| A-4 | Marker detection range | 3.0 m | **Cheap and high value.** Fly at a tag, log detections vs distance. Firmware's only claim is an inherited comment about ~1 m for a screen-displayed tag |
| A-5 | Camera horizontal FOV for detection | 1.0 rad | Derivable from `fx = 163.5` and 320 px width; should be checked against real detections |
| A-6 | Known Search Area depth | 14.0 m | Published in 2025, withdrawn in 2026. Derived as 20 − 6 |
| A-7 | Unknown Search Area doorways | 2, each 1.0 m | Rulebook shows gaps in a not-to-scale diagram |
| A-8 | ToF ring synchronous at tick rate | 20 Hz, zero skew | Same as F-1 |

**A-4 is the one to measure first.** Detection range sets how much area a drone sweeps per metre
flown, which is the dominant term in every search-policy comparison this simulator will produce.
Getting it wrong by 2x changes which policy wins.

## 4. Inherited from ir-sim

| Thing | Consequence |
|---|---|
| Process-global RNG | Two in-process environments cannot have independent seeded streams. Parallel sweeps use separate processes (R-DET-4) |
| Unconditional figure creation | Figures leak per environment even headless; episodic loops must `plt.close('all')` |
| Shapely is strictly 2D | All height reasoning is ours, via explicit `height_m` gating in our raycaster |
| `env.done()` conflates arrival and collision | We never use it; termination is our own predicate |

## 5. Things we know are wrong elsewhere, and did not inherit

Found during recon; recorded so nobody reintroduces them.

- **The team's `gazebo_environment.md` encodes the 2025 arena** — 8x8 m Unknown Area and "Danger
  Zones" that no longer exist in the 2026 ruleset. Re-derived here from the 2026 booklet.
- **`Lidar2D` freezes its scan** when the sensor origin is inside an obstacle. Our raycaster has no
  such path, and R-SENS-8 tests for it.
- **Nano_Swarm_Mapping's Python reprojection advances its bearing accumulator inside a `continue`**,
  so every column after an invalid one is mis-bearinged by up to 39°. Do not use
  `evaluation/log.py:extract_points` as a golden reference; port `scan.c` instead.
- **The target paper's `openness` metric behaves opposite to its stated intent**, and its collided
  agents are permanently frozen, which confounds its headline coverage gain. Detailed in
  [related work](09-related-work.md).

## 6. Added during implementation

| # | Divergence | Impact |
|---|---|---|
| F-11 | Mission markers are excluded from ir-sim's obstacle list and get a height-gated collision check of our own instead | Correct at cruise altitude; means marker collision is decided by our code, not shapely |
| F-12 | The south field edge is netting in the rules but is modelled as a solid boundary at net height | Drones cannot leave the field, which is required since ir-sim has no world bounds. A drone that would have flown out is stopped rather than lost |
| F-13 | *Superseded by F-16.* An earlier runner flew a fixed descent at the climb-rate limit; landing is now instantaneous | — |
| F-14 | *Superseded by F-7 and F-17.* An earlier runner held yaw and altitude with proportional controllers; it now contains no controller, and a commanded velocity reaches the kinematics unmodified | — |
| F-15 | `collision_behaviour="stop"` freezes a drone permanently on any contact | Faithful to "no mid-run repair", but harsh: a graze is fatal. Use `unobstructed` as the control when comparing search strategies |
| F-16 | `Land()` settles the drone to the floor in the tick it is issued | The descent is not modelled. A policy that wants a realistic approach can fly it with `Velocity(vz=...)` and issue `Land` at the bottom, but the commitment itself is instantaneous |
| F-17 | No flight-phase model at all: no arming, no take-off sequence, no altitude hold | Deliberate. Those are guidance, and guidance belongs to the policy. It means a policy must climb before it can fly, and must hold its own altitude |
| F-18 | Each ToF zone is a 5.625 x 5.625 degree **cone** on the real sensor | Cast as an infinitely thin ray | At the 3 m gate a real zone spans about +/-15 cm, so a thin ray can slip past a pillar edge the hardware would have caught. Makes the simulated sensor slightly *worse* at spotting thin obstacles than the real one, which is the safe direction to be wrong |
| F-19 | `TOF_RATE_HZ` (15 Hz) is recorded but not used | The ring samples synchronously at the tick rate | Sensor physics we wrote down and did not implement; F-1 is its consequence. The sensor's FoV *is* now used — it derives the zone width — but only as an angle, not as a beam width (F-18) |
| F-20 | The drone's airframe fits a 30 cm cube (`DRONE_BBOX_M`, the competition limit) | Collision uses `DRONE_RADIUS_M` = 0.18 m, i.e. a 36 cm diameter disc | That figure is the real VFH planner's *safety* radius, not the airframe. The simulated drone is about 20% wider than the real one -- conservative, so crashes are over- rather than under-reported |
| F-21 | A drone between the camera and a tag hides the tag | The marker camera's occlusion test uses structure and solid landmarks only; other drones do not occlude it | A marker behind a teammate is reported as seen. Pre-existing behaviour, now stated. The ring *is* occluded by drones, so the two sensors disagree about teammates |
| F-22 | — | `examples/03_custom_sensor.py` ships a range-only beacon sensor | A **template for the sensor contract, not a model of any hardware**: uncalibrated, invented noise, no wall bias, and the airframe carries no UWB. Any sensor added on the contract owes this ledger an entry before its readings are quoted |
