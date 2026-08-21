# The hardware we are mirroring

Reconstructed by reading all nine `nus-safmc` repositories, including the private ones. The
authoritative repo is **`esp-everything`**, HEAD `99cde05` ("Competition over", 2026-04-02) — the
firmware actually flown at SAFMC 2026.

Architecture, from `esp-everything/CLAUDE.md:7`:

> "An ESP32-S3 runs onboard each drone as a companion computer, communicating with PX4 via MAVLink
> over UART. A laptop-side Python application coordinates the fleet over WiFi/UDP."

## The airframe

| Part | What |
|---|---|
| Onboard computer | Seeed XIAO ESP32-S3 Sense, IDF 5.5.1, octal PSRAM |
| Flight controller | STM32F765IIT6 running PX4, MAVLink v2 over UART at 921600 baud |
| Ranging | **8 x ST VL53L5CX** multizone ToF behind a TCA9548A I2C mux |
| Camera | OV2640, grayscale QVGA 320x240, pitched **45 degrees nose-down**, 0.02 m forward |
| Optical flow / downward range | **Not on the ESP32.** Listed as "Potential Expansion" in `pcb_hardware`. Height comes from PX4's `LOCAL_POSITION_NED.z` |
| Collision radius used by nav | 0.18 m (`vfh.h:43`) |

The PX4 forks contain **no SAFMC customisation** worth porting. Across every branch the only
team-authored commits add two Gazebo SITL airframes whose entire parameter delta is
`SYS_HAS_GPS=0`, `SIM_GPS_USED=0`, `EKF2_GPS_CTRL=0` — i.e. GPS-denied indoor flight on optical
flow plus a downward rangefinder. No custom module, no controller change, no custom uORB topic.

## The ToF ring — the sensor that matters

This is the geometry [R-SENS-2](SPEC.md) reproduces.

**Layout** (`tof_task.h:26-36`, `tof_task.c:183`): 8 sensors mounted counter-clockwise, 45 degrees
apart, full 360 degree coverage with no gaps and no overlap.

```c
SENSOR_ANGLES[i] = (float)(((TOF_FRONT_SENSOR_IDX - i) * 45 % 360 + 360) % 360);
#define TOF_SENSOR_HALF_WIDTH_DEG 22.5f
```

**Mounting** (from the gen-1 URDF, `safmc-ros/safmc_mapping/urdf/robot.urdf`): the four cardinal
sensors sit on a **40 mm** radius, the four diagonals at 24.04 mm per axis (≈34 mm radius — a
rectangular PCB). **Every optical axis is horizontal**; no sensor is pitched.

**Per sensor**: VL53L5CX at **8x8 = 64 zones**, one target per zone, 45x45 degree square FoV.

**Timing** (`tof_task.h:16-18`, `tof_task.c:475`): round-robin, one sensor per 8 ms tick.
8 x 8 ms = 64 ms per full cycle, so **each sensor refreshes at ~15 Hz and the ring is skewed by up
to 64 ms across sensors**. It is never a synchronous snapshot. Assumption A-8 records that v0.1
ignores this skew.

**Gating**: `TOF_MIN_VALID_MM = 50`, `TOF_MAX_VALID_MM = 3000` — note the firmware gates at 3 m
even though the sensor reaches 4 m. All 8 sensors must initialise or the drone refuses to arm.

### What the firmware actually keeps

The driver offers per zone: `distance_mm`, `target_status`, `range_sigma_mm`, `ambient_per_spad`,
`reflectance`, `signal_per_spad`, `nb_spads_enabled`. **The firmware copies only `distance_mm` and
`target_status`** (`tof_task.c:461-466`) and discards the rest.

So the minimum a simulator must produce per zone is `(distance_mm, target_status)`. Status
semantics the firmware relies on (`tof_task.c:266-277`):

| status | meaning | firmware behaviour |
|---|---|---|
| 255 | out of range / nothing there | treated as `INFINITY` — free space |
| 5 | valid | use `distance_mm`, gated to [50, 3000] mm |
| 9 | valid, weak signal | same as 5 |
| anything else | unreliable | **substituted with a hard-coded 0.40 m** — "assume obstacle" |

### The 64-bin collapsed scan — the only form nav consumes

`tof_get_collapsed_scan()` reduces 8 sensors x 64 zones to `float ranges[64]`:

- **Only rows 4-7 are used** (`tof_task.c:261`) — "sensors mounted upside-down". Half the vertical
  FoV is discarded; the effective vertical slice is ~22.5 degrees of the 45.
- Column to angle: `angle_deg = SENSOR_ANGLES[s] + (3.5 - col) * 5.625`.
- Binned into **64 bins of 5.625 degrees over 360 degrees, min-pooled**, index 0 = straight ahead,
  clockwise positive, `INFINITY` for empty bins.

Downstream, `vfh.c:54-62` halves this again into a **32-bin VFH histogram** of 11.25 degrees.

[R-SENS-5](SPEC.md) requires we emit this exact product, because it is the only interface the real
navigation stack has ever consumed.

## The action interface — what a policy may command

Everything goes out as `SET_POSITION_TARGET_LOCAL_NED` in `MAV_FRAME_LOCAL_NED` with different
type masks (`mavlink_task.c:43-56`). Public API (`mavlink_task.h:73-89`):

```c
void mavlink_set_velocity_ned(float vx, float vy, float vz, float yaw_rate);
void mavlink_set_velocity_xy_position_z(float vx, float vy, float z, float yaw);  // the workhorse
void mavlink_set_position_ned(float x, float y, float z, float yaw);
void mavlink_set_hold(void);
```

plus `MAV_CMD_COMPONENT_ARM_DISARM`, `MAV_CMD_DO_SET_MODE` (offboard), `MAV_CMD_NAV_LAND`.

This set — and nothing else — is what [R-POL-5](SPEC.md) mirrors.

Rates and watchdogs worth copying:
- MAVLink task runs at **20 Hz**; the offboard setpoint must be refreshed every 50 ms or PX4 drops
  offboard mode after ~500 ms.
- `SP_STALE_TIMEOUT_MS = 300`: a stale velocity setpoint auto-reverts to position hold.
- Cruise 0.45 m/s (`nav_task.h:24`), ramped to 30% within 1.5 m of an obstacle; yaw tolerance
  0.10 rad; arrival radius 0.25 m; cruise altitude 0.5 m.

### What the drone actually knows about itself

`mavlink_task.c:316-367` parses exactly **three** message IDs from PX4 and drops everything else:
`LOCAL_POSITION_NED` (x,y,z,vx,vy,vz), `ATTITUDE` (yaw only), `HEARTBEAT` (armed + mode).

**No IMU, no attitude rates, no battery, no GPS, no distance sensor is read.** The observation
available to a real policy is therefore `x, y, z, vx, vy, vz, heading, armed, mode` — which is
what [R-POL-3](SPEC.md) restricts `Observation` to.

## Communications — and why we are not simulating it

**There is no ESP-NOW and no drone-to-drone radio link at all.** Every drone is a WiFi station on
one AP; all inter-drone information is relayed through the laptop.

| Direction | Port | Size | Rate |
|---|---|---|---|
| Telemetry, drone to laptop | 5005 | 55 B | 10 Hz |
| Commands, laptop to drone | 5006 | 22 B | event-driven |
| Peer positions, laptop to drone | 5006 | 3 + 8N B | 5 Hz |
| Nav tags, laptop to drone | 5006 | 11 + 9N B | every 20 s |
| ToF debug (front sensor only) | 5007 | 199 B | 10 Hz |

Serialization is **raw packed C structs, little-endian** — no protobuf, no CBOR, no JSON. The
laptop mirrors them with Python `struct` formats: telemetry is `"<BBfffBbf32sBH"`, command is
`"<BBff12b"`.

**There are no sequence numbers, no acknowledgements, and no retransmission anywhere.** Reliability
is periodic re-send only. Drone RX buffer is 256 B, which caps any command.

The most interesting failure mode is the killswitch (`nav_task.c:480-501`): link down triggers
immediate hold; link down for >3 s while armed **disarms the motors**.

> The team's decision for v0.1 is a **perfect shared blackboard** — no radio model. That is
> defensible precisely because the real topology is star-through-a-laptop over WiFi, not a mesh,
> and because the laptop already has every drone's position. It is recorded as a deferred layer at
> the [`Blackboard` seam](SPEC.md#11-seams-for-deferred-work), not as a claim that comms are free.

## Marker detection

Family **tag16h5**, tag size 0.12 m, intrinsics `fx=163.5 fy=153.2 cx=154.0 cy=107.1`. The ESP
produces a **full 6-DoF pose**, gated by `hamming <= 1`, `decision_margin > 55`, pose error < 0.5.
Nominal loop 10 Hz but `CLAUDE.md:54` estimates **~2 Hz** in practice — the detector task runs at
the lowest priority in the system. Hence the 2 Hz default in [R-SENS-10](SPEC.md).

**Detection range and pose accuracy are UNVERIFIED.** No test data exists in any repo. The only
claim is an inherited upstream comment about "about 1 meter (tested with tag on screen, not on
paper)" that describes *different* detector parameters than the ones actually set. Assumptions A-4
and A-5 cover this; measuring it is a cheap, high-value experiment for the team.

## State estimation

**There is no EKF, UKF, particle filter or complementary filter anywhere in the four firmware
repos.** All 6-DoF estimation is delegated to PX4's EKF2, which these repos never configure.

What exists on the ESP is a **translation-only frame corrector** (`odom.c:118-148`): when a
surveyed nav tag is seen, the drift between inferred and reported odometry is computed and the
`map_T_odom` offset is **hard-overwritten** — zero filtering, no gain, no covariance, no outlier
rejection beyond the pose-error gate. Z is never corrected.

> This is the single largest gap between the real system and any claim about robustness, and it is
> exactly what the deferred `PoseSource` layer is for.

## Documentation that is stale — verified

`esp-everything/CLAUDE.md` and `laptop/setup.yaml` disagree with the code in several places. Trust
the code:

| Claim | Where | Reality |
|---|---|---|
| `COLLISION_DANGER_M` 0.37 / `CLEAR` 0.47 | CLAUDE.md:91 | `nav_task.c:122-123` → **0.40 / 0.50** |
| `NAV_CRUISE_SPEED_MS` 0.5 | CLAUDE.md:89 | `nav_task.h:24` → **0.45** |
| drone sends `VISION_POSITION_ESTIMATE` | setup.yaml:9-10 | no such call exists |
| command packet is 18 bytes | protocol.py:8 | it is **22** |
| `TOF_MAX_VALID_MM` = 4000 | laptop/tof_debug.py:29 | firmware uses **3000** |

Two genuine bugs found in the older `esp-mapper` generation, both fixed in current firmware but
worth knowing: the AprilTag `memcpy` copied `matd_t` struct headers rather than pose values, and
the ROS-side parser reads `R` before `t` while the C struct declares `t` first.

Also flagged: the vendored `tag16h5.c` truncates `codedata` to 12 entries while still setting
`ncodes = 30` — an out-of-bounds read for IDs 12-29, which is exactly the range `setup.yaml`
assigns to nav tags. **Worth checking on real hardware.**
