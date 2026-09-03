# Porting to ROS 2 / DDS

The point of this document is that the port should be **an adapter, not a rewrite**. A policy
developed here should run on the real stack without its author editing it.

That is achievable because the interface is deliberately narrow: a policy touches exactly two
types, `Observation` and `Command`, and both were shaped by reading what the flown firmware
actually sends and receives rather than by what was convenient to simulate.

## The seam

```
    your Policy.step(obs) -> cmd          <-- unchanged
              |
    +---------+---------+
    |                   |
  simulator          ROS 2 node          <-- the only thing that gets written
    |                   |
  ir-sim            MAVLink / uXRCE-DDS
```

Everything a port needs is: fill an `Observation` from subscriptions, and turn a `Command` into
a publication. Both mappings are below and both are total.

## Frames

The simulator's ARENA frame is `x` East, `y` North, `z` up, `theta` counter-clockwise from
`+x`. The flight stack's map frame is NED: `x` North, `y` East, `z` down, heading clockwise
from North.

`safmc_sim.frames` is the only place that converts, and the conversion is tested exact to
1e-9 including across the wrap discontinuity:

```python
from safmc_sim.frames import arena_to_ned, ned_to_arena
ned_x, ned_y, ned_z, heading = arena_to_ned(x, y, z, theta)
x, y, z, theta = ned_to_arena(ned_x, ned_y, ned_z, heading)
```

Concretely: `ned_x = y`, `ned_y = x`, `ned_z = -z`, `heading = wrap_2pi(pi/2 - theta)`.

Do not scatter this. One tested adapter is the difference between a port and a haunting.

## Commands to MAVLink

Everything goes out as `SET_POSITION_TARGET_LOCAL_NED` in `MAV_FRAME_LOCAL_NED` with a
different type mask, exactly as `esp-everything/main/mavlink_task.c:43-56` does.

| `Command` | Firmware call | Type mask |
|---|---|---|
| `Velocity(vx, vy, vz, yaw_rate)` | `mavlink_set_velocity_ned`, after converting ARENA → NED: swap x and y, negate z, and the yaw rate flips sign because NED heading is clockwise | `0x5C7` |
| `Land()` | `MAV_CMD_NAV_LAND` | — |

Two commands, because that is the whole action space (R-POL-5). Arming and offboard mode are
the node's business before the first `Velocity`, not a command a policy sends. The firmware's
other helpers — `set_velocity_xy_position_z`, `set_position_ned`, `set_hold` — are guidance,
and guidance lives in the policy; an earlier version mirrored them and was removed at C7.

Four details from the firmware that a naive port gets wrong:

1. **Unused fields must be `NaN`, not zero.** The firmware comments this explicitly: a yaw
   field of `0.0` commands "face North", which is a PX4 behaviour and almost never intended.
   A `Velocity` fills only the velocity and yaw-rate fields; every position and yaw field in
   the setpoint must be `NaN`.
2. **The offboard setpoint must be refreshed at 20 Hz.** PX4 drops offboard mode if it stops
   arriving for ~500 ms. Our tick rate is 20 Hz for exactly this reason.
3. **A stale velocity setpoint auto-reverts to position hold after 300 ms**
   (`SP_STALE_TIMEOUT_MS`). A policy that stops commanding does not coast.
4. **Arm and mode are retried every 500 ms until the heartbeat confirms.** There is no
   `COMMAND_ACK` handling; confirmation is via `HEARTBEAT` state only.

## Observation from subscriptions

`mavlink_task.c:316-367` parses exactly three message IDs and drops everything else. That is
the real information budget, and `Observation` was built to match it.

| `Observation` field | Source on the real drone |
|---|---|
| `pose.x`, `pose.y`, `pose.z` | `LOCAL_POSITION_NED` (convert NED → ARENA) |
| `pose.theta` | `ATTITUDE.yaw` (convert heading → ARENA yaw) |
| `velocity_xy` | `LOCAL_POSITION_NED.vx/vy` |
| `lifecycle` | `HEARTBEAT` armed flag + `custom_main_mode`, plus your own state machine |
| `sensors["tof"]` (`obs.tof`) | `tof_get_collapsed_scan()` — the same 64 values, but indexed by clockwise bearing; reorder into `(ranger, zone)` through `zone_bearings_rad`, see below |
| `sensors["markers"]` (`obs.markers`) | AprilTag detections, id + pose, converted to range and bearing |
| `sensors[<yours>]` | One subscription per sensor you added; its reading dataclass is the message you must fill |
| `peers` | Whatever your `Blackboard` implementation is backed by |
| `arena` | Static configuration |

**No IMU, no attitude rates, no battery, no GPS and no distance sensor are read by the real
firmware.** If your policy wants one of those, the port is bigger than an adapter and you
should know that before you write the policy, not after.

## The blackboard on real hardware

`Blackboard` has three methods: `snapshot(reader_id)`, `publish(agent_id, values)`, `commit()`.

The real topology is not a mesh. Every drone is a WiFi station talking to one laptop, which
already holds every drone's position and re-broadcasts peer lists at 5 Hz. There is **no
drone-to-drone radio at all** — a grep for `esp_now` across all four firmware repos finds only
unused autogenerated config. So a faithful implementation is a ROS 2 node that mirrors the
laptop coordinator: subscribe to peer telemetry, publish your own, and expose the result as a
snapshot.

Constraints worth honouring, from the wire format:

- Telemetry is **55 bytes at 10 Hz**; commands are **22 bytes**; the drone's receive buffer is
  **256 bytes**, which hard-caps any message.
- **No sequence numbers, no acknowledgements, no retransmission.** Reliability is periodic
  re-send only.
- Link loss triggers hold immediately and **disarms after 3 seconds**.

None of that is simulated in v0.1 (ADR-0003). A policy that broadcasts an occupancy grid every
tick will work here and fail there. Design against the budget above.

## What a port does not have to reimplement

- **Frames** — one tested function each way.
- **The ToF product** — the same 64 values as `tof_scan_collapsed_t`, and the same `inf`
  semantics, but **not in the same order**. `ranges_m` is `(ranger, zone)`, anticlockwise from
  the nose; the firmware indexes by absolute *clockwise* bearing. They are a permutation of each
  other, so the conversion is four lines of reindexing off `zone_bearings_rad` — but it is not a
  no-op, and assuming it is gives you a rotated scan that will look plausible and be wrong. The
  recipe is in [06-sensors.md](06-sensors.md#if-you-need-firmware-index-order).
- **Scoring** — `mission.py` is pure geometry over landed positions; it runs anywhere, and is
  useful on real flight logs for asking "what would that run have scored".
- **The log format** — `recorder.py` writes plain JSONL and NPZ. A ROS 2 node writing the same
  schema gets the replay viewer and every metric for free, and makes simulated and real runs
  directly comparable in the same tooling.

## The honest caveat

A policy validated here has been validated against **ground-truth pose** and **free
communication**. The port does not change that; it only moves it. Before flying a strategy
that was tuned in this simulator, implement a drifting `PoseSource` and a lossy `Blackboard`
and re-run the comparison. Both are single classes behind seams that exist precisely so this is
cheap — see [ADR-0003](adr/0003-ground-truth-pose-and-perfect-comms.md) and
[FIDELITY.md](FIDELITY.md).
