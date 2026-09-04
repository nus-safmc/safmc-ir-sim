# Sensor models

## What a sensor is

Every sensor in this simulator is the same shape: a function from the drone's **true** state
and the world to a frozen reading, sampled at a rate that divides the tick rate, driven by
the runner through one contract in `sensors/base.py`. Your policy reads every reading by
name from `obs.sensors`; `obs.tof` and `obs.markers` are shorthands for the two below.

This page is what those two report, and what the one sensor the platform models but the
airframe does not carry — a [UWB ranging tag](#the-uwb-ranging-tag--not-flown) — would. The
contract itself, how to add a sensor, and what a **landmark** is — the thing you place in the
arena for a sensor to find — are in
[Adding a sensor or a landmark](10-adding-sensors-and-landmarks.md).

## The ToF ring

Eight **ST VL53L5CX** multizone sensors. One raycast per drone computes all of them.

> **The part number matters.** The VL53L5CX has a 65° *diagonal* FoV, which ST specifies as a
> 45° × 45° square. Its pin- and driver-compatible near-twin the **VL53L7CX** is 90° diagonal
> (60° × 60° square) with 3.5 m of range instead of 4.0 m. Swapping one for the other changes
> the zone width from 5.625° to 7.5° and turns a gapless ring into one with 120° of overlap.
>
> `nus-safmc/gazebo-slam-prototype`'s repo description says "8 × VL53L7CX". That is stale — the
> flown firmware links ST's `vl53l5cx` driver and the mount geometry only tiles with the L5CX. Why one and
not eight is [ADR-0002](adr/0002-single-vectorised-tof-sensor.md).

### Geometry — reproduced from the flown firmware

| Property | Value | Source |
|---|---|---|
| Rangers | 8, mounted counter-clockwise, 45° apart, gapless 360° | `tof_task.h:26-36`, `tof_task.c:183` |
| Zones per ranger | 8 columns of 5.625° across the 45° FoV | `tof_task.c:258` |
| Mount radius | 0.040 m cardinal, 0.034 m diagonal (the PCB is rectangular) | `safmc-ros/.../robot.urdf` |
| Optical axes | all horizontal, no pitch or roll | `safmc-ros/.../robot.urdf` |
| Square FoV per sensor | 45° × 45° (65° diagonal) | datasheet |
| Zone width | 5.625° — **derived** as FoV ÷ 8, never written down twice | |
| Ring coverage | 8 × 45° = exactly 360°, no gap, no overlap | |
| Physical reach | 4.0 m | datasheet |
| Max rate at 8×8 | 15 Hz (the sensor only does 60 Hz at 4×4) | datasheet |
| Firmware gate | `[0.05, 3.0]` m | `tof_task.h:16-18` |

Ranger `i` sits at `i * 45°` counter-clockwise from the nose, and zone `j` at
`(j - 3.5) * 5.625°` from its ranger's axis. Both are checked against the firmware's own
arithmetic (`tof_task.c:183` and `:258`) by a test, and the mount positions against the URDF,
so if anyone changes the ring a test tells them rather than a flight.

`front_index` says which ranger points forward. It defaults to 0; the flown `sdkconfig` had it
at 1 for that airframe, so set it per drone if you care. Note that **no zone points exactly along the
ranger axis** — the eight zones straddle it. A drone facing a wall 2 m away reads
`2.0 / cos(2.8125°) = 2.00236 m` on its two innermost forward zones, not 2.0 m.

### What you get

`ToFScan` has exactly four members. That is the whole sensor API:

```python
scan = obs.tof
scan.ranges_m             # (8, 8) float, inf where there was no valid return
scan.zone_bearings_rad    # (8, 8) body-frame bearings, CCW from nose. Constant.
scan.ranger_bearings_rad  # (8,)   body-frame ranger axes, CCW from nose. Constant.
scan.min_range_m          # nearest valid return anywhere on the ring
```

**`inf` means no valid return. It never means "maximum range".** Those are different facts and
the real sensor distinguishes them too, via `target_status` 255. A policy that does
`np.minimum(ranges, 4.0)` is choosing to conflate them — which is often the right choice for a
potential-field controller, but should be a choice.

There is **no `status` array and no `collapsed_m`**. Both existed in an earlier version and were
withdrawn in `50e2643`; see [SPEC](SPEC.md) R-SENS-4 and R-SENS-5 for why. Short version: the
simulator only ever emitted status `5` or `255`, which `isfinite(range)` already tells you, and
the collapsed scan was `ranges_m` reordered.

#### If you need firmware index order

The firmware's `tof_scan_collapsed_t` holds the same 64 values indexed by **absolute clockwise
bearing** (`tof_task.c:243-293`). Ours are in `(ranger, zone)` order, **anticlockwise from the
nose**. They are a permutation of each other — same numbers, different slots — so converting is
pure reindexing:

```python
import numpy as np

def collapsed_m(scan, n_bins=64):
    """ranges_m -> firmware bin order: index 0 straight ahead, clockwise, 5.625 deg per bin."""
    cw  = (-scan.zone_bearings_rad) % (2 * np.pi)          # CCW body frame -> CW, as firmware
    idx = np.minimum((cw * n_bins / (2 * np.pi)).astype(int), n_bins - 1).reshape(-1)
    out = np.full(n_bins, np.inf)
    np.minimum.at(out, idx, scan.ranges_m.reshape(-1))     # min-pool, as firmware
    return out
```

With the flown geometry the 64 zones land in 64 distinct bins, so the min-pool pools nothing —
but it is kept because a non-default `ToFConfig` (more rangers, more zones) can genuinely
collide, and then pooling is what the firmware would do.

> **Do not assume `tof.npz` columns are firmware bin indices.** They are `(ranger, zone)` order.
> Always map through `zone_bearings_rad`, which is stored in the same file.

### Height gating — the whole of the 2.5D model

A ray at altitude `z` is occluded by a primitive only if `z_min <= z < height`. That is one
comparison, and it is the reason the ring is our own sensor rather than ir-sim's: shapely is
strictly 2D and `ObjectBase.z` is dead code.

| Object | Height | Blocks a ray at 0.5 m cruise? |
|---|---|---|
| Perimeter wall | 1.5 m | yes |
| Inner wall, Unknown-area wall | 2.0 m | yes |
| Pillar | 2.0 m | yes |
| **Mission marker** | **1.0 m** | **yes at 0.5 m, no at or above 1.0 m** |
| Any other solid landmark | its own `height_m` | below its height only |
| Point landmark (a tag, a mark, an anchor) | none | never — a ray cannot hit a point |
| Another drone | **every altitude** | always for the ring, matching the 2D collision model; **never for the camera** (F-21) |

Since the rules cap flight at 1.4 m and every wall is at least 1.5 m, structure is never
overflyable — 2D is exact for navigation. Altitude earns its keep for markers and other placed
bodies. Drones occlude each other at every altitude deliberately: collision is ir-sim's and
strictly 2D, so what can kill you is what you can see.

### Accuracy

Closed-form ray-circle and ray-segment intersection, verified against two independently
derived analytic references to **1e-9 m** over 800 random rays. No polygonisation error: for
comparison, ir-sim's `Lidar2D` carries about 1.5 mm of error from tessellating circles.

A ray whose origin is *inside* an obstacle returns the distance to where it exits. This is the
exact case where `Lidar2D` silently freezes its whole scan at the previous tick's values, and
there is a test that walks a sensor through an obstacle asserting no two consecutive scans are
identical.

### Noise

`ToFConfig(noise_std_m=...)` adds Gaussian range noise from the sensor's own generator, applied
before gating and **never** to a no-return — perturbing "nothing is there" is meaningless.

Off by default, because the target paper's arenas are noiseless and reproducing it is our
first regression test. Turn it on for robustness sweeps; producing exactly those sweeps is
something the paper lists as future work.

### What is not modelled

The ring is sampled **synchronously at the tick rate**. The real ring is round-robin, one
sensor per 8 ms, so each refreshes at ~15 Hz and the ring is **skewed by up to 64 ms** across
sensors — it is never a synchronous snapshot. That matters for scan-matching and for a drone
turning fast, and not much for reactive avoidance. Assumption A-8. `ToFConfig(rate_hz=...)`
will at least decimate it.

Also absent: `range_sigma_mm`, `reflectance`, `ambient_per_spad` (the firmware discards them
too); the firmware's conservative "substitute 0.40 m for an unreliable return"; and any
vertical structure within a zone, which a 2.5D sim cannot have.

## The marker detector

Stands in for the AprilTag pipeline. Models the **geometry** — range, field of view, occlusion
— and nothing about image formation.

```python
for m in obs.markers:
    m.marker_id, m.kind, m.range_m, m.bearing_rad
```

Range is measured to the marker's **near surface**, not its centre. Occlusion is tested to
just short of that surface, so a marker never occludes itself but another marker can occlude
it. Other drones do not occlude it (F-21). Default rate is 2 Hz, the measured AprilTag rate on
the real hardware — nominally 10 Hz, but the detector task runs at the lowest priority in the
system — so `obs.markers` is fresh every tenth tick and `obs.stale_ticks["markers"]` counts
1–9 in between.

**It detects landmarks by kind.** `MarkerCamConfig.kinds` defaults to the three mission-marker
kinds. The real detector also reads the surveyed navigation tags (ids 12–29 in
`laptop/setup.yaml`); in the simulator that is a [landmark](10-adding-sensors-and-landmarks.md#landmarks) of kind `"nav_tag"`
plus one entry in `kinds`, and the detection comes back with `kind == "nav_tag"` so a policy
can tell it from a victim — exactly as the real one can from the id range.

Reporting `kind` is not cheating: the markers are team-supplied and the flown configuration
assigns tag ids by role (0-11 landing targets, 8-11 bonus). A drone that reads a tag id does
know what it is looking at.

> **Assumption A-4 — detection range, default 3.0 m — is the single most important unmeasured
> number in this simulator.** It sets how much area a drone sweeps per metre flown, which is
> the dominant term in every search-policy comparison. Nothing in any repository measures it;
> the only claim is an inherited comment about "about 1 meter (tested with tag on screen, not
> on paper)" describing *different* detector parameters than the ones actually set. Getting it
> wrong by a factor of two could change which policy wins.
>
> Measuring it is one afternoon: fly at a tag, log detections against distance. Do it before
> quoting any headline number.

A-5 (camera FOV, 1.0 rad) is derived from the QVGA intrinsics rather than measured, and the
real camera is pitched 45° nose-down while ours is modelled horizontal (F-6) — so it sees the
floor ahead rather than the horizon, and ground markers enter view differently.

## The UWB ranging tag — not flown

**The airframe does not carry one.** The rules permit ultra-wideband by name (booklet §6.3)
and say where an anchor may stand (§3.3.1 r.14–17), and the team may buy a module; this is
what a policy would be written against if it did, and what a future `PoseSource` would
consume. It is a model of the literature on DW1000/DW3000-class hobby modules, not of any
device on the bench, and every number in it is an assumption with an ID.
[ADR-0006](adr/0006-uwb-ranging-sensor.md) has the decisions; `sensors/uwb.py` is the source
of truth for the model. Opt in with `RunConfig(sensors=flown_sensors() + (UWBConfig(),))` and
put anchors in the arena as landmarks of kind `"uwb_anchor"`.

```python
uwb = obs.sensors["uwb"]
uwb.anchor_ids       # ("start_w0", ...)  every anchor, in arena order, fixed for the run
uwb.anchor_xyz_m     # (N, 3) surveyed positions, at the configured mount height. Constant.
uwb.ranges_m         # (N,)   reported range per anchor, inf where nothing was heard
uwb.heard            # (N,)   bool, isfinite(ranges_m)
```

That is the whole reading, and it is what a real tag reports: a tag in a real-time-location
network is configured with its anchors' surveyed coordinates and returns a distance per
anchor or nothing. **No bearing, no quality factor, and no flag saying which ranges are
biased.** A range behind a wall looks exactly like a clean one. That is the problem a
localiser has to solve, and the reading is built not to solve it for you. The surveyed
positions are the team's own configuration — the same `ArenaConfig` that placed the anchors
— not a leaked world position ([R-POL-3](SPEC.md) as amended).

### The model

| Step | What happens | Number |
|---|---|---|
| Range | Three-dimensional distance from the drone's true position to the anchor at `anchor_height_m` — a 2.0 m anchor 3 m away reads 3.35 m | `anchor_height_m` 2.0 m, a tripod |
| Reach | Beyond `max_range_m`: `inf` | **A-10**, 20 m. Datasheets say 60 m; hobby firmware delivers 12–20 m indoors |
| Obstruction | The line-of-sight test the mission uses, against **walls and pillars only**, at the drone's altitude. A marker is a cardboard box and a teammate a 30 cm airframe: radio goes through both | — |
| Clear | true range + N(0, `los_noise_std_m`) | **A-9**, 0.05 m. DW1000 timestamp noise is 3–4.5 cm; testbeds report 2–8 cm |
| Obstructed | dropped (`inf`) with probability `nlos_drop_probability`, otherwise true range + `nlos_bias_m` + N(0, `nlos_noise_std_m`) | **A-12**, 0.10; **A-11**, +0.15 m and 0.40 m: 0.49 ns and 1.39 ns (0.42 m, rounded) through one wall or obstacle on a DW1000 in a concrete-panel flat |
| Outlier | with probability `outlier_probability`, a further positive error uniform on [0, `outlier_max_m`] | **A-13**, 0 (off) and 1.5 m — the tail is documented, its rate is not |
| Floor | a reported range is never negative | — |

Every anchor is measured in the same tick, at 10 Hz (`rate_hz`); the real tag sweeps them
one at a time in 3–4 ms slots, so the first-to-last skew is under 40 ms for ten anchors and
under 2 cm at cruise speed. PANS itself returns four ranges per 100 ms frame; a ten-anchor
sweep at 10 Hz assumes MaUWB-class firmware (F-23). Every draw comes
from the sensor's own generator, four per anchor per sweep whatever the geometry, so the
noise stream depends on the seed alone.

### What is not modelled, and matters

- **Wall count and material** (F-24). One wall's numbers apply behind three, where the source
  measured four times the bias — through concrete panels that still ranged; metal is where a
  link dies. Nobody has measured the venue's walls.
- **Reach** (A-10). At 20 m the whole field is within reach of three Start Area anchors, only
  just; at the 12 m hobby-firmware floor the far third hears none of them. Even in reach, six
  anchors in a 5 m-deep strip give poor along-field geometry beyond about 14 m and the room's
  walls obstruct — which is why the Known Search Area's ten aids matter, and why the example
  puts four there.
- **Calibration** (F-26). An uncalibrated antenna delay adds 10–30 cm per anchor pair; the
  model assumes the team calibrates.
- **Anchors above 2.0 m** (F-25): the obstruction test is made at the drone's altitude, exact
  up to 2.0 m — every wall and pillar an interior path can cross is 2.0 m — and over-reporting
  obstruction above.
- **The consumer.** A range-only sensor is half a feature until a `PoseSource` fuses it
  ([ADR-0003](adr/0003-ground-truth-pose-and-perfect-comms.md)). That is the next piece of
  platform work.

### Placing anchors, legally

An anchor is a `Landmark` of the tag's kind. A point (no footprint) is invisible to the ring
and to collision, which is right for a radio, but a point at fixed coordinates in the Known
Search Area can end up inside a generated wall; give it a tripod base, `radius_m=0.25`, and
the generator draws around it while the ring and collision still ignore it. The rulebook's
placement rules — any number in the Start Area, at most ten in the Known Search Area, none
in the Unknown Search Area, each within 1 m x 1 m — are `validate_nav_aids(arena,
("uwb_anchor",))` in `world/arena.py`. **The runner does not call it** (R-WORLD-9): a run
with an anchor in the room is a legitimate experiment when you asked for it, and the check
exists so that nobody quotes one without knowing. Collinear anchors leave a mirror
ambiguity, so use both rows of the Start Area. `examples/04_uwb_ranging.py` does all of
this and then grades the sensor from the log alone: true ranges from `states.npz`, anchor
positions from `uwb.npz`, and the recorded arena to say which paths crossed a wall.
