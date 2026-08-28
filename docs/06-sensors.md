# Sensor models

## The ToF ring

One ir-sim sensor per drone, computing all rangers in a single vectorised raycast. Why one and
not eight is [ADR-0002](adr/0002-single-vectorised-tof-sensor.md).

### Geometry — reproduced from the flown firmware

| Property | Value | Source |
|---|---|---|
| Rangers | 8, mounted counter-clockwise, 45° apart, gapless 360° | `tof_task.h:26-36`, `tof_task.c:183` |
| Zones per ranger | 8 columns of 5.625° across the 45° FoV | `tof_task.c:258` |
| Mount radius | 0.040 m, all optical axes horizontal | `safmc-ros/.../robot.urdf` |
| Physical reach | 4.0 m (VL53L5CX) | datasheet |
| Firmware gate | `[0.05, 3.0]` m | `tof_task.h:16-18` |

Ranger `i` sits at `i * 45°` counter-clockwise from the nose, and zone `j` at
`(j - 3.5) * 5.625°` from its ranger's axis. Note that **no zone points exactly along the
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
| **Mission marker** | **1.0 m** | **yes at 0.5 m, no above 1.0 m** |
| Another drone | its own altitude ±0.05 m | only if at a similar altitude |

Since the rules cap flight at 1.4 m and every wall is at least 1.5 m, structure is never
overflyable — 2D is exact for navigation. Altitude earns its keep for markers and for other
drones.

### Accuracy

Closed-form ray-circle and ray-segment intersection, verified against two independently
derived analytic references to **1e-9 m** over 800 random rays. No polygonisation error: for
comparison, ir-sim's `Lidar2D` carries about 1.5 mm of error from tessellating circles.

A ray whose origin is *inside* an obstacle returns the distance to where it exits. This is the
exact case where `Lidar2D` silently freezes its whole scan at the previous tick's values, and
there is a test that walks a sensor through an obstacle asserting no two consecutive scans are
identical.

### Noise

`ToFConfig(noise_std_m=...)` adds Gaussian range noise from the agent's own generator, applied
before gating and **never** to a no-return — perturbing "nothing is there" is meaningless.

Off by default, because the target paper's arenas are noiseless and reproducing it is our
first regression test. Turn it on for robustness sweeps; producing exactly those sweeps is
something the paper lists as future work.

### What is not modelled

The ring is sampled **synchronously at the tick rate**. The real ring is round-robin, one
sensor per 8 ms, so each refreshes at ~15 Hz and the ring is **skewed by up to 64 ms** across
sensors — it is never a synchronous snapshot. That matters for scan-matching and for a drone
turning fast, and not much for reactive avoidance. Assumption A-8.

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
it. Default rate is 2 Hz, the measured AprilTag rate on the real hardware — nominally 10 Hz,
but the detector task runs at the lowest priority in the system.

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

## Adding a sensor

ir-sim has no sensor registry — `SensorFactory.create_sensor` is a hardcoded if/elif — so
registration is a narrow patch applied at import, in `tof_ring.install()`. Copy that pattern.

The duck-typed contract is small: `.step(state_3x1)`, `.sensor_type`, `.parent` (assigned for
you), and `.plot` / `.step_plot` / `.plot_clear` if you want to be drawn.

Two things to get right:

- Read the world through `self._world_scene`, and get altitude from `self.parent.state[3, 0]`.
  ir-sim hands `step()` only `[x, y, theta]`.
- Do not name your `sensor_type` `lidar2d` or `fmcw_lidar2d` unless you want ir-sim to elect
  it as `obj.lidar`, which changes what `env.get_lidar_scan()` and `FogMap` use.

A `SensorRegistry` mirroring ir-sim's existing `GridMapGenerator` auto-registration would be
roughly a 30-line upstream PR and would remove the patch entirely. Worth doing if anyone has
an afternoon.
