# ADR-0002: One vectorised ToF sensor per drone, not N instances of `Lidar2D`

**Status:** Accepted · **Date:** 2026-08-21

## Context

The real drone carries 8 x VL53L5CX rangers at 45 degree spacing, each 8x8 zones. ir-sim's
`Lidar2D` models a *contiguous* beam fan, so a ring of separated rangers can only be built as 8
separate sensor instances per drone.

Measured: `Lidar2D` cost is dominated by fixed per-sensor GEOS overhead, not per-beam cost — 4
beams to 100 beams costs only 2.4x. So N one-beam instances is the worst possible cost per unit of
information. At 8 drones x 4 rangers, ir-sim manages ~449 steps/s; a vectorised numpy raycaster
does the same work 14-31x faster.

`Lidar2D` also has a silent stale-data defect: `calculate_range_vel()` never resets `range_data`,
so when the sensor origin is inside an obstacle the scan freezes at the previous tick's values
with no error flag.

Separately, the 2.5D requirement needs **height-gated** occlusion — a 1.0 m mission marker blocks a
ray at 0.5 m cruise altitude but not at 1.2 m. Shapely is strictly 2D and `obj.z` is dead code.

## Decision

Implement **one** ir-sim sensor object per drone, `ToFRing`, that computes all 8 rangers x 8 zones
in a single vectorised numpy raycast against closed-form primitives (circles, segments), with
per-obstacle height gating.

ir-sim has no sensor registry — `SensorFactory.create_sensor` is a hardcoded if/elif — so
registration is by a narrow, explicit factory patch applied at import.

## Consequences

- **Speed:** one numpy call per drone per tick instead of 8 GEOS boolean differences.
- **Correctness:** closed-form intersection agrees with analytic ground truth to 1e-9 m; no
  polygonisation error, no stale-data path.
- **2.5D becomes possible at all.** Height gating is a column comparison in the raycaster. There is
  no way to get it from `Lidar2D`.
- **Fidelity:** we can emit the firmware's exact `(distance_mm, target_status)` per zone and its
  64-bin collapsed scan, rather than a `LaserScan`-shaped approximation.
- **Cost:** we own the raycaster, so it needs real tests. R-SENS-7 mandates agreement with an
  independent analytic implementation.
- **Cost:** we forgo `FogMap`'s automatic lidar-driven reveal, which keys off `obj.lidar`. Coverage
  is computed by our own metrics layer instead.
- **Upstream option:** a `SensorRegistry` mirroring ir-sim's existing `GridMapGenerator`
  auto-registration would be a ~30-line PR and would remove the factory patch.
