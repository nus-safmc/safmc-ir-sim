# Architecture

## The layering rule

> **ir-sim owns the world. We own the drone, the sensing and the mission. You own the policy.**

The corollary is a hard rule: the simulator contains **no guidance, control or strategy**. One
motion command (a velocity), one commitment (land), and nothing in between. Anything that
decides *where to go* belongs to whoever is being measured.

ir-sim is good at geometry, collision, fixed-step integration, scene description and drawing.
It is not good at sparse ranging, and it has no opinion at all about altitude, mapping, comms,
missions or logs. The split follows that line exactly.

```
                        ┌─────────────────────────────────────────┐
   author a policy ───► │  policies/         step(obs) -> cmd     │
                        └───────────────┬─────────────────────────┘
                                        │ Observation / Command  (frozen dataclasses)
                        ┌───────────────┴─────────────────────────┐
                        │  runner.py     the tick loop            │
                        │    ├─ PoseSource      ◄── seam R-SEAM-1 │
                        │    ├─ Blackboard      ◄── seam R-SEAM-2 │
                        │    ├─ mission.py      scoring + rules   │
                        │    └─ recorder.py     structured log    │
                        └───────────────┬─────────────────────────┘
                                        │ env.step(actions)
                        ┌───────────────┴─────────────────────────┐
                        │  ir-sim  2.10.2                         │
                        │    world · STRtree collision · YAML     │
                        │    render · fixed-step integration      │
                        │    ├─ kinematics.py  @register_kinematics
                        │    └─ sensors/       ToFRing, MarkerCam │
                        └─────────────────────────────────────────┘
```

Everything crossing a horizontal line is a typed, frozen dataclass. Nothing below a line reaches up.

## Module map

| Module | Owns | Key requirement |
|---|---|---|
| `frames.py` | ARENA frame, angle wrapping, the NED bijection | R-FRAME-1..5 |
| `kinematics.py` | The 2.5D quad model, registered with ir-sim | R-DRONE-1..8 |
| `sensors/raycast.py` | Vectorised closed-form ray casting with height gating | R-SENS-6, R-SENS-7 |
| `sensors/tof_ring.py` | The 8-ranger ring; zones, statuses, 64-bin collapsed scan | R-SENS-1..5 |
| `sensors/marker_cam.py` | Marker detection with FOV + LOS + range | R-SENS-10 |
| `world/arena.py` | Seeded arena generation and validation; emits ir-sim YAML | R-WORLD-1..6 |
| `api.py` | `Observation`, `Command`, `Policy`, the policy registry | R-POL-1..7 |
| `blackboard.py` | Double-buffered perfect shared store | R-POL-8, R-SEAM-2 |
| `pose.py` | `PoseSource`; v0.1 ships ground truth | R-SCOPE-3, R-SEAM-1 |
| `mission.py` | Targets, LOS, scoring, fire coupling, relay, wave rules | R-MISS-1..8 |
| `runner.py` | The tick loop, lifecycle state machine, determinism | R-TIME-*, R-DET-* |
| `recorder.py` | Versioned structured log | R-OBS-1..4, R-OBS-6 |
| `metrics.py` | Coverage, score, time-to-X, alive-agent-seconds | R-OBS-2 |

## The tick

One tick is exactly this, and the order is load-bearing:

1. **Snapshot the blackboard.** Every agent this tick reads the same immutable view. Writes go to
   the back buffer and become visible next tick. Without this, agent 0's publication would be
   visible to agent 1 but not vice versa, and results would depend on index order (R-POL-8).
2. **Build each agent's `Observation`** from its `PoseSource`, its sensors' last sample, and the
   blackboard snapshot. Sensors that are decimated return their most recent sample, with `stale_ticks`
   set so a policy can tell.
3. **Call each policy's `step(obs)`.** Exceptions propagate — a crashed policy aborts the run with
   agent id, tick and traceback. It is never swallowed into a hover (R-POL-9).
4. **Resolve commands to ir-sim actions** through the lifecycle state machine. A `LANDED` agent
   ignores its command permanently. Take-off is checked against the two-wave rule.
5. **`env.step(actions)`** — ir-sim integrates every object, rebuilds the STRtree, steps sensors.
6. **Update the mission**: landings, scoring, LOS, rule violations.
7. **Record.** Then swap the blackboard buffers.

## Where the seams are, and why they are drawn there

Two things are deferred by team decision: pose noise and radio modelling
([ADR-0003](adr/0003-ground-truth-pose-and-perfect-comms.md)). Deferred work only stays cheap if
the seam is the *only* path.

**`PoseSource`** is the single producer of the pose in an `Observation`. A policy cannot reach
`obj.state` because it cannot reach `obj`. Adding drift means writing one class.

**`Blackboard`** is the single channel between agents. A policy holds no reference to another
policy or another agent's object. Adding loss and range limits means writing one class.

These are enforced structurally, not by convention: `Observation` is frozen and contains only
plain data, and R-POL-4 requires that an auditor can enumerate everything reachable from an
`Observation` and find no environment, arena, mission or peer object.

## Determinism

A run is `(scenario, seed, policy_id, policy_config)` and nothing else.

- Per-agent generators come from `SeedSequence(seed).spawn(n)`, so adding a 9th drone does not
  perturb the first eight.
- No module-level `numpy.random` anywhere (R-DET-2).
- ir-sim's process-global RNG is seeded from the same seed — and because it *is* process-global,
  **parallel sweeps must use separate processes** (R-DET-4). This is an ir-sim constraint we
  inherit, documented rather than worked around.
- Wall-clock values live only in the log's `meta` block, so two identical runs produce
  byte-identical logs everywhere else (R-DET-1).

## Why the ToF ring is one sensor

The single most important structural decision. Covered in
[ADR-0002](adr/0002-single-vectorised-tof-sensor.md); the short version is that it is simultaneously
the fast option, the only correct option for 2.5D height gating, and the only way to emit the
firmware's real data product. It also happens to remove the one thing that would make a 25-drone
fleet intractable.
