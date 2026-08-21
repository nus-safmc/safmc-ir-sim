# Overview

## The thesis

The team previously ran Gazebo + PX4 SITL ([`gazebo-slam-prototype`](https://github.com/nus-safmc/gazebo-slam-prototype)).
That stack simulates rigid-body dynamics, motor mixing, EKF2 and a full autopilot — none of which
is in question. What *is* in question is whether a swarm of drones carrying eight cheap ToF rangers
can explore an unknown room and find targets fast enough to win.

So: throw away the physics, keep the geometry.

**The existence proof is not hypothetical.** arXiv:2607.25195 — *Decentralized Scalable Exploration
via Emergent Adaptive Lévy Walks on Minimal-Sensing Platforms*, by Wai Lun Leong and Rodney Teo of
**NUS**, accepted to **IROS 2026** — produced a publishable nano-UAV exploration result using
**ir-sim v2.9.0**, four single-ray range sensors, a first-order integrator, and a 10 Hz loop. No
Gazebo. No PX4. See [related work](09-related-work.md).

## What this simulator is for

Four questions, in priority order:

1. **Search policy.** Mapless (reactive, Lévy-walk-like) versus map-based (frontier, coverage
   planning). Which finds more targets in 600 seconds?
2. **Resource allocation.** Landing on a victim consumes that drone permanently. With up to 12
   targets, a relay chain worth a 2x multiplier, and 10-25 drones, *how many* to spend and *when*
   is a real optimisation problem — and it is pure geometry.
3. **Mapping.** Can eight 45-degree ToF cones at 15 Hz build a map good enough to plan on?
4. **Sensor fusion.** Deferred by decision (v0.1 uses ground-truth pose), but the architecture
   holds a seam open for it.

## What it is not for

Not flight-worthiness, not control tuning, not anything with mass in it. Also — worth knowing —
the rulebook forbids using simulation as evidence in the Team Challenge Video
(§5.2 g.2: computer-aided simulations may not be used to prove flight worthiness, animations not
allowed). **This is a development tool, not a submission artefact.** It is fair game for the live
presentation, which is 40% of the total score and explicitly asks teams to justify their search
strategy and positioning method.

## The four decisions that shaped v0.1

Taken by the team lead before the build started:

| | Decision | Deferred to |
|---|---|---|
| **Pose** | Ground truth from ir-sim. No estimation stack. | `PoseSource` seam |
| **Policy API** | Per-tick callback, `policy(obs) -> cmd`. | — |
| **Comms** | Perfect shared blackboard. No DDS, no radio model. | `Blackboard` seam |
| **Dimensions** | 2.5D: planning is 2D, but `z` is a real state. | — |

The first and third together mean v0.1 answers *"which search strategy is better, given perfect
state?"* — not *"does this survive real drift and a lossy link?"*. That is the right first
question, and it is only honest if the seams are real. They are load-bearing requirements in the
[spec](SPEC.md), not aspirations.

2.5D is not a compromise here, it is the correct model: the rules cap flight at **1.4 m** and
forbid flying over walls, which are **1.5-2.0 m** tall. The arena is genuinely planar. Altitude
earns its place for a different reason — mission markers are up to **1.0 m** tall, so at a 0.5 m
cruise altitude they occlude ToF rays, and whether a drone is at cruise or landed changes what it
can see and whether it scores.
