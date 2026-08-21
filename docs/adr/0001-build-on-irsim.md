# ADR-0001: Build on ir-sim rather than writing our own simulator

**Status:** Accepted · **Date:** 2026-08-21

## Context

The team decided to abandon Gazebo + PX4 SITL and adopt ir-sim. Before building, a recon agent was
tasked specifically with attacking that choice. It produced a well-evidenced case for writing an
~800-1500 LoC numpy simulator instead, measuring ir-sim at 14-31x slower than a vectorised
raycaster and documenting three correctness defects in `Lidar2D`, a matplotlib figure leak, and a
process-global RNG.

## Decision

**Build on ir-sim 2.10.2. Replace exactly one class — `Lidar2D` — with a vectorised sensor of our
own. Keep everything else.**

## Rationale

1. **The decisive evidence is external.** arXiv:2607.25195 (NUS; Leong and Teo; **IROS 2026**)
   produced an accepted multi-agent exploration result on **ir-sim v2.9.0**, using four single-ray
   sensors, a first-order integrator and a 10 Hz loop. The question "is ir-sim enough for this
   class of experiment" has a published answer.

2. **The performance critique targets one class.** ir-sim's own motion, collision and world
   stepping run at 0.3 ms/step with 20 robots. Profiling attributes **68% of total simulation time
   to a single GEOS boolean-difference call per lidar per step**. Replacing `Lidar2D` removes the
   objection without inheriting the rest.

3. **Three of the critique's four "absent" items are not ours to want.** No comms model (we chose a
   perfect blackboard), no occupancy mapping (that is the system under test, not the world), no
   structured logging (greenfield under every option, so not a differentiator). Only "no altitude"
   is a real gap, and carrying `z` as extra kinematics state rows is verified to work.

4. **Continuity beats elegance for a student team.** ir-sim is MIT, pip-installable in 20 seconds
   with no compiler and no ROS, actively maintained with fast issue turnaround. A hand-rolled
   simulator needs one person to own ~1k LoC of numpy across years of team turnover. That person
   may graduate.

5. **The target paper's code is a working reference implementation** of the ir-sim multi-agent +
   multi-ranger + custom-behaviour pattern, which de-risks the parts we have not written yet.

## Consequences

- **Accepted cost:** we inherit ir-sim's global RNG, so parallel sweeps must use separate processes
  (R-DET-4), and its unconditional figure creation, so episodic loops must `plt.close('all')`.
- **Accepted cost:** every ir-sim landmine in [docs/03-irsim.md](../03-irsim.md) is now our problem
  to route around. They are enumerated so they are routed around once.
- **Benefit:** we can reproduce the target paper as a regression test, since it targets the same
  library.
- **Revisit if:** a 25-drone fleet with the full ToF ring cannot sustain a useful sweep rate, or
  ir-sim upstream breaks the `register_kinematics` contract.

## Rejected alternatives

| Option | Why not |
|---|---|
| Own numpy sim | Fastest and exactly correct, but needs a permanent owner; discards the reproducibility link to the target paper |
| gym-pybullet-drones | Solves the physics problem we just deleted; no sparse ToF model |
| VMAS | Genuinely strong *if* we commit to learned policies; batching machinery is dead weight for hand-written heuristics |
| Webots | Only mainstream sim with real `DistanceSensor` **and** range-limited comms; ~1 GB desktop app, painful per-robot process IPC at N=25. Good late-stage validation rig |
| ROS 2 + Stage | Conceptually exactly right; a 45-star community port of 20-year-old C++, and back into colcon |
| Aerial Gym / Isaac | Requires an NVIDIA GPU and multi-GB install; absurd for a student team on laptops |
| Flightmare | Last pushed 2024-06-14. Dead |
