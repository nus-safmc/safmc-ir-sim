# Related work — the two systems this simulator must be able to host

## 1. SDLW — arXiv:2607.25195 (IROS 2026)

**Decentralized Scalable Exploration via Emergent Adaptive Lévy Walks on Minimal-Sensing
Platforms.** Wai Lun Leong, Teo Swee Huat Rodney. **National University of Singapore.** Accepted to
IROS 2026, Pittsburgh. Code: [williamleong/sdlw](https://github.com/williamleong/sdlw), MIT.

Verified to exist: `arxiv.org/abs/2607.25195` returns HTTP 200, the arXiv Atom API returns exactly
one result, and the PDF is 641 KB.

**Why it is the single most important reference for this project:** it is an NUS paper, from the
same university, that got an IROS 2026 acceptance out of **ir-sim v2.9.0** using four single-ray
range sensors, a first-order integrator and a 10 Hz control loop. It is the existence proof for
abandoning Gazebo, and it doubles as a reference implementation of the ir-sim multi-agent +
multi-ranger + custom-behaviour pattern.

### The method

Not learned. No network, no training, no RL. A hand-designed stochastic reactive controller.

- Each agent draws a Lévy exponent `α ~ Uniform(2,3)` **once** at init.
- Step length: `Pr[d=0] = 1/2` exactly; otherwise `Pr[d=j] ∝ j^(−α)` for integer `j` in
  `[⌈d_min⌉, ⌊d_max⌋]`. `d = 0` means a rotate-only cycle.
- Heading: weights `w_s = β_s · r_s²` over the four sensors (**range squared**), `β_back = 0.3`.
  Resultant gives mean direction `μ` and "openness" `o = ‖v‖ / W`. Concentration
  `κ(o) = κ_max − (κ_max − κ_min)·o`. Sample `θ ~ VonMises(μ, κ)`.
- Two-state machine, ROTATE and MOVE, with bang-bang yaw and ±0.2 m/s axial/lateral nudges when a
  range drops below `r_th = 0.4 m`.

Hyperparameters: 0.5 m/s forward, 0.5 rad/s max yaw, `d ∈ [0.5, 20] m`, `κ ∈ [0.6, 10]`, 10 Hz.
Baseline is UHLW — identical but with a uniform heading.

Evaluation: three 20x20 m arenas (open, rooms-and-corridors, cluttered-50), `k ∈ {1,2,4,8,12}`,
**50 trials**, **900 s** horizon, coverage on a 0.25 m grid. Reported gains: +79.6% open,
+43.1% rooms, +13.6% cluttered.

### Four findings that qualify the result

Recon read the released code and instrumented it. These are not nitpicks; they change how we build.

**G1 — ir-sim's omni kinematics does not integrate yaw, and the harness patches it externally.**
`run_simulation.py:311-314` does `robot._state[2,0] += yaw_rate * step_time` *after* `env.step()`.
That is forward Euler with **no angle wrapping**, writing `_state` directly so `mid_process` is
bypassed and the robot's collision geometry is never re-derived from the new yaw.
→ [R-DRONE-3](SPEC.md) requires our kinematics handler integrate and wrap yaw itself.

**G2 — collided agents are permanently dead.** Under `collision_mode: 'stop'`, `stop_flag` makes
`ObjectBase.step` return early, which skips the behaviour call, the geometry update *and*
`sensor_step()`. Measured on cluttered-50, k=4: **half the team was dead 35 s into a 900 s
episode.** Coverage and the collision metric are therefore not independent — "better heading
policy ⇒ more coverage" is confounded by "fewer deaths ⇒ more live agent-seconds".
→ Our metrics report coverage normalised by **live-agent-seconds**, and collision handling is a
selectable mode.

**G3 — the collision metric saturates.** It counts *distinct robots that ever collided*, so it is
bounded by `k`. The paper's k=12 figures (4.08 vs 4.80) are "mean number of the 12 robots that
died", not a collision rate. The paper half-concedes this in §IV-B.

**G4 — "openness" does not measure open space.** Because `w_s = r_s²` and left/right cancel, an
isotropically open agent at (4,4,4,4) gives `o = 0.212 → κ = 8.01`, i.e. nearly *maximal*
concentration. `o → 1` requires everything blocked but one direction. Measured over 300 s runs the
effective band is `κ ∈ [1.4, 10]` with median ~7; **`κ_min = 0.6` is never reached.** This inverts
the paper's §II-C prose, though the released README describes it accurately as "directional
consensus rather than simply large average clearance". Practical reading: SDLW is a stochastic
potential-field steer, and the superdiffusion claim rests on the step-length distribution, not the
heading policy.

A low-N spot check (n=3 seeds vs the paper's 50, k=4 only) reproduced the qualitative gain in open
and rooms arenas but came out **negative** in cluttered — tracking the dead-robot count exactly.
Do not over-read n=3; do note that the confound is visible immediately.

### What we owe it

Reproducing SDLW is our first regression test: it is cheap (~10-15 s per 900 s k=4 run), it pins
our sensor and kinematics semantics against a published result, and the released sweep driver is
*not* in the repo, so we must rebuild it anyway.

And the ablations the paper lists as future work — range noise, ToF cone geometry, dropout, yaw
drift, odometry drift — are exactly what this simulator is being built to provide.

## 2. Nano_Swarm_Mapping — ETH-PBL

Crazyflie-class nano-drones with a 4-direction multizone ToF deck, doing onboard scan collection,
ICP and pose-graph SLAM, with map merging across the swarm. **GPL-3.0** — see the licence note
below.

### Three clean seams for feeding it from our simulator

**Seam A — the occupancy-grid builder.** `evaluation/mapping_bridge.py:24`:
```python
pcloud_to_map(pos, tof_mm, max_range_mm, min_x, min_y, l_x, l_y, res)
#   pos:    (n,3) float32 -> [x_m, y_m, yaw_rad] world frame
#   tof_mm: (n,4,8) int16 -> mm, order [FRONT, BACK, LEFT, RIGHT] x 8 columns, -1 invalid
```
A pure function of `(pose, 4x8 ranges)`. **Our ToF ring already produces exactly this shape** — it
is a subset of our 8-ranger ring.

**Seam B — the `scan_t` blob**, 1144 bytes little-endian, the unit the swarm exchanges: a pose id
plus 15 sub-frames of `int16 dists[4][8]` with `(dx, dy, dyaw)` relative to the anchor pose. Each
sub-frame is sampled during a 45° yaw slew over 2 s. Synthesising these from our sim is ~80 lines.

**Seam C — the C algorithms host-compiled.** `icp/icp.c` has no firmware dependency beyond an
`ASSERT` macro; `ls-slam/*.c` already carries an `OFFDRONE` escape hatch. Both `ctypes`-wrap
cleanly. Recorded ICP console logs give ready-made unit-test vectors.

**Not portable:** the exploration policy (`exploration.c` is bound to `commanderSetSetpoint`,
`estimatorKalmanGetEstimatedPos` etc.) — but it is ~150 lines of trivially reimplementable logic,
and reimplementing it in Python against our observation interface is precisely what this simulator
exists for. The swarm radio layer is FreeRTOS-bound and we are not modelling radio anyway.

**Missing entirely:** proximity-based loop-closure detection. The paper describes it; the code
hardcodes 9 constraint pairs. If the team wants to test collaborative loop closure, that is an
implementation from the paper text, and sweeping its threshold in simulation is a genuinely novel
contribution available here.

### Two warnings

**Licence.** GPL-3.0 covers the whole repo. Linking `icp.c`, `ls-slam/*.c` or `mapping.c` into our
simulator — including via `ctypes` loading a `.so` built from their sources — makes the combined
work GPL-3.0 if distributed. For an internal university tool that is probably fine, but it must be
a **conscious decision**. The safe alternative is to reimplement ICP and the grid builder (both
textbook) and use only the **data format** and the **recorded logs**: a binary layout is not
copyrightable and logs are data.

**A bug in their reference reprojection.** `evaluation/log.py:76-88` advances the bearing
accumulator *after* a `continue`, so every column following an invalid one is mis-bearinged by
`k · 5.625°`, reaching 39° for the last column. The C source (`scan.c:52-56`) is correct — the
increment is in the `for` clause. Do **not** use `extract_points` as a golden reference; port
`scan.c`. The published map-RMSE figures derived through this path are suspect. **UNVERIFIED**
whether the paper's numbers used this code.

### Relevance caveat

Their system does wall-following in a 1 m-grid maze, with no inter-drone collision avoidance, no
relative localisation and all optimisation deferred until after landing. The **sensing model** and
the **ICP + pose-graph** parts transfer well. The **exploration policy** almost certainly does not:
it will not cover an open arena and it lands at the first dead end.
