# ir-sim — what we use, what we avoid

**Version pin: `ir-sim==2.10.2`.** Do not float this dependency.

2.9.0 and 2.10.2 are materially different APIs: 2.9.0 has no `register_kinematics`, no fog map, no
JPS/Informed-RRT*, and a 171-line `kinematics_handler.py` against 2.10.2's 480. The target paper's
released code pins 2.9.0; we pin 2.10.2 and note the difference where it matters.

MIT licensed, ~18.6k LoC, actively maintained (1115 stars, maintainer closes issues within days).
Pure pip, 9 dependencies, no ROS, no compiler, ~20 s install.

## The simulation contract

```python
env = irsim.make("world.yaml", display=False, disable_all_plot=True, seed=1)
for i in range(N):
    env.step(action)      # advance exactly one fixed dt
    env.render(0.05)      # no-op when disable_all_plot=True
    if env.done(): break
env.end()
```

`env.step()` returns **`None`** — this is not a Gym API. Order of operations inside one step
(`env_base.py:316-331`):

1. keyboard/group actions fill unset slots
2. **all** objects integrate, from the pre-step state
3. the shapely STRtree is rebuilt once
4. **all** sensors step — so every scan sees the same consistent post-motion snapshot
5. world clock increments, fog map is revealed
6. status/collision/arrival flags update

Step 2 followed by step 4 is a real guarantee and it is why multi-agent stepping has no
round-robin bias. We rely on it.

**Time is strictly fixed-step.** `world.step_time` is the only dt; `world.count` increments once
per `env.step()`; there is no accumulator, no sub-stepping, and no real-time sync. Headless runs as
fast as the CPU allows.

## Primitives we use

| Primitive | Where | Why |
|---|---|---|
| Shapely STRtree collision pipeline | `env_base.py:1085-1090` + `object_base.py:652-666` | Correct broad+narrow phase, rebuilt in the right place |
| `@register_kinematics` | `kinematics_handler.py:22-48` | Clean 4-method contract; our 2.5D quad drops in with no fork |
| YAML scene DSL + `ObjectFactory` | `object_factory.py` | `number` + `distribution` with rejection sampling; scenario generation solved |
| Fixed-step loop + per-env param binding | `env_base.py:156-162` | Multiple envs in one process actually work |
| Matplotlib renderer + overlay hooks | `env_base.py:481-555` | `draw_trajectory`, `draw_points`, `draw_box`, `draw_quiver`, GIF capture |
| Path planners | `lib/path_planners/` | A*, JPS, PRM, RRT, RRT*, Informed-RRT* all consume `env.get_map(res)` |

## Primitives we deliberately do **not** use

### `Lidar2D` — the big one

We do not use it, and [R-SENS-1](SPEC.md) forbids it. Three independent reasons:

1. **Wrong shape.** `angle_list = np.linspace(angle_min, angle_max, number)` is a *contiguous fan*.
   A ring of 8 separated rangers cannot be expressed as one sensor. The only native workaround is
   N separate `Lidar2D` instances.
2. **Wrong cost.** `Lidar2D.step()` is a shapely boolean `difference` of the whole beam fan against
   the obstacle union. Cost is dominated by **fixed per-sensor GEOS overhead, not per-beam cost** —
   going from 4 beams to 100 beams costs only 2.4x. So "N instances of a 1-beam Lidar2D" is the
   single most expensive way to buy sparse ranging. Measured against a vectorised numpy raycaster:
   **14-31x slower**. Profiling shows **68% of total simulation time in one GEOS `difference` call
   per lidar per step**.
3. **Wrong answers, silently.** Three confirmed defects, all reproduced during recon:
   - `calculate_range_vel()` never resets `range_data`, unlike `calculate_range()`. When the sensor
     origin ends up inside an obstacle, zero parts survive the origin filter and **the entire scan
     silently freezes at the previous tick's values, forever, with no error flag**.
   - `angle_range: 6.283185` becomes `WrapTo2Pi(2*pi) == 0.0`, collapsing every beam to 0 rad. No
     warning. Almost certainly upstream issue #184.
   - `number: 1` yields `linspace(a_min, a_max, 1) == [a_min]`, so a single-beam sensor points at
     `-angle_range/2`, not forward.

In its clean case `Lidar2D` is accurate — 1.5 mm max error against analytic ray-circle ground
truth, which is just shapely's circle polygonisation. The problem is not accuracy, it is shape,
cost and failure mode.

We replace it with one vectorised sensor per drone. See [sensor models](06-sensors.md).

### `FogMap`

`world.fog_map: true` gives a line-of-sight-revealed boolean grid with `explored_ratio`, updated
headless. It is a genuinely nice **coverage metric** and we use it as one.

It is **not a mapper**: one world-global boolean array shared by every sensing object, with no
free/occupied/unknown distinction, no log-odds, no per-agent maps and no merging. Occupancy mapping
is a thing this project exists to test, so it belongs to the policy layer, not the world.

### `env.get_map(resolution)`

Returns the **ground-truth** obstacle grid. That is the answer key, not an observation. It is
available to the evaluator and to path-planner experiments; [R-POL-3](SPEC.md) keeps it out of
`Observation`.

### Behaviours as the policy hook

ir-sim's `@register_behavior(kinematics, action)` is well-designed and we borrowed its *shape*. We
do not use it as our policy hook, for two reasons: it keys the policy on `(kinematics, name)`,
coupling strategy to dynamics; and duplicate registration raises `ValueError`, so a notebook reload
crashes. [R-POL-6](SPEC.md) registers by name alone and overwrites with a warning.

We do drive **non-ego traffic** with the built-in `rvo`/`sfm`/`wander` behaviours where a live
scene is useful.

## Landmines — all verified during recon

Kept here so nobody rediscovers them the hard way.

| # | Landmine |
|---|---|
| 1 | `sample_time < step_time` → `ZeroDivisionError` at `world.py:171`. Our loader rejects it first (R-TIME-4) |
| 2 | `env.done()` is True on **collision** as well as arrival — it conflates crash with success |
| 3 | **No arena walls exist.** A robot silently leaves the world; the rectangle is only a plot limit |
| 4 | `action_id` indexes `env.objects` (robots + obstacles + map), **not** `robot_list` |
| 5 | More actions than objects → `ValueError` from `zip(strict=True)` |
| 6 | An unknown `kinematics.name` silently becomes a **differential drive** plus a warning |
| 7 | Custom `kinematics:` YAML params raise `TypeError` — `create_kinematics` has a closed signature |
| 8 | A finite `acce` makes **every** clipped step log a WARNING — a full-throttle policy floods the log |
| 9 | `obj.z` always returns 0 and is dead code |
| 10 | Creating a second env resets the global `ObjectBase.id_iter`; `env.end()` clears `env_param.objects` |
| 11 | `env.quit()` raises `SystemExit(0)`; never enable keyboard mode in a headless loop |
| 12 | The matplotlib backend is chosen at **import** time, before you can intervene |
| 13 | `reset(random=True)` rebuilds from the **cached** parse, not the file on disk — use `reload()` |
| 14 | `EnvPlot.__init__` calls `plt.subplots()` **unconditionally**, even fully headless. Figures leak per env and are freed by neither `del env` nor `env.end()`. Call `plt.close('all')` |
| 15 | `set_seed()` reseeds **one process-global** `_rng`. Two in-process envs cannot have independent seeded streams — parallel sweeps must use separate processes (R-DET-4) |

Landmines 3, 14 and 15 are the ones that shaped our design.

## Measured performance

Apple arm64, Python 3.12.10, headless. Two independent recon agents benchmarked this; figures agree
to within run-to-run variance (±30%).

| Config | steps/s | ms/step |
|---|---|---|
| 4 robots, no sensors | 2910 | 0.34 |
| 20 robots, no sensors | 3202 | 0.31 |
| 4 robots, 32-beam lidar | 422 | 2.37 |
| 4 robots, 180-beam 360° lidar | 70 | 14.3 |
| 8 robots, 180-beam 360° lidar | 34 | 29.1 |
| 8 robots, 4 x 1-beam ToF (the naive ring) | 449 | 2.00 |

Rendering adds ~2 ms/step with Agg and no explicit draw, **~24 ms** for a live window, and
**~59 ms** with `savefig` for GIF capture. Budget accordingly.

The lesson: **ir-sim's own motion, collision and world stepping are cheap (0.3 ms/step at 20
robots). Essentially all of the cost is `Lidar2D`.** Removing it is what makes a 25-drone fleet
tractable, and it is why this project can use ir-sim rather than replace it.

## The honest counter-argument

A recon agent tasked with attacking the ir-sim decision concluded: write your own numpy sim
(~800-1500 LoC), because ir-sim has no altitude, no comms, no occupancy mapping and no structured
logs, and its sensor model is the wrong shape and 14-31x too slow.

That critique is **correct on every fact and wrong on the conclusion**, for reasons the team's own
decisions settle:

- *No comms* — we are not modelling comms (perfect blackboard).
- *No occupancy mapping* — mapping is the thing under test; it belongs to the policy layer wherever
  we build it.
- *No structured logs* — greenfield under every option, so it is not a differentiator.
- *No altitude* — we carry `z` as extra kinematics state rows, which is proven to work.
- *Sensor shape and speed* — real, and answered by replacing exactly one class rather than the
  other 85% of the library.

What survives the counter-argument and is genuinely load-bearing: the **global RNG** (so parallel
sweeps need processes) and the **figure leak** (so episodic loops need `plt.close('all')`). Both
are in the spec.

The decisive evidence is external: arXiv:2607.25195 got an IROS 2026 paper out of ir-sim 2.9.0
doing precisely this class of experiment. See [ADR-0001](adr/0001-build-on-irsim.md).
