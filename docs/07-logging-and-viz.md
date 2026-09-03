# Logging and visualisation

ir-sim has no state history and no replay format — `EnvLogger` is a 92-line loguru text
wrapper and the only durable artefact is a matplotlib GIF. So this is entirely ours, and it is
worth the effort: a run you cannot replay is a run you cannot debug, and a comparison you
cannot re-score offline is a comparison you cannot check.

## The log

One directory per run:

```
runs/sdlw_s3/
├── run.jsonl      header (config + full arena + versions) · events · footer (score)
├── states.npz     per-tick pose, velocity, lifecycle, command
├── tof.npz        per-tick ring scans -- one <sensor>.npz per recorded sensor
└── replay.html    generated on demand
```

`run.jsonl` line 1 is the header, then one line per event, then the footer. The header carries
the **complete resolved arena** — every wall, pillar, target and landmark, and whether it was
generated from the seed or supplied to the run — so the log is self-contained and does not
depend on the generator producing the same arena again.

```python
from safmc_sim.recorder import load_run, score_from_log
run = load_run("runs/sdlw_s3")
run["states"]["pose"]              # (T, N, 4) -> x, y, z, theta
run["states"]["lifecycle"]         # (T, N) int codes
run["header"]["sensors"]           # every sensor: name, config type, rate, recorded?
run["sensors"]["tof"]["ranges_m"]           # (T, N, 64) flattened (ranger, zone), CCW from the nose
run["sensors"]["tof"]["zone_bearings_rad"]  # (64,) the bearing of each column. Use it -- see below.
run["sensors"]["tof"]["sample_tick"]        # (T, N) the tick each row was sampled at; -1 is the pre-flight sample
score_from_log("runs/sdlw_s3")     # recomputed from the log alone
```

A sensor you add appears the same way, as `run["sensors"][name]`, if its `record()` returns
fixed-shape arrays — see [06-sensors.md](06-sensors.md#adding-a-sensor). `sample_tick` is
there so a held reading (a 2 Hz sensor between samples, a crashed drone) is never mistaken
for a fresh one.

> **`ranges_m` columns are not firmware bin indices.** They are `(ranger, zone)` order,
> anticlockwise from the nose; the firmware's `tof_scan_collapsed_t` holds the same 64 values
> indexed by absolute *clockwise* bearing. The two are a permutation of each other. Always map
> through `zone_bearings_rad`, stored in the same file — see
> [06-sensors.md](06-sensors.md#if-you-need-firmware-index-order).

Two properties the format is built for:

**Offline re-scoring must equal the online score exactly.** `score_from_log` rebuilds the arena
from the recorded geometry and re-runs the scoring rules against the final landed positions.
Keeping both paths and testing that they agree is the point: if they ever diverge, one of them
has a bug.

**Two identical runs produce logs that differ only in the `meta` block.** Every wall-clock
value is confined there, so `states.npz` is byte-identical between two runs of the same
`(scenario, seed, policy, config)`. There is a test.

Size, measured on a 12-drone 180 s run: about 0.7 MB of states and 3.2 MB of ToF — poses are
float64 so offline re-scoring is exact, and the ring is 64 float32 values per drone per tick.
Recording can be disabled (`record=False`) and is tested not to change simulation results,
with the flown sensors and with a noisy custom one.

## The replay viewer

```bash
safmc-run replay runs/sdlw_s3
# or: python tools/viz.py runs/sdlw_s3 --tof-every 10
```

A single self-contained HTML file — no server, no dependencies, works from disk. It reads
**only the log**, never the simulator, which is what makes it a replay rather than a second
rendering path that can quietly disagree with the first.

What it shows: the arena with walls, pillars, the Start Area band and the Unknown Search Area;
every drone with heading and trail; live ToF rays from the recorded scans; each target with
its 1 m scoring radius, turning green when serviced; every placed landmark, with id and kind
on hover; a scrubbable timeline with play/pause and
arrow-key stepping; the agent table with lifecycle and current command; the event log colour-
coded for crashes, scores and rule violations; and the full score arithmetic.

For long runs use `--pose-every` and `--tof-every` to control file size. Both are recorded in
the page so nothing silently misrepresents the sample rate.

## Metrics

```python
from safmc_sim.metrics import compute_metrics, summarise
m = compute_metrics("runs/sdlw_s3")
print(summarise([m1, m2, m3]))
```

Coverage is reported **two ways** because they measure different things:

- `path_coverage` — cells the drone passed within 0.1 m of. This is the metric
  arXiv:2607.25195 uses (its radius is ir-sim's `goal_threshold` default). It is a
  *path-proximity* measure, not "area sensed", and its absolute numbers are consequently tiny.
  Kept for comparability.
- `sensed_coverage` — cells actually swept by line of sight, cast with the same engine the
  sensor uses. For a search mission this is the honest one. Since C8 the free-cell
  denominator excludes solid landmarks, mission markers included, so a number recomputed
  from a pre-C8 log moves by about 0.1 % relative.

### The normalisation problem, stated honestly

Under `collision_behaviour="stop"` a crashed drone contributes nothing for the rest of the
episode, so raw coverage rewards *not crashing* as much as *searching well*. The obvious fix is
to divide by live-agent-seconds.

**It is not a clean fix.** Measured on v0.1's reference policies, dividing by live-agent-seconds
made the *crashing* run look better than the clean one — the denominator collapses while most
of the coverage has already happened, so it rewards dying early.

So: report raw coverage, per-live-minute **and** the crash count, and compare search strategies
under `unobstructed` as the control. Use `stop` to ask a different question — whether the
strategy is survivable. They are two experiments, not one, and on v0.1's evidence they can give
**opposite answers**.

> The policies that produced that measurement have since been removed — they were written by
> whoever wrote the simulator, which makes them assumptions rather than baselines. The
> methodological point survives independently of them; the numbers are recorded in
> [CHECKPOINTS.md](CHECKPOINTS.md) as history, not as a current claim.

Also reported: `crashed_agents` **and** `crash_events` separately, because the target paper
counts only distinct agents and therefore saturates at the fleet size — its k=12 figures are
"how many of the 12 died", not a collision rate.
