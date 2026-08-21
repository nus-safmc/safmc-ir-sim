# Logging and visualisation

ir-sim has no state history and no replay format — `EnvLogger` is a 92-line loguru text
wrapper and the only durable artefact is a matplotlib GIF. So this is entirely ours, and it is
worth the effort: a run you cannot replay is a run you cannot debug, and a comparison you
cannot re-score offline is a comparison you cannot check.

## The log

One directory per run:

```
runs/frontier_s3/
├── run.jsonl      header (config + full arena + versions) · events · footer (score)
├── states.npz     per-tick pose, velocity, lifecycle, command
├── tof.npz        per-tick 64-bin collapsed scans
└── replay.html    generated on demand
```

`run.jsonl` line 1 is the header, then one line per event, then the footer. The header carries
the **complete resolved arena** — every wall, pillar and target — so the log is self-contained
and does not depend on the generator producing the same arena again.

```python
from safmc_sim.recorder import load_run, score_from_log
run = load_run("runs/frontier_s3")
run["states"]["pose"]        # (T, N, 4) -> x, y, z, theta
run["states"]["lifecycle"]   # (T, N) int codes
run["tof"]["collapsed_m"]    # (T, N, 64)
score_from_log("runs/frontier_s3")   # recomputed from the log alone
```

Two properties the format is built for:

**Offline re-scoring must equal the online score exactly.** `score_from_log` rebuilds the arena
from the recorded geometry and re-runs the scoring rules against the final landed positions.
Keeping both paths and testing that they agree is the point: if they ever diverge, one of them
has a bug.

**Two identical runs produce logs that differ only in the `meta` block.** Every wall-clock
value is confined there, so `states.npz` is byte-identical between two runs of the same
`(scenario, seed, policy, config)`. There is a test.

Size, roughly: a 12-drone 180 s run is ~80 KB of states and ~280 KB of ToF. Recording can be
disabled (`record=False`) and is tested not to change simulation results.

## The replay viewer

```bash
safmc-run replay runs/frontier_s3
# or: python tools/viz.py runs/frontier_s3 --tof-every 10
```

A single self-contained HTML file — no server, no dependencies, works from disk. It reads
**only the log**, never the simulator, which is what makes it a replay rather than a second
rendering path that can quietly disagree with the first.

What it shows: the arena with walls, pillars, the Start Area band and the Unknown Search Area;
every drone with heading and trail; live ToF rays from the collapsed scan; each target with
its 1 m scoring radius, turning green when serviced; a scrubbable timeline with play/pause and
arrow-key stepping; the agent table with lifecycle and current command; the event log colour-
coded for crashes, scores and rule violations; and the full score arithmetic.

For long runs use `--pose-every` and `--tof-every` to control file size. Both are recorded in
the page so nothing silently misrepresents the sample rate.

## Metrics

```python
from safmc_sim.metrics import compute_metrics, summarise
m = compute_metrics("runs/frontier_s3")
print(summarise([m1, m2, m3]))
```

Coverage is reported **two ways** because they measure different things:

- `path_coverage` — cells the drone passed within 0.1 m of. This is the metric
  arXiv:2607.25195 uses (its radius is ir-sim's `goal_threshold` default). It is a
  *path-proximity* measure, not "area sensed", and its absolute numbers are consequently tiny.
  Kept for comparability.
- `sensed_coverage` — cells actually swept by line of sight, cast with the same engine the
  sensor uses. For a search mission this is the honest one.

### The normalisation problem, stated honestly

Under `collision_behaviour="stop"` a crashed drone contributes nothing for the rest of the
episode, so raw coverage rewards *not crashing* as much as *searching well*. The obvious fix is
to divide by live-agent-seconds. Measured on seed 3, 12 drones, 120 s:

| policy | mode | sensed | live agent-s | per live-minute | crashed |
|---|---|---|---|---|---|
| `frontier` | stop | 0.616 | 315 | **0.117** | 10 |
| `frontier` | unobstructed | 0.656 | 1197 | **0.033** | 0 |
| `sdlw` | stop | 0.459 | 1354 | 0.020 | 0 |
| `sdlw` | unobstructed | 0.459 | 1354 | 0.020 | 0 |

**The normalisation is not a clean fix.** Dividing by live-agent-seconds makes the crashing run
look four times *better*, because the denominator collapsed while most of the coverage had
already happened. It rewards dying early.

So: report raw coverage, per-live-minute, **and** the crash count, and compare search
strategies under `unobstructed` as the control. Use `stop` to ask a different question —
whether the strategy is survivable. They are two experiments, not one.

(`sdlw` is identical in both modes because it never collides, which is a useful check that the
two modes coincide when they should.)

Also reported: `crashed_agents` **and** `crash_events` separately, because the target paper
counts only distinct agents and therefore saturates at the fleet size — its k=12 figures are
"how many of the 12 died", not a collision rate.
