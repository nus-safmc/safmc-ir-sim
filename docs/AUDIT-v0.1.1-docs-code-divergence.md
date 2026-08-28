# v0.1.1 — documentation/code divergence audit

Run on 2026-08-26 against the merged tip of PR #1 (`e09566f`). External review: the whole PR
re-checked against its two sources of truth — the SAFMC 2026 booklet PDF and the flown firmware
at `nus-safmc/esp-everything@99cde05` — followed by installing the package and driving it.

## Result

**No behavioural defects were found.** Every number, every frame convention and every scoring
rule checked out. The suite passes, and the sensor model was verified against the firmware
numerically rather than by reading.

| Claim | How it was checked | Result |
|---|---|---|
| Rule constants match the booklet | Extracted the PDF; compared all ~25 | **All correct** |
| Hardware constants match the firmware | Opened each cited `file:line` | **All correct** |
| Ring reproduces the firmware geometry | Reimplemented `tof_task.c:258` and compared all 64 zones | **0.0e+00 deg error** |
| Two identical runs differ only in timestamp | Two separate processes, same seed | **1 line, 1 key** (`meta`) |
| Offline re-scoring equals online | `score_from_log` vs `RunResult` | **Exact** |
| 193 tests pass | `pytest tests -q` | **193 passed** |

One thing worth recording, because it inverts the usual direction of trust: where
`esp-everything/CLAUDE.md` disagrees with this repository about `NAV_CRUISE_SPEED_MS` (0.5 vs
0.45) and the collision thresholds (0.37/0.47 vs 0.40/0.50), **this repository is right and
`CLAUDE.md` is stale.** `docs/02-hardware.md` already says so; it was independently confirmed.

## What was actually wrong

Every defect found was **documentation describing code that no longer exists**, and all of it
traces to a single cause.

### The cause

Commit `50e2643` ("act on four blind reviews: shrink, unbreak, and close a real trap") removed
`collapsed_m` and `status` from `ToFScan`. **That removal was correct**, and its stated reasoning
independently reproduces: with the flown geometry the 64 zones map onto the 64 firmware bins
one-to-one, so the min-pool pools nothing and the collapsed scan is `ranges_m` reordered.

But the commit touched two files — `tof_ring.py` and `recorder.py` — while **eight** referred to
the removed API. The other six were left pointing at something that had ceased to exist. No test
could catch it: nothing fails because a markdown table is wrong, and the shipped example uses the
correct API, so anyone copying the example was unaffected.

### The defects

**1. A policy written from the primary guide crashed.**
`docs/05-policy-api.md` — "the only file you need to read" — listed `obs.tof.collapsed_m`.
Reproduced: a policy using it dies with
`PolicyError: policy 'doc_trap' for drone_00 raised at tick 25 (t=1.25s)`.

**2. `docs/06-sensors.md` documented three members that do not exist** — `scan.status`,
`scan.collapsed_m` and `scan.as_firmware_frame()` — and then explained the handedness of the
collapsed scan at length.

**3. `SPEC.md` carried three unmet MUSTs.** R-SENS-4 (per-zone status) and R-SENS-5 (64-bin
collapsed scan) require removed features. R-SENS-1 requires the ring to be *an ir-sim sensor
object*; it is deliberately not one, and
`test_each_drone_carries_exactly_one_ring_owned_by_the_runner` asserts the opposite. That test
cites R-SENS-1 while contradicting it.

**4. Four places mislabelled the recorded data.** `recorder.py` and `tools/viz.py` both called
`tof.npz` "the 64-bin collapsed scan"; `docs/07-logging-and-viz.md` documented the array as
`run["tof"]["collapsed_m"]`, a key that does not exist. It is the flattened `ranges_m` in
`(ranger, zone)` order, anticlockwise from the nose — a *permutation* of the firmware's
absolute-clockwise-bearing array, not the same array. Same numbers, different slots.

Worst of the four, `docs/08-porting-to-ros.md` listed the ToF product under **"what a port does
not have to reimplement"**, asserting it was "already `tof_scan_collapsed_t`, same 64 bins, same
clockwise ordering". Following that on real hardware yields a scan rotated by an amount that
varies with bearing — plausible-looking and wrong. This was the highest-consequence instance,
because it is the one that reaches a flying drone.

**5. `FIDELITY.md` F-3 described a status channel that does not exist** ("our sensor emits only
status 5 or 255"). It emits no status at all.

**6. Dead imports.** `tof_ring.py` still imported `TOF_COLLAPSED_BINS`, `TOF_STATUS_VALID`,
`TOF_STATUS_NO_RETURN` and `dataclasses.field`, none used — residue implying a status model that
is not implemented.

### Unrelated defects found in the same pass

**7. Every documented command was Unix-only.** `README.md` and `REVIEW.md` both instructed
`.venv/bin/…`; on Windows the layout is `.venv\Scripts\`. The quick start and the thirty-minute
review pass both failed verbatim on Windows.

**8. The test-runtime figure was platform-specific.** "~37 s" was measured on Linux at `50e2643`.
On Windows the same suite took 350 s cold and 139 s warm across two runs here — several times the
quoted figure, and highly variable. `REVIEW.md §6` already flags test speed as a risk; it had
already materialised on one platform.

**9. `_validate_gaps` had no docstring.** Its explanation sat *after* two assignments, making it a
dead string expression — `__doc__` was `None`, and the reasoning about which gaps are exempt was
silently lost.

**10. `R-DRONE-6` was cited but withdrawn.** `test_audit_regressions.py` cited it; SPEC's numbering
skips 5 → 7. It mandated a 0.5 m default cruise altitude and was withdrawn in `f7a2283` because
altitude holding is guidance, and guidance belongs to the policy.

**11. `REVIEW.md §5D` understated the code volume by 86%** — "~2,600 of simulator" against an
actual 4,838 lines in `src/safmc_sim`. The docs-to-code ratio is 0.64:1, not the ~1.1:1 implied,
and the figure is presented as the basis for a decision.

**12. Commit `50e2643`'s message claims it removed "the REFERENCE ONLY constants block".** It did
not touch `constants.py`; the block was created in `3c389cb` and is still at `constants.py:119`.
*(Recorded, not fixed — history is immutable.)*

## The fixes

Docs were aligned to the code rather than the code to the docs, because the original removal was
right.

| # | Fix |
|---|---|
| 1 | `docs/05-policy-api.md` — Observation table now lists the real four members |
| 2 | `docs/06-sensors.md` — "What you get" rewritten; added a verified recipe for firmware index order |
| 3 | `docs/SPEC.md` — R-SENS-1 amended, R-SENS-4/5 withdrawn, each with the reasoning and the commit. IDs kept, not deleted, so citations elsewhere still resolve |
| 4 | `recorder.py`, `tools/viz.py`, `docs/07-logging-and-viz.md`, `docs/08-porting-to-ros.md` — labels corrected, real npz keys documented, and the porting note now says the conversion is a required reindex rather than a no-op |
| 5 | `FIDELITY.md` F-3 — restated in terms of what the sensor actually reports |
| 6 | `tof_ring.py` — four dead imports removed |
| 7 | `README.md`, `REVIEW.md`, `examples/` — activate the venv once, then bare commands; Windows path given inline |
| 8 | `README.md`, `REVIEW.md` — runtime given for both platforms |
| 9 | `arena.py` — docstring moved above the assignments |
| 10 | `test_audit_regressions.py` — stale citation replaced with the reason for the withdrawal |
| 11 | `REVIEW.md §5D` — corrected to measured line counts |

### Deliberately not fixed

**`_validate_reachability` is weaker than the invariant it advertises.** It tests a *square*
window (accepting cells up to 1.485 m, 48% beyond the 1 m scoring radius) and never checks line of
sight, although scoring requires it. So it can pass a target that is reachable but unscoreable.

Probed across 30 seeds × 12 targets with a strict circle-and-LOS test: **0 affected.** The
generator's clearances currently mask it. Left alone because tightening it is a behavioural change
to arena acceptance and deserves its own decision — but it should be tightened before target
counts are raised or clearances reduced.

## A finding for the team, not a defect

Assumption **A-4** (marker detection range, 3.0 m, unmeasured) was swept at 1.5 / 3.0 / 6.0 m,
12 drones, 3 seeds each, one search policy:

| A-4 range | mean score | mean crashes |
|---|---|---|
| 1.5 m | 13.3 (0.50x) | 3.0 |
| 3.0 m *(default)* | 26.7 (1.00x) | 0.7 |
| 6.0 m | 15.0 (0.56x) | 4.0 |

Not a scaling factor — an **inverted U**, and the mechanism is the crash column. Too little sight
and drones wander into obstacles; too much and they converge on the same distant marker and
collide with each other. Detection range behaves as a *fleet-coordination* parameter.

The policy used was written and tuned at the 3.0 m default, which is very likely why the peak sits
there. That confound is the actual warning: **a wrong A-4 does not merely scale results, it
silently tunes every policy written against it toward the wrong regime**, invisibly, because
everything looks healthy at the value you tuned at. `REVIEW.md §7` says measure A-4 first. This is
a stronger reason than the one given there.

*(Directional only: n=3, one policy, 180 s runs. Not a benchmark.)*
