# v0.1 adversarial spec audit

Run on 2026-08-21 against commit `d359b9c`. Seven independent auditors, one per area of
[SPEC.md](SPEC.md), each required to locate the implementing code, locate the regression test,
and **attempt falsification** by writing and running its own scripts. Every reported violation
was then handed to a separate skeptic instructed to refute it.

57 agents, ~5.3M tokens, 1721 tool calls.

## Result

| | |
|---|---|
| Requirements audited | 71 |
| SATISFIED | 39 |
| UNTESTED (code correct, no regression barrier) | 18 |
| VIOLATED | 14 |
| Extra bugs reported outside any requirement | 36 |
| Claims sent to skeptics | 50 |
| **Survived refutation** | **24** |

Half of the reported claims did not survive independent verification, which is roughly what
you want from an adversarial process — enough pressure to find real defects, enough scepticism
that noise does not reach the fix list.

## The defects that mattered

Ordered by how badly each corrupted results.

### 1. A landed drone was not immobile

`R-DRONE-10` says a `LANDED` drone must be permanently immobile. The runner zeroed its
*command*, but velocity lives in the state and decays through a first-order lag — so a drone
that landed while still moving kept sliding, measured at up to **116 mm, 12% of the 1 m
scoring radius**.

That is not cosmetic. An auditor produced a case where a drone touched down **1.021 m** from a
bonus victim — outside the scoring radius, target not serviced — then slid to 0.906 m and
**scored 15 points it had not earned**. It also broke offline re-scoring (`R-MISS-8`), because
the online scorer latched the award at the landing tick while the offline scorer read the
final frame and saw the drone outside the radius.

Worse, `mission.py` documented the false invariant as fact: *"a landed drone never moves — so
the set only grows"*. The whole recompute-from-scratch scoring design rested on it.

**Fixed** by zeroing the velocity state rows on every terminal transition, and by making
mission servicing explicitly latched so scoring no longer depends on a physics detail in
another module.

### 2. The landing test was vacuous

Every drone landed on the same tick, the runner ended the run as soon as all agents were
terminal, and the test's "now try to take off again" branch never executed. An auditor proved
it by mutation: a **full drone-resurrection patch passed all 533 tests**.

**Fixed** — and the replacement was itself vacuous on the first attempt, because drones that
land from a stationary hover slide ~0 mm. It now keeps one drone flying so the run outlives
the landings, lands the rest *at cruise speed*, and asserts bit-identical positions across
every subsequent tick. Verified by mutation: reverting the freeze makes it fail.

### 3. The Unknown Search Area was 1.95 m from the perimeter in every seed

`R-WORLD-2` requires the published 2 m minimum wall gap. Gaps are between wall **faces**, but
the generator placed the room's **centre line** at `min_gap`, losing `thickness/2`. Validation
never caught it because `_validate_gaps` only ever compared inner walls against structure —
the room and the perimeter were never compared to each other at all.

**Fixed**, and this corrects a claim I made earlier: I had concluded the room's north-south
position was *forced*. It is not — that derivation required a 2 m gap on the room's south
side, which faces the Start Area's **virtual boundary line**, not a wall. The room actually has
~1.9 m of freedom. The strategic conclusion (a ~2 m corridor ring, where wall-following wins)
survives; the geometry claim did not.

### 4. A policy could re-aim its own sensor, permanently

`Observation` and `ToFScan` are frozen dataclasses, which blocks rebinding but **not in-place
numpy writes**. `ToFScan.zone_bearings_rad` handed out the sensor's own persistent array, so a
policy doing `obs.tof.zone_bearings_rad[:] += 0.5` permanently re-aimed the ring — while
`_collapsed_bins` kept the old mapping, so bins and bearings silently disagreed for the rest of
the run. Reproduced end-to-end into the recorded log. Violates `R-POL-2`.

**Fixed** with read-only views.

### 5. Blackboard publications were stored by reference

A policy that published a mutable value kept a live handle on it. Mutating it mid-tick changed
what agents stepped *after* the publisher read, while agents stepped *before* saw the old
value — index-order dependence, which is exactly what the double buffer exists to prevent
(`R-POL-8`). Invisible with immutable values, which is why the existing test missed it.

**Fixed** by deep-copying on publish.

### 6. `unobstructed` was not a control

The mode exists so search strategy can be compared without crash confounds — the thing
[07-logging-and-viz.md](07-logging-and-viz.md) says to use as the control. But mission markers
stayed lethal in it, so drones still died. Separately, disabling ir-sim's collision let drones
leave the world entirely: one was measured **55 m into a 20 m field**, silently corrupting every
coverage figure computed against the field's free area.

**Fixed**: markers respect the collision mode, and leaving the field is a `CRASHED` event with
its own reason in the log.

### 7. Smaller, still real

| Defect | Fix |
|---|---|
| A `NaN` in a command poisoned drone state and surfaced as `GEOSException: Points of LinearRing do not form a closed linestring` — no agent, no tick, no policy line | `PolicyError` naming the agent, tick and field |
| `json.dumps` wrote bare `Infinity`/`NaN` — **not valid JSON**; strict parsers reject it and the replay page rendered blank | non-finite floats encode as `null` |
| Pose recorded as float32, quantising a 20 m coordinate enough to flip a 1 m scoring decision offline | float64 |
| Marker detection had an altitude cut-off copied from the ToF ring, making a drone above 1.0 m **completely blind** — although the ceiling is 1.4 m and the real camera is pitched 45° nose-down, so it sees *below* itself | cut-off removed |
| Lifecycle transitions fell through to the flying branch for any unrecognised state | raises (`R-DRONE-9`) |
| `RunConfig.record` was decorative — never read | passing a recorder with `record=False` is now an error |
| A run aborted by `PolicyError` leaked an ir-sim env and a matplotlib figure per attempt | teardown in `finally` |
| `Observation.velocity_xy` bypassed the `PoseSource` seam and was always ground truth — a drifting pose source would still have leaked an exact motion signal | routed through `PoseSource.velocity_of` |
| `wrap_pi` returned exactly `-pi` for one representable double just above `pi`, outside its own declared half-open interval | clamped |
| The log's lifecycle and command integer codebook lived only in the code that wrote it | codebook travels in the log header |

## Two corrections to the specification itself

The audit was against the spec, so where the spec was wrong, the spec changed.

**`R-DRONE-9` listed an `ARMED` state.** Nothing implemented it, because no command arms
without taking off — the firmware's arm/offboard/climb sequence is atomic from a policy's point
of view. It was specified and never built. Removed rather than added: adding it would have
created a state with no transitions a policy could distinguish.

**`FIDELITY.md` F-3 claimed the firmware's 0.40 m "unreliable return" substitution was
modelled, off by default.** It is not modelled at all — our sensor emits only status 5 or 255,
never the firmware's unreliable case. The entry now says so, and flags that a policy tuned here
will meet phantom obstacles near glass and dark surfaces that this simulator never produces.

## What is still open

Eighteen requirements came back **UNTESTED** — correct code, no regression barrier. The cheapest
have been closed with guard tests (published constants, default tick rate, one-sensor-per-drone,
the units convention, the `PoseSource` seam). The rest are recorded here rather than quietly
dropped:

- **`R-POL-7`** — a policy gets its own `Generator`, but nothing *prevents* it calling
  `numpy.random` directly. Enforcement would need a check around `step()`.
- **`R-DET-4`** — parallel sweeps use separate processes, which is correct, but no test would
  catch a regression to in-process environments.
- **`R-SENS-11`** — no test asserts a sensor cannot reach another agent's state.
- **`R-TIME-2`** — nothing would catch a `time.sleep(dt)` inserted into the tick loop.
- **`R-DRONE-7`** — the spec says "default cruise speed 0.45 m/s", and the implementation binds
  that to `QuadParams.speed_max_ms`, which is a *cap* rather than a cruise speed. Worth a
  deliberate decision rather than a test.

## Reproducing

The audit was a `Workflow` run; per-agent transcripts and every falsification script are under
the session's `subagents/workflows/` directory. Each finding above carries a reproduction in
the audit output, and each fixed defect now has a named regression test in
`tests/test_audit_regressions.py`.
