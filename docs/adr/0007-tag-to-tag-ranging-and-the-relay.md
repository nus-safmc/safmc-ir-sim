# ADR-0007: Tag-to-tag ranging, and a relay trial built on it

**Status:** Accepted · **Date:** 2026-09-17

## Context

ADR-0006 gave the simulator a UWB tag that ranges to **anchors** — landmarks the team placed
and surveyed. Nothing ranges drone to drone. The team now wants to use UWB to **form a
relay**, with tags on the drones and one anchor at the start line.

Three facts decide what that means here.

**The relay is a scoring rule, not a radio link.** Booklet 3.3.7, implemented as R-MISS-4 and
`mission.py:_find_relay`: a chain of **landed** drones from the drone that rescued a bonus
victim to a drone inside the Start Area, every adjacent pair at most **1.0 m** apart with
mutual floor-level line of sight, doubles the whole score. It has never fired in any recorded
run, and no shipped policy can reach it (`REVIEW.md`). The docs are equally clear that the
real fleet has no drone-to-drone radio at all — every drone is a WiFi station on one laptop
(`docs/02-hardware.md`, `blackboard.py`) — so a UWB *communications* relay would be a new
hardware claim with no requirement behind it. This record is about the scoring relay.

**UWB is the right sensor for it, for one specific reason.** The flown firmware carries no
estimator — a tag sighting hard-overwrites the frame offset (ADR-0003) — so "we landed 0.8 m
apart" from two dead-reckoned poses is unverifiable in the room, where no navigation aid may
stand. A DW3000 tag-to-tag double-sided exchange measures the scored quantity itself, at a
line-of-sight standard deviation of about 5 cm (A-14), and two drones 0.8 m apart in a 2.4 m
corridor are almost always in line of sight. The spacing target is 0.8 m, not the 5–10 m of a
communications chain, which is why the through-wall terms (A-16, A-17) barely enter.

**The platform does not let a sensor see another drone's tag.** Three things stand in the
way, found by audit before this change:

1. `WorldScene` exposes teammates only as anonymous ray-hittable circles — `x, y` only,
   keyed by ir-sim id, with no altitude and no agent id — and its arrays are private
   (`scene.py`). Reaching in violates R-SENS-15.
2. `Runner._sense` feeds only `ACTIVE` robots to `refresh_drones` (`runner.py`). A relay is
   made of **landed** drones, so a tag built on the current scene would go blind to exactly
   the nodes it needs.
3. Identity, determinism and timing conventions — string ids that the log refuses, a fixed
   draw count per sweep (R-DET-2), a TDMA budget that grows with the fleet (F-32) — all need
   restating for a reading whose targets are the fleet rather than the arena.

## Decision

**1. `WorldScene` gains one query, `fleet`, and the runner fills it every tick.** A frozen
`Fleet(agent_ids, object_ids, xyz)` naming **every drone in the run, whatever its lifecycle**,
in run order, with its true `(x, y, z)`. `refresh_fleet(entries, cache_key)` is called by the
runner once per tick beside `refresh_drones`, from the same post-step state. It is a second
method rather than a widening of `refresh_drones` because the two answer different questions:
`refresh_drones` builds *bodies* under the rule "what can kill you is what you can see", and a
landed drone is not a body (the runner makes it `unobstructed`); a radio does not care whether
the airframe under it is flying, parked or wrecked. The `Fleet` type is banned from
`Observation` by the R-POL-4 walk, as `Landmark` and `WorldScene` are. What a sensor *does*
with the fleet is R-SENS-11's review obligation: a sensor that returned `fleet.xyz` would be a
leak the walk cannot see, exactly as an anchor sensor that returned every landmark would.

**2. `UWBTag` ranges to peers when asked: `UWBConfig(peers=True)`, off by default.** One radio,
one config, one TDMA budget — a second sensor would let a run range to anchors at one rate
and to peers at another on hardware that cannot. With `peers` on, the same `UWBRanges` reading
gains two fields: `peer_ids`, every agent id in the run in run order, fixed for the run, the
tag's own id included; and `peer_ranges_m`, one reported range per peer, `inf` for the tag
itself and wherever nothing was heard. With `peers` off both are empty and **every existing
run reproduces byte for byte**; with `peers` on the anchor noise is still the same run,
because the peer draws come from a child generator spawned from the tag's own at build. (The
first design drew peers from the parent after the anchors; that held within a sweep and
failed by the next one, which a test caught before this record was committed.)

**3. The physics is the ADR-0006 model applied to a second tag.** The true range is the
three-dimensional distance between the two drones' true positions — a hovering drone 0.8 m
from a landed one reads 0.94 m, and a policy must know that. Line of sight is the structural
segment test at the **lower** of the two altitudes; walls and pillar shafts are 2.0 m, so for
them the altitude does not matter below the ceiling, while a pillar's 0.15 m base obstructs a
landed tag's link and not a hovering one's — and the scorer's own floor-level line of sight
counts that base, so the lower altitude is what makes the tag agree with the rule (F-34). Airframes
and markers are transparent, as R-SENS-17 already says for anchors. Noise, bias, dropout and
outliers are A-14..A-18 unchanged; nothing about a peer link is measured differently from an
anchor link in any source this repository has, and inventing a difference would dress a guess
in precision.

**4. The sweep rate now has a peer term, and it is pessimistic on purpose.**
`peer_sweep_rate_hz(n_tags, n_anchors)`: a tag with `A` anchors and `P − 1` peers makes
`A + P − 1` exchanges in its own slot; a shipping firmware ranges to eight per 10 ms slot
(`UWB_MAX_ANCHORS_FIRMWARE`, `UWB_SLOT_S`), so it needs `ceil((A + P − 1) / 8)` slots and the
superframe is `n_tags` of them. Ten drones and one anchor: 5 Hz. Twenty-five and one: **1 Hz**.
That is A-19, and its two halves are both assumptions: that every tag initiates to every
other (a symmetric protocol needs half the exchanges) and that the exchange budget is the AT
firmware's 1.25 ms rather than the ~0.5 ms of airtime. A broadcast swarm-ranging protocol
measured 16 Hz per pair at 13–14 drones on a DW1000 (Shan et al., INFOCOM 2021), against
3.6–3.9 Hz from this budget at that fleet size — about four times, and the gap widens with
the fleet because the naive schedule is quadratic in it and a broadcast is linear. The
default is the shipping firmware because that is what the team would fly first; F-33 records
the gap. Like `sweep_rate_hz`, it is a helper the
scenario author calls and the runner never applies (F-32).

**5. The log gains `peer_ranges_m` shaped `(ticks, agents, agents)`.** Column `j` is the
`j`-th entry of the header's `agents` list. Peer ids are not stored, for the reason anchor ids
are not: the log holds numeric arrays only, and the order is already in the header.

**6. A metric says when the relay formed, not only whether.** `compute_metrics` gains
`time_to_relay_s` — the first tick at which a chain existed, from `states.npz` and the header
arena by the mission's own adjacency rule replayed at every tick a drone landed — and
`relay_chain_drones`, the length of the footer's final chain, which can be a different chain
from the first one (a later bonus rescuer landing beside the tail shortens it). Until now
`relay_formed` was a single bit read from the footer.

**7. The relay controller is an example, not a policy.** `policies/__init__.py` states the
rule: a strategy written by the people who wrote the simulator is not a baseline. The trial
lives in `examples/06_uwb_relay.py`, imports what it needs, and is graded from its own log.

## The trial: an elastic band along the head's trail

The design the trial implements, so that it can be audited against this record rather than
against the code. Notation: node `0` is the anchor, nodes `1..n` are relay drones, node `n+1`
is the **head** — the drone landed on a bonus victim. `r_{i,j}` is the *measured* UWB range
between nodes, `d*` the rest spacing (0.8 m: the 1.0 m rule less four A-14 standard
deviations), `s_i` node `i`'s arclength along the **trail**.

**The trail.** Every searcher publishes a breadcrumb — its pose and its ring's minimum range
— each time it has moved 0.25 m. A relay accumulates every searcher's crumbs from tick 0 and
**cuts loops**: when a new crumb comes within 0.6 m of an earlier crumb, and the chord plus a
body radius fits inside the ring clearance recorded at *either* crumb (capped at 0.8 m), the
trail between them is dropped. The chord then lies inside a disc a ring saw empty, so it is
flyable and in line of sight without a map. The trail a relay follows for head `h` begins at
an anchor and ends at `h`'s landing crumb (how it is chosen is amendment 5 below). Its length
`L` decides the cost: the anchor and the head are the fixed ends, so
`n_needed = ceil(L / 0.9) − 1`.

**The potential.** Each relay descends a one-dimensional potential on measured ranges,

    U_i = ½ k (r_{i,i−1} − d*)² + ½ k (r_{i,i+1} − d*)²
    ṡ_i = −k (r_{i,i−1} − d*) + k (r_{i,i+1} − d*)

with `r_{1,0}` the anchor range and `r_{n,n+1}` the range to the landed head, each reduced to
the horizontal using the relay's own altitude and the neighbour's known one (0 m landed,
cruise otherwise). Both ends are fixed, so the chain converges to **equal spacing
`L / (n + 1)`** whatever `d*` is — a discrete heat equation whose slowest mode decays as
`exp(−k π² t / (n+1)²)` — and the trail parametrisation removes the joint angles a
range-only chain cannot otherwise fix (a path graph is not rigid in the plane). The
two-dimensional command is the trail point at `s_i`, tracked with a proportional velocity
saturated at cruise, plus the wasp_v5 linear-falloff repulsion from the ring — below 0.6 m in
transit, below 0.3 m once on the trail (amendment 1).

**Two deployments, one controller.** *Dispatch*: relays wait on the ground at their grid
positions until a head has landed and its trail is known, choose the head whose trail costs
fewest drones, launch in single file — highest `s` first — to their equal-spacing targets, and
let the band trim. *Train*: the relays follow the lead's growing trail from take-off at equal
spacing between the anchor and the lead, entering the chain one at a time as `L` grows past
`(m + 1) d*`, and hold when the lead lands as head. The lead in both modes lands only on a
bonus victim; the other searchers are unmodified `wasp_v5` with the mission wrapper.

**The landing gate is UWB alone.** A relay declares itself in place when both neighbour
ranges, reduced to the horizontal, have measured at most 0.9 m for three consecutive fresh
sweeps (amendment 2); relay 1's predecessor is the anchor, so its anchor range at most 0.9 m
horizontal with the anchor at `y = 5.0` puts it at `y ≤ 5.9` — **inside the Start Area by
measurement**, not by pose. A range certifies the rule's distance; its floor-level line of
sight comes from the trail's construction and is checked by the grader, not measured. When
every relay's `in_place` is true in the same blackboard snapshot, every relay lands on the
same tick.

**What the trial must show (falsifiable, audited in C15).**

- **T-1** With one bonus victim placed 3 m north of the anchor column in the Known Search
  Area and no other targets, the dispatch trial forms a scoring relay on the seed under test
  (`relay_formed` true, `time_to_relay_s` finite).
- **T-2** The chain at the moment it formed is graded offline from `states.npz` — every link's
  floor distance and line of sight, the tail's zone — and every relay that landed by then is
  a link of it. (An auditor noted the first half is tautological once T-1 holds, because the
  chain comes from the mission's own rule; the second half is not.)
- **T-3** Every relay that landed did so with its last fresh `peer_ranges_m` to both chain
  neighbours finite and within the gate — the log, not the policy, says the gate held — **and
  a relay whose links cannot be brought inside the gate never lands**: one relay on a trail
  that needs two hovers to the end of the run. That second clause is what a gate that ignored
  the ranges would fail.
- **T-4** With `peers=False` the example refuses to run: the controller must not fall back to
  pose for spacing silently.
- **T-5** The sweep over seeds and `n_relay` reports, per cell, `P(relay)`, mean score with
  and without the relay drones searching instead, crashes, and take-off waves — and the
  write-up states the conditions (ground-truth pose for trail following, perfect blackboard
  for crumbs and consensus, A-14..A-19 unmeasured).

## Rationale

1. **Positions of every tag, not bodies of the active ones.** The alternative — extend the
   drone scene with ids and altitudes and include terminal drones — would put landed drones
   back into the ring's view and the collision check, undoing PR #8 and the "what can kill
   you is what you can see" rule. A radio and a ray disagree about a parked airframe, and the
   two views should be two methods.
2. **Every lifecycle, including crashed.** A tag is a radio, not a rotor; the electronics on
   a drone that clipped a wall keep answering. Reporting a range to a crashed drone leaks
   nothing a policy could not read from its last published pose, and dropping it would model
   a failure mode nobody has measured.
3. **Self is `inf`, and self is in the list.** A square `(agents, agents)` array whose column
   order is the header's agent list is recoverable from the log with no side table; the tag
   cannot range to itself, and `inf` already means "no measurement".
4. **One flag on one sensor.** The slot budget is shared between anchors and peers on the real
   radio (A-19). Two sensors could be configured to contradict that.
5. **The trail is a searcher's flown path, not a plan.** The room's layout is unknown by rule
   and no map exists on the drones; a relay that plans needs one. A path a teammate flew is
   flyable by construction, and cutting loops against the ring's own clearance keeps it that
   way without a map. The cost is that a Lévy walker's trail is longer than the shortest
   path, which is what the `n_needed` number reports.
6. **Equal spacing, not `d*` spacing.** With both ends fixed the spring chain's equilibrium is
   equal spacing regardless of rest length; `d*` only sets the gate. Reeling out at exactly
   `d*` would detach the tail from the Start Area whenever `L > n d*`, while equal spacing
   keeps the relay feasible up to `L = (n+1) × 1.0 m`.
7. **The anchor certifies the tail.** The one place a pose claim would otherwise decide a
   scoring predicate — "the tail is in the Start Area" — is the one place a single range can
   settle it, because 0.9 m of horizontal range from an anchor at `y = 5.0` cannot reach
   `y = 6.0`. That is the whole reason the anchor is in the trial.

## Consequences

- `sensors/scene.py`: `Fleet`, `WorldScene.fleet`, `refresh_fleet`. `runner.py`: one call.
  `sensors/uwb.py`: `peers`, `peer_ids`, `peer_ranges_m`, `peer_sweep_rate_hz`,
  `peer_line_of_sight`. `constants.py`: A-19 (`UWB_PEER_EXCHANGES_PER_SLOT`, the firmware's
  eight). `metrics.py`: `time_to_relay_s`, `relay_chain_drones`, and `relay_timeline`.
  `examples/06_uwb_relay.py`. SPEC: R-SENS-15 amended, **R-SENS-18** added, A-19 registered.
  FIDELITY: F-33..F-36.
- **Cost:** a `peers=True` run's log grows by `ticks × agents²` float32 — 12 000 × 625 × 4 B =
  30 MB at 25 drones over 600 s, before compression. `Recorder(sensor_every=...)` thins it.
- **Cost:** the R-POL-4 walk gains a banned type; a reading that carried a `Fleet` is refused.
- **Cost:** results on the relay trial are conditional on more than the tag's five numbers.
  Trail following and role assignment run on ground-truth pose; crumbs and the landing
  consensus run on the perfect blackboard (ADR-0003). The UWB-gated spacing and the tail's
  anchor certification are the two decisions that survive those caveats, and the write-up
  says so (F-36).
- **Cost:** the two-wave take-off rule (R-MISS-6) binds on any deployment that feeds relays
  through one trail entry in single file: at 0.8 m and cruise speed a drone crosses the line
  every 1.8 s, so more than about five relays span more than one 10 s window. The trial
  reports the wave count rather than choreographing an abreast crossing; that is a second
  trial.

## Amendments from the build, 2026-09-17

The trial as specified above was implemented and driven against a planted scenario and a
full run before any sweep, then audited twice and sent to a skeptic. Nine things changed,
each because a trace, an auditor or the skeptic falsified a sentence of this record. The
controller's shape did not change.

1. **The band steps only while the relay is on its carrot.** As written, the potential moved
   the arclength on every fresh sweep. The airframe answers through a 0.35 s lag (A-2) and
   sweeps come every 0.2 s at ten drones, so the band pushed again before the drone had
   answered the last push, and one relay's arclength slammed between 0 and 2.3 m every few
   sweeps. A step is now commanded only when the relay is within 0.12 m of its carrot, and
   steps under 0.02 m — A-14 through `K` — are not commanded at all.
2. **The gate is per link, not balanced.** "Within 0.10 m of each other" was a proxy for
   convergence that the noise defeated: each horizontal range carries A-14, so their
   difference has a 7 cm standard deviation and the three-consecutive rule rarely passed.
   The rule needs each link inside 1.0 m, not equal links, and both ends of every link
   check it — so every relay in place still means every link inside the rule. Equal spacing
   remains the band's equilibrium; it is no longer a landing condition.
3. **The anchor stands at tag height, 0.5 m, not on the 2.0 m tripod.** A range is
   three-dimensional and the tail reduces its anchor range to the horizontal by the height
   gap; at 0.77 m across and 1.5 m up that multiplies A-14 by `r / h` = 2.2 and the tail's
   certificate flickered. Geometric dilution, not noise. `anchor_height_m` was already a
   deployment choice; the trial sets it. **The anchor's *row* stays at `y = 5.0`**, which is
   the certificate.
4. **Chain order is the order relays reach the trail, and the relay nearest the anchor
   launches first.** Launching the farthest first made two approach paths cross — the second
   overtook the first and took its slot, so the chain's neighbours were not its spatial
   neighbours and the band never settled. With the nearest launching first each later joiner
   flies north on a column east of every earlier one and west behind them; and the chain
   reads seniority from the tick each relay reached the trail, so an overtake could not
   misorder it in any case.
5. **One certification point is expensive, and the trail is routed, not copied.** On the
   first full run a head landed on a bonus victim at `x = 14.3`, a metre north of the line;
   the chain had to come 12.5 m west along the Start Area to the single anchor to be
   certified — sixteen relays for a victim one relay from the line — and even with an anchor
   nearby, the head's *own* trail had wandered six metres east before landing. Two changes:
   the trial gains an `anchors` knob, a row along `y = 5.0` at 1.9 m spacing so a crossing
   point is never more than 0.95 m from an anchor while the row is whole — amendment 8 thins
   it on the seeds where a wall reaches into the row (rule 3.3.1 r.16 allows any number in
   the Start Area; **one remains the default and the brief**, and the sweep prices the
   row); and
   in dispatch the trail is the **shortest route through every searcher's crumbs** to the
   nearest anchor — consecutive crumbs of one searcher (a flown segment), cross-links where
   two crumbs' ring-clearance discs cover the chord with room for a body (the loop cutter's
   test between trails), and an anchor to any crumb within 0.9 m (Start Area free space) —
   by Dijkstra, once per head. Still no map: every edge is a segment a drone flew or a chord
   a ring saw empty. Train mode still follows the lead's own trail, which is one of the
   things the sweep now compares.

6. **The crumb network dropped every anchor edge between 0.6 and 0.9 m.** Its cell hash was
   0.6 m wide and the anchor's reach 0.9 m, so a head whose trail began 0.7–0.87 m from the
   anchor was declared infeasible. Found by the audit with a four-line reproduction; anchor
   edges are now computed directly. Every dispatch result before the fix was biased toward
   "infeasible", and the sweep was rerun.
7. **The north leg hands over early, staggered, and slides.** Driving a searcher at a wall
   face with attraction and repulsion alone parked it there — the room's south face can stand
   0.05 m north of the line — so the leg now hands over to wasp_v5 when the ring sees anything
   1.4 m ahead past the anchor row (at 0.9 m wasp had two seconds and lost drones at corners),
   drones start the leg 0, 1.5 and 3 s apart by index mod 3 so neighbours do not reach the
   face together and turn into each other, and `toward()` slides along an obstacle instead of
   pushing on it. The
   audit's comparison stands as a caveat: the trial's searchers reach the room sooner than the
   plain wasp_v5 baseline and meet its walls sooner; the sweep reports crashes per cell.
8. **An anchor with structure inside its disc is left out for that seed.** The generator on
   `main` lets an inner wall reach below the start line — 22 of 200 seeds, as low as
   `y = 4.8` — which the docs say cannot happen and which the network's "an anchor's 0.9 m
   disc is free" assumption relied on. `anchor_row` surveys the generated arena and drops an
   anchor with structure within 0.95 m (the disc plus half a wall), the first included unless
   it is the only one — on 3 of 200 seeds an inner wall stands 0.78–0.92 m from the lead's
   column. A first version used 1.2 m and exempted the first anchor; the skeptic showed that
   also caught the room's *legal* south face (1.05 m from the row at its nearest) and thinned
   the row on 46 seeds, two of them in the sweep, silently.
9. **A relay joins the trail at its nearest point, and slides only past obstacles nearer than
   its goal.** The skeptic found three sweep runs in which the second relay hovered 130–420 s
   before landing: aiming at the anchor *point*, its approach read the first relay — hovering
   0.9 m from the anchor, in the ±34° cone — as a wall and slid for ever, and the write-up had
   called the delay flight time. A relay now enters at the projection onto the trail's first
   1.5 m, and `toward()` slides only when the obstacle is nearer than the goal. The same three
   runs form the relay at 36, 49 and 63 s.

Also found: recomposing a trail from scratch each tick was quadratic in the crumb count and
a 600 s run took six minutes of wall time; trails are built incrementally and routes are
computed once per head. The 0.1 m gate margin covers A-14 and **not** the per-unit
antenna-delay offsets F-34 documents (12–30 cm between two tags), which on hardware must
come off `MAX_LINK_M`. A relay that crashes mid-chain keeps its last blackboard tuple for
ever, so the chain either lands around the wreck or hovers to the end — open. And two claims
in the sensor's own spec were wrong before this section was written — see C13.

## Rejected alternatives

| Option | Why not |
|---|---|
| Extend `refresh_drones` with ids, altitude and terminal drones | Puts landed drones back in the ring's view and the collision set; a radio's view and a ray's view are different questions |
| A separate `UWBPeerConfig` sensor | Two rate settings for one radio; the slot budget is shared on hardware (A-19) |
| Range only to `ACTIVE` peers | The relay is made of landed drones; blind to the nodes it needs |
| Exclude self from `peer_ids` | Column order would depend on the reader; a square array keyed by the header's agent list needs no side table |
| A per-peer line-of-sight or quality flag | Tells the policy what no tag can know (ADR-0006 rationale 1) |
| A comms-range `Blackboard` in the same change | Nothing in the rules or the hardware asks for a comms relay; R-SEAM-2 work stays separate |
| A free-space range-only potential (dither, stop-and-go, extremum seeking) | A path graph has `n − 2` free joint angles under range-only control; the trail fixes them, and the literature's fixes are slow and jittery under 10 % dropout |
| Position-based potential on ground-truth pose, UWB as a gate only | Proves nothing about UWB; the straight-line pull stalls on walls |
| Range-only relative-localisation EKF first | The right long-term seam (a UWB `PoseSource`), but a converged, stationary chain is its unobservable case; second trial |
| Ship the relay controller in `policies/` | `policies/__init__.py`: an invented strategy is not a baseline |
