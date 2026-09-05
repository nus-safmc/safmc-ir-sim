# ADR-0006: A UWB ranging tag as a first-class sensor

**Status:** Accepted · **Date:** 2026-09-04 · **Amended:** 2026-09-05 (the part is chosen)

## Context

*(Written before the part was chosen. The addendum at the foot of this record supersedes the
hardware framing below: the part is the Qorvo DW3000, and several figures here are DW1000
ones. The decision itself stands.)*

ADR-0005 made a sensor one file on one contract and shipped `examples/03_custom_sensor.py`, a
range-only "beacon" sensor that is explicitly a template and not a model of anything (F-22).
The team now wants the real thing: an ultra-wideband ranging tag that a policy can be written
against, that a future `PoseSource` can consume, and whose every number says where it came
from.

Three facts shape it. The rulebook permits UWB by name (Category Swarm booklet §6.3) and says
exactly where a navigation aid may stand: any number in the Start Area, at most ten in the
Known Search Area, none in the Unknown Search Area, each within 1 m x 1 m with no height
limit, secured and not hung from overhead (§3.3.1 r.14–17) — a tripod, in practice. The
flown airframe carries no UWB module, so nothing here is a claim about hardware the team
owns. And the module the team would buy is unchosen, so the physics comes from the
literature on DW1000/DW3000-class hobby modules (DWM1001, Bitcraze Loco, Pozyx, MaUWB)
rather than from a bench.

What that literature supports, with its thinness stated: line-of-sight ranging noise of a
few centimetres (DW1000 timestamp noise 3–4.5 cm; testbeds 2–8 cm), independent of distance
once the antenna delay is calibrated; through one wall or obstacle a positive bias of about
0.15 m with a spread of about 0.42 m, sometimes negative, and behind several walls about
0.58 m and 0.61 m — measured on a DW1000 in a flat of pre-stressed concrete panels; a heavy
positive tail of outliers whose frequency nobody has published for these modules; a
datasheet reach of 60 m that hobby firmware delivers as 12–20 m indoors; and ranging that
is always sequential — one anchor at a time in a 3–4 ms TDMA slot; PANS returns four ranges
per tag per 100 ms frame, MaUWB-class firmware eight per slot. Failed ranges are omitted or
masked by every module surveyed; none reports one as a valid measurement, which is what
`inf` renders.

## Decision

**One module, `sensors/uwb.py`, on the ADR-0005 contract.** `UWBConfig` builds a `UWBTag`
whose reading is `UWBRanges(anchor_ids, anchor_xyz_m, ranges_m)`: every landmark of the
configured kind (default `"uwb_anchor"`; a mission kind is refused, because a tag that ranged
to the markers would report every target's true position), in arena order, fixed for the
run, with one reported range per anchor and `inf` where no measurement was obtained. It is
**not** in `flown_sensors()`; a run opts in with
`RunConfig(sensors=flown_sensors() + (UWBConfig(),))`.

**The reading carries the surveyed anchor positions.** A tag in a DRTLS network is configured
with its anchors' coordinates and reports them beside each distance (`dwm_loc_get` returns
`an_pos` with `dist`); the positions are the team's own survey, written into the same
`ArenaConfig` that placed the anchors. Reporting them from the sensor keeps one source of
truth — a policy that had to copy the coordinates into `policy_config` would drift from the
arena silently. Anchor height is one number on the config (`anchor_height_m`, default 2.0 m,
a tripod at the height of the inner walls): the sim is 2.5D, a drone's altitude comes from
PX4 rather than from UWB, and a per-anchor height would need a `Landmark` subclass whose
extra field the log would drop.

**Range is three-dimensional, obstruction is by structure only, at the tag's altitude.** The
true range is the Euclidean distance from the drone's true position to the anchor at its
mount height, because a 2.0 m anchor 3 m away reads 3.35 m and a policy must know why. Line
of sight is the mission's R-MISS-2 segment test, height-gated per R-SENS-6, against a new
`WorldScene.structural_scene` — walls and pillars only. A mission marker is a cardboard box
and a teammate is a 30 cm airframe; radio goes through both, so neither blocks. The test is
made at the drone's altitude, which is exact while anchors stand no taller than the 2.0 m
inner walls and pessimistic above that.

**Noise is a three-part mixture, every parameter a named assumption.** In line of sight the
reported range is the true range plus Gaussian noise (A-9, 0.05 m). Obstructed, it is dropped
with probability A-12 (0.10) and otherwise biased by A-11 (+0.15 m, spread 0.40 m). With
probability A-13 any reported range gains a positive uniform error of up to 1.5 m — off by
default, because the tail is documented and its frequency is not. Anything beyond A-10
(20 m) is `inf`. A reported range is never negative. Every draw comes from the sensor's own
generator and the same number of draws is made per sample whatever the geometry, so the
noise stream is a function of the seed alone.

**Sampling is a synchronous sweep at 10 Hz.** Every anchor is measured in the same tick. The
real tag measures them one after another, but a sweep of up to about twelve at 3–4 ms each
fits inside one 50 ms tick, and the first-to-last skew — under 40 ms for ten — is under 2 cm
at cruise speed, below the noise. F-23.

**Recorded as `uwb.npz`.** `ranges_m` stacked to `(ticks, agents, anchors)`, anchor
positions as a static array. Anchor ids are not stored — the log refuses string arrays — and
do not need to be: column `j` is the `j`-th landmark of the sensor's kind in the header's
landmark list, which the recorder writes in the same order the sensor reads.

**The nav-aid rule is a check, not a referee.** `validate_nav_aids(arena, kinds)` in
`world/arena.py` refuses a layout the booklet would not allow, taking the kinds that count as
aids from the caller because the arena does not know a kind's meaning (ADR-0005). The runner
never calls it. A run with an anchor in the Unknown Search Area answers a real question —
what would perfect localisation there be worth? — and the check exists so that nobody quotes
such a run without knowing that is what it was.

## Rationale

1. **Range-only, and nothing a tag would not know.** No bearing, no quality factor, no
   line-of-sight flag. The whole difficulty of UWB indoors is that a biased range looks like
   a clean one; a reading that told the policy which ranges were obstructed would delete the
   problem the sensor exists to pose.
2. **Boolean obstruction rather than wall-counting.** The literature gives one number pair
   for "behind a wall or obstacle" and one four times larger for "behind several"
   (Kolakowski & Modelski, Table 1), both through pre-stressed concrete panels; it says
   nothing about the venue's walls. A pair applied per crossing would dress an unknown in
   precision. The raycaster's private helpers can count segment hits once someone has
   ranged through the venue's walls, and the multi-wall pair is the number to start from.
3. **Pessimistic defaults where the direction matters.** A 20 m reach keeps the whole field
   within reach of three Start Area anchors, but only just; at 12 m the far third of the
   field hears nothing from the Start Area. *(The addendum sharpens this: on the DW3000 the
   figure is a firmware setting more than a chip limit — 20 m stock, past 90 m indoors at
   850 kb/s with a long preamble. F-29.)* Over-estimating localisation coverage is the mistake that survives to the
   live run; under-estimating it is caught on the bench.
4. **Outliers off by default, like ToF noise.** A component with no published rate should
   not silently shape a default result. It is one config field away.
5. **No `obs.uwb` shorthand and no `PoseSource`.** The shorthands are for the flown pair
   (ADR-0005). A UWB-fed pose source is the highest-value single class in the project
   (ADR-0003) and it is the next piece of work, not this one: the sensor has to exist before
   anything can consume it.

## Consequences

- `sensors/uwb.py`, `UWBConfig` / `UWBRanges` / `UWBTag`; `WorldScene.structural_scene`;
  `validate_nav_aids`; constants A-9..A-13 and `NAV_AID_*`; `examples/04_uwb_ranging.py`,
  which places a legal anchor set, runs the flown suite plus the tag, and grades the sensor
  offline from the log alone.
- `Observation` is unchanged. R-POL-3 is amended to say that a surveyed nav aid's position
  is the team's configuration, not a leaked world position.
- Every number is an assumption with an ID, and A-10 is the one to measure first for this
  arena: whether the module reaches 12 m or 20 m decides whether the far third of the field
  is covered from the Start Area at all.
- **Cost:** a run whose arena holds `uwb_anchor` points cannot be flown without the sensor
  (R-WORLD-8 refuses an unperceivable point). Drop the anchors for a blind control: a
  footprint alone is still refused, and a footprint plus a height makes them bodies the ring
  sees and a drone can hit.
- **Cost:** results on this sensor are conditional on five unmeasured numbers, on walls of
  unknown material, and on a module the team has not bought. `docs/FIDELITY.md` says so
  (F-23..F-27).
- **Cost:** an anchor placed at fixed coordinates in the Known Search Area can land inside a
  generated wall unless it has a footprint; the example gives its Known-Area anchors a
  0.25 m tripod base so the generator draws around them.

## Rejected alternatives

| Option | Why not |
|---|---|
| Keep UWB as the template in `examples/03` | Asked for explicitly; and the template's numbers are invented, which F-22 already says |
| Count wall crossings for the NLOS bias | No published per-wall numbers for these modules; a boolean with cited one-wall numbers is honest, the count is one helper away when measured |
| Anchor positions through `policy_config` | Two copies of the survey that can disagree; the real tag reports them from its network configuration |
| A `UWBAnchor(Landmark)` subclass with a per-anchor height | The log records base fields only, so the height would not round-trip; one config height serves a 2.5D sim |
| Obstruct with `static_sensing_scene` (markers block) | A 1.0 m cardboard marker would block radio at cruise altitude and not above it — the height gate is right for light, wrong for radio |
| Sequential per-anchor sampling in TDMA slots | A full sweep fits in one tick and the skew is below the noise; A-8 already accepts the same simplification for the ring |
| Enforce the nav-aid rule in the runner or in arena validation | The arena does not know which kinds are aids; and the simulator reports rather than referees — the two-wave rule is scored the same way |
| A line-of-sight flag in the reading | Tells the policy what no tag can know |
| A quality factor per range | Real kit reports one (F-27), but it does not reliably say whether a range is biased; left out so nothing here leans on it |
| A UWB `PoseSource` in the same change | Different seam, different audit; the sensor must exist first |

---

## Addendum, 2026-09-05: the part is the Qorvo DW3000

The original decision modelled "DW1000/DW3000-class hobby modules" because no module had been
chosen. The team has now chosen the **Qorvo DW3000**. That does not change the contract, the
reading, or the shape of the model; it changes what every number in it is *about*, and it
adds one fact the rules care about.

**What the choice settles.**

- **The rules are satisfied, provably.** §6.3 bans wireless transmission in 5.7–5.9 GHz on
  pain of immediate disqualification and permits ultra-wideband in the same sentence. The
  DW3000 has exactly two channels — 5 at 6489.6 MHz and 9 at 7987.2 MHz, each 499.2 MHz wide
  — so its nearest band edge, 6240.0 MHz, is 340 MHz clear of 5900 MHz, and no configuration
  tunes it into the banned band. Worth noting it does *not* offer channel 7, whose wider
  bandwidth about the same centre would have reached down to 5948.8 MHz. `constants.py`
  carries the numbers and a test asserts the arithmetic.
- **Three priors were wrong.** The datasheet's headline "10 cm" is marketing copy from its
  first page; the electrical specification is ±6 cm calibrated, ±15 cm uncalibrated, and a
  1.5 cm ranging standard deviation. The DW3000 is **not faster** than the DW1000 — per-frame
  airtime is essentially the same — and it is **shorter-ranged**, having dropped the DW1000's
  110 kb/s long-range mode. Its real gains are power and 802.15.4z security.
- **The sweep rate depends on the fleet, not the anchors.** TDMA slots are per *tag* and every
  anchor is ranged inside one, so ten drones get 10 Hz each and twenty-five get 4 Hz on the
  same radio. `sweep_rate_hz()` computes it; nothing applies it automatically, because a
  sensor config knows nothing about `RunConfig.n_drones` and wiring the two together would
  make a sensor's timing depend invisibly on the fleet. F-28.

**What it does not settle, and this is the honest part.** The defaults barely moved, because
the DW3000 evidence is thinner than the DW1000 evidence it replaces:

- **No DW3000 through-wall measurement in the form this model needs.** Two independent
  literature searches disagreed on whether one exists at all. What is citable is an aggregate
  mean absolute error, and the *spread* remains a DW1000 number from a paper this repository
  read in full. That mixed joint is F-28, and it is the weakest thing in the model.
- **The model is measurably optimistic.** Against the one independent measurement of the part
  — mean absolute error 5.7 cm in line of sight and 46.7 cm obstructed, 90th percentiles
  13.7 cm and 129.5 cm — this model at its defaults gives 4.0/34.2 cm and 8.2/70.3 cm. The
  body is within a factor of two; the tail is out by 1.8×, because a Gaussian tail is not a
  UWB tail. The `outlier_probability` term exists to fix exactly that and is switched off for
  want of a published rate. F-30 records it and a test pins it, so it cannot drift quietly.
- **Antenna-delay calibration is a per-unit constant this model has no term for.** ±15 cm
  uncalibrated against ±6 cm calibrated, and a DWM3000 ships uncalibrated while a DWM3001C
  ships factory-calibrated — so which module is bought moves the error by more than A-9 does.
  F-31.

**Consequence for anyone quoting a result.** "Given a DW3000" is now a meaningful clause, and
it is narrower than it sounds: the geometry is the part's, the line-of-sight noise is
bracketed by two measurements of it, and the through-wall behaviour is still half inherited
from a different radio. Nothing here has been measured on the team's own hardware in the
competition hall, which is what A-9 through A-13 exist to ask for.

