# The competition — SAFMC 2026 Category Swarm

Everything here is from the **2026** rulebooks. The **2027 rulebook is not public** as of
2026-08-21; safmc.com.sg/registration says only "Registration for SAFMC 2026 is closed. Stay tuned
for SAFMC 2027!" and a Wayback CDX sweep of the uploads directory returns zero 2027 assets.

Primary sources:
- [2026 Category Swarm Challenge Booklet v2.0](https://www.safmc.com.sg/wp-content/uploads/2026/02/SAFMC_2026_CAT_SWARM_CHALLENGE_BOOKLET_release_v2.pdf) (06 Feb 2026, 27 pp)
- [2026 Common Rules and Regulations v2.0](https://www.safmc.com.sg/wp-content/uploads/2022/12/SAFMC_2026_Common_Rules_and_Regulations_release_v2.pdf) (05 Jan 2026, 14 pp)

> **The "D1/D2/D3" category naming is obsolete.** 2026 renamed categories descriptively. The
> swarm search-and-rescue category was **Category E** in 2025 and is **Category Swarm** in 2026.

**This team runs Category Swarm.** Confirmed directly:
`nus-safmc/esp-everything/CLAUDE.md:7` says "Autonomous drone swarm system for the SAFMC 2026 Cat
Swarm Challenge", and that repo's final commit is `99cde05`, *"Competition over"*, dated
2026-04-02 — matching the Category Swarm challenge window of 01-02 April 2026.

## Arena

Indoor, Singapore EXPO Hall 2B. No GNSS. Netting on all sides, safety net 8 m up.

| Element | Value |
|---|---|
| Play field | 20.0 m x 20.0 m |
| Start Area | 20.0 m x 6.0 m, full width of the south edge |
| Known Search Area | 20.0 m x 14.0 m (derived; **the 2026 table withdrew this figure**) |
| Unknown Search Area | 10.0 m x 10.0 m walled room with open doorways |
| Perimeter wall | 1.5 m tall, **three sides only** (W, N, E); south edge is netting |
| Inner wall | 2.0 m tall |
| Pillar | 0.30 m diameter, 2.0 m tall, on a 0.50 m x 0.15 m weighted base |
| Min gap, wall to wall | 2.0 m |
| Min gap, pillar to pillar / pillar to wall | 1.0 m |
| **Max flight height** | **1.4 m** — "Drones are not allowed to fly over walls" |
| Wall thickness | **NOT PUBLISHED** — see assumption A-1 |

The boundary between Start Area and Known Search Area is a **virtual line**, not a wall. Crossing
it starts the run clock.

### The rules force randomisation on you

Verbatim from §3.2:

- "The layout is **not drawn to scale**."
- "The placements of the inner walls will follow the diagram, but the **exact positions and
  dimensions will NOT be given**."
- "The layout within the Unknown Search Area is **intentionally NOT shown**."
- Victim and fire placement shown is "for illustration purposes only".

This is why [R-WORLD-1 and R-WORLD-3](SPEC.md) require a *generator*, not a map file. A policy
tuned on one hand-drawn arena is measuring the wrong thing.

## Mission

10 to 25 drones, fully autonomous, centralised or decentralised, heterogeneous allowed.

1. Take off **simultaneously** from the Start Area.
2. Search for victims and fires.
3. **Extinguish a fire** by landing next to it and staying until the end.
4. **Rescue a victim** by landing next to it and staying until the end.
5. **Form a relay** from a bonus victim back to the Start Area.

Targets are **team-supplied passive markers** — the rules mandate no particular detection
modality. Each must be non-electronic, within 30 x 30 x 100 cm, under 1 kg. Officials place them;
positions and counts are unknown to the team. The Unknown Search Area is guaranteed to contain
bonus victim(s), wall(s), pillar(s) and fire(s).

This is why the team prints its own AprilTags: they own the markers.

## Scoring

| Event | Points |
|---|---|
| Regular victim rescued (4 exist) | +5 each |
| Bonus victim rescued (4 exist) | +15 each |
| Fire extinguished (4 exist) | +10 each |
| Victim within 2.5 m of an **unextinguished** fire | that victim scores **0** |
| At least one relay formed | **2x multiplier on total mission score** |

Rescue/extinguish condition: at least one drone **landed within 1.0 m** of the organiser-set
target position, **with line of sight** — "no walls or pillars on the line". Each target scores
once; extra drones add nothing. Measurement is from the *organiser's* recorded position, not the
marker, which does not move even if knocked over.

Theoretical maximum: `(4x5 + 4x15 + 4x10) x 2 = 240`.

**Relay** (§3.3.7): a chain of **landed** drones where adjacent drones are ≤1.0 m apart *and* have
mutual LOS, the head rescued a bonus victim, and the tail is inside the Start Area. One relay is
enough; two score the same.

> At ≤1 m spacing, a chain from a mid-field bonus victim to the Start Area needs roughly 8-15
> drones. **The relay is worth as much as the entire rest of the mission and is pure 2D geometry.**
> It is the single highest-return thing this simulator can help with.

Overall competition score: Video 10% + Live Presentation 40% + Mission 50%.

## Constraints that shape the simulator

| Constraint | Value |
|---|---|
| Run time | **600 s**, two runs, best counts |
| Take-off waves | **Exactly two**, each wave's last drone within 10 s of its first |
| Fleet | 10-25; **fewer than 10 forfeits the run** |
| Platform | ≤1.0 kg, fits a 30 cm cube including props |
| Autonomy | **Zero human input during the run.** Offboard compute and a GCS are explicitly allowed |
| Nav aids | ≤10 in the Known Search Area, unlimited in the Start Area, **zero in the Unknown Search Area** |
| RF | 5.7-5.9 GHz banned, **immediate disqualification**. UWB permitted |
| GPS | Not mentioned; indoors under a net — treat as unavailable |

**The nav-aid rule is the deepest technical constraint in the whole competition.** Teams may never
enter the Unknown Search Area, so no fiducials or UWB anchors can be placed in the exact 10 x 10 m
room where the highest-value targets live. Localisation there is dead-reckoning plus onboard
sensing, full stop. That is precisely the regime a sensing-focused simulator exists to explore —
and precisely what v0.1 defers by using ground-truth pose. See [FIDELITY.md](FIDELITY.md).

## 2025 to 2026 drift — the best available signal for 2027

| | 2025 Cat E | 2026 Swarm |
|---|---|---|
| Unknown Search Area | 8 x 8 m | **10 x 10 m** |
| Known Search Area dims | published | **withdrawn** |
| Hazard mechanic | Danger Zones, −2 pts | **removed** |
| Fires | — | **added**, +10, and zero out victims within 2.5 m |
| Relay | — | **added**, 2x multiplier |
| RF | IMDA general | **5.7-5.9 GHz = instant DQ** |

Stable across both editions: 20 x 20 m, 1.4 m ceiling, 10-25 drones, two waves, 10-minute runs,
land-within-1 m-with-LOS scoring, the nav-aid zoning, and the 1 kg / 30 cm platform limits.

**Volatile: the hazard and bonus mechanic layer.** Hence [R-MISS-1](SPEC.md) and the decision to
keep scoring data-driven — see [ADR-0004](adr/0004-data-driven-mission-layer.md).

## Warning: the team's existing arena spec is stale

`gazebo-slam-prototype/gazebo_environment.md` (commit `bee82c6`, 25 Feb 2026) encodes the **2025**
arena:

- Unknown Search Area 8 x 8 m — **wrong for 2026**, should be 10 x 10 m.
- "Danger Zones, 1.5 m radius" — **danger zones do not exist in the 2026 ruleset**. They were
  replaced by fires, which are scoring targets, not hazards. The 1.5 m radius appears in no
  rulebook; it is a team invention.
- Wall thickness ~0.2 m — an unlabelled team assumption; SAFMC does not publish it.

**Do not port those numbers.** They are re-derived here from the 2026 booklet.
