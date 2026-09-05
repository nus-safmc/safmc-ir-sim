# Building a map library

**Start here if you want to generate a set of arenas once, look at them, put your own objects in
them, and then run against them repeatedly.**

Generating a fresh arena inside every run models something that does not happen. The organisers
build the field once. Teams walk the Known Search Area during setup, see what is in it, and place
their navigation aids. Then two runs are flown on that same field (3.3.1 r.2). The simulator
should be driven the same way round: **generate, survey, place, start.**

Nothing new is needed to hold a map. An `ArenaSpec` is already frozen, validated, hashable and
reusable, and `run()` takes one directly. It *is* the map object.

---

## 1. Generate, once

```python
from safmc_sim.world.arena import generate_arena
from safmc_sim.world.store import save_arena

for k in range(10):
    save_arena(generate_arena(0, layout_seed=k, unknown_seed=7), f"maps/known_{k:02d}.json")
```

Ten different Known Search Areas, each with the *same* interior maze. Or hold the known area and
vary what is behind the doorways:

```python
[generate_arena(0, layout_seed=3, unknown_seed=j) for j in range(20)]
```

### The three streams

| Stream | Controls | Why it is separable |
|---|---|---|
| `layout_seed` | Known Search Area walls and pillars, **and the room's position** | The room is a 2 m walled box in plain sight; the team walks around it at setup. Where it sits is public geometry that shapes the known area's corridor ring. Only its interior is unknown. |
| `unknown_seed` | The maze inside the room, and the pillars in it | Everything §3.2 calls "intentionally NOT shown" |
| `mission_seed` | Victim, bonus-victim and fire placement | Undisclosed independently of either layout (3.3.3 r.1, 3.3.5 r.1) |

Each defaults to a child of `seed` via `SeedSequence.spawn`, so `generate_arena(k)` still
reproduces a whole arena on its own. Moving the room translates the maze rather than resampling
it: the same `unknown_seed` is the same maze wherever the room lands.

> **A fixed map set is a development set.** Policies overfit to it — precisely the failure the
> rulebook's withholding is designed to punish. Use the library for development, debugging and
> regression. Quote final numbers from seeds you have never looked at, and treat the gap between
> the two as a measurement of how much you overfit.

## 2. Survey, then place

A navigation aid's position generally **cannot** be fixed in `ArenaConfig`. The room's position is
sampled per seed, and 33 m² at the centre of the field falls inside it for *every* seed, so a
fixed coordinate there can never be legal. Generate first, then place against the arena you got:

```python
import dataclasses
from safmc_sim.world.landmark import Landmark

arena = load_arena("maps/known_03.json")
aids = tuple(Landmark(f"aid_{i}", "nav_tag", x, y)
             for i, (x, y) in enumerate(spots) if arena.in_known_area(x, y))
setup = dataclasses.replace(arena, landmarks=aids)
```

`in_known_area` is the predicate that says where a team may legally put something. The three zones
differ in *permission*, not only geometry:

| Zone | Team may enter | Aids |
|---|---|---|
| Start Area | yes | unlimited |
| Known Search Area | at setup only | ≤10 (3.3.1 r.15) |
| Unknown Search Area | **never** (r.17) | **zero** |

`validate_arena` enforces the zone rule: a landmark inside the room is refused, whether it arrived
through `ArenaConfig` or through `dataclasses.replace`. Generated mission markers are exempt —
3.3.9 r.2 *requires* bonus victims and fires in there. The ≤10 count is **not** enforced, because
a `Landmark` may equally be scenery or a venue feature and the primitive carries no field saying
which; assert it yourself against `NAV_AID_MAX_KNOWN_AREA`.

A point landmark that no configured sensor detects is a `ConfigError`, not a silent no-op — give
the camera the kind:

```python
MarkerCamConfig(kinds=TARGET_KINDS + ("nav_tag",))
```

## 3. Start

```python
run(RunConfig(seed=99, n_drones=10, sensors=sensors), arena=setup)
```

The log header records the full geometry with `arena_source: "supplied"`, so the run stays
reproducible from its log even though it is not reproducible from `RunConfig.seed` alone.
`header.seed` still seeds the policies, the sensors and the take-off grid; `header.arena.seed` is
the arena's own.

Two runs on one field — the rulebook's two attempts — is the same map with two different
`RunConfig.seed` values.

---

## What the map file holds

`save_arena` writes JSON carrying the geometry, the per-stream seeds and the **full
`ArenaConfig`**. The config matters: a map generated with a 1.4 m `min_gap_wall_m` would otherwise
be revalidated against the 2.0 m default and rejected for a violation it never committed. Config
fields are serialised from `dataclasses.fields`, so a knob added later cannot silently vanish.

`load_arena` validates by default. A map on disk gets hand-edited far more often than one in
memory, and a map that quietly breaks a published constraint would invalidate every run it ever
appears in. Pass `validate=False` to opt out deliberately.

A file whose schema differs, or whose config carries a field this build does not know, is refused
rather than silently downgraded — an unknown knob means the map was written by a build that knew
something this one does not.

## Why not `arena_from_log`

`recorder.arena_from_log` also rebuilds an arena, but from a run log, and it substitutes a default
`ArenaConfig`. That is sound in its own context — the log header records the config that was
flown, and re-scoring needs only the shapes. A standalone map has no surrounding header, which is
why `store.py` exists rather than reusing that path.
