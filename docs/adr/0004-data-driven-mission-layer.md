# ADR-0004: The arena and mission layers are data, not code

**Status:** Accepted · **Date:** 2026-08-21

## Context

Two independent pressures point the same way.

**The rules force randomisation.** The 2026 booklet states inner-wall positions "will NOT be
given", the Unknown Search Area layout is "intentionally NOT shown", the diagram is "not drawn to
scale", and target placement shown is "for illustration purposes only". A policy tuned against one
hand-drawn map measures overfitting.

**The scoring layer is the volatile part.** Comparing 2025 Cat E to 2026 Swarm: Danger Zones were
removed entirely, fires were added with a coupling rule that zeroes nearby victims, and the relay
with its 2x multiplier was added. Meanwhile the *stable* core across both editions is the 20x20 m
field, the 1.4 m ceiling, 10-25 drones, two take-off waves, 10-minute runs, land-within-1 m-with-LOS
scoring and the nav-aid zoning.

The 2027 rulebook is not public. The mechanic layer is the part most likely to move again.

## Decision

- **Arena** is produced by a seeded *generator* from a scenario descriptor, not loaded from a fixed
  map file. It validates its own output (gaps, connectivity, targets not inside obstacles) and
  raises on violation.
- **Mission and scoring** are a declarative rule set: target kinds with point values, a radius and
  LOS predicate, a coupling rule, and a multiplier rule. Changing next year's mechanics should be a
  data edit plus a test, not a refactor.

## Consequences

- Every reported result must be a distribution over seeds. A single-arena number is meaningless and
  the metrics layer should make it awkward to produce one.
- Arena validation can fail at generation time, which is correct — a silently disconnected arena
  would invalidate every run on it.
- Some genuinely structural rules stay in code: the two-wave take-off constraint and the
  landed-drone-is-consumed rule are lifecycle facts, not scoring parameters.
- The relay rule is geometric and combinatorial rather than a simple predicate, so it gets its own
  evaluator. It is worth the attention: at 2x total score it is the highest-leverage thing in the
  competition.
