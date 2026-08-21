# ADR-0003: Ground-truth pose and a perfect blackboard in v0.1 — behind seams

**Status:** Accepted · **Date:** 2026-08-21 · **Decided by:** team lead

## Context

Two of the four pillars this project could address are sensor fusion and communications. Both were
explicitly deferred:

- **Pose:** ground truth from ir-sim, no external estimation stack. Noise and drift are future work.
- **Comms:** perfect shared blackboard. No DDS, no radio model. Assume comms are solved.

## Decision

Ship `GroundTruthPose` and `PerfectBlackboard` in v0.1, but make both **the only** path by which a
policy obtains pose and peer data, so a later implementation is a substitution rather than a
rewrite.

## Consequences — stated plainly

**What v0.1 can honestly claim:** "Search policy A beats policy B, *given perfect state and free
communication*."

**What it cannot claim:** anything about robustness to drift or link loss. This matters more than
usual here, because the competition's highest-value targets sit in the Unknown Search Area, where
the rules forbid placing any navigation aid and forbid teams from ever entering. Localisation there
is dead reckoning plus onboard sensing. A policy validated on perfect pose has not been tested in
the regime that decides the score.

That is an acceptable v0.1 scope. It is only acceptable **if the seams are real**, which is why
R-SEAM-1 and R-SEAM-2 are auditable requirements rather than intentions.

The target paper has exactly this limitation and states it as future work (§IV-C: "simulation
assumes perfect noiseless sensing"). Producing the noise/drift ablation it lacks is a genuine,
publishable delta available to this team — and it is unlocked by one implementation behind one
seam.
