# Checkpoints

Each checkpoint is a commit. For each: what was built, what was verified, and what was still open
at that point. Written so an auditor can start at any checkpoint and check the claims against the
tree at that commit.

Verification vocabulary:
- **TESTED** — an automated test exists that fails if the claim is false.
- **MEASURED** — a number produced by running something, quoted with its conditions.
- **ASSERTED** — believed correct, no test yet. Every ASSERTED item is a debt.
