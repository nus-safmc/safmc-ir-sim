# safmc-sim documentation

A lightweight 2.5D multi-drone simulator for developing and comparing **search policies** for the
SAFMC Category Swarm challenge. Built on [ir-sim](https://github.com/hanruihua/ir-sim).

It deliberately does **not** simulate physics. It simulates geometry, sparse ranging, marker
detection, multi-agent coordination and the competition's scoring rules — the things that actually
decide whether a strategy works.

## Read in this order

| # | Document | What it answers |
|---|----------|-----------------|
| 0 | [Overview](00-overview.md) | Why this exists, what replaced what, the one-paragraph thesis |
| 1 | [The competition](01-competition.md) | Arena, mission, scoring, constraints — the requirements source |
| 2 | [The hardware](02-hardware.md) | The real drone we are mirroring: sensors, geometry, protocol |
| 3 | [ir-sim](03-irsim.md) | What the library gives us, what it does not, and its landmines |
| 4 | [Architecture](04-architecture.md) | How the pieces fit; which layer owns what |
| 5 | [**Writing a policy**](05-policy-api.md) | **Start here if you are a dev writing search strategy** |
| 6 | [Sensor models](06-sensors.md) | Exactly what the ToF ring, the marker detector and the opt-in UWB tag produce |
| 7 | [Logging and visualisation](07-logging-and-viz.md) | The log schema and the replay viewer |
| 8 | [Porting to ROS 2](08-porting-to-ros.md) | The seam that keeps this from being a dead end |
| 9 | [Related work](09-related-work.md) | The two papers/codebases this must be able to host |
| 10 | [**Adding a sensor or a landmark**](10-adding-sensors-and-landmarks.md) | **Start here if you are adding a sensor, or placing something for one to find** |
| 11 | [**Building a map library**](11-map-libraries.md) | **Start here if you want a fixed set of arenas to develop against**: the three generation streams, saving maps, placing your own objects |

## Normative and process documents

- [**SPEC.md**](SPEC.md) — the numbered, auditable contract. Change this before changing behaviour.
- [FIDELITY.md](FIDELITY.md) — every known divergence from reality, and every assumption.
- [CHECKPOINTS.md](CHECKPOINTS.md) — what was built at each commit, and what was verified.
- [adr/](adr/) — decision records: what we chose, what we rejected, and why.
- [AUDIT-v0.1.md](AUDIT-v0.1.md) — the adversarial spec audit, and the defects it forced.
- [AUDIT-v0.1.1-docs-code-divergence.md](AUDIT-v0.1.1-docs-code-divergence.md) — external review
  of the merged PR against the booklet and the firmware. No behavioural defects; four documents
  describing a removed API.

## Provenance

Everything factual in these documents came from a parallel recon sweep on 2026-08-21 that read:
the ir-sim source at v2.10.2, the SAFMC 2026 rulebooks, all nine `nus-safmc` repositories
including the private ones, arXiv:2607.25195 and its released code, and ETH-PBL's
Nano_Swarm_Mapping. Claims are cited to `file:line` or URL. Where recon could not verify
something, it is marked **UNVERIFIED** and stays that way until someone checks.
