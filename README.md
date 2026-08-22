# safmc-sim

A lightweight 2.5D multi-drone simulator for developing **search policies** for the
[SAFMC](https://www.safmc.com.sg) Category Swarm challenge. Built on
[ir-sim](https://github.com/hanruihua/ir-sim).

It deliberately does not simulate physics. It simulates geometry, sparse time-of-flight
ranging, marker detection, multi-agent coordination, and the competition's actual scoring
rules — the things that decide whether a strategy works.

## Why this exists

The team previously ran Gazebo + PX4 SITL. That stack simulates rigid-body dynamics, motor
mixing, EKF2 and a full autopilot, none of which is in question. What *is* in question is
whether a swarm carrying eight cheap ToF rangers can explore an unknown room fast enough to
win.

The existence proof is not hypothetical. **arXiv:2607.25195** — Leong and Teo, **NUS**,
accepted to **IROS 2026** — produced a publishable nano-UAV exploration result using
**ir-sim v2.9.0**, four single-ray sensors, a first-order integrator and a 10 Hz loop. Its
policy ships here as a reference implementation and a regression test.

## Quick start

```bash
python3 -m venv .venv && .venv/bin/pip install -e ".[dev]"
.venv/bin/python examples/01_hello_policy.py
.venv/bin/safmc-run replay runs/hello            # -> runs/hello/replay.html, open it
```

Then run **your own** policy. Your file registers itself when imported, and nothing imports it
unless you say so — that is what `--import` is for:

```bash
.venv/bin/safmc-run run --import my_search.py --policy my_search --drones 12 --duration 600
.venv/bin/safmc-run sweep --import my_search.py --policy my_search sdlw --seeds 0-9
```

> The one shipped policy, `sdlw`, **scores zero on purpose** — it is a pure *coverage* baseline
> from a published paper and never lands, and landing is how you score. Compare it on
> `sensed_coverage`, not on score. If your quick start prints `score 0`, nothing is broken.

**New to the codebase?** [ARCHITECTURE.md](ARCHITECTURE.md) is the map — what the pieces are,
which ones you touch, and what happens when you press run. Four diagrams, no jargon.

## Writing a policy

One class, one method, **two commands**. Full guide: [docs/05-policy-api.md](docs/05-policy-api.md).

```python
from safmc_sim.api import Observation, Command, Policy, Velocity, register_policy

@register_policy("my_strategy")
class MyStrategy(Policy):
    def step(self, obs: Observation) -> Command:
        if obs.pose.z < 0.5:                 # you decide when and how to climb
            return Velocity(vz=0.4)
        if obs.tof.min_range_m < 0.5:
            return Velocity(yaw_rate=0.8)
        return Velocity(vx=0.45)
```

`Velocity` and `Land` are the entire action space. There is no path following, no altitude
hold, no take-off sequence and no obstacle avoidance anywhere in the simulator — those are
strategy, and shipping them would mean every policy silently inherited the same ones.

A policy sees only what a real drone could: its own pose and velocity, the ToF ring, marker
detections, and whatever peers broadcast. It cannot reach the map, the obstacles or the target
positions — that is enforced structurally, and there is a test that walks everything reachable
from an `Observation` to prove it.

## What is modelled

| | |
|---|---|
| **Arena** | Seeded generation from the 2026 Category Swarm rulebook: 20 x 20 m, Start Area, a 10 x 10 m walled Unknown Search Area with doorways, pillars, randomised layout, self-validating |
| **Drone** | 2.5D: `[x, y, theta, z, vx, vy]`, first-order velocity lag, 1.4 m ceiling, `ACTIVE` / `LANDED` / `CRASHED` |
| **Sensing** | 8 x VL53L5CX ring, 8 zones each, reproducing the flown geometry and gating, plus the firmware's 64-bin collapsed scan. Height-gated occlusion |
| **Toolbox** | Opt-in building blocks (frame rotation, sensor reduction, a log-odds grid) that the framework never imports |
| **Mission** | Victims +5, bonus +15, fires +10, the 2.5 m fire-suppression coupling, and the relay's 2x multiplier evaluated as a real graph search |
| **Rules** | Two take-off waves, 10-25 drones, 600 s runs, landing spends the drone permanently |

## What is not modelled, on purpose

Physics, attitude, motors, battery — the whole premise.

And two deliberate v0.1 deferrals, each behind a seam so adding them later changes no policy:

- **Pose is ground truth.** No drift, no estimator.
- **Communication is a perfect shared blackboard.** No range, loss or latency.

So the honest claim from a v0.1 result is *"strategy A beats strategy B, given perfect state
and free communication"*. Everything known to diverge from reality, and every value chosen
without published data, is listed in [docs/FIDELITY.md](docs/FIDELITY.md). Read it before
quoting a number.

## Documentation

| | |
|---|---|
| [**ARCHITECTURE.md**](ARCHITECTURE.md) | **The map of the codebase — start here to understand it** |
| [docs/](docs/README.md) | Index and reading order |
| [**docs/05-policy-api.md**](docs/05-policy-api.md) | **Start here to write a policy** |
| [docs/SPEC.md](docs/SPEC.md) | The numbered, auditable contract |
| [docs/FIDELITY.md](docs/FIDELITY.md) | Every divergence and every assumption |
| [docs/CHECKPOINTS.md](docs/CHECKPOINTS.md) | What was built and verified at each commit |
| [docs/adr/](docs/adr/) | Why ir-sim, why one ToF sensor, why these seams |

## Tests

```bash
.venv/bin/python -m pytest tests -q
```

The raycaster is verified against two independently derived analytic references to **1e-9 m**.
Frames round-trip exactly across the wrap discontinuity. Two identical runs produce logs that
differ only in their `meta` block. Offline re-scoring from the log equals the online score
exactly.

## Status

v0.1. Runs end to end, 595 tests, audited adversarially against `SPEC.md`
([report](docs/AUDIT-v0.1.md)).

**One reference policy ships**: a port of arXiv:2607.25195, because a strategy written by
whoever wrote the simulator is not a baseline. Everything else — guidance, avoidance, mapping,
search, coordination — is yours to write, which is the point.

Open questions are listed at the end of [docs/CHECKPOINTS.md](docs/CHECKPOINTS.md). The one
worth doing first is measuring assumption **A-4**, marker detection range: it is a guess at
3.0 m, nothing measures it, and it dominates every search comparison this simulator will
produce.
