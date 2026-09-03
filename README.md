# safmc-sim

A small simulator for developing **drone search strategies** for the
[SAFMC](https://www.safmc.com.sg) Category Swarm challenge. It wraps
[ir-sim](https://github.com/hanruihua/ir-sim).

It does not simulate physics. It simulates geometry, sparse time-of-flight ranging, marker
detection and the competition's scoring — the things that decide whether a strategy works.

```
                 you write this
                       │
        ┌──────────────▼──────────────┐
        │  step(observation) -> command │
        └──────────────┬──────────────┘
                       │
   arena · ToF ring · collisions · scoring · logs
```

## Quick start

```bash
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -e ".[dev]"

python examples/01_hello_policy.py
safmc-run replay runs/hello      # writes runs/hello/replay.html — open it
```

Every command below assumes that activated environment. If you would rather not activate, prefix
with `.venv/bin/` on macOS and Linux, or `.venv\Scripts\` on Windows — the layout differs by
platform, which is why these examples activate instead.

Now run your own. Your file registers itself when imported, and nothing imports it unless you
say so — that is what `--import` is for:

```bash
safmc-run run   --import my_search.py --policy my_search --drones 12 --duration 600
safmc-run sweep --import my_search.py --policy my_search sdlw --seeds 0-9
```

## Writing a strategy

One class, one method, **two commands**.

```python
from safmc_sim.api import Observation, Command, Policy, Velocity, Land, register_policy

@register_policy("my_search")
class MySearch(Policy):
    def step(self, obs: Observation) -> Command:
        if obs.pose.z < 0.5:                      # you decide when and how to climb
            return Velocity(vz=0.4)
        if obs.markers and obs.markers[0].range_m < 0.6:
            return Land()                         # landing scores, and spends the drone
        if obs.tof.min_range_m < 0.8:
            return Velocity(yaw_rate=0.8)
        return Velocity(vx=0.45)                  # ARENA frame: +x is East
```

`Velocity` and `Land` are the entire action space. There is **no** path following, altitude
hold, take-off sequence or obstacle avoidance anywhere in the simulator — those are strategy,
and shipping them would mean every policy silently inherited the same ones.

Full guide: **[docs/05-policy-api.md](docs/05-policy-api.md)**.

## What is modelled

| | |
|---|---|
| **Arena** | Seeded generation from the 2026 rulebook — 20×20 m, a walled 10×10 m unknown room, pillars, randomised per seed, self-validating |
| **Drone** | 2.5D `[x, y, θ, z, vx, vy]`, first-order velocity lag, 1.4 m ceiling, `ACTIVE` / `LANDED` / `CRASHED` |
| **Sensing** | 8 × VL53L5CX ring, 8 zones each, reproducing the flown geometry and gating; a geometric AprilTag camera. Height-gated occlusion. Both on one sensor contract, so a sensor of your own is one file |
| **Landmarks** | Things placed for sensors to find — mission markers, and any nav tag, start mark or anchor you add. Solid ones occlude and can be hit; points do neither |
| **Mission** | Victims, bonus victims, fires, the fire-suppression coupling, and the relay's 2× multiplier |
| **Observability** | Structured log, offline re-scoring, metrics, a self-contained HTML replay |
| **Toolbox** | Opt-in helpers — frame rotation, sensor reduction, a log-odds grid. The framework never imports them |

## What is not modelled

Physics, attitude, motors, battery. That is the premise, not an omission.

And two deliberate deferrals, each behind a seam so adding them changes no policy:

- **Pose is ground truth.** No drift, no estimator.
- **Communication is a perfect shared blackboard.** No range, loss or latency.

So an honest claim from a result here is *"strategy A beats strategy B, given perfect state and
free communication"*. Everything known to diverge from reality, and every number chosen without
published data, is in **[docs/FIDELITY.md](docs/FIDELITY.md)**. Read it before quoting a figure.

## Documentation

| | |
|---|---|
| [**ARCHITECTURE.md**](ARCHITECTURE.md) | **The map — what the pieces are and what happens when you press run** |
| [**REVIEW.md**](REVIEW.md) | How to review this work, and what to be skeptical of |
| [docs/05-policy-api.md](docs/05-policy-api.md) | Writing a strategy |
| [docs/06-sensors.md](docs/06-sensors.md) | Exactly what the sensors report |
| [docs/01-competition.md](docs/01-competition.md) | The rules this is built against |
| [docs/FIDELITY.md](docs/FIDELITY.md) | Every divergence and assumption |
| [docs/07-logging-and-viz.md](docs/07-logging-and-viz.md) | Logs, metrics, replay |
| [docs/08-porting-to-ros.md](docs/08-porting-to-ros.md) | Taking a policy to real drones |
| [docs/adr/](docs/adr/) | Why ir-sim, why one ToF sensor, why these seams, why sensors and landmarks are primitives |
| [docs/SPEC.md](docs/SPEC.md) | The numbered contract the tests check against |

## Tests

```bash
python -m pytest tests -q      # 274 tests
```

Runtime is very platform-sensitive: ~37 s on the Linux machine it was developed on, but 2-6
minutes on Windows and variable between runs. Two integration tests dominate either way.

The raycaster is checked against two independently derived analytic references to 1e-9 m. Two
identical runs produce logs that differ only in their timestamp block. The score recomputed
offline from the log must equal the online score exactly.

## Status

v0.1. One reference policy ships — a port of [arXiv:2607.25195](https://arxiv.org/abs/2607.25195)
(NUS, IROS 2026). It is a pure *coverage* baseline and **never lands, so it scores zero by
design**; compare it on `sensed_coverage`, not score. Everything else is yours to write.

Open questions and decisions awaiting the team are in [REVIEW.md](REVIEW.md).
