# Writing a policy

**Start here.** This is the only file you need to read to write a search strategy.

## The shape

One class, one method. You get an `Observation` each tick, you return a `Command`.

```python
from safmc_sim.api import Observation, Command, Policy, Velocity, Land, register_policy

@register_policy("my_strategy")
class MyStrategy(Policy):
    def reset(self) -> None:
        self.turning = False                    # per-episode state lives on self

    def step(self, obs: Observation) -> Command:
        if obs.pose.z < 0.5:                    # you decide when and how to climb
            return Velocity(vz=0.4)
        if obs.tof.min_range_m < 0.5:
            return Velocity(yaw_rate=0.8)
        return Velocity(vx=0.45)                # ARENA frame: +x is East
```

Run it. **`--import` is not optional** — your policy registers itself when its file is
imported, and nothing imports your file unless you say so:

```bash
safmc-run run --import my_strategy.py --policy my_strategy --drones 12 --duration 600
```

Without it you get `unknown policy 'my_strategy'. Registered: ['sdlw']`.

## Two commands. That is the whole action space.

| Command | Meaning |
|---|---|
| `Velocity(vx, vy, vz, yaw_rate)` | ARENA-frame linear velocity and yaw rate. Clipped by the drone's limits and tracked through a first-order lag. Nothing else happens to it |
| `Land()` | Commit to the ground here, **permanently**. The drone is spent for the rest of the run |

That is deliberately austere. There is no `PositionWorld`, no `Hold`, no `Takeoff`.

**Why:** an earlier version had six commands mirroring the firmware's MAVLink helpers. But
those helpers are *guidance*, and copying them meant the simulator contained a path follower,
a yaw controller and an altitude hold. Every policy inherited them without asking, and two
policies being "compared" were mostly comparing the same borrowed controller. The firmware is
the reference for what the drone **is** — sensors, geometry, limits, frames — not for how it
flies.

Practical consequences you will notice immediately:

- **Nothing holds your altitude.** Stop commanding `vz` and you stop climbing; you do not
  hover. If you want altitude hold, write it — it is three lines and now it is yours.
- **There is no take-off.** A drone starts on the ground and flies when you command it up.
- **Yaw is a rate, not a setpoint.** Turning to a heading is a control problem you own.

## What you get: `Observation`

| Field | What it is |
|---|---|
| `pose` | `.x .y .z .theta` in ARENA metres/radians. **Ground truth in v0.1** |
| `velocity_xy` | `(vx, vy)` in the ARENA frame |
| `lifecycle` | `ACTIVE`, `LANDED` or `CRASHED`. Both terminal states are permanent |
| `tof` | The ring. `.ranges_m` is `(8, 8)` metres — `inf` where there was no return. `.zone_bearings_rad` and `.ranger_bearings_rad` give the matching body-frame directions. `.min_range_m` is the nearest return anywhere |
| `markers` | Tuple of `(marker_id, kind, range_m, bearing_rad)` currently visible |
| `peers` | The blackboard, as of the **start** of this tick, keyed by agent id |
| `arena` | Published field dimensions only — width, depth, ceiling, start-area depth, run duration |
| `tick`, `sim_time_s` | Where you are |

Convenience: `obs.in_start_area`, `obs.time_remaining_s`, `obs.pose.xy`.

Facts you will want and would otherwise have to dig for:

- **The tick rate is 20 Hz**, so `dt` is 0.05 s. `yaw_rate=1.2` turns 0.06 rad per tick.
- **Marker kinds** are exactly `"victim"`, `"bonus_victim"` and `"fire"`.
- **Rangers are numbered anticlockwise from the nose**: `ranges_m[0]` is forward, `[2]` is
  your left, `[4]` is behind, `[6]` is your right.
- **Your drone index** is not part of the contract. If you need to differentiate drones, use
  `self.rng` — every policy instance gets its own seeded generator.

**You cannot see the map, the obstacles, or where the targets are.** That is enforced
structurally: `Observation` is frozen and holds only plain data, with no reference to the
environment, the arena, the mission, or another agent. There is a test that walks everything
reachable from an `Observation` and asserts none of those appear.

## What actually kills you

Crashing will dominate your score before anything clever does, so know the rules:

- **Collision is 2D.** Two drones at *any* altitudes collide if their footprints overlap.
  Altitude is **not** a deconfliction axis — flying higher does not let you pass over a
  teammate.
- **What can kill you is what you can see.** Other drones occlude your ring at every altitude,
  matching the collision model. If your ring is clear, you are clear.
- **Walls and pillars are taller than the ceiling**, so they always block and always hurt.
- **`collision_behaviour="stop"`** (the default) means one touch ends that drone's run — there
  is no repair. `--collision unobstructed` turns collisions off entirely, and is the right
  control when you want to compare *search strategy* without crash rate confounding it.

## Three rules that will bite you if you ignore them

**Landing is irreversible and spends the drone.** A `LANDED` drone never moves again. With up
to 12 targets, a relay chain worth a 2× multiplier, and 10–25 drones, deciding *how many*
drones to spend and *when* is the actual strategic problem.

**Randomness comes from `self.rng`.** Every policy gets its own `numpy.random.Generator`.
Calling `numpy.random.uniform(...)` directly breaks seeded replay.

**Peer data is one tick old.** `self.publish(key, value)` becomes visible — to everyone,
including you — on the *next* tick. That is not a latency model; it is what makes the result
independent of the order agents happen to be indexed.

## Coordinating a fleet

`self.publish(key, value)` and `obs.peers` are the whole communication API.

```python
def step(self, obs):
    mine = self._pick_goal(obs)
    taken = {v.get("goal") for a, v in obs.peers.items() if a != self.agent_id}
    ...
    self.publish("goal", mine)
```

In v0.1 the blackboard is **perfect**: lossless, rangeless, instantaneous. Design as if
bandwidth is scarce anyway — the day someone implements a lossy `Blackboard`, a policy that
broadcasts its entire occupancy grid every tick will stop working and yours should not.

## The toolbox — optional, and not part of the framework

`safmc_sim.toolbox` holds building blocks. **Nothing in the framework imports it.** They are
examples to copy, adapt, or ignore.

| Helper | What it does |
|---|---|
| `body_to_world(vx, vy, theta)` | Rotate a body-frame velocity into the ARENA frame. Four lines, in the open |
| `ring_quadrants(obs)` | Reduce the 8-ranger ring to front/left/right/back minima. Lossless in those directions |
| `climb(obs, target_m)` | A `Velocity` that climbs, or `None` once there. A sign test, not a controller |
| `descend(obs)` | A `Velocity` that descends. Pair with `Land()` at the bottom |
| `OccupancyMap` | A textbook log-odds grid with frontier extraction |

They live outside the platform on purpose: each encodes a choice the simulator exists to let
you *compare*. If `OccupancyMap` were the mapper, nobody would ever measure a better one.

## The pose caveat, stated plainly

`obs.pose` is **exact**. No drift, no noise, no estimator.

That is a v0.1 scope decision, and it means an honest claim from a result here is *"strategy A
beats strategy B, given perfect state"*. It matters most in the Unknown Search Area, where the
rules forbid any navigation aid and forbid teams from ever entering — localisation there is
dead reckoning plus onboard sensing.

`PoseSource` is the seam. `NoisyPose` already exists as a worked example. Swapping it in
requires **no change to your policy**.

## Testing your policy

```python
import numpy as np
from safmc_sim.testing import make_observation

def test_backs_off_from_a_wall():
    policy = MyStrategy("drone_00", {}, np.random.default_rng(0), make_observation().arena)
    policy.reset()
    blocked = make_observation(front_range_m=0.3)
    assert policy.step(blocked).vx < 0.1
```

`safmc_sim.testing.make_observation()` builds a plausible `Observation` with everything clear,
and lets you set individual ranges. Policies are plain objects with injected dependencies, so
they unit-test without a simulator. Use that — a full run is 12 000 ticks.

## The one reference policy

`src/safmc_sim/policies/sdlw.py` is a port of [arXiv:2607.25195](https://arxiv.org/abs/2607.25195) — an NUS
paper accepted to IROS 2026. It is the only policy that ships, deliberately: a strategy written
by whoever wrote the simulator is not a baseline, it is the simulator's own assumptions wearing
a policy's clothes. SDLW is externally authored, externally published, and citable.

Note it never lands — the paper's task is pure coverage — so it scores **zero** on the
competition mission by design. It is a *search* baseline and a regression test that our sensor
and kinematics semantics reproduce a published result.
