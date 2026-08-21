# Writing a policy

**Start here.** This is the only file you need to read to write a search strategy.

## The shape

One class, one method. You get an `Observation` each tick, you return a `Command`.

```python
from safmc_sim.api import Observation, Command, Policy, VelocityBody, Land, register_policy

@register_policy("my_strategy")
class MyStrategy(Policy):
    def reset(self) -> None:
        self.turning = False                    # per-episode state lives on self

    def step(self, obs: Observation) -> Command:
        if obs.lifecycle == "IDLE":
            return Takeoff()
        if obs.tof.min_range_m < 0.5:
            return VelocityBody(vx=0.0, yaw_rate=0.8)
        return VelocityBody(vx=0.45)
```

Run it:

```bash
safmc-run run --policy my_strategy --drones 12 --seed 0 --duration 600
```

Better: subclass `SearchPolicy`, which already handles take-off, spotting a marker, closing on
it and landing, so you only write the search:

```python
from safmc_sim.policies.base import SearchPolicy

@register_policy("my_strategy")
class MyStrategy(SearchPolicy):
    def choose_motion(self, obs: Observation) -> Command:
        return VelocityBody(vx=self.cruise_speed_ms)
```

## What you get: `Observation`

| Field | What it is |
|---|---|
| `pose` | `.x .y .z .theta` in ARENA metres/radians. **Ground truth in v0.1** — see the caveat below |
| `velocity_xy` | `(vx, vy)` in the ARENA frame |
| `lifecycle` | `IDLE`, `TAKEOFF`, `FLYING`, `LANDING`, `LANDED`, `CRASHED` |
| `tof` | The ring. `.ranges_m` is `(8, 8)`, `.collapsed_m` is the 64-bin scan, `.min_range_m` is the nearest return anywhere |
| `markers` | Tuple of `(marker_id, kind, range_m, bearing_rad)` for what is visible right now |
| `peers` | The blackboard, as of the **start** of this tick, keyed by agent id |
| `arena` | Published field dimensions only — width, depth, ceiling, start-area depth, run duration |
| `tick`, `sim_time_s` | Where you are |
| `tof_stale_ticks`, `marker_stale_ticks` | Non-zero only if you decimated that sensor |

Convenience: `obs.in_start_area`, `obs.time_remaining_s`, `obs.pose.xy`.

**You cannot see the map, the obstacles, or where the targets are.** That is enforced
structurally, not by convention: `Observation` is frozen and holds only plain data, with no
reference to the environment, the arena, the mission, or another agent. If you find a way to
reach ground truth from an `Observation`, that is a bug — please report it.

## What you return: `Command`

Exactly six, mirroring the real firmware's MAVLink API and nothing more.

| Command | Meaning | Real equivalent |
|---|---|---|
| `Takeoff(altitude_m=0.5)` | Arm and climb. Subject to the two-wave rule | arm + offboard + climb |
| `VelocityBody(vx, vy, vz, yaw_rate)` | Body-frame velocity | `mavlink_set_velocity_ned` |
| `VelocityWorld(vx, vy, z, yaw)` | ARENA-frame velocity, held altitude, absolute yaw | `mavlink_set_velocity_xy_position_z` |
| `PositionWorld(x, y, z, yaw, speed_ms)` | Fly to a point | `mavlink_set_position_ned` |
| `Hold()` | Stop and hold | `mavlink_set_hold` |
| `Land()` | Descend and land, **permanently** | `MAV_CMD_NAV_LAND` |

`VelocityWorld.yaw=None` and `PositionWorld.yaw=None` hold the current heading. That is
deliberate: the real firmware warns that commanding yaw `0.0` means "face North", which is
almost never what a caller intended.

## Three rules that will bite you if you ignore them

**Landing is irreversible and spends the drone.** The rules require a rescuing drone to stay
"until the end of the mission". A `LANDED` drone never moves again. With up to 12 targets, a
relay chain worth a 2x multiplier, and 10-25 drones, deciding *how many* drones to spend and
*when* is the actual strategic problem. This is not a coverage benchmark.

**Randomness comes from `self.rng`.** Every policy gets its own `numpy.random.Generator`.
Calling `numpy.random.uniform(...)` directly breaks seeded replay, and a run you cannot replay
is a run you cannot debug.

**Peer data is one tick old.** `self.publish(key, value)` becomes visible — to everyone,
including you — on the *next* tick. That is not a latency model; it is what makes the result
independent of the order agents happen to be indexed. Without it, agent 0's publication would
reach agent 1 but not the reverse.

## Coordinating a fleet

`self.publish(key, value)` and `obs.peers` are the whole communication API.

```python
def choose_motion(self, obs):
    mine = self._pick_frontier(obs)
    taken = {v.get("goal") for a, v in obs.peers.items() if a != self.agent_id}
    ...
    self.publish("goal", mine.tolist())
```

`SearchPolicy` already publishes `claimed_target` so two drones do not both land on the same
victim — which would permanently waste one of them, since each target scores once.

In v0.1 the blackboard is **perfect**: lossless, rangeless, instantaneous. That is a
simplification, recorded in [ADR-0003](adr/0003-ground-truth-pose-and-perfect-comms.md).
Design as if bandwidth is scarce anyway; the day someone implements a lossy `Blackboard`, a
policy that broadcasts its entire occupancy grid every tick will stop working and yours should
not.

## The pose caveat, stated plainly

`obs.pose` is **exact**. There is no drift, no noise, no estimator.

That is a deliberate v0.1 scope decision, and it means an honest claim from a result here is
*"strategy A beats strategy B, given perfect state"*. It is not a claim about the real flight.
It matters most in the Unknown Search Area, where the rules forbid placing any navigation aid
and forbid teams from ever entering — localisation there is dead reckoning plus onboard
sensing, and the real firmware has no filter at all.

`PoseSource` is the seam. `NoisyPose` already exists as a worked example. When you want to
know whether your strategy survives drift, swapping it in requires **no change to your policy**.

## Testing your policy

```python
def test_backs_off_from_a_wall():
    policy = MyStrategy("drone_00", {}, np.random.default_rng(0), arena_info)
    policy.reset()
    obs = make_observation(front_range=0.3)          # see tests/test_tof_ring.py for helpers
    assert isinstance(policy.step(obs), VelocityBody)
```

Policies are plain objects with injected dependencies, so they unit-test without a simulator.
Use that: a full run is 12 000 ticks, and a bug in a corner case is far cheaper to find here.

## Reading the reference policies

In increasing order of complexity:

1. `policies/simple.py` — `hold`, `random_walk`, `wall_follow`. Twenty lines each.
2. `policies/sdlw.py` — a faithful port of arXiv:2607.25195. Mapless, stochastic, and the
   published baseline to beat.
3. `policies/frontier.py` — log-odds occupancy mapping, frontier selection, VFH avoidance.
   The map-based counterpart.

`policies/base.py` has two helpers worth knowing: `ring_quadrants(obs)` reduces the 8-ranger
ring to the front/left/right/back minima that Crazyflie-style controllers expect, and
`vfh_steer(obs, desired_bearing, clearance)` returns the nearest free bearing to a goal,
shaped like the histogram the real firmware flies.
