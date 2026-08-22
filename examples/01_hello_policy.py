"""The smallest useful policy, written entirely in primitives.

Run:  .venv/bin/python examples/01_hello_policy.py
"""

import numpy as np

from safmc_sim.api import Command, Land, Observation, Policy, Velocity, register_policy
from safmc_sim.recorder import Recorder
from safmc_sim.runner import RunConfig, run
from safmc_sim.toolbox import body_to_world

CRUISE_M = 0.5


@register_policy("hello")
class Hello(Policy):
    """Climb, fly forward, turn away from what is ahead, land on the first marker it reaches."""

    def reset(self) -> None:
        self.turn_direction = 1.0 if self.rng.random() < 0.5 else -1.0

    def step(self, obs: Observation) -> Command:
        # Nothing holds altitude for you -- climbing is a velocity, and deciding when to stop
        # is your call. `toolbox.climb` does exactly this if you would rather not write it.
        if obs.pose.z < CRUISE_M - 0.02:
            return Velocity(vz=0.4)

        # Landing is how you score -- and it spends the drone permanently.
        for marker in obs.markers:
            if marker.range_m < 0.6:
                return Land()
            forward, lateral = 0.3, 0.0
            return Velocity(
                *body_to_world(forward, lateral, obs.pose.theta),
                yaw_rate=float(np.clip(2.0 * marker.bearing_rad, -1.5, 1.5)),
            )

        # React only to what is AHEAD. Using obs.tof.min_range_m would react to the whole
        # 360-degree ring -- including the wall behind you and the drone beside you -- which
        # makes a fleet turn on the spot forever without leaving the Start Area.
        ahead = float(np.min(obs.tof.ranges_m[0]))
        if ahead < 0.8:
            return Velocity(yaw_rate=self.turn_direction * 1.2)

        self.publish("position", [round(obs.pose.x, 1), round(obs.pose.y, 1)])
        return Velocity(*body_to_world(0.45, 0.0, obs.pose.theta))


if __name__ == "__main__":
    result = run(
        RunConfig(seed=0, n_drones=10, policy="hello", duration_s=120.0),
        recorder=Recorder("runs/hello"),
    )
    print(f"score      {result.score.explain()}")
    print(f"landed     {len(result.landed)}   crashed {len(result.crashed)}")
    print(f"sim time   {result.sim_time_s:.0f}s in {result.wall_time_s:.1f}s wall clock")
    print(f"log        {result.log_path}")
    print("\nreplay:    .venv/bin/safmc-run replay runs/hello")
