"""The smallest useful policy, and how to look at what it did.

Run:  .venv/bin/python examples/01_hello_policy.py
"""

import numpy as np

from safmc_sim.api import (
    Command, Land, Observation, Policy, Takeoff, VelocityBody, register_policy,
)
from safmc_sim.recorder import Recorder
from safmc_sim.runner import RunConfig, run


@register_policy("hello")
class Hello(Policy):
    """Fly forward, turn away from obstacles, land on the first marker it gets close to."""

    def reset(self) -> None:
        self.turn_direction = 1.0 if self.rng.random() < 0.5 else -1.0

    def step(self, obs: Observation) -> Command:
        if obs.lifecycle == "IDLE":
            return Takeoff()
        if obs.lifecycle != "FLYING":
            return VelocityBody()

        # Landing is how you score -- and it spends the drone permanently.
        for marker in obs.markers:
            if marker.range_m < 0.6:
                return Land()
            # Steer at it: bearing is body-frame, CCW from the nose.
            return VelocityBody(vx=0.3, yaw_rate=float(np.clip(2.0 * marker.bearing_rad, -1.5, 1.5)))

        # React only to what is AHEAD. Using obs.tof.min_range_m here instead would react to
        # the whole 360-degree ring, including the wall behind you and the drone beside you --
        # which makes a fleet turn on the spot forever without ever leaving the Start Area.
        # ranges_m[0] is the forward ranger; inf means nothing in range.
        ahead = float(np.min(obs.tof.ranges_m[0]))
        if ahead < 0.8:
            return VelocityBody(vx=0.05, yaw_rate=self.turn_direction * 1.2)

        # Tell the others roughly where we are. Visible to them next tick.
        self.publish("position", [round(obs.pose.x, 1), round(obs.pose.y, 1)])
        return VelocityBody(vx=0.45)


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
