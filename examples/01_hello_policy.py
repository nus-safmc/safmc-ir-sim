"""The smallest useful policy, written entirely in primitives.

Run:  .venv/bin/python examples/01_hello_policy.py
"""

import numpy as np

from safmc_sim.api import Command, Land, Observation, Policy, Velocity, register_policy
from safmc_sim.recorder import Recorder
from safmc_sim.runner import RunConfig, run
from safmc_sim.frames import wrap_pi
from safmc_sim.toolbox import body_to_world

CRUISE_M = 0.5


@register_policy("hello")
class Hello(Policy):
    """Climb, fly forward, turn away from what is ahead, land on the first marker it reaches."""

    def reset(self) -> None:
        # Fan the fleet out. Every drone starts in a row facing North, so if they all fly
        # straight they stay parallel -- and the moment one turns, it cuts across its
        # neighbours. Giving each a different bearing is the cheapest possible coordination.
        self.bias = self.rng.uniform(-np.pi, np.pi)

    def step(self, obs: Observation) -> Command:
        # Nothing holds altitude for you -- climbing is a velocity, and deciding when to stop
        # is your call. `toolbox.climb` does exactly this if you would rather not write it.
        if obs.pose.z < CRUISE_M - 0.02:
            return Velocity(vz=0.4)

        # Landing is how you score -- and it spends the drone permanently.
        for marker in obs.markers:
            if marker.range_m < 0.6:
                return Land()
            return Velocity(
                *body_to_world(0.3, 0.0, obs.pose.theta),
                yaw_rate=float(np.clip(2.0 * marker.bearing_rad, -1.5, 1.5)),
            )

        # Steer toward whichever ranger sees the most space, biased by our fan-out heading.
        # Other drones occlude the ring at every altitude, so this avoids the team as well as
        # the walls -- an earlier version watched only the forward ranger and lost 8 of 10
        # drones to side-on collisions with its own fleet.
        per_ranger = np.min(obs.tof.ranges_m, axis=1)
        clear = np.where(np.isfinite(per_ranger), per_ranger, obs.tof.ranges_m.shape[1] * 10.0)
        want = obs.tof.ranger_bearings_rad[int(np.argmax(clear))]
        if np.min(clear) > 2.0:                       # nothing near: hold the fan-out bearing
            want = wrap_pi(self.bias - obs.pose.theta)

        speed = 0.45 if float(np.min(per_ranger[[0, 1, -1]])) > 1.0 else 0.1
        self.publish("position", [round(obs.pose.x, 1), round(obs.pose.y, 1)])
        return Velocity(
            *body_to_world(speed, 0.0, obs.pose.theta),
            yaw_rate=float(np.clip(1.5 * want, -1.5, 1.5)),
        )


if __name__ == "__main__":
    result = run(
        RunConfig(seed=0, n_drones=10, policy="hello", duration_s=120.0),
        recorder=Recorder("runs/hello", overwrite=True),
    )
    print(f"score      {result.score.explain()}")
    print(f"landed     {len(result.landed)}   crashed {len(result.crashed)}")
    print(f"sim time   {result.sim_time_s:.0f}s in {result.wall_time_s:.1f}s wall clock")
    print(f"log        {result.log_path}")
    print("\nreplay:    .venv/bin/safmc-run replay runs/hello")
