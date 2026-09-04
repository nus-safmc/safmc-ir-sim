"""Add a sensor and something for it to sense. The whole pattern in one file.

This is a TEMPLATE, not a model of any hardware. The sensor below reports the range to every
radio anchor in the arena, which is roughly what a UWB tag does -- but nothing here is
calibrated, the noise is a guess, and the airframe does not carry one. Copy the *shape*:
a reading, a config, a sensor, a landmark kind for it to perceive, and a policy that reads it
by name. Replace the physics with yours. (The real model of a UWB tag, with the physics cited
and every number an assumption with an ID, is ``sensors/uwb.py`` and
``examples/04_uwb_ranging.py``. This file stays the template.)

Run:  python examples/03_custom_sensor.py
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from safmc_sim.api import Command, Observation, Policy, Velocity, register_policy
from safmc_sim.errors import ConfigError
from safmc_sim.recorder import Recorder, load_run
from safmc_sim.runner import RunConfig, flown_sensors, run
from safmc_sim.sensors.base import Sensor, SensorConfig, TrueState, read_only
from safmc_sim.sensors.scene import WorldScene
from safmc_sim.toolbox import body_to_world
from safmc_sim.world.arena import ArenaConfig
from safmc_sim.world.landmark import Landmark

# ------------------------------------------------------------------------------------------
# 1. The reading -- the only thing a policy will ever see of this sensor
# ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class BeaconRanges:
    """Range to each anchor, in the order the anchors were placed. ``inf`` = out of range.

    No bearing: a range-only radio gives you a distance and nothing else, which is precisely
    what makes it interesting to write a policy against.
    """

    anchor_ids: tuple[str, ...]
    ranges_m: np.ndarray


# ------------------------------------------------------------------------------------------
# 2. The config -- frozen, validated at construction, knows how to build its sensor
# ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class BeaconConfig(SensorConfig):
    name: str = "beacons"
    rate_hz: float | None = 10.0
    kind: str = "uwb_anchor"
    """The landmark kind this sensor perceives. Anything else in the arena is invisible to it."""
    max_range_m: float = 15.0
    noise_std_m: float = 0.10

    def __post_init__(self) -> None:
        super().__post_init__()          # name and rate checks live in the base class
        if self.max_range_m <= 0:
            raise ConfigError(f"max_range_m must be > 0, got {self.max_range_m}")
        if self.noise_std_m < 0:
            raise ConfigError(f"noise_std_m must be >= 0, got {self.noise_std_m}")

    @property
    def landmark_kinds(self) -> tuple[str, ...]:
        # Declaring the kind lets the runner refuse an arena whose anchors nobody can hear.
        return (self.kind,)

    def build(self, rng: np.random.Generator) -> "BeaconRanger":
        return BeaconRanger(self, rng)


# ------------------------------------------------------------------------------------------
# 3. The sensor -- a pure function of the true state and the world, plus its own noise
# ------------------------------------------------------------------------------------------


def range_to_anchors(anchors, origin_xy, max_range_m) -> np.ndarray:
    """The geometry, kept pure so it can be unit-tested without a runner."""
    if not anchors:
        return np.zeros(0)
    centres = np.array([[a.x, a.y] for a in anchors])
    ranges = np.linalg.norm(centres - np.asarray(origin_xy, dtype=float), axis=1)
    return np.where(ranges <= max_range_m, ranges, np.inf)


class BeaconRanger(Sensor):
    config: BeaconConfig

    def sample(self, truth: TrueState, world: WorldScene, tick: int) -> BeaconRanges:
        anchors = world.landmarks_of(self.config.kind)
        ranges = range_to_anchors(anchors, truth.xy, self.config.max_range_m)
        if self.config.noise_std_m > 0:
            hit = np.isfinite(ranges)
            # Noise on "nothing heard" is meaningless -- there is nothing to perturb.
            ranges = np.where(hit, ranges + self.rng.normal(0.0, self.config.noise_std_m, ranges.shape), ranges)
        return BeaconRanges(
            anchor_ids=tuple(a.id for a in anchors),
            ranges_m=read_only(ranges),
        )

    # Optional. Give the log a fixed-shape row and the reading appears in beacons.npz.
    def record(self, reading: BeaconRanges):
        return {"ranges_m": reading.ranges_m}


# ------------------------------------------------------------------------------------------
# 4. Something to sense -- four anchors, one in each corner of the field
# ------------------------------------------------------------------------------------------

ANCHORS = (
    Landmark("anchor_sw", "uwb_anchor", 0.5, 0.5),
    Landmark("anchor_se", "uwb_anchor", 19.5, 0.5),
    Landmark("anchor_nw", "uwb_anchor", 0.5, 19.5),
    Landmark("anchor_ne", "uwb_anchor", 19.5, 19.5),
)
# Points, not bodies: no footprint and no height, so the ring cannot see them and a drone
# cannot hit them. Give a landmark both to make it a post the ring ranges and drones avoid.


# ------------------------------------------------------------------------------------------
# 5. A policy that reads it -- by name, like any other sensor
# ------------------------------------------------------------------------------------------


@register_policy("beacon_walk")
class BeaconWalk(Policy):
    """Climb, wander with the ring, and publish which corner is nearest by radio.

    Not a localiser. Trilateration from four noisy ranges is a nice afternoon's work and it is
    yours to do; this only shows the reading arriving.
    """

    def reset(self) -> None:
        self.bias = self.rng.uniform(-np.pi, np.pi)

    def step(self, obs: Observation) -> Command:
        if obs.pose.z < 0.48:
            return Velocity(vz=0.4)

        beacons: BeaconRanges = obs.sensors["beacons"]
        if np.isfinite(beacons.ranges_m).any():
            nearest = beacons.anchor_ids[int(np.argmin(beacons.ranges_m))]
            self.publish("nearest_anchor", nearest)

        per_ranger = np.min(obs.tof.ranges_m, axis=1)
        clear = np.where(np.isfinite(per_ranger), per_ranger, 99.0)
        want = obs.tof.ranger_bearings_rad[int(np.argmax(clear))]
        if np.min(clear) > 2.0:
            want = float(np.arctan2(np.sin(self.bias - obs.pose.theta), np.cos(self.bias - obs.pose.theta)))
        speed = 0.45 if float(np.min(per_ranger[[0, 1, -1]])) > 1.0 else 0.1
        return Velocity(*body_to_world(speed, 0.0, obs.pose.theta),
                        yaw_rate=float(np.clip(1.5 * want, -1.5, 1.5)))


# ------------------------------------------------------------------------------------------
# 6. Wire it into a run: the flown sensors plus ours, and the arena with anchors in it
# ------------------------------------------------------------------------------------------


def make_config(seed: int = 0, duration_s: float = 60.0) -> RunConfig:
    return RunConfig(
        seed=seed,
        n_drones=10,
        policy="beacon_walk",
        duration_s=duration_s,
        arena_config=ArenaConfig(landmarks=ANCHORS),
        sensors=flown_sensors() + (BeaconConfig(),),
    )


if __name__ == "__main__":
    result = run(make_config(), recorder=Recorder("runs/beacons", overwrite=True))
    log = load_run(result.log_path)

    print(f"sensors in the log header: {[s['name'] for s in log['header']['sensors']]}")
    beacons = log["sensors"]["beacons"]
    print(f"beacons.npz ranges_m shape (ticks, drones, anchors): {beacons['ranges_m'].shape}")
    fresh = beacons["sample_tick"][:, 0]
    print(f"drone_00 fresh every {int(np.diff(fresh[fresh >= 0]).max())} ticks (10 Hz on a 20 Hz loop)")
    print(f"last ranges for drone_00: {np.round(beacons['ranges_m'][-1, 0], 2)}")
    print(f"landed {len(result.landed)}   crashed {len(result.crashed)}   log {result.log_path}")
