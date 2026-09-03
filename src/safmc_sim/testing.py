"""Helpers for unit-testing a policy without building a simulator.

A full run is 12 000 ticks. Most policy bugs are a single decision made from a single
observation, and finding those by running a whole mission is slow and imprecise.

    from safmc_sim.testing import make_observation

    obs = make_observation(front_range_m=0.3)      # a wall 30 cm ahead, everything else clear
    assert my_policy.step(obs).vx < 0.1
"""

from __future__ import annotations

from types import MappingProxyType
from typing import Any, Mapping

import numpy as np

from .api import ArenaInfo, Observation, Pose
from .constants import (
    FIELD_DEPTH_M,
    FIELD_WIDTH_M,
    CEILING_M,
    RUN_DURATION_S,
    START_AREA_DEPTH_M,
)
from .sensors.tof_ring import ToFConfig, ToFScan, _ranger_bearings, _zone_offsets

__all__ = ["make_observation", "make_scan"]


def make_scan(config: ToFConfig | None = None, **ranger_ranges: float) -> ToFScan:
    """A scan with everything out of range, except the rangers you name.

    Rangers are named by direction: ``front``, ``left``, ``back``, ``right`` — or by index,
    ``r0`` .. ``r7``. Anything unset reads ``inf``, meaning nothing in range.

        make_scan(front=0.4, left=1.2)
    """
    cfg = config or ToFConfig()
    ranges = np.full((cfg.n_rangers, cfg.zones_per_ranger), np.inf)
    quarter = cfg.n_rangers // 4
    named = {"front": 0, "left": quarter, "back": 2 * quarter, "right": 3 * quarter}

    for key, value in ranger_ranges.items():
        if key in named:
            index = named[key]
        elif key.startswith("r") and key[1:].isdigit():
            index = int(key[1:])
        else:
            raise ValueError(
                f"unknown ranger {key!r}; use front/left/back/right or r0..r{cfg.n_rangers - 1}"
            )
        ranges[index, :] = value

    bearings = _ranger_bearings(cfg)
    return ToFScan(
        ranges_m=ranges,
        zone_bearings_rad=bearings[:, None] + _zone_offsets(cfg)[None, :],
        ranger_bearings_rad=bearings,
    )


def make_observation(
    x: float = 10.0,
    y: float = 10.0,
    z: float = 0.5,
    theta: float = 0.0,
    lifecycle: str = "ACTIVE",
    tick: int = 0,
    markers: tuple = (),
    peers: dict | None = None,
    front_range_m: float | None = None,
    sensors: Mapping[str, Any] | None = None,
    stale_ticks: Mapping[str, int] | None = None,
    **ranger_ranges: float,
) -> Observation:
    """An ``Observation`` with sensible defaults: mid-arena, at cruise, nothing in range.

    Name any ranger to put something in front of it -- ``front_range_m=0.3`` is the common
    case and has its own argument. ``sensors`` adds or overrides readings by sensor name, for
    a policy that reads a sensor of your own:

        make_observation(sensors={"beacons": BeaconReading(...)})
    """
    if front_range_m is not None:
        ranger_ranges.setdefault("front", front_range_m)
    readings: dict[str, Any] = {"tof": make_scan(**ranger_ranges), "markers": tuple(markers)}
    if sensors:
        readings.update(sensors)
    ages = {name: 0 for name in readings}
    if stale_ticks:
        ages.update(stale_ticks)
    return Observation(
        agent_id="drone_00",
        tick=tick,
        sim_time_s=tick * 0.05,
        pose=Pose(x=x, y=y, z=z, theta=theta),
        velocity_xy=(0.0, 0.0),
        lifecycle=lifecycle,
        sensors=MappingProxyType(readings),
        stale_ticks=MappingProxyType(ages),
        peers=peers or {},
        arena=ArenaInfo(
            width_m=FIELD_WIDTH_M,
            depth_m=FIELD_DEPTH_M,
            ceiling_m=CEILING_M,
            start_area_depth_m=START_AREA_DEPTH_M,
            run_duration_s=RUN_DURATION_S,
        ),
    )
