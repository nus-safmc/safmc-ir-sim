"""Range to surveyed UWB anchors, placed where the rules allow, and grade the sensor from the log.

Three things this shows, in order:

1. **Placing anchors legally.** The rulebook allows any number of navigation aids in the
   Start Area, at most ten in the Known Search Area and none in the Unknown Search Area
   (booklet 3.3.1 r.14-17). The layout below is six anchors across both rows of the Start
   Area -- collinear anchors leave a mirror ambiguity, so use both rows -- and four on
   tripods in the Known Search Area, two in its far corners and two half-way up its sides,
   a rectangle rather than a line, checked with ``validate_nav_aids``.
   The runner would not have refused an illegal layout; the check is yours to call.
2. **Reading the tag.** ``obs.sensors["uwb"]`` is ``UWBRanges``: anchor ids, their surveyed
   positions, and one range each, ``inf`` where nothing was heard. No bearing, no flag that
   says which ranges are biased -- that is the problem a localiser has to solve, and it is
   yours to write. This policy only wanders with the ring and publishes the nearest anchor.
3. **Grading the sensor offline.** ``uwb.npz`` plus the header is enough: true ranges come
   from ``states.npz`` and the anchor positions stored beside the readings, and the arena in
   the header says which paths crossed a wall. Nothing here re-runs the simulator.

Every number in the model is an assumption with an ID (A-9..A-13, docs/FIDELITY.md), and
the airframe does not carry a UWB module. See sensors/uwb.py and ADR-0006.

Run:  python examples/04_uwb_ranging.py
"""

from __future__ import annotations

import numpy as np

from safmc_sim.api import Command, Observation, Policy, Velocity, register_policy
from safmc_sim.recorder import Recorder, arena_from_log, load_run
from safmc_sim.runner import RunConfig, flown_sensors, run
from safmc_sim.sensors.raycast import segment_clear
from safmc_sim.sensors.uwb import UWBConfig, UWBRanges
from safmc_sim.toolbox import body_to_world
from safmc_sim.world.arena import ArenaConfig, generate_arena, validate_nav_aids
from safmc_sim.world.landmark import Landmark

# ------------------------------------------------------------------------------------------
# 1. The anchors -- points in the Start Area, tripods with a base in the Known Search Area
# ------------------------------------------------------------------------------------------

# Nothing is ever generated in the Start Area, so a point is safe there. A point at fixed
# coordinates in the Known Search Area could end up inside a generated wall; a 0.25 m base
# makes it a flat mark the generator draws around, still invisible to the ring and to
# collision (a mark is not solid).
ANCHORS = (
    Landmark("start_w0", "uwb_anchor", 0.5, 0.5),
    Landmark("start_m0", "uwb_anchor", 10.0, 0.5),
    Landmark("start_e0", "uwb_anchor", 19.5, 0.5),
    Landmark("start_w1", "uwb_anchor", 0.5, 5.5),
    Landmark("start_m1", "uwb_anchor", 10.0, 5.5),
    Landmark("start_e1", "uwb_anchor", 19.5, 5.5),
    Landmark("known_sw", "uwb_anchor", 1.0, 12.0, radius_m=0.25),
    Landmark("known_se", "uwb_anchor", 19.0, 12.0, radius_m=0.25),
    Landmark("known_nw", "uwb_anchor", 1.0, 19.0, radius_m=0.25),
    Landmark("known_ne", "uwb_anchor", 19.0, 19.0, radius_m=0.25),
)


# ------------------------------------------------------------------------------------------
# 2. A policy that reads the tag -- by name, like any other sensor
# ------------------------------------------------------------------------------------------


@register_policy("uwb_walk")
class UWBWalk(Policy):
    """Climb, wander with the ring, and publish the nearest anchor by radio.

    Deliberately not a localiser. Least-squares trilateration from ten biased ranges, with
    the anchor geometry in ``anchor_xyz_m`` and your own altitude from ``obs.pose.z``, is the
    afternoon's work this sensor exists to make possible -- and a ``PoseSource`` that fuses
    it is the next piece of platform work (ADR-0003).
    """

    def reset(self) -> None:
        self.bias = self.rng.uniform(-np.pi, np.pi)

    def step(self, obs: Observation) -> Command:
        if obs.pose.z < 0.48:
            return Velocity(vz=0.4)

        uwb: UWBRanges = obs.sensors["uwb"]
        if uwb.heard.any():
            nearest = uwb.anchor_ids[int(np.argmin(uwb.ranges_m))]
            self.publish("nearest_anchor", nearest)
            self.publish("anchors_heard", int(uwb.heard.sum()))

        per_ranger = np.min(obs.tof.ranges_m, axis=1)
        clear = np.where(np.isfinite(per_ranger), per_ranger, 99.0)
        want = obs.tof.ranger_bearings_rad[int(np.argmax(clear))]
        if np.min(clear) > 2.0:
            want = float(np.arctan2(np.sin(self.bias - obs.pose.theta), np.cos(self.bias - obs.pose.theta)))
        speed = 0.45 if float(np.min(per_ranger[[0, 1, -1]])) > 1.0 else 0.1
        return Velocity(*body_to_world(speed, 0.0, obs.pose.theta),
                        yaw_rate=float(np.clip(1.5 * want, -1.5, 1.5)))


# ------------------------------------------------------------------------------------------
# 3. The run: the flown suite plus the tag, in an arena with the anchors in it
# ------------------------------------------------------------------------------------------


def make_config(seed: int = 0, duration_s: float = 60.0) -> RunConfig:
    return RunConfig(
        seed=seed,
        n_drones=10,
        policy="uwb_walk",
        duration_s=duration_s,
        arena_config=ArenaConfig(landmarks=ANCHORS),
        sensors=flown_sensors() + (UWBConfig(),),
    )


# ------------------------------------------------------------------------------------------
# 4. Grading the sensor from the log alone
# ------------------------------------------------------------------------------------------


def grade(log_dir: str) -> dict[str, float]:
    """Range error statistics, split by whether the path crossed a wall. Reads only the log."""
    log = load_run(log_dir)
    uwb = log["sensors"]["uwb"]
    pose = log["states"]["pose"]                                   # (T, N, 4): x, y, z, theta
    anchor_xyz = uwb["anchor_xyz_m"]                               # (A, 3)
    n_anchors = len(anchor_xyz)

    fresh = uwb["sample_tick"] == uwb["ticks"][:, None]            # held rows would count twice
    t_idx, n_idx = np.nonzero(fresh)
    tag_xyz = pose[t_idx, n_idx, :3]                               # (M, 3)
    true = np.linalg.norm(tag_xyz[:, None, :] - anchor_xyz[None, :, :], axis=-1)
    reported = uwb["ranges_m"][t_idx, n_idx]                       # (M, A)
    heard = np.isfinite(reported)

    # Which paths crossed a wall: the arena recorded in the header, tested at cruise altitude.
    # Every wall and pillar is 2.0 m and the ceiling 1.4 m, so any altitude a drone can fly
    # gives the same answer. Walls and pillars only -- what obstructs radio.
    walls = arena_from_log(log["header"]).structural_scene()
    clear = np.empty_like(heard)
    for j in range(n_anchors):
        clear[:, j] = segment_clear(walls, tag_xyz[:, :2], np.tile(anchor_xyz[j, :2], (len(t_idx), 1)), 0.5)

    # An anchor beyond the tag's reach is silent whether or not a wall is in the way; the
    # tag's config is in the header, so the grade can tell the two apart.
    tag_cfg = next(s for s in log["header"]["config"]["sensors"] if s["name"] == "uwb")
    in_reach = true <= tag_cfg["max_range_m"]

    error = reported - true
    los = clear & heard
    nlos = ~clear & heard
    return {
        "sweeps": int(len(t_idx)),
        "paths_in_line_of_sight": float(clear.mean()),
        "paths_in_reach": float(in_reach.mean()),
        "heard_in_line_of_sight_and_reach": float(heard[clear & in_reach].mean()),
        "heard_behind_a_wall_in_reach": float(heard[~clear & in_reach].mean()),
        "los_error_mean_m": float(error[los].mean()) if los.any() else float("nan"),
        "los_error_std_m": float(error[los].std()) if los.any() else float("nan"),
        "nlos_error_mean_m": float(error[nlos].mean()) if nlos.any() else float("nan"),
        "nlos_error_std_m": float(error[nlos].std()) if nlos.any() else float("nan"),
    }


if __name__ == "__main__":
    config = make_config()
    # The runner does not referee anchor placement (R-WORLD-9); this does.
    validate_nav_aids(generate_arena(config.seed, config.arena_config), ("uwb_anchor",))

    result = run(config, recorder=Recorder("runs/uwb", overwrite=True))
    log = load_run(result.log_path)
    ids = [lm["id"] for lm in log["header"]["arena"]["landmarks"] if lm["kind"] == "uwb_anchor"]
    print(f"anchors (uwb.npz column order): {ids}")
    print(f"uwb.npz ranges_m shape (ticks, drones, anchors): {log['sensors']['uwb']['ranges_m'].shape}")
    for key, value in grade(result.log_path).items():
        print(f"{key:>26}: {value:.3f}" if isinstance(value, float) else f"{key:>26}: {value}")
    print(f"landed {len(result.landed)}   crashed {len(result.crashed)}   log {result.log_path}")
