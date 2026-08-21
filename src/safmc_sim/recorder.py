"""Structured run recording.

ir-sim offers no state history and no replay format -- ``EnvLogger`` is a 92-line loguru text
wrapper, and the only durable artefact is a matplotlib GIF. So this is greenfield, and it is
worth doing properly: a run that cannot be replayed cannot be debugged, and a comparison that
cannot be re-scored offline cannot be trusted.

Layout, per run directory:

``run.jsonl``
    Line 1 is the header: schema version, the full resolved config, the complete arena
    (every wall, pillar and target), the seed, and package versions. Then one line per event.
    The last line is the footer: final score with its full arithmetic, per-agent lifecycles,
    and the mission summary.
``states.npz``
    Dense per-tick arrays: time, pose, velocity, lifecycle, and the command each agent was
    issued, with its arguments.
``tof.npz``
    The 64-bin collapsed scans. Separate because it dominates size and is only needed when
    debugging sensing.

Two properties the format is built for:

**Offline re-scoring (R-MISS-8).** The header carries the arena and the states carry final
landed positions, so ``score_from_log`` recomputes the score without re-simulating, and it must
agree exactly with the online result. If they ever disagree, one of them has a bug -- which is
the point of keeping both.

**Byte-identical logs for identical runs (R-DET-1).** Every wall-clock value is confined to
the header's ``meta`` block, so two runs of the same ``(scenario, seed, policy, config)`` differ
only there.
"""

from __future__ import annotations

import json
import math
import platform
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from .api import Lifecycle
from .errors import LogFormatError

__all__ = ["SCHEMA_VERSION", "Recorder", "load_run", "score_from_log", "LIFECYCLE_CODES",
           "COMMAND_CODES"]

SCHEMA_VERSION = "safmc-sim/run/1"

LIFECYCLE_CODES = {name: i for i, name in enumerate(Lifecycle.ALL)}
LIFECYCLE_NAMES = {i: name for name, i in LIFECYCLE_CODES.items()}

COMMAND_CODES = {
    "Takeoff": 0, "VelocityBody": 1, "VelocityWorld": 2,
    "PositionWorld": 3, "Hold": 4, "Land": 5,
}
COMMAND_NAMES = {i: name for name, i in COMMAND_CODES.items()}


def _jsonable(value: Any) -> Any:
    """Make a value JSON-safe without losing information silently."""
    if is_dataclass(value) and not isinstance(value, type):
        return {k: _jsonable(v) for k, v in asdict(value).items()}
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, float):
        # json.dumps writes bare Infinity/NaN, which is NOT valid JSON: every strict parser
        # rejects it, including JSON.parse in the replay page, which then renders blank. The
        # log carries inf legitimately (a ToF no-return), so it has to be encoded, not banned.
        return value if math.isfinite(value) else None
    if value is None or isinstance(value, (str, int, bool)):
        return value
    return repr(value)


def _command_row(command) -> tuple[int, list[float]]:
    """Encode a command as ``(kind_code, four_args)``. Unused slots are NaN, not zero."""
    name = type(command).__name__
    code = COMMAND_CODES.get(name)
    if code is None:
        raise LogFormatError(f"cannot record unknown command type {name}")
    nan = float("nan")
    if name == "Takeoff":
        return code, [command.altitude_m, nan, nan, nan]
    if name == "VelocityBody":
        return code, [command.vx, command.vy, command.vz, command.yaw_rate]
    if name == "VelocityWorld":
        yaw = nan if command.yaw is None else command.yaw
        return code, [command.vx, command.vy, command.z, yaw]
    if name == "PositionWorld":
        yaw = nan if command.yaw is None else command.yaw
        return code, [command.x, command.y, command.z, yaw]
    return code, [nan, nan, nan, nan]


class Recorder:
    """Writes one run to a directory."""

    def __init__(self, directory: str | Path, record_tof: bool = True, tof_every: int = 1) -> None:
        self.directory = Path(directory)
        self.record_tof = record_tof
        self.tof_every = max(int(tof_every), 1)
        self._agents: list[str] = []
        self._times: list[float] = []
        self._pose: list[np.ndarray] = []
        self._velocity: list[np.ndarray] = []
        self._lifecycle: list[np.ndarray] = []
        self._command_kind: list[np.ndarray] = []
        self._command_args: list[np.ndarray] = []
        self._tof_ticks: list[int] = []
        self._tof: list[np.ndarray] = []
        self._header: dict[str, Any] | None = None

    # -- lifecycle ---------------------------------------------------------------------------

    def begin(self, config, arena, agent_ids: Sequence[str]) -> None:
        self._agents = list(agent_ids)
        self._header = {
            "schema": SCHEMA_VERSION,
            "record": "header",
            # Every wall-clock value lives here and nowhere else, so two identical runs
            # produce logs that differ only in this block (R-DET-1).
            "meta": {
                "written_at": datetime.now(timezone.utc).isoformat(),
                "python": platform.python_version(),
                "platform": platform.platform(),
                "versions": _package_versions(),
            },
            "seed": config.seed,
            "config": _jsonable(config),
            "agents": self._agents,
            # The codebook travels with the log. Without it, states.npz is a wall of integers
            # whose meaning lives in whatever version of recorder.py happened to write it.
            "codebook": {
                "lifecycle": {str(code): name for code, name in LIFECYCLE_NAMES.items()},
                "command": {str(code): name for code, name in COMMAND_NAMES.items()},
                "pose_columns": ["x", "y", "z", "theta"],
                "velocity_columns": ["vx", "vy"],
                "tof_no_return": "inf",
            },
            "arena": _jsonable(
                {
                    "seed": arena.seed,
                    "width_m": arena.width_m,
                    "depth_m": arena.depth_m,
                    "ceiling_m": arena.ceiling_m,
                    "start_area_depth_m": arena.start_area_depth_m,
                    "unknown_area": arena.unknown_area,
                    "walls": [asdict(w) for w in arena.walls],
                    "pillars": [asdict(p) for p in arena.pillars],
                    "targets": [asdict(t) for t in arena.targets],
                }
            ),
        }

    def tick(self, tick: int, sim_time_s: float, agents, commands) -> None:
        self._times.append(sim_time_s)
        # float64, not float32. Offline re-scoring compares positions against a 1.0 m radius,
        # and float32 quantises a 20 m coordinate at ~1e-6 m -- enough for a drone parked
        # exactly on the boundary to be scored one way online and the other way offline
        # (R-MISS-8). The extra bytes compress away.
        self._pose.append(
            np.array(
                [
                    [
                        float(a.state[0, 0]), float(a.state[1, 0]),
                        float(a.state[3, 0]), float(a.state[2, 0]),
                    ]
                    for a in agents
                ],
                dtype=np.float64,
            )
        )
        self._velocity.append(
            np.array(
                [[float(a.state[4, 0]), float(a.state[5, 0])] for a in agents],
                dtype=np.float32,
            )
        )
        self._lifecycle.append(
            np.array([LIFECYCLE_CODES[a.lifecycle] for a in agents], dtype=np.int8)
        )
        rows = [_command_row(c) for c in commands]
        self._command_kind.append(np.array([r[0] for r in rows], dtype=np.int8))
        self._command_args.append(np.array([r[1] for r in rows], dtype=np.float32))

        if self.record_tof and tick % self.tof_every == 0:
            self._tof_ticks.append(tick)
            self._tof.append(
                np.array(
                    [
                        a.last_scan.collapsed_m if a.last_scan is not None
                        else np.full(64, np.inf)
                        for a in agents
                    ],
                    dtype=np.float32,
                )
            )

    def finish(self, result) -> str:
        if self._header is None:
            raise LogFormatError("finish() called before begin()")
        self.directory.mkdir(parents=True, exist_ok=True)

        with (self.directory / "run.jsonl").open("w") as handle:
            handle.write(json.dumps(self._header, sort_keys=True) + "\n")
            for event in result.events:
                handle.write(
                    json.dumps(
                        {"record": "event", **_jsonable(event)}, sort_keys=True
                    )
                    + "\n"
                )
            handle.write(
                json.dumps(
                    {
                        "record": "footer",
                        "ticks": result.ticks,
                        "sim_time_s": result.sim_time_s,
                        "mission_started_tick": result.mission_started_tick,
                        "lifecycles": _jsonable(result.lifecycles),
                        "score": _jsonable(result.score),
                        "mission_summary": _jsonable(result.mission_summary),
                    },
                    sort_keys=True,
                )
                + "\n"
            )

        np.savez_compressed(
            self.directory / "states.npz",
            time_s=np.array(self._times, dtype=np.float64),
            pose=np.stack(self._pose) if self._pose else np.zeros((0, 0, 4), np.float64),
            velocity=np.stack(self._velocity) if self._velocity else np.zeros((0, 0, 2), np.float32),
            lifecycle=np.stack(self._lifecycle) if self._lifecycle else np.zeros((0, 0), np.int8),
            command_kind=np.stack(self._command_kind) if self._command_kind else np.zeros((0, 0), np.int8),
            command_args=np.stack(self._command_args) if self._command_args else np.zeros((0, 0, 4), np.float32),
        )
        if self.record_tof and self._tof:
            np.savez_compressed(
                self.directory / "tof.npz",
                ticks=np.array(self._tof_ticks, dtype=np.int32),
                collapsed_m=np.stack(self._tof),
            )
        return str(self.directory)


def _package_versions() -> dict[str, str]:
    import importlib.metadata as metadata

    out = {}
    for name in ("ir-sim", "numpy", "shapely", "matplotlib", "pyyaml"):
        try:
            out[name] = metadata.version(name)
        except Exception:  # noqa: BLE001 -- a missing optional package is not a log failure
            out[name] = "unknown"
    return out


# ------------------------------------------------------------------------------------------
# Reading
# ------------------------------------------------------------------------------------------


def load_run(directory: str | Path) -> dict[str, Any]:
    """Load a recorded run. The visualiser reads only this -- never the simulator (R-OBS-3)."""
    directory = Path(directory)
    jsonl = directory / "run.jsonl"
    if not jsonl.exists():
        raise LogFormatError(f"no run.jsonl in {directory}")

    header: dict[str, Any] | None = None
    footer: dict[str, Any] | None = None
    events: list[dict[str, Any]] = []
    with jsonl.open() as handle:
        for line_number, line in enumerate(handle, start=1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise LogFormatError(f"{jsonl}:{line_number} is not valid JSON") from exc
            kind = record.get("record")
            if kind == "header":
                header = record
            elif kind == "event":
                events.append(record)
            elif kind == "footer":
                footer = record
            else:
                raise LogFormatError(f"{jsonl}:{line_number} has unknown record type {kind!r}")

    if header is None or footer is None:
        raise LogFormatError(f"{jsonl} is missing its header or footer")
    if header.get("schema") != SCHEMA_VERSION:
        raise LogFormatError(
            f"{jsonl} has schema {header.get('schema')!r}, this build reads {SCHEMA_VERSION!r}"
        )

    out: dict[str, Any] = {"header": header, "events": events, "footer": footer}
    states = directory / "states.npz"
    if states.exists():
        with np.load(states) as data:
            out["states"] = {k: data[k] for k in data.files}
    tof = directory / "tof.npz"
    if tof.exists():
        with np.load(tof) as data:
            out["tof"] = {k: data[k] for k in data.files}
    return out


def score_from_log(directory: str | Path):
    """Recompute the score from the log alone. Must equal the online result exactly (R-MISS-8).

    Rebuilds the arena from the recorded geometry rather than regenerating it from the seed --
    otherwise this would test the generator's determinism rather than the scoring, and would
    silently pass if the recorded arena and the simulated one had diverged.
    """
    from .mission import Mission
    from .world.arena import ArenaConfig, ArenaSpec, Pillar, Target, Wall

    run = load_run(directory)
    spec = run["header"]["arena"]
    arena = ArenaSpec(
        seed=spec["seed"],
        width_m=spec["width_m"],
        depth_m=spec["depth_m"],
        ceiling_m=spec["ceiling_m"],
        start_area_depth_m=spec["start_area_depth_m"],
        unknown_area=tuple(spec["unknown_area"]),
        walls=tuple(Wall(**w) for w in spec["walls"]),
        pillars=tuple(Pillar(**p) for p in spec["pillars"]),
        targets=tuple(Target(**t) for t in spec["targets"]),
        config=ArenaConfig(),
    )
    mission = Mission(arena)

    states = run.get("states")
    if states is None or not len(states["pose"]):
        raise LogFormatError("log has no recorded states to score")

    agents = run["header"]["agents"]
    final_pose = states["pose"][-1]
    final_lifecycle = states["lifecycle"][-1]
    landed = {
        agents[i]: np.array([final_pose[i, 0], final_pose[i, 1]], dtype=float)
        for i in range(len(agents))
        if LIFECYCLE_NAMES[int(final_lifecycle[i])] == Lifecycle.LANDED
    }
    mission.update(int(states["pose"].shape[0]) - 1, float(states["time_s"][-1]), landed)
    return mission.score(landed)
