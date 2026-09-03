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
``<sensor>.npz``, one per recorded sensor
    Whatever that sensor's ``record()`` returns, stacked per tick: each key becomes an array
    shaped ``(ticks, agents, *row)``, beside ``ticks`` and ``sample_tick`` -- the tick each
    agent's reading was actually sampled at, so a held reading (a decimated sensor, or a drone
    that stopped sensing) is distinguishable from a fresh one. Constants from
    ``record_static()`` are stored once. The header's ``sensors`` block lists every sensor
    and whether it was recorded; a sensor whose reading has no fixed shape is not.

    ``tof.npz`` holds ``ranges_m`` shaped ``(ticks, agents, 64)`` -- the flattened
    ``ToFScan.ranges_m`` in ``(ranger, zone)`` order, anticlockwise from the nose -- and
    ``zone_bearings_rad``, the only safe way to map a column back to a direction. Not to be
    confused with the firmware's ``tof_scan_collapsed_t``, which holds the same 64 values
    indexed by *absolute clockwise bearing*; see ``docs/06-sensors.md``. Sensor files are
    separate because they dominate size and are only needed when debugging sensing.

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

__all__ = ["SCHEMA_VERSION", "Recorder", "load_run", "score_from_log", "arena_from_log",
           "LIFECYCLE_CODES", "COMMAND_CODES"]

SCHEMA_VERSION = "safmc-sim/run/1"

LIFECYCLE_CODES = {name: i for i, name in enumerate(Lifecycle.ALL)}
LIFECYCLE_NAMES = {i: name for name, i in LIFECYCLE_CODES.items()}

COMMAND_CODES = {"Velocity": 0, "Land": 1}
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
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, (float, np.floating)):
        value = float(value)
        # json.dumps writes bare Infinity/NaN, which is NOT valid JSON: every strict parser
        # rejects it, including JSON.parse in the replay page, which then renders blank. The
        # log carries inf legitimately (a ToF no-return), so it has to be encoded, not banned.
        return value if math.isfinite(value) else None
    if value is None or isinstance(value, (str, int, bool)):
        return value
    raise LogFormatError(
        f"cannot record a {type(value).__name__} in the log. Earlier this fell back to "
        f"repr(), which embeds a memory address and silently broke the guarantee that two "
        f"identical runs produce byte-identical logs."
    )


def _command_row(command) -> tuple[int, list[float]]:
    """Encode a command as ``(kind_code, four_args)``. Unused slots are NaN, not zero."""
    name = type(command).__name__
    code = COMMAND_CODES.get(name)
    if code is None:
        raise LogFormatError(f"cannot record unknown command type {name}")
    nan = float("nan")
    if name == "Velocity":
        return code, [command.vx, command.vy, command.vz, command.yaw_rate]
    return code, [nan, nan, nan, nan]


class Recorder:
    """Writes one run to a directory."""

    def __init__(self, directory: str | Path, record_sensors: bool = True,
                 sensor_every: int = 1, overwrite: bool = False) -> None:
        self.directory = Path(directory)
        self.overwrite = overwrite
        self.record_sensors = record_sensors
        self.sensor_every = max(int(sensor_every), 1)
        self._agents: list[str] = []
        self._times: list[float] = []
        self._pose: list[np.ndarray] = []
        self._lifecycle: list[np.ndarray] = []
        self._command_kind: list[np.ndarray] = []
        self._command_args: list[np.ndarray] = []
        self._sensor_ticks: list[int] = []
        # sensor name -> record key -> one (agents, *row) array per recorded tick
        self._sensor_rows: dict[str, dict[str, list[np.ndarray]]] = {}
        self._sensor_sample_tick: dict[str, list[np.ndarray]] = {}
        self._sensor_static: dict[str, dict[str, np.ndarray]] = {}
        self._sensors_skipped: set[str] = set()
        self._sensor_names: list[str] = []
        self._header: dict[str, Any] | None = None

    # -- lifecycle ---------------------------------------------------------------------------

    def begin(self, config, arena, agent_ids: Sequence[str], sensors: Sequence = ()) -> None:
        """Start a run. ``sensors`` is one drone's sensor list, for names and static arrays."""
        self._agents = list(agent_ids)
        self._sensor_names = [s.name for s in sensors]
        for sensor in sensors:
            static = {k: np.asarray(v) for k, v in sensor.record_static().items()}
            for key in static:
                if key in _RESERVED_SENSOR_KEYS:
                    raise LogFormatError(
                        f"sensor {sensor.name!r} record_static() uses reserved key {key!r}"
                    )
            self._sensor_static[sensor.name] = static
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
                "sensor_sample_tick": "the tick whose post-motion world the recorded reading "
                                      "reflects; -1 is the sample taken before the first tick",
            },
            # One entry per sensor in config order. `recorded` is filled in at finish(),
            # because whether a reading fits a table is only known once one has been seen.
            "sensors": [
                {
                    "name": sensor.name,
                    "type": f"{type(sensor.config).__module__}.{type(sensor.config).__qualname__}",
                    "rate_hz": sensor.config.rate_hz,
                    "recorded": False,
                }
                for sensor in sensors
            ],
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
                    "landmarks": [asdict(lm) for lm in arena.landmarks],
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
        self._lifecycle.append(
            np.array([LIFECYCLE_CODES[a.lifecycle] for a in agents], dtype=np.int8)
        )
        rows = [_command_row(c) for c in commands]
        self._command_kind.append(np.array([r[0] for r in rows], dtype=np.int8))
        self._command_args.append(np.array([r[1] for r in rows], dtype=np.float32))

        if self.record_sensors and tick % self.sensor_every == 0:
            self._sensor_ticks.append(tick)
            for j, name in enumerate(self._sensor_names):
                if name in self._sensors_skipped:
                    continue
                rows = [a.sensors[j].record(a.readings[name]) for a in agents]
                if any(r is None for r in rows):
                    # All or nothing per sensor: a reading with no fixed shape is left out of
                    # the table entirely rather than recorded for some ticks and not others.
                    self._sensors_skipped.add(name)
                    self._sensor_rows.pop(name, None)
                    self._sensor_sample_tick.pop(name, None)
                    continue
                store = self._sensor_rows.setdefault(name, {})
                for key in rows[0]:
                    if key in _RESERVED_SENSOR_KEYS:
                        raise LogFormatError(
                            f"sensor {name!r} record() uses reserved key {key!r}"
                        )
                    store.setdefault(key, []).append(
                        np.array([np.asarray(r[key], dtype=np.float32) for r in rows])
                    )
                self._sensor_sample_tick.setdefault(name, []).append(
                    np.array([a.sample_tick[name] for a in agents], dtype=np.int32)
                )

    def finish(self, result) -> str:
        if self._header is None:
            raise LogFormatError("finish() called before begin()")
        if not self._pose:
            raise LogFormatError(
                "no ticks were recorded, so there is nothing to write. A log of nothing still "
                "loads and still produces a complete-looking metrics row."
            )
        if (self.directory / "run.jsonl").exists() and not self.overwrite:
            raise LogFormatError(
                f"{self.directory} already holds a run. Refusing to overwrite it -- pick "
                f"another directory, or pass overwrite=True if you meant to replace it."
            )
        self.directory.mkdir(parents=True, exist_ok=True)
        for entry in self._header["sensors"]:
            entry["recorded"] = entry["name"] in self._sensor_rows

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
            lifecycle=np.stack(self._lifecycle) if self._lifecycle else np.zeros((0, 0), np.int8),
            command_kind=np.stack(self._command_kind) if self._command_kind else np.zeros((0, 0), np.int8),
            command_args=np.stack(self._command_args) if self._command_args else np.zeros((0, 0, 4), np.float32),
        )
        for name, store in self._sensor_rows.items():
            np.savez_compressed(
                self.directory / f"{name}.npz",
                ticks=np.array(self._sensor_ticks, dtype=np.int32),
                sample_tick=np.stack(self._sensor_sample_tick[name]),
                **{key: np.stack(frames) for key, frames in store.items()},
                **self._sensor_static.get(name, {}),
            )
        return str(self.directory)


_RESERVED_SENSOR_KEYS = frozenset({"ticks", "sample_tick"})


def _package_versions() -> dict[str, str]:
    import importlib.metadata as metadata

    # All five are hard dependencies. If one is missing the environment is broken and the
    # log's provenance -- the point of this block, given ir-sim==2.10.2 is a load-bearing
    # pin -- would be a lie. Let it raise.
    return {name: metadata.version(name)
            for name in ("ir-sim", "numpy", "shapely", "matplotlib", "pyyaml")}


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

    out: dict[str, Any] = {"header": header, "events": events, "footer": footer, "sensors": {}}
    states = directory / "states.npz"
    if states.exists():
        with np.load(states) as data:
            out["states"] = {k: data[k] for k in data.files}
    for entry in header.get("sensors", []):
        path = directory / f"{entry['name']}.npz"
        if entry.get("recorded") and path.exists():
            with np.load(path) as data:
                out["sensors"][entry["name"]] = {k: data[k] for k in data.files}
    return out


def arena_from_log(header: Mapping[str, Any]):
    """Rebuild the arena from the recorded geometry, not from the seed.

    Regenerating from the seed would test the generator's determinism rather than reading what
    was actually simulated, and would hide a divergence between the two.
    """
    from .world.arena import ArenaConfig, ArenaSpec, Landmark, Pillar, Target, Wall

    spec = header["arena"]
    return ArenaSpec(
        seed=spec["seed"],
        width_m=spec["width_m"],
        depth_m=spec["depth_m"],
        ceiling_m=spec["ceiling_m"],
        start_area_depth_m=spec["start_area_depth_m"],
        unknown_area=tuple(spec["unknown_area"]),
        walls=tuple(Wall(**w) for w in spec["walls"]),
        pillars=tuple(Pillar(**p) for p in spec["pillars"]),
        targets=tuple(Target(**t) for t in spec["targets"]),
        landmarks=tuple(Landmark(**lm) for lm in spec.get("landmarks", [])),
        config=ArenaConfig(),
    )


def score_from_log(directory: str | Path):
    """Recompute the score from the log alone. Must equal the online result exactly (R-MISS-8).

    Rebuilds the arena from the recorded geometry rather than regenerating it from the seed --
    otherwise this would test the generator's determinism rather than the scoring, and would
    silently pass if the recorded arena and the simulated one had diverged.
    """
    from .mission import Mission

    run = load_run(directory)
    arena = arena_from_log(run["header"])
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
