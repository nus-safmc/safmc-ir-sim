"""Structured run recording.

ir-sim offers no state history and no replay format -- ``EnvLogger`` is a 92-line loguru text
wrapper, and the only durable artefact is a matplotlib GIF. So this is greenfield, and it is
worth doing properly: a run that cannot be replayed cannot be debugged, and a comparison that
cannot be re-scored offline cannot be trusted.

Layout, per run directory:

``run.jsonl``
    Line 1 is the header: schema version, the full resolved config, the complete arena
    (every wall, pillar, target and landmark), whether that arena was generated from the seed
    or supplied, the sensor suite, the seed, and package versions. Then one line per event.
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
        # sensor name -> record key -> row shape, fixed at begin() and enforced every tick
        self._schema: dict[str, dict[str, tuple[int, ...]]] = {}
        self._header: dict[str, Any] | None = None

    # -- lifecycle ---------------------------------------------------------------------------

    def begin(self, config, arena, agent_ids: Sequence[str], sensors: Sequence = (),
              sample_readings: Mapping[str, Any] | None = None,
              arena_source: str = "generated") -> None:
        """Start a run.

        ``sensors`` is one drone's sensor list, for names and static arrays;
        ``sample_readings`` that drone's first readings, so that every sensor's row schema --
        which keys, what shapes -- is fixed and checked here, before a tick is recorded,
        rather than discovered as a misaligned file at the end of the run.
        """
        self._agents = list(agent_ids)
        self._sensor_names = [s.name for s in sensors]
        for sensor in sensors:
            static = {}
            for key, value in sensor.record_static().items():
                if key in _RESERVED_SENSOR_KEYS:
                    raise LogFormatError(
                        f"sensor {sensor.name!r} record_static() uses reserved key {key!r}"
                    )
                static[key] = _numeric_array(value, f"sensor {sensor.name!r} record_static()[{key!r}]")
            self._sensor_static[sensor.name] = static
            if sample_readings is not None and sensor.name in sample_readings:
                self._learn_schema(sensor, sensor.record(sample_readings[sensor.name]))

        config_json = _jsonable(config)
        # asdict() drops the class, and the class is what makes a sensor config meaningful.
        for entry, sensor in zip(config_json.get("sensors", []), sensors):
            entry["type"] = _type_name(sensor.config)
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
            "config": config_json,
            "agents": self._agents,
            "arena_source": arena_source,
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
                    "type": _type_name(sensor.config),
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
                    # Base fields only, so a Landmark subclass with fields of its own still
                    # reads back through Landmark(**row) when the log is re-scored.
                    "targets": [_landmark_row(t) for t in arena.targets],
                    "landmarks": [_landmark_row(lm) for lm in arena.landmarks],
                }
            ),
        }

    def _learn_schema(self, sensor, row) -> None:
        """Fix a sensor's row keys and shapes from its first record(), or mark it skipped."""
        name = sensor.name
        if row is None:
            self._sensors_skipped.add(name)
            return
        schema: dict[str, tuple[int, ...]] = {}
        for key, value in row.items():
            if key in _RESERVED_SENSOR_KEYS:
                raise LogFormatError(f"sensor {name!r} record() uses reserved key {key!r}")
            if key in self._sensor_static.get(name, {}):
                raise LogFormatError(
                    f"sensor {name!r} uses key {key!r} in both record() and record_static()"
                )
            schema[key] = _numeric_array(value, f"sensor {name!r} record()[{key!r}]").shape
        if not schema:
            raise LogFormatError(
                f"sensor {name!r} record() returned an empty mapping; return None to leave "
                f"the sensor out of the log"
            )
        self._schema[name] = schema

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
                try:
                    rows = [a.sensors[j].record(a.readings[name]) for a in agents]
                except Exception as exc:  # noqa: BLE001 -- re-raised with sensor and tick
                    raise LogFormatError(
                        f"sensor {name!r} record() raised at tick {tick}: {exc!r}"
                    ) from exc
                if name in self._sensors_skipped:
                    if any(r is not None for r in rows):
                        raise LogFormatError(
                            f"sensor {name!r} record() returned None at its first sample and "
                            f"rows at tick {tick}. A sensor is recorded for the whole run or "
                            f"not at all; return rows from the first sample, or None always."
                        )
                    continue
                if name not in self._schema:
                    # begin() was not given sample readings; learn from the first tick.
                    self._learn_schema(agents[0].sensors[j], rows[0])
                    if name in self._sensors_skipped:
                        continue
                schema = self._schema[name]
                store = self._sensor_rows.setdefault(name, {})
                frames: dict[str, list[np.ndarray]] = {key: [] for key in schema}
                for agent, row in zip(agents, rows):
                    # A sensor that records on some ticks and not others, or changes its
                    # keys or shapes, would produce a misaligned file that loads without
                    # complaint. Refuse it at the tick it happens, naming everything.
                    if row is None or set(row) != set(schema):
                        raise LogFormatError(
                            f"sensor {name!r} record() for {agent.agent_id} at tick {tick} "
                            f"returned keys {sorted(row) if row else None}; the schema fixed "
                            f"at the first sample was {sorted(schema)}"
                        )
                    for key, shape in schema.items():
                        value = np.asarray(row[key], dtype=np.float32)
                        if value.shape != shape:
                            raise LogFormatError(
                                f"sensor {name!r} record()[{key!r}] for {agent.agent_id} at "
                                f"tick {tick} has shape {value.shape}; the schema fixed at "
                                f"the first sample was {shape}"
                            )
                        frames[key].append(value)
                for key, values in frames.items():
                    store.setdefault(key, []).append(np.stack(values))
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
        for entry in self._header["sensors"]:
            entry["recorded"] = entry["name"] in self._sensor_rows

        # Assemble every array before writing anything, so a stacking failure cannot leave a
        # directory holding run.jsonl and states.npz but not the sensor file the header
        # promises. Either the whole run lands on disk or none of it does.
        states = dict(
            time_s=np.array(self._times, dtype=np.float64),
            pose=np.stack(self._pose) if self._pose else np.zeros((0, 0, 4), np.float64),
            lifecycle=np.stack(self._lifecycle) if self._lifecycle else np.zeros((0, 0), np.int8),
            command_kind=np.stack(self._command_kind) if self._command_kind else np.zeros((0, 0), np.int8),
            command_args=np.stack(self._command_args) if self._command_args else np.zeros((0, 0, 4), np.float32),
        )
        sensor_files: dict[str, dict[str, np.ndarray]] = {}
        for name, store in self._sensor_rows.items():
            sensor_files[name] = dict(
                ticks=np.array(self._sensor_ticks, dtype=np.int32),
                sample_tick=np.stack(self._sensor_sample_tick[name]),
                **{key: np.stack(frames) for key, frames in store.items()},
                **self._sensor_static.get(name, {}),
            )

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

        np.savez_compressed(self.directory / "states.npz", **states)
        for name, arrays in sensor_files.items():
            np.savez_compressed(self.directory / f"{name}.npz", **arrays)
        return str(self.directory)


_RESERVED_SENSOR_KEYS = frozenset({"ticks", "sample_tick"})

_LANDMARK_FIELDS = ("id", "kind", "x", "y", "radius_m", "height_m")


def _numeric_array(value, what: str) -> np.ndarray:
    """A numeric ndarray, or a LogFormatError naming what was wrong.

    An object array saves fine and then fails to load without ``allow_pickle`` -- the kind
    of failure that surfaces a run later, on someone else's machine.
    """
    try:
        array = np.asarray(value)
    except Exception as exc:  # noqa: BLE001
        raise LogFormatError(f"{what} is not array-like: {exc}") from exc
    if array.dtype.kind not in "biuf":
        raise LogFormatError(
            f"{what} has dtype {array.dtype}; sensor arrays must be numeric or boolean"
        )
    return array


def _landmark_row(landmark) -> dict[str, Any]:
    return {name: getattr(landmark, name) for name in _LANDMARK_FIELDS}


def _type_name(obj) -> str:
    return f"{type(obj).__module__}.{type(obj).__qualname__}"


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
    if "sensors" in header:
        for entry in header["sensors"]:
            path = directory / f"{entry['name']}.npz"
            if entry.get("recorded") and path.exists():
                with np.load(path) as data:
                    out["sensors"][entry["name"]] = {k: data[k] for k in data.files}
    elif (directory / "tof.npz").exists():
        # A log written before the sensor suite was recorded in the header (C8) carried the
        # ring alone, always as tof.npz. Still readable; it has no sample_tick column.
        with np.load(directory / "tof.npz") as data:
            out["sensors"]["tof"] = {k: data[k] for k in data.files}
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
