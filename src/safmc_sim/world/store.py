"""Saving and loading a single arena, so a map can outlive the process that generated it.

An :class:`~safmc_sim.world.arena.ArenaSpec` is already frozen, validated and reusable -- it is
the map object. What it lacked was a home on disk. Until now an arena round-tripped only inside
a run log (:func:`safmc_sim.recorder.arena_from_log`), which meant "keep these ten maps" had no
answer that did not involve flying ten runs first.

Why a map file carries its config and its seeds, and a run log does not
----------------------------------------------------------------------
``arena_from_log`` rebuilds geometry and substitutes a default ``ArenaConfig``, which is sound
there: the log's own header records the config that was flown, and re-scoring only needs the
shapes. A standalone map has no surrounding header to fall back on. If it dropped the config,
``validate_arena`` would re-check a map generated with, say, a 1.5 m ``min_gap_wall_m`` against
the 2.0 m default and reject a map that was never wrong. So the file carries the config, and it
carries the per-stream seeds, so a map built with ``layout_seed``/``unknown_seed`` overrides can
be *regenerated* rather than only replayed.

The config is serialised field by field from ``dataclasses.fields``, not from a hand-written
list, so a knob added to ``ArenaConfig`` tomorrow cannot silently vanish from saved maps. That
includes ``ArenaConfig.landmarks``, even though ``ArenaSpec.landmarks`` usually holds the same
tuple: ``dataclasses.replace(arena, landmarks=...)`` -- the documented way to place objects onto
a generated map -- rewrites the spec's copy and leaves the config's alone, so the two legitimately
diverge and collapsing them loses which was which.

Round-tripping is exact for geometry, not merely close: walls, pillars and targets restore to
equal dataclasses, so ``load_arena(save_arena(a)) == a`` holds for a generated arena.
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path
from typing import Any

from ..errors import LogFormatError

__all__ = ["SCHEMA_VERSION", "arena_to_dict", "arena_from_dict", "save_arena", "load_arena"]

SCHEMA_VERSION = "safmc-sim/arena/1"


def _landmark_row(lm) -> dict[str, Any]:
    """Base fields only, so a Landmark subclass still reads back through ``Landmark(**row)``."""
    return {"id": lm.id, "kind": lm.kind, "x": lm.x, "y": lm.y,
            "radius_m": lm.radius_m, "height_m": lm.height_m}


def arena_to_dict(spec) -> dict[str, Any]:
    """The full state of an arena as plain JSON-able data."""
    config = {f.name: getattr(spec.config, f.name) for f in dataclasses.fields(spec.config)}
    config["inner_wall_length_range_m"] = list(config["inner_wall_length_range_m"])
    config["landmarks"] = [_landmark_row(lm) for lm in spec.config.landmarks]
    return {
        "schema": SCHEMA_VERSION,
        "seed": spec.seed,
        "layout_seed": spec.layout_seed,
        "unknown_seed": spec.unknown_seed,
        "mission_seed": spec.mission_seed,
        "width_m": spec.width_m,
        "depth_m": spec.depth_m,
        "ceiling_m": spec.ceiling_m,
        "start_area_depth_m": spec.start_area_depth_m,
        "unknown_area": list(spec.unknown_area),
        "walls": [dataclasses.asdict(w) for w in spec.walls],
        "pillars": [dataclasses.asdict(p) for p in spec.pillars],
        "targets": [_landmark_row(t) for t in spec.targets],
        "landmarks": [_landmark_row(lm) for lm in spec.landmarks],
        "config": config,
    }


def arena_from_dict(data: dict[str, Any]):
    """Rebuild an arena from :func:`arena_to_dict`. Geometry is read, never regenerated.

    Regenerating from the seeds would test the generator's determinism rather than restore the
    map that was saved, and would hide a divergence between the two -- the same reasoning
    ``arena_from_log`` gives.
    """
    from .arena import ArenaConfig, ArenaSpec, Landmark, Pillar, Target, Wall

    schema = data.get("schema")
    if schema != SCHEMA_VERSION:
        raise LogFormatError(
            f"arena file has schema {schema!r}, this build reads {SCHEMA_VERSION!r}"
        )

    landmarks = tuple(Landmark(**lm) for lm in data.get("landmarks", []))
    raw = dict(data.get("config", {}))
    if "inner_wall_length_range_m" in raw:
        raw["inner_wall_length_range_m"] = tuple(raw["inner_wall_length_range_m"])
    raw["landmarks"] = tuple(Landmark(**lm) for lm in raw.get("landmarks", []))
    known = {f.name for f in dataclasses.fields(ArenaConfig)}
    unknown_keys = set(raw) - known
    if unknown_keys:
        raise LogFormatError(
            f"arena file sets ArenaConfig fields this build does not have: "
            f"{sorted(unknown_keys)}. It was written by a newer build."
        )
    config = ArenaConfig(**raw)

    return ArenaSpec(
        seed=data["seed"],
        width_m=data["width_m"],
        depth_m=data["depth_m"],
        ceiling_m=data["ceiling_m"],
        start_area_depth_m=data["start_area_depth_m"],
        unknown_area=tuple(data["unknown_area"]),
        walls=tuple(Wall(**w) for w in data["walls"]),
        pillars=tuple(Pillar(**p) for p in data["pillars"]),
        targets=tuple(Target(**t) for t in data["targets"]),
        landmarks=landmarks,
        config=config,
        layout_seed=data.get("layout_seed"),
        unknown_seed=data.get("unknown_seed"),
        mission_seed=data.get("mission_seed"),
    )


def save_arena(spec, path: str | Path) -> Path:
    """Write an arena to ``path`` as JSON. Returns the path written."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(arena_to_dict(spec), indent=1), encoding="utf-8")
    return path


def load_arena(path: str | Path, validate: bool = True):
    """Read an arena written by :func:`save_arena`.

    Validated by default. A map on disk has been edited by hand more often than one in memory,
    and a map that quietly violates a published constraint would invalidate every run it is
    ever used for -- exactly what ``generate_arena`` refuses to do at the other end.
    """
    from .arena import validate_arena

    spec = arena_from_dict(json.loads(Path(path).read_text(encoding="utf-8")))
    if validate:
        validate_arena(spec)
    return spec
