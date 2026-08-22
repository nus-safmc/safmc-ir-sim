"""Command line entry point: run, sweep, replay.

    safmc-run run     --policy sdlw --drones 12 --seed 3
    safmc-run run     --import my_search.py --policy my_search --drones 12
    safmc-run sweep   --policy sdlw my_policy --seeds 0-9 --drones 12
    safmc-run replay  runs/my_run
    safmc-run policies

A sweep runs each combination in a **separate process**. That is not an optimisation, it is a
correctness requirement: ir-sim seeds one process-global RNG, so two environments in one
process cannot have independent seeded streams (R-DET-4).
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

from . import policies  # noqa: F401 -- registers the built-in policies
from .api import policy_names
from .recorder import Recorder
from .runner import RunConfig, run as run_once

__all__ = ["main"]


def load_policy_module(target: str) -> None:
    """Import a file or module so that its ``@register_policy`` decorators run.

    Without this there is no way to run a policy you wrote: the registry is populated by
    import side-effect, and nothing imports your file. Accepts either a path to a .py file or
    a dotted module name, because both are things people reasonably try.
    """
    path = Path(target)
    if path.suffix == ".py":
        if not path.exists():
            raise SystemExit(f"no such file: {path}")
        spec = importlib.util.spec_from_file_location(path.stem, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[path.stem] = module
        spec.loader.exec_module(module)
        return
    try:
        importlib.import_module(target)
    except ImportError as exc:
        raise SystemExit(
            f"could not import {target!r}: {exc}. Pass a path to a .py file, or a module "
            f"name that is importable from the current directory."
        ) from exc


def _parse_seeds(text: str) -> list[int]:
    seeds: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if "-" in part[1:]:
            lo, hi = part.split("-", 1)
            seeds.extend(range(int(lo), int(hi) + 1))
        elif part:
            seeds.append(int(part))
    if not seeds:
        raise SystemExit(f"no seeds parsed from {text!r}")
    return seeds


def _config_from_args(args, seed: int, policy: str) -> RunConfig:
    return RunConfig(
        seed=seed,
        n_drones=args.drones,
        policy=policy,
        policy_config=json.loads(args.policy_config) if args.policy_config else {},
        duration_s=args.duration,
        collision_behaviour=args.collision,
        record=not args.no_record,
    )


def _run_and_report(config: RunConfig, out_dir: Path | None):
    recorder = Recorder(out_dir) if out_dir is not None else None
    result = run_once(config, recorder=recorder)
    return {
        "policy": config.policy,
        "seed": config.seed,
        "drones": config.n_drones,
        "score": result.score.total,
        "raw": result.score.raw_total,
        "relay": result.score.relay_formed,
        "landed": len(result.landed),
        "crashed": len(result.crashed),
        "sim_time_s": round(result.sim_time_s, 1),
        "wall_time_s": round(result.wall_time_s, 1),
        "log": result.log_path,
    }


def _sweep_worker(payload):
    config, out_dir = payload
    return _run_and_report(config, Path(out_dir) if out_dir else None)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="safmc-run", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    def common(p):
        p.add_argument(
            "--import", dest="import_", action="append", default=[], metavar="FILE_OR_MODULE",
            help="import a .py file or module so its @register_policy runs. Repeatable.",
        )
        p.add_argument("--drones", type=int, default=12,
                       help="fleet size, 10-25 (default: %(default)s)")
        p.add_argument("--duration", type=float, default=600.0,
                       help="simulated seconds (default: %(default)s)")
        p.add_argument(
            "--collision", choices=("stop", "unobstructed"), default="stop",
            help="stop: a collision ends that drone's run (default). unobstructed: nothing "
                 "can crash -- use this as the control when comparing search strategies.",
        )
        p.add_argument("--policy-config", default=None,
                       help='JSON object passed to the policy, e.g. \'{"speed": 0.3}\'')
        p.add_argument("--no-record", action="store_true", help="do not write a log")
        p.add_argument("--out", type=Path, default=Path("runs"),
                       help="where logs go (default: %(default)s)")

    p_run = sub.add_parser("run", help="one run")
    p_run.add_argument("--policy", default="sdlw")
    p_run.add_argument("--seed", type=int, default=0)
    common(p_run)

    p_sweep = sub.add_parser("sweep", help="policies x seeds, one process per run")
    p_sweep.add_argument("--policy", nargs="+", required=True)
    p_sweep.add_argument("--seeds", default="0-9")
    p_sweep.add_argument("--workers", type=int, default=4)
    common(p_sweep)

    p_replay = sub.add_parser("replay", help="build an HTML replay from a recorded run")
    p_replay.add_argument("run_dir", type=Path)
    p_replay.add_argument("--pose-every", type=int, default=1,
                          help="keep every Nth tick; raise it for long runs (default: 1)")
    p_replay.add_argument("--tof-every", type=int, default=10,
                          help="keep every Nth ToF frame (default: 10)")

    p_pol = sub.add_parser("policies", help="list registered policies")
    p_pol.add_argument(
        "--import", dest="import_", action="append", default=[], metavar="FILE_OR_MODULE",
        help="import a .py file or module first, to check it registers",
    )

    args = parser.parse_args(argv)

    for target in getattr(args, "import_", []):
        load_policy_module(target)

    if args.command == "policies":
        for name in policy_names():
            print(name)
        return 0

    if args.command == "replay":
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
        import viz

        payload = viz.build_payload(args.run_dir, args.tof_every, args.pose_every)
        output = args.run_dir / "replay.html"
        output.write_text(
            viz.TEMPLATE.replace("__PAYLOAD__", json.dumps(payload, separators=(",", ":"))),
            encoding="utf-8",
        )
        print(output)
        return 0

    if args.command == "run":
        config = _config_from_args(args, args.seed, args.policy)
        out_dir = None if args.no_record else args.out / f"{args.policy}_s{args.seed}"
        report = _run_and_report(config, out_dir)
        print(json.dumps(report, indent=2))
        return 0

    seeds = _parse_seeds(args.seeds)
    jobs = []
    for policy in args.policy:
        for seed in seeds:
            config = _config_from_args(args, seed, policy)
            out_dir = None if args.no_record else str(args.out / f"{policy}_s{seed}")
            jobs.append((config, out_dir))

    print(f"{len(jobs)} runs across {args.workers} processes", file=sys.stderr)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        reports = list(pool.map(_sweep_worker, jobs))

    header = f"{'policy':<14}{'seed':>5}{'score':>7}{'raw':>6}{'landed':>8}{'crashed':>9}{'wall_s':>8}"
    print(header)
    print("-" * len(header))
    for r in reports:
        print(
            f"{r['policy']:<14}{r['seed']:>5}{r['score']:>7}{r['raw']:>6}"
            f"{r['landed']:>8}{r['crashed']:>9}{r['wall_time_s']:>8.1f}"
        )
    by_policy: dict[str, list[int]] = {}
    for r in reports:
        by_policy.setdefault(r["policy"], []).append(r["score"])
    print()
    print(f"{'policy':<14}{'n':>4}{'mean':>8}{'min':>6}{'max':>6}")
    for policy, scores in by_policy.items():
        print(
            f"{policy:<14}{len(scores):>4}{sum(scores)/len(scores):>8.1f}"
            f"{min(scores):>6}{max(scores):>6}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
