"""Compare SDLW and all WASP v5 modes with the shared mission wrapper.

Each run executes in its own process because ir-sim owns process-global RNG state.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from itertools import product
from pathlib import Path

from safmc_sim import policies  # noqa: F401
from safmc_sim.metrics import compute_metrics, summarise
from safmc_sim.recorder import Recorder
from safmc_sim.runner import RunConfig, run

DEFAULT_SEEDS = 5
POLICIES = (
    "sdlw",
    "wasp_v5_uniform_levy",
    "wasp_v5_circular_resultant",
    "wasp_v5_pca_heading",
)
COLLISION_MODE = "stop"
MAX_WORKERS = 8


def run_one(policy: str, mode: str, seed: int) -> tuple[str, str, int, object]:
    directory = Path(f"runs/cmp_{policy}_{mode}_s{seed}")
    run(
        RunConfig(
            seed=seed,
            n_drones=12,
            policy=policy,
            duration_s=180.0,
            policy_config={"mission_wrapper": True},
            collision_behaviour=mode,
        ),
        recorder=Recorder(directory, overwrite=True),
    )
    return policy, mode, seed, compute_metrics(directory)


def run_job(job: tuple[str, str, int]) -> tuple[str, str, int, object]:
    return run_one(*job)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workers",
        type=int,
        default=MAX_WORKERS,
        help=f"concurrent run processes (default: {MAX_WORKERS})",
    )
    parser.add_argument(
        "--seeds",
        type=int,
        default=DEFAULT_SEEDS,
        help=f"number of seeds from 0 (default: {DEFAULT_SEEDS})",
    )
    args = parser.parse_args()
    if args.workers < 1:
        parser.error("--workers must be at least 1")
    if args.seeds < 1:
        parser.error("--seeds must be at least 1")
    return args


def main() -> None:
    args = parse_args()
    jobs = list(product(POLICIES, (COLLISION_MODE,), range(args.seeds)))
    results = []
    print(f"Running {len(jobs)} comparisons with {args.workers} workers.")
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        for result in executor.map(run_job, jobs):
            results.append(result)
    metrics = [metric for _, result_mode, _, metric in results if result_mode == COLLISION_MODE]
    print(f"\n=== collision_behaviour = {COLLISION_MODE} ===")
    print(summarise(metrics))


if __name__ == "__main__":
    main()
