"""Compare strategies properly: many seeds, both collision modes, full metrics.

The point of this example is the METHOD, not the numbers. A single-seed result is not a
result -- the rulebook guarantees the arena layout is not given in advance, so every arena is
a draw from a distribution.

`sdlw` is the only policy that ships. Import your own alongside it -- that is the comparison
this simulator exists to make.

Run:  python examples/02_compare_policies.py
"""

from safmc_sim import policies  # noqa: F401 -- registers sdlw
from safmc_sim.metrics import compute_metrics, summarise
from safmc_sim.recorder import Recorder
from safmc_sim.runner import RunConfig, run

import examples  # noqa: F401
from examples.hello_import import ensure_hello  # see below

SEEDS = range(5)

if __name__ == "__main__":
    ensure_hello()
    for mode in ("unobstructed", "stop"):
        collected = []
        for policy in ("sdlw", "hello"):
            for seed in SEEDS:
                directory = f"runs/cmp_{policy}_{mode}_s{seed}"
                run(
                    RunConfig(
                        seed=seed, n_drones=12, policy=policy,
                        duration_s=180.0, collision_behaviour=mode,
                    ),
                    recorder=Recorder(directory),
                )
                collected.append(compute_metrics(directory))
        print(f"\n=== collision_behaviour = {mode} ===")
        print(summarise(collected))

    print(
        "\nRead 'unobstructed' as the search-strategy comparison and 'stop' as the "
        "survivability question. They are two experiments. Coverage per live-minute is NOT a "
        "clean correction for crashes -- it rewards dying early. See docs/07-logging-and-viz.md."
        "\n\nNote sdlw never lands, so it scores zero by design -- compare it on coverage, "
        "not on score."
    )
