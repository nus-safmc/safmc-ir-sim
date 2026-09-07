from pathlib import Path
import subprocess
import runpy
import sys


def test_parallel_comparison_help_exposes_workers_flag():
    script = Path(__file__).parents[1] / "examples" / "05_compare_policies_parallel.py"
    result = subprocess.run(
        [sys.executable, str(script), "--help"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "--workers" in result.stdout
    assert "--seeds" in result.stdout
    assert "--collision-modes" in result.stdout


def test_parallel_comparison_parses_seed_count(monkeypatch):
    script = Path(__file__).parents[1] / "examples" / "05_compare_policies_parallel.py"
    module = runpy.run_path(script)
    monkeypatch.setattr(sys, "argv", [str(script), "--seeds", "100", "--collision-modes", "stop"])
    args = module["parse_args"]()
    assert args.seeds == 100
    assert args.collision_modes == ("stop",)
