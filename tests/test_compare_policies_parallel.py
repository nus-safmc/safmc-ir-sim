from pathlib import Path
import subprocess
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
