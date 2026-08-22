"""Register the example policy from `01_hello_policy.py` without running it."""

import importlib.util
from pathlib import Path


def ensure_hello() -> None:
    spec = importlib.util.spec_from_file_location(
        "_hello_policy", Path(__file__).with_name("01_hello_policy.py")
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
