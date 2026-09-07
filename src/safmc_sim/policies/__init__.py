"""Reference policies.

Exactly one, deliberately: a port of an externally published result. Nothing invented here
ships as a policy, because a strategy written by the person who wrote the simulator is not a
baseline -- it is the simulator's own assumptions wearing a policy's clothes.

Write yours against :mod:`safmc_sim.api`, and copy from :mod:`safmc_sim.toolbox` if useful.
"""

from . import sdlw, wasp_v5  # noqa: F401  -- importing registers built-ins

__all__ = ["sdlw", "wasp_v5"]
