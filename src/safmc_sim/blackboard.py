"""Inter-agent data exchange -- and the seam where a real radio model will go.

v0.1 ships a **perfect shared blackboard** by team decision (ADR-0003): every agent sees
everything, instantly, for free. That is defensible for this system specifically, because the
real topology is not a mesh -- every drone is a WiFi station talking to one laptop, which
already holds every drone's position and re-broadcasts peer lists at 5 Hz
(esp-everything/laptop/comms.py:288-311). There is no drone-to-drone radio at all.

It is still a simplification, and the honest claim from a v0.1 result is "given free
communication". R-SEAM-2 requires that this class is the *only* channel between agents, so
adding range limits, packet loss and latency later is one new subclass and zero changes to any
policy.

Double buffering is not an implementation detail. Reads within a tick come from a frozen
snapshot and writes land in the back buffer, so no agent can see another's publication in the
same tick and results cannot depend on the order agents were stepped (R-POL-8).
"""

from __future__ import annotations

import copy
from abc import ABC, abstractmethod
from types import MappingProxyType
from typing import Any, Mapping

import numpy as np

__all__ = ["Blackboard", "PerfectBlackboard", "frozen"]


def frozen(value: Any) -> Any:
    """An immutable equivalent of a published value: tuples, mapping proxies, read-only arrays.

    Deep-copying on publish protects readers from the *publisher* (see below). It does not
    protect readers from each other: every reader was handed the same copy, so a policy that
    appended to a peer's list changed what every agent stepped after it saw in the same tick
    -- the R-POL-8 order dependence again, from the reading side. Freezing at commit makes
    the snapshot structurally read-only instead of read-only by convention.
    """
    if isinstance(value, Mapping):
        return MappingProxyType({k: frozen(v) for k, v in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(frozen(v) for v in value)
    if isinstance(value, (set, frozenset)):
        return frozenset(frozen(v) for v in value)
    if isinstance(value, np.ndarray):
        out = np.array(value, copy=True)
        out.flags.writeable = False
        return out
    return value


class Blackboard(ABC):
    """The contract a communications model must satisfy."""

    @abstractmethod
    def snapshot(self, reader_id: str) -> Mapping[str, Mapping[str, Any]]:
        """What ``reader_id`` can see this tick, keyed by publisher agent id.

        A lossy or range-limited implementation returns different views to different readers;
        that is exactly why the reader's identity is a parameter here even though the perfect
        implementation ignores it.
        """

    @abstractmethod
    def publish(self, agent_id: str, values: Mapping[str, Any]) -> None:
        """Stage ``values`` for visibility from the next tick."""

    @abstractmethod
    def commit(self) -> None:
        """End of tick: make staged publications visible."""

    @abstractmethod
    def reset(self) -> None: ...


class PerfectBlackboard(Blackboard):
    """Lossless, rangeless, latency-free apart from the deliberate one-tick delay."""

    def __init__(self) -> None:
        self._front: dict[str, Mapping[str, Any]] = {}
        self._back: dict[str, dict[str, Any]] = {}
        self._frozen: Mapping[str, Mapping[str, Any]] = MappingProxyType({})

    def snapshot(self, reader_id: str) -> Mapping[str, Mapping[str, Any]]:
        return self._frozen

    def publish(self, agent_id: str, values: Mapping[str, Any]) -> None:
        if not values:
            return
        # Deep-copy on publish. Storing by reference let a publisher keep a handle on a
        # mutable value it had already broadcast: mutating it mid-tick changed what agents
        # stepped *after* the publisher read, while agents stepped before it saw the old
        # value. That is precisely the index-order dependence the double buffer exists to
        # prevent (R-POL-8), and it is invisible until a policy publishes a dict or a list.
        self._back.setdefault(agent_id, {}).update(copy.deepcopy(dict(values)))

    def commit(self) -> None:
        for agent_id, values in self._back.items():
            merged = dict(self._front.get(agent_id, {}))
            merged.update({k: frozen(v) for k, v in values.items()})
            self._front[agent_id] = MappingProxyType(merged)
        self._back.clear()
        self._frozen = MappingProxyType(dict(self._front))

    def reset(self) -> None:
        self._front.clear()
        self._back.clear()
        self._frozen = MappingProxyType({})
