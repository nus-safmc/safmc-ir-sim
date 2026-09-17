"""Form the scoring relay with UWB: an elastic band of drones along a teammate's trail.

The relay of rulebook 3.3.7 (R-MISS-4) is a chain of **landed** drones from the one that rescued
a bonus victim back into the Start Area, every adjacent pair at most 1.0 m apart with floor-level
line of sight, and it doubles the whole score. This example is the trial ADR-0007 specifies: tags
on every drone, **one anchor** in the Start Area, and a potential-field controller that converges
the relay drones onto a chain and lands them on a UWB measurement alone.

What it shows, in order:

1. **Roles.** ``drone_00`` is the *lead* -- a wasp_v5 searcher that lands only on a bonus victim.
   ``n_relay`` drones are *relays*, taken from the southern take-off row first so that no parked
   relay sits north of a searcher (:func:`relay_roles`). Everyone else is an unmodified wasp_v5
   searcher with the mission wrapper. Every searcher first flies straight north out of the Start Area along
   its own column, then walks; every searcher publishes a **breadcrumb** each 0.25 m of travel,
   with its ring's clearance, and announces itself as a **head** when it lands on a bonus victim.
2. **The trail.** A relay assembles a head's trail from the anchor northwards and **cuts loops**
   where the head revisited a spot and its own ring had seen the chord empty -- flyable and in
   line of sight without a map. The trail's length ``L`` decides how many relays the chain
   needs: ``ceil(L / 0.9) - 1``.
3. **The potential.** Node 0 is the anchor, nodes ``1..n`` the relays, node ``n+1`` the head.
   Each relay carries an arclength ``s`` along the trail and, once in place, descends

       U = 1/2 k (r_pred - d*)^2 + 1/2 k (r_succ - d*)^2
       ds = -K (r_pred - d*) + K (r_succ - d*)          per fresh sweep

   on **measured** UWB ranges to its two chain neighbours, each reduced to the horizontal by the
   altitude it knows the neighbour to be at. With both ends fixed the chain converges to equal
   spacing ``L / (n + 1)`` whatever ``d*`` is; the trail fixes the joint angles a range-only chain
   cannot (ADR-0007). The two-dimensional command tracks the trail point at ``s``.
4. **The landing gate is UWB alone.** A relay is *in place* when both neighbour ranges, reduced
   to the horizontal, have measured at most 0.9 m for three consecutive fresh sweeps -- both
   ends of every link check it, so every relay in place means every link inside the rule's
   *distance*. A range cannot certify the rule's floor-level line of sight; that comes from
   the trail's construction (flown segments and ring-evidenced chords) and is checked by the
   grader, not measured by the gate. The tail's predecessor is the anchor, and with the
   anchor at ``y = 5.0`` a 0.9 m horizontal range puts it inside the Start Area **by
   measurement**. When every relay on the trail is in place in the same blackboard snapshot,
   every relay lands on the same tick.
5. **Two deployments, one controller.** ``mode="dispatch"``: relays wait on the ground until a head
   has landed, pick the cheapest feasible head, and launch in single file. ``mode="train"``: relays
   follow the lead's growing trail from take-off at equal spacing between the anchor and the lead,
   joining one at a time, and hold when the lead lands.
6. **Grading from the log alone** (T-2, T-3 in ADR-0007): every link of the recorded chain checked
   by the mission's own rule from ``states.npz``, and every landed relay's last fresh peer ranges
   checked against the gate from ``uwb.npz``.

**Read this before quoting a number.** Trail following, the carrot, the approach, the
crumbs themselves and the ``start``/``xy`` publications that pace launches all run on
ground-truth pose, and every horizontal correction uses the relay's own true altitude; crumbs,
head announcements, chain order and the landing consensus run on the perfect blackboard
(ADR-0003). Roles come from the tag's roster, not from pose. The two decisions that survive
those caveats are the ones the trial exists to test: the band's direction and the landing
gate on measured peer ranges, and the tail certified by one anchor range. Every UWB number is
A-14..A-19 and none is measured on the team's kit. F-36.

Run:  python examples/06_uwb_relay.py                       # one dispatch run, seed 0, graded
      python examples/06_uwb_relay.py --mode train --n-relay 6 --seed 3
      python examples/06_uwb_relay.py --sweep --seeds 5     # dispatch vs train vs no relay
"""

from __future__ import annotations

import argparse
import dataclasses
import math
from concurrent.futures import ProcessPoolExecutor
from itertools import product
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from safmc_sim import policies  # noqa: F401 -- registers wasp_v5
from safmc_sim.api import Command, Land, Observation, Policy, Velocity, register_policy
from safmc_sim.constants import (
    CRUISE_ALT_M,
    CRUISE_SPEED_MS,
    DRONE_RADIUS_M,
    START_AREA_DEPTH_M,
)
from safmc_sim.errors import PolicyError
from safmc_sim.metrics import relay_timeline
from safmc_sim.mission import takeoff_waves
from safmc_sim.policies.mission_wrapper import MissionWrapper
from safmc_sim.policies.wasp_v5 import WaspV5Policy
from safmc_sim.recorder import Recorder, arena_from_log, load_run
from safmc_sim.runner import RunConfig, flown_sensors, run
from safmc_sim.sensors.raycast import segment_clear
from safmc_sim.sensors.uwb import UWBConfig, UWBRanges, peer_sweep_rate_hz
from safmc_sim.toolbox import body_to_world
from safmc_sim.world.arena import ArenaConfig
from safmc_sim.world.landmark import Landmark

# ------------------------------------------------------------------------------------------
# The trial's numbers. Deployment choices, not claims about the world.
# ------------------------------------------------------------------------------------------

ANCHOR_Y = 5.0
"""The anchor row. What makes the tail's certificate work: 0.9 m of horizontal range from
``y = 5.0`` cannot reach ``y = 6.0``, so a tail that measures itself within 0.9 m of an anchor
here is in the Start Area by measurement."""

ANCHOR_X0 = 1.75
"""The first anchor stands on the lead's launch column -- the middle of the take-off grid's
first column (``START_WALL_MARGIN_M + U(0, 0.5)``) -- so the lead's north leg passes within
0.25 m of it."""

ANCHOR_SPACING_M = 1.9
"""With more than one anchor, their spacing along the row: a head's crossing point is then
never more than 0.95 m from an anchor, so the chain's Start-Area leg costs at most one relay.
Rule 3.3.1 r.16 allows any number of navigation aids in the Start Area."""


ANCHOR_CLEARANCE_M = 0.95
"""An anchor must have no inner wall or pillar within this: the 0.9 m disc the network treats
as free space (``MAX_LINK_M``), plus half a wall's thickness. The Start Area is supposed to be empty, but the
generator on ``main`` lets an inner wall reach below the line -- 22 of 200 seeds, as low as
``y = 4.8`` -- so an anchor with structure inside its disc is dropped for that seed. A first
version used 1.2 m, which also caught the room's *legal* south face (``y0`` as low as 6.05,
1.05 m from the row) and silently thinned the row on 46 of 200 seeds, two of them in the
sweep; the skeptic found it. At 0.95 m only the walls that really reach into the row drop
anchors. With a single anchor there is nothing to fall back to, so it is kept regardless and
the run's report says so."""


def anchor_row(n_anchors: int, arena=None) -> tuple[Landmark, ...]:
    """``n_anchors`` anchors along ``y = ANCHOR_Y`` from the lead's column eastwards.

    One is the brief this trial was given, and the default. One certification point means the
    chain must come back to it: a head that landed 12 m east of it, a metre north of the line,
    needed sixteen relays on the first full run. A row makes the certificate available along
    the whole line, and the sweep prices the difference. With ``arena`` given, an anchor that
    has structure within :data:`ANCHOR_CLEARANCE_M` is left out (see the constant), the first
    included -- unless it is the only one.
    """
    if n_anchors < 1:
        raise ValueError("at least one anchor")
    xs = [ANCHOR_X0 + k * ANCHOR_SPACING_M for k in range(n_anchors)]
    if xs[-1] > 19.5:
        raise ValueError(f"{n_anchors} anchors at {ANCHOR_SPACING_M} m run off the field")
    row = []
    for k, x in enumerate(xs):
        if arena is not None and structure_within(arena, x, ANCHOR_Y, ANCHOR_CLEARANCE_M):
            continue
        row.append(Landmark(f"start_anchor_{k}", "uwb_anchor", x, ANCHOR_Y))
    if not row:
        # Every anchor has structure inside its disc (3 of 200 seeds for the lead's column
        # alone). One anchor is still needed for the tail's certificate; keep the first.
        row.append(Landmark("start_anchor_0", "uwb_anchor", xs[0], ANCHOR_Y))
    return tuple(row)


def structure_within(arena, x: float, y: float, radius_m: float) -> bool:
    """Is any inner wall or pillar within ``radius_m`` of ``(x, y)``? Perimeter walls excluded."""
    p = np.array([x, y])
    for w in arena.walls:
        if w.kind in ("perimeter_wall", "net"):
            continue
        a, b = np.array([w.x1, w.y1]), np.array([w.x2, w.y2])
        ab = b - a
        t = float(np.clip(np.dot(p - a, ab) / max(float(np.dot(ab, ab)), 1e-9), 0.0, 1.0))
        if float(np.linalg.norm(p - (a + t * ab))) < radius_m + w.thickness_m / 2:
            return True
    for pil in arena.pillars:
        if float(np.hypot(pil.x - x, pil.y - y)) < radius_m + max(pil.radius_m, 0.25):
            return True
    return False

D_STAR_M = 0.8
"""Rest spacing of the band: the 1.0 m rule less four A-14 standard deviations."""

MAX_LINK_M = 0.9
"""What a link may *measure* and still be landed on. The 0.1 m margin against the rule covers
A-14 -- two sigmas, and three consecutive sweeps make a true 1.0 m link's chance of passing
about 1e-5 -- and nothing else. It does **not** cover the per-unit antenna-delay offsets the
model has no term for: 12 cm between two calibrated DWM3001Cs, up to 30 cm between two
DWM3000s out of the box (F-31, F-34). On hardware, lower this by the offset you measure."""

ANCHOR_HEIGHT_M = 0.5
"""The anchor's antenna stands at cruise altitude, not on the 2.0 m tripod the anchor model
defaults to. A range is three-dimensional, and the tail reduces its anchor range to the
horizontal by the height gap: at 0.77 m across and 1.5 m up that multiplies A-14's 5 cm by
``r / h`` = 2.2, and the tail's certificate flickered. At tag height there is no dilution.
A deployment choice the team makes with a shorter stand."""

BAND_DEADBAND_M = 0.02
"""A band step smaller than this is noise (A-14 through ``K``) and is not commanded."""

TRAIL_ENTRY_M = 1.5
"""A relay may join the trail anywhere on its first this-many metres, at the arclength of the
nearest trail point, instead of only at the anchor. See :meth:`RelayNode._approach`."""

ON_CARROT_M = 0.12
"""The band moves a relay's carrot only while the relay is this close to it. The airframe
answers a velocity through a 0.35 s lag (A-2); a band that stepped every 0.2 s sweep while
the drone was still catching up overshot and never settled."""

SETTLE_SWEEPS = 3
"""Consecutive fresh sweeps the gate must pass before a relay declares itself in place."""

BAND_GAIN = 0.25
"""Per-sweep gain ``K`` of the discrete band. The chain's coupling has eigenvalues in (0, 4),
so the update is stable below 0.5 and dead-beat for the fastest mode at 0.25."""

CRUMB_M = 0.25
"""A searcher drops a breadcrumb every this far. One 4-tuple on the blackboard per crumb."""

CUT_M = 0.6
"""A new crumb this close to an earlier one is a candidate loop cut."""

CUT_CLEARANCE_CAP_M = 0.8
"""Ring clearance is trusted only up to this. The eight rangers sit at the airframe's rim,
not at its centre, so at the boundary bearing between two rangers a 0.10 m wall end-on falls
between the last ray of one and the first of the next from about 0.79 m out (ranger-origin
parallax, not zone width -- measured in the audit). A chord the cutter accepts is at most
``0.8 - 0.18`` = 0.62 m, inside the range where every wall is seen."""

LINE_CLEAR_M = 0.4
"""How far past the start line a searcher's north leg runs before it starts walking."""

LEG_STAGGER_S = 1.5
"""Drones start their north leg this far apart in time, in three bands by drone index mod 3
(column mod 3 in the first row; the second row is offset by one band). Every
searcher starts from the same row at the same speed, so without this they all reached the
room's face together, all handed over to wasp_v5 with the same wall ahead, and neighbours
1.25 m apart turned into each other -- two head-on losses at t = 14 s on seed 2 that the
plain wasp_v5 baseline, which never lines up, does not show. A stagger in *height* did not
help, because the handover fires at a fixed distance from the wall whatever the band."""

BLOCKED_M = 0.9
"""A ring return this close in the direction of travel counts as blocked, and a drone flying
to a point slides along the obstacle instead of pushing on it."""

HANDOVER_M = 1.4
"""A searcher on its north leg hands over to wasp_v5 -- which knows how to walk along a wall
-- when the ring sees anything this far ahead. The room's south face can stand as little as
0.05 m north of the start line (``y0`` is drawn from [6.05, 7.95]), so a column that meets
the face rather than its doorway meets it before the leg's nominal end. At 0.9 m the handover
left wasp_v5 two seconds at cruise to turn a drone away from a corner, and its 0.4 m
repulsion did not; at 1.4 m it has three, and the same seeds no longer crash."""

APPROACH_GAIN = 2.0
"""Proportional gain from position error to commanded speed, saturated at cruise."""

AVOID_RANGE_M = 0.6
AVOID_GAIN = 0.6
"""wasp_v5's linear-falloff repulsion, a little wider, for relays in transit."""

CHAIN_AVOID_RANGE_M = 0.3
"""On the trail the repulsion narrows to an emergency stop. Chain neighbours sit 0.6-0.9 m
apart centre to centre -- 0.24-0.54 m between bodies -- and a 0.6 m repulsion fired on every
one of them, pushing relays off the trail and against the band. Measured before this
constant existed: one relay's arclength slammed between 0 and 2.3 m every few sweeps."""

MAX_BAND_STEP_M = 0.3
"""Largest arclength change one sweep may command; a dropout or an outlier cannot yank a
relay more than this."""

# Relay states, published as small ints so the blackboard payload stays tiny.
GROUND, APPROACH, FOLLOW, BAND, DOWN = 0, 1, 2, 3, 4


# ------------------------------------------------------------------------------------------
# The trail: crumbs, arclength, loop cuts. Pure geometry.
# ------------------------------------------------------------------------------------------


class Trail:
    """A polyline of crumbs with cumulative arclength and map-free loop cutting.

    A crumb is ``(x, y, clearance)`` where ``clearance`` is the minimum range the dropping
    drone's ring saw there. A new crumb within :data:`CUT_M` of an earlier one, whose chord
    fits inside either crumb's free disc with room for a body, replaces everything between:
    the chord lies inside a disc the ring saw empty, so it is flyable and in line of sight.
    """

    def __init__(self) -> None:
        self.xy: list[np.ndarray] = []
        self.clear: list[float] = []
        self.cum: list[float] = []

    def __len__(self) -> int:
        return len(self.xy)

    @property
    def length(self) -> float:
        return self.cum[-1] if self.cum else 0.0

    def append(self, x: float, y: float, clearance: float) -> None:
        p = np.array([float(x), float(y)])
        if self.xy and float(np.linalg.norm(p - self.xy[-1])) < 1e-6:
            return
        if len(self.xy) >= 3:
            earlier = np.array(self.xy[:-2])
            chord = np.linalg.norm(earlier - p, axis=1)
            free = np.minimum(np.array(self.clear[:-2]), CUT_CLEARANCE_CAP_M)
            free = np.maximum(free, min(clearance, CUT_CLEARANCE_CAP_M))
            ok = (chord <= CUT_M) & (chord + DRONE_RADIUS_M <= free)
            if ok.any():
                q = int(np.flatnonzero(ok)[0])
                del self.xy[q + 1:]
                del self.clear[q + 1:]
                del self.cum[q + 1:]
        self.xy.append(p)
        self.clear.append(float(clearance))
        self.cum.append(self.cum[-1] + float(np.linalg.norm(p - self.xy[-2])) if len(self.xy) > 1 else 0.0)

    def project(self, xy: np.ndarray, s_max: float) -> tuple[float, float]:
        """``(s, distance)`` of the closest point to ``xy`` on the trail's first ``s_max`` metres."""
        p = np.asarray(xy, dtype=float)
        best_s, best_d = 0.0, float(np.linalg.norm(p - self.xy[0]))
        for j in range(1, len(self.xy)):
            if self.cum[j - 1] > s_max:
                break
            a, b = self.xy[j - 1], self.xy[j]
            ab = b - a
            span = float(np.dot(ab, ab))
            t = float(np.clip(np.dot(p - a, ab) / span, 0.0, 1.0)) if span > 0 else 0.0
            d = float(np.linalg.norm(p - (a + t * ab)))
            if d < best_d:
                best_d, best_s = d, self.cum[j - 1] + t * (self.cum[j] - self.cum[j - 1])
        return min(best_s, s_max), best_d

    def point_at(self, s: float) -> np.ndarray:
        """The point at arclength ``s``, clamped to the trail's ends."""
        if not self.xy:
            raise ValueError("empty trail")
        if len(self.xy) == 1 or s <= 0.0:
            return self.xy[0]
        if s >= self.cum[-1]:
            return self.xy[-1]
        j = int(np.searchsorted(self.cum, s, side="right"))          # cum[j-1] <= s < cum[j]
        a, b = self.xy[j - 1], self.xy[j]
        span = self.cum[j] - self.cum[j - 1]
        t = (s - self.cum[j - 1]) / span if span > 0 else 0.0
        return a + t * (b - a)


class TrailBuilder:
    """A head's trail as a relay follows it, built incrementally as crumbs arrive: the anchor
    nearest to where the head's north leg reached the anchor row, then the crumbs from there.
    The first hop is along the row, Start Area free space; the loop cutter handles the rest.
    Incremental because a 600 s Lévy walk drops a thousand crumbs, and recomposing the
    trail from scratch each tick was quadratic -- one run took six minutes of wall time."""

    def __init__(self, anchors_xy: np.ndarray) -> None:
        self.anchors_xy = np.asarray(anchors_xy, dtype=float).reshape(-1, 2)
        self.anchor_y = float(self.anchors_xy[0, 1])
        self.anchor_index: int | None = None
        self.trail = Trail()
        self.consumed = 0

    def update(self, crumbs: list[tuple[float, float, float]]) -> Trail:
        for x, y, c in crumbs[self.consumed:]:
            if self.anchor_index is None:
                if y < self.anchor_y:
                    continue
                self.anchor_index = int(np.argmin(np.abs(self.anchors_xy[:, 0] - x)))
                ax, ay = self.anchors_xy[self.anchor_index]
                self.trail.append(ax, ay, CUT_CLEARANCE_CAP_M)
            self.trail.append(x, y, c)
        self.consumed = len(crumbs)
        return self.trail


def compose_trail(anchors_xy: np.ndarray, crumbs: list[tuple[float, float, float]]) -> TrailBuilder:
    """The whole trail at once; see :class:`TrailBuilder`."""
    builder = TrailBuilder(anchors_xy)
    builder.update(crumbs)
    return builder


class CrumbNetwork:
    """Every searcher's crumbs as one graph, and the shortest flyable route from a head to an anchor.

    Nodes are the anchors and every crumb every searcher has published. Edges are (a)
    consecutive crumbs of one searcher -- a segment that drone flew; (b) cross-links between
    any two crumbs within :data:`CUT_M` whose chord, plus a body radius, fits inside either
    crumb's ring-clearance disc -- the loop cutter's test, applied between trails as well as
    along one; (c) an anchor to any crumb within :data:`MAX_LINK_M` horizontally, which is
    Start Area free space by the anchor row's position. Dijkstra from the head's landing crumb
    to the nearest anchor is then the chain's trail. Still no map: every edge is a segment a
    drone flew or a chord a ring saw empty.

    Why: a head's own trail is what it *flew*, and a Lévy walker that landed a metre north of
    the line may have wandered six metres east first. Five other searchers crossed that strip
    on their way north; their crumbs are the shortcut.
    """

    def __init__(self, anchors_xy: np.ndarray) -> None:
        self.anchors_xy = np.asarray(anchors_xy, dtype=float).reshape(-1, 2)

    def route(self, crumbs_by_searcher: Mapping[str, list[tuple[float, float, float]]],
              head_id: str) -> tuple[Trail, int] | None:
        """``(trail, anchor_index)`` from the nearest anchor to ``head_id``'s last crumb, or None."""
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import dijkstra

        n_anchor = len(self.anchors_xy)
        xy_list = [self.anchors_xy]
        clear_list = [np.full(n_anchor, CUT_CLEARANCE_CAP_M)]
        rows, cols, weights = [], [], []
        offset = n_anchor
        head_node = None
        for sid, crumbs in crumbs_by_searcher.items():
            if not crumbs:
                continue
            arr = np.array(crumbs, dtype=float)
            n = len(arr)
            xy_list.append(arr[:, :2])
            clear_list.append(np.minimum(arr[:, 2], CUT_CLEARANCE_CAP_M))
            if n > 1:
                d = np.linalg.norm(np.diff(arr[:, :2], axis=0), axis=1)
                idx = np.arange(offset, offset + n - 1)
                rows.append(idx); cols.append(idx + 1); weights.append(d)
            if sid == head_id:
                head_node = offset + n - 1
            offset += n
        if head_node is None:
            return None
        xy = np.vstack(xy_list)
        clear = np.concatenate(clear_list)
        n_nodes = len(xy)

        xl, yl, wl = [], [], []
        # Anchor to crumb: any crumb within the certificate's reach, computed directly -- the
        # reach (0.9 m) is wider than the cell hash below (0.6 m), and an auditor found the
        # hashed version dropped every anchor edge between 0.6 and 0.9 m.
        crumbs_xy = xy[n_anchor:]
        for a in range(n_anchor):
            chord = np.linalg.norm(crumbs_xy - xy[a], axis=1)
            ok = np.flatnonzero(chord <= MAX_LINK_M)
            if len(ok):
                xl.append(np.full(len(ok), a)); yl.append(ok + n_anchor); wl.append(chord[ok])
        # Crumb to crumb by a cell hash: candidates within CUT_M are in the same or an adjacent cell.
        cell = np.floor(crumbs_xy / CUT_M).astype(int)
        buckets: dict[tuple[int, int], list[int]] = {}
        for i, (cx, cy) in enumerate(map(tuple, cell)):
            buckets.setdefault((cx, cy), []).append(i + n_anchor)
        for (cx, cy), members in buckets.items():
            neigh = []
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neigh.extend(buckets.get((cx + dx, cy + dy), ()))
            neigh = np.array(neigh)
            for i in members:
                js = neigh[neigh > i]
                if not len(js):
                    continue
                chord = np.linalg.norm(xy[js] - xy[i], axis=1)
                free = np.maximum(clear[js], clear[i])
                ok = (chord <= CUT_M) & (chord + DRONE_RADIUS_M <= free)
                if ok.any():
                    xl.append(np.full(int(ok.sum()), i)); yl.append(js[ok]); wl.append(chord[ok])
        if xl:
            rows.append(np.concatenate(xl)); cols.append(np.concatenate(yl)); weights.append(np.concatenate(wl))
        if not rows:
            return None
        r = np.concatenate(rows); c = np.concatenate(cols); w = np.concatenate(weights)
        graph = coo_matrix((np.concatenate([w, w]), (np.concatenate([r, c]), np.concatenate([c, r]))),
                           shape=(n_nodes, n_nodes)).tocsr()
        dist, pred = dijkstra(graph, directed=False, indices=head_node, return_predecessors=True)
        reach = dist[:n_anchor]
        if not np.isfinite(reach).any():
            return None
        a = int(np.argmin(reach))
        path = [a]
        while path[-1] != head_node:
            nxt = int(pred[path[-1]])
            if nxt < 0:
                return None
            path.append(nxt)
        trail = Trail()
        for node in path:
            trail.append(xy[node, 0], xy[node, 1], clear[node])
        return trail, a


def relays_needed(length_m: float) -> int:
    """How many relays make every link of a chain of this length at most :data:`MAX_LINK_M`:
    the anchor and the head are the fixed ends, so ``n + 1`` links of ``L / (n + 1)``."""
    return max(1, math.ceil(length_m / MAX_LINK_M - 1.0))


def relays_with_room(length_m: float) -> int:
    """How many relays a trail of this length has room for at the rest spacing -- the train's
    joining rule, so the chain is never crammed below ``d*`` while the lead is still moving."""
    return max(1, math.ceil(length_m / D_STAR_M) - 1)


def horizontal(range_m: float, dz_m: float) -> float:
    """A 3-D range reduced to the horizontal, given the altitude gap the reader knows."""
    return float(math.sqrt(max(range_m * range_m - dz_m * dz_m, 0.0)))


# ------------------------------------------------------------------------------------------
# Shared helpers for the two roles
# ------------------------------------------------------------------------------------------


def ring_repulsion(obs: Observation, range_m: float = AVOID_RANGE_M, gain: float = AVOID_GAIN) -> np.ndarray:
    """wasp_v5's linear-falloff repulsion from every zone under ``range_m``, in the world frame."""
    ranges = obs.tof.ranges_m.reshape(-1)
    bearings = obs.tof.zone_bearings_rad.reshape(-1)
    near = ranges < range_m
    if not near.any():
        return np.zeros(2)
    w = np.where(near, (range_m - ranges) / range_m, 0.0)
    push_body = -gain * np.array([np.sum(w * np.cos(bearings)), np.sum(w * np.sin(bearings))])
    return np.array(body_to_world(push_body[0], push_body[1], obs.pose.theta))


def ring_clearance(obs: Observation) -> float:
    """The nearest thing the ring sees, or its reach if nothing."""
    ranges = obs.tof.ranges_m
    finite = ranges[np.isfinite(ranges)]
    return float(finite.min()) if len(finite) else 3.0


def blocked_ahead(obs: Observation, direction_xy: np.ndarray, within_m: float = BLOCKED_M,
                  half_angle_rad: float = 0.6) -> float:
    """The nearest ring return inside ``half_angle_rad`` of a world-frame direction, or inf."""
    ranges = obs.tof.ranges_m.reshape(-1)
    bearings = obs.tof.zone_bearings_rad.reshape(-1) + obs.pose.theta          # to world frame
    heading = float(np.arctan2(direction_xy[1], direction_xy[0]))
    off = np.abs(np.arctan2(np.sin(bearings - heading), np.cos(bearings - heading)))
    sector = ranges[off <= half_angle_rad]
    nearest = float(sector.min()) if len(sector) else math.inf
    return nearest if nearest < within_m else math.inf


def toward(obs: Observation, goal_xy: np.ndarray, speed: float = CRUISE_SPEED_MS,
           avoid_range_m: float = AVOID_RANGE_M) -> Velocity:
    """A saturated proportional velocity toward ``goal_xy`` at cruise altitude, plus repulsion,
    plus a slide along whatever blocks the way.

    Pure attraction-plus-repulsion has no tangential term, so a drone driven at a wall face
    pushes on it forever: an auditor found searchers parked against the room's south face at
    ``y = 5.5`` for a whole run. With something inside :data:`BLOCKED_M` in the goal's
    direction the drone instead moves perpendicular to it, toward whichever side the ring
    reports freer, and resumes the goal once the way is clear.
    """
    delta = goal_xy - obs.pose.xy
    distance = float(np.linalg.norm(delta))
    v = delta * min(APPROACH_GAIN, speed / distance) if distance > 1e-6 else np.zeros(2)
    nearest = blocked_ahead(obs, delta) if distance > 0.3 else math.inf
    # Slide only when the obstacle is nearer than the goal. A teammate hovering just beyond
    # the goal point is in the cone too, and sliding away from it held a relay 0.6 m from its
    # carrot for the rest of a run.
    if math.isfinite(nearest) and nearest < distance - 0.1:
        d = delta / distance
        left = np.array([-d[1], d[0]])
        ranges = obs.tof.ranges_m.reshape(-1)
        bearings = obs.tof.zone_bearings_rad.reshape(-1) + obs.pose.theta
        heading = float(np.arctan2(d[1], d[0]))
        rel = np.arctan2(np.sin(bearings - heading), np.cos(bearings - heading))
        side = lambda sign: ranges[(sign * rel > 0.6) & (sign * rel < 2.2)]      # the flank, 35-125 deg
        free_left = float(np.min(side(+1))) if len(side(+1)) else math.inf
        free_right = float(np.min(side(-1))) if len(side(-1)) else math.inf
        v = (left if free_left >= free_right else -left) * speed * 0.7
    v = v + ring_repulsion(obs, range_m=avoid_range_m)
    norm = float(np.linalg.norm(v))
    if norm > speed:
        v = v * (speed / norm)
    vz = float(np.clip(2.0 * (CRUISE_ALT_M - obs.pose.z), -0.4, 0.4))
    return Velocity(vx=float(v[0]), vy=float(v[1]), vz=vz)


def require_peer_ranging(obs: Observation) -> UWBRanges:
    """T-4: this trial spaces by peer ranges and must not fall back to pose silently."""
    reading = obs.sensors.get("uwb")
    if not isinstance(reading, UWBRanges) or not reading.peer_ids:
        raise PolicyError(
            "uwb_relay needs the tag with peer ranging: "
            "RunConfig(sensors=flown_sensors() + (UWBConfig(peers=True, ...),)). "
            "Without peer ranges the relay would have to be spaced by pose, which is the "
            "one thing this trial exists not to do."
        )
    if len(reading.anchor_ids) < 1:
        raise PolicyError("uwb_relay needs at least one anchor: it is the tail's certificate")
    return reading


# ------------------------------------------------------------------------------------------
# The searcher: wasp_v5 with a north leg, breadcrumbs, and a head announcement
# ------------------------------------------------------------------------------------------


class RecordingWrapper(MissionWrapper):
    """The shared mission wrapper, restricted to some kinds, remembering what it landed on."""

    def __init__(self, kinds: Mapping[str, int], **kw) -> None:
        super().__init__(**kw)
        self.kinds = dict(kinds)
        self.landed_on = None

    def command(self, obs: Observation):
        markers = [m for m in obs.markers if m.kind in self.kinds]
        if not markers:
            return None
        marker = min(markers, key=lambda m: (self.kinds[m.kind], m.range_m))
        if marker.range_m <= self.land_range_m:
            self.landed_on = marker
            return Land()
        vx, vy = body_to_world(self.approach_speed_ms, 0.0, obs.pose.theta)
        from safmc_sim.frames import wrap_pi
        yaw_rate = float(np.clip(self.yaw_p_gain * wrap_pi(marker.bearing_rad),
                                 -self.max_yaw_rate, self.max_yaw_rate))
        return Velocity(vx=vx, vy=vy, yaw_rate=yaw_rate)


class RelaySearcher(WaspV5Policy):
    """wasp_v5 that leaves the Start Area straight north, drops crumbs, and announces a head."""

    def __init__(self, agent_id, config, rng, arena, bonus_only: bool) -> None:
        cfg = dict(config)
        cfg["mission_wrapper"] = True
        super().__init__(agent_id, cfg, rng, arena)
        kinds = {"bonus_victim": 0} if bonus_only else {"fire": 0, "bonus_victim": 1, "victim": 2}
        self.mission = RecordingWrapper(
            kinds, land_range_m=float(cfg.get("land_range_m", 0.75)),
            approach_speed_ms=float(cfg.get("approach_speed_ms", 0.25)),
            yaw_p_gain=self.yaw_p_gain, max_yaw_rate=self.max_yaw_rate,
        )
        self._crossed = False
        self._crumb_k = -1
        self._last_crumb: np.ndarray | None = None
        self._leg_end_y = START_AREA_DEPTH_M + LINE_CLEAR_M
        self._leg_start_s = LEG_STAGGER_S * (int(agent_id[-2:]) % 3)

    def _drop_crumb(self, obs: Observation, force: bool = False) -> None:
        here = obs.pose.xy
        if not force and self._last_crumb is not None \
                and float(np.linalg.norm(here - self._last_crumb)) < CRUMB_M:
            return
        self._crumb_k += 1
        self._last_crumb = here
        self.publish("crumb", (self._crumb_k, float(here[0]), float(here[1]), ring_clearance(obs)))

    def step(self, obs: Observation) -> Command:
        if obs.pose.z >= self.cruise_alt_m - 0.02:
            self._drop_crumb(obs)
        if obs.pose.z < self.cruise_alt_m - 0.02:
            return Velocity(vz=self.climb_rate_ms)
        if not self._crossed:
            if obs.sim_time_s < self._leg_start_s:
                return Velocity()                       # my band's turn has not come
            north = np.array([0.0, 1.0])
            past_the_line = obs.pose.y > self._leg_end_y
            # Past the anchor row and something ahead: the room's south face, a wall foot
            # (22 of 200 seeds have an inner wall reaching below the line) or a pillar. Hand
            # over to wasp_v5 rather than push on it -- an auditor found a quarter of seeds
            # with two to four searchers parked against the face for the whole run.
            wall_ahead = obs.pose.y > ANCHOR_Y and math.isfinite(blocked_ahead(obs, north, HANDOVER_M))
            if past_the_line or wall_ahead:
                self._crossed = True
            else:
                return toward(obs, np.array([obs.pose.x, self._leg_end_y + 1.0]))
        command = super().step(obs)
        if isinstance(command, Land):
            self._drop_crumb(obs, force=True)
            landed_on = self.mission.landed_on
            if landed_on is not None and landed_on.kind == "bonus_victim":
                self.publish("head", (float(obs.pose.x), float(obs.pose.y)))
        return command


# ------------------------------------------------------------------------------------------
# The relay node: approach, follow, band, land
# ------------------------------------------------------------------------------------------


class RelayNode(Policy):
    """One link of the chain. See the module docstring for the controller."""

    def __init__(self, agent_id, config, rng, arena, *, rank: int, relay_ids: tuple[str, ...],
                 searcher_ids: tuple[str, ...], lead_id: str, mode: str) -> None:
        super().__init__(agent_id, config, rng, arena)
        self.rank = rank                    # 0 joins first and ends nearest the head
        self.relay_ids = relay_ids          # in rank order
        self.searcher_ids = searcher_ids
        self.lead_id = lead_id
        self.mode = mode
        self.state = GROUND
        self.crumbs: dict[str, list[tuple[float, float, float]]] = {s: [] for s in searcher_ids}
        self.crumb_seen: dict[str, int] = {s: -1 for s in searcher_ids}
        self.heads: dict[str, tuple[float, float]] = {}
        self.head_trails: dict[str, tuple[Trail, int] | None] = {}   # dispatch: routed once per head
        self.lead_builder: TrailBuilder | None = None    # train: grows with the lead
        self.head_id: str | None = None
        self.trail: Trail | None = None
        self.anchor_index: int | None = None             # the anchor my chain's tail certifies against
        self.n_needed: int | None = None
        self.s = 0.0
        self.settled = 0
        self.in_place = False
        self.start_xy: np.ndarray | None = None
        self.anchors_xy: np.ndarray | None = None
        self.last_sweep_tick = -1
        self.entered_tick: int | None = None      # when I reached the trail: my chain seniority

    # -- what the blackboard says ------------------------------------------------------------

    def _ingest(self, obs: Observation) -> None:
        for sid in self.searcher_ids:
            data = obs.peers.get(sid)
            if not data:
                continue
            crumb = data.get("crumb")
            if crumb is not None and int(crumb[0]) > self.crumb_seen[sid]:
                self.crumb_seen[sid] = int(crumb[0])
                self.crumbs[sid].append((float(crumb[1]), float(crumb[2]), float(crumb[3])))
            head = data.get("head")
            if head is not None and sid not in self.heads:
                self.heads[sid] = (float(head[0]), float(head[1]))

    def _relay_state(self, obs: Observation, rid: str) -> tuple[int, float, bool, int]:
        """``(state, s, in_place, entered_tick)`` of a relay -- mine from my own bookkeeping,
        another's from the snapshot; GROUND if it has said nothing yet."""
        if rid == self.agent_id:
            return self.state, self.s, self.in_place, -1 if self.entered_tick is None else self.entered_tick
        data = obs.peers.get(rid)
        if not data or "relay" not in data:
            return GROUND, 0.0, False, -1
        state, _rank, s, in_place, entered = data["relay"]
        return int(state), float(s), bool(in_place), int(entered)

    def _published_in_place(self, obs: Observation, rid: str) -> bool:
        """``in_place`` as the *snapshot* has it, for me as for anyone else."""
        data = obs.peers.get(rid)
        return bool(data and "relay" in data and data["relay"][3])

    def _launched(self, obs: Observation) -> list[str]:
        """Relays that have left the ground, in rank order."""
        return [rid for rid in self.relay_ids if self._relay_state(obs, rid)[0] != GROUND]

    def _chain(self, obs: Observation) -> list[str]:
        """Relays on the trail, head-most first: the order they reached the trail in."""
        on_trail = []
        for rid in self.relay_ids:
            state, _s, _p, entered = self._relay_state(obs, rid)
            if state >= FOLLOW:
                on_trail.append((entered, self.relay_ids.index(rid), rid))
        return [rid for _e, _r, rid in sorted(on_trail)]

    # -- choosing a head ------------------------------------------------------------------------

    def _choose_head(self) -> None:
        """Dispatch: the announced head whose route costs fewest relays and is feasible.

        The route is the shortest path through every searcher's crumbs (:class:`CrumbNetwork`),
        computed once per head when it is announced -- its own last crumb arrives in the same
        snapshot -- from the crumbs known at that moment.
        """
        best = None
        for hid in self.heads:
            if hid not in self.head_trails:
                self.head_trails[hid] = CrumbNetwork(self.anchors_xy).route(self.crumbs, hid)
            routed = self.head_trails[hid]
            if routed is None or len(routed[0]) < 2:
                continue
            trail, anchor_index = routed
            need = relays_needed(trail.length)
            if need > len(self.relay_ids):
                continue
            if best is None or trail.length < best[1].length:
                best = (hid, trail, anchor_index, need)
        if best is not None:
            self.head_id, self.trail, self.anchor_index, self.n_needed = best

    def _may_join(self, obs: Observation) -> bool:
        """Relays join in rank order, one at a time, while the trail has room for another."""
        if self.mode == "dispatch":
            if self.head_id is None:
                self._choose_head()
            if self.head_id is None:
                return False
            wanted = self.n_needed
        else:
            if self.trail is None or self.anchor_index is None or self.trail.length < D_STAR_M:
                return False
            wanted = min(len(self.relay_ids), relays_with_room(self.trail.length))
        if self.rank >= wanted or len(self._launched(obs)) != self.rank:
            return False
        if self.rank == 0:
            return True
        # The airspace over the grid is clear once the previous joiner has moved off its spot.
        previous = self.relay_ids[self.rank - 1]
        prev_xy = obs.peers.get(previous, {}).get("xy")
        start = self._start_of(previous)
        return prev_xy is not None and float(np.hypot(prev_xy[0] - start[0], prev_xy[1] - start[1])) > 1.0

    def _start_of(self, rid: str) -> tuple[float, float]:
        return tuple(self._starts.get(rid, (float("nan"), float("nan"))))

    # -- the chain's neighbours ------------------------------------------------------------------

    def _neighbours(self, chain: list[str]) -> tuple[str | None, str | None]:
        """``(pred, succ)``: the relay ids toward the anchor and toward the head, or None for
        the anchor / the head themselves."""
        i = chain.index(self.agent_id)
        pred = chain[i + 1] if i + 1 < len(chain) else None        # reached the trail after me
        succ = chain[i - 1] if i >= 1 else None                      # reached the trail before me
        return pred, succ

    def _neighbour_range(self, obs: Observation, reading: UWBRanges, rid: str | None,
                         toward_head: bool) -> float:
        """The horizontal range to a chain neighbour from this sweep's reading, or inf."""
        if rid is None and not toward_head:
            a = self.anchor_index if self.anchor_index is not None else 0
            r = float(reading.ranges_m[a])
            anchor_z = float(reading.anchor_xyz_m[a, 2])
            return horizontal(r, anchor_z - obs.pose.z) if np.isfinite(r) else math.inf
        if rid is None:
            rid = self.head_id if self.head_id is not None else self.lead_id
            landed = rid in self.heads
            dz = obs.pose.z if landed else 0.0
        else:
            state = self._relay_state(obs, rid)[0]
            dz = obs.pose.z if state == DOWN else 0.0
        j = reading.peer_ids.index(rid)
        r = float(reading.peer_ranges_m[j])
        return horizontal(r, dz) if np.isfinite(r) else math.inf

    # -- the step --------------------------------------------------------------------------------

    def step(self, obs: Observation) -> Command:
        reading = require_peer_ranging(obs)
        if self.anchors_xy is None:
            self.anchors_xy = np.array(reading.anchor_xyz_m[:, :2])
            self.start_xy = obs.pose.xy.copy()
            self._starts: dict[str, tuple[float, float]] = {}
        self._ingest(obs)
        for rid in self.relay_ids:
            start = obs.peers.get(rid, {}).get("start")
            if start is not None:
                self._starts[rid] = (float(start[0]), float(start[1]))
        if self.mode == "train":
            if self.lead_builder is None:
                self.lead_builder = TrailBuilder(self.anchors_xy)
            self.trail = self.lead_builder.update(self.crumbs[self.lead_id])
            self.anchor_index = self.lead_builder.anchor_index
            self.head_id = self.lead_id if self.lead_id in self.heads else None
            self.n_needed = len(self.relay_ids)
        self.publish("start", (float(self.start_xy[0]), float(self.start_xy[1])))
        self.publish("xy", (float(obs.pose.x), float(obs.pose.y)))

        command: Command
        if self.state == GROUND:
            command = self._on_ground(obs)
        elif self.state == APPROACH:
            command = self._approach(obs)
        elif self.state == FOLLOW:
            command = self._follow(obs, reading)
        elif self.state == BAND:
            command = self._band(obs, reading)
        else:
            command = Velocity()
        self.publish("relay", (self.state, self.rank, float(self.s), bool(self.in_place),
                               -1 if self.entered_tick is None else self.entered_tick))
        return command

    def _on_ground(self, obs: Observation) -> Command:
        if self._may_join(obs):
            self.state = APPROACH
            return Velocity(vz=0.4)
        return Velocity()

    @property
    def anchor_xy(self) -> np.ndarray:
        return self.anchors_xy[self.anchor_index if self.anchor_index is not None else 0]

    def _approach(self, obs: Observation) -> Command:
        """North along my own column to the anchor row, then along it to the trail's start.

        The trail is entered anywhere on its first :data:`TRAIL_ENTRY_M`, at the arclength
        of the nearest point, not at the anchor itself. A relay already on the trail hovers
        about 0.9 m from the anchor, in exactly the cone the next relay's approach reads as
        blocked; aiming at the anchor point deadlocked the second relay for minutes on three
        sweep runs (the skeptic found 130-420 s of hovering written up as flight time).
        """
        if obs.pose.z < CRUISE_ALT_M - 0.05:
            return Velocity(vz=0.4)
        row_y = float(self.anchor_xy[1])
        if abs(obs.pose.y - row_y) > 0.3 and abs(obs.pose.x - self.anchor_xy[0]) > 0.5:
            return toward(obs, np.array([obs.pose.x, row_y]))
        if self.trail is not None and len(self.trail) >= 2:
            s_entry, gap = self.trail.project(obs.pose.xy, TRAIL_ENTRY_M)
        else:
            s_entry, gap = 0.0, float(np.linalg.norm(obs.pose.xy - self.anchor_xy))
        if gap > 0.3 and float(np.linalg.norm(obs.pose.xy - self.anchor_xy)) > 0.3:
            return toward(obs, self.anchor_xy)
        self.state = FOLLOW
        self.entered_tick = obs.tick
        self.s = s_entry
        return Velocity()

    def _target_s(self, obs: Observation) -> float:
        chain = self._chain(obs)
        n_active = len(chain)
        m = n_active - chain.index(self.agent_id)                   # 1 nearest the anchor
        return m * self.trail.length / (n_active + 1)

    def _track(self, obs: Observation, s_goal: float, advance: bool) -> Command:
        """Move the carrot ``s`` toward ``s_goal`` while the drone keeps up, and track it."""
        here = self.trail.point_at(self.s)
        if advance and float(np.linalg.norm(obs.pose.xy - here)) < 0.35:
            step = CRUISE_SPEED_MS * 0.05
            self.s += float(np.clip(s_goal - self.s, -step, step))
        return toward(obs, self.trail.point_at(self.s), avoid_range_m=CHAIN_AVOID_RANGE_M)

    def _follow(self, obs: Observation, reading: UWBRanges) -> Command:
        if self.trail is None or len(self.trail) < 2:
            return toward(obs, self.anchor_xy, avoid_range_m=CHAIN_AVOID_RANGE_M)
        s_goal = self._target_s(obs)
        command = self._track(obs, s_goal, advance=True)
        arrived = abs(s_goal - self.s) < 0.05 \
            and float(np.linalg.norm(obs.pose.xy - self.trail.point_at(self.s))) < 0.25
        if arrived:
            self.state = BAND
            self.settled = 0
        return command

    def _band(self, obs: Observation, reading: UWBRanges) -> Command:
        chain = self._chain(obs)
        pred, succ = self._neighbours(chain)
        fresh = obs.stale_ticks.get("uwb", 1) == 0 and obs.tick != self.last_sweep_tick
        head_down = (self.head_id is not None) and (self.head_id in self.heads)
        if fresh:
            self.last_sweep_tick = obs.tick
            r_pred = self._neighbour_range(obs, reading, pred, toward_head=False)
            r_succ = self._neighbour_range(obs, reading, succ, toward_head=True)
            ds = 0.0
            if math.isfinite(r_pred):
                ds -= BAND_GAIN * (r_pred - D_STAR_M)
            if math.isfinite(r_succ):
                ds += BAND_GAIN * (r_succ - D_STAR_M)
            ds = float(np.clip(ds, -MAX_BAND_STEP_M, MAX_BAND_STEP_M))
            on_carrot = float(np.linalg.norm(obs.pose.xy - self.trail.point_at(self.s))) < ON_CARROT_M
            if on_carrot and abs(ds) >= BAND_DEADBAND_M:
                self.s = float(np.clip(self.s + ds, 0.0, self.trail.length))
            gate = (
                head_down
                and math.isfinite(r_pred) and math.isfinite(r_succ)
                and r_pred <= MAX_LINK_M and r_succ <= MAX_LINK_M
            )
            self.settled = self.settled + 1 if gate else 0
            self.in_place = self.settled >= SETTLE_SWEEPS
        # Everyone lands together: the chain is as long as the trail needs, nobody launched is
        # still on the way, and every chain member is in place -- me included as I *published*
        # it, so every relay evaluates the same predicate on the same snapshot (R-POL-8).
        if head_down:
            needed = min(len(self.relay_ids), relays_needed(self.trail.length))
            complete = len(chain) >= needed and len(chain) == len(self._launched(obs))
            everyone = all(self._published_in_place(obs, rid) for rid in chain)
            if complete and everyone:
                self.state = DOWN
                return Land()
        # In train mode the head keeps moving and the trail keeps growing; re-anchor the
        # carrot to the equal-spacing target so the band trims around it.
        if self.mode == "train" and not head_down:
            return self._track(obs, self._target_s(obs), advance=True)
        return self._track(obs, self.s, advance=False)


# ------------------------------------------------------------------------------------------
# One registered policy that picks a role per drone
# ------------------------------------------------------------------------------------------


def relay_roles(fleet: tuple[str, ...], n_relay: int, width_m: float) -> tuple[str, ...]:
    """Which drones are relays, in launch order.

    The take-off grid fills rows from the south, ``per_row`` per row (the runner's own
    arithmetic: field width less two wall margins, over the grid spacing). A relay waits on
    the ground while the searchers fly north, so a grounded relay must never sit in a row
    *north* of a searcher: relays are taken from the southern row first, highest column
    first, and only then from the row above. With twenty-five drones the first version took
    "the last eight ids", which is the northern row -- and ten searchers behind a wall of
    parked relays found one target in ten minutes where ten alone find three.

    Launch order is northern-row relays first (they clear out of the southern ones' way), and
    within a row the relay nearest the anchor first, so each later joiner flies north on a
    column east of every earlier one and then west behind them, and no two approach paths
    cross. (Launching the farthest first made them cross.)
    """
    from safmc_sim.constants import START_SPACING_M, START_WALL_MARGIN_M

    per_row = max(1, int((width_m - 2.0 * START_WALL_MARGIN_M) // START_SPACING_M))
    candidates = list(range(1, len(fleet)))                         # drone_00 is the lead
    by_row_south_first = sorted(candidates, key=lambda i: (i // per_row, -(i % per_row)))
    chosen = by_row_south_first[:n_relay]
    launch_order = sorted(chosen, key=lambda i: (-(i // per_row), i % per_row))
    return tuple(fleet[i] for i in launch_order)


@register_policy("uwb_relay")
class UWBRelayTrial(Policy):
    """Per-drone role dispatch: lead, searcher or relay, decided from the fleet roster.

    ``policy_config``: ``n_relay`` (default 4), ``mode`` (``"dispatch"`` or ``"train"``), and
    anything wasp_v5 accepts for the searchers.
    """

    def __init__(self, agent_id, config, rng, arena) -> None:
        super().__init__(agent_id, config, rng, arena)
        self.n_relay = int(self.config.get("n_relay", 4))
        self.mode = str(self.config.get("mode", "dispatch"))
        if self.mode not in ("dispatch", "train"):
            raise PolicyError(f"mode must be 'dispatch' or 'train', got {self.mode!r}")
        if self.n_relay < 0:
            raise PolicyError(f"n_relay must be >= 0, got {self.n_relay}")
        self.inner: Policy | None = None

    def _assign(self, obs: Observation) -> Policy:
        reading = require_peer_ranging(obs)
        fleet = tuple(reading.peer_ids)
        if self.n_relay >= len(fleet):
            raise PolicyError(f"n_relay={self.n_relay} leaves no searcher in a fleet of {len(fleet)}")
        lead = fleet[0]
        relays = relay_roles(fleet, self.n_relay, self.arena.width_m)
        searchers = tuple(a for a in fleet if a not in relays)
        cfg = {k: v for k, v in self.config.items() if k not in ("n_relay", "mode")}
        if self.agent_id in relays:
            return RelayNode(self.agent_id, cfg, self.rng, self.arena, rank=relays.index(self.agent_id),
                             relay_ids=relays, searcher_ids=searchers, lead_id=lead, mode=self.mode)
        return RelaySearcher(self.agent_id, cfg, self.rng, self.arena, bonus_only=(self.agent_id == lead))

    def step(self, obs: Observation) -> Command:
        if self.inner is None:
            self.inner = self._assign(obs)
        return self.inner.step(obs)

    def drain_outbox(self) -> dict[str, Any]:
        out = super().drain_outbox()
        if self.inner is not None:
            out.update(self.inner.drain_outbox())
        return out


# ------------------------------------------------------------------------------------------
# The run
# ------------------------------------------------------------------------------------------


def make_config(seed: int = 0, n_drones: int = 10, n_relay: int = 4, mode: str = "dispatch",
                duration_s: float = 600.0, collision_behaviour: str = "stop",
                n_anchors: int = 1) -> RunConfig:
    """The trial: flown suite plus the tag with peers, an anchor row, roles from the config.

    The sweep rate is the fleet's under A-19: 5 Hz at ten drones and one anchor, 1 Hz at
    twenty-five; with ten anchors 3.33 Hz and 0.8 Hz. Not every combination divides the
    20 Hz tick, and the runner refuses the ones that do not rather than rounding.
    """
    # Survey the generated arena, then place: a point landmark does not change the layout, so
    # the runner's regeneration from the same seed and config draws the same arena.
    from safmc_sim.world.arena import generate_arena
    row = anchor_row(n_anchors, generate_arena(seed))
    return RunConfig(
        seed=seed,
        n_drones=n_drones,
        policy="uwb_relay",
        policy_config={"n_relay": n_relay, "mode": mode},
        duration_s=duration_s,
        collision_behaviour=collision_behaviour,
        arena_config=ArenaConfig(landmarks=row),
        sensors=flown_sensors() + (
            UWBConfig(peers=True, anchor_height_m=ANCHOR_HEIGHT_M,
                      rate_hz=peer_sweep_rate_hz(n_drones, len(row))),
        ),
    )


# ------------------------------------------------------------------------------------------
# Grading from the log alone: T-2 and T-3 of ADR-0007
# ------------------------------------------------------------------------------------------


def grade(directory: str | Path) -> dict[str, Any]:
    """Check the recorded chain by the mission's rule and the gate by the recorded ranges."""
    log = load_run(directory)
    header, footer, states = log["header"], log["footer"], log["states"]
    agents = header["agents"]
    arena = arena_from_log(header)
    # The chain graded is the one that existed the moment a relay first formed -- the one the
    # relays built -- not the footer's, which is the shortest of all chains at the end of the
    # run and can start from a different bonus rescuer that landed beside the tail later.
    # (relay_timeline directly, not compute_metrics: the coverage raycast is most of a
    # minute on a 25-drone log and none of it is needed here.)
    first = next((m for m in relay_timeline(log) if m.chain), None)
    chain = list(first.chain) if first is not None else []
    report: dict[str, Any] = {
        "relay_formed": bool(footer["score"]["relay_formed"]),
        "time_to_relay_s": None if first is None else first.sim_time_s,
        "chain": chain,
        "final_chain": list(footer["score"].get("relay_chain", [])),
        "score": footer["score"]["total"],
        "raw": footer["score"]["raw_total"],
        "crashed": sum(1 for v in footer["lifecycles"].values() if v == "CRASHED"),
        "waves": len(takeoff_waves([e["sim_time_s"] for e in log["events"] if e["kind"] == "departed"])),
        "links_m": [],
        "links_clear": [],
        "tail_in_start_area": None,
        "gate": [],
    }
    if not chain:
        return report

    at_formation = states["pose"][first.tick]
    idx = {a: i for i, a in enumerate(agents)}
    xy = np.array([at_formation[idx[a], :2] for a in chain])
    # T-2: every adjacent pair at most 1.0 m apart with floor-level line of sight, by the
    # same segment test the mission uses; the tail inside the Start Area.
    report["links_m"] = [float(v) for v in np.linalg.norm(np.diff(xy, axis=0), axis=1)]
    report["links_clear"] = [bool(v) for v in segment_clear(arena.structural_scene(), xy[:-1], xy[1:], 0.0)]
    report["tail_in_start_area"] = bool(arena.in_start_area(float(xy[-1, 0]), float(xy[-1, 1])))

    # T-3: for every relay down at formation, the last fresh sweep before it landed measured at
    # least two links inside the gate -- to the head, to other landed drones, or to an anchor
    # -- and at least one relay's anchor link was inside it: the tail's certificate. Graded
    # over every landed relay rather than the scored chain, because the mission's BFS returns
    # the *shortest* chain and can skip the anchor-certified tail when another relay happens
    # to have landed a centimetre inside the Start Area.
    uwb = log["sensors"].get("uwb")
    if uwb is None or "peer_ranges_m" not in uwb:
        report["gate"] = ["no uwb.npz with peer ranges recorded"]
        return report
    landed_code = int([c for c, n in header["codebook"]["lifecycle"].items() if n == "LANDED"][0])
    lifecycle = states["lifecycle"]
    down = [agents[i] for i in np.flatnonzero(lifecycle[first.tick] == landed_code)]
    rescuers = {a for v in footer["mission_summary"].values() for a in v["serviced_by"]}
    relays = [a for a in down if a not in rescuers]
    report["tail_certified"] = False
    for agent in relays:
        i = idx[agent]
        t_land = int(np.flatnonzero(lifecycle[:, i] == landed_code)[0])
        rows = np.flatnonzero((uwb["sample_tick"][:, i] <= t_land - 1) & (uwb["sample_tick"][:, i] >= 0))
        if not len(rows):
            report["gate"].append({"agent": agent, "ok": False, "why": "no sweep before landing"})
            continue
        r = int(rows[-1])
        t_sweep = int(uwb["sample_tick"][r, i])
        z_me = float(states["pose"][t_sweep, i, 2])
        links: dict[str, float] = {}
        for other in down:
            if other == agent:
                continue
            k = idx[other]
            rr = float(uwb["peer_ranges_m"][r, i, k])
            links[other] = horizontal(rr, z_me - float(states["pose"][t_sweep, k, 2])) if np.isfinite(rr) else math.inf
        for a_idx in range(uwb["anchor_xyz_m"].shape[0]):
            a = float(uwb["ranges_m"][r, i, a_idx])
            anchor_z = float(uwb["anchor_xyz_m"][a_idx, 2])
            links[f"anchor_{a_idx}"] = horizontal(a, anchor_z - z_me) if np.isfinite(a) else math.inf
        inside = {name: d for name, d in links.items() if d <= MAX_LINK_M}
        if any(name.startswith("anchor_") for name in inside):
            report["tail_certified"] = True
        report["gate"].append({"agent": agent, "t_land": t_land, "t_sweep": t_sweep,
                               "inside": {k: round(v, 2) for k, v in inside.items()},
                               "ok": len(inside) >= 2})
    return report


def print_report(report: Mapping[str, Any]) -> None:
    print(f"relay formed: {report['relay_formed']}   score {report['score']} (raw {report['raw']})   "
          f"crashed {report['crashed']}   take-off waves {report['waves']}")
    if report["time_to_relay_s"] is not None:
        print(f"first relay at t = {report['time_to_relay_s']:.1f} s")
    if report["chain"]:
        print("chain (head first): " + " -> ".join(report["chain"]))
        links = ", ".join(f"{d:.2f}{'' if ok else '!'}" for d, ok in zip(report["links_m"], report["links_clear"]))
        print(f"links (m, ! = no line of sight): {links}   tail in Start Area: {report['tail_in_start_area']}")
        print(f"tail certified by an anchor range: {report.get('tail_certified')}")
        for g in report["gate"]:
            if isinstance(g, dict) and "inside" in g:
                print(f"  {g['agent']}: last sweep t={g['t_sweep']} links inside the gate {g['inside']}"
                      f" -> {'ok' if g['ok'] else 'FEWER THAN TWO'}")
            else:
                print(f"  {g}")


# ------------------------------------------------------------------------------------------
# The sweep: dispatch vs train vs no relay, over seeds
# ------------------------------------------------------------------------------------------


def run_job(job: tuple[str, int, int, int, float, int]) -> dict[str, Any]:
    mode, n_relay, n_drones, seed, duration, n_anchors = job
    directory = Path(f"runs/relay_{mode}_r{n_relay}_n{n_drones}_a{n_anchors}_s{seed}")
    result = run(make_config(seed, n_drones, n_relay, mode, duration, n_anchors=n_anchors),
                 recorder=Recorder(directory, overwrite=True, sensor_every=4))
    report = grade(directory)
    report.update(mode=mode, n_relay=n_relay, n_drones=n_drones, seed=seed, n_anchors=n_anchors,
                  targets=sum(1 for v in result.mission_summary.values() if v["serviced"]))
    return report


def summarise_sweep(reports: list[dict[str, Any]]) -> str:
    cells: dict[tuple[str, int, int, int], list[dict[str, Any]]] = {}
    for r in reports:
        cells.setdefault((r["mode"], r["n_anchors"], r["n_relay"], r["n_drones"]), []).append(r)
    lines = [f"{'mode':<10}{'anchors':>8}{'relays':>7}{'drones':>7}{'n':>4}{'P(relay)':>10}{'score':>8}{'+/-':>6}"
             f"{'t_relay':>9}{'targets':>9}{'crashed':>9}{'waves':>7}"]
    lines.append("-" * len(lines[0]))
    for (mode, n_anchors, n_relay, n_drones), rs in sorted(cells.items()):
        scores = np.array([r["score"] for r in rs], float)
        formed = [r for r in rs if r["relay_formed"]]
        t_relay = np.mean([r["time_to_relay_s"] for r in formed]) if formed else float("nan")
        lines.append(
            f"{mode:<10}{n_anchors:>8}{n_relay:>7}{n_drones:>7}{len(rs):>4}{len(formed) / len(rs):>10.2f}"
            f"{scores.mean():>8.1f}{scores.std():>6.1f}{t_relay:>9.0f}"
            f"{np.mean([r['targets'] for r in rs]):>9.1f}{np.mean([r['crashed'] for r in rs]):>9.1f}"
            f"{np.mean([r['waves'] for r in rs]):>7.1f}"
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--n-drones", type=int, default=10)
    parser.add_argument("--n-relay", type=int, default=4)
    parser.add_argument("--mode", choices=("dispatch", "train"), default="dispatch")
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--sweep", action="store_true", help="dispatch vs train vs none over seeds")
    parser.add_argument("--seeds", type=int, default=5)
    parser.add_argument("--relays", type=str, default="0,3,5,8", help="n_relay values for the sweep")
    parser.add_argument("--anchors", type=str, default="1", help="anchor-row sizes; one is the brief")
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()
    anchors = [int(v) for v in args.anchors.split(",")]

    if not args.sweep:
        directory = Path(f"runs/relay_{args.mode}_r{args.n_relay}_n{args.n_drones}_a{anchors[0]}_s{args.seed}")
        result = run(make_config(args.seed, args.n_drones, args.n_relay, args.mode, args.duration,
                                 n_anchors=anchors[0]),
                     recorder=Recorder(directory, overwrite=True, sensor_every=4))
        print(result.score.explain())
        print_report(grade(directory))
        print(f"log: {directory}")
        return

    relays = [int(v) for v in args.relays.split(",")]
    jobs = []
    for n_relay, seed, n_anchors in product(relays, range(args.seeds), anchors):
        if n_relay == 0 and n_anchors != anchors[0]:
            continue                       # the no-relay baseline does not depend on anchors
        modes = ("none",) if n_relay == 0 else ("dispatch", "train")
        for mode in modes:
            jobs.append((mode if mode != "none" else "dispatch", n_relay, args.n_drones, seed,
                         args.duration, n_anchors))
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        reports = list(pool.map(run_job, jobs))
    for r in reports:
        if r["n_relay"] == 0:
            r["mode"] = "none"
    print(summarise_sweep(reports))
    print("\nConditions: ground-truth pose for trail following, perfect blackboard for crumbs and the "
          "landing consensus, A-14..A-19 unmeasured (F-36).")


if __name__ == "__main__":
    main()
