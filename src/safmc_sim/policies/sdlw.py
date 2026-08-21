"""Sensor-Driven Levy Walk -- a faithful port of arXiv:2607.25195 (IROS 2026).

*Decentralized Scalable Exploration via Emergent Adaptive Levy Walks on Minimal-Sensing
Platforms*, Wai Lun Leong and Rodney Teo, National University of Singapore. Released code:
https://github.com/williamleong/sdlw (MIT).

Ported here for three reasons. It is the paper the team wants to test. It is an NUS result
built on **ir-sim itself**, so reproducing it validates our sensor and kinematics semantics
against a published baseline. And it is a genuinely strong mapless strategy to compare
map-based search against.

The algorithm, per agent, with no map, no global position and no communication:

* Draw a Levy exponent ``alpha ~ U(2, 3)`` **once** at initialisation.
* Step length: ``Pr[d = 0] = 1/2`` exactly; otherwise ``Pr[d = j] proportional to
  j ** -alpha`` over integer ``j`` in ``[ceil(d_min), floor(d_max)]``. ``d = 0`` means a
  rotate-only cycle.
* Heading: weight each of four ranges by ``beta_s * r_s ** 2`` -- range **squared** -- with the
  backward sensor discounted. The weighted resultant gives a mean direction ``mu`` and an
  "openness" ``o``; concentration ``kappa(o)`` decreases linearly in ``o``; sample
  ``theta ~ VonMises(mu, kappa)``.
* Two states, ROTATE and MOVE, with bang-bang yaw and small axial and lateral nudges when a
  range drops below a threshold.

Set ``variant: "uhlw"`` for the paper's baseline: identical, but the heading is drawn
uniformly. That is the comparison the paper's headline numbers are against.

Three things recon found in the released implementation that a reader should know
---------------------------------------------------------------------------------

**"Openness" does not measure open space.** Because the weights are ``r ** 2`` and the left
and right terms cancel, an isotropically open agent at (4, 4, 4, 4) gives ``o = 0.212``, hence
``kappa = 8.01`` -- nearly *maximal* concentration. ``o -> 1`` needs everything blocked except
one direction. Measured over 300 s runs the effective band is ``kappa`` in [1.4, 10] with
median around 7, and ``kappa_min = 0.6`` is never reached. The released README describes this
accurately as "directional consensus rather than simply large average clearance"; the paper's
section II-C prose describes the opposite. In practice SDLW is a stochastic potential-field
steer, and the superdiffusion claim rests on the step-length distribution.

**Collided agents are permanently dead** under ``collision_mode: stop``, and the paper's
coverage gain is entangled with its collision reduction: fewer deaths means more live
agent-seconds. Recon measured half a four-drone team dead 35 s into a 900 s episode. Compare
under both collision modes and normalise coverage by live-agent-seconds.

**The reference harness integrates yaw outside the simulator**, without wrapping the angle or
refreshing collision geometry. Ours integrates yaw inside the kinematics handler, which is
correct and slightly different. Expect small divergence from the published trajectories.

The paper assumes 4 m noiseless single-ray sensors. Our ring gates at the firmware's 3 m by
default, so a faithful reproduction needs ``ToFConfig(max_valid_m=4.0)``.
"""

from __future__ import annotations

import numpy as np

from ..api import Command, Observation, VelocityBody, register_policy
from ..frames import wrap_pi
from .base import SearchPolicy, ring_quadrants

__all__ = ["SDLW"]

_ROTATE, _MOVE = "ROTATE", "MOVE"


@register_policy("sdlw")
class SDLW(SearchPolicy):
    """Sensor-driven Levy walk. ``variant="uhlw"`` gives the paper's uniform-heading baseline."""

    def __init__(self, agent_id, config, rng, arena):
        super().__init__(agent_id, config, rng, arena)
        c = self.config
        self.variant = str(c.get("variant", "sdlw")).lower()
        if self.variant not in ("sdlw", "uhlw"):
            raise ValueError(f"variant must be 'sdlw' or 'uhlw', got {self.variant!r}")

        self.max_vel = float(c.get("max_vel", 0.5))
        self.max_yaw = float(c.get("max_yaw", 0.5))
        self.alpha_min = float(c.get("alpha_min", 2.0))
        self.alpha_max = float(c.get("alpha_max", 3.0))
        self.d_min = float(c.get("min_flight_distance", 0.5))
        self.d_max = float(c.get("max_flight_distance", 20.0))
        self.kappa_min = float(c.get("kappa_min", 0.6))
        self.kappa_max = float(c.get("kappa_max", 10.0))
        self.back_weight = float(c.get("back_sensor_weight", 0.3))
        self.heading_tol = float(c.get("heading_tolerance_rad", 0.087))  # 5 degrees
        self.collision_threshold = float(c.get("collision_threshold", 0.4))
        self.nudge = float(c.get("nudge_ms", 0.2))
        self.range_max = float(c.get("range_max", 4.0))

        # alpha is drawn once per agent and never resampled -- that is what makes the team's
        # aggregate behaviour a mixture of exponents rather than a single walk.
        self.alpha = float(self.rng.uniform(self.alpha_min, self.alpha_max))
        self._support = np.arange(
            int(np.ceil(self.d_min)), int(np.floor(self.d_max)) + 1, dtype=float
        )
        if not len(self._support):
            raise ValueError(
                f"empty Levy support: ceil({self.d_min}) > floor({self.d_max})"
            )
        weights = self._support ** (-self.alpha)
        self._probabilities = weights / weights.sum()

        self.state = _ROTATE
        self.target_heading = 0.0
        self.target_distance = 0.0
        self.travelled = 0.0
        self._last_xy: np.ndarray | None = None

    def reset(self) -> None:
        super().reset()
        self.state = _ROTATE
        self.target_heading = 0.0
        self.target_distance = 0.0
        self.travelled = 0.0
        self._last_xy = None

    # -- the two samplers ---------------------------------------------------------------------

    def sample_distance(self) -> float:
        """Discrete Levy step. Half of all cycles are rotate-only, exactly as in the paper."""
        if self.rng.random() < 0.5:
            return 0.0
        return float(self.rng.choice(self._support, p=self._probabilities))

    def sample_heading(self, obs: Observation, psi: float) -> float:
        if self.variant == "uhlw":
            return float(self.rng.uniform(-np.pi, np.pi))

        quadrants = ring_quadrants(obs)
        # The controller assumes a 4 m sensor that saturates rather than reporting "nothing".
        ranges = np.array(
            [
                min(quadrants["front"], self.range_max),
                min(quadrants["left"], self.range_max),
                min(quadrants["right"], self.range_max),
                min(quadrants["back"], self.range_max),
            ]
        )
        ranges = np.where(np.isfinite(ranges), ranges, self.range_max)
        bearings = psi + np.array([0.0, np.pi / 2, -np.pi / 2, np.pi])
        weights = np.array([1.0, 1.0, 1.0, self.back_weight]) * ranges**2

        vector = np.array(
            [np.sum(weights * np.cos(bearings)), np.sum(weights * np.sin(bearings))]
        )
        total = weights.sum()
        if total <= 1e-6:
            # Not in the paper; present in the released code. Fully enclosed: turn around.
            return wrap_pi(psi + np.pi)

        mu = float(np.arctan2(vector[1], vector[0]))
        openness = float(np.linalg.norm(vector) / (total + 1e-9))
        kappa = self.kappa_max - (self.kappa_max - self.kappa_min) * np.clip(openness, 0.0, 1.0)
        return float(wrap_pi(self.rng.vonmises(mu, max(kappa, 1e-6))))

    # -- the controller -------------------------------------------------------------------------

    def choose_motion(self, obs: Observation) -> Command:
        psi = obs.pose.theta
        xy = obs.pose.xy
        if self._last_xy is not None:
            self.travelled += float(np.linalg.norm(xy - self._last_xy))
        self._last_xy = xy.copy()

        quadrants = ring_quadrants(obs)
        threshold = self.collision_threshold

        vx = vy = 0.0
        # Lateral nudges apply in both states, matching the released implementation.
        if quadrants["left"] < threshold:
            vy = -self.nudge
        elif quadrants["right"] < threshold:
            vy = self.nudge

        if self.state == _ROTATE:
            error = wrap_pi(self.target_heading - psi)
            if abs(error) <= self.heading_tol:
                self.target_distance = self.sample_distance()
                self.travelled = 0.0
                self.state = _MOVE
                return VelocityBody(vx=self.max_vel, vy=vy)
            if quadrants["front"] < threshold:
                vx = -self.nudge
            elif quadrants["back"] < threshold:
                vx = self.nudge
            return VelocityBody(
                vx=vx, vy=vy, yaw_rate=float(np.sign(error) * self.max_yaw)
            )

        if self.travelled >= self.target_distance or quadrants["front"] < threshold:
            self.target_heading = self.sample_heading(obs, psi)
            self.state = _ROTATE
            return VelocityBody(vx=0.0, vy=vy)

        return VelocityBody(vx=self.max_vel, vy=vy)
