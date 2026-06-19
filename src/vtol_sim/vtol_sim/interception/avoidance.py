"""Collision-aware steering for the fixed-wing interceptor.

The interceptor cannot stop or pivot, so obstacle avoidance is done by *steering*
(adding lateral acceleration), not by braking. For each building within a
look-ahead distance and roughly ahead of the flight path, we add an acceleration
perpendicular to the velocity that turns the nose around the obstacle, plus a
climb component to clear it vertically. The strength rises sharply as the gap
closes, so close-in avoidance overrides the guidance command.

This term is simply *added* to the guidance acceleration (PN/APN/pursuit):

    a_total = guidance.command(...) + field.avoid_accel(pos, vel)

so any guidance law becomes collision-aware without changing the law itself.
"""

from __future__ import annotations

import numpy as np

from .world import Obstacle


class ObstacleField:
    def __init__(self, obstacles: list[Obstacle], lookahead: float = 40.0,
                 safety: float = 5.0, vert_clear: float = 4.0, gain: float = 80.0):
        self.obstacles = obstacles
        self.lookahead = lookahead    # m — start reacting at this range to surface
        self.safety = safety          # m — extra keep-out beyond the footprint
        self.vert_clear = vert_clear  # m — treat as clear once this far above top
        self.gain = gain              # m/s^2 — avoidance authority (saturates turn)

    def avoid_accel(self, pos, vel) -> np.ndarray:
        pos = np.asarray(pos, float)
        vel = np.asarray(vel, float)
        v2 = vel[:2]
        sp2 = np.linalg.norm(v2)
        if sp2 < 1e-6:
            return np.zeros(3)
        vhat2 = v2 / sp2
        a = np.zeros(3)

        for o in self.obstacles:
            if pos[2] > o.height + self.vert_clear:
                continue                                   # flying safely over it
            d = pos[:2] - o.center                         # obstacle → us (outward)
            gap = float(np.linalg.norm(d)) - (o.radius + self.safety)
            outward = d / (np.linalg.norm(d) + 1e-9)

            if gap <= 0.0:                                 # inside keep-out: shove out
                a[:2] += self.gain * 3.0 * outward
                a[2] += self.gain * 0.5                    # and climb
                continue
            if gap > self.lookahead:
                continue
            ahead = -float(np.dot(outward, vhat2))         # >0 if obstacle ahead
            if ahead <= 0.0:
                continue                                   # already past / behind

            # Steer perpendicular to velocity, away from the obstacle.
            perp = outward - np.dot(outward, vhat2) * vhat2
            npn = np.linalg.norm(perp)
            if npn < 1e-3:                                 # dead head-on: break left
                perp = np.array([-vhat2[1], vhat2[0]])
            else:
                perp = perp / npn
            strength = (self.lookahead - gap) / self.lookahead   # 0..1, →1 when close
            a[:2] += self.gain * strength * ahead * perp
            a[2] += self.gain * 0.4 * strength * ahead     # climb to clear the top
        return a

    def in_collision(self, pos) -> bool:
        """True if `pos` is inside a building footprint and below its top."""
        pos = np.asarray(pos, float)
        for o in self.obstacles:
            if pos[2] <= o.height and np.linalg.norm(pos[:2] - o.center) <= o.radius:
                return True
        return False
