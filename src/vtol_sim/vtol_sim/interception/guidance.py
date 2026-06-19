"""Guidance laws for fixed-wing interception (classical, no RL).

All laws share one interface:

    accel_cmd = law.command(p_i, v_i, p_t, v_t, a_t)

where (p_i, v_i) are interceptor position/velocity, (p_t, v_t, a_t) are the
target's, and the return is a desired acceleration vector in the world frame
(m/s^2) that the platform (FixedWing.step) turns into bank + climb commands.

Three laws, in increasing capability — directly serving the PDF's
"compare model-based strategies" goal:

  * PurePursuit          : steer velocity straight at the target. Simple,
                           but lags a crossing/maneuvering target (tail chase).
  * ProportionalNav (PN) : null the line-of-sight (LOS) rotation rate. The
                           classic missile-guidance law; near-optimal against a
                           non-accelerating target, minimal control effort.
  * AugmentedPN (APN)    : PN plus a term proportional to the *target's*
                           lateral acceleration. This is the "we can do better"
                           core: a human jinking the kamikaze drone produces
                           exactly the lateral acceleration APN compensates for.

The geometry helper computes everything Layer-1 state estimation provides:
relative position, relative velocity, LOS unit vector, LOS rotation-rate
vector, and closing velocity.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Engagement:
    """Relative engagement geometry between interceptor and target."""
    r: float                 # range (m)
    los: np.ndarray          # unit line-of-sight, interceptor → target
    v_rel: np.ndarray        # relative velocity, v_t - v_i
    closing_speed: float     # +ve when range is decreasing (Vc)
    omega: np.ndarray        # LOS rotation-rate vector (rad/s)

    @classmethod
    def compute(cls, p_i, v_i, p_t, v_t) -> "Engagement":
        p_i, v_i, p_t, v_t = (np.asarray(a, float) for a in (p_i, v_i, p_t, v_t))
        R = p_t - p_i
        r = float(np.linalg.norm(R))
        r_safe = max(r, 1e-6)
        los = R / r_safe
        v_rel = v_t - v_i
        closing_speed = -float(np.dot(R, v_rel)) / r_safe
        # LOS angular velocity: omega = (R x V_rel) / |R|^2
        omega = np.cross(R, v_rel) / (r_safe * r_safe)
        return cls(r=r, los=los, v_rel=v_rel, closing_speed=closing_speed, omega=omega)


# Lateral acceleration (m/s^2) requested for a hard re-acquisition turn. Sized
# well above the platform's turn authority so FixedWing.step saturates to a
# maximum-rate turn toward the target. Used by PN/APN as a pursuit fallback.
REACQUIRE_ACCEL = 50.0


def _pursuit_dir(los: np.ndarray, v_i: np.ndarray):
    """Unit vector ⟂ to current velocity that steers the heading toward `los`.

    Returns (dir, speed). `dir` is the component of the LOS perpendicular to the
    velocity, normalised — point the nose this way to turn toward the target.
    """
    v_i = np.asarray(v_i, float)
    speed = float(np.linalg.norm(v_i))
    if speed < 1e-6:
        return np.zeros(3), 0.0
    v_hat = v_i / speed
    los_perp = los - np.dot(los, v_hat) * v_hat
    n = np.linalg.norm(los_perp)
    return (los_perp / n if n > 1e-6 else np.zeros(3)), speed


class GuidanceLaw:
    name = "base"

    def command(self, p_i, v_i, p_t, v_t, a_t=None) -> np.ndarray:
        raise NotImplementedError


class PurePursuit(GuidanceLaw):
    """Maximum-rate turn toward the current target position (tail chase).

    The platform is rate-limited, so commanding a large lateral acceleration
    toward the LOS makes the nose track the target as fast as the bank limit
    allows. Simple and always re-acquires, but lags a crossing target.
    """
    name = "pure_pursuit"

    def __init__(self, accel: float = REACQUIRE_ACCEL):
        self.accel = accel

    def command(self, p_i, v_i, p_t, v_t, a_t=None) -> np.ndarray:
        eng = Engagement.compute(p_i, v_i, p_t, v_t)
        pdir, speed = _pursuit_dir(eng.los, v_i)
        if speed < 1e-6:
            return np.zeros(3)
        return self.accel * pdir


class ProportionalNav(GuidanceLaw):
    """True PN: a = N * Vc * (omega x los), nulling the LOS rotation rate.

    Near-optimal once on a closing, roughly boresight engagement. A pursuit
    fallback re-acquires whenever the geometry opens (Vc <= 0) or the target is
    far off-boresight, so the interceptor never flies off ballistically.
    """
    name = "pn"

    def __init__(self, N: float = 4.0):
        self.N = N

    def _pn_term(self, eng: Engagement) -> np.ndarray:
        return self.N * eng.closing_speed * np.cross(eng.omega, eng.los)

    def command(self, p_i, v_i, p_t, v_t, a_t=None) -> np.ndarray:
        eng = Engagement.compute(p_i, v_i, p_t, v_t)
        pdir, speed = _pursuit_dir(eng.los, v_i)
        if speed < 1e-6:
            return np.zeros(3)
        v_hat = np.asarray(v_i, float) / speed
        boresight = math.acos(max(-1.0, min(1.0, float(np.dot(eng.los, v_hat)))))
        # Closing and roughly pointed at the target → trust PN.
        if eng.closing_speed > 0.0 and boresight < math.radians(60.0):
            return self._pn_term(eng)
        # Otherwise hard-turn back onto the target (acquisition / re-engagement).
        return REACQUIRE_ACCEL * pdir


class AugmentedPN(ProportionalNav):
    """APN: PN plus (N/2) * target lateral acceleration (⟂ to LOS).

    The extra term feeds the target's maneuver forward into the command, so the
    interceptor leads a jinking target instead of chasing its LOS history. Same
    pursuit fallback as PN for robust acquisition.
    """
    name = "apn"

    def command(self, p_i, v_i, p_t, v_t, a_t=None) -> np.ndarray:
        eng = Engagement.compute(p_i, v_i, p_t, v_t)
        pdir, speed = _pursuit_dir(eng.los, v_i)
        if speed < 1e-6:
            return np.zeros(3)
        v_hat = np.asarray(v_i, float) / speed
        boresight = math.acos(max(-1.0, min(1.0, float(np.dot(eng.los, v_hat)))))
        if not (eng.closing_speed > 0.0 and boresight < math.radians(60.0)):
            return REACQUIRE_ACCEL * pdir
        cmd = self._pn_term(eng)
        if a_t is not None:
            a_t = np.asarray(a_t, float)
            a_t_perp = a_t - np.dot(a_t, eng.los) * eng.los  # ⟂ to LOS
            cmd = cmd + 0.5 * self.N * a_t_perp
        return cmd


LAWS = {
    PurePursuit.name: PurePursuit,
    ProportionalNav.name: ProportionalNav,
    AugmentedPN.name: AugmentedPN,
}


def make_law(name: str, **kwargs) -> GuidanceLaw:
    """Factory: make_law('apn', N=4.0)."""
    key = name.lower()
    if key not in LAWS:
        raise ValueError(f"unknown guidance law {name!r}; choose from {list(LAWS)}")
    return LAWS[key](**kwargs)
