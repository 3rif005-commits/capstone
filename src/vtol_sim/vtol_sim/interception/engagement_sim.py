"""Headless engagement simulator — validates guidance with no ROS/Gazebo.

Mirrors the game: a kamikaze target dives from altitude toward a tank at the
origin (optionally weaving), while a fixed-wing interceptor starting at a random
sky point tries to reach it first. Runs every guidance law against every
scenario and prints a comparison table (success, intercept time, miss distance,
control effort) — the PDF's performance-evaluation metrics, computed offline.

Run:
    python3 -m vtol_sim.interception.engagement_sim
    ros2 run vtol_sim engagement_sim          # after colcon build
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .avoidance import ObstacleField
from .fixed_wing import FixedWing, FixedWingLimits
from .guidance import make_law
from .world import city

TANK = np.array([0.0, 0.0, 1.0])     # asset at origin
HIT_RADIUS = 6.0                      # m — interceptor catches kamikaze
TANK_RADIUS = 5.0                     # m — kamikaze reaches the tank
DT = 0.02                             # 50 Hz
TIMEOUT = 40.0                        # s

# Fixed-wing interception needs open space: a fast interceptor with a ~30 m turn
# radius cannot dogfight a slow agile target in a small box, so the kamikaze
# approaches the asset over a long inbound leg and is engaged mid-course.


# ── Kamikaze target ───────────────────────────────────────────────────────
@dataclass
class Kamikaze:
    """Constant-speed drone steering toward the tank, with optional weave."""
    pos: np.ndarray
    speed: float = 12.0
    turn_rate_max: float = math.radians(70.0)   # rad/s — agile multirotor
    weave_amp: float = 0.0                       # rad — heading weave amplitude
    weave_freq: float = 0.0                      # Hz
    psi: float = 0.0
    gamma: float = 0.0
    _vel: np.ndarray = field(default=None, repr=False)

    def __post_init__(self):
        self.pos = np.asarray(self.pos, float)
        to_tank = TANK - self.pos
        self.psi = math.atan2(to_tank[1], to_tank[0])
        self.gamma = math.atan2(to_tank[2], math.hypot(to_tank[0], to_tank[1]))
        self._vel = self._velocity()

    def _velocity(self):
        cg = math.cos(self.gamma)
        return self.speed * np.array([cg * math.cos(self.psi),
                                      cg * math.sin(self.psi),
                                      math.sin(self.gamma)])

    @property
    def velocity(self):
        return self._vel

    def step(self, dt: float, t: float):
        to_tank = TANK - self.pos
        des_psi = math.atan2(to_tank[1], to_tank[0])
        des_gamma = math.atan2(to_tank[2], math.hypot(to_tank[0], to_tank[1]))
        if self.weave_amp > 0.0:
            des_psi += self.weave_amp * math.sin(2 * math.pi * self.weave_freq * t)

        v_old = self._velocity()
        # Steer heading & flight-path toward desired, rate-limited.
        dpsi = math.atan2(math.sin(des_psi - self.psi), math.cos(des_psi - self.psi))
        self.psi += np.clip(dpsi, -self.turn_rate_max * dt, self.turn_rate_max * dt)
        dgam = des_gamma - self.gamma
        self.gamma += float(np.clip(dgam, -self.turn_rate_max * dt, self.turn_rate_max * dt))
        self._vel = self._velocity()
        self.pos = self.pos + self._vel * dt
        return (self._vel - v_old) / dt     # measured lateral acceleration


# ── Single run ──────────────────────────────────────────────────────────────
@dataclass
class Result:
    outcome: str          # 'INTERCEPT' | 'TANK_HIT' | 'CRASH' | 'TIMEOUT'
    time: float
    miss: float           # closest interceptor–kamikaze range achieved (m)
    effort: float         # mean |lateral accel| (m/s^2) — control effort proxy


def run(law_name: str, kam: Kamikaze, intc: FixedWing,
        field: ObstacleField | None = None, estimate_target_accel=True):
    """Fly one engagement. If `field` is given, the interceptor avoids buildings
    and a building strike ends the run as a CRASH."""
    law = make_law(law_name, **({} if law_name == "pure_pursuit" else {"N": 4.0}))
    t = 0.0
    min_range = float("inf")
    effort_sum, steps = 0.0, 0
    a_t_est = np.zeros(3)
    v_t_prev = kam.velocity.copy()

    while t < TIMEOUT:
        rng = float(np.linalg.norm(kam.pos - intc.pos))
        min_range = min(min_range, rng)
        if rng <= HIT_RADIUS:
            return Result("INTERCEPT", t, min_range, effort_sum / max(steps, 1))
        if float(np.linalg.norm(kam.pos - TANK)) <= TANK_RADIUS:
            return Result("TANK_HIT", t, min_range, effort_sum / max(steps, 1))
        if field is not None and field.in_collision(intc.pos):
            return Result("CRASH", t, min_range, effort_sum / max(steps, 1))

        a_cmd = law.command(intc.pos, intc.velocity, kam.pos, kam.velocity,
                            a_t_est if estimate_target_accel else None)
        if field is not None:
            a_cmd = a_cmd + field.avoid_accel(intc.pos, intc.velocity)
        info = intc.step(a_cmd, DT)
        effort_sum += info["a_lat"]
        steps += 1

        kam.step(DT, t)
        # First-order estimate of target accel (what the ROS node will do).
        a_t_est = 0.7 * a_t_est + 0.3 * (kam.velocity - v_t_prev) / DT
        v_t_prev = kam.velocity.copy()
        t += DT

    return Result("TIMEOUT", t, min_range, effort_sum / max(steps, 1))


# ── Scenarios (PDF experimental scenarios) ──────────────────────────────────
def scenarios():
    return {
        "1: straight-line": dict(speed=10.0, weave_amp=0.0, weave_freq=0.0),
        "2: fast target":   dict(speed=16.0, weave_amp=0.0, weave_freq=0.0),
        "3: heading change": dict(speed=12.0, weave_amp=math.radians(35), weave_freq=0.08),
        "4: hard maneuver": dict(speed=13.0, weave_amp=math.radians(55), weave_freq=0.25),
    }


def make_kamikaze(params, seed):
    rng = np.random.default_rng(seed)
    # Kamikaze enters from a random bearing on a long inbound leg: 90–150 m out,
    # 25–45 m altitude, diving toward the asset.
    bearing = rng.uniform(0, 2 * math.pi)
    dist = rng.uniform(90.0, 150.0)
    alt = rng.uniform(25.0, 45.0)
    pos = np.array([dist * math.cos(bearing), dist * math.sin(bearing), alt])
    return Kamikaze(pos=pos, **params)


def make_interceptor(seed):
    rng = np.random.default_rng(seed + 1000)
    # Interceptor on combat air patrol near the asset: 40–80 m out, 30–45 m alt.
    bearing = rng.uniform(0, 2 * math.pi)
    dist = rng.uniform(40.0, 80.0)
    alt = rng.uniform(30.0, 45.0)
    pos = np.array([dist * math.cos(bearing), dist * math.sin(bearing), alt])
    fw = FixedWing(pos=pos, speed=26.0, limits=FixedWingLimits())
    return fw


def aim_at(fw: FixedWing, target_pos: np.ndarray):
    """Point the interceptor's nose at a position (vectoring onto intercept)."""
    d = np.asarray(target_pos, float) - fw.pos
    fw.psi = math.atan2(d[1], d[0])
    fw.gamma = math.atan2(d[2], math.hypot(d[0], d[1]))
    return fw


def main():
    laws = ["pure_pursuit", "pn", "apn"]
    trials = 20
    field = ObstacleField(city())

    print("\nFixed-wing interception — offline guidance comparison")
    print(f"(per scenario: {trials} random engagements, HIT_RADIUS={HIT_RADIUS} m, "
          f"city obstacle field, avoidance ON)\n")
    header = (f"{'scenario':<18}{'law':<13}{'success':>8}{'crash':>7}"
              f"{'avg t(s)':>10}{'avg miss(m)':>13}{'effort':>9}")
    print(header)
    print("-" * len(header))

    for scen_name, params in scenarios().items():
        for law in laws:
            wins, crashes, times, misses, efforts = 0, 0, [], [], []
            for k in range(trials):
                kam = make_kamikaze(params, seed=k)
                intc = aim_at(make_interceptor(seed=k), kam.pos)
                res = run(law, kam, intc, field=field)
                if res.outcome == "INTERCEPT":
                    wins += 1
                    times.append(res.time)
                elif res.outcome == "CRASH":
                    crashes += 1
                misses.append(res.miss)
                efforts.append(res.effort)
            sr = 100.0 * wins / trials
            cr = 100.0 * crashes / trials
            at = f"{np.mean(times):.2f}" if times else "  -  "
            print(f"{scen_name:<18}{law:<13}{sr:>6.0f}%{cr:>6.0f}%{at:>10}"
                  f"{np.mean(misses):>13.2f}{np.mean(efforts):>9.2f}")
        print()


if __name__ == "__main__":
    main()
