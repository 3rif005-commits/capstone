"""The machinima scenario as data — the shot list for "Intercept".

See ``docs/machinima-scenario.md`` for the creative intent. Every shot is a
:class:`Shot`: a duration, a camera move (from :mod:`camera_moves`), and an
optional ``entities`` timeline that puppeteers the drones during that shot.
Cuts between shots are hard (trailer style).

This file is deliberately Gazebo-independent and data-heavy so it's the one
place to iterate on pacing/framing. ``build_shots(scene)`` returns the ordered
list the director plays. Tune freely — the geometry knobs are in :data:`SCENE`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from . import camera_moves as cm

# Pose tuple commanded to a puppet: (x, y, z, roll, pitch, yaw) world frame.
Pose6 = Tuple[float, float, float, float, float, float]
EntityFn = Callable[[float], Dict[str, Pose6]]


# ── Scene geometry (tune these) ─────────────────────────────────────────────────
SCENE = {
    'tank':       np.array([45.0, 0.0, 0.0]),    # the defended asset (N of centre)
    'centre':     np.array([0.0, 0.0, 0.0]),     # crossroads; kamikaze launch point
    'kam_hover':  np.array([0.0, 0.0, 8.0]),     # kamikaze briefing/launch altitude
    'park':       np.array([0.0, 0.0, -200.0]),  # underground = hidden off-camera
    'intercept':  np.array([26.0, 0.0, 8.0]),    # where the catch happens (Act 3)
}

KAM = 'kamikaze'
INT = 'interceptor'
TANK = 'target_tank'
FIRE = 'explosion_fireball'
SMOKE = 'explosion_smoke'


@dataclass
class Shot:
    name: str
    duration: float
    camera: cm.CameraMove
    entities: Optional[EntityFn] = None     # u in [0,1] -> {model: pose6}
    record: bool = True                     # include this shot in the take


# ── Small helpers ───────────────────────────────────────────────────────────────
def _park(*names: str) -> Dict[str, Pose6]:
    x, y, z = SCENE['park']
    return {n: (float(x), float(y), float(z), 0.0, 0.0, 0.0) for n in names}


def _heading(path: Callable[[float], np.ndarray], u: float) -> float:
    """Yaw (rad) from a path's local direction of travel."""
    eps = 1e-3
    a = path(max(0.0, u - eps))
    b = path(min(1.0, u + eps))
    d = b - a
    if np.hypot(d[0], d[1]) < 1e-9:
        return 0.0
    return math.atan2(d[1], d[0])


def _pose(p: np.ndarray, yaw: float = 0.0, pitch: float = 0.0,
          roll: float = 0.0) -> Pose6:
    return (float(p[0]), float(p[1]), float(p[2]), roll, pitch, yaw)


def _line(a, b) -> Callable[[float], np.ndarray]:
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    return lambda u: a + (b - a) * cm.smoothstep(u)


# ── Scenario ─────────────────────────────────────────────────────────────────────
def build_shots(scene: dict | None = None) -> List[Shot]:
    s = scene or SCENE
    tank = s['tank']
    centre = s['centre']
    kam_h = s['kam_hover']
    intercept = s['intercept']

    shots: List[Shot] = []

    # ============================ ACT 1 — BRIEFINGS ============================

    # ---- Tank: "the heavy, the prize" — crane-down reveal + low-angle push-in.
    tank_look = tank + np.array([0.0, 0.0, 1.5])
    shots.append(Shot(
        'A1_tank_crane', 6.0,
        cm.crane_down(start_eye=tank + np.array([0, -4, 45]),
                      end_eye=tank + np.array([0, -14, 6]),
                      target=tank_look),
        entities=lambda u: {**_park(KAM, INT)},
    ))
    shots.append(Shot(
        'A1_tank_pushin', 5.0,
        cm.dolly(start_eye=tank + np.array([-16, -10, 4]),
                 end_eye=tank + np.array([-8, -5, 2.5]),
                 target=tank_look),
    ))

    # ---- Kamikaze: "the threat" — fast Dutch-tilt arc, then a buzz-by.
    def kam_hover_fn(u: float) -> Dict[str, Pose6]:
        # gentle idle bob + slow yaw so it reads as 'alive' while we orbit it.
        p = kam_h + np.array([0.0, 0.0, 0.3 * math.sin(2 * math.pi * u)])
        return {KAM: _pose(p, yaw=math.radians(40 * u)), **_park(INT)}

    shots.append(Shot(
        'A1_kam_arc', 5.0,
        cm.orbit(center=kam_h, radius=7.0, height=9.0,
                 start_deg=20, sweep_deg=200, look_height=0.0),
        entities=kam_hover_fn,
    ))

    # Buzz-by: kamikaze whips diagonally past a fixed, Dutch-tilted lens.
    kbuzz = _line(kam_h + np.array([-14, -10, 2]), kam_h + np.array([10, 12, -1]))
    shots.append(Shot(
        'A1_kam_buzz', 4.0,
        cm.track_subject(eye=kam_h + np.array([2, -16, 1]),
                         subject_path=kbuzz, roll=math.radians(8)),
        entities=lambda u: {KAM: _pose(kbuzz(u), yaw=_heading(kbuzz, u),
                                       pitch=math.radians(-12)), **_park(INT)},
    ))

    # ---- Interceptor: "the hero" — banking fly-by + tracking chase.
    ifly = _line(np.array([-60, -25, 30]), np.array([40, 18, 26]))
    shots.append(Shot(
        'A1_int_flyby', 4.0,
        cm.track_subject(eye=np.array([-5, -30, 18]), subject_path=ifly),
        entities=lambda u: {INT: _pose(ifly(u), yaw=_heading(ifly, u),
                                       roll=math.radians(-18)), **_park(KAM)},
    ))
    ichase = _line(np.array([40, 18, 26]), np.array([10, -10, 16]))
    shots.append(Shot(
        'A1_int_chase', 5.0,
        cm.chase(subject_path=ichase, offset=np.array([-9, -7, 3]), look_ahead=6.0),
        entities=lambda u: {INT: _pose(ichase(u), yaw=_heading(ichase, u),
                                       roll=math.radians(25 * math.sin(math.pi * u)))},
    ))

    # ======================= ACT 2 — COLD-OPEN TRAILER KILL =======================
    # Kamikaze sits at centre; interceptor screams down and slams it -> explosion.
    kam_sit = kam_h
    idive = _line(np.array([-35, -20, 40]), kam_sit)

    shots.append(Shot(
        'A2_dive', 3.5,
        cm.hold(eye=kam_sit + np.array([18, -14, 6]), target=kam_sit),
        entities=lambda u: {
            KAM: _pose(kam_sit, yaw=math.radians(20 * u)),
            INT: _pose(idive(u), yaw=_heading(idive, u),
                       pitch=math.radians(35), roll=math.radians(10)),
        },
    ))
    # Impact: hide both drones, bloom the fireball + smoke at the hit point.
    def blast(u: float) -> Dict[str, Pose6]:
        fx, fy, fz = kam_sit
        rise = fz + 2.0 * u
        return {
            **_park(KAM, INT),
            FIRE:  (float(fx), float(fy), float(rise), 0.0, 0.0, 0.0),
            SMOKE: (float(fx), float(fy), float(rise + 1.0), 0.0, 0.0, 0.0),
        }

    shots.append(Shot(
        'A2_impact', 3.0,
        cm.hold(eye=kam_sit + np.array([18, -14, 6]), target=kam_sit),
        entities=blast,
    ))

    # =========================== ACT 3 — THE REAL DUEL ===========================
    # Respawn: kamikaze recovers at centre and runs for the tank; the interceptor
    # curves in from patrol and catches it mid-course. Hide the Act-2 blast first.
    def hide_blast() -> Dict[str, Pose6]:
        return _park(FIRE, SMOKE)

    # Kamikaze racing line: centre -> a point just short of the tank.
    krun = _line(kam_h, tank + np.array([-6, 0, 6]))
    # Interceptor pursuit: patrol point -> the intercept point, arriving with the
    # kamikaze (both reach ~the same place near u=1).
    ipur = _line(np.array([55, -35, 34]), intercept)

    shots.append(Shot(
        'A3_launch', 3.0,
        cm.chase(subject_path=krun, offset=np.array([-10, -6, 4]), look_ahead=8.0),
        entities=lambda u: {
            **hide_blast(),
            KAM: _pose(krun(u * 0.4), yaw=_heading(krun, u * 0.4),
                       pitch=math.radians(-14)),
            INT: _pose(ipur(u * 0.4), yaw=_heading(ipur, u * 0.4),
                       roll=math.radians(-20)),
        },
    ))
    # Side-on tracking of the converging duel.
    def duel(u: float) -> Dict[str, Pose6]:
        ku = 0.4 + 0.6 * u
        iu = 0.4 + 0.6 * u
        return {
            KAM: _pose(krun(ku), yaw=_heading(krun, ku), pitch=math.radians(-16)),
            INT: _pose(ipur(iu), yaw=_heading(ipur, iu),
                       roll=math.radians(30 * math.sin(math.pi * u))),
        }

    shots.append(Shot(
        'A3_duel', 7.0,
        cm.orbit(center=intercept, radius=22.0, height=12.0,
                 start_deg=150, sweep_deg=70, look_height=0.0),
        entities=duel,
    ))
    # The catch: interceptor reaches the kamikaze -> explosion, hold on the kill.
    def catch(u: float) -> Dict[str, Pose6]:
        out = {}
        if u < 0.5:
            uu = u / 0.5
            out[KAM] = _pose(krun(1.0), yaw=_heading(krun, 1.0))
            out[INT] = _pose(cm.lerp(ipur(1.0), krun(1.0), uu),
                             yaw=_heading(ipur, 1.0), pitch=math.radians(10))
        else:
            fx, fy, fz = krun(1.0)
            out.update(_park(KAM, INT))
            out[FIRE] = (float(fx), float(fy), float(fz), 0.0, 0.0, 0.0)
            out[SMOKE] = (float(fx), float(fy), float(fz + 1.0), 0.0, 0.0, 0.0)
        return out

    shots.append(Shot(
        'A3_catch', 5.0,
        cm.dolly(start_eye=krun(1.0) + np.array([16, -12, 7]),
                 end_eye=krun(1.0) + np.array([11, -8, 5]),
                 target=krun(1.0)),
        entities=catch,
    ))

    return shots
