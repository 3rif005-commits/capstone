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

    # ---- Tank: "the heavy, the prize" — a single crane-down reveal.
    tank_look = tank + np.array([0.0, 0.0, 1.5])
    shots.append(Shot(
        'A1_tank_crane', 6.0,
        cm.crane_down(start_eye=tank + np.array([0, -4, 45]),
                      end_eye=tank + np.array([0, -14, 6]),
                      target=tank_look),
        entities=lambda u: {**_park(KAM, INT)},
    ))

    # ---- Kamikaze: "the threat" — fast Dutch-tilt arc, then a buzz-by.
    def kam_hover_fn(u: float) -> Dict[str, Pose6]:
        # gentle idle bob + slow yaw so it reads as 'alive' while we orbit it.
        p = kam_h + np.array([0.0, 0.0, 0.3 * math.sin(2 * math.pi * u)])
        return {KAM: _pose(p, yaw=math.radians(40 * u)), **_park(INT)}

    # Tight, slow detail orbit — close enough to read the body, arms and rotors.
    # NOTE: orbit `height` is an OFFSET above the centre, so keep it small (~1 m)
    # to sit right beside the drone; a big value puts the camera way overhead.
    shots.append(Shot(
        'A1_kam_arc', 5.0,
        cm.orbit(center=kam_h, radius=2.4, height=0.8,
                 start_deg=20, sweep_deg=160, look_height=0.0),
        entities=kam_hover_fn,
    ))

    # Buzz-by: kamikaze whips close past a fixed, Dutch-tilted lens.
    kbuzz = _line(kam_h + np.array([-9, -6, 1.5]), kam_h + np.array([7, 8, -0.5]))
    shots.append(Shot(
        'A1_kam_buzz', 4.0,
        cm.track_subject(eye=kam_h + np.array([1.5, -6, 0.5]),
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
    # Kamikaze sits at centre; the interceptor screams down and slams it. A big
    # flash "starts" the game. kam_sit doubles as the rebirth point (continuity).
    kam_sit = kam_h
    idive = _line(np.array([-35, -22, 42]), kam_sit + np.array([0, 0, 1]))
    # One 'money' viewpoint shared by the kill AND the rebirth, so Act 3 flows
    # straight out of the explosion with no jarring cut. Kept ~30 m back so the
    # fireball reads as a blast, not a wall of orange.
    impact_eye = kam_sit + np.array([20, -20, 10])

    shots.append(Shot(
        'A2_dive', 3.5,
        cm.hold(eye=impact_eye, target=kam_sit),
        entities=lambda u: {
            KAM: _pose(kam_sit, yaw=math.radians(25 * u)),
            INT: _pose(idive(u), yaw=_heading(idive, u),
                       pitch=math.radians(40), roll=math.radians(12)),
        },
    ))

    # Impact: hide both drones, bloom a big bright fireball at the hit point.
    def blast_centre(u: float) -> Dict[str, Pose6]:
        fx, fy, fz = kam_sit
        return {
            **_park(KAM, INT),
            FIRE:  (float(fx), float(fy), float(fz), 0.0, 0.0, 0.0),
            SMOKE: (float(fx), float(fy), float(fz + 1.5), 0.0, 0.0, 0.0),
        }

    shots.append(Shot(
        'A2_impact', 2.2,
        cm.hold(eye=impact_eye, target=kam_sit),
        entities=blast_centre,
    ))

    # ===================== ACT 3 — REBIRTH + THE REAL DUEL ========================
    # Continuous from the blast (SAME camera): the fireball drops away, smoke
    # drifts up, and the kamikaze is "reborn" at the centre, ready to run.
    patrol = np.array([52.0, -34.0, 30.0])     # interceptor loiter point

    def rebirth(u: float) -> Dict[str, Pose6]:
        fx, fy, fz = kam_sit
        out: Dict[str, Pose6] = {}
        out.update(_park(FIRE))                # fireball gone (a jump -> hidden fast)
        out[SMOKE] = (float(fx), float(fy), float(fz + 2.0 + 4.0 * u), 0, 0, 0)  # drifts up
        rise = fz - 2.0 + 2.0 * cm.smoothstep(min(1.0, u * 1.5))            # fade in
        out[KAM] = _pose(np.array([fx, fy, rise]), yaw=math.radians(20 * u),
                         pitch=math.radians(-4))
        out[INT] = _pose(patrol, yaw=math.radians(120))                    # loiter
        return out

    shots.append(Shot(
        'A3_rebirth', 3.5,
        cm.hold(eye=impact_eye, target=kam_sit),
        entities=rebirth,
    ))

    # The run: kamikaze marches centre -> tank; interceptor dives to intercept but
    # MISSES (passes above & beside), then peels away. One global progress g drives
    # both so they stay in sync across the three shots.
    tank_dive = tank + np.array([-1.0, 0.0, 2.5])    # where the kamikaze hits

    def kpos(g):
        return cm.lerp(kam_sit, tank_dive, g)

    def ipos(g):
        # dive toward the kamikaze's mid-course, then climb away past it
        if g <= 0.6:
            return cm.lerp(patrol, np.array([28, 5, 9]), cm.smoothstep(g / 0.6))
        return cm.lerp(np.array([28, 5, 9]), np.array([-25, 35, 26]),
                       cm.smoothstep((g - 0.6) / 0.4))

    def yaw_of(fn, g):
        a = fn(max(0.0, g - 1e-3))
        b = fn(min(1.0, g + 1e-3))
        d = b - a
        return math.atan2(d[1], d[0]) if (abs(d[0]) + abs(d[1])) > 1e-9 else 0.0

    def hide_blast():
        return _park(FIRE, SMOKE)

    # A3_run: g 0.00 -> 0.45  (chase the kamikaze; interceptor enters from afar)
    shots.append(Shot(
        'A3_run', 6.0,
        cm.chase(subject_path=lambda u: kpos(0.45 * u),
                 offset=np.array([-10, -6, 4]), look_ahead=8.0),
        entities=lambda u: {
            **hide_blast(),
            KAM: _pose(kpos(0.45 * u), yaw=yaw_of(kpos, 0.45 * u),
                       pitch=math.radians(-12)),
            INT: _pose(ipos(0.45 * u), yaw=yaw_of(ipos, 0.45 * u),
                       roll=math.radians(-18)),
        },
    ))

    # A3_miss: g 0.45 -> 0.72  (side view of the near-miss: interceptor knifes
    # past the kamikaze and climbs away — it does NOT connect)
    def g_miss(u):
        return 0.45 + 0.27 * u

    shots.append(Shot(
        'A3_miss', 5.0,
        # camera from the open tank side (east) looking back west — clear of the
        # tall tower/buildings that crowd the centre.
        cm.track_subject(eye=np.array([48, -14, 16]),
                         subject_path=lambda u: kpos(g_miss(u))),
        entities=lambda u: {
            KAM: _pose(kpos(g_miss(u)), yaw=yaw_of(kpos, g_miss(u)),
                       pitch=math.radians(-14)),
            INT: _pose(ipos(g_miss(u)), yaw=yaw_of(ipos, g_miss(u)),
                       pitch=math.radians(15 * math.sin(math.pi * u)),
                       roll=math.radians(25)),
        },
    ))

    # A3_tank_hit: the kamikaze completes its dive and destroys the tank; the
    # interceptor is gone, climbing away. The explosion fires at u=0.5 and is
    # HELD on the tank for the rest of the shot (a clear money beat).
    def tank_kill(u: float) -> Dict[str, Pose6]:
        out: Dict[str, Pose6] = {}
        if u < 0.5:
            g = 0.72 + 0.28 * (u / 0.5)        # reach the tank by u = 0.5
            out[KAM] = _pose(kpos(g), yaw=yaw_of(kpos, g), pitch=math.radians(-24))
            out[INT] = _pose(ipos(min(1.0, g)), yaw=yaw_of(ipos, min(1.0, g)),
                             roll=math.radians(-15))
            out.update(_park(FIRE, SMOKE))
        else:
            tx, ty, tz = tank
            out.update(_park(KAM, INT))
            out[FIRE] = (float(tx), float(ty), float(tz + 1.5), 0, 0, 0)
            out[SMOKE] = (float(tx), float(ty), float(tz + 3.5), 0, 0, 0)
        return out

    shots.append(Shot(
        'A3_tank_hit', 5.5,
        # stay well back (~32 m) so the 7 m fireball reads as a ball engulfing
        # the tank with the city around it, not a lens-filling wall of orange.
        cm.dolly(start_eye=tank + np.array([24, -20, 13]),
                 end_eye=tank + np.array([27, -22, 15]),
                 target=tank + np.array([0, 0, 2])),
        entities=tank_kill,
    ))

    return shots
