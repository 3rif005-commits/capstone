"""Cinematic camera-move primitives — pure math, Gazebo-independent.

A camera move is a function of normalised time ``u in [0, 1]`` that returns a
:class:`Shot` sample: where the camera *eye* is, what world point it *looks at*,
and an optional *roll* (Dutch tilt) about the optical axis. The director turns
each sample into a Gazebo pose via :func:`look_at_quat`.

Gazebo camera convention: a camera sensor looks down its local **+X** axis with
**+Z** up (identity orientation faces +X). All quaternions here are world-frame
``(x, y, z, w)`` to match ``geometry_msgs/Pose`` / ``gz.msgs.Pose``.

These primitives compose: a "shot" in the scenario is just one of these closures
(or a custom lambda) sampled over its duration. Keep this module free of ROS and
Gazebo imports so it stays unit-testable.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Sequence, Tuple

import numpy as np

Vec3 = Sequence[float]


@dataclass
class CamSample:
    """One instant of a camera move."""
    eye: np.ndarray          # camera position, world frame
    look_at: np.ndarray      # world point the camera aims at
    roll: float = 0.0        # Dutch-tilt angle about the optical axis (rad)


# A camera move is u in [0,1] -> CamSample.
CameraMove = Callable[[float], CamSample]


# ── Easing ────────────────────────────────────────────────────────────────────
def smoothstep(u: float) -> float:
    """Ease in/out (C1). 0 at u=0, 1 at u=1, zero slope at both ends."""
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


def smootherstep(u: float) -> float:
    """Ease in/out (C2) — gentler starts/stops than :func:`smoothstep`."""
    u = min(1.0, max(0.0, u))
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


def lerp(a: Vec3, b: Vec3, u: float) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return a + (b - a) * u


# ── Orientation: look-at -> quaternion ──────────────────────────────────────────
def look_at_quat(eye: Vec3, target: Vec3, roll: float = 0.0,
                 world_up: Vec3 = (0.0, 0.0, 1.0)) -> Tuple[float, float, float, float]:
    """World-frame quaternion (x, y, z, w) orienting a Gazebo camera at ``eye``
    so its optical axis (+X) points toward ``target``, +Z roughly up.

    ``roll`` tilts the camera about its optical axis (Dutch angle), positive =
    clockwise as seen by the camera. Degenerate cases (looking straight up/down)
    fall back to an alternate up vector.
    """
    eye = np.asarray(eye, dtype=float)
    target = np.asarray(target, dtype=float)
    up = np.asarray(world_up, dtype=float)

    fwd = target - eye
    n = np.linalg.norm(fwd)
    if n < 1e-9:
        fwd = np.array([1.0, 0.0, 0.0])      # nowhere to look; face +X
    else:
        fwd = fwd / n

    # If forward is (anti)parallel to up, pick a different up to avoid a zero cross.
    if abs(float(np.dot(fwd, up))) > 0.999:
        up = np.array([0.0, 1.0, 0.0])

    left = np.cross(up, fwd)                  # camera local +Y points left
    left /= np.linalg.norm(left)
    cam_up = np.cross(fwd, left)              # camera local +Z

    if roll:
        c, s = math.cos(roll), math.sin(roll)
        left, cam_up = c * left + s * cam_up, -s * left + c * cam_up

    # Columns are the camera's local axes expressed in world frame.
    R = np.column_stack((fwd, left, cam_up))
    return _mat_to_quat(R)


def _mat_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Rotation matrix -> quaternion (x, y, z, w). Shepperd's method."""
    m00, m11, m22 = R[0, 0], R[1, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


# ── Move primitives ─────────────────────────────────────────────────────────────
def orbit(center: Vec3, radius: float, height: float,
          start_deg: float = 0.0, sweep_deg: float = 360.0,
          look_height: float | None = None,
          ease: Callable[[float], float] = smoothstep) -> CameraMove:
    """Circle the camera around ``center`` at fixed ``radius`` and ``height``,
    always looking at the center (optionally raised by ``look_height``)."""
    center = np.asarray(center, dtype=float)
    look = center.copy()
    if look_height is not None:
        look = look + np.array([0.0, 0.0, look_height])

    def move(u: float) -> CamSample:
        ang = math.radians(start_deg + sweep_deg * ease(u))
        eye = center + np.array([radius * math.cos(ang),
                                 radius * math.sin(ang), height])
        return CamSample(eye=eye, look_at=look)
    return move


def crane_down(start_eye: Vec3, end_eye: Vec3, target: Vec3,
               ease: Callable[[float], float] = smootherstep) -> CameraMove:
    """Descend (or rise) the camera from ``start_eye`` to ``end_eye`` while
    holding the same ``target`` — a crane/jib reveal."""
    target = np.asarray(target, dtype=float)

    def move(u: float) -> CamSample:
        return CamSample(eye=lerp(start_eye, end_eye, ease(u)), look_at=target)
    return move


def dolly(start_eye: Vec3, end_eye: Vec3, target: Vec3,
          ease: Callable[[float], float] = smoothstep) -> CameraMove:
    """Translate the camera in a straight line (push-in / pull-out / track)
    while looking at ``target``."""
    target = np.asarray(target, dtype=float)

    def move(u: float) -> CamSample:
        return CamSample(eye=lerp(start_eye, end_eye, ease(u)), look_at=target)
    return move


def track_subject(eye: Vec3, subject_path: Callable[[float], Vec3],
                  roll: float = 0.0) -> CameraMove:
    """Hold the camera fixed at ``eye`` and pan to follow a moving subject. As
    the subject whips past, the pan accelerates — a fly-by / whip-pan. Add
    ``roll`` for a Dutch tilt. ``subject_path(u)`` returns the subject position."""
    eye = np.asarray(eye, dtype=float)

    def move(u: float) -> CamSample:
        return CamSample(eye=eye, look_at=np.asarray(subject_path(u), dtype=float),
                         roll=roll)
    return move


def chase(subject_path: Callable[[float], Vec3], offset: Vec3,
          look_ahead: float = 0.0, roll: float = 0.0) -> CameraMove:
    """Fly alongside/behind a subject: eye = subject + ``offset`` (world frame),
    looking at the subject (optionally ``look_ahead`` metres past it along its
    motion). Good for the interceptor tracking shot."""
    offset = np.asarray(offset, dtype=float)

    def move(u: float) -> CamSample:
        p = np.asarray(subject_path(u), dtype=float)
        look = p
        if look_ahead:
            eps = 1e-3
            p2 = np.asarray(subject_path(min(1.0, u + eps)), dtype=float)
            d = p2 - p
            nd = np.linalg.norm(d)
            if nd > 1e-9:
                look = p + d / nd * look_ahead
        return CamSample(eye=p + offset, look_at=look, roll=roll)
    return move


def hold(eye: Vec3, target: Vec3, roll: float = 0.0) -> CameraMove:
    """A static locked-off shot."""
    eye = np.asarray(eye, dtype=float)
    target = np.asarray(target, dtype=float)

    def move(_u: float) -> CamSample:
        return CamSample(eye=eye, look_at=target, roll=roll)
    return move
