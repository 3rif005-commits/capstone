"""3D point-mass fixed-wing kinematic model (coordinated-turn).

This is the "kinematic model of a fixed-wing interceptor" (PDF Objective 1).
It deliberately models *kinematics*, not aerodynamics: enough to capture what
makes a fixed-wing different from the X3 multirotor —

  * it cannot hover or fly backwards: airspeed stays within [V_min, V_max];
  * it turns by banking: heading rate is limited by a maximum bank angle
    (coordinated turn, psi_dot = g*tan(phi) / (V*cos(gamma)));
  * its climb/dive rate is limited (flight-path angle gamma is bounded).

Coordinate convention matches the Gazebo world used by the game:
  +X = North, +Y = West, +Z = up.
Heading psi is measured in the horizontal plane from +X toward +Y.
Velocity = V * [cos(gamma)cos(psi), cos(gamma)sin(psi), sin(gamma)].

The guidance layer produces a desired lateral acceleration *vector* (world
frame, nominally perpendicular to velocity). `step()` converts that vector
into the achievable bank + flight-path-rate commands, applies the fixed-wing
limits, and integrates one timestep. Keeping the platform interface as an
acceleration vector lets any guidance law drive any platform unchanged.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.81  # m/s^2


@dataclass
class FixedWingLimits:
    v_min: float = 12.0          # m/s  — stall floor; cannot fly slower
    v_max: float = 28.0          # m/s  — top airspeed
    accel_max: float = 8.0       # m/s^2 along-track (throttle authority)
    # 70° bank → turn radius V^2/(g*tan70) ≈ 28 m at V=28: tight enough to close
    # on an agile target, while still a fixed-wing (cannot pivot like a quad).
    bank_max: float = np.radians(70.0)   # rad — max roll → max turn rate
    gamma_max: float = np.radians(35.0)  # rad — max climb/dive angle
    gamma_rate_max: float = np.radians(45.0)  # rad/s — pitch authority


@dataclass
class FixedWing:
    """Integrates the kinematic state given a desired lateral-accel command."""

    pos: np.ndarray                       # world position [x, y, z] (m)
    psi: float = 0.0                      # heading (rad), 0 = North
    gamma: float = 0.0                    # flight-path angle (rad)
    speed: float = 22.0                   # airspeed V (m/s)
    limits: FixedWingLimits = field(default_factory=FixedWingLimits)

    def __post_init__(self):
        self.pos = np.asarray(self.pos, dtype=float)
        self.speed = float(np.clip(self.speed, self.limits.v_min, self.limits.v_max))

    # ── Derived quantities ────────────────────────────────────────────────
    @property
    def velocity(self) -> np.ndarray:
        cg = np.cos(self.gamma)
        return self.speed * np.array([
            cg * np.cos(self.psi),
            cg * np.sin(self.psi),
            np.sin(self.gamma),
        ])

    @property
    def vel_unit(self) -> np.ndarray:
        v = self.velocity
        n = np.linalg.norm(v)
        return v / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])

    # ── One integration step ──────────────────────────────────────────────
    def step(self, accel_cmd: np.ndarray, dt: float, speed_cmd: float | None = None):
        """Advance the state by `dt` seconds.

        accel_cmd : desired acceleration vector (world frame, m/s^2). Only the
                    component perpendicular to the current velocity is used to
                    steer (turn + climb); the along-track component is ignored
                    here because airspeed is governed by `speed_cmd`.
        speed_cmd : target airspeed (m/s). Defaults to V_max (interceptors run
                    fast); clamped to [v_min, v_max] and rate-limited by accel_max.
        """
        lim = self.limits
        v_hat = self.vel_unit

        # Build a velocity frame: h_hat = horizontal "right", u_hat = "up".
        z_world = np.array([0.0, 0.0, 1.0])
        h_hat = np.cross(v_hat, z_world)         # horizontal, ⟂ to track
        nh = np.linalg.norm(h_hat)
        if nh < 1e-6:                            # velocity ~vertical: pick any horizontal
            h_hat = np.array([1.0, 0.0, 0.0])
        else:
            h_hat = h_hat / nh
        u_hat = np.cross(h_hat, v_hat)           # completes right-handed frame

        # Perpendicular acceleration components in the velocity frame.
        a_perp = accel_cmd - np.dot(accel_cmd, v_hat) * v_hat
        a_h = float(np.dot(a_perp, h_hat))       # turn (horizontal) channel
        a_v = float(np.dot(a_perp, u_hat))       # climb (vertical) channel

        # ── Horizontal: lateral accel → bank → heading rate ───────────────
        # With h_hat = v_hat x z = [sinψ, -cosψ, 0], the turning acceleration
        # Vh·ψ̇·[-sinψ, cosψ] projects onto h_hat as -Vh·ψ̇, so a positive a_h
        # turns the heading the *negative* way: ψ̇ = -a_h / (V·cosγ).
        a_h_max = G * np.tan(lim.bank_max)
        a_h = float(np.clip(a_h, -a_h_max, a_h_max))
        bank = np.arctan2(a_h, G)                # coordinated-turn bank angle
        cg = max(np.cos(self.gamma), 1e-3)
        psi_dot = -a_h / (self.speed * cg)

        # ── Vertical: lateral accel → flight-path-angle rate ──────────────
        gamma_dot = a_v / max(self.speed, 1e-3)
        gamma_dot = float(np.clip(gamma_dot, -lim.gamma_rate_max, lim.gamma_rate_max))

        # ── Airspeed toward command, throttle-limited ─────────────────────
        v_target = lim.v_max if speed_cmd is None else speed_cmd
        v_target = float(np.clip(v_target, lim.v_min, lim.v_max))
        dv = float(np.clip(v_target - self.speed, -lim.accel_max * dt, lim.accel_max * dt))

        # ── Integrate (forward Euler; dt is small, 4–20 ms) ───────────────
        self.psi = (self.psi + psi_dot * dt) % (2.0 * np.pi)
        self.gamma = float(np.clip(self.gamma + gamma_dot * dt,
                                   -lim.gamma_max, lim.gamma_max))
        self.speed = float(np.clip(self.speed + dv, lim.v_min, lim.v_max))
        self.pos = self.pos + self.velocity * dt

        # Realized (post-limit) lateral acceleration — the honest control-effort
        # figure, since a_perp above may have been clamped by bank/pitch limits.
        a_lat_realized = math.hypot(G * np.tan(bank), gamma_dot * self.speed)
        return {
            'bank': bank,
            'psi_dot': psi_dot,
            'gamma_dot': gamma_dot,
            'a_lat': float(a_lat_realized),
        }
