#!/usr/bin/env python3
"""
Keyboard teleop for VTOL drone.

Uses raw terminal + key-repeat timestamps for simultaneous key support.
When a key is held, the OS repeats it at ~30 Hz. We mark each key's
last-seen time and treat it as "active" for 150 ms after the last event.
Multiple held keys are summed into one Twist command.
"""

import math
import os
import sys
import select
import termios
import tty
import time

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# v2 kamikaze: faster than v1 (3.0 m/s) for the chase, but kept under the
# Gazebo controller's cap (maximumLinearVelocity = 8 m/s in vtol_world.sdf) with
# headroom so commands don't constantly saturate. Yaw kept gentle — aggressive
# yaw disturbs the thrust mix and causes altitude dip / instability.
LINEAR_SPEED  = 7.0   # m/s
ANGULAR_SPEED = 0.9   # rad/s
PUBLISH_RATE     = 20    # Hz
KEY_HOLD_TIMEOUT = 0.10  # seconds a key stays active after last repeat
                         # (lower = crisper stop on release; must stay above the
                         # ~33 ms OS key-repeat interval to avoid stutter)

# Altitude hold. The Gazebo plugin is a *velocity* controller: it drives
# vertical velocity to zero, not altitude to a setpoint, so any altitude lost
# during a maneuver (yaw, accelerating, drag) is never recovered. We close that
# loop here: when the pilot isn't commanding climb/descend, hold the last
# target altitude with a simple P controller on the odometry Z.
ALT_HOLD_KP = 1.2     # vertical velocity (m/s) commanded per metre of error
ALT_HOLD_VZ_MAX = 2.0  # cap on the hold's corrective climb/descent rate (m/s)

# Horizontal behaviour (v2): release-to-stop. v1 held a *latched* X/Y position
# (loiter), which for the kamikaze attack run felt like the drone "returning to
# its start" on its own. Instead, when no translation is commanded we simply
# command zero horizontal velocity — the controller brakes to a stop in place,
# with no fly-back to any held point.

HELP = """
VTOL Drone Keyboard Teleop  (simultaneous keys supported)
=========================================================
  Z / S              : Throttle up / down
  Q / D              : Yaw left / right
  Arrow Up / Down    : Pitch forward / backward
  Arrow Left / Right : Strafe left / right
  Space              : Hover (stop all motion)
  T                  : Auto-takeoff to 5 m
  Ctrl+C or Esc      : Quit

NOTE: press Z (or T) first to take off before moving horizontally.
"""

TAKEOFF_ALTITUDE_TIME = 3.0   # seconds to hold Z during auto-takeoff

# Raw byte sequences → key name
SEQUENCES = {
    b'z': 'z',  b'Z': 'z',
    b's': 's',  b'S': 's',
    b'q': 'q',  b'Q': 'q',
    b'd': 'd',  b'D': 'd',
    b't': 'takeoff', b'T': 'takeoff',
    b'\x1b[A': 'up',
    b'\x1b[B': 'down',
    b'\x1b[C': 'right',
    b'\x1b[D': 'left',
    b' ': 'space',
}

# Key name → (linear.x, linear.y, linear.z, angular.z)
KEY_ACTIONS = {
    'z':     ( 0.0,  0.0,  1.0,  0.0),
    's':     ( 0.0,  0.0, -1.0,  0.0),
    'q':     ( 0.0,  0.0,  0.0,  1.0),
    'd':     ( 0.0,  0.0,  0.0, -1.0),
    'up':    ( 1.0,  0.0,  0.0,  0.0),
    'down':  (-1.0,  0.0,  0.0,  0.0),
    'left':  ( 0.0,  1.0,  0.0,  0.0),
    'right': ( 0.0, -1.0,  0.0,  0.0),
    'space': ( 0.0,  0.0,  0.0,  0.0),
}


def parse_keys(data: bytes) -> list:
    """Parse raw stdin bytes into a list of key name strings."""
    keys = []
    i = 0
    while i < len(data):
        ch = data[i:i+1]
        if ch == b'\x1b':
            if data[i+1:i+2] == b'[' and i + 2 < len(data):
                seq = data[i:i+3]
                key = SEQUENCES.get(seq)
                if key:
                    keys.append(key)
                i += 3
            else:
                keys.append('esc')
                i += 2
        elif ch == b'\x03':        # Ctrl+C
            keys.append('ctrl_c')
            i += 1
        else:
            key = SEQUENCES.get(ch)
            if key:
                keys.append(key)
            i += 1
    return keys


def quat_to_rotation_matrix(x, y, z, w):
    """Body->world rotation matrix from a quaternion (x, y, z, w)."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.cmd_pub = self.create_publisher(
            Twist, '/X3/gazebo/command/twist', 10)
        self.enable_pub = self.create_publisher(
            Bool, '/X3/enable', 10)
        self.create_timer(1.0, self._send_enable)

        # Current body->world rotation and altitude, from odometry. _R is
        # needed because the velocity controller interprets the command in the
        # body frame; _z_now / _z_target drive the altitude-hold loop.
        self._R = np.eye(3)
        self._z_now = None
        self._z_target = None
        # Scoped by model name "x3" (lowercase), not the "X3" robotNamespace.
        self.create_subscription(
            Odometry, '/model/x3/odometry', self._on_odom, 10)

        self._last_seen = {}   # key_name -> monotonic timestamp
        self._running = True
        self._takeoff_until = 0.0   # monotonic time until auto-takeoff ends
        self._fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._fd)

    def _send_enable(self):
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self._R = quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
        p = msg.pose.pose.position
        self._z_now = p.z

    def _read_keys(self):
        """Non-blocking drain of all available stdin bytes."""
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if not rlist:
            return []
        try:
            data = os.read(self._fd, 64)
        except OSError:
            return []
        return parse_keys(data)

    def _compute_twist(self):
        now = time.monotonic()
        lx = ly = lz = az = 0.0

        # Auto-takeoff overrides everything until its timer expires
        if now < self._takeoff_until:
            lz = 1.0
        else:
            for key, ts in self._last_seen.items():
                if now - ts < KEY_HOLD_TIMEOUT and key in KEY_ACTIONS:
                    dx, dy, dz, da = KEY_ACTIONS[key]
                    lx += dx; ly += dy; lz += dz; az += da

        # Normalize the horizontal (forward, left) input so a diagonal isn't
        # faster than a straight move, then scale everything to real units.
        horiz = math.hypot(lx, ly)
        if horiz > 1.0:
            lx /= horiz
            ly /= horiz

        clamp = lambda v: max(-1.0, min(1.0, v))
        fwd  = clamp(lx) * LINEAR_SPEED
        left = clamp(ly) * LINEAR_SPEED

        # Vertical: if the pilot is actively commanding climb/descend (Z/S, or
        # auto-takeoff), obey it and remember the altitude reached. Otherwise
        # hold that target altitude with a P controller, so yaw/translation
        # transients and drag don't permanently bleed off height.
        if abs(lz) > 1e-6 or self._z_now is None:
            up = clamp(lz) * LINEAR_SPEED
            if self._z_now is not None:
                self._z_target = self._z_now
        else:
            if self._z_target is None:
                self._z_target = self._z_now
            err = self._z_target - self._z_now
            up = max(-ALT_HOLD_VZ_MAX, min(ALT_HOLD_VZ_MAX, ALT_HOLD_KP * err))

        # Horizontal velocity in the WORLD frame. While the pilot commands
        # translation, fly in the heading frame (forward = where the nose
        # points). Otherwise command zero velocity so the controller brakes to a
        # stop in place — no fly-back to a held position (v1's loiter behaviour).
        yaw = math.atan2(self._R[1, 0], self._R[0, 0])
        cy, sy = math.cos(yaw), math.sin(yaw)
        if horiz > 1e-6:
            vx_w = fwd * cy - left * sy
            vy_w = fwd * sy + left * cy
        else:
            vx_w = vy_w = 0.0

        v_world = np.array([vx_w, vy_w, up])  # world X, Y, Z(altitude)

        # The MulticopterVelocityControl plugin rotates the commanded linear
        # velocity by the FULL body orientation (roll+pitch+yaw) before using
        # it as the desired velocity. If we sent the world velocity directly,
        # tilting forward to fly forward would bleed into a downward command
        # and the drone would sink. Pre-multiplying by R^T cancels that: the
        # plugin then computes R * (R^T * v_world) = v_world exactly.
        cmd_body = self._R.T @ v_world

        msg = Twist()
        msg.linear.x  = float(cmd_body[0])
        msg.linear.y  = float(cmd_body[1])
        msg.linear.z  = float(cmd_body[2])
        msg.angular.z = clamp(az) * ANGULAR_SPEED  # yaw rate (body z)
        return msg

    def run(self):
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)

        tty.setraw(self._fd)
        try:
            sys.stdout.write(HELP.replace('\n', '\r\n'))
            sys.stdout.flush()

            interval = 1.0 / PUBLISH_RATE
            while self._running and rclpy.ok():
                executor.spin_once(timeout_sec=0)

                for key in self._read_keys():
                    if key in ('ctrl_c', 'esc'):
                        self._running = False
                        break
                    elif key == 'takeoff':
                        self._takeoff_until = time.monotonic() + TAKEOFF_ALTITUDE_TIME
                    else:
                        self._last_seen[key] = time.monotonic()

                self.cmd_pub.publish(self._compute_twist())
                time.sleep(interval)
        finally:
            termios.tcsetattr(self._fd, termios.TCSANOW, self._old_term)
            self.cmd_pub.publish(Twist())   # stop drone on exit
            sys.stdout.write('\r\n')
            sys.stdout.flush()


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
