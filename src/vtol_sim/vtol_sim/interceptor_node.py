#!/usr/bin/env python3
"""Autonomous fixed-wing interceptor (v2).

Runs the validated, Gazebo-independent guidance core (see interception/) inside
ROS2 and drives a fixed-wing model in Gazebo:

  Layer 1 (state estimation): subscribe to the kamikaze's odometry
      (/model/x3/odometry), estimate its world velocity + acceleration by
      finite-differencing position (frame-agnostic — no body/world confusion).
  Layer 2 (guidance):         Pure Pursuit / True PN / Augmented PN, made
      collision-aware by adding the city obstacle-avoidance term.
  Layer 3 (flight control):   the fixed-wing kinematic model integrates the
      commanded lateral acceleration under bank / climb / airspeed limits.

The resulting pose is pushed to Gazebo at 50 Hz via the bridged SetEntityPose
service (async, single-flight — no per-tick subprocess), and republished on
/interceptor/odometry for the game manager (referee).

Episode lifecycle is driven by the game manager via a latched /game/reset:
each reset picks a new random sky spawn, resets the kinematic state, and aims
the nose at the kamikaze (vectoring onto intercept).

Run (after the launch file is up):
  ros2 run vtol_sim interceptor_node
"""

import math
import subprocess
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String
from ros_gz_interfaces.srv import SetEntityPose

from vtol_sim.interception.fixed_wing import FixedWing, FixedWingLimits
from vtol_sim.interception.guidance import make_law
from vtol_sim.interception.avoidance import ObstacleField
from vtol_sim.interception.world import city

WORLD = 'vtol_world'
MODEL = 'interceptor'

RATE_HZ = 50.0
DT = 1.0 / RATE_HZ

# Random sky-patrol spawn (combat air patrol near the defended asset).
SPAWN_MIN_R = 35.0
SPAWN_MAX_R = 65.0
SPAWN_MIN_Z = 30.0
SPAWN_MAX_Z = 45.0

# The interceptor holds station (stationary in the air) until the kamikaze has
# moved this far from where it sat at the episode start, then it launches.
ARM_DISPLACEMENT = 4.0   # m
ARM_GRACE_S = 1.0        # s: ignore the episode-start teleport settling


# Inline fixed-wing visual (gravity off, no collision: pose is fully driven by
# SetEntityPose, so physics never fights the kinematic model). +X = nose.
def _compact(xml: str) -> str:
    import re
    return re.sub(r'\s+', ' ', xml).strip()


_INTERCEPTOR_SDF = _compact("""
  <sdf version='1.6'>
    <model name='interceptor'>
      <link name='body'>
        <gravity>false</gravity>
        <inertial><mass>2.0</mass>
          <inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.05</izz>
                   <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <visual name='fuselage'>
          <geometry><box><size>1.4 0.18 0.18</size></box></geometry>
          <material><ambient>0.10 0.10 0.12 1</ambient><diffuse>0.15 0.15 0.18 1</diffuse></material>
        </visual>
        <visual name='nose'>
          <pose>0.75 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.09</radius><length>0.25</length></cylinder></geometry>
          <material><ambient>0.80 0.20 0.05 1</ambient><diffuse>0.95 0.25 0.05 1</diffuse></material>
        </visual>
        <visual name='wing'>
          <pose>0.05 0 0 0 0 0</pose>
          <geometry><box><size>0.30 2.2 0.04</size></box></geometry>
          <material><ambient>0.12 0.32 0.70 1</ambient><diffuse>0.15 0.40 0.90 1</diffuse></material>
        </visual>
        <visual name='tailplane'>
          <pose>-0.6 0 0 0 0 0</pose>
          <geometry><box><size>0.22 0.8 0.03</size></box></geometry>
          <material><ambient>0.15 0.40 0.90 1</ambient><diffuse>0.15 0.40 0.90 1</diffuse></material>
        </visual>
        <visual name='fin'>
          <pose>-0.6 0 0.16 0 0 0</pose>
          <geometry><box><size>0.22 0.04 0.30</size></box></geometry>
          <material><ambient>0.15 0.40 0.90 1</ambient><diffuse>0.15 0.40 0.90 1</diffuse></material>
        </visual>
      </link>
    </model>
  </sdf>
""")


def euler_to_quat(roll: float, pitch: float, yaw: float):
    """ZYX intrinsic euler → (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class InterceptorNode(Node):
    def __init__(self):
        super().__init__('interceptor_node')

        self.declare_parameter('guidance_law', 'apn')      # apn | pn | pure_pursuit
        self.declare_parameter('nav_constant', 4.0)
        law_name = self.get_parameter('guidance_law').value
        N = float(self.get_parameter('nav_constant').value)
        self._law_name = law_name
        self._law = make_law(law_name, **({} if law_name == 'pure_pursuit' else {'N': N}))
        self._field = ObstacleField(city())

        # Interceptor kinematic state (created on first reset).
        self._fw: FixedWing | None = None
        self._active = False
        self._spawned = False
        self._model_ready = False   # True once Gazebo confirms the model exists
        self._last_bank = 0.0

        # Arming: hold station until the kamikaze starts moving.
        self._armed = False
        self._kam_start = None
        self._arm_grace_until = 0.0

        # Kamikaze (target) state, estimated from odometry by finite difference.
        self._tgt_pos = None
        self._tgt_vel = np.zeros(3)
        self._tgt_acc = np.zeros(3)
        self._tgt_prev_pos = None
        self._tgt_prev_t = None

        self._gz_ready = False
        self._pending_reset = False
        self._pose_future = None

        # ── ROS interfaces ────────────────────────────────────────────────
        self.create_subscription(Odometry, '/model/x3/odometry', self._on_target_odom, 10)

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Empty, '/game/reset', self._on_reset, latched)

        self._odom_pub = self.create_publisher(Odometry, '/interceptor/odometry', 10)
        self._status_pub = self.create_publisher(String, '/interceptor/status', 10)

        self._set_pose = self.create_client(SetEntityPose, f'/world/{WORLD}/set_pose')

        self.create_timer(DT, self._tick)

        threading.Thread(target=self._wait_for_gazebo, daemon=True).start()
        print(f'[INTERCEPTOR] fixed-wing guidance = {law_name.upper()} '
              f'(N={N}) — waiting for Gazebo...')

    # ── Gazebo readiness ──────────────────────────────────────────────────
    def _wait_for_gazebo(self):
        create_svc = f'/world/{WORLD}/create'
        while not self._gz_ready:
            try:
                r = subprocess.run(['gz', 'service', '--list'],
                                   capture_output=True, text=True, timeout=5)
                if create_svc in r.stdout:
                    time.sleep(1.0)
                    self._gz_ready = True
                    print('[INTERCEPTOR] Gazebo ready.')
                    return
            except Exception:
                pass
            time.sleep(1.0)

    # ── Gazebo spawn (blocking + confirmed, in a thread) ──────────────────
    def _spawn_model(self, x, y, z):
        """Create the model via the blocking service and confirm it exists
        before any set_pose is issued (avoids 'Unable to update pose' races)."""
        req = (f'sdf: "{_INTERCEPTOR_SDF}" '
               f'pose: {{position: {{x: {x:.2f} y: {y:.2f} z: {z:.2f}}}}}')
        try:
            r = subprocess.run(
                ['gz', 'service', '-s', f'/world/{WORLD}/create/blocking',
                 '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '8000', '--req', req],
                capture_output=True, text=True, timeout=12)
            if 'true' in r.stdout.lower():
                self._model_ready = True
                print('[INTERCEPTOR] model spawned in Gazebo.')
            else:
                print(f'[INTERCEPTOR] spawn FAILED: {r.stdout.strip()} {r.stderr.strip()}')
        except Exception as e:
            print(f'[INTERCEPTOR] spawn error: {e}')

    # ── Target odometry → world velocity / acceleration estimate ──────────
    def _on_target_odom(self, msg):
        p = msg.pose.pose.position
        pos = np.array([p.x, p.y, p.z])
        now = time.monotonic()
        if self._tgt_prev_pos is not None:
            dt = max(now - self._tgt_prev_t, 1e-3)
            v = (pos - self._tgt_prev_pos) / dt
            a = (v - self._tgt_vel) / dt
            # Low-pass: odometry diff is noisy.
            self._tgt_vel = 0.6 * self._tgt_vel + 0.4 * v
            self._tgt_acc = 0.8 * self._tgt_acc + 0.2 * a
        self._tgt_prev_pos = pos
        self._tgt_prev_t = now
        self._tgt_pos = pos

    # ── Episode reset ─────────────────────────────────────────────────────
    def _on_reset(self, _msg):
        if not self._gz_ready:
            # Defer until Gazebo is up; _tick will retry via _active flag.
            self._pending_reset = True
            return
        self._do_reset()

    def _do_reset(self):
        bearing = np.random.uniform(0, 2 * math.pi)
        dist = np.random.uniform(SPAWN_MIN_R, SPAWN_MAX_R)
        z = np.random.uniform(SPAWN_MIN_Z, SPAWN_MAX_Z)
        pos = np.array([dist * math.cos(bearing), dist * math.sin(bearing), z])

        # Start at stall speed so arming feels like a launch (ramps to v_max).
        lim = FixedWingLimits()
        self._fw = FixedWing(pos=pos, speed=lim.v_min, limits=lim)
        # Face the kamikaze (or origin) but hold a level attitude while loitering.
        aim = self._tgt_pos if self._tgt_pos is not None else np.array([0.0, 0.0, 5.0])
        d = aim - pos
        self._fw.psi = math.atan2(d[1], d[0])
        self._fw.gamma = 0.0
        self._last_bank = 0.0

        # Disarm: hold station until the kamikaze moves (after a settle grace).
        self._armed = False
        self._kam_start = None
        self._arm_grace_until = time.monotonic() + ARM_GRACE_S

        if not self._spawned:
            self._spawned = True
            threading.Thread(target=self._spawn_model, args=tuple(float(c) for c in pos),
                             daemon=True).start()
        self._active = True
        print(f'[INTERCEPTOR] holding station at {pos.round(1)} — '
              f'waiting for the kamikaze to move...')

    # ── Control loop (50 Hz) ──────────────────────────────────────────────
    def _tick(self):
        if getattr(self, '_pending_reset', False) and self._gz_ready:
            self._pending_reset = False
            self._do_reset()
        if not (self._active and self._fw is not None):
            return

        # ── Hold station until the kamikaze starts moving ─────────────────
        if not self._armed:
            now = time.monotonic()
            if now >= self._arm_grace_until and self._tgt_pos is not None:
                if self._kam_start is None:
                    self._kam_start = self._tgt_pos.copy()   # settled start point
                elif np.linalg.norm(self._tgt_pos - self._kam_start) > ARM_DISPLACEMENT:
                    self._armed = True
                    print('\n[INTERCEPTOR] kamikaze moving — launching intercept!')
            # Stationary in the air: keep the model parked, report zero velocity.
            self._send_pose()
            self._publish_odom(stationary=True)
            return

        # Guidance + collision-aware avoidance → lateral accel command.
        if self._tgt_pos is not None:
            a_cmd = self._law.command(self._fw.pos, self._fw.velocity,
                                      self._tgt_pos, self._tgt_vel, self._tgt_acc)
        else:
            a_cmd = np.zeros(3)              # no target yet: fly straight
        a_cmd = a_cmd + self._field.avoid_accel(self._fw.pos, self._fw.velocity)

        info = self._fw.step(a_cmd, DT)
        self._last_bank = info['bank']

        self._send_pose()
        self._publish_odom()

    # ── Drive the model pose in Gazebo (async, single-flight) ─────────────
    def _send_pose(self):
        if not self._model_ready or not self._set_pose.service_is_ready():
            return
        if self._pose_future is not None and not self._pose_future.done():
            return                          # previous call still in flight
        fw = self._fw
        # roll = bank (visual lean into turn), pitch = -gamma (nose up climbs),
        # yaw = psi. See euler_to_quat.
        qx, qy, qz, qw = euler_to_quat(self._last_bank, -fw.gamma, fw.psi)
        req = SetEntityPose.Request()
        req.entity.name = MODEL
        req.entity.type = 2                 # MODEL
        req.pose = Pose()
        req.pose.position.x = float(fw.pos[0])
        req.pose.position.y = float(fw.pos[1])
        req.pose.position.z = float(fw.pos[2])
        req.pose.orientation.x = qx
        req.pose.orientation.y = qy
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw
        self._pose_future = self._set_pose.call_async(req)

    def _publish_odom(self, stationary: bool = False):
        fw = self._fw
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = MODEL
        msg.pose.pose.position.x = float(fw.pos[0])
        msg.pose.pose.position.y = float(fw.pos[1])
        msg.pose.pose.position.z = float(fw.pos[2])
        qx, qy, qz, qw = euler_to_quat(self._last_bank, -fw.gamma, fw.psi)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        v = np.zeros(3) if stationary else fw.velocity
        msg.twist.twist.linear.x = float(v[0])
        msg.twist.twist.linear.y = float(v[1])
        msg.twist.twist.linear.z = float(v[2])
        self._odom_pub.publish(msg)

        if self._tgt_pos is not None:
            rng = float(np.linalg.norm(self._tgt_pos - fw.pos))
            s = String()
            s.data = f'{self._law_name} range={rng:.1f} spd={fw.speed:.1f}'
            self._status_pub.publish(s)


def main():
    rclpy.init()
    node = InterceptorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
