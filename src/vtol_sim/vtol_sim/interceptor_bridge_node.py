#!/usr/bin/env python3
"""Interceptor bridge node — PC side, HIL mode.

Replaces interceptor_node.py when the RPi is doing the guidance work.
All guidance computation (Layers 1-3) runs on the RPi; this node is
pure ROS2/Gazebo glue:

  ┌───────────────────────────────────────────────────────┐
  │  PC (this node)              RPi (interceptor_runner) │
  │  /model/x3/odometry          ┌─────────────────────┐  │
  │       │ raw pos + t  ──UDP──►│ Layer 1: est vel/acc│  │
  │       │              ◄──UDP──│ Layer 2: APN guidance│  │
  │  SetEntityPose (Gazebo)      │ Layer 3: FixedWing   │  │
  │  /interceptor/odometry       └─────────────────────┘  │
  └───────────────────────────────────────────────────────┘

Run (after the normal launch file is up, RPi runner already started):
  export RPI_IP=<your-rpi-ip>
  ros2 launch vtol_sim vtol_sim_hil.launch.py
"""

import json
import math
import socket
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

WORLD = 'vtol_world'
MODEL = 'interceptor'

# 15 Hz (was 30): each pose is a set_pose service round-trip through the gz
# service bridge, which was pegging a CPU core (~94%) and starving the gz->ros
# odometry bridge so the interceptor never received target ticks. 15 Hz is still
# smooth for guidance and roughly halves that load.
RATE_HZ = 15.0
DT      = 1.0 / RATE_HZ

SPAWN_MIN_R = 35.0
SPAWN_MAX_R = 65.0
SPAWN_MIN_Z = 30.0
SPAWN_MAX_Z = 45.0

RPI_PORT    = 5555   # RPi listens on this port
BRIDGE_PORT = 5556   # This node listens on this port


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
          <material><ambient>0.10 0.10 0.12 1</ambient>
                    <diffuse>0.15 0.15 0.18 1</diffuse></material>
        </visual>
        <visual name='nose'>
          <pose>0.75 0 0 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.09</radius><length>0.25</length></cylinder></geometry>
          <material><ambient>0.80 0.20 0.05 1</ambient>
                    <diffuse>0.95 0.25 0.05 1</diffuse></material>
        </visual>
        <visual name='wing'>
          <pose>0.05 0 0 0 0 0</pose>
          <geometry><box><size>0.30 2.2 0.04</size></box></geometry>
          <material><ambient>0.12 0.32 0.70 1</ambient>
                    <diffuse>0.15 0.40 0.90 1</diffuse></material>
        </visual>
        <visual name='tailplane'>
          <pose>-0.6 0 0 0 0 0</pose>
          <geometry><box><size>0.22 0.8 0.03</size></box></geometry>
          <material><ambient>0.15 0.40 0.90 1</ambient>
                    <diffuse>0.15 0.40 0.90 1</diffuse></material>
        </visual>
        <visual name='fin'>
          <pose>-0.6 0 0.16 0 0 0</pose>
          <geometry><box><size>0.22 0.04 0.30</size></box></geometry>
          <material><ambient>0.15 0.40 0.90 1</ambient>
                    <diffuse>0.15 0.40 0.90 1</diffuse></material>
        </visual>
      </link>
    </model>
  </sdf>
""")


def euler_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class InterceptorBridgeNode(Node):
    def __init__(self):
        super().__init__('interceptor_bridge_node')

        self.declare_parameter('rpi_ip',      '')
        self.declare_parameter('rpi_port',    RPI_PORT)
        self.declare_parameter('bridge_port', BRIDGE_PORT)
        self.declare_parameter('guidance_law','apn')

        rpi_ip      = self.get_parameter('rpi_ip').value
        rpi_port    = int(self.get_parameter('rpi_port').value)
        bridge_port = int(self.get_parameter('bridge_port').value)
        self._law_name = self.get_parameter('guidance_law').value

        self._rpi_addr = (rpi_ip, rpi_port)

        # Latest pose received from RPi
        self._rpi_pose: dict | None = None
        self._pose_lock = threading.Lock()

        # Gazebo / model bookkeeping
        self._gz_ready    = False
        self._spawned     = False
        self._model_ready = False
        self._active      = False
        self._pending_reset = False
        self._pose_future = None

        # Latest raw kamikaze position (forwarded to RPi as-is)
        self._tgt_pos: list | None = None
        self._tgt_t:   float | None = None

        # UDP: one TX socket (unbound, send only), one RX socket
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.bind(('', bridge_port))
        self._rx.settimeout(0.01)

        # ROS2 interfaces
        self.create_subscription(Odometry, '/model/x3/odometry',
                                 self._on_target_odom, 10)

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Empty, '/game/reset', self._on_reset, latched)

        self._odom_pub   = self.create_publisher(Odometry, '/interceptor/odometry', 10)
        self._status_pub = self.create_publisher(String,   '/interceptor/status',   10)
        self._set_pose   = self.create_client(SetEntityPose,
                                              f'/world/{WORLD}/set_pose')

        self.create_timer(DT, self._tick)

        threading.Thread(target=self._wait_for_gazebo, daemon=True).start()
        threading.Thread(target=self._rx_loop,         daemon=True).start()

        print(f'[BRIDGE] HIL mode  RPi={rpi_ip}:{rpi_port}  '
              f'bridge listening :{bridge_port}')

    # ── Receive loop (background thread) ──────────────────────────────────
    def _rx_loop(self):
        while True:
            try:
                data, _ = self._rx.recvfrom(4096)
                pkt = json.loads(data)
                with self._pose_lock:
                    self._rpi_pose = pkt
            except socket.timeout:
                pass
            except Exception:
                pass

    # ── Gazebo readiness ──────────────────────────────────────────────────
    def _wait_for_gazebo(self):
        svc = f'/world/{WORLD}/create'
        while not self._gz_ready:
            try:
                r = subprocess.run(['gz', 'service', '--list'],
                                   capture_output=True, text=True, timeout=5)
                if svc in r.stdout:
                    time.sleep(1.0)
                    self._gz_ready = True
                    print('[BRIDGE] Gazebo ready.')
                    return
            except Exception:
                pass
            time.sleep(1.0)

    def _spawn_model(self, x: float, y: float, z: float):
        req = (f'sdf: "{_INTERCEPTOR_SDF}" '
               f'pose: {{position: {{x: {x:.2f} y: {y:.2f} z: {z:.2f}}}}}')
        try:
            r = subprocess.run(
                ['gz', 'service', '-s', f'/world/{WORLD}/create/blocking',
                 '--reqtype', 'gz.msgs.EntityFactory',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '8000', '--req', req],
                capture_output=True, text=True, timeout=12)
            if 'true' in r.stdout.lower():
                self._model_ready = True
                print('[BRIDGE] interceptor model spawned.')
            else:
                print(f'[BRIDGE] spawn failed: {r.stdout.strip()} {r.stderr.strip()}')
        except Exception as e:
            print(f'[BRIDGE] spawn error: {e}')

    # ── Target odometry → forward raw pos to RPi ──────────────────────────
    def _on_target_odom(self, msg):
        p = msg.pose.pose.position
        pos = [p.x, p.y, p.z]
        t   = time.monotonic()
        self._tgt_pos = pos
        self._tgt_t   = t
        if self._active:
            pkt = json.dumps({'type': 'tick', 'tgt_pos': pos, 't': t}).encode()
            self._tx.sendto(pkt, self._rpi_addr)

    # ── Episode reset ─────────────────────────────────────────────────────
    def _on_reset(self, _msg):
        if not self._gz_ready:
            self._pending_reset = True
            return
        self._do_reset()

    def _do_reset(self):
        bearing = np.random.uniform(0, 2 * math.pi)
        dist    = np.random.uniform(SPAWN_MIN_R, SPAWN_MAX_R)
        z       = np.random.uniform(SPAWN_MIN_Z, SPAWN_MAX_Z)
        pos     = [dist * math.cos(bearing), dist * math.sin(bearing), z]

        aim  = self._tgt_pos if self._tgt_pos else [0.0, 0.0, 5.0]
        psi  = math.atan2(aim[1] - pos[1], aim[0] - pos[0])

        # Tell RPi to initialise at this spawn
        pkt = json.dumps({'type': 'reset', 'spawn_pos': pos, 'spawn_psi': psi}).encode()
        self._tx.sendto(pkt, self._rpi_addr)

        if not self._spawned:
            self._spawned = True
            threading.Thread(target=self._spawn_model,
                             args=(pos[0], pos[1], pos[2]), daemon=True).start()

        self._active = True
        print(f'[BRIDGE] reset → spawn={[round(v, 1) for v in pos]}  '
              f'psi={math.degrees(psi):.1f}°  sent to RPi')

    # ── Control loop: apply RPi pose to Gazebo ────────────────────────────
    def _tick(self):
        if self._pending_reset and self._gz_ready:
            self._pending_reset = False
            self._do_reset()

        if not self._active:
            return

        with self._pose_lock:
            pose = self._rpi_pose

        if pose is None:
            return   # waiting for first packet from RPi

        self._send_pose_to_gz(pose)
        self._publish_odom(pose)

    def _send_pose_to_gz(self, pose: dict):
        if not self._model_ready or not self._set_pose.service_is_ready():
            return
        if self._pose_future is not None and not self._pose_future.done():
            return  # previous call still in flight

        pos = pose['pos']
        qx, qy, qz, qw = euler_to_quat(pose['bank'], -pose['gamma'], pose['psi'])

        req = SetEntityPose.Request()
        req.entity.name = MODEL
        req.entity.type = 2
        req.pose = Pose()
        req.pose.position.x    = float(pos[0])
        req.pose.position.y    = float(pos[1])
        req.pose.position.z    = float(pos[2])
        req.pose.orientation.x = qx
        req.pose.orientation.y = qy
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw
        self._pose_future = self._set_pose.call_async(req)

    def _publish_odom(self, pose: dict):
        pos = pose['pos']
        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id  = MODEL
        msg.pose.pose.position.x    = float(pos[0])
        msg.pose.pose.position.y    = float(pos[1])
        msg.pose.pose.position.z    = float(pos[2])
        qx, qy, qz, qw = euler_to_quat(pose['bank'], -pose['gamma'], pose['psi'])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self._odom_pub.publish(msg)

        if self._tgt_pos:
            rng = float(np.linalg.norm(np.array(self._tgt_pos) - np.array(pos)))
            s = String()
            s.data = (f'{self._law_name}[RPi] range={rng:.1f} '
                      f'spd={pose.get("speed", 0.0):.1f}')
            self._status_pub.publish(s)


def main():
    rclpy.init()
    node = InterceptorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
