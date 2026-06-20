#!/usr/bin/env python3
"""Machinima director — plays the "Intercept" shot list in Gazebo.

Everything is puppeteered (no flight physics, no teleop): the director spawns
the tank + two drone stand-ins (gravity-off, collision-free visuals, exactly
like the interceptor model) plus the reusable fireball/smoke blast props, then
walks the :mod:`vtol_sim.machinima.shots` timeline. Each tick it:

  * samples the current shot's camera move and drives the baked-in ``cine_cam``
    model (a movable camera sensor) via ``SetEntityPose``;
  * samples the shot's puppet poses and drives each drone/prop the same way;
  * gates the recorder (``/machinima/record``) so the take is one clean clip.

The camera sensor publishes ``/cine_cam/image`` which ``ros_gz_image`` bridges to
ROS for :mod:`vtol_sim.machinima_recorder`.

Run (after the launch file is up):
  ros2 run vtol_sim machinima_director
"""
import math
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from ros_gz_interfaces.srv import SetEntityPose

from vtol_sim.machinima import shots as scenario
from vtol_sim.machinima.camera_moves import look_at_quat

WORLD = 'machinima_world'
CAM = 'cine_cam'
RATE_HZ = 30.0
DT = 1.0 / RATE_HZ
LEAD_IN_S = 1.0          # let first poses settle before the recorder rolls
OUTRO_S = 2.0            # after the take, hold then shut down cleanly


def _compact(xml: str) -> str:
    return re.sub(r'\s+', ' ', xml).strip()


def euler_to_quat(roll, pitch, yaw):
    """ZYX intrinsic euler -> (x, y, z, w). (Matches interceptor_node.)"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


# ── Puppet SDFs (visual only: gravity off, no collision; pose fully driven) ──────
_INTERCEPTOR_SDF = _compact("""
  <sdf version='1.6'><model name='interceptor'><link name='body'>
    <gravity>false</gravity>
    <inertial><mass>2.0</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.05</izz>
      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <visual name='fuselage'><geometry><box><size>1.4 0.18 0.18</size></box></geometry>
      <material><ambient>0.10 0.10 0.12 1</ambient><diffuse>0.15 0.15 0.18 1</diffuse></material></visual>
    <visual name='nose'><pose>0.75 0 0 0 1.5708 0</pose>
      <geometry><cylinder><radius>0.09</radius><length>0.25</length></cylinder></geometry>
      <material><ambient>0.80 0.20 0.05 1</ambient><diffuse>0.95 0.25 0.05 1</diffuse></material></visual>
    <visual name='wing'><pose>0.05 0 0 0 0 0</pose>
      <geometry><box><size>0.30 2.2 0.04</size></box></geometry>
      <material><ambient>0.12 0.32 0.70 1</ambient><diffuse>0.15 0.40 0.90 1</diffuse></material></visual>
    <visual name='tailplane'><pose>-0.6 0 0 0 0 0</pose>
      <geometry><box><size>0.22 0.8 0.03</size></box></geometry>
      <material><ambient>0.15 0.40 0.90 1</ambient><diffuse>0.15 0.40 0.90 1</diffuse></material></visual>
    <visual name='fin'><pose>-0.6 0 0.16 0 0 0</pose>
      <geometry><box><size>0.22 0.04 0.30</size></box></geometry>
      <material><ambient>0.15 0.40 0.90 1</ambient><diffuse>0.15 0.40 0.90 1</diffuse></material></visual>
  </link></model></sdf>
""")

# Menacing dark quad with red accents — the kamikaze stand-in.
_KAMIKAZE_SDF = _compact("""
  <sdf version='1.6'><model name='kamikaze'><link name='body'>
    <gravity>false</gravity>
    <inertial><mass>1.0</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.02</izz>
      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <visual name='core'><geometry><box><size>0.45 0.45 0.16</size></box></geometry>
      <material><ambient>0.05 0.05 0.06 1</ambient><diffuse>0.08 0.08 0.10 1</diffuse></material></visual>
    <visual name='warhead'><pose>0.28 0 0 0 1.5708 0</pose>
      <geometry><cylinder><radius>0.08</radius><length>0.22</length></cylinder></geometry>
      <material><ambient>0.80 0.10 0.05 1</ambient><diffuse>0.95 0.12 0.05 1</diffuse>
        <emissive>0.4 0.02 0.0 1</emissive></material></visual>
    <visual name='arm1'><pose>0 0 0 0 0 0.7854</pose>
      <geometry><box><size>0.95 0.05 0.04</size></box></geometry>
      <material><ambient>0.05 0.05 0.06 1</ambient><diffuse>0.08 0.08 0.10 1</diffuse></material></visual>
    <visual name='arm2'><pose>0 0 0 0 0 -0.7854</pose>
      <geometry><box><size>0.95 0.05 0.04</size></box></geometry>
      <material><ambient>0.05 0.05 0.06 1</ambient><diffuse>0.08 0.08 0.10 1</diffuse></material></visual>
    <visual name='r0'><pose>0.34 0.34 0.05 0 0 0</pose>
      <geometry><cylinder><radius>0.18</radius><length>0.02</length></cylinder></geometry>
      <material><ambient>0.10 0.10 0.12 0.6</ambient><diffuse>0.12 0.12 0.14 0.6</diffuse></material></visual>
    <visual name='r1'><pose>-0.34 0.34 0.05 0 0 0</pose>
      <geometry><cylinder><radius>0.18</radius><length>0.02</length></cylinder></geometry>
      <material><ambient>0.10 0.10 0.12 0.6</ambient><diffuse>0.12 0.12 0.14 0.6</diffuse></material></visual>
    <visual name='r2'><pose>0.34 -0.34 0.05 0 0 0</pose>
      <geometry><cylinder><radius>0.18</radius><length>0.02</length></cylinder></geometry>
      <material><ambient>0.10 0.10 0.12 0.6</ambient><diffuse>0.12 0.12 0.14 0.6</diffuse></material></visual>
    <visual name='r3'><pose>-0.34 -0.34 0.05 0 0 0</pose>
      <geometry><cylinder><radius>0.18</radius><length>0.02</length></cylinder></geometry>
      <material><ambient>0.10 0.10 0.12 0.6</ambient><diffuse>0.12 0.12 0.14 0.6</diffuse></material></visual>
  </link></model></sdf>
""")

_TANK_SDF = _compact("""
  <sdf version='1.6'><model name='target_tank'><static>true</static><link name='body'>
    <visual name='hull'><pose>0 0 0.75 0 0 0</pose>
      <geometry><box><size>4.0 2.0 1.5</size></box></geometry>
      <material><ambient>0.18 0.35 0.08 1</ambient><diffuse>0.22 0.42 0.10 1</diffuse></material></visual>
    <visual name='turret'><pose>0 0 1.80 0 0 0</pose>
      <geometry><box><size>1.6 1.6 1.0</size></box></geometry>
      <material><ambient>0.15 0.30 0.07 1</ambient><diffuse>0.17 0.35 0.08 1</diffuse></material></visual>
    <visual name='barrel'><pose>1.8 0 1.80 0 1.5708 0</pose>
      <geometry><cylinder><radius>0.12</radius><length>2.2</length></cylinder></geometry>
      <material><ambient>0.10 0.22 0.05 1</ambient><diffuse>0.10 0.22 0.05 1</diffuse></material></visual>
  </link></model></sdf>
""")

_FIREBALL_SDF = _compact("""
  <sdf version='1.6'><model name='explosion_fireball'><static>true</static><link name='blast'>
    <visual name='vis'><geometry><sphere><radius>7.0</radius></sphere></geometry>
      <material><ambient>1.0 0.40 0.00 0.90</ambient><diffuse>1.0 0.20 0.00 0.90</diffuse>
        <emissive>0.9 0.30 0.00 1.0</emissive></material></visual>
  </link></model></sdf>
""")

_SMOKE_SDF = _compact("""
  <sdf version='1.6'><model name='explosion_smoke'><static>true</static><link name='cloud'>
    <visual name='vis'><geometry><sphere><radius>4.0</radius></sphere></geometry>
      <material><ambient>0.07 0.07 0.07 0.80</ambient><diffuse>0.10 0.10 0.10 0.80</diffuse></material></visual>
  </link></model></sdf>
""")

# (model name, sdf, spawn-at-pose) — props spawn parked/underground until revealed.
_PARK = scenario.SCENE['park']
_PUPPETS = [
    (scenario.TANK, _TANK_SDF, tuple(scenario.SCENE['tank'])),
    (scenario.KAM,  _KAMIKAZE_SDF, tuple(_PARK)),
    (scenario.INT,  _INTERCEPTOR_SDF, tuple(_PARK)),
    (scenario.FIRE, _FIREBALL_SDF, tuple(_PARK)),
    (scenario.SMOKE, _SMOKE_SDF, tuple(_PARK)),
]


class MachinimaDirector(Node):
    def __init__(self):
        super().__init__('machinima_director')
        self.declare_parameter('autoplay', True)
        self._autoplay = bool(self.get_parameter('autoplay').value)

        self._shots = scenario.build_shots()
        self._total = sum(s.duration for s in self._shots)
        self._gz_ready = False
        self._spawned = False
        self._t0 = None
        self._recording = False
        self._done = False
        self._finish_mono = None
        self._futures = {}                  # per-entity single-flight set_pose

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self._rec_pub = self.create_publisher(Bool, '/machinima/record', latched)
        self._set_pose = self.create_client(SetEntityPose, f'/world/{WORLD}/set_pose')

        self.create_timer(DT, self._tick)
        threading.Thread(target=self._bringup, daemon=True).start()
        print(f'[DIRECTOR] "Intercept" — {len(self._shots)} shots, '
              f'{self._total:.0f}s — waiting for Gazebo...')

    # ── Bring-up: wait for gz, spawn the cast ───────────────────────────────────
    def _bringup(self):
        create_svc = f'/world/{WORLD}/create'
        while not self._gz_ready:
            try:
                r = subprocess.run(['gz', 'service', '--list'],
                                   capture_output=True, text=True, timeout=5)
                if create_svc in r.stdout:
                    time.sleep(1.0)
                    self._gz_ready = True
            except Exception:
                pass
            if not self._gz_ready:
                time.sleep(1.0)
        print('[DIRECTOR] Gazebo ready — spawning cast...')
        for name, sdf, pose in _PUPPETS:
            self._spawn(name, sdf, pose)
            time.sleep(0.4)             # don't hammer the create service
        self._spawned = True
        print('[DIRECTOR] cast ready.')

    def _spawn(self, name, sdf, pose, attempts=3):
        """Blocking create + confirm. The service can transiently time out under
        load, so retry a few times before giving up."""
        x, y, z = pose
        req = (f'sdf: "{sdf}" '
               f'pose: {{position: {{x: {x:.2f} y: {y:.2f} z: {z:.2f}}}}}')
        for i in range(1, attempts + 1):
            try:
                r = subprocess.run(
                    ['gz', 'service', '-s', f'/world/{WORLD}/create/blocking',
                     '--reqtype', 'gz.msgs.EntityFactory',
                     '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '8000', '--req', req],
                    capture_output=True, text=True, timeout=12)
                if 'true' in r.stdout.lower():
                    print(f'[DIRECTOR]   spawn {name}: ok'
                          f'{"" if i == 1 else f" (attempt {i})"}')
                    return
            except Exception as e:
                if i == attempts:
                    print(f'[DIRECTOR]   spawn {name} error: {e}')
            time.sleep(0.6)
        print(f'[DIRECTOR]   spawn {name}: FAILED after {attempts} attempts')

    # ── Pose driver (per-entity single-flight) ──────────────────────────────────
    def _set(self, name, x, y, z, qx, qy, qz, qw):
        if not self._set_pose.service_is_ready():
            return
        fut = self._futures.get(name)
        if fut is not None and not fut.done():
            return
        req = SetEntityPose.Request()
        req.entity.name = name
        req.entity.type = 2                 # MODEL
        req.pose = Pose()
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.x, req.pose.orientation.y = qx, qy
        req.pose.orientation.z, req.pose.orientation.w = qz, qw
        self._futures[name] = self._set_pose.call_async(req)

    # ── Timeline ────────────────────────────────────────────────────────────────
    def _current_shot(self, t):
        acc = 0.0
        for sh in self._shots:
            if t < acc + sh.duration:
                return sh, (t - acc) / sh.duration
            acc += sh.duration
        return None, 0.0

    def _tick(self):
        if self._done:
            # Take is over: give the recorder a beat to flush, then shut down so
            # the launch doesn't hang waiting on the director.
            if self._finish_mono and (time.monotonic() - self._finish_mono) > OUTRO_S:
                rclpy.try_shutdown()
            return
        if not (self._gz_ready and self._spawned and self._autoplay):
            return
        if self._t0 is None:
            self._t0 = time.monotonic()
            print(f'[DIRECTOR] action! (lead-in {LEAD_IN_S:.0f}s)')

        elapsed = time.monotonic() - self._t0
        t = elapsed - LEAD_IN_S             # negative during lead-in

        # Roll the recorder once the lead-in is over.
        if not self._recording and t >= 0.0:
            self._rec_pub.publish(Bool(data=True))
            self._recording = True

        # Drive the current shot (clamp to first shot during lead-in).
        sh, u = self._current_shot(max(0.0, t))
        if sh is None:
            self._finish()
            return
        self._apply(sh, u)

    def _apply(self, sh, u):
        cam = sh.camera(u)
        qx, qy, qz, qw = look_at_quat(cam.eye, cam.look_at, roll=cam.roll)
        self._set(CAM, cam.eye[0], cam.eye[1], cam.eye[2], qx, qy, qz, qw)

        if sh.entities is not None:
            for name, (x, y, z, roll, pitch, yaw) in sh.entities(u).items():
                eqx, eqy, eqz, eqw = euler_to_quat(roll, pitch, yaw)
                self._set(name, x, y, z, eqx, eqy, eqz, eqw)

    def _finish(self):
        self._done = True
        self._finish_mono = time.monotonic()
        if self._recording:
            self._rec_pub.publish(Bool(data=False))
        print(f'[DIRECTOR] cut! take complete ({self._total:.0f}s) — '
              f'saved under media/. Shutting down.')


def main(args=None):
    rclpy.init(args=args)
    node = MachinimaDirector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
