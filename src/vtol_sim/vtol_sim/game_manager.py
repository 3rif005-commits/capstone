#!/usr/bin/env python3
"""v2 duel game manager — referee + episode lifecycle.

The game is a duel:
  * YOU pilot the X3 multirotor as a KAMIKAZE, diving it into the TANK (asset).
  * An autonomous fixed-wing INTERCEPTOR (interceptor_node) tries to reach the
    kamikaze first, defending the tank.

This node is the referee: it spawns the tank, repositions the kamikaze far out
each episode, and decides the outcome —
  * kamikaze reaches tank  → KAMIKAZE_WIN (you score; tank explodes)
  * interceptor reaches kamikaze → DEFENSE_WIN (interceptor scores; kamikaze explodes)
  * neither within the time limit → TIMEOUT
It publishes a latched /game/reset so the interceptor re-spawns/re-aims, draws a
tactical minimap on /game/minimap, and logs per-episode metrics to v2_metrics.csv.

Run in a separate terminal:
  ros2 run vtol_sim game_manager
"""

import csv
import math
import os
import random
import re
import subprocess
import threading
import time
import sys

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String

# ── Tuning ────────────────────────────────────────────────────────────────────

TANK_HIT_DIST   = 4.0    # m: kamikaze reaches the tank → you win
INTERCEPT_DIST  = 6.0    # m: interceptor reaches kamikaze → defense wins
EPISODE_TIMEOUT = 90.0   # s: stalemate → new episode

# Kamikaze (X3) starts as an incoming attacker: far out, at altitude.
KAM_SPAWN_MIN_R = 90.0
KAM_SPAWN_MAX_R = 140.0
KAM_SPAWN_MIN_Z = 25.0
KAM_SPAWN_MAX_Z = 40.0
TANK_BASE_R     = 18.0   # tank spawns within this radius of origin (the base)

FIREBALL_SECS = 3.0
SMOKE_SECS    = 2.0

WORLD = 'vtol_world'
METRICS_CSV = 'v2_metrics.csv'

# ── Map image ─────────────────────────────────────────────────────────────────

MAP_PX    = 750          # image size (pixels, square)
MAP_RANGE = 150.0        # world metres from centre to each edge (engagement-scale)

# ── Static world geometry (mirrored from vtol_world.sdf) ─────────────────────
_WORLD_ROADS = [
    (0, 0, 100, 4,   (85,  85,  85)),
    (0, 0, 4,  100,  (85,  85,  85)),
]

_WORLD_BUILDINGS = [
    (-35,  0,    6,    4,    (155, 155, 160)),
    ( 30,  15,   4,    5,    (148, 150, 153)),
    ( 25, -20,   3,    3,    (210, 215, 220)),
    (-20,  35,  12.5,  6,    (105, 108, 112)),
    (-15, -30,   5,    5,    (168, 178, 192)),
    ( 40,   5,   4,    3,    (162, 172, 192)),
    (  5,  42,   3,    4,    (156, 168, 188)),
    (-40, -22,  11,    7.5,  ( 82,  86,  90)),
]

_CHIMNEY  = (-33, -18, 0.8)

_WORLD_TREES = [
    ( 18,  12, 2.3), ( 22,   8, 2.0), ( 16,  18, 2.5), ( 12,  14, 2.1),
    (-18, -12, 2.2), (-22,  -8, 2.4), (-14, -16, 2.0),
    ( -8,  28, 2.2), ( -4,  24, 2.1), (-12,  22, 2.5),
]


def _compact(xml: str) -> str:
    return re.sub(r'\s+', ' ', xml).strip()


_TANK_SDF = _compact("""
  <sdf version='1.6'>
    <model name='target_tank'>
      <static>true</static>
      <link name='body'>
        <visual name='hull'>
          <pose>0 0 0.75 0 0 0</pose>
          <geometry><box><size>4.0 2.0 1.5</size></box></geometry>
          <material><ambient>0.18 0.35 0.08 1</ambient><diffuse>0.22 0.42 0.10 1</diffuse></material>
        </visual>
        <collision name='hull_col'>
          <pose>0 0 0.75 0 0 0</pose>
          <geometry><box><size>4.0 2.0 1.5</size></box></geometry>
        </collision>
        <visual name='turret'>
          <pose>0 0 1.80 0 0 0</pose>
          <geometry><box><size>1.6 1.6 1.0</size></box></geometry>
          <material><ambient>0.15 0.30 0.07 1</ambient><diffuse>0.17 0.35 0.08 1</diffuse></material>
        </visual>
        <visual name='barrel'>
          <pose>1.8 0 1.80 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.12</radius><length>2.2</length></cylinder></geometry>
          <material><ambient>0.10 0.22 0.05 1</ambient><diffuse>0.10 0.22 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>
  </sdf>
""")

_FIREBALL_SDF = _compact("""
  <sdf version='1.6'>
    <model name='explosion_fireball'>
      <static>true</static>
      <link name='blast'>
        <visual name='vis'>
          <geometry><sphere><radius>5.0</radius></sphere></geometry>
          <material>
            <ambient>1.0 0.40 0.00 0.90</ambient>
            <diffuse>1.0 0.20 0.00 0.90</diffuse>
            <emissive>0.9 0.30 0.00 1.0</emissive>
          </material>
        </visual>
      </link>
    </model>
  </sdf>
""")

_SMOKE_SDF = _compact("""
  <sdf version='1.6'>
    <model name='explosion_smoke'>
      <static>true</static>
      <link name='cloud'>
        <visual name='vis'>
          <geometry><sphere><radius>4.0</radius></sphere></geometry>
          <material><ambient>0.07 0.07 0.07 0.80</ambient><diffuse>0.10 0.10 0.10 0.80</diffuse></material>
        </visual>
      </link>
    </model>
  </sdf>
""")


def _compass(wx: float, wy: float) -> str:
    bearing = math.degrees(math.atan2(-wy, wx)) % 360
    return ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'][round(bearing / 45) % 8]


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ── Game node ─────────────────────────────────────────────────────────────────

class GameManager(Node):
    def __init__(self):
        super().__init__('game_manager')

        # Kamikaze (player) state.
        self._kam_x = self._kam_y = self._kam_z = 0.0
        self._kam_yaw = 0.0
        self._kam_have = False

        # Interceptor (autonomous) state.
        self._int_x = self._int_y = self._int_z = 0.0
        self._int_yaw = 0.0
        self._int_have = False
        self._int_status = ''

        self._tank_x = self._tank_y = 0.0
        self._tank_spawned = False

        self._episode = 0
        self._state = 'WAITING'        # WAITING | ACTIVE | KAMIKAZE_WIN | DEFENSE_WIN
        self._ep_start = 0.0
        self._kam_score = 0            # tanks destroyed (you)
        self._def_score = 0            # kamikazes intercepted (defense)
        self._gz_ready = False

        self._bridge  = CvBridge()
        self._map_pub = self.create_publisher(Image, '/game/minimap', 10)

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self._reset_pub = self.create_publisher(Empty, '/game/reset', latched)

        self.create_subscription(Odometry, '/model/x3/odometry', self._on_kam_odom, 10)
        self.create_subscription(Odometry, '/interceptor/odometry', self._on_int_odom, 10)
        self.create_subscription(String, '/interceptor/status', self._on_int_status, 10)
        self.create_timer(0.1, self._tick)

        self._init_metrics()
        threading.Thread(target=self._wait_for_gazebo, daemon=True).start()
        print('[GAME] Drone Defense Duel — waiting for Gazebo world...')

    # ── Lifecycle ──────────────────────────────────────────────────────────
    def _wait_for_gazebo(self):
        create_svc = f'/world/{WORLD}/create'
        while not self._gz_ready:
            try:
                result = subprocess.run(['gz', 'service', '--list'],
                                        capture_output=True, text=True, timeout=5)
                if create_svc in result.stdout:
                    time.sleep(1.0)
                    self._gz_ready = True
                    print('[GAME] Gazebo ready — episode 1 starting...')
                    return
            except Exception:
                pass
            time.sleep(1.0)

    # ── Subscriptions ────────────────────────────────────────────────────────
    def _on_kam_odom(self, msg):
        p = msg.pose.pose.position
        self._kam_x, self._kam_y, self._kam_z = p.x, p.y, p.z
        self._kam_yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._kam_have = True

    def _on_int_odom(self, msg):
        p = msg.pose.pose.position
        self._int_x, self._int_y, self._int_z = p.x, p.y, p.z
        self._int_yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._int_have = True

    def _on_int_status(self, msg):
        self._int_status = msg.data

    # ── Gazebo service helpers ───────────────────────────────────────────────
    def _gz_spawn(self, sdf_inline: str, x: float, y: float, z: float = 0.0):
        req = (f'sdf: "{sdf_inline}" '
               f'pose: {{position: {{x: {x:.2f} y: {y:.2f} z: {z:.2f}}}}}')
        subprocess.Popen(
            ['gz', 'service', '-s', f'/world/{WORLD}/create',
             '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _gz_remove(self, name: str):
        subprocess.Popen(
            ['gz', 'service', '-s', f'/world/{WORLD}/remove',
             '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', f'name: "{name}" type: 2'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _gz_set_pose(self, model: str, x: float, y: float, z: float):
        req = f'name: "{model}" position: {{x: {x:.2f} y: {y:.2f} z: {z:.2f}}}'
        subprocess.Popen(
            ['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # ── Episode management ─────────────────────────────────────────────────
    def _tank_pos(self):
        while True:
            x = random.uniform(-TANK_BASE_R, TANK_BASE_R)
            y = random.uniform(-TANK_BASE_R, TANK_BASE_R)
            if math.hypot(x, y) <= TANK_BASE_R:
                return x, y

    def _kam_spawn(self):
        bearing = random.uniform(0, 2 * math.pi)
        dist = random.uniform(KAM_SPAWN_MIN_R, KAM_SPAWN_MAX_R)
        z = random.uniform(KAM_SPAWN_MIN_Z, KAM_SPAWN_MAX_Z)
        return dist * math.cos(bearing), dist * math.sin(bearing), z

    def _start_episode(self):
        self._episode += 1
        # Tank (asset) at the base.
        self._tank_x, self._tank_y = self._tank_pos()
        if self._tank_spawned:
            self._gz_remove('target_tank')
            self._tank_spawned = False
            time.sleep(0.3)
        self._gz_spawn(_TANK_SDF, self._tank_x, self._tank_y)
        self._tank_spawned = True

        # Reposition the kamikaze (X3) as an incoming attacker.
        kx, ky, kz = self._kam_spawn()
        self._gz_set_pose('x3', kx, ky, kz)

        # Tell the interceptor to (re)spawn and re-aim.
        self._reset_pub.publish(Empty())

        self._ep_start = time.monotonic()
        self._state = 'ACTIVE'
        dist = math.hypot(self._tank_x, self._tank_y)
        print(f'\n[EPISODE {self._episode}]  Tank at base; you spawn ~'
              f'{math.hypot(kx, ky):.0f} m out, {kz:.0f} m alt {_compass(kx, ky)}.')
        print('Dive into the tank before the interceptor catches you!')

    # ── Main tick (10 Hz) ─────────────────────────────────────────────────
    def _tick(self):
        if self._state == 'WAITING':
            if self._gz_ready:
                self._start_episode()
            self._publish_map()
            return

        if self._state in ('KAMIKAZE_WIN', 'DEFENSE_WIN'):
            self._publish_map()
            return

        # ── ACTIVE: referee ────────────────────────────────────────────────
        elapsed = time.monotonic() - self._ep_start
        if self._kam_have:
            d_tank = math.sqrt((self._kam_x - self._tank_x) ** 2 +
                               (self._kam_y - self._tank_y) ** 2 +
                               (self._kam_z - 1.0) ** 2)
        else:
            d_tank = 1e9
        if self._kam_have and self._int_have:
            d_int = math.sqrt((self._kam_x - self._int_x) ** 2 +
                              (self._kam_y - self._int_y) ** 2 +
                              (self._kam_z - self._int_z) ** 2)
        else:
            d_int = 1e9

        sys.stdout.write(
            f'\r  [Ep {self._episode}] {int(elapsed)//60:02d}:{int(elapsed)%60:02d}'
            f' | tank {d_tank:5.1f} m | interceptor {d_int:5.1f} m   '
        )
        sys.stdout.flush()
        self._publish_map(d_tank=d_tank, d_int=d_int, elapsed=elapsed)

        if d_int < INTERCEPT_DIST:
            self._end_episode('DEFENSE_WIN', elapsed, d_int, d_tank)
        elif d_tank < TANK_HIT_DIST:
            self._end_episode('KAMIKAZE_WIN', elapsed, d_int, d_tank)
        elif elapsed > EPISODE_TIMEOUT:
            self._end_episode('TIMEOUT', elapsed, d_int, d_tank)

    def _end_episode(self, outcome, elapsed, d_int, d_tank):
        self._state = outcome if outcome != 'TIMEOUT' else 'KAMIKAZE_WIN'  # draw map state
        self._log_metric(outcome, elapsed, d_int, d_tank)
        threading.Thread(target=self._resolve, args=(outcome, elapsed), daemon=True).start()

    def _resolve(self, outcome, elapsed):
        if outcome == 'DEFENSE_WIN':
            self._def_score += 1
            ex, ey, ez = self._kam_x, self._kam_y, self._kam_z   # kamikaze blown up
            print(f'\n[DEFENSE WIN] Interceptor splashed the kamikaze at '
                  f'{elapsed:04.1f}s.  Defense {self._def_score} : {self._kam_score} You')
        elif outcome == 'KAMIKAZE_WIN':
            self._kam_score += 1
            ex, ey, ez = self._tank_x, self._tank_y, 1.5         # tank blown up
            print(f'\n[KAMIKAZE WIN] Tank destroyed at {elapsed:04.1f}s!  '
                  f'You {self._kam_score} : {self._def_score} Defense')
        else:  # TIMEOUT
            ex = ey = ez = None
            print(f'\n[TIMEOUT] Stalemate at {elapsed:04.1f}s.')

        if ex is not None:
            self._gz_spawn(_FIREBALL_SDF, ex, ey, ez)
            time.sleep(FIREBALL_SECS)
            self._gz_remove('explosion_fireball')
            self._gz_spawn(_SMOKE_SDF, ex, ey, ez)
            time.sleep(SMOKE_SECS)
            self._gz_remove('explosion_smoke')

        self._start_episode()

    # ── Metrics ────────────────────────────────────────────────────────────
    def _init_metrics(self):
        self._metrics_path = os.path.abspath(METRICS_CSV)
        if not os.path.exists(self._metrics_path):
            with open(self._metrics_path, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['episode', 'outcome', 'duration_s',
                     'interceptor_kamikaze_range_m', 'kamikaze_tank_range_m',
                     'interceptor_status'])
        print(f'[GAME] metrics → {self._metrics_path}')

    def _log_metric(self, outcome, elapsed, d_int, d_tank):
        with open(self._metrics_path, 'a', newline='') as f:
            csv.writer(f).writerow(
                [self._episode, outcome, f'{elapsed:.2f}',
                 f'{d_int:.2f}', f'{d_tank:.2f}', self._int_status])

    # ── Minimap ──────────────────────────────────────────────────────────
    def _publish_map(self, d_tank=0.0, d_int=0.0, elapsed=0.0):
        img = self._build_map_image(d_tank, d_int, elapsed)
        msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self._map_pub.publish(msg)

    def _build_map_image(self, d_tank, d_int, elapsed) -> np.ndarray:
        S = MAP_PX
        img = np.full((S, S, 3), (110, 170, 75), dtype=np.uint8)

        def wpx(wx, wy):
            h = S / 2.0
            return (int(max(0, min(S - 1, h + (-wy / MAP_RANGE) * h))),
                    int(max(0, min(S - 1, h + (-wx / MAP_RANGE) * h))))

        def rect(wx, wy, hx, hy, fill, border=None, bt=2):
            c0, r0 = wpx(wx + hx, wy + hy)
            c1, r1 = wpx(wx - hx, wy - hy)
            cmin, cmax = min(c0, c1), max(c0, c1)
            rmin, rmax = min(r0, r1), max(r0, r1)
            if cmax <= cmin: cmax = cmin + 1
            if rmax <= rmin: rmax = rmin + 1
            cv2.rectangle(img, (cmin, rmin), (cmax, rmax), fill, -1)
            if border:
                cv2.rectangle(img, (cmin, rmin), (cmax, rmax), border, bt)

        font = cv2.FONT_HERSHEY_SIMPLEX

        # Roads
        for wx, wy, hx, hy, _ in _WORLD_ROADS:
            rect(wx, wy, hx, hy, (128, 125, 115))
        cv2.line(img, wpx(100, 0), wpx(-100, 0), (60, 215, 220), 1)
        cv2.line(img, wpx(0, -100), wpx(0, 100), (60, 215, 220), 1)

        # Trees
        for tx, ty, cr in _WORLD_TREES:
            pc, pr = wpx(tx, ty)
            r_px = max(7, int(cr / MAP_RANGE * (S // 2)))
            cv2.circle(img, (pc, pr), r_px, (30, 200, 30), -1)
            cv2.circle(img, (pc, pr), r_px, (0, 100, 0), 1)

        # Buildings
        _BLDG_COLORS = [
            (195, 200, 215), (185, 195, 210), (220, 225, 235), (160, 155, 145),
            (200, 205, 220), (210, 195, 180), (205, 190, 175), (135, 130, 125),
        ]
        for (wx, wy, hx, hy, _), fill in zip(_WORLD_BUILDINGS, _BLDG_COLORS):
            dark = tuple(max(0, c - 65) for c in fill)
            rect(wx, wy, hx, hy, fill, dark, bt=1)
        cc, cr = wpx(_CHIMNEY[0], _CHIMNEY[1])
        cv2.circle(img, (cc, cr), 4, (70, 65, 60), -1)

        # Origin marker
        oc, or_ = wpx(0, 0)
        cv2.line(img, (oc - 8, or_), (oc + 8, or_), (255, 255, 255), 1)
        cv2.line(img, (oc, or_ - 8), (oc, or_ + 8), (255, 255, 255), 1)

        # LOS line interceptor → kamikaze (the chase)
        if self._state == 'ACTIVE' and self._kam_have and self._int_have:
            kc, kr = wpx(self._kam_x, self._kam_y)
            ic, ir = wpx(self._int_x, self._int_y)
            steps = 20
            for i in range(0, steps, 2):
                t0, t1 = i / steps, (i + 0.6) / steps
                p0 = (int(ic + t0 * (kc - ic)), int(ir + t0 * (kr - ir)))
                p1 = (int(ic + t1 * (kc - ic)), int(ir + t1 * (kr - ir)))
                cv2.line(img, p0, p1, (40, 120, 255), 2)

        # Tank (asset)
        if self._tank_spawned or self._state != 'WAITING':
            tc, tr = wpx(self._tank_x, self._tank_y)
            if self._state == 'KAMIKAZE_WIN':
                for r, c in [(34, (20, 40, 200)), (22, (40, 110, 240)), (13, (100, 200, 255))]:
                    cv2.circle(img, (tc, tr), r, c, -1)
                cv2.putText(img, 'BOOM!', (tc - 28, tr - 38), font, 0.7, (0, 230, 255), 2)
            else:
                HW, HH = 16, 7
                cv2.rectangle(img, (tc - HW, tr - HH), (tc + HW, tr + HH), (20, 60, 10), -1)
                cv2.rectangle(img, (tc - HW, tr - HH), (tc + HW, tr + HH), (60, 230, 60), 2)
                cv2.circle(img, (tc, tr), 6, (15, 90, 15), -1)
                cv2.putText(img, 'TANK', (tc - 18, tr - HH - 5), font, 0.45, (60, 255, 60), 1)

        # Kamikaze (you) + interceptor as heading arrows
        self._draw_aircraft(img, wpx, self._kam_x, self._kam_y, self._kam_yaw,
                            (0, 245, 255), 'YOU', self._kam_have,
                            blown=(self._state == 'DEFENSE_WIN'))
        self._draw_aircraft(img, wpx, self._int_x, self._int_y, self._int_yaw,
                            (60, 90, 255), 'INTERCEPTOR', self._int_have, blown=False)

        # Compass
        for txt, pos in [('N', (S//2 - 8, 42)), ('S', (S//2 - 8, S - 8)),
                         ('W', (6, S//2 + 8)), ('E', (S - 26, S//2 + 8))]:
            cv2.putText(img, txt, (pos[0] + 1, pos[1] + 1), font, 0.7, (0, 0, 0), 2)
            cv2.putText(img, txt, pos, font, 0.7, (255, 255, 255), 2)

        # HUD
        cv2.rectangle(img, (0, 0), (S, 34), (20, 20, 20), -1)
        m, s = divmod(int(elapsed), 60)
        status = {'ACTIVE': 'ENGAGED', 'KAMIKAZE_WIN': 'TANK DESTROYED',
                  'DEFENSE_WIN': 'KAMIKAZE SPLASHED', 'WAITING': 'STANDBY'}.get(
                      self._state, self._state)
        hud = (f'Ep {self._episode}  {m:02d}:{s:02d}  You {self._kam_score}'
               f' : {self._def_score} Def   tank:{d_tank:5.1f}m  intc:{d_int:5.1f}m'
               f'   {status}')
        cv2.putText(img, hud, (6, 23), font, 0.5, (220, 220, 220), 1)
        cv2.rectangle(img, (0, 0), (S - 1, S - 1), (160, 160, 160), 2)
        return img

    def _draw_aircraft(self, img, wpx, wx, wy, yaw, color, label, have, blown):
        if not have:
            return
        dc, dr = wpx(wx, wy)
        if blown:
            for r, c in [(20, (20, 40, 200)), (12, (60, 140, 240))]:
                cv2.circle(img, (dc, dr), r, c, -1)
            cv2.putText(img, 'SPLASH', (dc - 28, dr - 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2)
            return
        hc, hr = -math.sin(yaw), -math.cos(yaw)
        rc, rr = hr, -hc
        L, W, B = 18, 10, 8
        nose  = (int(dc + L * hc), int(dr + L * hr))
        lwing = (int(dc - W * rc - B * hc), int(dr - W * rr - B * hr))
        rwing = (int(dc + W * rc - B * hc), int(dr + W * rr - B * hr))
        pts = np.array([nose, lwing, rwing], dtype=np.int32)
        cv2.fillPoly(img, [pts], (0, 0, 0))
        cv2.polylines(img, [pts.reshape(-1, 1, 2)], True, (0, 0, 0), 4)
        cv2.fillPoly(img, [pts], color)
        cv2.putText(img, label, (dc + 14, dr + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def main():
    rclpy.init()
    node = GameManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
