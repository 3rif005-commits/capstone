#!/usr/bin/env python3
"""
Kamikaze drone game manager.

Each episode: a target tank spawns at a random location.
Fly the drone into it (kamikaze) to destroy it.
A bird's-eye map is published on /game/minimap and shown in rqt_image_view.

Run in a separate terminal:
  ros2 run vtol_sim game_manager
"""

import math
import random
import subprocess
import threading
import time
import sys

import re
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

# ── Tuning ────────────────────────────────────────────────────────────────────

HIT_DISTANCE  = 3.5    # metres: distance threshold for a kamikaze hit
SPAWN_MIN     = 20.0   # tank at least this far from drone origin
SPAWN_MAX     = 60.0

FIREBALL_SECS = 3.0
SMOKE_SECS    = 2.0

WORLD = 'vtol_world'

# ── Map image ─────────────────────────────────────────────────────────────────

MAP_PX    = 750          # image size (pixels, square)
MAP_RANGE = 70.0         # world metres shown from centre to each edge

# ── Static world geometry (mirrored from vtol_world.sdf) ─────────────────────
# All positions are world (wx=North, wy=West) centre + half-extents.

_WORLD_ROADS = [
    # (wx, wy, half_x, half_y, bgr)
    (0, 0, 100, 4,   (85,  85,  85)),   # road_ns  — runs N/S, 8 m wide
    (0, 0, 4,  100,  (85,  85,  85)),   # road_ew  — runs E/W, 8 m wide
]

_WORLD_BUILDINGS = [
    # (wx, wy, half_x, half_y, bgr_fill)
    (-35,  0,    6,    4,    (155, 155, 160)),  # office_a   12×8
    ( 30,  15,   4,    5,    (148, 150, 153)),  # office_b    8×10
    ( 25, -20,   3,    3,    (210, 215, 220)),  # tower       6×6  (tallest)
    (-20,  35,  12.5,  6,    (105, 108, 112)),  # warehouse  25×12
    (-15, -30,   5,    5,    (168, 178, 192)),  # apartment  10×10
    ( 40,   5,   4,    3,    (162, 172, 192)),  # shop_a      8×6
    (  5,  42,   3,    4,    (156, 168, 188)),  # shop_b      6×8
    (-40, -22,  11,    7.5,  ( 82,  86,  90)),  # factory    22×15
]

_CHIMNEY  = (-33, -18, 0.8)   # (wx, wy, world_radius_m)

_WORLD_TREES = [
    # (wx, wy, crown_radius_m)
    ( 18,  12, 2.3), ( 22,   8, 2.0), ( 16,  18, 2.5), ( 12,  14, 2.1),
    (-18, -12, 2.2), (-22,  -8, 2.4), (-14, -16, 2.0),
    ( -8,  28, 2.2), ( -4,  24, 2.1), (-12,  22, 2.5),
]

# ── SDF templates (inline, single-quoted XML — no escaping needed in proto text) ──

def _compact(xml: str) -> str:
    """Strip indentation/newlines so the SDF fits on one proto-text line."""
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


# ── Helpers ───────────────────────────────────────────────────────────────────

def _world_to_px(wx: float, wy: float) -> tuple[int, int]:
    """World coords (+X=North, +Y=West) → image (col, row) pixel."""
    half = MAP_PX // 2
    col = int(half + (-wy / MAP_RANGE) * half)
    row = int(half + (-wx / MAP_RANGE) * half)
    return (max(0, min(MAP_PX - 1, col)),
            max(0, min(MAP_PX - 1, row)))


def _compass(wx: float, wy: float) -> str:
    bearing = math.degrees(math.atan2(-wy, wx)) % 360
    return ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'][round(bearing / 45) % 8]


def _heat(dist: float) -> str:
    if dist > 60: return 'ICE COLD'
    if dist > 40: return 'Cold'
    if dist > 25: return 'Warm'
    if dist > 15: return '** HOT **'
    if dist > 8:  return '>>> BURNING <<<'
    return               '!!! ON TARGET !!!'


# ── Game node ─────────────────────────────────────────────────────────────────

class GameManager(Node):
    def __init__(self):
        super().__init__('game_manager')

        self._drone_x   = 0.0
        self._drone_y   = 0.0
        self._drone_z   = 0.0
        self._drone_yaw = 0.0   # radians, 0 = facing North (+X)

        self._episode      = 0
        self._scores: list[float] = []
        self._tank_x       = 0.0
        self._tank_y       = 0.0
        self._ep_start     = 0.0
        self._state        = 'WAITING'
        self._tank_spawned = False   # True only while a tank entity exists in Gazebo
        self._gz_ready     = False   # True once Gazebo world services are registered

        self._bridge  = CvBridge()
        self._map_pub = self.create_publisher(Image, '/game/minimap', 10)

        self.create_subscription(Odometry, '/model/x3/odometry', self._on_odom, 10)
        self.create_timer(0.1, self._tick)

        # Poll gz services in the background so we don't spawn before Gazebo is ready
        threading.Thread(target=self._wait_for_gazebo, daemon=True).start()
        print('[GAME] Kamikaze Drone Hunt — waiting for Gazebo world...')

    # ── Lifecycle ──────────────────────────────────────────────────────────

    def destroy_node(self):
        super().destroy_node()

    def _wait_for_gazebo(self):
        """Background thread: poll gz service --list until the world is ready."""
        create_svc = f'/world/{WORLD}/create'
        while not self._gz_ready:
            try:
                result = subprocess.run(
                    ['gz', 'service', '--list'],
                    capture_output=True, text=True, timeout=5,
                )
                if create_svc in result.stdout:
                    time.sleep(1.0)   # 1 extra second so all services settle
                    self._gz_ready = True
                    print('[GAME] Gazebo ready — episode 1 starting...')
                    return
            except Exception:
                pass
            time.sleep(1.0)

    # ── Subscriptions ──────────────────────────────────────────────────────

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self._drone_x, self._drone_y, self._drone_z = p.x, p.y, p.z
        q = msg.pose.pose.orientation
        # Yaw from quaternion — 0 = facing North (+X world axis)
        self._drone_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    # ── Gazebo service helpers ─────────────────────────────────────────────

    def _gz_spawn(self, sdf_inline: str, x: float, y: float, z: float = 0.0):
        # Pass SDF inline (single-quoted XML → no proto-text escaping needed).
        # subprocess list args bypass the shell, so single quotes in the SDF
        # are passed verbatim to gz without any shell interpretation.
        req = (f'sdf: "{sdf_inline}" '
               f'pose: {{position: {{x: {x:.2f} y: {y:.2f} z: {z:.2f}}}}}')
        subprocess.Popen(
            ['gz', 'service', '-s', f'/world/{WORLD}/create',
             '--reqtype', 'gz.msgs.EntityFactory',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    def _gz_remove(self, name: str):
        # type: 2 = MODEL in gz.msgs.Entity EntityType enum
        subprocess.Popen(
            ['gz', 'service', '-s', f'/world/{WORLD}/remove',
             '--reqtype', 'gz.msgs.Entity',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000',
             '--req', f'name: "{name}" type: 2'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    def _gz_set_pose(self, model: str, x: float, y: float, z: float):
        req = f'name: "{model}" position: {{x: {x:.2f} y: {y:.2f} z: {z:.2f}}}'
        subprocess.Popen(
            ['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
             '--reqtype', 'gz.msgs.Pose',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    # ── Episode management ─────────────────────────────────────────────────

    def _random_pos(self):
        while True:
            x = random.uniform(-SPAWN_MAX, SPAWN_MAX)
            y = random.uniform(-SPAWN_MAX, SPAWN_MAX)
            if math.hypot(x, y) >= SPAWN_MIN:
                return x, y

    def _start_episode(self):
        self._episode += 1
        self._tank_x, self._tank_y = self._random_pos()
        # Only remove if a tank is actually alive in Gazebo — avoids "not found" errors
        if self._tank_spawned:
            self._gz_remove('target_tank')
            self._tank_spawned = False
            time.sleep(0.3)
        self._gz_spawn(_TANK_SDF, self._tank_x, self._tank_y)
        self._tank_spawned = True
        self._ep_start = time.monotonic()
        self._state = 'HUNTING'

        dist    = math.hypot(self._tank_x, self._tank_y)
        bearing = _compass(self._tank_x, self._tank_y)
        print(f'\n[EPISODE {self._episode}]  Tank ~{dist:.0f} m {bearing}')
        print('Take off with T, find the tank, dive into it!')

    # ── Main tick (10 Hz) ─────────────────────────────────────────────────

    def _tick(self):
        if self._state == 'WAITING':
            if self._gz_ready:
                self._start_episode()
            self._publish_map(dist=0.0, elapsed=0.0)
            return

        if self._state == 'HIT':
            self._publish_map(dist=0.0, elapsed=time.monotonic() - self._ep_start)
            return

        dist    = math.sqrt(
            (self._drone_x - self._tank_x) ** 2 +
            (self._drone_y - self._tank_y) ** 2 +
            (self._drone_z - 1.0) ** 2
        )
        elapsed = time.monotonic() - self._ep_start

        sys.stdout.write(
            f'\r  [Ep {self._episode}] {int(elapsed)//60:02d}:{int(elapsed)%60:02d}'
            f' | Dist: {dist:5.1f} m | {_heat(dist):<16}'
        )
        sys.stdout.flush()

        self._publish_map(dist=dist, elapsed=elapsed)

        if dist < HIT_DISTANCE:
            self._state = 'HIT'
            threading.Thread(
                target=self._explosion_sequence,
                args=(elapsed,),
                daemon=True,
            ).start()

    # ── Map image ─────────────────────────────────────────────────────────

    def _publish_map(self, dist: float, elapsed: float):
        img = self._build_map_image(dist, elapsed)
        msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self._map_pub.publish(msg)

    def _build_map_image(self, dist: float, elapsed: float) -> np.ndarray:
        S = MAP_PX
        # ── Light map background so ALL features have contrast ────────────
        img = np.full((S, S, 3), (110, 170, 75), dtype=np.uint8)  # bright grass

        def wpx(wx, wy):
            """World (North=+X, West=+Y) → (col, row) clamped pixel."""
            h = S / 2.0
            return (int(max(0, min(S - 1, h + (-wy / MAP_RANGE) * h))),
                    int(max(0, min(S - 1, h + (-wx / MAP_RANGE) * h))))

        def rect(wx, wy, hx, hy, fill, border=None, bt=2):
            c0, r0 = wpx(wx + hx, wy + hy)   # NW corner (max North, max West)
            c1, r1 = wpx(wx - hx, wy - hy)   # SE corner
            cmin, cmax = min(c0, c1), max(c0, c1)
            rmin, rmax = min(r0, r1), max(r0, r1)
            if cmax <= cmin: cmax = cmin + 1
            if rmax <= rmin: rmax = rmin + 1
            cv2.rectangle(img, (cmin, rmin), (cmax, rmax), fill, -1)
            if border:
                cv2.rectangle(img, (cmin, rmin), (cmax, rmax), border, bt)

        font = cv2.FONT_HERSHEY_SIMPLEX

        # ── Roads ─────────────────────────────────────────────────────────
        for wx, wy, hx, hy, _ in _WORLD_ROADS:
            rect(wx, wy, hx, hy, (128, 125, 115))          # asphalt
        # Yellow centre-line stripes on both roads
        ns0, _ = wpx(100, 0);  ns1, _ = wpx(-100, 0)
        oc, or_ = wpx(0, 0)
        ew0 = wpx(0, -100)[0]; ew1 = wpx(0, 100)[0]
        cv2.line(img, (oc, ns0[1] if False else or_ - (or_ - wpx(100,0)[1])),
                 (oc, or_ + (wpx(-100,0)[1] - or_)), (60, 215, 220), 1)
        # simpler: just draw the two centre lines
        cv2.line(img, wpx( 100, 0), wpx(-100,  0), (60, 215, 220), 1)  # N-S
        cv2.line(img, wpx(0, -100), wpx(  0, 100), (60, 215, 220), 1)  # E-W

        # ── Trees ─────────────────────────────────────────────────────────
        for tx, ty, cr in _WORLD_TREES:
            pc, pr = wpx(tx, ty)
            # World scale + generous minimum so they're always clearly visible
            r_px = max(14, int(cr / MAP_RANGE * (S // 2)))
            # Shadow
            cv2.circle(img, (pc + 3, pr + 3), r_px, (55, 90, 30), -1)
            # Crown (bright lime green — very different from ground)
            cv2.circle(img, (pc, pr), r_px, (30, 200, 30), -1)
            # Dark green outline so crown pops against lighter ground
            cv2.circle(img, (pc, pr), r_px, (0, 100, 0), 2)
            # Darker centre (trunk visible from above)
            cv2.circle(img, (pc, pr), max(3, r_px // 4), (25, 60, 20), -1)

        # ── Buildings ─────────────────────────────────────────────────────
        # Warm beige/stone colours — clearly different from the green ground
        _BLDG_COLORS = [
            (195, 200, 215),  # office_a   — light cool-grey
            (185, 195, 210),  # office_b
            (220, 225, 235),  # tower      — almost white (glass/concrete)
            (160, 155, 145),  # warehouse  — warm brown-grey
            (200, 205, 220),  # apartment
            (210, 195, 180),  # shop_a     — warm sandstone
            (205, 190, 175),  # shop_b
            (135, 130, 125),  # factory    — dark industrial
        ]
        for (wx, wy, hx, hy, _), fill in zip(_WORLD_BUILDINGS, _BLDG_COLORS):
            dark = tuple(max(0, c - 65) for c in fill)
            rect(wx, wy, hx, hy, fill, dark, bt=2)
            # Tiny roof-line cross so buildings don't look flat
            bc, br = wpx(wx, wy)
            cv2.line(img, (bc - 3, br), (bc + 3, br), dark, 1)
            cv2.line(img, (bc, br - 3), (bc, br + 3), dark, 1)

        # Chimney: dark cylinder viewed from above
        cc, cr = wpx(_CHIMNEY[0], _CHIMNEY[1])
        cv2.circle(img, (cc, cr), 6, (70, 65, 60), -1)
        cv2.circle(img, (cc, cr), 6, (40, 38, 35), 2)

        # ── Spawn-origin marker ───────────────────────────────────────────
        oc, or_ = wpx(0, 0)
        cv2.line(img, (oc - 10, or_), (oc + 10, or_), (255, 255, 255), 1)
        cv2.line(img, (oc, or_ - 10), (oc, or_ + 10), (255, 255, 255), 1)
        cv2.circle(img, (oc, or_), 4, (255, 255, 255), 1)

        # ── Dotted line from drone to tank while hunting ──────────────────
        if self._state == 'HUNTING':
            dc_pre, dr_pre = wpx(self._drone_x, self._drone_y)
            tc_pre, tr_pre = wpx(self._tank_x,  self._tank_y)
            # draw dashes
            steps = 18
            for i in range(steps):
                if i % 2 == 0:
                    t0 = i / steps;       t1 = (i + 0.5) / steps
                    p0 = (int(dc_pre + t0 * (tc_pre - dc_pre)),
                          int(dr_pre + t0 * (tr_pre - dr_pre)))
                    p1 = (int(dc_pre + t1 * (tc_pre - dc_pre)),
                          int(dr_pre + t1 * (tr_pre - dr_pre)))
                    cv2.line(img, p0, p1, (0, 0, 220), 1)

        # ── Tank ──────────────────────────────────────────────────────────
        if self._state in ('HUNTING', 'HIT'):
            tc, tr = wpx(self._tank_x, self._tank_y)
            if self._state == 'HIT':
                # Animated-style explosion rings
                for r, c in [(38, (20, 40, 200)), (26, (40, 110, 240)),
                              (16, (100, 200, 255))]:
                    cv2.circle(img, (tc, tr), r, c, -1)
                cv2.putText(img, 'BOOM!', (tc - 30, tr - 44),
                            font, 0.75, (0, 230, 255), 2)
            else:
                # Fixed display size (30×18 px) so it's always readable
                HW, HH = 30, 10   # half-width, half-height in pixels
                cv2.rectangle(img, (tc - HW, tr - HH), (tc + HW, tr + HH),
                              (20, 60, 10), -1)                   # dark olive hull
                cv2.rectangle(img, (tc - HW, tr - HH), (tc + HW, tr + HH),
                              (60, 230, 60), 3)                   # bright green border
                # Turret
                cv2.circle(img, (tc, tr), 9, (15, 90, 15), -1)
                cv2.circle(img, (tc, tr), 9, (60, 230, 60), 2)
                # Barrel — always points North (+X = up in image)
                bc, br = wpx(self._tank_x + 14, self._tank_y)
                cv2.line(img, (tc, tr), (bc, br), (80, 255, 80), 3)
                # Label above
                cv2.putText(img, 'TANK', (tc - 22, tr - HH - 7),
                            font, 0.55, (60, 255, 60), 2)

        # ── Drone — large arrow triangle pointing in heading direction ─────
        dc, dr = wpx(self._drone_x, self._drone_y)
        yaw = self._drone_yaw
        hc = -math.sin(yaw)   # heading col component (North=up → sin inverted)
        hr = -math.cos(yaw)   # heading row component
        rc =  hr               # right-perp col
        rr = -hc               # right-perp row
        L, W, B = 24, 13, 10  # nose length, wing half-width, tail setback (px)
        nose  = (int(dc + L * hc),          int(dr + L * hr))
        lwing = (int(dc - W * rc - B * hc), int(dr - W * rr - B * hr))
        rwing = (int(dc + W * rc - B * hc), int(dr + W * rr - B * hr))
        pts = np.array([nose, lwing, rwing], dtype=np.int32)
        # Draw thick black shadow/border first, then bright fill on top
        cv2.fillPoly(img, [pts], (0, 0, 0))
        # Expand pts outward by 3 px for fat border effect: just use polylines
        cv2.polylines(img, [pts.reshape(-1, 1, 2)], True, (0, 0, 0), 5)
        cv2.fillPoly(img, [pts], (0, 245, 255))              # bright yellow-cyan
        cv2.polylines(img, [pts.reshape(-1, 1, 2)], True, (0, 0, 0), 2)
        cv2.circle(img, (dc, dr), 4, (0, 0, 0), -1)          # pivot dot
        cv2.putText(img, 'YOU', (dc + 28, dr + 6),
                    font, 0.55, (0, 245, 255), 2)

        # ── Compass labels (black shadow + white) ─────────────────────────
        for txt, pos in [('N', (S//2 - 8,  46)), ('S', (S//2 - 8, S - 8)),
                         ('W', (6, S//2 + 8)),    ('E', (S - 26, S//2 + 8))]:
            cv2.putText(img, txt, (pos[0] + 1, pos[1] + 1), font, 0.7, (0,   0,  0), 2)
            cv2.putText(img, txt, pos,                       font, 0.7, (255,255,255), 2)

        # ── Mini legend (bottom-left corner) ─────────────────────────────
        lx, ly = 6, S - 60
        cv2.rectangle(img, (lx - 2, ly - 14), (lx + 110, ly + 46), (20,20,20), -1)
        cv2.rectangle(img, (lx - 2, ly - 14), (lx + 110, ly + 46), (80,80,80), 1)
        # tree swatch
        cv2.circle(img, (lx + 7, ly), 7, (30, 200, 30), -1)
        cv2.putText(img, 'Tree', (lx + 18, ly + 5), font, 0.4, (200, 200, 200), 1)
        # building swatch
        cv2.rectangle(img, (lx + 3, ly + 18), (lx + 14, ly + 30), (195, 200, 215), -1)
        cv2.putText(img, 'Building', (lx + 18, ly + 30), font, 0.4, (200,200,200), 1)

        # ── HUD bar ───────────────────────────────────────────────────────
        cv2.rectangle(img, (0, 0), (S, 32), (20, 20, 20), -1)
        m, s  = divmod(int(elapsed), 60)
        kills = len(self._scores)
        status = _heat(dist).strip() if self._state == 'HUNTING' else self._state
        hud = (f'Ep {self._episode}  {m:02d}:{s:02d}'
               f'  Dist:{dist:5.1f}m  {status}  Kills:{kills}')
        cv2.putText(img, hud, (6, 22), font, 0.52, (220, 220, 220), 1)
        cv2.rectangle(img, (0, 0), (S - 1, S - 1), (160, 160, 160), 2)
        return img

    # ── Explosion sequence (background thread) ────────────────────────────

    def _explosion_sequence(self, elapsed: float):
        tx, ty = self._tank_x, self._tank_y
        self._scores.append(elapsed)

        m, s  = divmod(int(elapsed), 60)
        kills = len(self._scores)
        times = '  '.join(f'{int(t)//60:02d}:{int(t)%60:02d}' for t in self._scores)

        self._gz_spawn(_FIREBALL_SDF, tx, ty, 1.5)

        print('\n')
        print(r'  ██████╗  ██████╗  ██████╗ ███╗   ███╗ ██╗')
        print(r'  ██╔══██╗██╔═══██╗██╔═══██╗████╗ ████║ ██║')
        print(r'  ██████╔╝██║   ██║██║   ██║██╔████╔██║ ██║')
        print(r'  ██╔══██╗██║   ██║██║   ██║██║╚██╔╝██║ ╚═╝')
        print(r'  ██████╔╝╚██████╔╝╚██████╔╝██║ ╚═╝ ██║ ██╗')
        print(r'  ╚═════╝  ╚═════╝  ╚═════╝ ╚═╝     ╚═╝ ╚═╝')
        print(f'\n  TANK DESTROYED!  Episode {self._episode}  —  {m:02d}:{s:02d}')
        print(f'  Kills: {kills}   Times: {times}')

        time.sleep(FIREBALL_SECS)

        self._gz_remove('explosion_fireball')
        self._gz_spawn(_SMOKE_SDF, tx, ty, 1.5)
        self._gz_remove('target_tank')
        self._tank_spawned = False

        time.sleep(SMOKE_SECS)
        self._gz_remove('explosion_smoke')

        self._gz_set_pose('x3', 0.0, 0.0, 0.30)
        time.sleep(0.8)

        self._start_episode()


# ── Entry point ───────────────────────────────────────────────────────────────

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
