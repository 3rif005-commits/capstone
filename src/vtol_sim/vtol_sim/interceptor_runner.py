#!/usr/bin/env python3
"""Fixed-wing interceptor guidance runner — executes on the RPi (HIL mode).

Receives raw kamikaze position from the PC bridge via UDP, runs the full
3-layer guidance pipeline locally, and sends the computed pose back.

  Layer 1 — state estimation : finite-difference raw position → velocity/accel
  Layer 2 — guidance         : APN / PN / Pure Pursuit + obstacle avoidance
  Layer 3 — flight control   : FixedWing coordinated-turn kinematic model

No ROS2 or Gazebo dependency — only Python 3 + numpy required.

Usage (on RPi):
  python3 interceptor_runner.py [--pc-ip 192.168.1.72] [--law apn]
"""

import argparse
import json
import math
import os
import socket
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))

from interception.fixed_wing import FixedWing, FixedWingLimits
from interception.guidance import make_law
from interception.avoidance import ObstacleField
from interception.world import city

LISTEN_PORT = 5555  # Pi receives on this port
SEND_PORT   = 5556  # PC bridge listens on this port

DT = 1.0 / 30.0    # nominal tick; actual integration uses packet timestamps

ARM_DISPLACEMENT = 2.5   # m — kamikaze must move this far to trigger the hunt
ARM_GRACE_S      = 1.0   # s — ignore settle jitter right after reset
LAUNCH_DELAY_S   = 2.0   # s — head-start for the player after kamikaze moves
AUTO_ARM_S       = 8.0   # s — fail-safe: launch this long after reset even if
                         #     the kamikaze barely moved, so a round always ends
                         #     in a real chase instead of a silent timeout

VEL_ALPHA = 0.4   # low-pass weight on new velocity measurement
ACC_ALPHA = 0.2   # low-pass weight on new acceleration measurement

# Terminal speed schedule. A fixed-wing's min turn radius is V^2/(g*tan(bank)):
# ~29 m at v_max=28, but only ~5 m at v_min=12. Against a slow multirotor the
# interceptor must SLOW DOWN as it closes, or it orbits at full speed and never
# crosses the kill radius. Full speed beyond TERMINAL_FAR; ramp to stall by
# TERMINAL_NEAR so the final turn is tight enough to curve onto the target.
TERMINAL_FAR  = 45.0   # m — beyond this, fly flat-out to close the gap
TERMINAL_NEAR = 12.0   # m — at/under this, fly at stall for the tightest turn

# Ground avoidance. The kinematic model integrates position freely, so a dive at
# a low target would fly it straight through the terrain. Pull up below
# GROUND_SOFT and never sink past GROUND_FLOOR — low enough to still chase a
# low-flying kamikaze, high enough to never hit the deck.
GROUND_FLOOR = 2.0     # m — hard minimum altitude (never below this)
GROUND_SOFT  = 6.0     # m — start adding climb push below this height
GROUND_GAIN  = 4.0     # m/s^2 per metre below GROUND_SOFT


def parse_args():
    p = argparse.ArgumentParser(description='RPi HIL guidance runner')
    p.add_argument('--pc-ip',       default='192.168.1.72')
    p.add_argument('--law',         default='apn',
                   choices=['apn', 'pn', 'pure_pursuit'])
    p.add_argument('--nav-n',       type=float, default=4.0)
    p.add_argument('--launch-delay',type=float, default=LAUNCH_DELAY_S)
    p.add_argument('--auto-arm',    type=float, default=AUTO_ARM_S)
    return p.parse_args()


def _send_pose(sock, addr, fw: FixedWing, bank: float, armed: bool):
    pkt = {
        'pos':   fw.pos.tolist(),
        'psi':   float(fw.psi),
        'gamma': float(fw.gamma),
        'speed': float(fw.speed),
        'bank':  float(bank),
        'armed': armed,
    }
    sock.sendto(json.dumps(pkt).encode(), addr)


def main():
    args = parse_args()

    law   = make_law(args.law, **({} if args.law == 'pure_pursuit' else {'N': args.nav_n}))
    field = ObstacleField(city())

    # Interceptor kinematic state
    fw:        FixedWing | None = None
    last_bank: float            = 0.0

    # Layer 1 — state estimation
    tgt_prev_pos: np.ndarray | None = None
    tgt_prev_t:   float | None      = None
    tgt_vel = np.zeros(3)
    tgt_acc = np.zeros(3)

    # Arming
    armed      = False
    kam_start: np.ndarray | None = None
    launch_at: float | None      = None
    arm_grace_until              = 0.0
    auto_arm_at: float | None    = None   # fail-safe launch time

    # UDP sockets — one for TX (to PC), one bound for RX (from PC)
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.bind(('', LISTEN_PORT))
    rx_sock.settimeout(0.05)   # 50 ms — never block longer than one tick

    send_addr = (args.pc_ip, SEND_PORT)

    print(f'[RPi] law={args.law.upper()}  N={args.nav_n}  '
          f'listening :{LISTEN_PORT}  →  PC {args.pc_ip}:{SEND_PORT}')

    try:
        while True:
            try:
                data, _ = rx_sock.recvfrom(4096)
            except socket.timeout:
                continue

            try:
                msg = json.loads(data)
            except Exception:
                continue

            mtype = msg.get('type')

            # ── Episode reset ─────────────────────────────────────────────
            if mtype == 'reset':
                spawn_pos = np.array(msg['spawn_pos'])
                spawn_psi = float(msg['spawn_psi'])
                lim = FixedWingLimits()
                fw = FixedWing(pos=spawn_pos.copy(), speed=lim.v_min,
                               psi=spawn_psi, gamma=0.0, limits=lim)
                last_bank = 0.0

                tgt_prev_pos = None
                tgt_prev_t   = None
                tgt_vel      = np.zeros(3)
                tgt_acc      = np.zeros(3)

                armed         = False
                kam_start     = None
                launch_at     = None
                arm_grace_until = time.monotonic() + ARM_GRACE_S
                auto_arm_at     = time.monotonic() + args.auto_arm

                print(f'[RPi] reset  spawn={spawn_pos.round(1)}  '
                      f'psi={math.degrees(spawn_psi):.1f}°')

                _send_pose(tx_sock, send_addr, fw, last_bank, armed=False)
                continue

            # ── Guidance tick ─────────────────────────────────────────────
            if mtype != 'tick' or fw is None:
                continue

            tgt_raw = np.array(msg['tgt_pos'])
            pkt_t   = float(msg['t'])
            now     = time.monotonic()

            # Layer 1: finite-difference position → velocity → acceleration
            if tgt_prev_pos is not None:
                dt_odom = max(pkt_t - tgt_prev_t, 1e-3)
                v_new   = (tgt_raw - tgt_prev_pos) / dt_odom
                a_new   = (v_new - tgt_vel) / dt_odom
                tgt_vel = (1 - VEL_ALPHA) * tgt_vel + VEL_ALPHA * v_new
                tgt_acc = (1 - ACC_ALPHA) * tgt_acc + ACC_ALPHA * a_new
            tgt_prev_pos = tgt_raw.copy()
            tgt_prev_t   = pkt_t

            # Arming logic: launch a short head-start after the kamikaze starts
            # moving — OR after a fail-safe timeout if it barely moves, so every
            # round becomes a real chase instead of the interceptor parking at
            # spawn for 90 s (which read as a "random" timeout restart).
            if not armed:
                if now >= arm_grace_until:
                    if kam_start is None:
                        kam_start = tgt_raw.copy()
                    elif (launch_at is None and
                          np.linalg.norm(tgt_raw - kam_start) > ARM_DISPLACEMENT):
                        launch_at = now + args.launch_delay
                        print(f'[RPi] kamikaze moving — launching in {args.launch_delay:.0f}s')
                # Fail-safe: arm even if the player never moved far enough.
                if (launch_at is None and auto_arm_at is not None
                        and now >= auto_arm_at):
                    launch_at = now + args.launch_delay
                    print(f'[RPi] auto-arm timeout — launching in {args.launch_delay:.0f}s')
                if launch_at is not None and now >= launch_at:
                    armed = True
                    print('[RPi] intercept launched!')

            # Layer 2 + 3: guidance + kinematic integration
            if armed:
                a_cmd = law.command(fw.pos, fw.velocity, tgt_raw, tgt_vel, tgt_acc)
                a_cmd = a_cmd + field.avoid_accel(fw.pos, fw.velocity)

                # Ground avoidance: push up as we approach the floor so a dive at
                # a low target pulls out instead of flying into the terrain.
                if fw.pos[2] < GROUND_SOFT:
                    a_cmd[2] += GROUND_GAIN * (GROUND_SOFT - fw.pos[2])

                # Terminal speed schedule: slow toward stall as we close so the
                # turn radius shrinks enough to curve onto the slow target.
                lim  = fw.limits
                rng  = float(np.linalg.norm(tgt_raw - fw.pos))
                frac = (rng - TERMINAL_NEAR) / (TERMINAL_FAR - TERMINAL_NEAR)
                frac = min(1.0, max(0.0, frac))
                speed_cmd = lim.v_min + (lim.v_max - lim.v_min) * frac

                info  = fw.step(a_cmd, DT, speed_cmd=speed_cmd)

                # Hard floor: never let the kinematic model sink through ground.
                if fw.pos[2] < GROUND_FLOOR:
                    fw.pos[2] = GROUND_FLOOR
                    if fw.gamma < 0.0:
                        fw.gamma = 0.0          # level off once on the deck
                last_bank = info['bank']

            _send_pose(tx_sock, send_addr, fw, last_bank, armed)

    except KeyboardInterrupt:
        print('[RPi] stopped.')


if __name__ == '__main__':
    main()
