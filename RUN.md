# Drone Defense Duel (v2) — How to Run

ROS 2 **Jazzy** + Gazebo **Harmonic** (gz-sim 8).

**The duel:** YOU pilot the X3 multirotor as a **kamikaze**, diving it into the
**tank**. An autonomous fixed-wing **interceptor** (classical guidance — Pure
Pursuit / PN / Augmented PN, *no RL*) spawns in the sky and tries to reach you
first to defend the tank.

> v1 (the original solo X3 kamikaze-hunt game) is preserved at git branch
> `v1-vtol-quadcopter` / tag `v1.0-quadcopter-game`. `git checkout v1-vtol-quadcopter`
> to return to it.

---

## 1. Build (first time, or after editing any source file)

```bash
cd ~/projects/capstone
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vtol_sim
```

---

## 2. Run — three terminals

In **each terminal**:

```bash
cd ~/projects/capstone
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Terminal 1 — Gazebo + bridges + interceptor + windows

```bash
ros2 launch vtol_sim vtol_sim.launch.py
```

Opens the Gazebo world, the ROS↔Gz topic/service bridges, the **autonomous
interceptor node**, the nose-camera window, and the tactical mini-map window.

To pick the interceptor's guidance law (default `apn`):

```bash
ros2 launch vtol_sim vtol_sim.launch.py
# or run the node standalone with a different law:
ros2 run vtol_sim interceptor_node --ros-args -p guidance_law:=pn -p nav_constant:=4.0
#   guidance_law: apn | pn | pure_pursuit
```

### Terminal 2 — Game manager (referee, episodes, scoring)

```bash
ros2 run vtol_sim game_manager
```

Spawns the tank, repositions you as an incoming attacker each episode, decides
the outcome (you reach the tank vs. interceptor reaches you), drives the
mini-map, and logs metrics to `v2_metrics.csv`.

### Terminal 3 — Your kamikaze controls

```bash
ros2 run vtol_sim keyboard_teleop
```

| Key | Action |
|-----|--------|
| `T` | Auto-takeoff / climb |
| `Z` / `S` | Throttle up / down |
| `Q` / `D` | Yaw left / right |
| Arrow ↑ / ↓ | Pitch forward / backward |
| Arrow ← / → | Strafe left / right |
| Space | Hover (stop all motion) |
| Esc / Ctrl-C | Quit |

You start at the **centre on the ground** — press **T** to take off, then fly
out to the **tank** and dive into it. The fixed-wing interceptor **holds station
in the air until you start moving**, then launches to intercept you. The mini-map
shows **YOU** (cyan), the **INTERCEPTOR** (blue), the **TANK** (green), and a
dashed line = the interceptor's lock on you.

---

## 3. Offline guidance study (no Gazebo)

Compare the guidance laws headlessly (success / intercept time / miss distance /
control effort / crash rate) — this is the algorithm-development workbench:

```bash
ros2 run vtol_sim engagement_sim
# or:  python3 -m vtol_sim.interception.engagement_sim   (from src/vtol_sim)
```

---

## 4. Tuning

| File | What to change |
|------|---------------|
| `vtol_sim/game_manager.py` | `INTERCEPT_DIST`, `TANK_HIT_DIST`, kamikaze spawn ranges, episode timeout |
| `vtol_sim/interceptor_node.py` | guidance law / `nav_constant`, spawn ranges |
| `vtol_sim/interception/fixed_wing.py` | `FixedWingLimits` (airspeed, bank, climb) |
| `vtol_sim/interception/guidance.py` | PN gain `N`, re-acquire behaviour |
| `vtol_sim/interception/avoidance.py` | `lookahead`, `safety`, avoidance `gain` |
| `vtol_sim/keyboard_teleop.py` | `LINEAR_SPEED`, `ANGULAR_SPEED` (kamikaze agility) |

Rebuild after any edit: `colcon build --packages-select vtol_sim`.

---

## 5. Useful checks

```bash
ros2 topic echo /interceptor/odometry --field pose.pose.position   # interceptor flying?
ros2 topic echo /interceptor/status                                # law + range readout
ros2 topic hz /game/minimap                                        # map publishing?
ros2 service list | grep set_pose                                  # service bridge up?
```
