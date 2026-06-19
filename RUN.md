# VTOL Kamikaze Drone Hunt — How to Run

ROS 2 **Jazzy** + Gazebo **Harmonic** (gz-sim 8). X3 quadcopter kamikaze game:
find the randomly-spawned tank and dive into it!

---

## 1. Build (first time, or after editing any source file)

```bash
cd ~/projects/capstone
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vtol_sim
```

---

## 2. Run — three terminals

Open **three terminals**. In **each one** run:

```bash
cd ~/projects/capstone
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Terminal 1 — Gazebo + bridges + map/camera windows

```bash
ros2 launch vtol_sim vtol_sim.launch.py
```

This opens:
- Gazebo 3-D world
- Camera feed window (drone nose camera)
- Mini-map window (bird's-eye tactical map)

### Terminal 2 — Game manager (episodes, tank, hit detection)

```bash
ros2 run vtol_sim game_manager
```

Spawns the tank, detects kamikaze hits, triggers explosions, and drives the
map image. Keep this terminal visible — it prints episode info and distance.

### Terminal 3 — Keyboard control

```bash
ros2 run vtol_sim keyboard_teleop
```

Needs an interactive terminal (reads keypresses). Controls:

| Key | Action |
|-----|--------|
| `T` | Auto-takeoff to ~5 m |
| `Z` / `S` | Throttle up / down |
| `Q` / `D` | Yaw left / right |
| Arrow ↑ / ↓ | Pitch forward / backward |
| Arrow ← / → | Strafe left / right |
| Space | Hover (stop all motion) |
| Esc / Ctrl-C | Quit |

---

## 3. Quick-start shortcut (Terminal 1 only)

`run_sim.sh` handles the build + sourcing + launch in one shot:

```bash
cd ~/projects/capstone
./run_sim.sh          # launch only (uses existing build)
./run_sim.sh --build  # force rebuild first
```

You still need Terminals 2 and 3 started manually.

---

## 4. How to play

1. Launch all three terminals in order (1 → 2 → 3).
2. Press **T** in Terminal 3 to take off.
3. Read the distance and heat indicator in Terminal 2 to locate the tank.
   The **mini-map window** shows the tank (green rectangle + "TANK" label)
   and your drone (cyan arrow pointing in your heading direction).
4. Fly toward the tank and **dive into it** (get within ~3.5 m).
5. Explosion plays, new episode starts automatically.

---

## 5. Tuning

| File | What to change |
|------|---------------|
| `src/vtol_sim/vtol_sim/game_manager.py` | `HIT_DISTANCE`, `SPAWN_MIN/MAX`, explosion timings |
| `src/vtol_sim/vtol_sim/keyboard_teleop.py` | `LINEAR_SPEED`, `ANGULAR_SPEED`, hold gains |
| `src/vtol_sim/worlds/vtol_world.sdf` | Gazebo controller gains (`MulticopterVelocityControl`) |

Rebuild after any edit:
```bash
colcon build --packages-select vtol_sim
```

---

## 6. Useful checks

```bash
# Is odometry flowing?
ros2 topic echo /model/x3/odometry --field pose.pose.position

# Is the game manager publishing the map?
ros2 topic hz /game/minimap
```
