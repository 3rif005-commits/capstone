# VTOL Drone Simulation — How to Run

ROS 2 **Jazzy** + Gazebo **Harmonic** (gz-sim 8). X3 quadcopter flown with a
velocity-controller plugin and a keyboard teleop.

## 1. Build (first time, or after editing the code)

```bash
cd ~/projects/capstone
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vtol_sim
```

## 2. Run

Open **two terminals**. Source the workspace in **both**:

```bash
cd ~/projects/capstone
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**Terminal 1 — simulator + bridge + camera view:**
```bash
ros2 launch vtol_sim vtol_sim.launch.py
```

**Terminal 2 — keyboard control:**
```bash
ros2 run vtol_sim keyboard_teleop
```

> Tip: `./run_sim.sh` does the build + sourcing + launch for Terminal 1 in one step.
> You still start the teleop yourself in Terminal 2 (it needs an interactive
> terminal for the keyboard).

## 3. Flying

| Key | Action |
|-----|--------|
| `T` | Auto-takeoff to ~5 m |
| `Z` / `S` | Throttle up / down |
| `Q` / `D` | Yaw left / right |
| Arrow ↑ / ↓ | Pitch forward / backward |
| Arrow ← / → | Strafe left / right |
| Space | Hover (stop motion) |
| Esc / Ctrl-C | Quit |

The teleop holds altitude and position when you are not commanding motion, so the
drone stays put instead of drifting.

## 4. Tuning knobs (top of `src/vtol_sim/vtol_sim/keyboard_teleop.py`)

| Constant | Meaning |
|----------|---------|
| `LINEAR_SPEED` (3.0) | Top translate speed. Lower it (e.g. 2.0) for a crisper stop / less coast. |
| `ANGULAR_SPEED` (0.7) | Yaw rate. Higher turns faster but adds altitude dip / drift. |
| `ALT_HOLD_KP` (1.2) | Altitude-hold stiffness. |
| `POS_HOLD_KP` / `POS_HOLD_KD` (1.0 / 0.8) | Position-hold stiffness / damping. |

Controller gains live in `src/vtol_sim/worlds/vtol_world.sdf`
(`MulticopterVelocityControl` plugin). Rebuild after any edit (step 1).

## Useful checks

```bash
# Is orientation/altitude feedback flowing? (teleop needs this)
ros2 topic echo /model/x3/odometry --field pose.pose.position
```
