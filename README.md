# Drone Defense Duel — Capstone Project

An autonomous aerial interception system built with ROS 2 Jazzy and Gazebo Harmonic.

**The duel:** you pilot an X3 multirotor as a **kamikaze drone** trying to dive into a tank. An autonomous fixed-wing **interceptor** (running classical guidance on a Raspberry Pi in Hardware-in-the-Loop) launches to intercept and neutralize you before you reach it.

---

## Demo

https://github.com/3rif005-commits/capstone/raw/master/docs/demo.mp4

---

## System Overview

| Entity | Role | Technology |
|--------|------|------------|
| **Tank** | Static objective — the defended asset | Mission objective |
| **Kamikaze (X3 multirotor)** | Player-flown threat, agile, can hover | Keyboard teleop, ROS 2 |
| **Interceptor (fixed-wing)** | Autonomous defender running on RPi | Augmented PN guidance, HIL |

**Guidance laws implemented:** Pure Pursuit · Proportional Navigation · Augmented PN (default).

---

## Quick Start

See **[RUN.md](RUN.md)** for the full setup and run guide.

```bash
# 1. Network setup (once per machine)
cp .env.example .env && nano .env   # set RPI_IP / PC_IP

# 2. Build
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vtol_sim

# 3. Deploy interceptor to RPi
./scripts/deploy_rpi.sh --start

# 4. Launch (Terminal 1)
source install/setup.bash && export $(grep -v '#' .env | xargs)
ros2 launch vtol_sim vtol_sim_hil.launch.py

# 5. Fly (Terminal 2)
ros2 run vtol_sim keyboard_teleop
```

**Controls:** `T` take off · `Z`/`S` throttle · `Q`/`D` yaw · arrows pitch/strafe · Space hover.

---

## Repo Layout

```
src/vtol_sim/
├── interception/        # Guidance laws (Pure Pursuit, PN, APN)
├── machinima/           # Cinematic demo director & camera primitives
├── game_manager.py      # Win/loss detection, HUD mini-map
├── interceptor_bridge.py# UDP bridge PC ↔ RPi
└── keyboard_teleop.py   # Player controls
docs/
├── machinima-scenario.md# 3-act cinematic storyboard
└── demo.mp4             # Gameplay demo
RUN.md                   # Full run / tune / debug guide
```

---

## Version History

| Branch / Tag | Description |
|---|---|
| `master` | v2 — autonomous HIL fixed-wing interceptor |
| `v1-vtol-quadcopter` / `v1.0-quadcopter-game` | v1 — solo kamikaze hunt (no interceptor) |
