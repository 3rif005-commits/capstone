# Drone Defense Duel (v2) — How to Run

ROS 2 **Jazzy** + Gazebo **Harmonic** (gz-sim 8).

**The duel:** YOU pilot the X3 multirotor as a **kamikaze**, diving it into the
**tank**. An autonomous fixed-wing **interceptor** (classical guidance — Pure
Pursuit / PN / Augmented PN, *no RL*) runs on the **Raspberry Pi** (HIL) and
tries to reach you first to defend the tank.

> v1 (the original solo X3 kamikaze-hunt game) is preserved at git branch
> `v1-vtol-quadcopter` / tag `v1.0-quadcopter-game`. `git checkout v1-vtol-quadcopter`
> to return to it.

---

## 0. First-time network setup (do once per machine)

IP addresses are **not** committed to git. Copy the example file and fill in
your local IPs:

```bash
cp .env.example .env
nano .env          # set RPI_IP and optionally PC_IP
```

Then load the variables into your shell (repeat this in every new terminal
that needs them, or add it to your `~/.bashrc`):

```bash
export $(grep -v '#' .env | xargs)
```

---

## 1. Build (first time, or after editing any source file)

```bash
cd ~/projects/capstone
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vtol_sim
```

---

## 2. Deploy interceptor to RPi (first time, or after editing guidance code)

```bash
./scripts/deploy_rpi.sh --start
```

This copies `interception/` + `interceptor_runner.py` to `~/interceptor/` on the
RPi and starts the runner in the background (logs to `/tmp/interceptor_runner.log`).

To restart the runner manually on the RPi:

```bash
ssh -i ~/.ssh/rpi_key ${RPI_USER:-ayoub}@$RPI_IP \
  "pkill -f interceptor_runner.py; cd ~/interceptor && \
   nohup python3 -u interceptor_runner.py --pc-ip $PC_IP --law apn \
   > /tmp/interceptor_runner.log 2>&1 &"
```

Check it started:

```bash
ssh -i ~/.ssh/rpi_key ${RPI_USER:-ayoub}@$RPI_IP "cat /tmp/interceptor_runner.log"
```

---

## 3. Run — two terminals

In **each terminal**:

```bash
cd ~/projects/capstone
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export $(grep -v '#' .env | xargs)   # load RPI_IP / PC_IP
```

### Terminal 1 — Gazebo + bridges + interceptor bridge + game manager

```bash
ros2 launch vtol_sim vtol_sim_hil.launch.py
```

Opens the Gazebo world, all ROS↔Gz bridges, the **interceptor bridge** (talks to
the RPi over UDP), the **game manager**, the nose-camera window, and the
tactical mini-map window.

### Terminal 2 — Your kamikaze controls

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

## 4. Offline guidance study (no Gazebo)

Compare the guidance laws headlessly (success / intercept time / miss distance /
control effort / crash rate):

```bash
ros2 run vtol_sim engagement_sim
# or:  python3 -m vtol_sim.interception.engagement_sim   (from src/vtol_sim)
```

---

## 5. Tuning

| File | What to change |
|------|---------------|
| `vtol_sim/game_manager.py` | `INTERCEPT_DIST`, `TANK_HIT_DIST`, kamikaze spawn ranges, episode timeout |
| `.env` | `RPI_IP`, `PC_IP` (network addresses — never committed) |
| `vtol_sim/interceptor_runner.py` | guidance law defaults, arming delays, terminal speed schedule |
| `vtol_sim/interception/fixed_wing.py` | `FixedWingLimits` (airspeed, bank, climb) |
| `vtol_sim/interception/guidance.py` | PN gain `N`, re-acquire behaviour |
| `vtol_sim/interception/avoidance.py` | `lookahead`, `safety`, avoidance `gain` |
| `vtol_sim/keyboard_teleop.py` | `LINEAR_SPEED`, `ANGULAR_SPEED` (kamikaze agility) |

Rebuild after any PC-side edit: `colcon build --packages-select vtol_sim`.
Re-deploy after any guidance edit: `./scripts/deploy_rpi.sh --start`.

---

## 6. Useful checks

```bash
# Is the RPi runner alive?
ssh -i ~/.ssh/rpi_key ${RPI_USER:-ayoub}@$RPI_IP "cat /tmp/interceptor_runner.log"

# Is the bridge receiving poses from the RPi?
ros2 topic echo /interceptor/odometry --field pose.pose.position

# Is the interceptor armed and chasing?
ros2 topic echo /interceptor/status

# Is the mini-map publishing?
ros2 topic hz /game/minimap

# Is the Gz service bridge up?
ros2 service list | grep set_pose
```
