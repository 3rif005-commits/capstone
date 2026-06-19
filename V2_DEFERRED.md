# v2 — What We Dropped / Deferred (and how to recover it)

This version (v2) is the **autonomous fixed-wing interceptor vs. player-flown
kamikaze** duel, using **classical guidance only**. To get a working, playable,
real-time system we deliberately dropped, deferred, or simplified several things
from the original capstone proposal (`d513...capstone-drone-interception.pdf`)
and from a "maximal" design. This file records each one so it can be recovered
later.

> v1 (the original solo X3 quadcopter kamikaze-hunt game) is fully preserved at
> git **branch `v1-vtol-quadcopter`** and **tag `v1.0-quadcopter-game`**.
> `git checkout v1-vtol-quadcopter` to return to it.

Legend: **What** it is · **Why** dropped · **Recover** how to bring it back.

---

## 1. Reinforcement Learning guidance  ← the biggest intentional drop
- **What:** The PDF proposes an RL guidance agent (PPO / DQN / SAC) trained in
  sim, compared against the classical laws. We implemented **only classical
  guidance** (Pure Pursuit, True PN, Augmented PN).
- **Why:** User decision — keep v2 tractable and explainable; "do better than
  the PDF" with strong classical guidance first.
- **Recover:** The guidance interface is already RL-ready. In
  `src/vtol_sim/vtol_sim/interception/guidance.py` every law is a class with
  `command(p_i, v_i, p_t, v_t, a_t) -> accel`. Add an `RLGuidance(GuidanceLaw)`
  that wraps a trained policy (observation = the `Engagement` geometry:
  relative pos/vel, LOS rate, closing speed; action = lateral-accel command),
  register it in the `LAWS` dict, and select it with the `guidance_law` param.
  Train offline against `engagement_sim.py` (it already is a fast, headless
  environment) — wrap it as a Gym env. Compare with the classical laws using the
  same metrics table.

## 2. Vision / sensor-fusion state estimation
- **What:** PDF Layer 1 lists GPS, vision-based, and GPS+Vision+Lidar fusion.
  We use **GPS-style only**: the interceptor reads the kamikaze's odometry and
  estimates its world velocity/accel by finite-differencing position
  (`interceptor_node._on_target_odom`).
- **Why:** Vision tracking of a non-cooperative target is a separate research
  problem; odometry gets the guidance working now.
- **Recover:** The X3 already carries a forward camera (`/X3/camera/image_raw`,
  bridged). Add a detector/tracker node that outputs the target's relative
  bearing/range and feed that into the interceptor instead of (or fused with)
  the odometry. The guidance layer is unchanged — it only needs a target
  position/velocity estimate.

## 3. Aerodynamic fixed-wing fidelity (Layer 3 flight control)
- **What:** The interceptor is a **kinematic point-mass** (coordinated-turn,
  `interception/fixed_wing.py`), pose-driven into Gazebo via `set_pose`. We did
  NOT model real aerodynamics (lift/drag) or low-level roll/pitch/heading/
  airspeed autopilot loops (PDF Layer 3).
- **Why:** A kinematic model matches PDF Objective 1, validates guidance fast,
  and ports cleanly to embedded HW. Full aero is heavy and slow to tune.
- **Recover:** Replace the `set_pose` driver with a real fixed-wing SDF that has
  the gz aerodynamics plugin + thruster/control surfaces, and add inner-loop
  controllers (roll/pitch/heading/airspeed) that track the guidance command.
  The guidance/avoidance core stays the same — only the platform layer changes.
  (`interceptor_node` already isolates the platform behind `FixedWing.step`.)

## 4. Hardware-in-the-Loop / embedded deployment
- **What:** PDF: deploy the guidance on embedded HW (Pixhawk, Jetson/RPi,
  STM32, PX4 SITL) and validate via HIL. We run **everything in software** on
  the PC.
- **Why:** User decision — validate the scenario in sim first; user has a
  Raspberry Pi 3 B+ for later.
- **Recover:** The guidance core (`interception/`) is pure Python with no ROS/
  Gazebo deps, so it runs unchanged on the Pi. Path: export the guidance loop,
  run it on the Pi, connect a Pixhawk/PX4 SITL as the flight controller, and
  bridge the sim ↔ HW. Start with PX4 SITL + Gazebo, then real Pixhawk.

## 5. Faster / more capable kamikaze (controller retune)
- **What:** The player's kamikaze is **v1's X3 multirotor at 3 m/s**
  (`keyboard_teleop.py`, restored byte-for-byte from v1). We tried bumping it to
  7–9 m/s and it **flipped into the ground** — the `MulticopterVelocityControl`
  gains in `worlds/vtol_world.sdf` are tuned for ~3 m/s.
- **Why:** Stable control > speed. Do not edit the teleop to go faster.
- **Recover:** Retune the **controller**, not the teleop: in
  `worlds/vtol_world.sdf` raise `maximumLinearVelocity` / `maximumLinearAcceleration`
  and re-tune `velocityGain` / `attitudeGain` / `angularRateGain` together so the
  X3 stays stable at higher speed, then raise `LINEAR_SPEED` in the teleop to
  match. Test incrementally (e.g. 4 → 5 → 6 m/s).

## 6. v2 performance reductions (recover on better hardware)
- **What:** To keep Gazebo's real-time factor near 1.0 on this PC we lowered:
  interceptor control/pose rate **50 → 30 Hz** (`interceptor_node.RATE_HZ`),
  minimap publish **10 → 5 Hz** (`game_manager._publish_map`). The nose-camera
  window is still on (offered to drop for more RTF).
- **Why:** Full-run RTF was 0.4–0.7 (half-speed), making flight feel sluggish.
- **Recover:** On a faster machine, raise `RATE_HZ` back to 50 and the minimap
  throttle back to every tick. To reclaim more RTF cheaply, drop the nose camera
  + its viewer from `launch/vtol_sim.launch.py` (`camera_bridge`, `rqt_image_view`)
  — the minimap is the primary display.

## 7. Collision-aware engagement (avoidance under-exercised)
- **What:** The interceptor has working city obstacle-avoidance
  (`interception/avoidance.py`, validated offline: it clears buildings by >20 m
  on paths that otherwise crash). But in the live duel, interception usually
  happens mid-course **before** the city, so avoidance rarely triggers.
- **Why:** Engagement geometry — not a bug.
- **Recover:** Add scenarios that force low, in-city engagements (kamikaze
  hugging the buildings; interceptor spawns such that its path crosses the
  cluster), and let the kamikaze use buildings as cover. Tune
  `ObstacleField(lookahead, safety, gain)`.

## 8. Harder offline scenarios (APN-vs-PN differentiation)
- **What:** `engagement_sim.py` currently shows **all laws at ~100%** at the
  chosen scale, so APN's advantage over PN isn't visible (only control-effort
  differs). 
- **Why:** The default scenarios are too easy.
- **Recover:** Add faster / more evasive / shorter-reaction-time scenarios
  (higher `weave_amp`/`weave_freq`, faster kamikaze, closer spawns, lower
  interceptor speed margin) so PN starts to miss and APN's target-accel term
  wins — that's the headline comparison for the thesis.

---

### Quick map of where things live
| Concern | File |
|---|---|
| Guidance laws (PN/APN/pursuit) | `interception/guidance.py` |
| Fixed-wing kinematics | `interception/fixed_wing.py` |
| Obstacle avoidance + city | `interception/avoidance.py`, `interception/world.py` |
| Offline study / metrics | `interception/engagement_sim.py` |
| Interceptor ROS node | `interceptor_node.py` |
| Game referee / duel / minimap | `game_manager.py` |
| Player kamikaze control (= v1) | `keyboard_teleop.py` |
| Multirotor controller gains | `worlds/vtol_world.sdf` |
