#!/usr/bin/env bash
# Quick-start for Terminal 1: build (optional) + source + launch.
#
# Usage:
#   ./run_sim.sh           # launch with existing build
#   ./run_sim.sh --build   # rebuild first, then launch
#
# You still need two more terminals:
#   Terminal 2:  ros2 run vtol_sim game_manager
#   Terminal 3:  ros2 run vtol_sim keyboard_teleop
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

source /opt/ros/jazzy/setup.bash

if [[ "${1:-}" == "--build" || ! -d install ]]; then
    echo ">> Building vtol_sim..."
    colcon build --packages-select vtol_sim
fi

source install/setup.bash

echo ""
echo "========================================================"
echo "  VTOL Kamikaze Drone Hunt"
echo "========================================================"
echo "  Terminal 2 (game manager):"
echo "    ros2 run vtol_sim game_manager"
echo ""
echo "  Terminal 3 (keyboard control):"
echo "    ros2 run vtol_sim keyboard_teleop"
echo "========================================================"
echo ""

ros2 launch vtol_sim vtol_sim.launch.py
