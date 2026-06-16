#!/usr/bin/env bash
# Build (if needed), source the workspace, and launch the VTOL simulation.
# This covers "Terminal 1". Start the keyboard teleop yourself in a second
# terminal with:
#   source /opt/ros/jazzy/setup.bash && source install/setup.bash
#   ros2 run vtol_sim keyboard_teleop
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

source /opt/ros/jazzy/setup.bash

# Build only if the install space is missing (pass --build to force a rebuild).
if [[ "${1:-}" == "--build" || ! -d install ]]; then
    echo ">> Building vtol_sim..."
    colcon build --packages-select vtol_sim
fi

source install/setup.bash

echo ">> Launching simulation (Ctrl-C to stop)."
echo ">> In another terminal run:  source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run vtol_sim keyboard_teleop"
ros2 launch vtol_sim vtol_sim.launch.py
