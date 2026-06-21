#!/bin/bash
# Bring up gz + bridges (no director), then run the camera-move probe.
WS=/home/ayoub/projects/capstone
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export PYTHONUNBUFFERED=1
WORLD="$WS/install/vtol_sim/share/vtol_sim/worlds/machinima_world.sdf"
CFG="$WS/install/vtol_sim/share/vtol_sim/config/gz_bridge_services_machinima.yaml"

pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ruby.*gz" 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f image_bridge 2>/dev/null || true
sleep 3
rm -f /tmp/cam_*.png

gz sim -s -r "$WORLD" >/tmp/gz_dbg.log 2>&1 & GZ=$!
sleep 10
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$CFG >/tmp/svc_dbg.log 2>&1 & SVC=$!
ros2 run ros_gz_image image_bridge /cine_cam/image >/tmp/img_dbg.log 2>&1 & IMG=$!
sleep 5

python3 "$WS/scripts/debug_camera.py"

sleep 1
kill $IMG $SVC $GZ 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ruby.*gz" 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f image_bridge 2>/dev/null || true
echo "DEBUG-DONE"
ls -la /tmp/cam_*.png 2>/dev/null
