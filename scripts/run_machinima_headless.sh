#!/bin/bash
# Headless one-shot: render + record the machinima, then retime the clip to
# real-time (headless renders < 30 fps, so the raw file plays fast). Writes a
# final real-time mp4 next to the raw one. All logs under /tmp.
set -e
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
pkill -9 -f machinima_director 2>/dev/null || true
pkill -9 -f machinima_recorder 2>/dev/null || true
sleep 3
rm -f "$WS"/src/vtol_sim/media/*.mp4

gz sim -s -r "$WORLD" >/tmp/gz_run.log 2>&1 & GZ=$!
sleep 10
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$CFG >/tmp/svc.log 2>&1 & SVC=$!
ros2 run ros_gz_image image_bridge /cine_cam/image >/tmp/img.log 2>&1 & IMG=$!
sleep 5
cd "$WS/src/vtol_sim"
ros2 run vtol_sim machinima_recorder >/tmp/rec.log 2>&1 & REC=$!
sleep 3
echo "[run] director starting..."
ros2 run vtol_sim machinima_director >/tmp/dir.log 2>&1 || true
echo "[run] director done."
sleep 3
kill $REC $IMG $SVC $GZ 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ruby.*gz" 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
pkill -9 -f image_bridge 2>/dev/null || true
sleep 2

RAW=$(ls -t "$WS"/src/vtol_sim/media/*.mp4 2>/dev/null | head -1)
echo "[run] raw clip: $RAW"
N=$(grep -oP '\(\K[0-9]+(?= frames)' /tmp/rec.log | head -1 || echo "")
echo "[run] recorder frames: $N"

# Retime to real time: record window is the ~50.5s scripted timeline.
if [ -n "$RAW" ] && [ -n "$N" ] && command -v ffmpeg >/dev/null 2>&1; then
  OUT="${RAW%.mp4}_realtime.mp4"
  # slow factor so N frames span 50.5s at 30fps output
  FACTOR=$(awk "BEGIN{printf \"%.4f\", 30.0*50.5/$N}")
  echo "[run] retiming x$FACTOR -> $OUT"
  ffmpeg -y -loglevel error -i "$RAW" -vf "setpts=${FACTOR}*PTS" -r 30 -an "$OUT" \
    && echo "[run] realtime clip: $OUT"
fi
echo "PIPELINE-DONE"
