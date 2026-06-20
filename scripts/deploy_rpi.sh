#!/usr/bin/env bash
# Deploy the interceptor guidance runner to the RPi and start it.
#
# Usage:
#   ./scripts/deploy_rpi.sh [--law apn|pn|pure_pursuit] [--start]
#
# What it does:
#   1. Copies interception/ package + runner to ~/interceptor/ on the RPi
#   2. Installs numpy on the RPi if missing
#   3. If --start is given: starts the runner in the background over SSH

set -euo pipefail

RPI_USER="ayoub"
RPI_HOST="192.168.1.79"
RPI_DIR="~/interceptor"
SSH_KEY="$HOME/.ssh/rpi_key"
LAW="apn"
START=false

for arg in "$@"; do
  case $arg in
    --law=*) LAW="${arg#*=}" ;;
    --start) START=true ;;
  esac
done

PC_IP=$(ip addr show wlp3s0 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d/ -f1)
PC_IP="${PC_IP:-192.168.1.72}"

SRC="$(cd "$(dirname "$0")/.." && pwd)/src/vtol_sim/vtol_sim"

echo "=== Deploying to $RPI_USER@$RPI_HOST:$RPI_DIR ==="
echo "    PC IP for runner: $PC_IP"
echo "    Guidance law:     $LAW"

ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$RPI_USER@$RPI_HOST" "mkdir -p $RPI_DIR"

rsync -az --info=progress2 \
  -e "ssh -i $SSH_KEY -o StrictHostKeyChecking=no" \
  "$SRC/interception/" \
  "$RPI_USER@$RPI_HOST:$RPI_DIR/interception/"

rsync -az --info=progress2 \
  -e "ssh -i $SSH_KEY -o StrictHostKeyChecking=no" \
  "$SRC/interceptor_runner.py" \
  "$RPI_USER@$RPI_HOST:$RPI_DIR/"

echo "=== Ensuring numpy is available on RPi ==="
ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$RPI_USER@$RPI_HOST" \
  "python3 -c 'import numpy' 2>/dev/null || pip3 install --quiet numpy"

echo "=== Deploy complete ==="

if $START; then
  echo "=== Starting runner on RPi (law=$LAW, pc-ip=$PC_IP) ==="
  ssh -i "$SSH_KEY" -o StrictHostKeyChecking=no "$RPI_USER@$RPI_HOST" \
    "pkill -f interceptor_runner.py 2>/dev/null; \
     cd $RPI_DIR && nohup python3 interceptor_runner.py \
       --pc-ip $PC_IP --law $LAW \
       > /tmp/interceptor_runner.log 2>&1 &
     sleep 1 && echo '[RPi] runner PID:' \$(pgrep -f interceptor_runner.py)"
  echo "=== Log: ssh to RPi and: tail -f /tmp/interceptor_runner.log ==="
else
  echo ""
  echo "Next steps:"
  echo "  1. On RPi, start the runner:"
  echo "     ssh -i $SSH_KEY $RPI_USER@$RPI_HOST"
  echo "     cd $RPI_DIR && python3 interceptor_runner.py --pc-ip $PC_IP --law $LAW"
  echo ""
  echo "  2. On PC, launch the sim then start the bridge:"
  echo "     ros2 launch vtol_sim vtol_sim.launch.py"
  echo "     ros2 run vtol_sim interceptor_bridge_node --ros-args -p rpi_ip:=$RPI_HOST"
fi
