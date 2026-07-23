#!/usr/bin/env bash
# Полный прогон миссии с логированием.
#   ./scripts/run.sh            — цель по умолчанию из mission.yaml
#   ./scripts/run.sh 25 80      — target_x target_y
#
# set -u здесь не используется: setup.bash из ROS обращается
# к неинициализированным переменным и падает при -u.

LOG_DIR="${LOG_DIR:-$HOME/ba_logs}"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/run_$(date +%Y%m%d_%H%M%S).log"

cleanup() {
  pkill -f arducopter; pkill -f mavproxy; pkill -f mavros_node
  pkill -f "gz sim"; pkill -f gz-sim; pkill -f parameter_bridge
  sleep 2
}
trap cleanup EXIT INT TERM
cleanup   # подчищаем хвосты прошлого прогона

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

ARGS=()
[ $# -ge 1 ] && ARGS+=("target_x:=$1")
[ $# -ge 2 ] && ARGS+=("target_y:=$2")

echo "лог: $LOG"
ros2 launch avoidance_sim mission.launch.py "${ARGS[@]}" 2>&1 | tee "$LOG"
