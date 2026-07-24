#!/usr/bin/env bash
# Полный прогон миссии.
#   ./scripts/run.sh            — цель по умолчанию из mission.yaml
#   ./scripts/run.sh 25 80      — target_x target_y
#
# Сцена пересобирается из config/buildings.yaml и пакет переустанавливается:
# launch читает мир из install/, куда файлы копируются, а не линкуются.
# set -u не используется: setup.bash из ROS падает при -u.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$SCRIPT_DIR")"
WS="$(cd "$REPO/../.." && pwd)"

LOG_DIR="${LOG_DIR:-$HOME/ba_logs}"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/run_$(date +%Y%m%d_%H%M%S).log"

cleanup() {
  pkill -f arducopter; pkill -f mavproxy; pkill -f mavros_node
  pkill -f "gz sim"; pkill -f gz-sim; pkill -f parameter_bridge
  sleep 2
}
trap cleanup EXIT INT TERM
cleanup

source /opt/ros/humble/setup.bash

python3 "$SCRIPT_DIR/gen_world.py" || exit 1
( cd "$WS" && colcon build --packages-select avoidance_sim avoidance_planner \
    --symlink-install > /tmp/ba_build.log 2>&1 ) \
  || { echo "сборка упала, см. /tmp/ba_build.log"; exit 1; }

source "$WS/install/setup.bash"

ARGS=()
[ $# -ge 1 ] && ARGS+=("target_x:=$1")
[ $# -ge 2 ] && ARGS+=("target_y:=$2")

BA_RUNTIME="$HOME/.ba_runtime"
mkdir -p "$BA_RUNTIME"
cd "$BA_RUNTIME"   # SITL пишет eeprom.bin и terrain/ в cwd

echo "лог: $LOG"
ros2 launch avoidance_sim mission.launch.py "${ARGS[@]}" 2>&1 | tee "$LOG"
