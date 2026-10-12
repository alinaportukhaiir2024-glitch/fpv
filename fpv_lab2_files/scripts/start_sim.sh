#!/bin/bash
set -e
SCRIPTS="$(cd "$(dirname "$0")" && pwd)"
"$SCRIPTS/stop_sim.sh"
source "$SCRIPTS/setup_env.sh"

export LIBGL_ALWAYS_SOFTWARE=1

RUN_DIR=$HOME/fpv_labs/run
mkdir -p "$RUN_DIR"
cd "$RUN_DIR"

echo "Запуск Gazebo + ArduPilot SITL..."
exec ros2 launch ardupilot_gz_bringup iris_maze.launch.py "$@"
