#!/bin/bash
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash 2>/dev/null || true
source ~/fpv_labs/install/setup.bash

DX=${1:-5.2}
DY=${2:-0.2}

echo "Запуск польоту do точки: dx=$DX, dy=$DY"
ros2 run fpv_lab2 goto_point --ros-args -p delta_x:=$DX -p delta_y:=$DY
