#!/bin/bash
PATTERNS="gz sim|ruby.*gz|arducopter|micro_ros_agent|MicroXRCEAgent|rviz2|mavproxy|parameter_bridge|robot_state_publisher|ros_gz_sim"
pkill -f "$PATTERNS" 2>/dev/null
sleep 1
pkill -9 -f "$PATTERNS" 2>/dev/null
exit 0
