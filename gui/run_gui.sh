#!/bin/bash
# Launch the PySide6 car console. Activates the ros1 env (ROS Noetic + rospy)
# and points it at the car's master, same as `roscar`.
set +u
source /home/zwa0839/Documents/Projects_jetson/car_env.sh >/dev/null 2>&1
set -u
exec python3 /home/zwa0839/Documents/Projects_jetson/gui/car_console.py "$@"
