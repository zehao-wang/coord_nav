#!/bin/bash
# Per-wheel motor test + processed-lidar readout.
# Activates the ROS env (roscar), connects to the car, runs wheel_test.py.
#
#   SAFETY: elevate the car (wheels off the ground) before running.
#
# Usage:  bash wheel_test.sh
set +u
source /home/zwa0839/Documents/Projects_jetson/car_env.sh >/dev/null 2>&1
set -u
exec python3 /home/zwa0839/Documents/Projects_jetson/smoke/wheel_test.py
