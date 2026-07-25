#!/bin/bash
# Launch the PySide6 car console. Sources car_env.sh (ROS Noetic conda env + car
# master), then runs the GUI on the current X display ($DISPLAY).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source "$HERE/../car_env.sh" >/dev/null 2>&1
set -u
exec python3 "$HERE/car_console.py" "$@"
