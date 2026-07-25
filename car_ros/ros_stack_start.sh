#!/bin/bash
# ExecStart wrapper for car-ros.service.
#
# systemd starts us with almost no environment, so ROS has to be sourced here
# rather than relying on ~/.bashrc (which returns early for non-interactive
# shells anyway).

# No `set -u` here on purpose: ROS's own setup scripts read variables that are
# not set yet (ROS_DISTRO among them), so strict mode kills the service before
# it starts.
source /opt/ros/melodic/setup.bash
source /home/jetson/catkin_ws/devel/setup.bash

# udev usually has the symlinks ready before we run, but on a cold boot the USB
# adapters can enumerate after multi-user.target. Starting without them leaves
# the driver retrying against a device that is not there, so wait it out.
for _ in $(seq 1 30); do
    if [ -e /dev/rplidar ] && [ -e /dev/myserial ]; then
        break
    fi
    sleep 1
done

[ -e /dev/rplidar ] || echo "warning: /dev/rplidar missing, starting anyway" >&2
[ -e /dev/myserial ] || echo "warning: /dev/myserial missing, starting anyway" >&2

# --- lidar clean-start ----------------------------------------------------
# A rplidarNode killed mid-scan (every restart) leaves the lidar STREAMING scan
# packets. The next node then sends GET_DEVICE_INFO but reads the leftover scan
# stream instead of the reply, so it STALLS right after "RPLIDAR running" -- no
# /scan until a second restart (the notorious "restart twice" behaviour, root-
# caused from rosout.log 2026-07-24). Send the RPLIDAR STOP command (0xA5 0x25)
# so roslaunch's rplidarNode always opens a quiet device. Idempotent + harmless
# if it is already stopped.
if [ -e /dev/rplidar ]; then
    stty -F /dev/rplidar 115200 raw -echo 2>/dev/null
    printf '\xa5\x25' > /dev/rplidar 2>/dev/null   # RPLIDAR_CMD_STOP
    sleep 0.1
    printf '\xa5\x25' > /dev/rplidar 2>/dev/null   # once more, then let it settle
    sleep 0.6
fi

# NOTE: WiFi power management (which spiked link latency to 3+ s and starved the
# workstation closed loop of fresh /odom) is disabled at the SYSTEM level, not
# here -- NetworkManager: coord_nav profile 802-11-wireless.powersave=2 plus
# /etc/NetworkManager/conf.d/*wifi-powersave*.conf wifi.powersave=2. Keep it off.

# exec so roslaunch becomes the service's main process and gets signals direct.
exec roslaunch car_base viz.launch
