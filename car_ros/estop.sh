#!/bin/sh
# HARD emergency stop -- the last line of defence. Deployed at /home/jetson/estop.sh.
#
# The MCU firmware has NO motor timeout: a lost "stop" latches the wheels and the
# ROS layer cannot recover it. On 2026-07-22 velocity-level stops could not halt a
# runaway (they go through the X3 firmware's internal loop); raw set_motor(0) could.
#
# ORDER IS FORCED BY THE SERIAL PORT. /dev/myserial is exclusive and car_base holds
# it, so this must stop car-ros before it can open the port and blast raw zeros.
# That costs time. Measured from the workstation: **1.23 s until the first zero
# reaches the motors** (0.85 s SSH handshake + service stop + python2/serial open);
# the script then keeps blasting zeros for 3 s, so it runs ~4.2 s in total. The
# number that matters is the 1.23 s -- about 0.44 m of travel at 0.36 m/s.
#
# THEREFORE: CarClient.estop() first blasts raw-PWM zeros on /wheel_cmd (milliseconds,
# down the link car_base already has open) and only then fires this script as the
# guaranteed backstop. Do not rely on this script alone for latency.
#
# Leaves the drive stack DOWN. Bring it back with:
#   sh ~/ros.sh --enable ; sh ~/perception.sh start

sudo -n systemctl stop car-ros 2>/dev/null
sleep 0.3

python2 - <<'PY'
import time, sys
sys.path.insert(0, "/home/jetson/Rosmaster-App/py_install_V3.3.1")
from Rosmaster_Lib import Rosmaster
try:
    bot = Rosmaster(com="/dev/myserial")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        bot.set_motor(0, 0, 0, 0)
        bot.set_car_motion(0, 0, 0)
        time.sleep(0.05)
    del bot
    print("estop: raw PWM zero burst done -- motors are OFF")
except Exception as e:
    print("estop: SERIAL FAILED (%s) -- PULL THE POWER" % e)
PY
echo "estop complete. drive stack is DOWN (car-ros stopped)."
