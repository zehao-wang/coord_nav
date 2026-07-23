#!/bin/bash
# Smoke test: verify workstation can reach the car (Jetson Nano) over the
# coord_nav hotspot and pull live ROS1 data.
#
# Usage:  bash smoke_test.sh
set -u

CAR_IP=10.42.0.187
CAR_HOST=jetson-desktop
SELF_IP=10.42.0.1
MASTER=http://${CAR_IP}:11311

pass=0; fail=0
ok()   { echo -e "  [ OK ] $1"; pass=$((pass+1)); }
bad()  { echo -e "  [FAIL] $1"; fail=$((fail+1)); }
hdr()  { echo; echo "== $1 =="; }

# --- 1. network layer ---------------------------------------------------
hdr "1. Network reachability"
if ping -c1 -W2 "$CAR_IP" >/dev/null 2>&1; then
  ok "ping $CAR_IP"
else
  bad "ping $CAR_IP  (car offline / not on coord_nav?)"
fi

if getent hosts "$CAR_HOST" | grep -q "$CAR_IP"; then
  ok "/etc/hosts: $CAR_HOST -> $CAR_IP"
else
  bad "/etc/hosts missing '$CAR_IP $CAR_HOST'  (ROS data flow will break)"
fi

if timeout 3 bash -c "cat < /dev/null > /dev/tcp/${CAR_IP}/11311" 2>/dev/null; then
  ok "roscore port 11311 reachable"
else
  bad "roscore port 11311 unreachable  (roscore not running on car?)"
fi

# --- 2. ROS environment -------------------------------------------------
hdr "2. ROS environment"
# conda/ROS activation scripts reference unbound vars -> disable nounset here
set +u
source /home/zwa0839/miniconda3/etc/profile.d/conda.sh 2>/dev/null
conda activate ros1 2>/dev/null
set -u
export ROS_MASTER_URI="$MASTER"
export ROS_IP="$SELF_IP"

if command -v rostopic >/dev/null 2>&1; then
  ok "ROS Noetic env active ($(rosversion -d 2>/dev/null))"
else
  bad "rostopic not found  (conda env 'ros1' broken)"
  echo; echo "RESULT: $pass passed, $fail failed"; exit 1
fi

# --- 3. master / topics -------------------------------------------------
hdr "3. Topics from car master"
TOPICS=$(timeout 8 rostopic list 2>/dev/null)
if [ -n "$TOPICS" ]; then
  ok "rostopic list ($(echo "$TOPICS" | wc -l) topics)"
  echo "$TOPICS" | sed 's/^/        /'
else
  bad "rostopic list returned nothing"
fi

# --- 4. live data flow (the real test) ----------------------------------
hdr "4. Live data (subscriber connects back -> checks ROS_IP)"
check_topic() {
  local t="$1"
  if timeout 8 rostopic echo -n1 "$t" >/dev/null 2>&1; then
    ok "received a message on $t"
  else
    bad "no data on $t within 8s"
  fi
}
check_topic /odom
check_topic /scan

# --- summary ------------------------------------------------------------
hdr "RESULT"
echo "  $pass passed, $fail failed"
[ "$fail" -eq 0 ] && echo "  ALL GOOD — workstation can read the car." || echo "  Some checks failed — see above."
exit "$fail"
