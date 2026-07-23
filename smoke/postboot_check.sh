#!/bin/bash
# Post-reboot verification for the Jetson car. Run from the workstation after
# rebooting the car. READ-ONLY: checks the boot path came up and no old TCP
# services linger. Does NOT move the car (drive test is separate).
#
#   bash smoke/postboot_check.sh
set -u

CAR_IP=10.42.0.187
KEY="$HOME/.ssh/id_ed25519"
SSH="ssh -o BatchMode=yes -o ConnectTimeout=5 -i $KEY jetson@$CAR_IP"
REMOTE_ROS='source /opt/ros/melodic/setup.bash; export ROS_MASTER_URI=http://localhost:11311;'

pass=0; fail=0
ok()  { echo "  [ OK ] $1"; pass=$((pass+1)); }
bad() { echo "  [FAIL] $1"; fail=$((fail+1)); }
hdr() { echo; echo "== $1 =="; }

# --- 1. car reachable (reboot takes time; wait up to ~120s) --------------
hdr "1. Car reachable"
for i in $(seq 1 40); do
  ping -c1 -W2 "$CAR_IP" >/dev/null 2>&1 && break
  sleep 3
done
if ping -c1 -W2 "$CAR_IP" >/dev/null 2>&1; then ok "ping $CAR_IP"; else bad "ping $CAR_IP (car still down?)"; echo; echo "RESULT: $pass passed, $fail failed"; exit 1; fi
for i in $(seq 1 20); do
  $SSH true 2>/dev/null && break
  sleep 3
done
if $SSH 'echo up' >/dev/null 2>&1; then ok "ssh jetson@$CAR_IP"; else bad "ssh (sshd not up yet)"; fi

# --- 2. car-ros service + expected nodes --------------------------------
hdr "2. car-ros auto-started with all nodes"
state=$($SSH 'systemctl is-active car-ros' 2>/dev/null)
[ "$state" = "active" ] && ok "car-ros.service active" || bad "car-ros.service = $state"

# nodes can take ~15s to register after the service starts
nodes=""
for i in $(seq 1 12); do
  nodes=$($SSH "$REMOTE_ROS timeout -s KILL 8 rosnode list 2>/dev/null")
  echo "$nodes" | grep -q obstacle_circles && break
  sleep 3
done
for n in rplidar car_base base_to_laser obstacle_circles drive_action; do
  echo "$nodes" | grep -qx "/$n" && ok "node /$n up" || bad "node /$n MISSING"
done
if echo "$nodes" | grep -qx "/scan_bridge"; then bad "old /scan_bridge still present"; else ok "no /scan_bridge (old TCP removed)"; fi

# --- 3. no old TCP ports -------------------------------------------------
hdr "3. Old TCP services gone"
ports=$($SSH 'ss -tlnp 2>/dev/null' 2>/dev/null)
for p in 9870 9871 8080; do
  echo "$ports" | grep -q ":$p " && bad "port :$p still listening (old service)" || ok "no :$p"
done

# --- 4. /obstacles live (with frame_id) ---------------------------------
hdr "4. /obstacles flowing"
set +u
source /home/zwa0839/Documents/Projects_jetson/car_env.sh >/dev/null 2>&1
set -u
obs=$(timeout -s KILL 40 python3 - <<'PY' 2>/dev/null
import rospy, time
from std_msgs.msg import Float32MultiArray
rospy.init_node("postboot", anonymous=True, disable_signals=True)
# lidar may take a while to spin up after boot: retry for ~30s
f=[]
rospy.Subscriber("/obstacles", Float32MultiArray, lambda m: f.append((m.data[0], (len(m.data)-1)//3, time.time())))
t0=time.time()
while time.time()-t0 < 30 and len(f) < 6:
    time.sleep(0.5)
if len(f) >= 2:
    dur=f[-1][2]-f[0][2]
    hz=(len(f)-1)/dur if dur>0 else 0
    print("OK %d %d %.2f" % (int(f[0][0]), int(f[-1][0]), hz))
else:
    print("NONE")
PY
)
if [ "${obs%% *}" = "OK" ]; then
  read _ fid0 fid1 hz <<< "$obs"
  ok "/obstacles live: frame_id $fid0 -> $fid1 (incrementing), ~${hz} Hz"
else
  bad "/obstacles NOT flowing (rplidar wedge? -> 'sudo systemctl restart car-ros' on car, wait ~20s, retry)"
fi

# --- summary ------------------------------------------------------------
hdr "RESULT"
echo "  $pass passed, $fail failed"
if [ "$fail" -eq 0 ]; then
  echo "  Boot path OK. Ready for the drive test (needs the car elevated)."
else
  echo "  Some checks failed — see above."
fi
exit "$fail"
