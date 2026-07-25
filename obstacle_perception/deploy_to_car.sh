#!/bin/bash
# Deploy the obstacle perception to the car, end to end: UPLOAD -> BUILD ->
# RESTART, so the car immediately runs the new /obstacles computation.
#
#   bash obstacle_perception/deploy_to_car.sh
#
# Syncs BOTH the C++ package AND car_ros/viz.launch -- viz.launch is what launches
# the node, and its <param margin/rate_hz/eps...> OVERRIDE the C++ defaults, so a
# param change only takes effect if viz.launch ships too. The car's viz.launch is
# backed up once to viz.launch.bak. On a build error the car is NOT restarted.
set -euo pipefail

# --- config (override via env if the setup changes) ---
CAR="${CAR_IP:-10.42.0.187}"
KEY="${CAR_SSH_KEY:-$HOME/.ssh/id_ed25519}"
USER=jetson
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"     # this package (source)
REPO_ROOT="$(cd "$PKG_DIR/.." && pwd)"
REMOTE_WS="/home/jetson/catkin_ws"
REMOTE_PKG="$REMOTE_WS/src/obstacle_perception"
LOCAL_VIZ="$REPO_ROOT/car_ros/viz.launch"
REMOTE_VIZ="$REMOTE_WS/src/car_base/launch/viz.launch"
SSH="ssh -o BatchMode=yes -o ConnectTimeout=8 -i $KEY $USER@$CAR"

echo ">> 1/4  reach $CAR (over SSH; the car's WiFi drops ICMP so we don't ping)"
$SSH true 2>/dev/null || { echo "   car unreachable over SSH -- on the hotspot? powered on? key ok?"; exit 1; }

echo ">> 2/4  upload source -> $CAR:$REMOTE_PKG"
if command -v rsync >/dev/null 2>&1 && $SSH 'command -v rsync' >/dev/null 2>&1; then
    rsync -az --delete -e "ssh -o BatchMode=yes -i $KEY" \
        --exclude='.git' --exclude='build' --exclude='devel' \
        "$PKG_DIR/" "$USER@$CAR:$REMOTE_PKG/"
else
    $SSH "mkdir -p $REMOTE_PKG"
    scp -q -i "$KEY" -r "$PKG_DIR/." "$USER@$CAR:$REMOTE_PKG/"
fi
if [ -f "$LOCAL_VIZ" ]; then
    echo "   + sync viz.launch (launches the node; margin/rate params live here)"
    $SSH "cp -n '$REMOTE_VIZ' '${REMOTE_VIZ}.bak' 2>/dev/null; true"   # keep the original as .bak
    scp -q -i "$KEY" "$LOCAL_VIZ" "$USER@$CAR:$REMOTE_VIZ"
fi
# normalise uploaded mtimes to the car's clock (the Nano isn't NTP-synced, so files
# arrive with "future" timestamps -> catkin warns "build may be incomplete")
$SSH "find '$REMOTE_PKG' -exec touch {} + 2>/dev/null; true"

echo ">> 3/4  build on car (catkin_make --pkg obstacle_perception)"
BUILD_LOG=$(mktemp)
if ! $SSH "source /opt/ros/melodic/setup.bash && cd $REMOTE_WS && catkin_make --pkg obstacle_perception" 2>&1 | tee "$BUILD_LOG"; then
    echo ""
    echo "   ✗ BUILD FAILED. Errors:"
    grep -iE "error:|error #|CMake Error|undefined reference" "$BUILD_LOG" | head -20 | sed 's/^/     /'
    echo "   Car NOT restarted -- the old, working build is still live. Fix the C++ and re-run."
    rm -f "$BUILD_LOG"; exit 1
fi
rm -f "$BUILD_LOG"

echo ">> 4/4  restart car-ros so the node runs the new build, then verify"
$SSH "sudo systemctl restart car-ros"
CARSH="source /opt/ros/melodic/setup.bash; export ROS_MASTER_URI=http://localhost:11311;"
rate=""
for i in $(seq 1 16); do        # car-ros needs ~15-20 s to come back
    sleep 2
    rate=$($SSH "$CARSH timeout 3 rostopic hz /obstacles 2>&1 | grep -m1 'average rate'" 2>/dev/null || true)
    [ -n "$rate" ] && break
done
if [ -z "$rate" ]; then
    echo "   ✗ /obstacles is NOT publishing -- the new node likely CRASHED at runtime. car-ros log:"
    $SSH "sudo journalctl -u car-ros --no-pager -n 60 2>&1 | grep -iE 'obstacle|error|fault|assert|terminate|exception|core dump' | tail -15" 2>/dev/null | sed 's/^/     /'
    echo "   Perception is down until you fix + re-run. Full log: ssh $USER@$CAR sudo journalctl -u car-ros -n 100"
    exit 1
fi
echo "   ✓ /obstacles publishing: $rate"
echo "DONE -- car is running the new obstacle computation."
