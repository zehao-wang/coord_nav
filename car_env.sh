# car_env.sh  —  source this to talk to the car (Jetson Nano) over coord_nav.
#   usage:  source car_env.sh     (or use the `roscar` shortcut)
#
# Sets up the ROS Noetic conda env and points it at the car's roscore.

# --- config (edit here if the setup changes) ---------------------------
CAR_IP=10.42.0.187          # car's fixed IP on the coord_nav hotspot
SELF_IP=10.42.0.1           # this workstation's IP on the hotspot
CONDA_ROOT=/home/zwa0839/miniconda3
ROS_ENV=ros1

# --- activate ROS Noetic (conda) ---------------------------------------
set +u
source "$CONDA_ROOT/etc/profile.d/conda.sh" 2>/dev/null
conda activate "$ROS_ENV" 2>/dev/null
set -u 2>/dev/null

# --- ROS networking ----------------------------------------------------
export ROS_MASTER_URI="http://${CAR_IP}:11311"
export ROS_IP="$SELF_IP"

echo "car env ready:"
echo "  ROS distro   : $(rosversion -d 2>/dev/null)"
echo "  ROS_MASTER_URI = $ROS_MASTER_URI"
echo "  ROS_IP         = $ROS_IP"
echo "  (try: rostopic list)"
