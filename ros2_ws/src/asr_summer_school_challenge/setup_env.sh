# Source this, do not run it.
#
#   source setup_env.sh sim            a Gazebo run on this machine
#   source setup_env.sh onboard 11     in an SSH shell ON robot 11
#   source setup_env.sh 11             on your laptop, talking to robot 11
#
# The two robot modes differ in one thing that matters: the robot runs the
# Zenoh router, so it must not be configured as a client pointing at itself.
#
# Sets the environment for one side of the system and prints what it did, so a
# shell that behaves oddly can be diagnosed by looking at it rather than by
# guessing.  Zenoh and Fast DDS cannot see each other and the symptom is an
# empty `ros2 topic list` rather than an error, which is worth one script.

_asr_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
_asr_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Put the run scripts on PATH, so `robot_bringup.sh` and `mission_run.sh` work
# from whatever directory the shell happens to be in.  They used to be
# documented as `./robot_bringup.sh`, which only works from inside the repo and
# fails as "No such file or directory" from anywhere else - a confusing error,
# because it names the script rather than the directory, and reads as though the
# file were missing.  Exported so a fresh shell that sources this gets it too.
export ASR_DIR="$_asr_dir"
case ":$PATH:" in
  *":$_asr_dir:"*) ;;
  *) export PATH="$_asr_dir:$PATH" ;;
esac

source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo/setup.bash ] && source /usr/share/gazebo/setup.bash
if [ -f "$_asr_ws/install/setup.bash" ]; then
  source "$_asr_ws/install/setup.bash"
else
  echo "!! $_asr_ws/install not found - run colcon build first"
fi

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02

case "${1:-}" in
  sim)
    # Deliberately not zenoh: there is no robot and no router on this network,
    # and zenoh in client mode pointed at an absent endpoint fails silently.
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset ZENOH_CONFIG_OVERRIDE
    export ROS_DOMAIN_ID=42
    export GAZEBO_MODEL_DATABASE_URI=""
    unset CAMERA_MODEL          # simulation has no camera driver to choose
    echo "ASR environment: SIMULATION"
    ;;
  onboard)
    if [ -z "${2:-}" ]; then
      echo "usage: source setup_env.sh onboard <robot number>"
      return 1 2>/dev/null || exit 1
    fi
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ROS_DOMAIN_ID="$2"
    # No client override: this machine hosts the router.
    unset ZENOH_CONFIG_OVERRIDE
    # Our robot has the RealSense.  Set CAMERA_MODEL before sourcing only if
    # you are ever on a machine fitted with the OAK-D instead.
    : "${CAMERA_MODEL:=realsense}"
    export CAMERA_MODEL
    echo "ASR environment: ON ROBOT $2"
    ;;
  [0-9]|[0-9][0-9])
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ROS_DOMAIN_ID="$1"
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/192.168.10.1$(printf '%02d' "$1"):7447\"]"
    unset CAMERA_MODEL          # the laptop starts no camera driver
    echo "ASR environment: LAPTOP -> ROBOT $1 at 192.168.10.1$(printf '%02d' "$1")"
    # Do NOT start a router here.  This shell is a zenoh CLIENT pointing at the
    # robot, and the router belongs at the endpoint it points to - on the robot.
    # A router started here listens on the laptop, nothing connects to it, and
    # the failure reads "Unable to connect to any of [tcp/192.168.10.1NN:7447]"
    # or, worse, an empty `ros2 topic list` with no error at all.
    echo "  the ROBOT hosts the router: run 'ros2 run rmw_zenoh_cpp rmw_zenohd'"
    echo "  over SSH on the robot, not here, before starting the bringup."
    ;;
  *)
    echo "usage: source setup_env.sh sim"
    echo "       source setup_env.sh onboard <robot number>   (on the robot)"
    echo "       source setup_env.sh <robot number>           (on your laptop)"
    echo
    echo "  we are robot 11:  source setup_env.sh 11          (laptop)"
    echo "                    source setup_env.sh onboard 11  (robot)"
    return 1 2>/dev/null || exit 1
    ;;
esac

echo "  run scripts on PATH: robot_bringup.sh, mission_run.sh, sim_run.sh"
for v in RMW_IMPLEMENTATION ROS_DOMAIN_ID TURTLEBOT3_MODEL LDS_MODEL \
         CAMERA_MODEL ZENOH_CONFIG_OVERRIDE; do
  printf '  %-22s %s\n' "$v" "${!v:-<unset>}"
done
unset _asr_ws _asr_dir
