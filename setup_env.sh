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
    : "${CAMERA_MODEL:=realsense}"
    export CAMERA_MODEL
    echo "ASR environment: ON ROBOT $2"
    echo "  CAMERA_MODEL is ${CAMERA_MODEL}; export it before sourcing if the"
    echo "  robot has an OAK-D instead."
    ;;
  [0-9]|[0-9][0-9])
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ROS_DOMAIN_ID="$1"
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/192.168.10.1$(printf '%02d' "$1"):7447\"]"
    unset CAMERA_MODEL          # the laptop starts no camera driver
    echo "ASR environment: LAPTOP -> ROBOT $1 at 192.168.10.1$(printf '%02d' "$1")"
    echo "  remember to run, in its own terminal:  ros2 run rmw_zenoh_cpp rmw_zenohd"
    ;;
  *)
    echo "usage: source setup_env.sh sim"
    echo "       source setup_env.sh onboard <robot number>   (on the robot)"
    echo "       source setup_env.sh <robot number>           (on your laptop)"
    return 1 2>/dev/null || exit 1
    ;;
esac

for v in RMW_IMPLEMENTATION ROS_DOMAIN_ID TURTLEBOT3_MODEL LDS_MODEL \
         CAMERA_MODEL ZENOH_CONFIG_OVERRIDE; do
  printf '  %-22s %s\n' "$v" "${!v:-<unset>}"
done
unset _asr_ws
