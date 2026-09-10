#!/bin/bash
# Copy this workspace to the robot and build it there.
#
#   deploy_to_robot.sh                 # robot 11, copy and build
#   deploy_to_robot.sh 11              # same, explicit
#   deploy_to_robot.sh 11 --no-build   # copy only, for a Python-only change
#
# The robot needs the workspace because the bringup is ours: our
# slam_toolbox.launch.py, our camera.launch.py at 1280x720, our apriltag config,
# and the C++ frontier detector. Only `turtlebot3_bringup` is a system package.
# Cloning on the robot instead would work, but only if the robot has internet,
# and on the day it may not - rsync over the robot's own wifi always does.
#
# What is sent is the source only: about 11 MB. build/, install/, log/, .git and
# the editor's caches are excluded, which is the difference between 11 MB and
# well over a gigabyte.
#
# The first deploy builds, which takes a few minutes because the vendored
# apriltag stack is compiled from source. After that, `--no-build` is enough for
# a Python-only change: the workspace is built with --symlink-install, so the
# installed Python files point at the sources this script overwrites.  A change
# to C++, to a launch file or to package.xml still needs a build.
#
# Deliberately no `set -u`: ROS 2's own setup.bash reads unbound variables.
set -e

ROBOT=11
BUILD=yes
for arg in "$@"; do
  case "$arg" in
    --no-build) BUILD=no ;;
    [0-9]|[0-9][0-9]) ROBOT=$arg ;;
    *) echo "usage: deploy_to_robot.sh [robot number] [--no-build]"; exit 1 ;;
  esac
done

HOST="192.168.10.1$(printf '%02d' "$ROBOT")"
USER=students
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"          # the ASR_YLS checkout

echo "== deploying $REPO -> $USER@$HOST:~/ASR_YLS"

if ! ping -c1 -W2 "$HOST" > /dev/null 2>&1; then
  echo "!! $HOST does not answer a ping."
  echo "   On the robot's wifi?  SSID SESASR_WiFi, password LED05_2024"
  exit 1
fi

# --delete so a file removed here is removed there: a stale launch file on the
# robot that no longer exists in the repo is exactly the kind of thing that
# wastes a lab slot.
rsync -az --delete \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  --exclude '.git/' --exclude '.vscode/' --exclude '.claude/' \
  --exclude '__pycache__/' --exclude '.pytest_cache/' \
  --exclude '*.pyc' \
  "$REPO/" "$USER@$HOST:~/ASR_YLS/"

echo "== source copied"

if [ "$BUILD" = no ]; then
  echo "== skipping the build (--no-build)"
  echo "   Only valid for a Python-only change; C++, launch files and"
  echo "   package.xml need a build."
  exit 0
fi

echo "== building on the robot (first time takes a few minutes)"
ssh "$USER@$HOST" 'bash -lc "
  set -e
  cd ~/ASR_YLS/ros2_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install
"'

echo
echo "== done.  On the robot now:"
echo "     source ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge/setup_env.sh onboard $ROBOT"
echo "     robot_bringup.sh myrun"
