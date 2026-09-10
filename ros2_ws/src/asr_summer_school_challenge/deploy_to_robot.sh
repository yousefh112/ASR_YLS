#!/bin/bash
# Copy this workspace to the robot and build it there.
#
#   deploy_to_robot.sh                 # robot 11, copy and build
#   deploy_to_robot.sh 11              # same, explicit
#   deploy_to_robot.sh 11 --no-build   # copy only, for a Python-only change
#   deploy_to_robot.sh 11 --setup-key  # install your ssh key first, then deploy
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
SETUP_KEY=no
for arg in "$@"; do
  case "$arg" in
    --no-build) BUILD=no ;;
    --setup-key) SETUP_KEY=yes ;;
    [0-9]|[0-9][0-9]) ROBOT=$arg ;;
    *) echo "usage: deploy_to_robot.sh [robot number] [--no-build] [--setup-key]"; exit 1 ;;
  esac
done

HOST="192.168.10.1$(printf '%02d' "$ROBOT")"
LOGIN=students
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"          # the ASR_YLS checkout

# This copies laptop -> robot.  Run on the robot it would rsync the robot to
# itself, from a repo that is not there yet, which is the confusing way to
# discover you are on the wrong machine.
if hostname -I 2>/dev/null | tr ' ' '\n' | grep -qx "$HOST" \
   || [ "$(hostname)" = "nuc$ROBOT" ]; then
  echo "!! This looks like the ROBOT ($(hostname))."
  echo "   deploy_to_robot.sh copies the workspace FROM your laptop TO here,"
  echo "   so run it on the laptop.  Nothing to do on the robot."
  exit 1
fi

echo "== deploying $REPO -> $LOGIN@$HOST:~/ASR_YLS"

if ! ping -c1 -W2 "$HOST" > /dev/null 2>&1; then
  echo "!! $HOST does not answer a ping."
  echo "   On the robot's wifi?  SSID SESASR_WiFi, password LED05_2024"
  exit 1
fi

# One SSH connection shared by the rsync and the build, so the password is asked
# once rather than once per command.  Without this the robot asks twice, which
# on a slot clock is two chances to mistype it.
CTL="${TMPDIR:-/tmp}/asr-deploy-$ROBOT-$$"
cleanup() { ssh -o ControlPath="$CTL" -O exit "$LOGIN@$HOST" 2>/dev/null || true; }
trap cleanup EXIT

if ssh -o BatchMode=yes -o ConnectTimeout=4 "$LOGIN@$HOST" true 2>/dev/null; then
  echo "== ssh key accepted, no password needed"
elif [ "$SETUP_KEY" = yes ]; then
  echo "== installing your public key on the robot (password once: sesasr)"
  ssh-copy-id "$LOGIN@$HOST" || { echo "!! ssh-copy-id failed"; exit 1; }
  echo "== key installed; nothing asks for a password from here on"
else
  echo "== the robot will ask for the password: sesasr"
  echo "   Asked once here, thanks to a shared connection - but every other ssh"
  echo "   and scp asks again.  Do this once at the start of a session and"
  echo "   none of them ever does:"
  echo "       deploy_to_robot.sh $ROBOT --setup-key"
fi

# Foreground, deliberately.  `ssh -fN` backgrounds itself BEFORE authenticating,
# so it cannot read a password from the terminal and falls back to ssh-askpass,
# a GUI helper that is not installed on a headless box - the failure reads
# "ssh_askpass: No such file or directory", which looks like a missing program
# rather than a password that was never asked for.  Running a trivial command in
# the foreground prompts normally, and ControlPersist keeps the authenticated
# connection open for the rsync and the build that follow.
if ! ssh -o ControlMaster=auto -o ControlPath="$CTL" -o ControlPersist=180 \
        "$LOGIN@$HOST" true; then
  echo
  echo "!! could not open an SSH connection to $LOGIN@$HOST"
  echo "   The password is 'sesasr'.  Check it by hand first:"
  echo "       ssh $LOGIN@$HOST"
  echo "   If that works and this does not, say so - it is not the network."
  exit 1
fi

SSH_SHARED="ssh -o ControlPath=$CTL"

# --delete so a file removed here is removed there: a stale launch file on the
# robot that no longer exists in the repo is exactly the kind of thing that
# wastes a lab slot.
rsync -az --delete --info=stats1 -e "$SSH_SHARED" \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  --exclude '.git/' --exclude '.vscode/' --exclude '.claude/' \
  --exclude '__pycache__/' --exclude '.pytest_cache/' \
  --exclude '*.pyc' \
  "$REPO/" "$LOGIN@$HOST:~/ASR_YLS/"

echo "== source copied"

if [ "$BUILD" = no ]; then
  echo "== skipping the build (--no-build)"
  echo "   Only valid for a Python-only change; C++, launch files and"
  echo "   package.xml need a build."
  exit 0
fi

echo "== building on the robot (first time takes a few minutes)"
$SSH_SHARED "$LOGIN@$HOST" 'bash -lc "
  set -e
  cd ~/ASR_YLS/ros2_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install
"'

echo
echo "== done.  On the robot now:"
echo "     source ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge/setup_env.sh onboard $ROBOT"
echo "     robot_bringup.sh myrun"
