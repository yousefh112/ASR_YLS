#!/bin/bash
# Terminal 1 on the robot: the bringup, with everything it prints kept.
#
#   ./robot_bringup.sh                 # run tag defaults to "robot"
#   ./robot_bringup.sh run3            # tag it
#   ./robot_bringup.sh run3 teleop:=false
#
# Same as `ros2 launch asr_summer_school bringup.launch.py`, except the output
# goes to a file as well as the screen.  That matters because the bringup is
# where the quiet failures announce themselves - the LiDAR's real range, a
# scan_preprocess warning, apriltag's parameters, the SLAM configuration - and
# on the day nobody is going to scroll back through a terminal to find them.
#
# Leave this running.  Then start the mission with mission_run.sh - on your
# LAPTOP, not here, so the deliverables are written there and need no copying
# off the robot afterwards.  See RUNBOOK section 3.0.
#
# Deliberately no `set -u`: ROS 2's own setup.bash reads unbound variables and
# aborts the script the moment it is sourced under nounset.
TAG=${1:-robot}
if [ $# -ge 1 ]; then shift; else shift $#; fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# This is the ROBOT's script: it opens the base, the LiDAR and the camera, all
# of which are USB devices plugged into the robot.  `setup_env.sh onboard 11`
# sets CAMERA_MODEL, while the laptop form deliberately unsets it because the
# laptop starts no camera driver - so an unset one here is a reliable sign of
# being on the wrong machine.  bringup.launch.py would refuse anyway, but it
# refuses with a message about an environment variable, which is not the same as
# being told you are on the wrong computer.
if [ -z "${CAMERA_MODEL:-}" ]; then
  echo "!! CAMERA_MODEL is not set, which usually means this is the LAPTOP."
  echo
  echo "   The bringup runs ON THE ROBOT - it opens USB devices that are"
  echo "   plugged in there:"
  echo "     ssh students@192.168.10.111"
  echo "     source ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge/setup_env.sh onboard 11"
  echo "     robot_bringup.sh ${1:-myrun}"
  echo
  echo "   From the laptop you want the mission instead, which is what writes"
  echo "   the deliverables:"
  echo "     mission_run.sh ${1:-myrun} 240"
  echo
  echo "   If you really did mean to run the bringup here, export CAMERA_MODEL"
  echo "   first.  See RUNBOOK section 3.0."
  exit 1
fi
OUT="$HOME/asr_mission_output/$TAG"
mkdir -p "$OUT"

echo "== bringup, logging to $OUT/bringup.log"
echo "   leave this running; start the mission with:"
echo "     ./mission_run.sh $TAG 240      (on your LAPTOP)"
echo

# stdbuf keeps the log in step with the screen; without it a crash can lose the
# last few lines to a half-full buffer, which are exactly the interesting ones.
stdbuf -oL -eL ros2 launch asr_summer_school bringup.launch.py "$@" 2>&1 \
  | tee "$OUT/bringup.log"
