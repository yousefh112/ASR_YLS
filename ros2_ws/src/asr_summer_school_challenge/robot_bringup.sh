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
