#!/bin/bash
# One-command simulation run: headless, logged, scored.
#
#   ./sim_run.sh                              600 s, default settings
#   ./sim_run.sh quick 300                    a 300 s run tagged "quick"
#   ./sim_run.sh nosweep 600 scan_rotation:=0.0
#
# Anything after the duration goes straight to mission.launch.py, which is how
# A/B comparisons are run.  Logs and deliverables land in
# ~/asr_mission_output/<tag>/ and the run is scored when it finishes.
#
# The three-terminal form in RUNBOOK.md is still the one for the robot, and for
# any run you want to watch in RViz.  This is for iterating.
#
# Deliberately no `set -u`: ROS 2's own setup.bash reads unbound variables
# (AMENT_TRACE_SETUP_FILES among them) and aborts the script the moment it is
# sourced under nounset.
TAG=${1:-run}
DURATION=${2:-600.0}
if [ $# -ge 2 ]; then shift 2; else shift $#; fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"          # ros2_ws
source "$HERE/setup_env.sh" sim > /dev/null

OUT="$HOME/asr_mission_output/$TAG"
rm -rf "$OUT"; mkdir -p "$OUT"
bash "$HERE/sim_stop.sh" > /dev/null
cd "$WS"

echo "== gazebo (headless)"
setsid nohup ros2 launch asr_summer_school project.launch.py gui:=false \
    > "$OUT/gazebo.log" 2>&1 < /dev/null &
sleep 22

echo "== bringup: SLAM, scan preprocessing, apriltag, frontier detector"
setsid nohup ros2 launch asr_summer_school bringup_simulation.launch.py use_sim_time:=true \
    > "$OUT/bringup.log" 2>&1 < /dev/null &
sleep 24

echo "== mission: ${DURATION}s ${*:-}"
ros2 launch asr_summer_school mission.launch.py use_sim_time:=true \
    mission_duration:="$DURATION" output_directory:="$OUT" "$@" \
    > "$OUT/mission.log" 2>&1

echo
sed 's/\x1b\[[0-9;]*m//g' "$OUT/mission.log" | grep -E "mission over|home:" | tail -2
ros2 run asr_summer_school score_report.py --run "$OUT" 2>/dev/null || true
echo "logs and deliverables: $OUT"
bash "$HERE/sim_stop.sh"
