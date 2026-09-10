#!/bin/bash
# One-command simulation run: headless, logged, scored.
#
# Sourcing setup_env.sh puts this on PATH, so it runs from any directory.
#
#   sim_run.sh                                240 s - the real window
#   sim_run.sh long 600                       the maze at its own scale
#   sim_run.sh nosweep 240 scan_rotation:=0.0
#
# 240 s is the announced competition window and the default here so the
# simulation exercises the same timings as the real run.  Note the simulated
# maze is 400 m2 against the real arena's 40, so a 240 s run of it maps about a
# fifth and finds three or four tags - that is the maze being ten times too big,
# not the stack failing.  Use `long 600` when you want the maze itself explored.
#
# Anything after the duration goes straight to mission.launch.py, which is how
# A/B comparisons are run.  Logs and deliverables land in
# ~/asr_mission_output/<tag>/ and the run is scored when it finishes.
#
# For the robot, see RUNBOOK section 3: the bringup runs there and
# mission_run.sh runs on the laptop, so the deliverables need no copying.
#
# Deliberately no `set -u`: ROS 2's own setup.bash reads unbound variables
# (AMENT_TRACE_SETUP_FILES among them) and aborts the script the moment it is
# sourced under nounset.
TAG=${1:-run}
DURATION=${2:-240.0}
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
