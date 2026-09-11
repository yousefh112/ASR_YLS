#!/bin/bash
# The timed mission, logged, with a diagnostics snapshot around it.
#
# RUN THIS WHERE YOU WANT THE RESULTS. It starts Nav2 and the orchestrator, and
# the orchestrator is what writes the deliverables - so running it on your laptop
# puts map.pgm, the semantic map and the logs straight onto your laptop, with
# nothing to copy off the robot afterwards. Running it over SSH on the robot
# leaves them there instead. Both work; the files land next to whoever ran this.
#
# The robot must already be running, in its own SSH sessions:
#     ros2 run rmw_zenoh_cpp rmw_zenohd        # the router. Robot only, always.
#     robot_bringup.sh <tag>                 # drivers, SLAM, camera, apriltag
#
# Those cannot move to the laptop: they open USB devices that are plugged into
# the robot, and apriltag has to sit next to the camera because raw 1280x720 at
# 15 fps is about 330 Mbit/s, which no wifi in this building will carry. What
# crosses the network in this split is /scan_filtered, /map, /tf, /odom, the
# detections and cmd_vel - well under 1 Mbit/s.
#
#   mission_run.sh                       # tag "robot", 240 s
#   mission_run.sh run3 240              # tag "run3"
#   mission_run.sh run3 240 scan_rotation:=0.0     # extra args go to the launch
#
# What this produces, in ~/asr_mission_output/<tag>/ ON THIS MACHINE:
#
#   mission.log        everything the mission printed
#   diagnostics.txt    the state of the graph before and after the run
#   coverage_report.txt  what the run looked at and what it did not
#   map.pgm/.yaml      the occupancy grid            +100
#   semantic_map.*     the tags, in three formats    +100
#   mission_report.json  timings, goal counts, map stats, SLAM jumps
#   mission_overlay.png  the grid with tags and start/finish drawn on
#   <tag>-<date>.tar.gz  all of the above, ready to hand over
#
# bringup.log is only here if robot_bringup.sh ran on THIS machine. In the split
# above it stays on the robot; fetch it with scp if a run needs diagnosing.
#
# `score_report.py` is NOT run here.  It scores against tag coordinates parsed
# out of the Gazebo world file, and there is no such file for a physical arena -
# it would either fail or, worse, score the run against the wrong arena.
#
# Deliberately no `set -u`: ROS 2's setup.bash reads unbound variables.
TAG=${1:-robot}
DURATION=${2:-240.0}
if [ $# -ge 2 ]; then shift 2; else shift $#; fi

# --now: launch the mission immediately.  For a TIMED run: the organisers'
# stopwatch starts when you press enter, and pre-flight plus the before-snapshot
# take about 45 s before the robot even starts.  Coming home even a few seconds
# past their deadline scores 0 instead of +150.  So run pre-flight separately,
# before they start timing, and then this with --now:
#     ros2 run asr_summer_school preflight.py      # untimed
#     mission_run.sh race 240 --now                 # timed, starts at once
# The after-snapshot and the tarball still happen, once the run is over.
NOW=no
ARGS=()
for a in "$@"; do
  if [ "$a" = "--now" ]; then NOW=yes; else ARGS+=("$a"); fi
done
set -- "${ARGS[@]}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$HOME/asr_mission_output/$TAG"
mkdir -p "$OUT"
DIAG="$OUT/diagnostics.txt"

# Every probe is wrapped in a timeout.  This runs immediately before a scored
# mission, so a hung `ros2 topic hz` against a dead publisher must cost five
# seconds, not the run.
probe() {
  echo "----- $* -----" >> "$DIAG"
  timeout -k 2 12 "$@" >> "$DIAG" 2>&1 || echo "(no answer within 12 s)" >> "$DIAG"
  echo >> "$DIAG"
}

# Sensor topics are published best-effort by some drivers and reliably by
# others, and `ros2 topic echo` defaults to reliable - against a best-effort
# publisher it subscribes happily and then receives nothing at all, which looks
# exactly like a dead topic.  Try the default, fall back to best-effort, and
# only then call it dead.
echo_once() {
  local topic=$1 field=$2
  echo "----- echo $topic $field -----" >> "$DIAG"
  if timeout -k 2 6 ros2 topic echo "$topic" --field "$field" --once >> "$DIAG" 2>&1; then
    :
  elif timeout -k 2 6 ros2 topic echo "$topic" --field "$field" --once \
       --qos-reliability best_effort >> "$DIAG" 2>&1; then
    echo "(needed best-effort QoS)" >> "$DIAG"
  else
    echo "(nothing received on either QoS)" >> "$DIAG"
  fi
  echo >> "$DIAG"
}

snapshot() {
  {
    echo "==================== $1 ===================="
    date -Is
  } >> "$DIAG"
  probe ros2 node list
  probe ros2 topic list
  # 4 s each: enough for a rate estimate on anything publishing above ~2 Hz,
  # and this whole snapshot runs twice, once of it before a scored mission.
  for t in /scan /scan_filtered /map /camera/image_raw /camera/detections; do
    echo "----- hz $t -----" >> "$DIAG"
    timeout -k 2 4 ros2 topic hz "$t" >> "$DIAG" 2>&1
    echo >> "$DIAG"
  done
  # The LiDAR's real horizon, and what scan_preprocess turned it into.  These
  # two numbers decide whether the mapper traces open space at all.
  echo_once /scan range_max
  echo_once /scan_filtered range_max
  # Two frames worth having on record: map->base_footprint is what the return
  # is judged against, and the camera's optical frame is what tag positions
  # hang off.  tf2_echo streams and has no --once in Humble, so it is the
  # timeout that ends it, not the tool.
  for pair in "map base_footprint" "base_link camera_color_optical_frame"; do
    echo "----- tf2_echo $pair -----" >> "$DIAG"
    timeout -k 2 5 ros2 run tf2_ros tf2_echo $pair >> "$DIAG" 2>&1
    echo >> "$DIAG"
  done
  # Which frame apriltag actually parents tags to decides whether the optical
  # correction is applied, and it differs between Gazebo and the RealSense.
  echo "----- tf frames containing 'camera' or 'tag' -----" >> "$DIAG"
  timeout -k 2 10 ros2 topic echo /tf_static --once >> "$DIAG" 2>&1
  echo >> "$DIAG"
}

{
  echo "############ RUN $TAG ############"
  echo "date          $(date -Is)"
  echo "duration      $DURATION s"
  echo "extra args    ${*:-none}"
  echo "host          $(hostname)"
  echo "git           $(cd "$HERE" && git rev-parse --short HEAD 2>/dev/null) \
$(cd "$HERE" && git status --porcelain 2>/dev/null | wc -l) file(s) modified"
  echo
  echo "--- environment ---"
  for v in RMW_IMPLEMENTATION ROS_DOMAIN_ID TURTLEBOT3_MODEL LDS_MODEL \
           CAMERA_MODEL ZENOH_CONFIG_OVERRIDE ZENOH_ROUTER_CONFIG_URI; do
    echo "$v=${!v}"
  done
  echo
} > "$DIAG"

# The bringup has to be up already: this script starts Nav2 and the mission, not
# the sensors.  Catching it here is much cheaper than watching the mission fail
# to find a TF tree ninety seconds in.
if ! timeout -k 2 15 ros2 node list 2>/dev/null | grep -q .; then
  echo "!! No ROS nodes visible."
  echo "   Is the bringup running ON THE ROBOT?   robot_bringup.sh $TAG"
  echo "   Is the Zenoh router up ON THE ROBOT?    ros2 run rmw_zenoh_cpp rmw_zenohd"
  echo "   Do the two shells agree on RMW_IMPLEMENTATION and ROS_DOMAIN_ID?"
  exit 1
fi

if [ "$NOW" = yes ]; then
  echo "== --now: launching at once, $(date +%H:%M:%S).  Pre-flight and the"
  echo "   before-snapshot are skipped - run pre-flight BEFORE the stopwatch starts."
else
  echo "== pre-flight"
  { echo "==================== PREFLIGHT ===================="; } >> "$DIAG"
  timeout -k 2 120 ros2 run asr_summer_school preflight.py 2>&1 | tee -a "$DIAG"
  PREFLIGHT=${PIPESTATUS[0]}
  if [ "$PREFLIGHT" -ne 0 ]; then
    echo
    echo "!! preflight reported a problem (exit $PREFLIGHT).  Read it above."
    echo "   Ctrl-C now to fix it, or wait 15 s to run anyway."
    sleep 15
  fi

  echo "== graph snapshot (before)"
  snapshot "BEFORE THE RUN"
fi

echo
echo "== mission: ${DURATION}s ${*:-}"
echo "   put the robot on the start point and do not touch it: intervening costs 50 points"
echo
stdbuf -oL -eL ros2 launch asr_summer_school mission.launch.py \
    mission_duration:="$DURATION" output_directory:="$OUT" "$@" 2>&1 \
  | tee "$OUT/mission.log"

echo
echo "== graph snapshot (after)"
snapshot "AFTER THE RUN"

echo
echo "== result"
sed 's/\x1b\[[0-9;]*m//g' "$OUT/mission.log" | grep -E "mission over|home:" | tail -2

BUNDLE="$OUT/$TAG-$(date +%Y%m%d-%H%M%S).tar.gz"
tar -czf "$BUNDLE" -C "$HOME/asr_mission_output" \
    --exclude='*.tar.gz' "$TAG" 2>/dev/null
echo
echo "deliverables and logs: $OUT"
echo "one file to hand over: $BUNDLE"
