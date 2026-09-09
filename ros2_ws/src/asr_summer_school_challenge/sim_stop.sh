#!/bin/bash
# Kill a simulation stack.
#
# The ancestor check is not decoration.  `pkill -f` matches whole command lines,
# and the shell that invokes this one usually carries the very patterns being
# searched for somewhere in its own arguments - so a naive pkill kills the thing
# that called it.  That happened twice while this project was being built.
ancestors=""
pid=$$
while [ "$pid" -gt 1 ]; do
  ancestors="$ancestors $pid"
  pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
  [ -z "$pid" ] && break
done

kill_pattern() {
  for p in $(pgrep -f "$1" 2>/dev/null); do
    skip=0
    for a in $ancestors; do [ "$p" = "$a" ] && skip=1; done
    [ "$skip" = "0" ] && kill -9 "$p" 2>/dev/null
  done
}

for pattern in gzserver gzclient robot_state_publisher spawn_entity \
               slam_toolbox scan_preprocess.py mission_control.py \
               frontier_detection_node apriltag_node detection2landmark \
               component_container lifecycle_manager \
               joy_linux teleop_node "bin/ros2 launch"; do
  kill_pattern "$pattern"
done
sleep 2
echo "simulation stopped"
