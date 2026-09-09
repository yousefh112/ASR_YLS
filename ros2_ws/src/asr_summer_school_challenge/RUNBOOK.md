# Runbook — Autonomous Search-and-Rescue, TurtleBot3

Everything needed to build, run and debug the mission, in the order you need it.
Written to be followed at 14:00 on competition day without reading anything else.

- [1. Build](#1-build)
- [2. Run in simulation](#2-run-in-simulation)
- [3. Run on the robot](#3-run-on-the-robot)
- [4. What comes out](#4-what-comes-out)
- [5. Tuning for the real arena](#5-tuning-for-the-real-arena)
- [6. Diagnosing a bad run](#6-diagnosing-a-bad-run)
- [7. What was built and why](#7-what-was-built-and-why)

---

## 1. Build

```bash
cd ~/ASR_YLS/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`apriltag`, `apriltag_msgs` and `apriltag_ros` are vendored under
`src/third_party/` and build with everything else, so no apt install is needed
for tag detection. On the robot PC they are almost certainly already installed
system-wide; the workspace copies simply take precedence and behave identically.

Offline tests — no ROS, no simulator, under a second:

```bash
python3 -m pytest src/asr_summer_school_challenge/asr_summer_school/test -q
```

Run these after any edit to the mission logic. They cover the exploration
policy, tag fusion, the deadline budget, the export formats, and the scan-range
invariants that localisation depends on.

---

## 2. Run in simulation

Four terminals, each with the workspace sourced. Simulation is for validating
the stack, not for tuning constants — the physical arena is a different place.

```bash
# --- every terminal ---
source ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge/setup_env.sh sim
```

That sources the workspace and sets the middleware to Fast DDS, which matters:
your `.bashrc` points Zenoh at the robot's IP, and Zenoh in client mode aimed at
an absent endpoint fails silently rather than loudly. The script prints what it
set, so a shell behaving oddly can be diagnosed by looking rather than guessing.

```bash
# 1  Gazebo.  gui:=false runs it headless, which is much faster.
ros2 launch asr_summer_school project.launch.py

# 2  SLAM, scan preprocessing, camera TF, AprilTag, frontier detector
ros2 launch asr_summer_school bringup_simulation.launch.py use_sim_time:=true

# 3  Nav2 + the mission
ros2 launch asr_summer_school mission.launch.py use_sim_time:=true \
    mission_duration:=600.0 output_directory:=~/asr_mission_output

# 4  optional: watch it
ros2 launch turtlebot3_bringup rviz2.launch.py use_sim_time:=true
```

Score the finished run against the arena's own ground truth:

```bash
ros2 run asr_summer_school score_report.py --run ~/asr_mission_output
```

### One command instead of three

For iterating, `sim_run.sh` does all of the above headless and scores the result:

```bash
cd ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge
./sim_run.sh                                     # 600 s, default settings
./sim_run.sh quick 300                           # 300 s, tagged "quick"
./sim_run.sh nosweep 600 scan_rotation:=0.0      # extra args go to mission.launch.py
./sim_stop.sh                                    # kill a stack left running
```

Logs and deliverables land in `~/asr_mission_output/<tag>/`. Measured scores and a
reference set of deliverables are in [`results/`](results/).

---

## 3. Run on the robot

### 3.1 Your laptop, once per session

```bash
source ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge/setup_env.sh <robot number>
ros2 run rmw_zenoh_cpp rmw_zenohd            # leave this running
```

Everyone on the team must be on `rmw_zenoh_cpp`. Zenoh and Fast DDS cannot see
each other, and the symptom is an empty `ros2 topic list` rather than an error.

### 3.2 On the robot, over SSH

```bash
ssh students@192.168.10.1<NN>                # password: sesasr

source ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge/setup_env.sh onboard <robot number>
```

`onboard` differs from the laptop form in one thing that matters: the robot
hosts the Zenoh router, so it must not be configured as a client pointing at
itself.

`bringup.launch.py` checks all four of those before starting anything and stops
with a message naming the missing one. Without that check an unset
`CAMERA_MODEL` surfaces as a `KeyError` from inside `turtlebot3_perception`
several seconds in, with half the stack already up.

**Terminal 1 — robot bringup** (base, LiDAR, camera, AprilTag, SLAM, frontier
detector; no Nav2):

```bash
ros2 launch asr_summer_school bringup.launch.py
# no joypad plugged in?  add  teleop:=false
```

Check the first line `scan_preprocess` prints. It reports the LiDAR's actual
range, and warns if that disagrees with what SLAM was configured for — see
[section 5](#5-tuning-for-the-real-arena).

**Terminal 2 — Nav2 and the mission:**

```bash
ros2 launch asr_summer_school mission.launch.py \
    use_sim_time:=false \
    mission_duration:=<seconds the organisers announce> \
    output_directory:=~/asr_mission_output
```

The robot spins once on the spot to seed the map, then explores. Put it at the
start point and leave it: touching it costs 50 points.

**To stop early and still keep the deliverables**, Ctrl-C the mission terminal
once. The export runs from a `finally` block, so the map and the semantic map
are written even on an interrupt. Two Ctrl-Cs kill it before that.

### 3.3 Pre-flight

One command, run after the bringup:

```bash
ros2 run asr_summer_school preflight.py            # after bringup
ros2 run asr_summer_school preflight.py --nav2     # after the mission too
```

It samples every topic the mission consumes with the QoS the publisher actually
uses, walks the TF chain, and checks the invariants that fail silently — the
no-return encoding localisation depends on, whether the colour camera frame is
reachable from `map`, whether the frontier detector is producing anything. Each
failure prints what to do about it. Exit status is 0 when nothing failed, so it
can gate a script.

Every check in it corresponds to something that actually broke during
development, and every one of those was silent: a dead scan preprocessor still
leaves a healthy `/scan`, a detached camera frame still produces detections.

Two things it cannot check, so check them yourself:

- [ ] **Batteries.** LiPo: 3S is 9.6–12.6 V, 4S is 12.8–16.8 V. Below minimum
      destroys the pack.
- [ ] **Hold a tag in front of the camera and re-run.** `preflight` warns rather
      than fails when no tag is in view, because that is normal — but it is the
      only way to confirm the whole detection path end to end before a run.

Then drive it a metre with the pad and watch the map in RViz.

## 4. What comes out

Written to `output_directory` at the end of every run, including an interrupted
one:

| File | What it is |
|---|---|
| `map.pgm`, `map.yaml` | The 2D occupancy grid, in exactly the format `nav2_map_server` writes. **+100** |
| `semantic_map.yaml` | Tag IDs and map-frame positions, in the shape the course's own `landmarks.yaml` uses. **+100** |
| `semantic_map.json` | The same tags plus per-tag provenance and full mission metadata |
| `semantic_map.csv` | One row per tag, for a spreadsheet |
| `mission_report.json` | Timings, goal counts, map statistics, detected SLAM jumps |
| `mission_overlay.png` | The grid with the tags, the start and the finish drawn on it |

The semantic map is written three ways because the required format is specified
nowhere in the handouts. Hand over whichever the organisers ask for. **Ask them
early** — it is 100 points.

---

## 5. Tuning for the real arena

Three numbers, in order of how much they matter.

**`mission_duration`** — the deadline, in seconds, from the organisers. Passed
on the command line. Returning on time is +150 and one minute late is −30, so
this is the single most valuable thing to get right. If in doubt, set it
*shorter* than announced.

**`laser_max_range`** — the LiDAR horizon in metres. The default 3.5 is correct
for the LDS-01 and the simulator; the LDS-02 reaches further. Check it on the
robot:

```bash
ros2 topic echo /scan --field range_max --once
```

If it disagrees, `scan_preprocess` prints a warning naming the value to use.
Pass `laser_max_range:=<value>` to `bringup.launch.py` and set
`max_laser_range` in `config/param_slam_toolbox.yaml` about 0.1 m below it.
Getting this wrong is not dangerous — readings past the configured horizon are
treated as "nothing there", which never invents an obstacle — but the robot maps
less per scan than it could.

**`mission_duration` again** — it is worth saying twice. Everything else on this
list is worth tens of points; this one is worth 180.

**`home_tolerance`** (`config/param_mission.yaml`, default 0.35 m) — the rule is
a circle of **50 cm radius** around the starting point. Aiming at 35 cm leaves
15 cm for the difference between where TF thinks the robot is and where it
physically is; measured drift is 3–8 cm, so the margin is real. Do not raise it
to 0.50: that puts the run on the boundary of a 150-point swing.

### Finishing early, and the one cut not to make

The rubric pays nothing extra for a short run, but a short run wins a tie on
time and cannot be caught out by a slow return. Two launch arguments, so nothing
has to be edited on the day:

```bash
stop_after_tags:=11    # leave the moment 11 tags are in hand
scan_rotation:=0.0     # drop the camera sweep at each frontier, ~8 s per goal
```

`stop_after_tags` works: 2 tags found at t+79 s, home by t+107 s of a 600 s
window with 493 s unused. **Only set it if the organisers announce the count** —
a tag is +50 and finishing early is +0, so leaving with tags unfound is a
straight loss.

`scan_rotation:=0.0` is the tempting cut and the wrong one. Spinning measured
48% of one 516 s run, but across four full runs removing or halving it left the
tag count unchanged at 4 and cost the *localisation*: accuracy 30 → 0, final
return 3 cm → 21 cm. A full turn feeds slam_toolbox about thirty
well-constrained pose-graph nodes from a known position.

| Camera sweep | Tags | Accuracy | Return | Total |
|---|---|---|---|---|
| Full turn (2π, default) | 5 / 4 | +30 | 0.05 / 0.03 m | **730 / 680** |
| Half turn (π) | 4 | +0 | 0.21 m | 650 |
| None | 4 | +0 | 0.21 m | 650 |

The real waste was *waiting*, and it is already gone: the no-frontier recovery
sat idle 25 s before each of three turns; it now waits 6 s and turns once.

Everything else lives in `config/param_mission.yaml` with a comment saying what
it trades off. If tag count is low and there is time to spare, `scan_rotation`
and `scan_rotation_min_slack` control the camera sweep at each frontier.

---

## 6. Diagnosing a bad run

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic list` empty | Middleware mismatch, or a stale daemon | Check `RMW_IMPLEMENTATION` on both ends; `pkill -9 -f ros && ros2 daemon stop` |
| "exploration complete" in seconds | `/frontier_centroids` silent, or every centroid rejected | The mission logs the reason. Check `ros2 topic echo /frontier_centroids --once` |
| Robot never moves, Nav2 active | No path to any frontier | `ros2 topic echo /global_costmap/costmap --once`; check the map has free space |
| Map stops growing | `/scan_filtered` dead | `ros2 topic hz /scan_filtered`. The whole stack reads it, so the bringup is configured to shut down if `scan_preprocess` exits rather than let everything run blind |
| Everything looks healthy but the robot explores a 4 m box | Same cause. Check the bringup log for a `scan_preprocess` traceback | |
| Tag positions off by tens of cm | A detection was fused against a stale transform | Should not happen: detections whose exact-timestamp lookup fails are dropped. If it recurs, check `ros2 topic hz /tf` |
| Map smears, walls double | False loop closure | The mission logs `SLAM moved the map by ...`. See below |
| Tags detected, positions nonsense | Optical-frame convention | `tag_manager` logs the frame it parented to and whether it corrected. Force with `optical_correction:=on|off` |
| Pad publishes `/joy`, robot still | Deadman button | `enable_button: 5` in `config/param_teleop.yaml` |
| `/dev/ttyACM0` error | Permissions | `sudo usermod -aG dialout students`, reboot |

**On the SLAM jump warning.** If the mission logs

```
SLAM moved the map by 3.06 m / 24 deg at t+434s - that is a graph
re-optimisation, not motion.
```

the scan matcher closed a loop against the wrong place. Everything expressed in
the map frame moved with it, including the recorded start pose. The mission
detects this and aims the return using the odometry anchor instead, which drifts
but cannot teleport. The run is still scoreable; the map and the tag positions
in it are only as good as the last re-optimisation. `mission_report.json`
records every jump under `map_jumps`.

---

## 7. What was built and why

The starter repo provides bringup, SLAM, Nav2, AprilTag detection and a complete
frontier *detector*. Nothing consumed the detector's output, and there was no
mission logic at all. This is what fills that gap.

### The mission stack

| File | Role |
|---|---|
| `asr_summer_school/mission_control.py` | The orchestrator. `INIT → EXPLORE → RETURN → FINALIZE`. Owns the deadline, dispatches goals, exports the deliverables from a `finally` block |
| `asr_summer_school/mission_clock.py` | The deadline and the return budget, re-estimated from a path the Nav2 planner actually produced |
| `asr_summer_school/frontier_policy.py` | Which frontier to drive to, and giving up on the ones that do not work out. No rclpy import, so it is unit tested |
| `asr_summer_school/frontier_client.py` | The transport half of the above |
| `asr_summer_school/tag_manager.py` | Tag TF frames in, fused map-frame landmarks out |
| `asr_summer_school/tag_map.py` | Deduplicate by ID, fuse repeats weighted by 1/range² with an outlier gate |
| `asr_summer_school/scan_preprocess.py` | Rewrites "no return" so SLAM can map open space. See below |
| `asr_summer_school/map_export.py` | The two deliverables, written without depending on any node still being alive |
| `asr_summer_school/map_recorder.py` | Holds the latest `/map` |
| `asr_summer_school/geometry.py` | Rigid transforms, no ROS dependency |
| `asr_summer_school/score_report.py` | Scores a finished run offline against the world file's own ground truth |

### The three defects that decided whether this worked at all

**1. Camera frames come from the URDF, not from us.** `robot_state_publisher` loads
`turtlebot3_gazebo/urdf/turtlebot3_burger.urdf` — not the one in
`turtlebot3_description`, which has no camera link — and that file already
provides `base_link -> camera_link -> camera_rgb_frame ->
camera_rgb_optical_frame`. Do not add static publishers for these: a second
parent for `camera_rgb_frame` makes the camera pose non-deterministic and tf2
does not warn about it. `preflight.py` checks the colour frame is reachable
from `map`.

**2. `inf` is not "nothing there" to Karto.** The LiDAR reports `inf` for a ray
that hits nothing within range. Karto's `OccupancyGrid::AddScan` discards any
reading `>= range_max` before it can trace anything, so open directions
produced no free space and the occupancy grid did not even grow to include
them. A robot in a corridor with both ends open mapped only the metre of wall
beside it, the frontier detector saw unknown cells adjacent to free ones in
every direction, DBSCAN merged that speckle into one cluster centred on the
robot, and the mission reported the arena explored three seconds in without
moving.

**3. The obvious fix for (2) destroys localisation.** Rewriting `inf` to just
above the mapper's threshold does make Karto trace the ray — and puts a ring of
phantom obstacles around every scan pose, because the scan matcher reads the
*unfiltered* point list, where Karto stores a point at the raw reported range
for exactly those rays. Correlation then peaks when the estimated pose sits on
a previous one. Measured against Gazebo ground truth that was 6 m of drift and
44° of heading in three minutes, on odometry accurate to 3 mm.

`scan_preprocess.py` reports a no-return ray at 20 m and raises `range_max`
above it. That is inside `range_max` so the grid traces it as free space, above
the range threshold so no obstacle is marked at its end, and far outside the
scan matcher's correlation grid so the phantom point is dropped. Drift over a
ten minute run went to 5 cm.

The full reasoning, with the Karto source it was derived from, is at the top of
`scan_preprocess.py`. Do not change those three numbers without reading it.

### Other corrections to the provided material

- `param_nav2.yaml` had no `smoother_server` or `velocity_smoother` section at
  all. Both are lifecycle-managed, so both silently ran on the wall clock in
  simulation, and `velocity_smoother` — which sits in the `cmd_vel` path —
  clamped to a generic robot's 0.5 m/s instead of the burger's 0.22.
- Inflation radius was 1.0 m locally and 0.55 m globally. In a 1 m corridor
  either leaves no zero-cost cell between the walls, so the gradient the
  controller steers down is flat. Both are now 0.28 m.
- `obstacle_layer` and `voxel_layer` were both subscribed to `/scan` in both
  costmaps: every ray marked and raytraced twice for identical results.
- `behavior_server` read `transform_timeout`; Humble declares
  `transform_tolerance`.
- `slam_toolbox.launch.py` drove a lifecycle handshake at a node that is not a
  lifecycle node in Humble, leaving launch blocked in an unbounded
  `wait_for_service` loop. Mapping worked, so the only symptom was a launch
  that would not shut down cleanly.
- The frontier detector's search radius was pinned to the sensor horizon. It
  flood-fills through *known free space*, so that made the robot short-sighted:
  as soon as its immediate pocket was mapped it reported no frontiers. Now 30 m.
- `joy_linux` missing aborted the entire bringup, taking SLAM and perception
  with it. Teleop is now conditional.
- Loop closure thresholds are deliberately stricter than the slam_toolbox
  defaults. A false closure does not degrade the estimate, it teleports it.
