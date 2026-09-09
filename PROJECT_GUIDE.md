# ASR Summer School Challenge — Project Guide

**Autonomous Search-and-Rescue with TurtleBot3. ROS 2 Humble.**

One document: what this is, how to run it, how it works, and what to change before the
final run. If you only read one section, read [Quick start](#2-quick-start) and
[Before the real run](#7-before-the-real-run).

- [1. What this is](#1-what-this-is)
- [2. Quick start](#2-quick-start)
- [3. What the robot actually does](#3-what-the-robot-actually-does)
- [4. The code](#4-the-code)
- [5. What went wrong, and why it was hard to see](#5-what-went-wrong-and-why-it-was-hard-to-see)
- [6. What comes out of a run](#6-what-comes-out-of-a-run)
- [7. Before the real run](#7-before-the-real-run)
- [8. When something goes wrong](#8-when-something-goes-wrong)
- [9. Tests](#9-tests)
- [10. Where everything lives](#10-where-everything-lives)

---

## 1. What this is

The robot must explore an unknown indoor arena on its own inside a fixed time limit, find
AprilTags and record where each one is on the map, produce a 2D occupancy grid and a
semantic map, and be back at its starting point within 50 cm before the deadline.

The course provides bringup, SLAM, Nav2, AprilTag detection and a frontier detector.
Nothing consumed the frontier detector's output and there was no mission logic at all —
that is what this project adds, plus a scan-preprocessing node that turned out to be the
difference between a working system and a broken one.

### Results

Four full 600 s runs in the 20 × 20 m simulated maze, plus a 240 s run on a from-scratch
rebuild. Scored against the arena's own ground truth by `score_report.py`:

| Run | Goals reached | Tags | SLAM drift | Return (true position) | Score |
|---|---|---|---|---|---|
| A (600 s) | 17 / 17 | 4 | 5 cm | 6 cm | **680** |
| B (600 s) | 18 / 19 | 2 | 4 cm | 6 cm | 580 |
| Clean-build smoke (240 s) | 7 / 7 | 3 | 1 cm | 3 cm | 630 |

The base 450 points — map, semantic map, autonomous exploration, on-time return — closes
reliably. Tag count varies because 600 s only covers about half of a 400 m² maze; the real
arena is smaller, so expect more.

---

## 2. Quick start

### 2.1 Build

```bash
cd ~/ASR_YLS/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Takes about a minute from clean. `apriltag`, `apriltag_msgs` and `apriltag_ros` are
vendored under `src/third_party/` and build with everything else, so tag detection works
without `sudo` and without `apt`.

### 2.2 Simulation

Four terminals. Every one of them starts with the same line:

```bash
source ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge/setup_env.sh sim
```

That sources the workspace and sets the middleware to Fast DDS. It matters: your
`.bashrc` points Zenoh at the robot's IP, and Zenoh in client mode aimed at an absent
endpoint fails **silently** — an empty `ros2 topic list` rather than an error. The script
prints everything it set, so a shell behaving oddly can be diagnosed by looking rather
than guessing.

```bash
# 1  Gazebo.  gui:=false runs it headless, which is much faster.
ros2 launch asr_summer_school project.launch.py

# 2  SLAM, scan preprocessing, camera TF, AprilTag, frontier detector
ros2 launch asr_summer_school bringup_simulation.launch.py use_sim_time:=true

# 3  Nav2 + the mission itself
ros2 launch asr_summer_school mission.launch.py use_sim_time:=true \
    mission_duration:=600.0 output_directory:=~/asr_mission_output

# 4  optional: watch it
ros2 launch turtlebot3_bringup rviz2.launch.py use_sim_time:=true
```

Score the finished run against the world file's own ground truth:

```bash
ros2 run asr_summer_school score_report.py --run ~/asr_mission_output
```

### 2.3 On the robot

**On your laptop**, once per session:

```bash
source ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge/setup_env.sh <robot number>
ros2 run rmw_zenoh_cpp rmw_zenohd            # leave this terminal running
```

**Over SSH to the robot** (`ssh students@192.168.10.1<NN>`, password `sesasr`), two
terminals:

```bash
source ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge/setup_env.sh onboard <robot number>
```

`CAMERA_MODEL=realsense` is the default and is what our robot has. The launch
still supports `oakd`, but nothing needs changing for us.

```bash
# terminal 1 — base, LiDAR, camera, AprilTag, SLAM, frontier detector.  No Nav2.
ros2 launch asr_summer_school bringup.launch.py
#   no joypad plugged in?  add  teleop:=false

# terminal 2 — Nav2 and the mission
ros2 launch asr_summer_school mission.launch.py \
    use_sim_time:=false \
    mission_duration:=<seconds the organisers announce> \
    output_directory:=~/asr_mission_output
```

`onboard` differs from the laptop form in one thing that matters: the robot hosts the
Zenoh router, so it must not be configured as a client pointing at itself.

`bringup.launch.py` checks `TURTLEBOT3_MODEL`, `LDS_MODEL` and `CAMERA_MODEL` before it
starts anything and stops with one line naming the missing one. Without that check an
unset `CAMERA_MODEL` surfaces as a `KeyError` from inside `turtlebot3_perception` several
seconds in, with half the stack already up.

Put the robot on the start point and leave it alone. Touching it costs 50 points.

**To stop early and keep the deliverables**, press Ctrl-C in the mission terminal
**once**. The export runs from a `finally` block, so the map and the semantic map are
written even on an interrupt. Two Ctrl-Cs kill it before that.

---

## 3. What the robot actually does

A plain state machine, not a behaviour tree — a BT earns no points here and is slower to
reason about when a run misbehaves with fifteen minutes of lab time left.

```
INIT ──▶ EXPLORE ──▶ RETURN ──▶ FINALIZE ──▶ DONE
```

**INIT.** Wait for Nav2 (`waitUntilNav2Active(localizer='controller_server')` — there is
no AMCL, SLAM provides `map→odom`). Ask TF which base frame actually exists rather than
trusting the config. Record home in **both** the `map` and `odom` frames. Then turn a full
circle on the spot to seed the map: one stationary scan gives slam_toolbox a speckled map
whose unknown gaps the frontier detector reads as frontiers, all clustered onto the
robot's own position.

**EXPLORE.** Every 0.4 s:

1. Re-estimate the cost of getting home — from a path the Nav2 planner actually produced,
   not from straight-line distance, so walls between the robot and the start are counted.
2. If the time left has shrunk to that estimate, cancel and go home.
3. If a goal is in flight: check whether the detector still reports a frontier there. If
   it does not, the robot's own LiDAR filled it in on the way — cancel and count it
   reached. Otherwise let it run until it finishes or exceeds a timeout scaled by how far
   away it was.
4. Otherwise pick a new frontier: rank the shortlist by real planner cost, penalise
   candidates behind the robot, skip anything blacklisted.
5. On arrival, turn a full circle so the 60° camera sweeps the area instead of only the
   direction of travel — but only while there is slack, because a tag is 50 points and
   being a minute late is 180.

If no frontier can be chosen for 25 s the robot spins to refresh the map, up to three
times. If every remaining frontier is blacklisted it drops the whole blacklist and tries
again rather than declare the arena explored — a run that banned its last frontier went
home with 5 % of the arena mapped.

**RETURN.** Drive to the recorded start pose, up to three attempts, clearing costmaps
between them. If SLAM was seen to jump (see [§5](#5-what-went-wrong-and-why-it-was-hard-to-see)),
aim at the odometry anchor instead.

**FINALIZE.** Write everything, from a `finally` block, so it happens whatever went wrong
above. 200 of the available points live in those files.

**Tags**, throughout: `apriltag_ros` broadcasts a TF frame per detected tag named
`tag36h11:<id>`. That frame exists identically in simulation and on the robot, which the
detection *topics* do not. The tag manager reads each frame's parent straight out of TF —
so it needs no camera-frame configuration — transforms into `map` at the detection's own
timestamp, and fuses repeats into a running mean weighted by 1/range² behind an outlier
gate. Closer sightings dominate; a bad one cannot drag a good tag off its position.

---

## 4. The code

Everything we wrote is in
`ros2_ws/src/asr_summer_school_challenge/asr_summer_school/`. About 3,300 lines of Python.

| File | What it does |
|---|---|
| `asr_summer_school/mission_control.py` | The orchestrator. The state machine above, the return budget, the map-jump watchdog, the export |
| `asr_summer_school/mission_clock.py` | Deadline and return-cost estimation |
| `asr_summer_school/frontier_policy.py` | Which frontier to drive to; expiring blacklist, hysteresis, turn penalty. **No rclpy** — unit tested |
| `asr_summer_school/frontier_client.py` | Subscribes to `frontier_centroids` (transient-local QoS, or it silently receives nothing) |
| `asr_summer_school/tag_manager.py` | Tag TF frames in, fused map-frame landmarks out |
| `asr_summer_school/tag_map.py` | Dedup by ID, 1/r² fusion, outlier gate. **No rclpy** |
| `asr_summer_school/scan_preprocess.py` | Rewrites "no return" so SLAM can map open space. Read its header before changing anything in it |
| `asr_summer_school/map_export.py` | Both deliverables plus a PNG overlay. **No rclpy** |
| `asr_summer_school/map_recorder.py` | Holds the newest `/map` |
| `asr_summer_school/geometry.py` | Rigid transforms. **No rclpy** |
| `asr_summer_school/score_report.py` | Scores a finished run offline against the world file's ground truth |
| `asr_summer_school/preflight.py` | Checks the whole stack in one command before a run |
| `config/param_mission.yaml` | Every mission tunable, each with a comment saying what it trades off |
| `launch/mission.launch.py` | Nav2 + the orchestrator. **This is the one you run** |
| `setup_env.sh` | `sim` / `onboard <n>` / `<n>` environment setup |
| `RUNBOOK.md` | Pre-flight checklist and diagnosis table |

Anything that can be tested without a running graph does not import rclpy. That is what
lets 72 tests run in half a second on a laptop with nothing sourced. Six more are C++
gtest cases over the frontier search, run by `colcon test`.

---

## 5. What went wrong, and why it was hard to see

Every item here was silent. None produced an error; each produced a run that
finished cleanly with a low score, which is the expensive kind of bug.

### 5.1 `inf` is not "nothing there" to the mapper

The LiDAR reports `inf` for a ray that hits nothing within range. Karto — the mapper
inside slam_toolbox — throws those away before they can clear anything:

```cpp
// karto_sdk  OccupancyGrid::AddScan
if (rangeReading <= minRange || rangeReading >= maxRange || isnan(rangeReading)) {
  continue;                    // <- inf lands here, and contributes nothing
} else if (rangeReading >= rangeThreshold) {
  ...                          // <- traced as free space, no obstacle marked
}
```

`maxRange` is the scan message's own `range_max`, so `inf >= 3.5` is discarded. A robot
standing in a corridor with both ends open therefore mapped only the metre of wall beside
it — the occupancy grid did not even grow to include the open directions. The frontier
detector then saw unknown cells adjacent to free ones in every direction at once, DBSCAN
merged that speckle into a single cluster centred on the robot, and the mission declared
the arena explored three seconds in, without moving.

### 5.2 The obvious fix for that silently destroys localisation

Rewriting `inf` to just above the mapper's range threshold — 3.45 m against a 3.4 m
threshold — does make Karto trace the ray. Exploration starts working. It also destroys
localisation, which is far more expensive and does not announce itself.

Karto's scan matcher does not read the range-filtered point list. Both
`ScanMatcher::AddScan` and `GridIndexLookup::ComputeOffsets` call `GetPointReadings()`,
whose default is `wantFiltered = false`, and `LocalizedRangeScan::Update` stores an
unfiltered point at the **raw reported range** for exactly the rays outside the threshold.
A finite 3.45 m therefore puts a ring of ~150 phantom obstacles around every scan pose.
Those rings are rigidly attached to their own poses, so correlation peaks when the robot's
estimated pose sits on a previous one, and the estimate gets dragged backwards.

Measured against Gazebo ground truth: **6 m of drift and 44° of heading in three
minutes**, on odometry that was accurate to 3 mm.

**The fix** (`scan_preprocess.py`): report a no-return ray at **20 m** and raise the
published `range_max` to **25 m**. That threads three needles at once:

- `20 < range_max`, so the occupancy grid does not discard the ray, and
  `20 >= range_threshold` (3.4), so it is traced as **free space** out to the threshold
  with **no obstacle** at its end.
- The phantom point lands 20 m from the scan pose — far outside the correlation grid,
  whose half-width is the range threshold plus the search window. `ScanMatcher::AddScan`
  drops points outside the grid, so the matcher never sees it.
- Nav2 clips rays to `raytrace_max_range` before clearing and ignores anything past
  `obstacle_max_range` when marking, so a 20 m reading clears the corridor ahead without
  marking a phantom wall.

Drift went from 6 m to **5 cm**. Do not change those three numbers without reading the
header of `scan_preprocess.py`, which carries the full derivation.

### 5.3 The map's own edge was invisible to the frontier search

`preprocess_frontier_cells` skipped neighbours outside the grid, so a free cell
against the map boundary was never counted as a frontier — though what lies past
the edge is by definition unobserved.

That matters because of how Karto sizes the grid: to the bounding box of the scan
*endpoints*, which excludes the no-return rays, while still rastering those rays as
free space out to the range threshold. Free space is therefore routinely clipped at
the boundary with no unknown margin beyond it, and a robot in open space finds no
frontier at all.

Measured: 87% of a 4 × 3.5 m map known, the other 396 m² of the arena never visited,
`exploration complete` logged after two goals. Treating out-of-bounds as unknown took
the same 300 s mission from 1 tag and 2 goals to **3 tags and 9 goals**. Guarded by
`test/test_frontier_detection.cpp`.

### 5.4 A camera transform we added and did not need

Recorded because it was our own mistake, and the shape of it recurs.

`turtlebot3_description`'s burger URDF has no camera link, so `camera_rgb_frame` — the
frame `apriltag_ros` parents every tag to — looked absent from TF, and
`bringup_simulation.launch.py` grew two static transform publishers to supply it,
with offsets read out of the Gazebo SDF.

That was the wrong file. `robot_state_publisher` in this workspace loads
`turtlebot3_gazebo/urdf/turtlebot3_burger.urdf`, which already carries the whole
chain: `base_link → camera_link → camera_rgb_frame → camera_rgb_optical_frame`.

So the publishers gave `camera_rgb_frame` a **second parent**, with an offset 2.3 cm
out in z and 1.8 cm out in y. tf2 does not warn about a reparent; it serves whichever
transform arrived last. The camera pose was silently non-deterministic — on a quantity
the rubric scores to 15 cm. Both publishers have been removed and the chain verified
single-parented.

The lesson worth keeping: check which file is actually loaded before concluding a frame
is missing, and trust `ros2 run tf2_ros tf2_echo` over reading a URDF.

### 5.5 Everything else that was wrong

All fixed; the list is kept because each one will look like something else when it
reappears. Full detail in [`CLAUDE.md` §8](CLAUDE.md).

- **False loop closures teleport the map.** One moved the pose 3 m and 24° in a single
  graph optimisation, and the robot drove "home" to a point 0.84 m from the real start —
  outside the 50 cm circle, 150 points gone. Loop-closure thresholds are now stricter than
  the slam_toolbox defaults, and the mission watches `map→odom` for the discontinuity and
  falls back to an odometry anchor for the return.
- **A dead `scan_preprocess` looked like a healthy system.** It crashed at startup on an
  undeclared parameter; the map silently stopped at 4 × 3 m and the run scored one tag.
  The launch now takes the whole stack down if that node exits, and a test asserts every
  parameter read by any of our nodes is also declared.
- **Tag positions fused against stale transforms.** When the exact-timestamp lookup
  failed, the tag manager fell back to the latest transform. During the camera sweep the
  robot turns at ~1 rad/s, so half a second of staleness rotated a sighting 30° and put a
  tag 63 cm from its true position. Detections whose exact lookup fails are now dropped —
  they arrive at 10 Hz, so it costs nothing.
- **Goals timing out** because the controller could not transform the robot pose into the
  plan's frame under load. Transform tolerance 0.2 → 0.5 s. Goal success went 14/18 → 17/17.
- **Frontier search radius pinned to the sensor horizon**, which made the robot
  short-sighted and it stopped exploring as soon as its immediate pocket was mapped.
- **Exploration gave up after three seconds** — patience was counted in state-machine
  ticks, not seconds.
- **Nav2 params**: no `smoother_server` or `velocity_smoother` section at all (so
  `velocity_smoother`, which sits in the `cmd_vel` path to the wheels, clamped to a
  generic robot's 0.5 m/s instead of the burger's 0.22); inflation radii that leave no
  clear lane in a 1 m corridor; both costmaps marking and raytracing the same scan twice;
  `behavior_server` reading the Foxy spelling `transform_timeout`.
- **A missing `joy_linux` aborted the entire bringup**, taking SLAM and perception with
  it. Teleop is now conditional.

---

## 6. What comes out of a run

Written to `output_directory` at the end of every run, including an interrupted one.
There is a complete example in `~/asr_mission_output/example_sim_run/`.

| File | What it is | Points |
|---|---|---|
| `map.pgm` + `map.yaml` | 2D occupancy grid, in exactly the format `nav2_map_server` writes | **+100** |
| `semantic_map.yaml` | Tag IDs and map-frame positions, in the shape the course's own `landmarks.yaml` uses | **+100** |
| `semantic_map.json` | Same tags plus per-tag provenance and full mission metadata | |
| `semantic_map.csv` | One row per tag, for a spreadsheet | |
| `mission_report.json` | Timings, goal counts, map statistics, any detected SLAM jumps | |
| `mission_overlay.png` | The grid with the tags, the start and the finish drawn on it | |

The semantic map is written three ways because the required format is specified nowhere in
any handout. Hand over whichever they ask for — **and ask them early, it is 100 points.**

The grid is written from the raw `OccupancyGrid` we already hold, not through a service
call to a node that has to still be alive and responsive at minute twenty.

---

## 7. Before the real run

Three numbers, in order of how much they matter.

**1. `mission_duration`** — the deadline in seconds, from the organisers, passed on the
command line. On-time is +150 and one minute late is −30: a 180-point swing, worth 3.6
tags. It is worth more than everything else on this page combined. **If in doubt, set it
shorter than announced.**

**2. `laser_max_range`** — the LiDAR horizon in metres. The default 3.5 is right for the
LDS-01 and the simulator; the **LDS-02 fitted to these robots reaches further**. Check it:

```bash
ros2 topic echo /scan --field range_max --once
```

If it disagrees, `scan_preprocess` prints a warning naming the value to use. Pass
`laser_max_range:=<value>` to `bringup.launch.py` and set `max_laser_range` in
`config/param_slam_toolbox.yaml` about 0.1 m below it. Getting this wrong is not dangerous
— readings past the configured horizon are treated as "nothing there", which never invents
an obstacle — but the robot maps less per scan than it could.

**3. `home_tolerance`** (`config/param_mission.yaml`, default 0.35 m). The rule is a
circle of **50 cm radius** around the starting point. Aiming at 35 cm leaves 15 cm for the
drift between where TF thinks the robot is and where it physically is; measured drift is
3–8 cm. Do not raise it to 0.50 — that puts the run on the boundary of a 150-point swing.

### Finishing early, and the one cut not to make

The rubric pays nothing for a short run, but a short run wins a tie on time. Two launch
arguments:

```bash
stop_after_tags:=11    # leave the moment 11 tags are in hand (only if the count is known)
scan_rotation:=0.0     # drop the camera sweep, ~8 s per goal
```

`stop_after_tags` works: 2 tags found at t+79 s, home by t+107 s of a 600 s window.

`scan_rotation:=0.0` is the tempting cut and the wrong one. Spinning measured 48% of one
516 s run, but across four full runs removing or halving it left the tag count unchanged at
4 and cost the *localisation* — accuracy 30 → 0, return 3 cm → 21 cm — because a full turn
feeds slam_toolbox about thirty well-constrained pose-graph nodes from a known position.

| Camera sweep | Tags | Accuracy | Return | Total |
|---|---|---|---|---|
| Full turn (2π, default) | 5 / 4 | +30 | 0.05 / 0.03 m | **730 / 680** |
| Half turn (π) | 4 | +0 | 0.21 m | 650 |
| None | 4 | +0 | 0.21 m | 650 |

The real waste was *waiting*: the no-frontier recovery sat idle 25 s before each of three
turns. It now waits 6 s and turns once.

Everything else lives in `config/param_mission.yaml` with a comment saying what it trades
off. If the tag count is low and there is time to spare, `scan_rotation` and
`scan_rotation_min_slack` control the camera sweep at each frontier.

### Pre-flight

One command, after the bringup:

```bash
ros2 run asr_summer_school preflight.py            # after bringup
ros2 run asr_summer_school preflight.py --nav2     # after the mission too
```

It samples every topic the mission consumes with the QoS the publisher actually uses,
walks the TF chain, and checks the invariants that fail silently — the no-return
encoding localisation depends on, whether the colour camera frame is reachable from
`map`, whether the frontier detector is producing anything at all. Each failure prints
what to do about it, and the exit status is 0 only when nothing failed.

Every check corresponds to something that actually broke while this was being built.

Two things it cannot check:

- [ ] **Batteries.** LiPo: 3S is 9.6–12.6 V, 4S is 12.8–16.8 V. Below minimum destroys
      the pack.
- [ ] **Hold a tag36h11 in front of the camera and re-run.** It warns rather than fails
      with no tag in view, because that is normal — but it is the only end-to-end
      confirmation of the detection path.

Then drive it a metre with the pad and watch the map in RViz.

## 8. When something goes wrong

| Symptom | Cause | What to do |
|---|---|---|
| `ros2 topic list` empty | Middleware mismatch, or a stale daemon | Check `RMW_IMPLEMENTATION` on **both** ends; `pkill -9 -f ros && ros2 daemon stop` |
| "exploration complete" within seconds | `/frontier_centroids` silent, or every centroid rejected | The mission logs the reason. `ros2 topic echo /frontier_centroids --once` |
| Exploration stops early, map is a small box | The frontier search found nothing. Run `preflight.py` — it reports the centroid count |
| Map stops growing / robot explores a 4 m box | `/scan_filtered` dead | `ros2 topic hz /scan_filtered`; check the bringup log for a `scan_preprocess` traceback. The whole stack reads that topic |
| Map smears, walls double | False loop closure | The mission logs `SLAM moved the map by ...`. See below |
| Tag IDs right, positions nonsense | Optical-frame convention | `tag_manager` logs which frame it parented to and whether it corrected. Force with `optical_correction:=on\|off` |
| Robot never moves, Nav2 active | No path to any frontier | Check the global costmap has free space |
| Pad publishes `/joy`, robot still | Deadman button | `enable_button: 5` in `config/param_teleop.yaml` |
| `/dev/ttyACM0` error | Permissions | `sudo usermod -aG dialout students`, reboot |

**The SLAM jump warning.** If you see

```
SLAM moved the map by 3.06 m / 24 deg at t+434s - that is a graph
re-optimisation, not motion.
```

the scan matcher closed a loop against the wrong place, and everything expressed in the
map frame moved with it — including the recorded start pose. The mission detects this and
aims the return using the odometry anchor instead, which drifts but cannot teleport. The
run is still scoreable; the map and the tag positions in it are only as good as the last
re-optimisation. `mission_report.json` records every jump under `map_jumps`.

---

## 9. Tests

**72 Python tests**, no ROS and no simulator, in under a second:

```bash
python3 -m pytest ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge/asr_summer_school/test -q
```

plus **6 C++ gtest cases** over the frontier search, run by `colcon test`:

```bash
cd ~/ASR_YLS/ros2_ws && colcon test --packages-select asr_summer_school \
  && colcon test-result --test-result-base build/asr_summer_school
```

Together they cover the exploration policy, tag fusion, the deadline budget, the export
formats, the scan-range invariants localisation depends on, the frontier search's
boundary handling, and a set of configuration checks that
would have caught several of the defects above — that every managed Nav2 node can be told
the clock, that the costmaps read the same scan the mapper does, that inflation leaves a
lane in a 1 m corridor, and that every parameter any of our nodes reads is also declared.

**Run them after any change to the mission logic.**

---

## 10. Where everything lives

```
~/ASR_YLS/
  CLAUDE.md              team brief: scoring, reasoning, all 15 defects in full
  PROJECT_GUIDE.md       this file
  ros2_ws/src/
    asr_summer_school_challenge/
      RUNBOOK.md         pre-flight checklist and diagnosis table
      setup_env.sh       environment setup: sim | onboard <n> | <n>
      asr_summer_school/ our package — all our code is here
      VENDORED.md        upstreams and pinned commits for the three below
      laser_filters/     vendored upstream, do not edit (no longer in the pipeline)
      turtlebot3_perception/    vendored upstream, do not edit
      turtlebot3_simulations/   vendored upstream, do not edit
    third_party/         vendored apriltag stack, builds with the workspace
~/asr_mission_output/
  example_sim_run/       a complete set of deliverables from a scored run
```

All our code goes in `asr_summer_school`. The other four directories are upstream code,
checked in rather than referenced as submodules so that a single `git clone` gives a
teammate a workspace that builds — verified by cloning this repository from scratch and
building all ten packages in 53 seconds. **Do not edit them**; `VENDORED.md` records where
each came from and how to take an update.

### Git

**One repository.** `github.com/yousefh112/ASR_YLS`, branch `main`. A teammate needs:

```bash
git clone https://github.com/yousefh112/ASR_YLS.git ~/ASR_YLS
cd ~/ASR_YLS/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

That is the whole setup. No `--recurse-submodules`, no submodule init, nothing to forget.

It did not used to be. The challenge code lived in a second git repository nested inside
this one, recorded only as a commit hash with no `.gitmodules` mapping — and those commits
existed on one laptop and nowhere else, because its `origin` is the course's repository,
which we cannot push to. Cloning this project gave you an **empty** directory and a
`git submodule status` that failed. None of the mission code reached anyone.

It is now absorbed into this repository as tracked content, with its commit history
preserved through `git subtree`, and the three upstream packages it referenced
(`laser_filters`, `turtlebot3_perception`, `turtlebot3_simulations` — 3.4 MB together) are
checked in beside it. `VENDORED.md` records the upstream URL and pinned commit for each,
so taking an update is still a reviewable diff.

Verified by cloning this repository into an empty directory and building all ten packages
from nothing in 53 seconds, then running the 72 offline tests in the clone.

Bundles of both repositories as they were before the restructuring are in
`~/ASR_YLS_backup/`, in case anything needs to be recovered.

---

## Still open with the organisers

1. **Semantic map format.** Worth +100 and specified nowhere. We export YAML, JSON and CSV
   to cover it, but ask. Highest priority.
2. The deadline duration for the final run, and the physical arena layout.
3. Our robot number, which sets both `ROS_DOMAIN_ID` and the NUC address.
