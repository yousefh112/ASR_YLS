# ASR Summer School Challenge

**Autonomous Search-and-Rescue with TurtleBot3.** ROS 2 Humble.

This file is the team brief and the context file for Claude Code. Anything the team agrees on
should be edited in here rather than kept in someone's head.

**Operational detail lives in [`ros2_ws/src/asr_summer_school_challenge/RUNBOOK.md`](ros2_ws/src/asr_summer_school_challenge/RUNBOOK.md)** —
build, run, deploy, pre-flight checklist, diagnosis table. This file is the brief and the
reasoning; the runbook is what you follow on the day.

Starter code: https://github.com/SESASR-Course/asr_summer_school_challenge

---

## 1. The task

**The real run: 4 minutes, 12 tags, about 40 m².** Announced by the organisers; everything in
`param_mission.yaml` is tuned to those three numbers, and `RUNBOOK.md` section 0 spells out what
they change.

That arena is a tenth the area of our practice maze and the window is 40% as long, which makes it
a *different* problem rather than a smaller one. In 40 m² the LiDAR maps 96% of the arena from a
standstill, so exploration finishes almost at once and **the run becomes a viewpoint problem**:
pointing a 69° camera at twelve tag faces in 240 s. `patrol.py` is therefore the main behaviour of
a real run, not the fallback it is in the maze. And because the tag count is now known,
`stop_after_tags` is set to 12 — the one case where ending early is right, since it banks the +150
return the moment there is nothing left to find.

The simulation still runs the 20 × 20 m maze, deliberately: it is there to exercise the code, not
to predict the score. A 240 s run of it finds three or four tags, and that is the maze being ten
times too big.

Explore an unknown indoor environment autonomously inside a fixed time limit. Detect as many
AprilTags (tag36h11) as possible. Associate each with its unique ID and its position in the `map`
frame. Produce a 2D occupancy grid and a semantic map. Avoid obstacles. Return to the starting
position within 50 cm before the deadline.

Robot: TurtleBot3 Burger. LDS-02 2D LiDAR, **Intel RealSense** RGB-D camera, OpenCR board for
motors, IMU, encoders and battery, Intel NUC as robot PC. Middleware is `rmw_zenoh_cpp`, not the
Fast DDS default.

---

## 2. Scoring, and what it means for the plan

| Category | Item | Points |
|---|---|---|
| Deliverables | 2D occupancy grid map | +100 |
| Deliverables | Semantic map | +100 |
| Autonomy | Complete exploration, no human intervention | +100 |
| Autonomy | Return to start, within deadline | +150 |
| Autonomy | Return, 30 s past deadline | 0 |
| Autonomy | Return, 1 min past | -30 |
| Autonomy | Return, 2 min past | -70 |
| Perception | Unique tag detection | +50 each |
| Perception | Localization accuracy, error < 15 cm | +30 (capped, total) |
| Perception | Localization accuracy, error 15-30 cm | +15 (capped, total) |
| Penalties | Manual intervention | -50 |
| Penalties | Collision | -20 |

**The return circle is 50 cm.** The robot counts as home when it stops inside a circle of
50 cm radius around its starting point. `home_tolerance` is set to 0.35 m — deliberately
tighter — so that the drift between where TF believes the robot is and where it physically
stands still leaves it inside the circle that scores. Measured SLAM drift is 3–8 cm, so the
15 cm of margin is real.

**Never come home early with time on the clock.** The rubric pays nothing for finishing early
and +50 for every tag, so an unused second is only ever a wasted one. This used to be the other
way round: the mission returned the moment the frontier search ran dry, and the 730-point
reference run ended with **84 s unused, 68% of the arena mapped and six of eleven tags never
found**.

The mistake was treating a finished map as a finished search. **They are not the same job.** The
LiDAR sees 360°, so a wall gets mapped from whatever heading the robot happened to have; the
camera sees 59° in Gazebo and 69° on the robot, so that same wall is only *photographed*
if the robot was pointing at it.
Coverage of positions is not coverage of viewing directions, and only the second one finds tags.

So when frontiers run out and the clock has not, the robot now **patrols**: it drives to known
free space it has not yet swept the camera from, sweeps there, and repeats until the return
budget — not the frontier list — sends it home (`patrol.py`). The only honest reason to stop
early is that every reachable place has been looked at from, and that is now what "exploration
complete" means.

`stop_after_tags` is the one deliberate early exit, and it is now **12**, because the organisers
have announced the count. That is the whole condition for it being safe: ending early is only ever
right when there is provably nothing left to find, and until the number was published, leaving with
tags unfound cost 50 each. Set it back to `0` if the count is ever withdrawn.

**The deadline still dominates the downside.** On-time (+150) versus one minute late (-30) is
a 180-point swing, worth 3.6 tags. The mission timer and the go-home behaviour were built
first and are re-estimated from a real planner path every three seconds.

**Accuracy is the smallest term.** +30 total, not per tag. This is why `max_detection_range`
is set by what can be *decoded* rather than what can be decoded *precisely*.

**The camera sweep is worth keeping. Its *speed* was costing us a minute a run.** Spinning was
measured at 48% of one 516 s run. The A/B below was read as proving the sweep pays for itself
through localisation, and the sweep was left slow on that basis. That reading was wrong, and it
is worth being precise about why, because it is the most expensive mistake in this file.

| Camera sweep | Tags | Accuracy | Return | Total |
|---|---|---|---|---|
| Full turn (2π) | 5 / 4 | +30 | 0.05 / 0.03 m | **730 / 680** |
| Half turn (π) | 4 | +0 | 0.21 m | 650 |
| None | 4 | +0 | 0.21 m | 650 |

The stated mechanism — "a full turn feeds slam_toolbox about thirty well-constrained pose-graph
nodes" — **does not exist**. From `slam_toolbox_common.cpp` (Humble 2.6.10):

```cpp
// check moved enough, within 10% for correction error
const double dist2 = last_pose.SquaredDistance(pose);
if (dist2 < 0.8 * min_dist2 || scan_ctr < 5) { return false; }
```

`shouldProcessScan` gates on **translation only**. There is no heading test, so a robot turning on
the spot has `dist2 ≈ 0` and every scan of that turn is discarded before Karto sees it — our
`minimum_travel_heading` never gets a say. Which is obvious once stated: the LDS-02 is a **360°
sensor**, so rotating it in place acquires no new range data. A stationary turn contributes
**zero** pose-graph nodes and **zero** grid cells.

So the table is a **confound**, not a mechanism: n=1 per condition, and a different sweep setting
sends the robot down a different path, into different loop closures. The localisation difference
was path luck. Three claims elsewhere in the repo inherited the same false premise and are also
wrong — the recovery spin does not "refresh the map", and `initial_spin` does not "seed the map"
(the startup speckle it was blamed on is a `min_pass_through` effect, now fixed by setting it
to 0 — see defect 20).

What the sweep **does** do is point the camera at tags — 59° in Gazebo, **69° on the robot** at the
1280×720 profile — which is worth +50 each and is why it stays. `scan_rotation` is 5.30 rad rather
than 2π because a full circle re-photographs the first frame; 5.30 leaves a little overlap at both
fields of view. But that makes it a camera action, so its speed is bounded by **motion blur, not SLAM** —
and it was running at 1.0 rad/s against the burger's 2.84 ceiling. Now 1.9 rad/s over 5.30 rad
(2π minus one camera FOV re-photographs nothing), which halves every turn in the run.

**The lesson is the general one:** a measured number and an explanation of that number are
different things, and only the first was measured. Before trusting a mechanism in this file,
check it against the source.

Priority order: **timed return, then map export, then coverage, then tag count, then accuracy —
and among equal outcomes, the shorter run.**

---

## 3. What is provided and what we wrote

Provided and working: robot bringup, async `slam_toolbox` in mapping mode, Nav2, joystick teleop,
camera bringup, `apriltag_ros` detection (family 36h11, size 0.16 m), a `detection2landmark` node
giving range and bearing in `base_link`, and a **complete** C++ frontier detector.

The repo README states the gap directly:

> "The autonomous exploration policy, unique-tag management, transformation and storage of
> detections in the map frame, semantic-map export, and timed return-to-start behavior are the main
> components to be developed as part of the challenge."

All five are written and validated end to end in simulation, plus a sixth nobody predicted:

| # | Piece | Where |
|---|---|---|
| 1 | **Mission timer + return home** | `mission_clock.py`, `mission_control.py`. Return cost re-estimated every 3 s from a path the Nav2 planner actually produced. Detects a SLAM map jump and falls back to an odometry anchor |
| 2 | **Map + semantic map export** | `map_export.py`. Written from the raw `OccupancyGrid` in a `finally` block, so an interrupted run still produces both deliverables. Semantic map in YAML, JSON and CSV because the required format is **still unspecified** |
| 3 | **Frontier consumer** | `frontier_policy.py` (no rclpy, unit tested) + `frontier_client.py`. Ranks by real planner cost, blacklists what fails, expires planner refusals |
| 4 | **Tag manager** | `tag_manager.py` + `tag_map.py`. Reads each tag's parent frame from TF so it needs no per-environment configuration; fuses repeats weighted by 1/range² behind an outlier gate |
| 5 | **Orchestrator** | `mission_control.py`. A plain state machine: `INIT → EXPLORE → RETURN → FINALIZE`. A behaviour tree is unrewarded here and slower to debug under time pressure |
| 6 | **Scan preprocessing** | `scan_preprocess.py`. Not foreseen, and the difference between a working system and a broken one — see section 8 |
| 7 | **Tag patrol** | `patrol.py` (no rclpy, 21 offline tests) + `patrol_target` in `mission_control.py`. What the robot does when the map is finished and the clock is not: keeps sweeping the camera from places it has not looked, instead of going home to wait |

Measured in simulation over a 600 s run: SLAM drift **3–8 cm** against Gazebo ground truth, tags
localised to **1–7 cm**, return within 10 cm of the start, all deliverables written.

---

## 4. Timeline

| Day | Lab slot | Duration |
|---|---|---|
| Mon 7 | 14:00-15:30, 16:00-17:30 | 3 h, mostly briefing and tutorials |
| Tue 8 | 14:00-15:30 | 1.5 h |
| Wed 9 | 14:00-15:30, 16:00-17:30 | 3 h |
| Thu 10 | 16:00-18:30 | 2.5 h |
| Fri 11 | 14:00-17:30 | 3.5 h, includes the run |

About **7 hours of real development** across Tuesday to Thursday. The counter: the repo ships the
arena as a Gazebo world with all 11 tags at known coordinates and a `/ground_truth` pose topic, so
the whole stack is built and scored offline outside lab hours. **Treat robot time as validation,
not development.**

---

## 5. Setup

Everything is in one repository. A plain clone produces a workspace that builds — no
`--recurse-submodules`, nothing to initialise, nothing to forget on the day of the run.

```bash
git clone https://github.com/yousefh112/ASR_YLS.git ~/ASR_YLS
cd ~/ASR_YLS/ros2_ws
rosdep install --from-path src --ignore-src -y

# rosdep will NOT pull these, package.xml under-declares them
sudo apt install -y ros-humble-teleop-twist-joy ros-humble-joy-linux \
  ros-humble-nav2-bringup ros-humble-nav2-simple-commander \
  ros-humble-slam-toolbox ros-humble-turtlebot3-gazebo ros-humble-gazebo-ros-pkgs \
  ros-humble-rqt-robot-steering ros-humble-rmw-zenoh-cpp

colcon build --symlink-install
source install/setup.bash
```

`apriltag`, `apriltag_msgs` and `apriltag_ros` are deliberately **not** in that apt list: they are
vendored under `src/third_party/` and build with the workspace, so tag detection works on a machine
with no sudo. Where the system packages exist they are equivalent, and the overlay simply shadows
them.

Pull every morning:

```bash
cd ~/ASR_YLS && git pull
```

### The simulated arena

`worlds/hard_maze_apriltag.world`. 20 x 20 m, 41 box obstacles, 11 static tag36h11 plates at
z = 0.240 m. Ground truth, so we can score ourselves before Friday — `score_report.py` parses it
straight out of the world file rather than duplicating it here, so the two cannot drift apart.

The physical arena will differ, and it will almost certainly be smaller. **Use the sim to validate
the pipeline, never to tune constants.**

---

## 6. Robot operations

| Item | Value |
|---|---|
| Robot wifi | SSID `SESASR_WiFi`, password `LED05_2024` |
| Auditorium wifi | SSID `Auditorium`, password `4ud1t0r1um` |
| Robot PC | **`192.168.10.111`**, host `nuc11`, user `students`, password `sesasr` |
| Our robot number | **11** — sets both `ROS_DOMAIN_ID` and the last octet above |
| Course share | https://naspic4ser.polito.it/files/sharing/ code `CoxrAN2A1` |

Exact commands are in the [runbook](ros2_ws/src/asr_summer_school_challenge/RUNBOOK.md#3-run-on-the-robot).

**Everyone must be on the same middleware.** Zenoh and Fast DDS do not see each other, and the
symptom is an empty `ros2 topic list` rather than an error. Check this before anything else.

### Batteries

LiPo. Check voltage periodically, discharging below minimum destroys the pack.
One cell 3.2-4.2 V, 3S 9.6-12.6 V, 4S 12.8-16.8 V. NUC runs 15-19 V from 4S, OpenCR max 12 V from 3S.

---

## 7. Repo map

```
ros2_ws/src/
  asr_summer_school_challenge/
    asr_summer_school/        the only package authored for this challenge
    laser_filters/            vendored, do not edit.  No longer in the pipeline
    turtlebot3_perception/    vendored, do not edit
    turtlebot3_simulations/   vendored, do not edit
    RUNBOOK.md                how to run it
    sim_run.sh, sim_stop.sh   one-command simulated run, headless and scored
    deploy_to_robot.sh        rsync the workspace to the robot and build it
                              there.  The robot needs it: the bringup is ours
    robot_bringup.sh          runs ON THE ROBOT: drivers, SLAM, camera, apriltag
    mission_run.sh            runs ON THE LAPTOP: Nav2 and the orchestrator, so
                              the deliverables land there and need no copying.
                              Logs, a graph snapshot either side, one tarball
    results/                  measured scores and a reference set of deliverables
    VENDORED.md               upstreams and pinned commits for the three above
  third_party/                vendored apriltag stack, built from source
```

All our code goes in `asr_summer_school`. The other four directories are upstream code,
checked in rather than referenced as submodules so that one `git clone` gives a teammate a
workspace that builds. That used to be enforced by edits being lost on the next `git pull`;
now it is only a convention, so: **do not edit them.** `VENDORED.md` records where each
came from and how to take an update as a reviewable diff.

### Launch files

| File | Starts |
|---|---|
| `bringup.launch.py` | Real robot: base, SLAM, scan preprocessing, teleop, camera, AprilTag, frontier detector. **No Nav2.** Checks the required environment variables first and stops with a clear message if one is missing |
| `bringup_simulation.launch.py` | Same minus base and camera, plus the two static camera transforms Gazebo does not publish |
| `slam_toolbox.launch.py` | `scan_preprocess` + async slam_toolbox |
| `nav2.launch.py` | Nav2 bringup. Started by `mission.launch.py` |
| `mission.launch.py` | Nav2 + the orchestrator. This is the one you run |
| `project.launch.py` | Gazebo with `hard_maze_apriltag.world`. `gui:=false` for headless |
| `teleop.launch.py` | `joy_linux` + `teleop_twist_joy` |

### Perception interfaces

- TF frame per tag, named `tag36h11:<id>` — **the interface we consume**, because it exists
  identically in simulation and on the robot
- `/camera/detections` (`apriltag_msgs/AprilTagDetectionArray`)
- `/camera/landmarks` (`landmark_msgs/LandmarkArray`), range and bearing in `base_link`

---

## 8. Defects found, and one we caused

Mostly in the provided material; item 3 was ours, and is kept for the same reason as the rest.
Ordered by what they cost. Everything here is **fixed**; the list stays because each one is a
trap that will look like something else when it reappears, and because every one of them was
silent — none produced an error, each produced a run that finished cleanly with a low score.

**1. `inf` is not "nothing there" to Karto.** *This was the difference between a working system and
a broken one.* The LiDAR reports `inf` for a ray that hits nothing within range.
`OccupancyGrid::AddScan` discards any reading `>= range_max` before it can trace anything, so open
directions produced no free space and the occupancy grid did not even grow to include them. A robot
in a corridor with both ends open mapped only the metre of wall beside it; the frontier detector saw
unknown cells adjacent to free ones in every direction, DBSCAN merged that speckle into one cluster
centred on the robot, and the mission declared the arena explored three seconds in without moving.
Fixed by `scan_preprocess.py`.

**2. The obvious fix for (1) silently destroys localisation.** Rewriting `inf` to just above the
mapper's range threshold does make Karto trace the ray. It also puts a ring of phantom obstacles
around every scan pose, because the scan matcher reads the *unfiltered* point list where Karto
stores a point at the raw reported range for exactly those rays. Correlation then peaks when the
estimated pose sits on a previous one. Measured against ground truth: **6 m of drift and 44° of
heading in three minutes**, on odometry accurate to 3 mm. A no-return ray is now reported at 20 m
with `range_max` raised above it — inside `range_max` so the grid traces it, above the threshold so
nothing is marked, and far outside the correlation grid so the matcher never sees it. Read the top
of `scan_preprocess.py` before touching those numbers.

**3. Free space at the edge of the map was invisible to the frontier search.**
`preprocess_frontier_cells` skipped any neighbour outside the grid, so a free cell pressed
against the map boundary was never a frontier - even though what lies past the edge is by
definition unobserved. That is not a corner case: Karto sizes the occupancy grid to the
bounding box of the scan *endpoints*, which excludes the no-return rays, while happily
rastering those same rays as free space out to the range threshold. Free space is therefore
routinely clipped at the boundary with no unknown margin beyond it. A robot in open space
then finds no frontier at all. Measured: 87% of a 4 x 3.5 m map known, the other 396 m² of
the arena never visited, "exploration complete" logged after two goals. Fixing it took a
300 s run from 1 tag and 2 goals to **3 tags and 9 goals**. Guarded now by
`test/test_frontier_detection.cpp`.

**4. A camera transform we added and did not need.** *Recorded because it was our own
mistake, and because the shape of it recurs.* `turtlebot3_description`'s burger URDF has no
camera link, so `camera_rgb_frame` — the frame `apriltag_ros` parents every tag to — looked
absent, and `bringup_simulation.launch.py` grew two static transform publishers to supply it.
But `robot_state_publisher` in this workspace loads
`turtlebot3_gazebo/urdf/turtlebot3_burger.urdf`, a *different file*, which carries the whole
chain already: `base_link -> camera_link -> camera_rgb_frame -> camera_rgb_optical_frame`.

The publishers therefore gave `camera_rgb_frame` a second parent with an offset 2.3 cm out in
z and 1.8 cm out in y. tf2 does not warn about a reparent — it serves whichever arrived last —
so the camera pose was silently non-deterministic, on a quantity scored to 15 cm. Both
publishers are gone. **Check which file is actually loaded before concluding a frame is
missing**, and prefer `ros2 run tf2_ros tf2_echo` over reading a URDF.

**5. False loop closures teleport the map.** A maze of parallel corridors seen through a 3.5 m
sensor is exactly the geometry that produces them. One moved the pose 3 m and 24° in a single graph
optimisation, and the robot then drove "home" to a point 0.84 m from the real start — outside the
50 cm circle, so 150 points gone. Loop closure thresholds are now deliberately stricter than the
slam_toolbox defaults, and `mission_control` watches `map→odom` for the discontinuity and falls back
to an odometry anchor for the return.

**6. Frontier search radius was pinned to the sensor horizon.** The detector flood-fills through
*known free space*, so capping it at 3.5 m made the robot short-sighted: as soon as its immediate
pocket was mapped it reported no frontiers and went home. Now 30 m, which is a different concern
from the sensor horizon and is now configured separately.

**7. Exploration gave up after three seconds.** Patience was counted in state-machine ticks, and
eight ticks at 0.4 s is what a map looks like immediately after startup. Now measured in seconds and
backed by recovery spins. A bootstrap rotation gives the camera a look around the start
pose. (It was believed to seed the map as well. It does not — see section 2.)

**8. A single planner refusal blacklisted a frontier permanently.** Early in a run the costmap
between the robot and a frontier is mostly unknown and "no path" means "not yet". Planner refusals
are now counted separately from drive failures and their ban expires.

**9. `param_nav2.yaml` had no `smoother_server` or `velocity_smoother` section.** Both are
lifecycle-managed, so both silently ran on the wall clock in simulation, and `velocity_smoother` —
which sits in the `cmd_vel` path to the wheels — clamped to a generic robot's 0.5 m/s instead of the
burger's 0.22.

**10. Inflation radius was 1.0 m locally, 0.55 m globally.** In a 1 m corridor either leaves no
zero-cost cell between the walls, so the gradient the controller steers down is flat. Both 0.28 m.

**11. Both costmaps ran `obstacle_layer` and `voxel_layer` on the same `/scan`** — every ray marked
and raytraced twice for identical results.

**12. `slam_toolbox.launch.py` drove a lifecycle handshake at a node that is not a lifecycle node**
in Humble, leaving launch blocked in an unbounded `wait_for_service` loop. Mapping worked, so the
only symptom was a launch that would not shut down cleanly — which is the wrong thing to discover
between runs on competition day.

**13. A missing `joy_linux` aborted the entire bringup**, taking SLAM and perception with it.
Teleop is now conditional.

**14. `behavior_server` read `transform_timeout`**; Humble declares `transform_tolerance`. Plus
`enable_groot_monitoring` and eight Foxy-era node sections that no Humble node answers to.

**15. `camera.launch.py` reads `CAMERA_MODEL` with a bare dict lookup** — unset throws a `KeyError`
from inside a submodule, a wrong value silently starts nothing. `bringup.launch.py` now checks all
four required variables up front.

**16. `package.xml` under-declares runtime dependencies.** See the apt list in section 5.

**17. The real LDS-02 emits a varying number of readings per scan, and Karto rejects the
mismatch.** *Hardware-only; found in the arena on the day.* Measured on nuc11: 206-209 readings
per scan on one boot, 203-204 on the next, with `angle_min`, `angle_max` and
`angle_increment` all drifting so that `(max - min) / inc + 1` is not even an integer. Karto
fixes a sensor's reading count from the first scan and rejects every later one that differs,
logging it once (`LaserRangeScan contains 209 range readings, expected 210`) and then
silently. `/map` published nothing: no frontiers, no navigation, no return, no grid. Gazebo's
LiDAR always emits exactly 360. `scan_preprocess` now resamples every scan onto the first
scan's grid, nearest-neighbour by bearing, with uncovered bearings as NaN; in simulation it is an
exact identity. Guarded by `test/test_scan_resample.py`, using the real metadata.

**18. AprilTag was subscribed to a topic nothing published.** *Hardware-only.* The RealSense node
comes up as `/camera`, so it publishes `/camera/color/image_raw`; the vendored relative remap
`camera/color/image_raw` inside namespace `/camera` pointed AprilTag at
`/camera/camera/color/image_raw`, which has no publisher. No image all run, no error, no tag.
It survived one inspection because `ros2 topic list` lists a topic that has only a subscriber,
so both names appeared and looked matched. **Check publisher counts, never topic names.**

**19. `apriltag_ros` defaults `qos_profile` to RELIABLE; the RealSense publishes BEST_EFFORT.**
A reliable subscriber receives nothing from a best-effort publisher, and neither side reports it.
`config/apriltag.yaml` sets `sensor_data`. It is a read-only parameter, so a running bringup keeps
the old value until restarted - which made the fix look ineffective the first time.

**20. `min_pass_through: 2` left the real map at 1 m² from a standstill.** *Hardware-only; cost
all three arena rehearsals.* Karto marks a cell only when `cellPassCnt > min_pass_through`
(`Karto.h`, `OccupancyGrid::UpdateCell`), so 2 means **three** rays through a cell. A robot that
has not moved has one processed scan, and the LDS-02's ~200 beams give three rays only within
~0.5 m. Measured on nuc11: **1.02 m² known** out of an 82 m² grid, while the scan itself held 159
wall hits out to 4.95 m. The frontier detector saw one ring round the robot, the mission rejected
it as "within 0.35 m", patrol had nowhere unswept, and every run spun twice and drove home at
t+19 s. Gazebo's 360 beams gave ~10 m², so simulation never showed it. Now `0`: **12.3 m²** known
from the same standstill.

---

## 9. Conventions

- New Python nodes go in `asr_summer_school/asr_summer_school/`. **Add each to
  `install(PROGRAMS ...)` in `CMakeLists.txt`** or `ros2 run` will not find it. The package is
  `ament_cmake`, not `ament_python`, despite what the README says.
- Logic that can be tested without a graph should not import rclpy. `frontier_policy`,
  `tag_map`, `mission_clock`, `geometry`, `map_export` and `score_report` all follow this, and the
  74 offline tests run in under a second.  The frontier search is C++ and is covered by
  `test/test_frontier_detection.cpp`, run with `colcon test`.
- Use `BasicNavigator` for goal dispatch. Call `waitUntilNav2Active(localizer='controller_server')`,
  **not** the default, because we run SLAM and there is no AMCL.
- Never edit the vendored upstream packages.  See `VENDORED.md`.
- Build with `--symlink-install` so Python edits apply without rebuilding.
- Run the offline tests after any change to the mission logic.

## 10. Open questions for the organizers

1. **Semantic map format.** Worth +100 and specified nowhere. We export YAML, JSON and CSV to cover
   it, but ask. Highest priority.
2. ~~Deadline duration for the final run~~ — **answered: 240 s, 12 tags, ~40 m².**
   The physical arena layout is still unseen.
3. ~~Our robot number~~ — **answered: 11.** `ROS_DOMAIN_ID=11`, NUC at
   `192.168.10.111`.
