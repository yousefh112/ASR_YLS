# Runbook — Autonomous Search-and-Rescue, TurtleBot3

Everything needed to build, run and debug the mission, in the order you need it.
Written to be followed at 14:00 on competition day without reading anything else.

- [0. The brief, and what the robot and the simulation do not share](#0-the-brief)
- [1. Build](#1-build)
- [2. Run in simulation](#2-run-in-simulation)
- [3. Run on the robot](#3-run-on-the-robot)
- [4. What comes out](#4-what-comes-out)
- [5. Tuning for the real arena](#5-tuning-for-the-real-arena)
- [6. Diagnosing a bad run](#6-diagnosing-a-bad-run)
- [7. What was built and why](#7-what-was-built-and-why)

---

## 0. The brief

**The real run: 4 minutes, 12 tags, about 40 m².** Everything in `param_mission.yaml`
is tuned to those three numbers.

That arena is a tenth the area of the practice maze and the window is 40% as
long, and it is a *different problem*, not a smaller one:

| | practice maze | real arena |
|---|---|---|
| area | 400 m² | **40 m²** |
| window | 600 s | **240 s** |
| tags | 11 | **12** |
| LiDAR from a standstill | 9% of it | **96% of it** |
| furthest point from home | ~14 m | **~4.4 m** |
| drive home | ~100 s | **~25 s** |

In 40 m² the map finishes almost immediately, so **exploration stops being the
constraint**. What is left is pointing a 69° camera at twelve tag faces inside
240 seconds, which is a viewpoint problem — so `patrol.py` stops being a
fallback for when frontiers run dry and becomes the main behaviour of the run.

`stop_after_tags` is set to **12** for the same reason. It is only safe to end a
run early when the true tag count is known, and now it is: the moment all twelve
are in hand the robot goes home and banks the +150 rather than risking the clock.

### The simulation is for the algorithm, not the arena

The simulation still runs the 20 × 20 m practice maze. It is there to exercise
the code paths — exploration, patrol, the return, the exports — **not** to
predict the score. A 240 s run of that maze finds three or four tags, and that
is the maze being ten times too big, not the stack failing.

### What the robot and the simulation genuinely do not share

These are not tuning preferences. They are different hardware, and every one of
them is a place where a number that is right in one is wrong in the other.

| | simulation | real robot | set in |
|---|---|---|---|
| LiDAR | Gazebo LDS-01, 3.5 m, 5 Hz | **LDS-02, 8.0 m**, 5 Hz | `bringup*.launch.py`, from `LDS_MODEL` |
| camera | 1920×1080, 59° HFOV, no motion blur | **RealSense 1280×720, 69° HFOV**, rolling shutter | `camera.launch.py` (robot only) |
| detector decimate | 2.0 | **1.0** | `apriltag_sim.yaml` / `apriltag.yaml` |
| effective decode range | ~7 m | **~8 m** | follows from the two rows above |
| clock | `/clock` from Gazebo | wall clock | `use_sim_time`, default **false** |

The camera rows are the ones to watch. Gazebo renders **no motion blur and no
rolling shutter**, so it cannot tell you anything about whether a 1.9 rad/s sweep
smears tags past decoding — that is a hardware check, in
[3.3](#the-four-things-simulation-structurally-cannot-tell-us).

The decimate split exists so the two are comparable at all: the same `decimate`
against two different sensor resolutions is two different detectors. What is
matched is the *effective* resolution, not the parameter.

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
    mission_duration:=240.0 output_directory:=~/asr_mission_output/manual

# 4  optional: watch it
ros2 launch turtlebot3_bringup rviz2.launch.py use_sim_time:=true
```

**Give every run its own subdirectory**, as above. `output_directory` is written
into literally, so pointing several runs at `~/asr_mission_output` itself leaves
their files loose in the parent, mixed in with the tagged folders `sim_run.sh`
creates — and a run then looks like it produced nothing when in fact its results
are sitting one level up. The last line the mission prints is the absolute path
it used; if in doubt, read that.

Note also that this three-terminal form prints to the terminal and keeps no
`mission.log`. `sim_run.sh` below keeps one, which is why it is the better way
to iterate.

Score the finished run against the arena's own ground truth:

```bash
ros2 run asr_summer_school score_report.py --run ~/asr_mission_output/manual
```

### One command instead of three

For iterating, `sim_run.sh` does all of the above headless and scores the result:

```bash
cd ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge
./sim_run.sh comp 240                            # the real window: 4 minutes
./sim_run.sh long 600                            # the maze at its own scale
./sim_run.sh nosweep 240 scan_rotation:=0.0      # extra args go to mission.launch.py
./sim_stop.sh                                    # kill a stack left running
```

**A 240 s run of this maze finds three or four tags, and that is correct.** The
maze is 400 m² and the real arena is 40; four minutes buys about 50 m of driving,
which covers a tenth of it. Use the 240 s form to check that the *behaviour* is
right — that patrol takes over when frontiers run dry, that the return fires on
time, that all six files are written — and the 600 s form when you want the maze
explored properly. Neither predicts the score on the day.

Logs and deliverables land in `~/asr_mission_output/<tag>/`. Measured scores and a
reference set of deliverables are in [`results/`](results/).

---

## 3. Run on the robot

### 3.1 Your laptop, once per session

```bash
source ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge/setup_env.sh <robot number>
```

**No router on the laptop.** This shell is configured as a Zenoh *client* whose
endpoint is the robot, so the router has to be running on the robot — see
[3.2](#32-on-the-robot-over-ssh). Starting `rmw_zenohd` here instead listens on
the laptop, the client still finds nothing at the robot's address, and it fails
as `Unable to connect to any of [tcp/192.168.10.1NN:7447]`.

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

**Start the router here, before anything else, and leave it running:**

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd            # its own SSH session
```

Nothing on either machine discovers anything until this is up — the laptop is a
client aimed at this robot's `:7447`, and with no router listening there every
`ros2` command on the laptop either fails to connect or returns an empty topic
list. This is the single most likely reason for "nothing works" at the start of
a session.

`bringup.launch.py` checks all four of those before starting anything and stops
with a message naming the missing one. Without that check an unset
`CAMERA_MODEL` surfaces as a `KeyError` from inside `turtlebot3_perception`
several seconds in, with half the stack already up.

**Use the two logging wrappers, not the bare launches.** They are the same
commands with everything they print kept, plus a snapshot of the graph either
side of the run and a single tarball at the end. A run whose only record was a
terminal that has since been closed cannot be diagnosed afterwards, and there is
no second attempt on the day.

```bash
./robot_bringup.sh myrun                        # terminal 1, leave running
./robot_run.sh     myrun <seconds>              # terminal 2
```

Both take the same run tag, and everything lands in `~/asr_mission_output/myrun/`.
The bare launches below are what those wrappers run, for when something needs
driving by hand.

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
    mission_duration:=240.0 \
    output_directory:=~/asr_mission_output/<run name>
```

`use_sim_time` is no longer passed: it defaults to false, which is what the robot
needs. 240 is the announced window — pass a different number only if the
organisers change it. Give the run its own subdirectory, or its files land loose
in the parent alongside every other run's.

The robot sweeps the camera once on the spot — for tags, not for the map; a
stationary turn feeds slam_toolbox nothing — then explores. Put it at the
start point and leave it: touching it costs 50 points.

**To stop early and still keep the deliverables**, Ctrl-C the mission terminal
once. The export runs from a `finally` block, so the map and the semantic map
are written even on an interrupt. Two Ctrl-Cs kill it before that.

### 3.2b What a run leaves behind, and what to send for analysis

`robot_run.sh` finishes by printing the path to a single tarball:

```text
one file to hand over: ~/asr_mission_output/myrun/myrun-20260910-1530.tar.gz
```

**That tarball is the thing to send.** It contains everything needed to work out
what a run did without having been there:

| File | Why it matters |
|---|---|
| `mission.log` | Every goal, every sweep, the frontier reasoning, the return budget at each decision, and the final `mission over` line |
| `bringup.log` | Where the quiet failures surface — the LiDAR's real range, `scan_preprocess` warnings, which frames AprilTag parented to |
| `diagnostics.txt` | The graph before and after: node and topic lists, publish rates, `/scan` vs `/scan_filtered` `range_max`, the TF chain, the pre-flight report, and the environment variables |
| `mission_report.json` | Timings, goal and patrol counts, camera sweeps, map statistics, any detected SLAM jump, the final distance from home |
| `map.pgm` / `map.yaml` | The occupancy grid deliverable, **+100** |
| `semantic_map.yaml/json/csv` | The tag deliverable, **+100** |
| `mission_overlay.png` | The grid with tags, start and finish drawn on — the fastest way to see what happened |

Copy it off the robot with:

```bash
scp students@192.168.10.1<NN>:~/asr_mission_output/myrun/myrun-*.tar.gz .
```

Two numbers in `diagnostics.txt` are worth reading yourself before sending it,
because they answer questions this repo has been guessing at:

- **`echo /scan range_max`** — the LiDAR's true horizon. `3.5` means an LDS-01
  and the current `max_laser_range` is right; anything larger means an LDS-02
  and there is coverage being left on the table ([section 5](#5-tuning-for-the-real-arena)).
- **`echo /scan_filtered range_max`** — should be `25.0`. If it is not, the scan
  preprocessing is not in the pipeline, and that is the difference between a
  working system and one that maps a four-metre box.

There is no `score_report.py` step on the robot. It scores against tag
coordinates parsed out of the Gazebo world file, and no such file exists for a
physical arena — running it would score the arena we are not in.

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

#### The four things simulation structurally cannot tell us

Every number in `results/` comes from Gazebo, and Gazebo differs from the robot
in ways that are invisible until the robot runs. These four checks **must**
happen on hardware, in this order, because each is worth more than anything the
simulation measures.

**1. The no-return encoding.** Gazebo reports `inf` for a ray that hits nothing;
a real LDS reports `0.0`. Both must become free space — if `0.0` is treated as a
near obstacle the mapper discards every open direction and the mission ends
seconds in with the arena "explored". `scan_preprocess` handles both
(`zero_is_no_return`, default true) and `preflight` checks it. Confirm the
no-return value appears in open space:

```bash
ros2 topic echo /scan_filtered --field ranges --once | tr ',' '\n' | sort -u | tail -3
```

Expect the 20 m no-return value. If every open ray is `nan`, this is the bug.

**2. Tag decode range.** `config/apriltag.yaml` sets `decimate: 1.0`, because at
the vendored `2.0` a 16 cm tag stops decoding at about 2.5 m while the mission
gates detections at 5 m. The Gazebo camera is 1920x1080 with no motion blur, so
it cannot show this either way. Walk a tag backwards and watch where detections
stop:

```bash
ros2 topic echo /camera/detections --field detections
```

Set `max_detection_range` in `param_mission.yaml` to whatever range it actually
reaches. A gate wider than the detector is a gate that admits nothing.

**3. Sweep speed against motion blur.** The camera sweep runs at
`max_rotational_vel: 1.9` rad/s. That number is bounded by blur, **not** by
SLAM — a stationary turn contributes nothing to the map at any speed, see
CLAUDE.md section 2. Gazebo renders no motion blur, so 1.9 is unvalidated on
hardware. Sweep in front of a tag and confirm detections still arrive:

```bash
ros2 topic hz /camera/detections
```

If they drop out, lower `max_rotational_vel` in `param_nav2.yaml` until they
come back. This is the one speed change that can cost tags.

**4. LiDAR horizon.** `max_laser_range` is 3.4 m, which is the LDS-01 and the
simulator. An LDS-02 reaches considerably further, and every metre of horizon is
area mapped per stop. Check what the driver reports:

```bash
ros2 topic echo /scan --field range_max --once
```

Raising it is **not** free — read the header of `scan_preprocess.py` first, and
note that `slam_toolbox.launch.py` overwrites this key, so editing the YAML
alone does nothing.

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
| `coverage_report.txt` | **What the run looked at and what it did not.** Also printed into `mission.log` |

### The coverage report

`score_report.py` needs the Gazebo world file to know where the tags really are,
so it cannot run on the robot — there is no ground truth for a physical arena.
The coverage report answers a different question that does not need one, and it
prints identically in simulation and on the robot:

**Was a tag missed because the robot never went there, or because it went there
and never pointed the camera at it?**

Those two have opposite fixes, and no other output distinguishes them: the grid
looks complete, every goal reports success, and the tag is simply absent. So the
report splits the arena into what the LiDAR mapped and what the camera actually
saw — by casting rays from every sweep position, so walls block the view the way
they really do.

```text
LEFT UNEXPLORED
  2.80 m2 of frontier was still open when the run ended.
  The arena was NOT fully explored: there was somewhere left to go
  and the clock, not the map, ended the run.

SEEN BY THE CAMERA
  sweep positions  12
  seen                83.10 m2   (97% of mapped free space, within 6.5 m ...)
  never seen           2.20 m2   total
     of which          0.32 m2   in 1 pocket(s) big enough to drive to:
         0.32 m2 around (0.11, -6.32)
     and               1.88 m2   in slivers smaller than the robot,
                                 mostly grazing angles along walls

TAGS
  found            3 of 12
  missing          0, 1, 2, 3, 5, 7, 8, 10, 11

WHAT TO DO ABOUT IT
  ...
```

Read the last block first. It says which of the two causes was in play, and
therefore which lever is worth pulling:

| What it says | What it means | What to change |
|---|---|---|
| arena not fully explored | ran out of clock | exploration speed; or accept it |
| mapped but never seen | drove past without looking | lower `patrol_spacing`, more sweeps |
| both | both | more time helps, more sweeps help |
| fully explored and fully seen | the tag was in frame and not decoded | detection range, motion blur, or the tag was facing away |

`expected_tags` (12) is what lets it name the *missing* ids rather than only
count what was found. Set it to 0 if the count is ever unannounced.

The semantic map is written three ways because the required format is specified
nowhere in the handouts. Hand over whichever the organisers ask for. **Ask them
early** — it is 100 points.

---

## 5. Tuning for the real arena

Three numbers, in order of how much they matter.

**`mission_duration`** — the deadline, in seconds. **240 for the real run**, and
that is already the default in `param_mission.yaml`. Returning on time is +150
and one minute late is −30, so this is the single most valuable thing to get
right. If the organisers change it on the day, pass `mission_duration:=<seconds>`
rather than editing the file, and if in doubt set it *shorter* than announced.

**`laser_max_range`** — the LiDAR horizon in metres. On the robot this now
defaults from `LDS_MODEL` (LDS-01 3.5, **LDS-02 8.0**, LDS-03 12.0) rather than
being hardcoded to the simulator's 3.5, which threw away more than half of the
real sensor's reach and produced a map that grew far more slowly than the
hardware allowed. Nothing warned about it, because a shorter horizon is not an
error. Verify it anyway:

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
| `Unable to connect to any of [tcp/192.168.10.1NN:7447]`, or an empty topic list from the laptop | No Zenoh router running **on the robot**. The laptop is a client aimed at the robot's `:7447`; a router started on the laptop listens in the wrong place | SSH to the robot and run `ros2 run rmw_zenoh_cpp rmw_zenohd`, leave it up, then retry. See [3.2](#32-on-the-robot-over-ssh) |
| Mission starts, timer never advances, robot never returns | `use_sim_time` true on the robot, so every node waits on a `/clock` nothing publishes | `mission.launch.py` now defaults it to false. If overridden, drop `use_sim_time:=true` |
| Robot explores fine but finds few tags | Detector range shorter than `max_detection_range`, or the sweep is blurring | Walk a tag back from the camera and watch `/camera/detections` for the real range; if detections vanish only while sweeping, lower `max_rotational_vel` |
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
