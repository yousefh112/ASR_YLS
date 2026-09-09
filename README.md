# ASR_YLS — Autonomous Search-and-Rescue with TurtleBot3

Our entry for the Autonomous Service Robotics Summer School challenge (PIC4SeR,
Politecnico di Torino). ROS 2 Humble, TurtleBot3 Burger.

The robot explores an unknown indoor arena on its own, finds AprilTags and records where
each one is on the map, produces a 2D occupancy grid and a semantic map, and returns to
within 50 cm of its starting point before the deadline.

**Best measured simulation run: 730 points** — 5 tags at 7.6 cm mean error, 19 of 19 goals
reached, home 4.6 cm from the start with 84 s to spare. See [`results/`](ros2_ws/src/asr_summer_school_challenge/results/).

## Start here

```bash
git clone https://github.com/yousefh112/ASR_YLS.git ~/ASR_YLS
cd ~/ASR_YLS/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

One repository, one clone — no submodules to initialise. Then:

```bash
# a full simulated mission, headless, scored when it finishes
cd ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge && ./sim_run.sh
```

## The documents

| | |
|---|---|
| **[PROJECT_GUIDE.md](PROJECT_GUIDE.md)** | Start here if you have not seen this project. What it is, how to run it in simulation and on the robot, how it works, and the findings that decided whether it worked at all. |
| **[RUNBOOK.md](ros2_ws/src/asr_summer_school_challenge/RUNBOOK.md)** | What you follow on the day. Build, deploy, pre-flight, and a symptom-to-fix table. |
| **[CLAUDE.md](CLAUDE.md)** | The team brief and the reasoning: the scoring table and what it implies, what we wrote against what the course provided, and every defect found with what it costs. |
| **[results/](ros2_ws/src/asr_summer_school_challenge/results/)** | Measured scores, and one complete set of deliverables as a reference for what good output looks like. |

## Layout

```
CLAUDE.md  PROJECT_GUIDE.md          the brief, and the handover document
ros2_ws/src/
  asr_summer_school_challenge/
    RUNBOOK.md                       what you follow on competition day
    setup_env.sh                     sim | onboard <n> | <n>
    sim_run.sh  sim_stop.sh          one-command simulated run, and cleanup
    results/                         measured scores and a reference run
    asr_summer_school/               all of our code
    laser_filters/  turtlebot3_*/    vendored upstream, do not edit
  third_party/                       vendored apriltag stack
```

Everything we wrote is in `asr_summer_school`. The other directories are upstream code,
checked in rather than referenced as submodules so a single clone builds — see
[VENDORED.md](ros2_ws/src/asr_summer_school_challenge/VENDORED.md).

## Tests

```bash
cd ~/ASR_YLS/ros2_ws
python3 -m pytest src/asr_summer_school_challenge/asr_summer_school/test -q   # 74, no ROS needed
colcon test --packages-select asr_summer_school                              # + 6 C++ cases
```

Run them after any change to the mission logic.
