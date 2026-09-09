# ASR Summer School: Search-and-Rescue Challenge

> **This is the course's original README, kept for the activity description.**
> Our own documentation is [`RUNBOOK.md`](RUNBOOK.md) (what to run),
> [`../../../PROJECT_GUIDE.md`](../../../PROJECT_GUIDE.md) (how it works) and
> [`../../../CLAUDE.md`](../../../CLAUDE.md) (the brief and every defect found).
> Statements below about the software have been corrected where the stack has
> since changed; the sections describing the *challenge* are untouched.

## Activity overview

This repository supports a **search-and-rescue robotics laboratory** for master's students and early PhD students. The activity is carried out in teams using a TurtleBot3 Burger.

The robot is equipped with:

- a 2D LiDAR for mapping and obstacle avoidance;
- an RGB-D camera for AprilTag detection;
- wheel odometry and an IMU for motion estimation;
- ROS 2 software, including `slam_toolbox`, `nav2`, and `apriltag_ros`.

Each team starts in an unknown indoor environment and develops an autonomous system capable of:

1. building a 2D occupancy map;
2. exploring the environment autonomously;
3. detecting as many AprilTags as possible;
4. associating every detection with its unique tag ID;
5. estimating and storing each tag position in the map frame;
6. returning to the starting position before the available time expires;
7. saving both the occupancy map and a semantic map of the detected targets.

The final task is a fixed-time challenge: **find the largest possible number of AprilTags and return home**. A successful solution must therefore balance exploration, perception, navigation, and the time required for a safe return.

## Repository contents

The ROS 2 workspace contains the following packages and support resources:

```text
src/asr_summer_school_challenge/
├── apriltag-imgs/           # AprilTag families and tag-to-SVG utility
├── asr_summer_school/       # Bringup, autonomy, launch, and configuration
├── laser_filters/            # LiDAR filtering package (vendored; no longer in the pipeline)
├── turtlebot3_perception/    # Camera, AprilTag, and landmark support
└── turtlebot3_simulations/   # Fake node, Gazebo, and Ignition simulation
```

### `asr_summer_school`

`asr_summer_school` is an `ament_cmake` ROS 2 package (not `ament_python` — every runnable
Python node must be listed in `install(PROGRAMS ...)` in `CMakeLists.txt` or `ros2 run` will
not find it) that collects the launch files, parameter sets, autonomy nodes, Python utilities, tests, and simulation assets used during the laboratory. Its `CMakeLists.txt` installs the `launch/` and `config/` directories into the package share directory.

```text
asr_summer_school/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── param_nav2.yaml
│   ├── param_slam_toolbox.yaml
│   └── param_teleop.yaml
├── include/asr_summer_school/
│   └── frontier_detection.h
├── launch/
│   ├── bringup.launch.py
│   ├── bringup_simulation.launch.py
│   ├── nav2.launch.py
│   ├── project.launch.py
│   ├── project_ignition.launch.py
│   ├── slam_toolbox.launch.py
│   └── teleop.launch.py
├── src/
│   ├── frontier_detection.cpp
│   └── frontier_detection_node.cpp
├── asr_summer_school/
│   ├── example_nav_to_pose.py
│   └── sensor_monitor.py
├── scripts/generate_apriltag_maze.py
├── test/frontier_detection_test.launch.py
├── models/
└── worlds/
```

The package includes frontier detection, navigation and sensor-monitoring examples, an AprilTag maze generator, and Gazebo models and worlds for simulation.

#### Launch files

- `bringup.launch.py` starts the TurtleBot3 base, SLAM, joystick teleoperation,  RGB-D camera, and AprilTag detector as one integrated system.
- `bringup_simulation.launch.py` starts the corresponding simulation bringup.
- `slam_toolbox.launch.py` starts asynchronous `slam_toolbox` and, ahead of it,
  `scan_preprocess.py`, which republishes `/scan` as `/scan_filtered`.
  *(Corrected: slam_toolbox is not a lifecycle node in Humble, so nothing manages its
  lifecycle; and the `laser_filters` chain that used to sit here could not express "no
  return" in a way Karto accepts — see PROJECT_GUIDE §5.)*
- `nav2.launch.py` starts the Nav2 navigation stack. It supports mapping or localization, namespaces, composition, respawning, simulation time, and a custom parameter file.
- `project.launch.py` starts the project-specific laboratory configuration in Gazebo Classic, on the `hard_maze_apriltag.world` maze.
- `project_ignition.launch.py` starts the same maze under Ignition/gz, on `hard_maze_apriltag_ignition.world`. It goes through `turtlebot3_ignition`, so remove that package's `COLCON_IGNORE` and rebuild before using it.
- `teleop.launch.py` starts `joy_linux` and `teleop_twist_joy`, allowing the TurtleBot3 to be driven with a game controller.

#### Configuration files

- `param_slam_toolbox.yaml` configures the LiDAR binning filter and `slam_toolbox`, including frames, filtered scan topic, map resolution, scan matching, and loop closure.
- `param_nav2.yaml` configures localization, behavior-tree navigation, controller and planner servers, costmaps, obstacle processing, recovery behaviors, and velocity limits for the TurtleBot3.
- `param_teleop.yaml` defines the joystick axes, enable button, and linear and angular velocity scales.

#### Simulation assets

`worlds/` holds the maze in three pieces: `hard_maze_base.world`, the untagged Gazebo Classic maze, and the two tagged worlds built from it, `hard_maze_apriltag.world` for Gazebo Classic and `hard_maze_apriltag_ignition.world` for Ignition/gz. `scripts/generate_apriltag_maze.py` emits both tagged worlds from a single tag placement, so the same tag IDs sit at the same poses whichever simulator is used.

`models/` holds one directory per tag, each serving both simulators:

- `model.sdf` describes the plate with an Ogre material script, which Gazebo Classic renders, after [koide3/gazebo_apriltag](https://github.com/koide3/gazebo_apriltag);
- `model_gz.sdf` describes it with a PBR albedo map, which Ignition/gz needs since Ogre2 does not read Classic's material scripts, after [rickarmstrong/gazebo_apriltag](https://github.com/rickarmstrong/gazebo_apriltag) (`harmonic` branch);
- both read the same texture under `materials/textures/`, and `model.config` offers each SDF version so every simulator is handed the file it can parse.

The package environment hook exports `models/` and `worlds/` on `GAZEBO_MODEL_PATH`, `IGN_GAZEBO_RESOURCE_PATH`, and `GZ_SIM_RESOURCE_PATH`, so the tags resolve in either simulator after sourcing the workspace.

The package also contains `frontier_detection`, its ROS 2 node entry point, Python examples for navigation and sensor monitoring, and a launch test for frontier detection.

### Supporting packages

- `laser_filters` is still built but is **no longer in the pipeline**.
  `asr_summer_school/scan_preprocess.py` publishes `/scan_filtered` instead, because the
  encoding of a no-return ray has to satisfy three constraints at once that no
  `laser_filters` plugin can express — it must rewrite `range_max` as well as the ranges.
  Both `slam_toolbox` and both Nav2 costmaps consume `/scan_filtered`.
- `turtlebot3_perception` provides the camera and AprilTag launch support used by the main bringup file. Its configuration includes AprilTag, landmark, OAK-D, and RViz settings, and its Python nodes convert detections to landmarks, simulate landmarks, and process laser scans into lines.
- `landmark_msgs` defines `Landmark` and `LandmarkArray` messages for representing detected landmarks.
- `turtlebot3_simulations` provides the TurtleBot3 fake node and Gazebo simulation packages. The source tree also includes an Ignition simulation package, currently marked with `COLCON_IGNORE`.
- `apriltag-imgs` contains the supported AprilTag families and the `tag_to_svg.py` conversion utility.

The current package supplies the robot bringup, mapping, navigation, teleoperation, and perception foundations. The autonomous exploration policy, unique-tag management, transformation and storage of detections in the map frame, semantic-map export, and timed return-to-start behavior are the main components to be developed as part of the challenge.
