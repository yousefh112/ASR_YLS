from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
	use_sim_time = LaunchConfiguration("use_sim_time", default="true")
	# One number for the real sensor horizon: it caps what slam_toolbox rasters
	# and how far the frontier detector is allowed to propose targets.
	laser_max_range = LaunchConfiguration("laser_max_range", default="3.5")

	slam_toolbox = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'slam_toolbox.launch.py']
			)
		),
		launch_arguments={'use_sim_time': use_sim_time,
		                  'laser_max_range': laser_max_range}.items()
	)

	# Off by default here.  joy_linux is not installed on every development
	# machine, and a missing package makes the whole launch file abort, taking
	# SLAM and perception down with it.  The mission is autonomous anyway, and
	# driving the robot by hand costs 50 points.
	teleop = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'teleop.launch.py']
			)
		),
		condition=IfCondition(LaunchConfiguration('teleop', default='false')),
		launch_arguments={'use_sim_time': use_sim_time}.items()
	)

	# No static camera transforms here on purpose.
	#
	# turtlebot3_gazebo/urdf/turtlebot3_burger.urdf - the URDF that this
	# package's own robot_state_publisher.launch.py loads, which is NOT the one
	# in turtlebot3_description - already carries the whole chain:
	#
	#   base_link -> camera_link -> camera_rgb_frame -> camera_rgb_optical_frame
	#
	# so camera_rgb_frame, the frame apriltag_ros parents every tag to, is in TF
	# from the moment the robot spawns.  Publishing it again from here gave that
	# frame a second parent with a different offset (2.3 cm in z, 1.8 cm in y).
	# tf2 does not warn about a reparent, it just serves whichever arrived last,
	# so the camera pose silently became non-deterministic - and tag positions
	# are scored to 15 cm.

	apriltag = Node(
		package='apriltag_ros',
		executable='apriltag_node',
		namespace='/camera',
		name='apriltag',
		output='screen',
		parameters=[
			{'use_sim_time': use_sim_time},
			# apriltag_SIM.yaml, not apriltag.yaml: Gazebo's camera is 1920x1080
			# and the robot's RealSense is 640x480, so matching the robot means
			# matching the detector's effective resolution rather than copying
			# its decimate.  That file explains the arithmetic.
			#
			# A per-parameter override dict here does NOT work - a later dict
			# loses to an earlier params file rather than overriding it, which
			# was verified with `ros2 param get /camera/apriltag
			# detector.decimate` returning the file's value.  Hence a whole
			# separate file.
			os.path.join(get_package_share_directory('asr_summer_school'), "config", "apriltag_sim.yaml"
    )],
		remappings=[('image_rect', 'image_raw')]
	)

	# NOT the sensor horizon.  preprocess_frontier_cells() flood-fills from the
	# robot cell through *known free* cells only, so every frontier it reaches
	# is connected to the robot by traversable space; the radius is a cap on
	# how far along that free space the search may run, not on what the LiDAR
	# can see.  Capping it at 3.5 m makes the robot short-sighted: as soon as
	# the local pocket is mapped it reports "no frontiers" and the mission ends
	# with most of the arena unexplored.  The arena is 20 x 20 m, so 30 m
	# covers it end to end.  Sensor horizon is a separate concern and stays on
	# laser_max_range, which is what slam_toolbox rasters with.
	active_area_radius = LaunchConfiguration(
		'active_area_radius', default='30.0')

	frontier_detection = Node(
		package='asr_summer_school',
		executable='frontier_detection_node_exe',
		name='frontier_detection_node',
		output='screen',
		parameters=[{
			'map_topic': 'map',
			# The node defaults to amcl_pose; this stack runs SLAM, not AMCL,
			# and without this override the robot pose sits at (0, 0).
			'pose_topic': 'pose',
			'epsilon': 0.5,
			'min_points': 3,
			# ~0.6 m of frontier edge at 0.05 m/cell: large enough to ignore
			# single-scan speckle, small enough to keep a doorway.
			'min_frontier_size': 12,
			'active_area_radius': active_area_radius,
			'use_sim_time': use_sim_time
		}]
	)

	# Defect 6: bringup.launch.py runs this on hardware but the simulation
	# bringup omitted it, so /camera/landmarks existed on the robot and not in
	# Gazebo and code written against one interface broke on the other.
	detection2landmark = Node(
		package='turtlebot3_perception',
		executable='detection2landmark',
		namespace='/camera',
		name='detection2landmark',
		output='screen',
		parameters=[{
			'use_sim_time': use_sim_time,
			'robot_base_frame': 'base_footprint',
		}],
	)

	return LaunchDescription([
		DeclareLaunchArgument(
			'teleop', default_value='false',
			description='Start the joypad teleop stack.  Needs joy_linux.'),
		DeclareLaunchArgument(
			'use_sim_time', default_value='true',
			description='Use the Gazebo clock'),
		DeclareLaunchArgument(
			'laser_max_range', default_value='3.5',
			description='Sensor horizon shared by SLAM and the frontier detector'),
		slam_toolbox,
		teleop,
		apriltag,
		detection2landmark,
		frontier_detection
	])

