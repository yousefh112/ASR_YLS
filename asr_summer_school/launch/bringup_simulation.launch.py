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

	# The simulated camera exists only in the Gazebo SDF.  The burger URDF that
	# robot_state_publisher loads has no camera link at all, so nothing ever
	# connects base_footprint to the frame the images are stamped with, and
	# every map-frame tag lookup fails with "camera_rgb_frame does not exist".
	# Without these two publishers the whole perception half of the mission is
	# silently dead in simulation: apriltag_ros still broadcasts
	# tag36h11:<id> off camera_rgb_frame, but that subtree is detached from
	# the robot.
	#
	# The offset is read straight out of models/turtlebot3_burger/model.sdf,
	# where the camera_rgb_frame link sits at (-0.059, 0, 0.230) in the model
	# frame with no rotation.  The model frame is base_footprint.
	camera_tf = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='camera_rgb_frame_tf',
		output='log',
		arguments=['--x', '-0.059', '--y', '0.0', '--z', '0.230',
		           '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
		           '--frame-id', 'base_footprint',
		           '--child-frame-id', 'camera_rgb_frame'],
		parameters=[{'use_sim_time': use_sim_time}],
	)

	# The optical frame every calibrated driver stamps: z forward, x right,
	# y down.  Gazebo does not publish it, but rviz and anything that expects
	# the REP-103 optical convention does, and it gives the mission stack the
	# same frame name it will find on the robot.
	camera_optical_tf = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='camera_rgb_optical_frame_tf',
		output='log',
		arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
		           '--roll', '-1.5707963267948966',
		           '--pitch', '0.0',
		           '--yaw', '-1.5707963267948966',
		           '--frame-id', 'camera_rgb_frame',
		           '--child-frame-id', 'camera_rgb_optical_frame'],
		parameters=[{'use_sim_time': use_sim_time}],
	)

	apriltag = Node(
		package='apriltag_ros',
		executable='apriltag_node',
		namespace='/camera',
		name='apriltag',
		output='screen',
		parameters=[
			{'use_sim_time': use_sim_time},
			os.path.join(get_package_share_directory('turtlebot3_perception'), "config", "apriltag.yaml"
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
		camera_tf,
		camera_optical_tf,
		apriltag,
		detection2landmark,
		frontier_detection
	])

