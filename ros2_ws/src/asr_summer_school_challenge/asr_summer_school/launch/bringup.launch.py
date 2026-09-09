import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, Shutdown)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


# Environment the robot bringup will not start without.  turtlebot3_bringup and
# turtlebot3_perception both read these with a bare `os.environ[...]`, so an
# unset one surfaces as a KeyError from inside somebody else's launch file
# several seconds in, with the rest of the stack already half up.  Checking
# here turns that into one line naming the variable.
_REQUIRED_ENV = {
    'TURTLEBOT3_MODEL': ('burger',),
    'LDS_MODEL': ('LDS-01', 'LDS-02', 'LDS-03'),
    # camera.launch.py matches on this: unset raises KeyError, and a value it
    # does not recognise silently starts nothing at all, which looks exactly
    # like a camera that failed to enumerate on USB.  Our robot is fitted with
    # the RealSense; 'oakd' stays accepted because the course's launch file
    # supports it, not because we expect to need it.
    'CAMERA_MODEL': ('realsense', 'oakd'),
}


def _check_environment(context, *args, **kwargs):
    problems = []
    for name, allowed in _REQUIRED_ENV.items():
        value = os.environ.get(name)
        if value is None:
            problems.append(
                '{} is not set (expected one of: {})'.format(
                    name, ', '.join(allowed)))
        elif value not in allowed:
            problems.append(
                '{}="{}" is not one of: {}'.format(name, value, ', '.join(allowed)))
    if problems:
        return [LogInfo(msg='\n'.join(
            ['', 'Cannot start the robot bringup:']
            + ['  - ' + p for p in problems]
            + ['',
               'Set them before launching, for example:',
               '  export TURTLEBOT3_MODEL=burger',
               '  export LDS_MODEL=LDS-02',
               '  export CAMERA_MODEL=realsense   # what our robot has',
               ''])),
                Shutdown(reason='required environment variables are missing')]
    return [LogInfo(msg='environment OK: {}'.format(
        ', '.join('{}={}'.format(n, os.environ[n]) for n in _REQUIRED_ENV)))]


def generate_launch_description():
	# The real sensor horizon, shared by slam_toolbox and the frontier detector.
	laser_max_range = LaunchConfiguration('laser_max_range', default='3.5')

	robot_bringup = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('turtlebot3_bringup'), 'launch', 'robot.launch.py']
			)
		)
	)

	slam_toolbox = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'slam_toolbox.launch.py']
			)
		),
		launch_arguments={'use_sim_time': 'false',
		                  'laser_max_range': laser_max_range}.items()
	)

	# On by default on the robot: the pad is the manual stop of last resort.
	# `teleop:=false` if the pad is absent, so a missing joy_linux cannot abort
	# the bringup and take SLAM and the camera down with it.
	teleop = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'teleop.launch.py']
			)
		),
		condition=IfCondition(LaunchConfiguration('teleop', default='true'))
	)

	camera = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('turtlebot3_perception'), 'launch', 'camera.launch.py']
			)
		)
	)

	apriltag = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('turtlebot3_perception'), 'launch', 'apriltag.launch.py']
			)
		)
	)

	frontier_detection = Node(
		package='asr_summer_school',
		executable='frontier_detection_node_exe',
		name='frontier_detection_node',
		output='screen',
		parameters=[{
			'map_topic': 'map',
			'pose_topic': 'pose',
			'epsilon': 0.5,
			'min_points': 3,
			'min_frontier_size': 12,
			# Deliberately not laser_max_range: the frontier search flood-fills
			# through known free space, so this caps how far along already
			# mapped corridors it may look, not how far the LiDAR sees.  At the
			# sensor horizon the robot stops exploring as soon as its immediate
			# pocket is mapped.
			'active_area_radius': LaunchConfiguration('active_area_radius',
				default='30.0')
		}]
	)

	return LaunchDescription([
		OpaqueFunction(function=_check_environment),
		DeclareLaunchArgument(
			'teleop', default_value='true',
			description='Start the joypad teleop stack.  Needs joy_linux and a pad.'),
		DeclareLaunchArgument(
			'laser_max_range', default_value='3.5',
			description='LiDAR horizon in metres.  Check the real value with '
			            '`ros2 topic echo /scan --field range_max` on the robot; '
			            'scan_preprocess warns if this disagrees with it.'),
		robot_bringup,
		slam_toolbox,
		teleop,
		camera,
		apriltag,
		frontier_detection
	])

