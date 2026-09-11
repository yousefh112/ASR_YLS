import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, Shutdown)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


# What the REAL sensors do, from the ROBOTIS specifications - not from the
# Gazebo models, which differ and are what the simulation numbers describe.
#
#   LDS-01   0.12 - 3.5 m,  1 deg, 5 Hz    (the Gazebo model is this one)
#   LDS-02   0.16 - 8.0 m,  1 deg, 5 Hz    (what our robot carries)
#   LDS-03   0.16 - 12  m,  1 deg, 10 Hz
#
# Used as the default sensor horizon below.  Verify it on the day rather than
# trusting this table - `ros2 topic echo /scan --field range_max --once` - and
# pass laser_max_range:=<metres> if the driver disagrees.  scan_preprocess also
# prints a warning when the driver's range_max and this value diverge.
_LDS_RANGE_M = {
    'LDS-01': '3.5',
    'LDS-02': '8.0',
    'LDS-03': '12.0',
}

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
	# The real sensor horizon, shared by slam_toolbox and scan_preprocess.
	#
	# THIS FILE IS THE ROBOT. The default is taken from LDS_MODEL rather than
	# copied from the simulator, because the two are not the same sensor and the
	# difference is large: the Gazebo model is an LDS-01 reaching 3.5 m, while
	# the robot carries an LDS-02 reaching 8 m. Hard-coding 3.5 here - which is
	# what this used to do, and what bringup_simulation.launch.py correctly still
	# does - throws away more than half of the real LiDAR's reach and produces a
	# map that grows far more slowly than the hardware allows. Nothing warns
	# about it, because a shorter horizon is not an error.
	laser_max_range = LaunchConfiguration(
		'laser_max_range', default=_LDS_RANGE_M.get(os.environ.get('LDS_MODEL'), '3.5'))

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
		# trace_no_return false: the real arena is walled, so a beam that returns
		# nothing went through a gap, and tracing it painted the outside free.
		launch_arguments={'use_sim_time': 'false',
		                  'laser_max_range': laser_max_range,
		                  'trace_no_return': 'false'}.items()
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

	# Ours, not turtlebot3_perception's.  Same camera, same static transform; the
	# difference is the colour profile, which the vendored file hardcodes at
	# 640x480 with no override.  That caps tag decoding at about 5.3 m, against
	# 8.5 m at 1280x720 - and since a sweep covers a disc, 2.6x the ground per
	# stop.  See config note at the top of our camera.launch.py.
	camera = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'camera.launch.py']
			)
		)
	)

	# Ours, not turtlebot3_perception's.  Same composable node and the same
	# detection2landmark alongside it; the difference is the detector config,
	# which the vendored launch bakes in with no override hook.  Against the
	# 1280x720 profile our camera.launch.py sets, its decimate of 2.0 would put
	# the decode horizon near 4 m where 1.0 reaches about 8 m - and a sweep
	# covers a disc, so that is four times the ground per stop.  Arithmetic in
	# config/apriltag.yaml.
	apriltag = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution(
				[FindPackageShare('asr_summer_school'), 'launch', 'apriltag.launch.py']
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
			# 20 cells = 1 m: real arena, ignore corner slivers and chase the big
			# openings (the doorway to its largest section was ~38 cells).
			'min_frontier_size': 20,
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
			'laser_max_range',
			default_value=_LDS_RANGE_M.get(os.environ.get('LDS_MODEL'), '3.5'),
			description='LiDAR horizon in metres.  Defaults from LDS_MODEL: '
			            'LDS-01 3.5, LDS-02 8.0, LDS-03 12.0.  Verify on the '
			            'robot with `ros2 topic echo /scan --field range_max '
			            '--once`; scan_preprocess warns if the driver disagrees.'),
		robot_bringup,
		slam_toolbox,
		teleop,
		camera,
		apriltag,
		frontier_detection
	])

