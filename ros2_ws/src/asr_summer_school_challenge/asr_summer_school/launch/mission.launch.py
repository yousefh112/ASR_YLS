"""Launch the autonomous mission: Nav2 plus the orchestrator.

Assumes a bringup (`bringup_simulation.launch.py` or `bringup.launch.py`) is
already running, since that is what provides SLAM, the camera, AprilTag
detection and the frontier detector.  Neither bringup file starts Nav2, so this
file does, and then starts the mission itself.

    ros2 launch asr_summer_school mission.launch.py use_sim_time:=true
    ros2 launch asr_summer_school mission.launch.py mission_duration:=420.0

Parameters come from `config/param_mission.yaml` and are overridden through
RewrittenYaml rather than through `parameters=[{...}]`: the mission process
hosts several nodes, so a plain dict override would be written under the `/**`
wildcard, and a fully qualified node section in the YAML beats a wildcard no
matter which is specified later.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_dir = get_package_share_directory('asr_summer_school')

    use_sim_time = LaunchConfiguration('use_sim_time')
    # Deliberately NOT called `params_file`. IncludeLaunchDescription inherits
    # the parent scope's launch configurations, and an included file's own
    # DeclareLaunchArgument only supplies a default when the configuration is
    # not already set. A `params_file` here would therefore silently replace
    # Nav2's param_nav2.yaml with this file, leaving controller_server with no
    # FollowPath critics and Nav2 stuck in configure forever.
    mission_params_file = LaunchConfiguration('mission_params_file')
    mission_duration = LaunchConfiguration('mission_duration')
    output_directory = LaunchConfiguration('output_directory')
    optical_correction = LaunchConfiguration('optical_correction')
    scan_rotation = LaunchConfiguration('scan_rotation')
    stop_after_tags = LaunchConfiguration('stop_after_tags')
    start_nav2 = LaunchConfiguration('start_nav2')
    autostart = LaunchConfiguration('autostart')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the Gazebo clock')
    declare_mission_params_file = DeclareLaunchArgument(
        'mission_params_file',
        default_value=os.path.join(package_dir, 'config', 'param_mission.yaml'),
        description='Parameters for the mission stack (not for Nav2)')
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(package_dir, 'config', 'param_nav2.yaml'),
        description='Parameters for Nav2')
    declare_mission_duration = DeclareLaunchArgument(
        'mission_duration', default_value='600.0',
        description='Seconds from start to the return deadline')
    declare_output_directory = DeclareLaunchArgument(
        'output_directory', default_value='~/asr_mission_output',
        description='Where the map, semantic map and report are written')
    declare_scan_rotation = DeclareLaunchArgument(
        'scan_rotation', default_value='6.28',
        description='Radians to turn on the spot at each frontier so the '
                    '60-degree camera sweeps the area.  0.0 disables it.  '
                    'Costs about 8 s per goal, so it is the first thing to '
                    'trade away when the run has to be short.')
    declare_stop_after_tags = DeclareLaunchArgument(
        'stop_after_tags', default_value='0',
        description='Go home as soon as this many unique tags are found. '
                    '0 disables it; only set it if the true count is known.')
    declare_optical_correction = DeclareLaunchArgument(
        'optical_correction', default_value='auto',
        description='auto | on | off.  Whether to rotate AprilTag poses out '
                    'of the optical convention.  "auto" decides from the name '
                    'of the frame apriltag_ros parented the tag to, which is '
                    'correct in Gazebo and on both cameras.')
    declare_start_nav2 = DeclareLaunchArgument(
        'start_nav2', default_value='true',
        description='Start Nav2; neither bringup file does')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Autostart the Nav2 lifecycle nodes')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=mission_params_file,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'mission_duration': mission_duration,
                'output_directory': output_directory,
                'optical_correction': optical_correction,
                'scan_rotation': scan_rotation,
                'stop_after_tags': stop_after_tags,
            },
            convert_types=True),
        allow_substs=True)

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'nav2.launch.py')),
        condition=IfCondition(start_nav2),
        # params_file is passed explicitly rather than left to nav2.launch.py's
        # own default, so this include cannot inherit anything unintended.
        launch_arguments={'params_file': LaunchConfiguration('nav2_params_file'),
                          'use_sim_time': use_sim_time,
                          'autostart': autostart}.items())

    # No `name=` here on purpose: this executable hosts several nodes, and
    # `name=` would rewrite __node for all of them and collide.
    mission = Node(
        package='asr_summer_school',
        executable='mission_control.py',
        output='screen',
        emulate_tty=True,
        parameters=[configured_params])

    return LaunchDescription([
        declare_use_sim_time,
        declare_mission_params_file,
        declare_nav2_params_file,
        declare_mission_duration,
        declare_output_directory,
        declare_optical_correction,
        declare_scan_rotation,
        declare_stop_after_tags,
        declare_start_nav2,
        declare_autostart,
        nav2,
        mission,
    ])
