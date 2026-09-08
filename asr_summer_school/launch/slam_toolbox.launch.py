import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    laser_max_range = LaunchConfiguration('laser_max_range')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("asr_summer_school"),
                                   'config', 'param_slam_toolbox.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    # The LDS-02 and its Gazebo model both stop at 3.5 m. Rastering rays out
    # past anything actually measured is CLAUDE.md defect 5; exposed here so
    # the physical arena can be matched without editing the config.
    declare_laser_max_range_cmd = DeclareLaunchArgument(
        'laser_max_range', default_value='3.5',
        description='Maximum usable laser range, metres')

    # `laser_max_range` is the *sensor* horizon: 3.5 m for the LDS-02 and for
    # its Gazebo model.  What the mapper wants is a range *threshold* slightly
    # below it, so that the no-return rays scan_preprocess.py reports at 20 m
    # land above the threshold and are traced as free space rather than marked
    # as obstacles.  The 0.1 m of headroom is applied here so that changing the
    # arena's sensor horizon is a one-number edit.
    rewritten_slam_params = RewrittenYaml(
        source_file=slam_params_file,
        param_rewrites={
            'max_laser_range': PythonExpression(
                ['str(float("', laser_max_range, '") - 0.10)']),
        },
        convert_types=True,
    )
    slam_params_file_w_subst = ParameterFile(
        rewritten_slam_params,
        allow_substs=True,
    )
    
    # A plain Node, not a LifecycleNode.  slam_toolbox 2.6.10 as shipped in
    # Humble is not a managed node: `strings` finds no lifecycle symbol in
    # async_slam_toolbox_node, it never advertises /slam_toolbox/change_state,
    # and it starts mapping from its constructor.  The upstream
    # online_async_launch.py launches it as a plain Node for the same reason.
    #
    # Driving a lifecycle handshake at it anyway - which is what this file used
    # to do - left launch_ros blocked in an unbounded wait_for_service loop
    # registered as a launch completion future.  Mapping still worked, so the
    # only visible symptom was a launch that would not shut down cleanly on
    # Ctrl-C, which is exactly the wrong thing to discover between runs on
    # competition day.
    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file_w_subst,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    # Everything downstream - slam_toolbox and both Nav2 costmaps - reads
    # /scan_filtered rather than /scan, so this node is on the critical path
    # for the whole stack.  See scan_preprocess.py for why it exists: without
    # it the map does not grow into open space and exploration stops before it
    # starts.  It replaces the laser_filters chain that used to sit here, which
    # cannot rewrite range_max and so cannot express "no return" in the one way
    # Karto will accept.
    #
    # use_sim_time matters: without it this runs on the wall clock while
    # everything around it runs on the Gazebo clock, and /scan_filtered arrives
    # with timestamps slam_toolbox cannot match.
    scan_preprocess = Node(
        package='asr_summer_school',
        executable='scan_preprocess.py',
        name='scan_preprocess',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'sensor_max_range': laser_max_range,
        }],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_laser_max_range_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(scan_preprocess)

    return ld