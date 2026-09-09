from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os   


def generate_launch_description():
    config_filepath = LaunchConfiguration('config_filepath')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_filepath',
            default_value=os.path.join(
                get_package_share_directory("asr_summer_school"), 'config', 'param_teleop.yaml'
            )
        ,),


        # Node(
        #     package='joy',
        #     executable='game_controller_node',
        #     name='joy_node',
        #     parameters=[{
        #         'deadzone': 0.2,
        #         'autorepeat_rate': 20.0,
        #     }],
        # ),

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.2,
                'autorepeat_rate': 20.0,
            }],
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath],
            remappings=[('/cmd_vel', '/cmd_vel')],
        ),
    ])
