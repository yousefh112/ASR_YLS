"""Joypad teleop: the manual stop of last resort.

Whichever joystick driver the machine actually has
--------------------------------------------------
There are two, they publish the same `/joy`, and which one is installed varies
by machine: `joy_linux` is the older standalone driver, `joy` is the one that
ships with most Humble installs. This file used to name `joy_linux`
unconditionally, with `joy` commented out beneath it.

That is not a cosmetic problem, because a launch file naming a package that is
not installed raises at *description construction* time - before anything
starts - and takes the whole bringup down with it:

    [ERROR] [launch]: Caught exception in launch:
        "package 'joy_linux' not found"

SLAM, the camera, AprilTag and the frontier detector never start, and the error
names a joypad, which is the least important thing in the launch. This is
CLAUDE.md defect 13: it was addressed by making teleop conditional on a launch
argument, but that argument defaults to true, so the bringup still aborted on
any machine without `joy_linux` - which is what happened on nuc11.

So: use whichever driver is present, prefer `joy_linux` because
`config/param_teleop.yaml`'s button mapping was written against it, and if
neither is installed start nothing and say so. Never raise.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _installed(package):
    try:
        get_package_share_directory(package)
        return True
    except (PackageNotFoundError, LookupError):
        return False


# (package, executable), in order of preference.
_JOY_DRIVERS = (
    ('joy_linux', 'joy_linux_node'),
    ('joy', 'joy_node'),
)


def generate_launch_description():
    config_filepath = LaunchConfiguration('config_filepath')

    actions = [
        DeclareLaunchArgument(
            'config_filepath',
            default_value=os.path.join(
                get_package_share_directory('asr_summer_school'),
                'config', 'param_teleop.yaml')),
    ]

    driver = next(((p, e) for p, e in _JOY_DRIVERS if _installed(p)), None)

    if driver is None:
        # Deliberately not an error: the pad is a convenience and a safety
        # backstop, and losing it must not cost the mission. The bringup carries
        # on without it.
        actions.append(LogInfo(msg=(
            'teleop: neither joy_linux nor joy is installed, so no joypad '
            'support. Everything else starts normally. To add it:\n'
            '  sudo apt install ros-humble-joy-linux ros-humble-teleop-twist-joy\n'
            'The pad is the manual stop of last resort, so it is worth having '
            'before a scored run - but on the robot only if the pad is plugged '
            'in there. See RUNBOOK section 3.0.')))
        return LaunchDescription(actions)

    package, executable = driver
    actions.append(LogInfo(msg='teleop: using the {} driver'.format(package)))
    actions.append(Node(
        package=package,
        executable=executable,
        name='joy_node',
        parameters=[{
            'deadzone': 0.2,
            'autorepeat_rate': 20.0,
        }],
    ))

    if not _installed('teleop_twist_joy'):
        actions.append(LogInfo(msg=(
            'teleop: teleop_twist_joy is not installed, so the pad will publish '
            '/joy but nothing will turn it into /cmd_vel.')))
        return LaunchDescription(actions)

    actions.append(Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_filepath],
        remappings=[('/cmd_vel', '/cmd_vel')],
    ))

    return LaunchDescription(actions)
