#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The AprilTag maze under Ignition/gz, the counterpart of project.launch.py.

Needs the turtlebot3_ignition package, which ships with a COLCON_IGNORE: delete
that file and rebuild the workspace before launching this.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory("turtlebot3_ignition"), "launch")

    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    world = os.path.join(
        get_package_share_directory("asr_summer_school"), "worlds", "hard_maze_apriltag_ignition.world"
    )

    world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, "world.launch.py")),
        launch_arguments={"world": world, "x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    ld = LaunchDescription()

    ld.add_action(world_cmd)

    return ld
