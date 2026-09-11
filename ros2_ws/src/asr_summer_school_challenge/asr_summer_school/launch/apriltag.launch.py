"""AprilTag detection on the real robot, with our detector settings.

A copy of `turtlebot3_perception/launch/apriltag.launch.py` in everything except
which config file it reads.  The vendored version loads its own YAML with
`yaml_to_dict` and hands the dict straight to the composable node, so a detector
setting cannot be overridden from a launch argument or a parameter file - and
`decimate` had to change, because at its vendored value the detector cannot
decode a 16 cm tag beyond about 2.5 m while the mission was gating detections at
5 m.  See config/apriltag.yaml for that arithmetic.

Editing the vendored package was the alternative and is not allowed: it would be
lost on the next upstream sync and invisible in review.  This file is the same
launch with `asr_summer_school`'s config, and it must be kept in step if
turtlebot3_perception's ever changes - VENDORED.md records the pinned commit.
"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

import yaml


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as handle:
        return yaml.load(handle, Loader=yaml.SafeLoader)


def generate_launch_description():
    params = yaml_to_dict(os.path.join(
        get_package_share_directory('asr_summer_school'),
        'config', 'apriltag.yaml'))

    load_composable_nodes = LoadComposableNodes(
        target_container='/camera/camera_container',
        composable_node_descriptions=[
            ComposableNode(
                namespace='camera',
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag',
                # ABSOLUTE, and /camera/color/..., not the vendored relative
                # 'camera/color/...'.  Inside namespace /camera that relative
                # name resolves to /camera/camera/color/image_raw - but the
                # RealSense node on nuc11 comes up as /camera, not
                # /camera/camera, so it publishes /camera/color/image_raw.
                # Measured by publisher count, not by topic name:
                #   /camera/color/image_raw         publishers=[camera]  subscribers=[]
                #   /camera/camera/color/image_raw  publishers=[]        subscribers=[apriltag]
                # AprilTag subscribed to a topic nobody published, received no
                # image all run, raised no error - and found no tag.  A bare
                # `ros2 topic list` shows both names, because a subscription
                # alone makes a topic appear, which is how this hid.
                remappings=[
                    ('image_rect', '/camera/color/image_raw'),
                    ('camera_info', '/camera/color/camera_info'),
                ],
                parameters=[params['camera']['apriltag']['ros__parameters']],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    # Unchanged from the vendored launch: range and bearing in base_link, which
    # the challenge brief lists as a provided interface.  The mission itself
    # consumes the TF frames rather than this topic, because those exist
    # identically in Gazebo and on the robot.
    landmark_node = Node(
        namespace='camera',
        package='turtlebot3_perception',
        executable='detection2landmark',
        output='screen',
        emulate_tty=True,
        parameters=[{'robot_base_frame': 'base_link'}],
    )

    return LaunchDescription([load_composable_nodes, landmark_node])
