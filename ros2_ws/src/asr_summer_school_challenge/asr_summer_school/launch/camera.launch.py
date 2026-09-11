"""Camera bringup, at a resolution that can actually reach the tags.

A copy of `turtlebot3_perception/launch/camera.launch.py` in everything except
the colour profile, which that file hardcodes with no override hook - the same
reason `apriltag.launch.py` is duplicated here.

Why 1280x720 and not the vendored 640x480
-----------------------------------------
Tag decoding is decided by how many pixels span the tag, and that follows from
the sensor's angular resolution: a 0.16 m tag at range r subtends 0.16/r rad,
and one pixel subtends HFOV/width.

    profile          HFOV      px across a 0.16 m tag at range r
    640x480  (4:3)   55 deg    (0.16/r)/0.96  * 640  = 107/r
    1280x720 (16:9)  69 deg    (0.16/r)/1.204 * 1280 = 170/r

A 36h11 tag is ten cells across counting the border and needs roughly twenty
pixels to decode at all.  That puts the decode horizon at about **5.3 m** on the
vendored profile and about **8.5 m** on this one.

Area is what the mission actually spends: a camera sweep covers a disc, so
(8.5/5.3)^2 is about **2.6x the ground per sweep**, for the same seconds spent
turning.  A tag is worth 50 points and the whole run is bounded by how much
ground the robot can cover in the window, so this is the largest single lever on
tag count available anywhere in the stack - larger than any amount of driving
faster, because the robot is already at the burger's 0.22 m/s ceiling.

16:9 also widens the field of view from 55 to 69 degrees, which is 26% more
arena in every frame while driving.

Why 15 fps and not 30
---------------------
1280x720 is four times the pixels of 640x480 and AprilTag's cost is roughly
linear in them, on a NUC that is also running SLAM, Nav2 and two costmaps.
Halving the frame rate pays most of that back.

Nothing is lost: a sweep takes about 3 s, so 15 fps still puts around 45 frames
on the arena per stop, and a tag stays inside a 69 degree field of view for
1.2/1.9 = 0.63 s, i.e. nine frames, at the configured sweep rate.  Range decides
whether a tag can be decoded at all; frame rate only decides how many chances
there are to decode one already within reach.  Range is worth more.

If the NUC still cannot keep up, lower the frame rate again before touching the
resolution, and check with `ros2 topic hz /camera/color/image_raw`.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# One place to change it on the day.  "<width>,<height>,<fps>".
COLOR_PROFILE = '1280,720,15'


def generate_launch_description():
    ld = LaunchDescription()

    # A bare os.environ[...] here is the vendored behaviour and it throws a
    # KeyError from inside a submodule when unset.  bringup.launch.py checks
    # this variable up front so that never reaches this file, but the default
    # keeps this launch usable on its own.
    camera_model = os.environ.get('CAMERA_MODEL', 'realsense')

    if camera_model == 'realsense':
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('turtlebot3_perception'),
                        'launch', 'rs_launch_composable.launch.py'])
                ),
                launch_arguments={
                    'rgb_camera.color_profile': COLOR_PROFILE,
                    # `enable_depth`, not `depth_module.enable_depth`.  Only the
                    # first is a declared argument of rs_launch_composable; the
                    # second - which the vendored launch passes, and which this
                    # file inherited - is silently ignored, so depth kept
                    # streaming at 848x480x30 alongside the colour we actually
                    # use.  Confirmed in the robot's own bringup log:
                    #   Open profile: Depth, Z16, 848x480, FPS: 30
                    # We never read depth.  Turning it off returns USB bandwidth
                    # and NUC cycles to the colour stream and to AprilTag, which
                    # is the thing that scores.
                    'enable_depth': 'false',
                    'enable_infra1': 'false',
                    'enable_infra2': 'false',
                }.items(),
            )
        )
        # Unchanged from the vendored launch.  Note the camera sits at
        # z = 0.240, which is also the height the arena's tag plates are
        # centred on - tags are at eye level, so a level sweep sees them.
        ld.add_action(
            Node(
                name='camera_tf_static',
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    '--x', '-0.059', '--y', '0.0', '--z', '0.240',
                    '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                    '--frame-id', 'base_footprint',
                    '--child-frame-id', 'camera_link',
                ],
                output='screen',
            )
        )

    elif camera_model == 'oakd':
        params_file = PathJoinSubstitution([
            FindPackageShare('turtlebot3_perception'), 'config', 'oakd_config.yaml'])
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('depthai_ros_driver'),
                        'launch', 'camera.launch.py'])
                ),
                launch_arguments={
                    'rs_compat': 'true',
                    'parent_frame': 'base_footprint',
                    'cam_pos_x': '-0.062',
                    'cam_pos_y': '0.0',
                    'cam_pos_z': '0.245',
                    'cam_roll': '0.0',
                    'cam_pitch': '0.0',
                    'cam_yaw': '0.0',
                    'params_file': params_file,
                    'rgb_camera.color_profile': COLOR_PROFILE,
                    'enable_depth': 'false',
                    'enable_infra1': 'false',
                    'enable_infra2': 'false',
                }.items(),
            )
        )

    return ld
