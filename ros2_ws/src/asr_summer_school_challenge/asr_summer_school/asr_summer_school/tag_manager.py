#! /usr/bin/env python3
"""ROS plumbing for AprilTag detection: TF in, fused map-frame landmarks out.

Detections are consumed through TF rather than through a detection topic.
`apriltag_ros` broadcasts one frame per detected tag, named `tag36h11:<id>`,
and that frame exists identically in simulation and on the robot, whereas the
detection topics do not: `bringup_simulation.launch.py` omits
`detection2landmark`, so `/camera/landmarks` is hardware-only.  TF is the one
perception interface both environments agree on, and reading it needs no
message package beyond what the workspace already builds.

The dedup-and-fuse logic lives in `tag_map.TagMap`; this file only decides
*what counts as a live detection* and *where it is in the map frame*.

The optical-frame correction
----------------------------
`apriltag_ros` parents each tag frame to whatever frame the *image* was stamped
with, and solves the pose in the optical convention (z forward, x right, y
down) that every calibrated camera uses.

On the robot the RealSense and OAK-D drivers stamp a real optical frame, the two
conventions agree and the transform is used as-is.  In Gazebo the burger SDF
leaves `<frame_name>` commented out, so `libgazebo_ros_camera` falls back to the
link name and stamps `camera_rgb_frame`, which is body-aligned (x forward, z
up).  The tag frame then hangs off a parent whose axes are not the ones the pose
was solved in: IDs come out correct and positions come out rotated.  The
measurement is right, only its basis is wrong, so `geometry.optical_rotation()`
rotates it back into the frame it was actually stamped in.

Both the parent frame and whether it needs correcting are worked out at runtime
rather than configured.  The parent is read from TF itself, which knows what
`apriltag_ros` actually broadcast; and the correction is applied when that
parent's name does not look like an optical frame.  Two settings that had to be
right for the mission to score anything, that differ between the two
environments this runs in, and that fail silently when wrong, are worth not
asking a human to remember at 14:00 on the day.  `optical_correction` overrides
the guess when the auto-detection is wrong.
"""

import re
import threading

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from asr_summer_school.geometry import (matrix_from_transform_msg,
                                        optical_rotation, translation_of)
from asr_summer_school.tag_map import TagMap


class TagManager(Node):
    """Turns tag TF frames into fused map-frame landmarks.

    Spin it in a background thread; read `tag_map` from the mission thread.
    """

    def __init__(self, node_name='tag_manager'):
        super().__init__(node_name)

        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('map_frame', 'map')
        # No camera frame parameter: the parent of a tag frame is read from TF,
        # which is authoritative because it is whatever apriltag_ros stamped
        # the image with.  A configured name here would be one more thing to
        # get wrong when moving between Gazebo and the robot.
        #
        # 'auto'  apply the rotation unless the parent frame looks optical
        # 'on'    always apply it   (Gazebo, if auto ever guesses wrong)
        # 'off'   never apply it    (a driver that stamps a true optical frame)
        self.declare_parameter('optical_correction', 'auto')
        self.declare_parameter('poll_period', 0.2)
        # A tag frame older than this is a leftover in the TF buffer rather
        # than a live sighting: apriltag_ros stops broadcasting when the tag
        # leaves view, but tf2 keeps serving the last transform.
        self.declare_parameter('max_detection_age', 1.0)
        self.declare_parameter('max_detection_range', 4.0)
        self.declare_parameter('gate_distance', 0.75)
        self.declare_parameter('gate_patience', 5)
        self.declare_parameter('publish_markers', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.optical_correction = str(
            self.get_parameter('optical_correction').value).strip().lower()
        self.max_detection_age = self.get_parameter('max_detection_age').value
        self._publish_markers = bool(self.get_parameter('publish_markers').value)

        # Kept as an attribute as well as inside TagMap: the coverage report
        # needs to know how far the camera could see, and it lives in
        # mission_control, which is a different node.  Reading another node's
        # parameter across that boundary would work but reads as though
        # mission_control declared it, which it must not.
        self.max_detection_range = float(
            self.get_parameter('max_detection_range').value)

        self.tag_map = TagMap(
            max_range=self.max_detection_range,
            gate_distance=self.get_parameter('gate_distance').value,
            gate_patience=self.get_parameter('gate_patience').value)

        self._lock = threading.Lock()
        self._last_stamp = {}
        self._start_time = None
        self._optical_fix = optical_rotation()
        self._announced_parent = None
        self._skipped = 0
        self._frame_pattern = re.compile(
            r'^/?' + re.escape(self.get_parameter('tag_family').value) + r':(\d+)$')

        self._buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._listener = tf2_ros.TransformListener(self._buffer, self, spin_thread=False)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._marker_pub = self.create_publisher(MarkerArray, 'semantic_map', latched)

        self.create_timer(self.get_parameter('poll_period').value, self._poll)

    # ------------------------------------------------------------------ #
    # Public surface
    # ------------------------------------------------------------------ #

    @property
    def skipped(self):
        """Detections dropped because TF could not answer at their timestamp."""
        with self._lock:
            return self._skipped

    @property
    def count(self):
        with self._lock:
            return len(self.tag_map)

    @property
    def ids(self):
        with self._lock:
            return self.tag_map.ids

    def as_list(self):
        """Landmarks as plain dicts, sorted by ID, ready to serialise."""
        with self._lock:
            return self.tag_map.to_list()

    # ------------------------------------------------------------------ #
    # Detection polling
    # ------------------------------------------------------------------ #

    def _now(self):
        return self.get_clock().now()

    def _mission_time(self):
        now = self._now().nanoseconds / 1e9
        if self._start_time is None:
            self._start_time = now
        return now - self._start_time

    def _tag_frames(self):
        """Every `<family>:<id>` frame TF knows about, with its parent.

        `all_frames_as_yaml()` is the only way to enumerate frames from rclpy,
        and it is also the only place the *parent* of each frame is exposed,
        which is what removes the need to configure a camera frame name:

            tag36h11:4:
              parent: 'camera_rgb_frame'
              broadcaster: 'default_authority'
              ...

        Frame names are the sole unindented keys, written as "<name>: ".  Tag
        frame names contain a colon of their own, so only the trailing one is
        stripped.
        """
        frames = []
        try:
            listing = self._buffer.all_frames_as_yaml()
        except Exception:  # TF not ready yet
            return frames

        pending = None
        for line in listing.splitlines():
            if not line:
                continue
            if not line[0].isspace():
                pending = None
                stripped = line.rstrip()
                if not stripped.endswith(':'):
                    continue
                name = stripped[:-1].strip()
                match = self._frame_pattern.match(name)
                if match:
                    pending = (name, int(match.group(1)))
                continue
            if pending is not None:
                key, _, value = line.strip().partition(':')
                if key == 'parent':
                    frames.append(pending + (value.strip().strip("'\""),))
                    pending = None
        return frames

    def _needs_optical_fix(self, parent):
        """Whether the tag pose has to be rotated out of the optical basis.

        A driver that stamps a true optical frame names it so; every ROS camera
        driver in use here follows that convention
        (`camera_color_optical_frame`, `camera_rgb_optical_frame`, ...).  Gazebo
        falls back to the link name, which does not.
        """
        if self.optical_correction == 'on':
            return True
        if self.optical_correction == 'off':
            return False
        return 'optical' not in parent.lower()

    def _poll(self):
        for frame, tag_id, parent in self._tag_frames():
            try:
                self._process(frame, tag_id, parent)
            except tf2_ros.TransformException:
                continue

    def _process(self, frame, tag_id, parent):
        if parent != self._announced_parent:
            self._announced_parent = parent
            self.get_logger().info(
                'tag frames are parented to "{}"; optical correction {} '
                '(optical_correction={})'.format(
                    parent,
                    'ON' if self._needs_optical_fix(parent) else 'off',
                    self.optical_correction))

        detection = self._buffer.lookup_transform(parent, frame, Time())

        stamp = Time.from_msg(detection.header.stamp)
        if (self._now() - stamp).nanoseconds / 1e9 > self.max_detection_age:
            return  # stale buffer entry, the tag is no longer in view
        if self._last_stamp.get(tag_id) == stamp.nanoseconds:
            return  # this exact detection has already been fused
        self._last_stamp[tag_id] = stamp.nanoseconds

        camera_to_tag = matrix_from_transform_msg(detection.transform)
        if self._needs_optical_fix(parent):
            camera_to_tag = self._optical_fix @ camera_to_tag

        detection_range = float(np.linalg.norm(translation_of(camera_to_tag)))

        # Strictly at the detection's own timestamp.  Falling back to the
        # latest transform when the exact one is unavailable looks harmless and
        # is not: the camera sweeps at about a radian per second, so half a
        # second of staleness rotates the sighting by thirty degrees, and at
        # 1.3 m that put a tag 63 cm from where it actually was - four times
        # the error that separates full accuracy points from none.  Detections
        # arrive at 10 Hz and the tag stays in view for a second or more, so
        # dropping one costs nothing at all.
        try:
            to_map = self._buffer.lookup_transform(
                self.map_frame, parent, stamp, timeout=Duration(seconds=0.2))
        except tf2_ros.TransformException:
            self._skipped += 1
            return

        x, y, z = translation_of(
            matrix_from_transform_msg(to_map.transform) @ camera_to_tag)

        with self._lock:
            outcome = self.tag_map.observe(
                tag_id, x, y, z, self._mission_time(), range_m=detection_range)
            total = len(self.tag_map)
            snapshot = [(t.id, t.position, t.observations) for t in self.tag_map.tags()]

        if outcome == TagMap.NEW:
            self.get_logger().info(
                'tag {} found at ({:.2f}, {:.2f}) range {:.2f} m  [{} unique]'
                .format(tag_id, x, y, detection_range, total))
        if self._publish_markers and outcome in (TagMap.NEW, TagMap.FUSED):
            self._publish(snapshot)

    # ------------------------------------------------------------------ #
    # Visualisation
    # ------------------------------------------------------------------ #

    def _publish(self, snapshot):
        markers = MarkerArray()
        stamp = self._now().to_msg()
        for tag_id, (x, y, z), observations in snapshot:
            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = stamp
            sphere.ns = 'apriltags'
            sphere.id = tag_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = Point(x=x, y=y, z=z)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25
            sphere.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.9)
            markers.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = stamp
            label.ns = 'apriltag_labels'
            label.id = tag_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(x=x, y=y, z=z + 0.3)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.3
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = 'id {} (x{})'.format(tag_id, observations)
            markers.markers.append(label)

        self._marker_pub.publish(markers)


def main(args=None):
    """Standalone entry point, handy for watching detections during a teleop run."""
    rclpy.init(args=args)
    node = TagManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(node.tag_map.summary())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
