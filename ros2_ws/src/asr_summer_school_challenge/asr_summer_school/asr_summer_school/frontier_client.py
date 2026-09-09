"""ROS plumbing for the frontier detector.

`frontier_detection_node` is complete and already publishes DBSCAN-clustered
centroids, but nothing in the starter workspace subscribes to them, so the
robot never goes anywhere.  This module is only the transport half of closing
that loop; the ranking, blacklisting and hysteresis all live in
`frontier_policy.FrontierPolicy`, which has no rclpy import and can be
exercised offline.

Two details about the publisher matter.  It publishes a single
`visualization_msgs/Marker` of type POINTS, so the centroids arrive as
`marker.points` rather than as a pose array; and it publishes with
`transient_local` durability, so a subscriber must match that or it will
silently receive nothing at all.
"""

import threading

from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from visualization_msgs.msg import Marker


class FrontierMonitor(Node):
    """Caches the newest frontier centroids.  Spin it in a background thread."""

    def __init__(self, node_name='frontier_monitor', topic='frontier_centroids'):
        super().__init__(node_name)

        self._lock = threading.Lock()
        self._centroids = []
        self._frame_id = 'map'
        self._updates = 0

        # Must match the publisher's QoS(1).transient_local() exactly, or this
        # subscription connects and never receives anything.
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Marker, topic, self._callback, qos)

    def _callback(self, msg):
        with self._lock:
            self._centroids = [(p.x, p.y) for p in msg.points]
            self._frame_id = msg.header.frame_id or 'map'
            self._updates += 1

    @property
    def centroids(self):
        """Latest list of (x, y) frontier centroids in the map frame."""
        with self._lock:
            return list(self._centroids)

    @property
    def frame_id(self):
        with self._lock:
            return self._frame_id

    @property
    def updates(self):
        """How many frontier messages have arrived; 0 means the detector is silent."""
        with self._lock:
            return self._updates

    def wait_for_frontiers(self, timeout=30.0):
        """Block until at least one frontier message arrives.  True on success."""
        deadline = self.get_clock().now().nanoseconds + timeout * 1e9
        while self.updates == 0:
            if self.get_clock().now().nanoseconds > deadline:
                return False
            threading.Event().wait(0.1)
        return True
