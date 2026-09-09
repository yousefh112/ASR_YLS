"""Subscriber that keeps the newest occupancy grid available for export.

Kept apart from `map_export`, which is deliberately free of any rclpy import so
its PGM and semantic-map rendering can be unit tested without a running graph.
This is the transport half: it holds the last `/map` message so the mission can
write the deliverables at any point, including after an interrupt.
"""

import threading

from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)


class MapRecorder(Node):
    """Caches the newest `/map`.  Spin it in a background thread."""

    def __init__(self, node_name='map_recorder', topic='map'):
        super().__init__(node_name)
        self._lock = threading.Lock()
        self._grid = None
        self._updates = 0

        # slam_toolbox latches the map, so the subscription has to be
        # transient_local or it will sit connected and receive nothing.
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, topic, self._callback, qos)

    def _callback(self, msg):
        with self._lock:
            self._grid = msg
            self._updates += 1

    @property
    def grid(self):
        """Latest nav_msgs/OccupancyGrid, or None if nothing has arrived."""
        with self._lock:
            return self._grid

    @property
    def updates(self):
        with self._lock:
            return self._updates

    def wait_for_map(self, timeout=30.0):
        """Block until a map arrives.  True on success, False on timeout."""
        deadline = self.get_clock().now().nanoseconds + timeout * 1e9
        while self.grid is None:
            if self.get_clock().now().nanoseconds > deadline:
                return False
            threading.Event().wait(0.1)
        return True
