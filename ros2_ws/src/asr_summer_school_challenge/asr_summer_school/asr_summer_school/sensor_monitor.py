"""
Sample sensor node.

Subscribes to the robot sensor topics and keeps only the *latest* message of
each of them in its attributes, so that the main script can read them at any
time without dealing with callbacks.

The node is meant to be spun in its own thread, see `start()` / `stop()` or the
context-manager usage:

    with SensorMonitor() as sensors:
        scan = sensors.scan          # latest sensor_msgs/LaserScan (or None)
        d = sensors.min_range        # closest obstacle distance, meters
"""

import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SensorMonitor(Node):
    """Caches the last message received on every subscribed sensor topic."""

    def __init__(self, node_name='sensor_monitor', scan_topic='/scan', odom_topic='/odom'):
        super().__init__(node_name)

        # Guards every cached attribute: callbacks run in the spin thread while
        # the main thread reads the properties.
        self._lock = threading.Lock()

        self._scan = None
        self._odom = None

        self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, odom_topic, self._odom_callback, qos_profile_sensor_data)

        self._executor = None
        self._thread = None

    # ------------------------------------------------------------------ #
    # Callbacks: keep them short, they only store the incoming message.
    # ------------------------------------------------------------------ #

    def _scan_callback(self, msg):
        with self._lock:
            self._scan = msg

    def _odom_callback(self, msg):
        with self._lock:
            self._odom = msg

    # ------------------------------------------------------------------ #
    # Properties: what the main script reads.
    # ------------------------------------------------------------------ #

    @property
    def scan(self):
        """Last sensor_msgs/LaserScan received, or None if nothing arrived yet."""
        with self._lock:
            return self._scan

    @property
    def odom(self):
        """Last nav_msgs/Odometry received, or None if nothing arrived yet."""
        with self._lock:
            return self._odom


    def wait_for_data(self, timeout=10.0):
        """Block until both a scan and an odometry message have been cached.

        Returns True on success, False if `timeout` seconds elapsed first.
        """
        deadline = self.get_clock().now().nanoseconds + timeout * 1e9
        while self.scan is None or self.odom is None:
            if self.get_clock().now().nanoseconds > deadline:
                return False
            threading.Event().wait(0.05)
        return True

    # ------------------------------------------------------------------ #
    # Independent spinning.
    # ------------------------------------------------------------------ #

    def start(self):
        """Spin this node in a background thread."""
        if self._thread is not None:
            return self
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        """Stop the background thread and destroy the node."""
        if self._executor is not None and self._thread is not None:
            self._executor.shutdown()
            self._thread.join(timeout=2.0)
            self._executor.remove_node(self)
            self._executor = None
            self._thread = None
        self.destroy_node()

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
        return False