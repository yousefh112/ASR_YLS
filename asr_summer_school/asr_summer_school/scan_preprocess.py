#! /usr/bin/env python3
"""Republish /scan so that "nothing out there" becomes mappable free space.

The problem
-----------
The LDS-02 and its Gazebo model both report `inf` for a ray that reaches the
end of its 3.5 m range without hitting anything.  Karto, the mapper inside
slam_toolbox, throws those rays away entirely:

    // karto_sdk OccupancyGrid::AddScan
    if (rangeReading <= minRange || rangeReading >= maxRange || isnan(rangeReading)) {
      continue;                       // <- inf lands here
    } else if (rangeReading >= rangeThreshold) {
      ...                             // <- traced as free, no obstacle marked
    }

`maxRange` is the scan message's own `range_max`, so `inf >= 3.5` is discarded
before it can clear anything.  A robot standing in a corridor with both ends
open therefore maps only the metre of wall on either side of it: the occupancy
grid does not even grow to include the open directions.  The exploration stack
downstream then has no free space to find frontiers on, sees unknown cells
adjacent to free ones in every direction at once, clusters that speckle into a
single blob centred on the robot, and reports the arena fully explored while
the robot has not moved.

The trap in the obvious fix
---------------------------
Rewriting `inf` to something just above the mapper's range threshold - 3.45 m
against a 3.4 m threshold - does make Karto trace the ray, and exploration
starts working.  It also destroys localisation, which is a far more expensive
failure and does not announce itself.

Karto's scan matcher does not use the range-filtered point list.  Both
`ScanMatcher::AddScan` (which builds the reference grid) and
`GridIndexLookup::ComputeOffsets` (which projects the query scan) call
`GetPointReadings()`, whose default argument is `wantFiltered = false`, and
`LocalizedRangeScan::Update` places an *unfiltered* point at the raw reported
range for every ray outside the threshold.  A finite 3.45 m therefore puts a
ring of 150 phantom obstacles around every scan pose.  Those rings are rigidly
attached to their own poses, so correlation peaks when the robot's estimated
pose coincides with a previous one, and the estimate is dragged backwards.
Measured against Gazebo ground truth that cost 6 m of drift and 44 degrees of
heading in three minutes, on odometry that was accurate to 3 mm.

The fix
-------
Put the phantom point somewhere the scan matcher cannot see it.  A no-return
ray is reported at `no_return_range` (20 m by default) and `range_max` is
raised above it, which threads all three needles at once:

  * `20 < range_max` so `OccupancyGrid::AddScan` does not discard the ray, and
    `20 >= range_threshold` so it is traced as free space out to the threshold
    with no obstacle at the end.  Exactly the behaviour we want.
  * The phantom point lands 20 m from the scan pose, far outside the
    correlation grid, whose half-width is the range threshold plus the search
    window.  `ScanMatcher::AddScan` drops points outside the grid ROI, so the
    matcher never sees it.
  * Nav2's costmaps clip rays to `raytrace_max_range` before clearing and
    ignore anything past `obstacle_max_range` when marking, so a 20 m reading
    clears the corridor ahead without marking a phantom wall.

Readings below the sensor minimum become NaN rather than a range: something is
there, we just cannot trust the distance, and both Karto and Nav2 skip NaN.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


def map_range(value, horizon, floor, no_return):
    """One ray's reported range, rewritten for Karto and Nav2.

    Split out from the node so the rule can be tested without a running graph,
    because getting it wrong is silent in both directions: too small a
    `no_return` and the scan matcher acquires a ring of phantom obstacles, too
    large and there is nothing to test because the ray is discarded before it
    can clear anything.
    """
    if not math.isfinite(value) or value >= horizon:
        return no_return
    if value <= floor:
        # Something is there but the distance is not trustworthy.  NaN is the
        # one encoding both Karto and Nav2 agree to skip.
        return float('nan')
    return value


class ScanPreprocess(Node):
    def __init__(self):
        super().__init__('scan_preprocess')

        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_filtered')
        # The sensor's true horizon.  Rays reported at or beyond this, and
        # non-finite ones, are treated as "nothing out there".
        # The sensor's true horizon.  3.5 m is right for the LDS-01 and for
        # the Gazebo burger; the LDS-02 fitted to these robots reports further.
        # Set it to 0.0 to take whatever the driver puts in range_max, or pass
        # `laser_max_range:=<metres>` to the bringup.  A mismatch is not fatal
        # - readings beyond it are simply treated as "nothing there", which
        # costs mapping range but never invents an obstacle - so this node
        # checks the incoming scan and says so rather than failing.
        # Where a no-return ray is reported.  Must be far enough that the
        # phantom point falls outside Karto's correlation grid, whose
        # half-width is roughly the mapper's range threshold plus half the
        # correlation search window - under 4 m for this robot.
        self.declare_parameter('no_return_range', 20.0)
        # Published range_max.  Must exceed no_return_range or Karto discards
        # the ray again, which is the bug this node exists to fix.
        self.declare_parameter('published_range_max', 25.0)
        self.declare_parameter('sensor_min_range', 0.12)

        self.sensor_max = float(self.get_parameter('sensor_max_range').value)
        self.auto_max = self.sensor_max <= 0.0
        self.sensor_min = float(self.get_parameter('sensor_min_range').value)
        self.no_return = float(self.get_parameter('no_return_range').value)
        self.published_max = float(self.get_parameter('published_range_max').value)

        if self.published_max <= self.no_return:
            self.get_logger().error(
                'published_range_max ({}) must be greater than no_return_range '
                '({}), otherwise Karto discards every no-return ray and the map '
                'stops growing.  Raising it.'.format(
                    self.published_max, self.no_return))
            self.published_max = self.no_return * 1.25

        self._reported = False
        self._published = 0

        # Sensor-data QoS on the way in: the LDS-02 driver and the Gazebo
        # plugin both publish best-effort, and a reliable subscription would
        # silently never connect.  Out is reliable, which is what slam_toolbox
        # and the Nav2 costmaps subscribe with.
        self.publisher = self.create_publisher(
            LaserScan, self.get_parameter('output_topic').value,
            QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE))
        self.create_subscription(
            LaserScan, self.get_parameter('input_topic').value, self._callback,
            QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT))

    def _callback(self, scan):
        if self.auto_max:
            horizon = scan.range_max
        else:
            horizon = min(self.sensor_max, scan.range_max)
            if not self._reported and scan.range_max > self.sensor_max * 1.1:
                self.get_logger().warn(
                    'the driver reports range_max {:.2f} m but sensor_max_range '
                    'is {:.2f} m, so everything beyond {:.2f} m is being '
                    'discarded as a non-return.  Mapping still works, but the '
                    'robot sees less per scan than the LiDAR can.  Pass '
                    'laser_max_range:={:.1f} to the bringup, and set '
                    'max_laser_range slightly below it in '
                    'param_slam_toolbox.yaml.'.format(
                        scan.range_max, self.sensor_max, self.sensor_max,
                        scan.range_max))
        floor = max(self.sensor_min, scan.range_min)

        ranges = [map_range(v, horizon, floor, self.no_return)
                  for v in scan.ranges]
        no_return = sum(1 for v in ranges if v == self.no_return)
        too_close = sum(1 for v in ranges if v != v)

        scan.ranges = ranges
        scan.range_max = self.published_max
        self.publisher.publish(scan)

        self._published += 1
        if not self._reported:
            self._reported = True
            self.get_logger().info(
                'driver reports range [{:.2f}, {:.2f}] m; using a {:.2f} m '
                'horizon.  {} of {} rays saw nothing and are republished at '
                '{:.1f} m with range_max {:.1f}; {} were closer than {:.2f} m'
                .format(scan.range_min, scan.range_max, horizon,
                        no_return, len(ranges), self.no_return,
                        self.published_max, too_close, floor))


def main(args=None):
    rclpy.init(args=args)
    node = ScanPreprocess()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
