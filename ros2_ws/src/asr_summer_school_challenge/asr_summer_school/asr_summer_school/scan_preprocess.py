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


def map_range(value, horizon, floor, no_return, zero_is_no_return=True):
    """One ray's reported range, rewritten for Karto and Nav2.

    Split out from the node so the rule can be tested without a running graph,
    because getting it wrong is silent in both directions: too small a
    `no_return` and the scan matcher acquires a ring of phantom obstacles, too
    large and there is nothing to test because the ray is discarded before it
    can clear anything.
    """
    if not math.isfinite(value) or value >= horizon:
        return no_return
    if value == 0.0 and zero_is_no_return:
        # A real LDS reports 0.0 for "this ray came back with nothing", where
        # the Gazebo model reports inf.  Both mean the same thing and both have
        # to be traced as free space.
        #
        # This is CLAUDE.md defect 1 waiting to happen on hardware, and the
        # simulation cannot show it: mapping 0.0 to NaN below made Karto discard
        # every open direction, which is precisely the failure where the robot
        # maps the metre of wall beside it, finds frontiers in all directions at
        # once, and declares the arena explored without moving.
        #
        # Safe because 0.0 cannot be a genuine obstacle: the LiDAR sits near the
        # middle of a robot of radius 0.105 m, so anything it could legitimately
        # report below range_min is already touching the chassis.  If a driver
        # ever does use 0.0 for a real near return, set zero_is_no_return False
        # and it goes back to being NaN.
        return no_return
    if value <= floor:
        # Something is there but the distance is not trustworthy.  NaN is the
        # one encoding both Karto and Nav2 agree to skip.
        return float('nan')
    return value


def resample(values, angle_min, angle_increment, grid):
    """Nearest-neighbour resample of one scan onto a fixed angular grid.

    `grid` is (n_out, out_min, out_increment).  Output beam i sits at bearing
    out_min + i * out_increment and takes the input reading nearest that
    bearing.  A bearing the input does not cover - the sensor's own gap, or the
    edge of a scan that came out a beam short - becomes NaN, which Karto and
    both Nav2 costmaps skip.  Not the no-return value: that would trace free
    space down a bearing the sensor never measured.

    Why this exists: the real LDS-02 does not emit a fixed number of readings.
    Measured on nuc11 over 100 consecutive scans: 206, 207, 208 and 209, with
    angle_min, angle_max and angle_increment all varying too - so much that
    (angle_max - angle_min) / angle_increment + 1 is not even an integer
    (207.502).  Karto fixes the sensor's reading count from the first scan it
    sees and rejects every later scan whose count differs:

        LaserRangeScan contains 209 range readings, expected 210

    so SLAM discarded almost the whole stream, /map stayed empty, and with it
    went the frontiers, the navigation, the return and the grid deliverable.
    Gazebo's LiDAR always emits exactly 360, so no simulated run could show it.
    """
    n_out, out_min, out_inc = grid
    out = [float('nan')] * n_out
    n_in = len(values)
    if n_in == 0 or angle_increment <= 0.0:
        return out
    for i in range(n_out):
        j = int(round((out_min + i * out_inc - angle_min) / angle_increment))
        if 0 <= j < n_in:
            out[i] = values[j]
    return out


def drop_no_return(ranges, no_return):
    """Turn the no-return code into NaN, which Karto and the costmaps skip.

    For an ENCLOSED arena.  There a beam that returns nothing has gone through
    a gap between wall panels, and tracing it as free space out to the mapper's
    range threshold paints the world outside the arena as explored.  In the
    final arena rehearsal that starburst made 95 m2 "known" around a ~20 m2
    arena, and every goal the robot failed - five of eight, ~150 s of 240 - was
    outside the walls.  Real hits still carve free space along their own
    rays, which in a walled arena is every direction that matters.
    """
    return [float('nan') if v == no_return else v for v in ranges]


class ScanPreprocess(Node):
    def __init__(self):
        super().__init__('scan_preprocess')

        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_filtered')
        # The sensor's true horizon: rays at or beyond it, and non-finite ones,
        # are "nothing out there".  3.5 m is right for the LDS-01 and for the
        # Gazebo burger; the LDS-02 fitted to these robots reports further.
        # Set it to 0.0 to take whatever the driver puts in range_max, or pass
        # `laser_max_range:=<metres>` to the bringup.  A mismatch is not fatal
        # - readings beyond it are simply treated as "nothing there", which
        # costs mapping range but never invents an obstacle - so this node
        # checks the incoming scan and warns rather than failing.
        self.declare_parameter('sensor_max_range', 3.5)
        # Where a no-return ray is reported.  Must be far enough that the
        # phantom point falls outside Karto's correlation grid, whose
        # half-width is roughly the mapper's range threshold plus half the
        # correlation search window - under 4 m for this robot.
        self.declare_parameter('no_return_range', 20.0)
        # Published range_max.  Must exceed no_return_range or Karto discards
        # the ray again, which is the bug this node exists to fix.
        self.declare_parameter('published_range_max', 25.0)
        self.declare_parameter('sensor_min_range', 0.12)
        # Whether a reported 0.0 means "no return" (real LDS hardware) rather
        # than "an obstacle at zero range" (nothing reports that).  Gazebo uses
        # inf and never exercises this; the robot does.  See map_range.
        self.declare_parameter('zero_is_no_return', True)
        # Resample every scan onto the first scan's angular grid, so the
        # reading count never changes.  See resample(): the real LDS-02 varies
        # between 206 and 209 readings a scan and Karto rejects any mismatch.
        # In Gazebo the count is constant and this is an exact identity.
        self.declare_parameter('fixed_beam_count', True)
        # Trace no-return beams as free space (the no_return_range trick
        # above).  True for the open maze, where it is CLAUDE.md defect 1's fix.
        # The robot bringup passes False: in a walled arena a no-return beam is
        # a leak through a gap, not open space.  See drop_no_return().
        self.declare_parameter('trace_no_return', True)

        self.sensor_max = float(self.get_parameter('sensor_max_range').value)
        self.trace_no_return = bool(self.get_parameter('trace_no_return').value)
        self.get_logger().info(
            'no-return beams: {}'.format(
                'traced as free space (open-maze mode)' if self.trace_no_return
                else 'DROPPED - walled-arena mode, no free space leaks '
                     'through gaps in the walls'))
        self.auto_max = self.sensor_max <= 0.0
        self.sensor_min = float(self.get_parameter('sensor_min_range').value)
        self.no_return = float(self.get_parameter('no_return_range').value)
        self.published_max = float(self.get_parameter('published_range_max').value)
        self.zero_is_no_return = bool(
            self.get_parameter('zero_is_no_return').value)
        self.fixed_beam_count = bool(
            self.get_parameter('fixed_beam_count').value)
        self._grid = None            # (n, angle_min, angle_increment), from scan 1
        self._counts_seen = set()
        self._reported_resample = False

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

        # Captured before the message is rewritten: the log below reports what
        # the driver said, and scan.range_max is about to stop being that.
        driver_max = scan.range_max

        ranges = [map_range(v, horizon, floor, self.no_return,
                            self.zero_is_no_return)
                  for v in scan.ranges]
        no_return = sum(1 for v in ranges if v == self.no_return)
        too_close = sum(1 for v in ranges if v != v)
        if not self.trace_no_return:
            ranges = drop_no_return(ranges, self.no_return)

        if self.fixed_beam_count and ranges:
            if self._grid is None:
                # The first scan defines the grid.  angle_max is recomputed
                # from it rather than copied from the driver, whose own value is
                # inconsistent with its count - so that Karto's expected count,
                # round((max - min) / increment) + 1, is exactly n on every scan.
                self._grid = (len(ranges), scan.angle_min, scan.angle_increment)
            n_out, out_min, out_inc = self._grid
            self._counts_seen.add(len(ranges))
            total_time = scan.time_increment * len(ranges)
            intensities = list(scan.intensities)
            if len(ranges) != n_out or scan.angle_min != out_min \
                    or scan.angle_increment != out_inc:
                ranges = resample(ranges, scan.angle_min, scan.angle_increment,
                                  self._grid)
                intensities = (resample(intensities, scan.angle_min,
                                        scan.angle_increment, self._grid)
                               if len(intensities) == len(scan.ranges) else [])
            scan.angle_min = out_min
            scan.angle_increment = out_inc
            scan.angle_max = out_min + (n_out - 1) * out_inc
            scan.time_increment = total_time / n_out if n_out else 0.0
            scan.intensities = intensities
            if not self._reported_resample and len(self._counts_seen) > 1:
                self._reported_resample = True
                self.get_logger().warn(
                    'the LiDAR is emitting a varying number of readings per '
                    'scan ({}); resampling every scan to a fixed {} so '
                    'slam_toolbox does not reject them.  Expected on a real '
                    'LDS-02.'.format(
                        ', '.join(str(c) for c in sorted(self._counts_seen)),
                        n_out))

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
                .format(scan.range_min, driver_max, horizon,
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
