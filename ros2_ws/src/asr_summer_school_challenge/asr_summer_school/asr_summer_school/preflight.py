#! /usr/bin/env python3
"""Check the whole stack in one command, before it matters.

    ros2 run asr_summer_school preflight.py            # after the bringup
    ros2 run asr_summer_school preflight.py --nav2     # after the mission too

The runbook's pre-flight is eight commands run by hand, each needing someone to
know what a good answer looks like.  On competition day, with a battery draining
and a queue for the arena, that is the wrong time to be reading `ros2 topic hz`
output and remembering whether 4.9 Hz is fine.

Every check here failed for real at least once while the stack was being built,
and each one is silent when it fails: a dead scan preprocessor still leaves a
healthy-looking `/scan`, a camera missing from TF still produces detections, a
mapper whose horizon is set for the wrong LiDAR still makes a map.  The point is
to turn twenty wasted minutes into thirty seconds.

Exit status is 0 when nothing failed, 1 otherwise, so it can gate a script.
"""

import argparse
import math
import sys
import time
from collections import defaultdict

import rclpy
import tf2_ros
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from visualization_msgs.msg import Marker

PASS, WARN, FAIL = 'PASS', 'WARN', 'FAIL'
_MARK = {PASS: 'ok  ', WARN: 'warn', FAIL: 'FAIL'}

# apriltag_ros publishes one of these per frame, empty or not, so its rate is
# proof the camera path is alive - and it is small enough to cross the wifi,
# which the images deliberately are not.
DETECTIONS_TOPIC = '/camera/detections'

SENSOR_QOS = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST)
RELIABLE_QOS = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE,
                          history=QoSHistoryPolicy.KEEP_LAST)
LATCHED_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class Result:
    def __init__(self, name, status, detail, remedy=None):
        self.name = name
        self.status = status
        self.detail = detail
        self.remedy = remedy


class Preflight(Node):
    def __init__(self, window, use_sim_time, check_nav2):
        super().__init__('preflight')
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])
        self.window = window
        self.check_nav2 = check_nav2

        self.counts = defaultdict(int)
        self.last = {}

        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.listener = tf2_ros.TransformListener(self.buffer, self,
                                                  spin_thread=False)

        # Everything the mission consumes, subscribed with the QoS the
        # publisher actually uses.  A reliable subscription to a best-effort
        # publisher connects and then receives nothing at all, which is the
        # failure this whole file exists to catch, so it would be a poor joke
        # to reproduce it here.
        self._watch('/scan', LaserScan, SENSOR_QOS)
        self._watch('/scan_filtered', LaserScan, RELIABLE_QOS)
        self._watch('/odom', Odometry, RELIABLE_QOS)
        self._watch('/imu', Imu, SENSOR_QOS)
        self._watch('/map', OccupancyGrid, LATCHED_QOS)
        self._watch('/pose', PoseWithCovarianceStamped, RELIABLE_QOS)
        self._watch('/frontier_centroids', Marker, LATCHED_QOS)
        for topic in self._camera_topics():
            self._watch(topic, CameraInfo, SENSOR_QOS)
        self._watch(DETECTIONS_TOPIC, AprilTagDetectionArray, RELIABLE_QOS)

    # ------------------------------------------------------------------ #

    def _camera_topics(self):
        """Whichever image topic this environment publishes.

        Gazebo puts it on /camera/image_raw; the RealSense and the OAK-D in
        RealSense-compatibility mode both put it on /camera/camera/color/...
        Watching all of them and requiring one keeps this file honest across
        both, instead of encoding an assumption that only holds in simulation.
        """
        # camera_info, NOT the image, on purpose.  Every driver publishes one
        # CameraInfo per frame, so its rate IS the frame rate - but it is a few
        # hundred bytes where a 1280x720 frame is 2.7 MB.  Subscribing to the
        # image from the laptop pulls ~330 Mbit/s across the wifi, which is
        # exactly what the split deployment exists to avoid: measured on nuc11,
        # it dragged /odom from 20 Hz to 4.8 and /scan_filtered from 11 Hz to 3
        # for the length of the check - so preflight was damaging the link it
        # was meant to be measuring, and reporting the damage as the robot's.
        #
        # /camera/color/... is what the RealSense on nuc11 actually publishes:
        # its node comes up as /camera, not /camera/camera.  The other two are
        # Gazebo's name and the vendored layout, kept so this still works if a
        # driver update renames it.
        return ['/camera/camera_info', '/camera/color/camera_info',
                '/camera/camera/color/camera_info']

    def _watch(self, topic, kind, qos):
        def callback(msg, topic=topic):
            self.counts[topic] += 1
            self.last[topic] = msg
        self.create_subscription(kind, topic, callback, qos)

    def collect(self):
        deadline = time.time() + self.window
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def rate(self, topic):
        return self.counts[topic] / self.window

    # ------------------------------------------------------------------ #
    # Checks
    # ------------------------------------------------------------------ #

    def check_scan(self):
        out = []
        raw = self.rate('/scan')
        if raw <= 0.0:
            out.append(Result(
                'LiDAR /scan', FAIL, 'no messages',
                'The base bringup is not running, or the LiDAR is not '
                'enumerated. Check LDS_MODEL and /dev/ttyUSB*.'))
            return out
        out.append(Result('LiDAR /scan', PASS, '{:.1f} Hz'.format(raw)))

        filtered = self.rate('/scan_filtered')
        if filtered <= 0.0:
            out.append(Result(
                'scan_preprocess', FAIL, '/scan_filtered silent',
                'slam_toolbox AND both Nav2 costmaps read this topic, so the '
                'robot is blind. Check the bringup log for a scan_preprocess '
                'traceback.'))
            return out
        if filtered < raw * 0.8:
            out.append(Result(
                'scan_preprocess', WARN,
                '/scan_filtered {:.1f} Hz vs /scan {:.1f} Hz'.format(filtered, raw),
                'Dropping scans. Usually CPU load.'))
        else:
            out.append(Result('scan_preprocess', PASS,
                              '/scan_filtered {:.1f} Hz'.format(filtered)))

        # The invariant localisation depends on.  A no-return ray has to come
        # back finite and beyond the mapper's threshold, and range_max has to
        # be above it, or Karto either discards the ray (the map stops growing)
        # or marks a phantom obstacle at its end (the scan matcher drifts).
        scan = self.last.get('/scan_filtered')
        if scan is not None:
            ranges = list(scan.ranges)
            infinite = sum(1 for v in ranges if math.isinf(v))
            finite = [v for v in ranges if math.isfinite(v)]
            longest = max(finite) if finite else 0.0
            if infinite:
                out.append(Result(
                    'no-return encoding', FAIL,
                    '{} rays are still inf'.format(infinite),
                    'Karto discards those before they can clear anything and '
                    'the map will not grow into open space.'))
            elif longest >= scan.range_max:
                out.append(Result(
                    'no-return encoding', FAIL,
                    'longest ray {:.1f} m >= range_max {:.1f} m'.format(
                        longest, scan.range_max),
                    'Karto ignores readings at or past range_max. Raise '
                    'published_range_max above no_return_range.'))
            else:
                out.append(Result(
                    'no-return encoding', PASS,
                    'no-return at {:.1f} m, range_max {:.1f} m'.format(
                        longest, scan.range_max)))

        raw_scan = self.last.get('/scan')
        if raw_scan is not None:
            out.append(Result(
                'LiDAR horizon', PASS,
                'driver reports {:.2f} m'.format(raw_scan.range_max),
                'This is the driver NOMINAL maximum, not a usable horizon - '
                'the LDS-02 reports 100 m and cannot see anything like that. '
                'Do NOT set max_laser_range just below it: rays are rastered '
                'out to that range and the scan matcher acquires a ring of '
                'phantom obstacles, which is CLAUDE.md defect 2 and cost 6 m '
                'of drift in three minutes. laser_max_range is set from '
                'LDS_MODEL instead (LDS-01 3.5, LDS-02 8.0, LDS-03 12.0).'))
        return out

    def check_odometry(self):
        out = []
        for topic, label in (('/odom', 'wheel odometry'), ('/imu', 'IMU')):
            rate = self.rate(topic)
            if rate <= 0.0:
                out.append(Result(label, FAIL, '{} silent'.format(topic),
                                  'The OpenCR board is not talking. Check '
                                  '/dev/ttyACM0 and the dialout group.'))
            else:
                out.append(Result(label, PASS, '{:.1f} Hz'.format(rate)))
        return out

    def check_clock_skew(self):
        """Is this machine's clock close enough to the robot's?

        tag_manager drops any tag frame older than max_detection_age (1.0 s),
        and it compares the stamp apriltag_ros wrote - the ROBOT's clock - with
        `self.get_clock().now()`, which on the laptop is the LAPTOP's clock.
        That comparison is the only cross-machine one in the tag path; the TF
        lookup that follows is robot-stamped end to end and is immune.

        So if this machine runs more than a second ahead of the robot, every
        single detection is discarded at that gate, silently: no log, no
        counter, and the TF-based tag check below still reports tags in view
        because it asks for the latest transform rather than a stamped one.
        A whole run of zero tags, with nothing anywhere saying why.

        /odom is robot-stamped, always present and published at 20 Hz, so the
        gap between its newest stamp and this clock IS the quantity that gate
        compares - measured over the real path rather than inferred.
        """
        msg = self.last.get('/odom')
        if msg is None:
            return []            # check_odometry already reports the silence

        stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        skew = self.get_clock().now().nanoseconds / 1e9 - stamp
        limit = 1.0              # tag_manager's max_detection_age default

        if skew > limit:
            return [Result(
                'clock skew', FAIL,
                'this machine is {:.2f} s ahead of the robot'.format(skew),
                'Every AprilTag detection will be silently discarded: '
                'tag_manager drops frames older than max_detection_age '
                '({:.1f} s) measured against THIS clock. Fix before the run:\n'
                '    ssh -t students@192.168.10.111 '
                "'sudo timedatectl set-ntp true'\n"
                'then re-run this. If that is not available, raise '
                'max_detection_age above {:.1f} s as a stopgap - it only '
                'weakens the staleness test, where the alternative is zero '
                'tags.'.format(limit, skew + 0.5))]

        if skew > limit * 0.5:
            return [Result(
                'clock skew', WARN,
                '{:+.2f} s ahead of the robot'.format(skew),
                'Within tolerance but not by much; past {:.1f} s every '
                'detection is dropped. Worth syncing before the run.'.format(limit))]

        return [Result('clock skew', PASS,
                       '{:+.2f} s against the robot'.format(skew))]

    def check_camera(self):
        live = [(t, self.rate(t)) for t in self._camera_topics()
                if self.rate(t) > 0.0]
        if live:
            topic, rate = live[0]
            return [Result('camera', PASS,
                           '{} at {:.1f} Hz'.format(topic, rate))]

        # No images - but that is EXPECTED when preflight runs on the laptop.
        # 1280x720 at 15 fps is about 330 Mbit/s, so the images deliberately
        # never leave the robot; only the detections do (RUNBOOK section 3.0).
        # Failing here would condemn a perfectly healthy camera for behaving
        # exactly as the architecture intends.
        #
        # /camera/detections is the honest remote check: apriltag_ros publishes
        # an array on every frame, empty or not, so a live rate proves the whole
        # path - driver, image, detector - without pulling a single image across
        # the wifi.
        detections = self.rate(DETECTIONS_TOPIC)
        if detections > 0.0:
            return [Result(
                'camera', PASS,
                'no camera_info here, but {} at {:.1f} Hz'.format(
                    DETECTIONS_TOPIC, detections),
                'Normal when this runs on the laptop: the images stay on the '
                'robot by design and only the detections cross. The detector '
                'is running on live frames, which is what matters.')]

        return [Result(
            'camera', FAIL, 'no camera_info and no detections',
            'On the ROBOT: CAMERA_MODEL wrong, or the camera did not enumerate '
            'on USB - check the bringup log for "RealSense Node Is Up!". '
            'On the LAPTOP: this means apriltag is not running on the robot, '
            'since its detections would have crossed even though images do '
            'not. Tags are 50 points each; do not start a run without this.')]

    def check_transforms(self):
        """Each link has a different owner, so each gets its own remedy.

        A single "SLAM is not publishing" for all three would send someone
        restarting slam_toolbox when the actual problem is that the wheels
        never came up.
        """
        links = (
            ('map', 'odom', 'map -> odom (SLAM)',
             'slam_toolbox is not publishing a correction. It needs scans on '
             'the topic named by scan_topic, and odom -> base_footprint below.'),
            ('odom', 'base_footprint', 'odom -> base_footprint',
             'The base is not publishing odometry. On the robot that is the '
             'OpenCR over /dev/ttyACM0; in Gazebo it is the diff drive plugin.'),
            ('base_footprint', 'base_scan', 'base_footprint -> base_scan',
             'robot_state_publisher is not running or has no URDF. Check '
             'TURTLEBOT3_MODEL.'),
        )
        out = []
        for target, source, label, remedy in links:
            try:
                self.buffer.lookup_transform(target, source, Time(),
                                             timeout=Duration(seconds=2.0))
                out.append(Result(label, PASS, 'present'))
            except tf2_ros.TransformException as error:
                out.append(Result(label, FAIL,
                                  str(error).split('\n')[0].strip(), remedy))
        return out

    def check_tag_pipeline(self):
        """The camera has to be reachable from map, or no tag can be placed.

        This is the failure that cost the most time in simulation: tags were
        detected perfectly and every map-frame lookup then failed, because the
        frame apriltag_ros parents them to was not connected to the robot.
        """
        out = []
        try:
            listing = self.buffer.all_frames_as_yaml()
        except Exception:
            listing = ''

        frames = [line.rstrip()[:-1].strip()
                  for line in listing.splitlines()
                  if line and not line[0].isspace() and line.rstrip().endswith(':')]
        camera_frames = [f for f in frames
                         if 'camera' in f.lower() or f.lower().startswith('oak')]
        tag_frames = [f for f in frames if ':' in f and f.split(':')[0].startswith('tag')]

        if not camera_frames:
            out.append(Result(
                'camera in TF', FAIL, 'no camera frame in the TF tree',
                'apriltag_ros parents every tag to the image frame. If that '
                'frame is not connected to the robot, every tag is detected '
                'and then thrown away.'))
            return out

        reachable = []
        for frame in camera_frames:
            try:
                self.buffer.lookup_transform('map', frame, Time(),
                                             timeout=Duration(seconds=0.5))
                reachable.append(frame)
            except tf2_ros.TransformException:
                pass
        if reachable:
            # Lead with the colour frame: that is the one apriltag_ros parents
            # tags to, and the only one whose absence loses points.
            colour = [f for f in reachable
                      if ('rgb' in f.lower() or 'color' in f.lower())
                      and 'optical' not in f.lower()]
            shown = colour + [f for f in sorted(reachable) if f not in colour]
            out.append(Result(
                'camera in TF', PASS if colour else WARN,
                'map -> {}{}'.format(
                    ', '.join(shown[:3]),
                    '' if len(shown) <= 3 else ' (+{} more)'.format(len(shown) - 3)),
                None if colour else
                'No colour frame reachable. Tags are parented to the frame the '
                'image is stamped with; check the camera driver started.'))
        else:
            out.append(Result(
                'camera in TF', FAIL,
                'camera frames exist but none reach map: {}'.format(
                    ', '.join(sorted(camera_frames)[:4])),
                'The camera subtree is detached from the robot.'))

        if tag_frames:
            optical = [f for f in reachable if 'optical' in f.lower()]
            out.append(Result(
                'tag detection', PASS,
                '{} tag frame(s) in view: {}'.format(
                    len(tag_frames), ', '.join(sorted(tag_frames)[:4])),
                None if optical or not reachable else
                'Parent frame is not named "optical", so the optical '
                'correction will be applied. Correct under Gazebo.'))
        else:
            out.append(Result(
                'tag detection', WARN, 'no tag currently in view',
                'Hold a tag36h11 in front of the camera and re-run to confirm '
                'the whole detection path end to end.'))
        return out

    def check_mapping(self):
        out = []
        grid = self.last.get('/map')
        if grid is None:
            out.append(Result(
                'occupancy grid', FAIL, '/map has published nothing',
                'slam_toolbox is not running or has no scans to work with.'))
            return out
        info = grid.info
        known = sum(1 for v in grid.data if v >= 0)
        out.append(Result(
            'occupancy grid', PASS,
            '{}x{} at {:.2f} m, {} known cells'.format(
                info.width, info.height, info.resolution, known)))

        if self.rate('/pose') <= 0.0:
            out.append(Result(
                'slam /pose', WARN, 'no pose published during the window',
                'Normal while the robot is stationary; slam_toolbox only '
                'publishes when it processes a scan into the graph. The '
                'frontier detector centres its search on this.'))
        else:
            out.append(Result('slam /pose', PASS,
                              '{:.1f} Hz'.format(self.rate('/pose'))))

        marker = self.last.get('/frontier_centroids')
        if marker is None:
            out.append(Result(
                'frontier detector', FAIL, 'silent',
                'Nothing will tell the mission where to drive. Check '
                'frontier_detection_node started and its pose_topic is "pose".'))
        elif not marker.points:
            out.append(Result(
                'frontier detector', WARN, 'publishing, but 0 centroids',
                'With none, the mission spins its recovery turns and then goes '
                'home. Normal before the robot has turned once; suspicious if '
                'it persists after the opening spin.'))
        else:
            out.append(Result(
                'frontier detector', PASS,
                '{} centroid(s)'.format(len(marker.points))))
        return out

    def check_nav2_nodes(self):
        if not self.check_nav2:
            return []
        wanted = ['controller_server', 'planner_server', 'behavior_server',
                  'bt_navigator', 'velocity_smoother', 'smoother_server']
        present = {n for n, _ in self.get_node_names_and_namespaces()}
        missing = [n for n in wanted if n not in present]
        if missing:
            return [Result(
                'Nav2', FAIL, 'missing: {}'.format(', '.join(missing)),
                'Start mission.launch.py. Neither bringup file starts Nav2.')]
        return [Result('Nav2', PASS, '{} servers up'.format(len(wanted)))]

    def run(self):
        results = []
        results += self.check_scan()
        results += self.check_odometry()
        # After odometry, because it uses /odom's stamp as the robot's clock.
        results += self.check_clock_skew()
        results += self.check_camera()
        results += self.check_transforms()
        results += self.check_tag_pipeline()
        results += self.check_mapping()
        results += self.check_nav2_nodes()
        return results


def render(results):
    width = max(len(r.name) for r in results)
    lines = ['', 'PRE-FLIGHT', '']
    for r in results:
        lines.append('  [{}] {:<{w}}  {}'.format(
            _MARK[r.status], r.name, r.detail, w=width))
        if r.remedy and r.status != PASS:
            lines.append('       {:<{w}}  -> {}'.format('', r.remedy, w=width))
        elif r.remedy:
            lines.append('       {:<{w}}  note: {}'.format('', r.remedy, w=width))
    failed = [r for r in results if r.status == FAIL]
    warned = [r for r in results if r.status == WARN]
    lines.append('')
    if failed:
        lines.append('  {} FAILED, {} warning(s). Do not start a run.'.format(
            len(failed), len(warned)))
    elif warned:
        lines.append('  All checks passed with {} warning(s).'.format(len(warned)))
    else:
        lines.append('  All checks passed.')
    lines.append('')
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--window', type=float, default=6.0,
                        help='seconds to sample topic rates over')
    parser.add_argument('--sim', action='store_true',
                        help='use the simulation clock')
    parser.add_argument('--nav2', action='store_true',
                        help='also require the Nav2 servers to be up')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Preflight(args.window, args.sim, args.nav2)
    try:
        node.collect()
        results = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(render(results))
    return 1 if any(r.status == FAIL for r in results) else 0


if __name__ == '__main__':
    sys.exit(main())
