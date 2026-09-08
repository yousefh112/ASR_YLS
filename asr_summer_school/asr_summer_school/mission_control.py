#! /usr/bin/env python3
"""Mission orchestrator: the state machine that runs the whole challenge.

    INIT -> EXPLORE -> RETURN -> FINALIZE -> DONE

A plain state machine rather than a behavior tree: the scoring table gives no
credit for a BT, and a state machine is far quicker to reason about when a run
misbehaves with fifteen minutes of lab time left.

The ordering of concerns follows the scoring table.  Returning on time is
+150 against -30 for being a minute late, so the return budget is re-checked
every few seconds against a path the Nav2 planner actually produced, and
exploration is abandoned the moment the reserve is spent.  The two map exports
are worth 200 together, so they run in a `finally` block and happen even if the
mission is interrupted or throws.

Note on the two navigators: `BasicNavigator.getPath()` overwrites the same
`goal_handle`, `result_future` and `status` attributes that `isTaskComplete()`
and `cancelTask()` read.  Querying the return path on the driving navigator
while a goal is in flight would therefore make the orchestrator believe the
robot had arrived.  A second `BasicNavigator` instance is used for path queries
so the two cannot interfere.
"""

import math
import os
import time
from enum import Enum

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from asr_summer_school.frontier_client import FrontierMonitor
from asr_summer_school.frontier_policy import FrontierPolicy
from asr_summer_school.map_export import (grid_statistics, save_occupancy_grid,
                                          save_overlay, save_report,
                                          save_semantic_map)
from asr_summer_school.map_recorder import MapRecorder
from asr_summer_school.mission_clock import MissionClock, path_length
from asr_summer_school.tag_manager import TagManager


class State(Enum):
    INIT = 'init'
    EXPLORE = 'explore'
    RETURN = 'return'
    FINALIZE = 'finalize'
    DONE = 'done'


def yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionSupport(Node):
    """Parameters and TF lookups.  Spun in the background executor."""

    def __init__(self):
        super().__init__('mission_control')

        self.declare_parameter('mission_duration', 600.0)
        self.declare_parameter('map_frame', 'map')
        # base_footprint is the TurtleBot3 convention and what slam_toolbox
        # publishes against.
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('output_directory', '~/asr_mission_output')

        # Return-home budget.
        self.declare_parameter('return_speed', 0.15)
        self.declare_parameter('return_safety_factor', 1.3)
        self.declare_parameter('return_fixed_margin', 20.0)
        self.declare_parameter('budget_update_period', 3.0)
        self.declare_parameter('home_tolerance', 0.5)
        self.declare_parameter('odom_frame', 'odom')
        # A discontinuity in map->odom this large in one budget period is a
        # graph re-optimisation, not the robot moving.  Legitimate corrections
        # arrive in centimetres.
        self.declare_parameter('map_jump_threshold', 0.35)
        self.declare_parameter('map_jump_threshold_rad', 0.26)
        # After a jump, aim at the start pose as odometry remembers it rather
        # than as the map does.  See MissionControl.home_goal().
        self.declare_parameter('trust_odom_after_jump', True)
        self.declare_parameter('return_retries', 3)
        self.declare_parameter('return_grace', 45.0)

        # Exploration.
        # A goal's patience scales with how far away it is: a flat 60 s is
        # generous for a 2 m hop and mean for a 7 m one across the arena, and
        # both cases occur in the same run.  The estimate uses the same
        # effective speed as the return budget.
        self.declare_parameter('goal_timeout_min', 30.0)
        self.declare_parameter('goal_timeout_max', 90.0)
        self.declare_parameter('goal_timeout_factor', 2.0)
        # Seconds, not ticks.  The tick count this replaced expired after three
        # seconds of an empty frontier list, which is what a map looks like a
        # moment after startup, so the mission declared the arena explored
        # before the robot had moved.
        self.declare_parameter('no_frontier_timeout', 25.0)
        # Empty frontier list survives this many recovery spins before the
        # arena is accepted as explored.
        self.declare_parameter('recovery_spins', 3)
        # A full turn on the spot before exploring.  One stationary scan gives
        # slam_toolbox a speckled map with unknown cells scattered through the
        # free space, and the detector then clusters that speckle into a single
        # centroid sitting on top of the robot.  A turn costs about fifteen
        # seconds and produces a map the frontier detector can actually use.
        self.declare_parameter('initial_spin', 6.28)
        self.declare_parameter('spin_time_allowance', 25.0)
        self.declare_parameter('loop_period', 0.4)
        self.declare_parameter('min_frontier_distance', 0.45)
        self.declare_parameter('blacklist_radius', 0.8)
        self.declare_parameter('frontier_max_attempts', 2)
        # Planner refusals are counted separately from drive failures and the
        # resulting ban lapses: early in a run the costmap between here and a
        # frontier is mostly unknown, and "no path" means "not yet".
        self.declare_parameter('frontier_unreachable_attempts', 3)
        self.declare_parameter('frontier_unreachable_ttl', 90.0)
        self.declare_parameter('turn_penalty', 0.6)
        self.declare_parameter('hysteresis', 1.35)
        # Ask the planner for the true cost of the top few candidates instead
        # of trusting euclidean distance, which is a poor proxy in a maze.
        self.declare_parameter('use_planner_costs', True)
        self.declare_parameter('candidates_to_plan', 5)
        # Below this much slack, stop starting long trips across the arena.
        self.declare_parameter('horizon_slack_threshold', 150.0)
        self.declare_parameter('min_horizon', 1.5)
        # Rotate on arrival at a frontier so the forward-facing camera sweeps
        # the whole area rather than only the direction of travel.  The camera
        # sees about 60 degrees, so a tag on a wall the robot drove past
        # without turning towards is simply never detected - and a tag is 50
        # points against the 30 the entire accuracy category is worth.
        self.declare_parameter('scan_rotation', 6.28)
        # ...but only while there is time to spare.  A sweep is worth roughly
        # ten seconds; spending it when the return budget is thin trades 50
        # points for a 180 point swing.
        self.declare_parameter('scan_rotation_min_slack', 90.0)

        self.declare_parameter('startup_timeout', 120.0)

        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.listener = tf2_ros.TransformListener(self.buffer, self, spin_thread=False)

    def param(self, name):
        return self.get_parameter(name).value

    @property
    def output_directory(self):
        return os.path.expanduser(self.param('output_directory'))

    # ------------------------------------------------------------------ #
    # Pose
    # ------------------------------------------------------------------ #

    def resolve_base_frame(self, timeout=20.0):
        """Return whichever base frame TF actually has.

        The provided configs disagree: slam_toolbox and amcl use
        `base_footprint` while bt_navigator and both costmaps use `base_link`.
        Rather than guess, ask TF.
        """
        preferred = self.param('base_frame')
        map_frame = self.param('map_frame')
        deadline = time.time() + timeout
        while time.time() < deadline:
            for candidate in (preferred, 'base_footprint', 'base_link'):
                if self.buffer.can_transform(map_frame, candidate, Time(),
                                             timeout=Duration(seconds=0.2)):
                    return candidate
            time.sleep(0.2)
        return None

    def transform(self, target, source, timeout=0.3):
        """(x, y, yaw) of `source` expressed in `target`, or None."""
        try:
            t = self.buffer.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=timeout))
        except tf2_ros.TransformException:
            return None
        p = t.transform.translation
        return (p.x, p.y, yaw_from_quaternion(t.transform.rotation))

    def robot_pose(self, base_frame):
        """(x, y, yaw) in the map frame, or None if TF cannot answer."""
        try:
            transform = self.buffer.lookup_transform(
                self.param('map_frame'), base_frame, Time(),
                timeout=Duration(seconds=0.3))
        except tf2_ros.TransformException:
            return None
        t = transform.transform.translation
        return (t.x, t.y, yaw_from_quaternion(transform.transform.rotation))


class MissionControl:
    """Drives the mission from INIT to DONE."""

    def __init__(self, support, navigator, planner, frontiers, tags, maps):
        self.support = support
        self.navigator = navigator
        self.planner = planner
        self.frontiers = frontiers
        self.tags = tags
        self.maps = maps
        self.log = support.get_logger()

        self.state = State.INIT
        self.base_frame = None
        self.home = None
        self.clock = MissionClock(
            support.get_clock(),
            support.param('mission_duration'),
            return_speed=support.param('return_speed'),
            safety_factor=support.param('return_safety_factor'),
            fixed_margin=support.param('return_fixed_margin'))
        self.selector = FrontierPolicy(
            min_distance=support.param('min_frontier_distance'),
            blacklist_radius=support.param('blacklist_radius'),
            max_attempts=support.param('frontier_max_attempts'),
            turn_penalty=support.param('turn_penalty'),
            hysteresis=support.param('hysteresis'),
            candidates_to_plan=support.param('candidates_to_plan'),
            unreachable_attempts=support.param('frontier_unreachable_attempts'),
            unreachable_ttl=support.param('frontier_unreachable_ttl'))

        self.home_odom = None
        self.map_to_odom = None
        self.map_jumps = []

        self.target = None
        self.goal_started_at = None
        self.goal_patience_s = 60.0
        self.empty_since = None
        self.recoveries_used = 0
        self.last_budget_update = 0.0

        self.goals_sent = 0
        self.goals_reached = 0
        self.goals_failed = 0
        self.returned_home = False
        self.final_distance = None
        self.final_pose = None
        self.stop_reason = 'not started'

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def pose(self):
        return self.support.robot_pose(self.base_frame)

    def pose_stamped(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = self.support.param('map_frame')
        goal.header.stamp = self.support.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation = yaw_to_quaternion(yaw)
        return goal

    def distance_home(self):
        current = self.pose()
        home = self.home_goal()
        if current is None or home is None:
            return None
        return math.hypot(current[0] - home[0], current[1] - home[1])

    # ------------------------------------------------------------------ #
    # INIT
    # ------------------------------------------------------------------ #

    def initialise(self):
        timeout = self.support.param('startup_timeout')

        self.log.info('waiting for Nav2 to become active...')
        # There is no AMCL in this stack: SLAM provides the map->odom
        # transform, so the localizer to wait on is the controller server.
        self.navigator.waitUntilNav2Active(localizer='controller_server')
        self.log.info('Nav2 active')

        self.base_frame = self.support.resolve_base_frame(timeout=timeout)
        if self.base_frame is None:
            self.stop_reason = 'no map->base transform; is SLAM running?'
            self.log.error(self.stop_reason)
            return False
        self.log.info('using base frame "{}"'.format(self.base_frame))

        if not self.maps.wait_for_map(timeout=timeout):
            self.log.warn('no /map yet; continuing, export may be empty')

        if not self.frontiers.wait_for_frontiers(timeout=30.0):
            self.log.warn('no frontier centroids yet; is frontier_detection_node running '
                          'and is its pose_topic set to "pose"?')

        start = self.pose()
        if start is None:
            self.stop_reason = 'could not read the start pose'
            self.log.error(self.stop_reason)
            return False

        self.home = start
        # The same physical spot, remembered in the odometry frame.  map and
        # odom coincide at this instant, so the two agree now and can only
        # disagree later - which is precisely the signal worth having.
        self.home_odom = self.support.transform(
            self.support.param('odom_frame'), self.base_frame, timeout=2.0)
        self.map_to_odom = self.support.transform(
            self.support.param('map_frame'), self.support.param('odom_frame'),
            timeout=2.0)
        if self.home_odom is None:
            self.log.warn('no {}->{} transform; the odometry fallback for the '
                          'return is unavailable'.format(
                              self.support.param('odom_frame'), self.base_frame))
        self.clock.start()
        self.log.info('home recorded at ({:.2f}, {:.2f}), mission window {:.0f} s'
                      .format(self.home[0], self.home[1], self.clock.duration))
        self.state = State.EXPLORE

        # Seed the map before asking the detector for frontiers.  From a single
        # stationary scan slam_toolbox produces a fan of free cells riddled
        # with unknown gaps, the detector marks every one of those gaps as a
        # frontier, and DBSCAN merges the lot into one centroid a few
        # centimetres from the robot, which is then rejected as too close.  The
        # symptom is "exploration complete" three seconds into the run.
        self.spin_in_place(self.support.param('initial_spin'),
                           reason='seeding the map')
        return True

    # ------------------------------------------------------------------ #
    # Return budget
    # ------------------------------------------------------------------ #

    def check_for_map_jump(self):
        """Watch map->odom for the discontinuity a bad loop closure leaves.

        SLAM corrects the map->odom transform continuously, in centimetres, as
        the scan matcher refines the estimate.  A loop closure that matches the
        wrong place instead moves it metres at once, and everything expressed
        in the map frame - including the recorded start pose - silently moves
        with it.  Nothing else in the stack notices: Nav2 happily drives to a
        goal that is no longer where the robot started.
        """
        current = self.support.transform(
            self.support.param('map_frame'), self.support.param('odom_frame'))
        if current is None:
            return
        previous, self.map_to_odom = self.map_to_odom, current
        if previous is None:
            return

        shift = math.hypot(current[0] - previous[0], current[1] - previous[1])
        turn = abs(math.atan2(math.sin(current[2] - previous[2]),
                              math.cos(current[2] - previous[2])))
        if (shift < self.support.param('map_jump_threshold')
                and turn < self.support.param('map_jump_threshold_rad')):
            return

        self.map_jumps.append({'at_s': round(self.clock.elapsed(), 1),
                               'shift_m': round(shift, 3),
                               'turn_deg': round(math.degrees(turn), 1)})
        self.log.warn(
            'SLAM moved the map by {:.2f} m / {:.0f} deg at t+{:.0f}s - that is '
            'a graph re-optimisation, not motion. The recorded start pose moved '
            'with it; the return will be aimed using odometry instead.'
            .format(shift, math.degrees(turn), self.clock.elapsed()))

    def home_goal(self):
        """Where to drive to get physically back to the start.

        Normally the start pose as first recorded in the map frame: SLAM is the
        better estimate and small map corrections are improvements, which the
        fixed map coordinate correctly ignores.

        After a jump, that coordinate no longer denotes the place the robot
        started from, so the odometry anchor is converted into current map
        coordinates instead.  Odometry drifts, but it drifts slowly and
        continuously - it cannot teleport - and over a ten minute run that is
        the lesser error by a wide margin.
        """
        if (not self.map_jumps
                or not self.support.param('trust_odom_after_jump')
                or self.home_odom is None or self.map_to_odom is None):
            return self.home

        ox, oy, oyaw = self.home_odom
        mx, my, myaw = self.map_to_odom
        cos, sin = math.cos(myaw), math.sin(myaw)
        return (mx + cos * ox - sin * oy,
                my + sin * ox + cos * oy,
                myaw + oyaw)

    def update_budget(self, force=False):
        """Re-estimate the cost of getting home from where the robot is now."""
        now = self.clock.elapsed()
        if not force and now - self.last_budget_update < self.support.param('budget_update_period'):
            return
        self.last_budget_update = now
        self.check_for_map_jump()

        current = self.pose()
        home = self.home_goal()
        if current is None or home is None:
            return

        straight = math.hypot(current[0] - home[0], current[1] - home[1])

        path = None
        # BasicNavigator._getPathImpl opens with an unbounded
        # `while not wait_for_server(timeout_sec=1.0)` loop that never raises,
        # so a dead or deactivated planner_server would block this thread
        # forever and the robot would never start its return: the single most
        # expensive failure available. Check the server first, and fall back to
        # the straight-line estimate rather than waiting on it.
        try:
            server_up = self.planner.compute_path_to_pose_client.wait_for_server(
                timeout_sec=1.0)
        except Exception:
            server_up = False

        if server_up:
            try:
                start = self.pose_stamped(*current)
                goal = self.pose_stamped(home[0], home[1], home[2])
                # use_start=False: let the planner start from the live robot pose.
                path = self.planner.getPath(start, goal, use_start=False)
            except Exception as error:
                self.log.debug('return path query failed: {}'.format(error))
        else:
            self.log.warn('planner_server unavailable; using the straight-line '
                          'return estimate')

        if self.clock.update_from_path(path) is None:
            # No path available; assume a detour around whatever is in the way.
            self.clock.update_from_distance(straight)

    # ------------------------------------------------------------------ #
    # EXPLORE
    # ------------------------------------------------------------------ #

    def frontier_horizon(self):
        """Cap on how far a new target may be, tightened as time runs out."""
        slack = self.clock.slack()
        if slack >= self.support.param('horizon_slack_threshold'):
            return None
        # Whatever is left after reserving the trip home, spent half on getting
        # there: starting a run across the arena this late loses the deadline.
        reachable = slack * self.support.param('return_speed') * 0.5
        return max(self.support.param('min_horizon'), reachable)

    def within_horizon(self, centroids, current):
        horizon = self.frontier_horizon()
        if horizon is None:
            return centroids
        return [c for c in centroids
                if math.hypot(c[0] - current[0], c[1] - current[1]) <= horizon]

    def plan_cost(self, candidate):
        """True path length to a candidate, or None if the planner refuses it.

        None is meaningful to FrontierPolicy: it blacklists the candidate on
        the spot, which is far cheaper than discovering it by driving there.
        Only called once the planner is known to be up, so None really does
        mean unreachable rather than unavailable.
        """
        current = self.pose()
        if current is None:
            return None
        heading = math.atan2(candidate[1] - current[1], candidate[0] - current[0])
        try:
            path = self.planner.getPath(
                self.pose_stamped(*current),
                self.pose_stamped(candidate[0], candidate[1], heading),
                use_start=False)
        except Exception as error:
            self.log.debug('cost query failed: {}'.format(error))
            return None
        if path is None or len(path.poses) < 2:
            return None
        return path_length(path)

    def planner_cost_fn(self):
        """`plan_cost`, or None when the euclidean heuristic should be used."""
        if not self.support.param('use_planner_costs'):
            return None
        # Near the deadline the cheap heuristic wins: a slow planner query
        # costs more than a slightly worse choice of frontier.
        if self.clock.slack() < self.support.param('horizon_slack_threshold'):
            return None
        try:
            if not self.planner.compute_path_to_pose_client.wait_for_server(
                    timeout_sec=1.0):
                return None
        except Exception:
            return None
        return self.plan_cost

    def goal_patience(self, distance):
        """How long to let one frontier goal run, given how far away it is."""
        speed = max(0.05, self.support.param('return_speed'))
        estimate = (distance / speed) * self.support.param('goal_timeout_factor')
        return min(max(estimate, self.support.param('goal_timeout_min')),
                   self.support.param('goal_timeout_max'))

    def dispatch(self, target, current):
        """Send the robot to a frontier, facing the way it travelled."""
        heading = math.atan2(target[1] - current[1], target[0] - current[0])
        goal = self.pose_stamped(target[0], target[1], heading)
        self.navigator.goToPose(goal)
        self.target = target
        self.goal_started_at = self.clock.elapsed()
        distance = math.hypot(target[0] - current[0], target[1] - current[1])
        self.goal_patience_s = self.goal_patience(distance)
        self.goals_sent += 1
        self.log.info(
            'goal {}: ({:.2f}, {:.2f})  {:.1f} m away, {:.0f} s allowed  |  {}'
            .format(self.goals_sent, target[0], target[1], distance,
                    self.goal_patience_s, self.clock.summary()))

    def finish_goal(self, reached, note):
        if self.target is not None:
            if reached:
                self.selector.note_success(self.target)
                self.goals_reached += 1
            else:
                # Two failures blacklist the frontier; one may just have been a
                # transient costmap obstruction.
                if self.selector.note_failure(self.target, now=self.clock.elapsed()):
                    note += ', blacklisted'
                self.goals_failed += 1
            self.log.info('goal {} {} ({} tags so far)'.format(
                self.goals_sent, note, self.tags.count))
        self.target = None
        self.goal_started_at = None

    def spin_in_place(self, angle, allowance=None, reason=''):
        """Rotate on the spot, aborting the moment the return budget is due.

        Nav2's Spin behaviour is used rather than raw cmd_vel so the rotation
        still respects the collision monitor and shows up in the BT log.  A
        failure here is never fatal: the caller carries on without the sweep.
        """
        if angle is None or angle <= 0.0:
            return False
        if allowance is None:
            allowance = self.support.param('spin_time_allowance')
        if reason:
            self.log.info('spinning {:.1f} rad: {}'.format(angle, reason))
        try:
            self.navigator.spin(spin_dist=float(angle),
                                time_allowance=int(allowance))
            while not self.navigator.isTaskComplete():
                if self.clock.must_return() or not rclpy.ok():
                    self.navigator.cancelTask()
                    return False
                time.sleep(0.2)
            return self.navigator.getResult() == TaskResult.SUCCEEDED
        except Exception as error:
            self.log.warn('spin failed: {}'.format(error))
            return False

    def look_around(self):
        """Sweep the forward-facing camera over the area around a frontier.

        Skipped when the return budget is tight: the sweep is worth a tag at
        50 points, and being a minute late costs 180.
        """
        angle = self.support.param('scan_rotation')
        if angle <= 0.0:
            return
        slack = self.clock.slack()
        if slack < self.support.param('scan_rotation_min_slack'):
            self.log.info('skipping the camera sweep, only {:.0f} s of slack'
                          .format(slack))
            return
        self.spin_in_place(angle, reason='sweeping for tags')

    def nothing_to_explore(self, now):
        """Handle a selection that came back empty.

        An empty frontier list is ambiguous: it is what a finished arena looks
        like, and also what a map looks like a few seconds after startup, right
        after a costmap wipe, or while the detector is between publications.
        Treating the first empty tick as "done" ends the mission in three
        seconds; treating it as never-done leaves the robot idling until the
        deadline.  So an empty list has to persist for `no_frontier_timeout`
        seconds, and then survive a full turn on the spot, which both refreshes
        the map and often exposes a frontier that a stale costmap was hiding.
        """
        if self.empty_since is None:
            self.empty_since = now
            self.log.info('no frontier to drive to: {}'.format(
                self.selector.last_rejection))
            return

        if now - self.empty_since < self.support.param('no_frontier_timeout'):
            return

        if self.recoveries_used < int(self.support.param('recovery_spins')):
            self.recoveries_used += 1
            self.log.warn(
                'no frontiers for {:.0f} s ({}); recovery {}/{}'.format(
                    now - self.empty_since, self.selector.last_rejection,
                    self.recoveries_used, int(self.support.param('recovery_spins'))))
            try:
                self.navigator.clearAllCostmaps()
            except Exception as error:
                self.log.debug('costmap clear failed: {}'.format(error))
            self.spin_in_place(6.28, reason='looking for frontiers')
            # Give the detector a fresh map to work from before judging again.
            self.empty_since = self.clock.elapsed()
            return

        self.stop_reason = 'exploration complete, no frontiers left ({})'.format(
            self.selector.last_rejection)
        self.log.info(self.stop_reason)
        self.state = State.RETURN

    def explore_step(self):
        self.update_budget()
        now = self.clock.elapsed()

        if self.clock.must_return():
            self.stop_reason = 'return budget spent'
            if self.target is not None:
                self.navigator.cancelTask()
                self.finish_goal(False, 'cancelled for return')
            self.state = State.RETURN
            return

        if self.target is not None:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.finish_goal(True, 'reached')
                    self.look_around()
                else:
                    self.finish_goal(False, 'failed ({})'.format(result))
                    # Sweep here too.  The robot is somewhere it has not been
                    # even if it did not reach the frontier, the camera only
                    # sees 60 degrees, and turning on the spot doubles as a
                    # recovery from whatever blocked the approach.
                    self.look_around()
            elif now - self.goal_started_at > self.goal_patience_s:
                self.navigator.cancelTask()
                self.finish_goal(False, 'timed out')
                self.look_around()
            return

        current = self.pose()
        if current is None:
            return

        self.selector.prune(now)
        self.selector.update(self.within_horizon(self.frontiers.centroids, current))
        target = self.selector.select(
            (current[0], current[1]),
            robot_yaw=current[2],
            current_goal=self.target,
            cost_fn=self.planner_cost_fn(),
            now=now)

        if target is None:
            self.nothing_to_explore(now)
            return

        self.empty_since = None
        self.dispatch(target, current)

    # ------------------------------------------------------------------ #
    # RETURN
    # ------------------------------------------------------------------ #

    def go_home(self):
        tolerance = self.support.param('home_tolerance')
        retries = int(self.support.param('return_retries'))
        grace = self.support.param('return_grace')

        self.log.info('heading home: {}'.format(self.clock.summary()))

        for attempt in range(1, retries + 1):
            # Re-read each attempt: a jump between attempts moves the target.
            home = self.home_goal()
            goal = self.pose_stamped(home[0], home[1], home[2])
            self.navigator.goToPose(goal)

            while not self.navigator.isTaskComplete():
                if self.clock.remaining() < -grace:
                    self.log.error('past the deadline by more than {:.0f} s, giving up '
                                   'on the drive home'.format(grace))
                    self.navigator.cancelTask()
                    break
                time.sleep(0.2)

            self.final_pose = self.pose()
            distance = self.distance_home()
            if distance is not None and distance <= tolerance:
                self.returned_home = True
                self.final_distance = distance
                self.log.info('home: {:.2f} m from start with {:.0f} s to spare'
                              .format(distance, self.clock.remaining()))
                return

            self.final_distance = distance
            self.log.warn('attempt {}/{} ended {} m from home'.format(
                attempt, retries,
                'unknown' if distance is None else '{:.2f}'.format(distance)))

            if self.clock.remaining() < -grace:
                break
            # A stale costmap is the usual reason the final approach fails.
            self.navigator.clearAllCostmaps()
            time.sleep(1.0)

        self.log.error('did not get within {:.2f} m of the start'.format(tolerance))

    # ------------------------------------------------------------------ #
    # FINALIZE
    # ------------------------------------------------------------------ #

    def export(self):
        directory = self.support.output_directory
        written = []
        # One last look, in case a map arrived while the robot was driving home.
        grid = self.maps.grid

        if grid is None:
            self.log.error('no occupancy grid was ever received; '
                           'the map deliverable cannot be written')
        else:
            try:
                written.extend(save_occupancy_grid(grid, directory))
                self.log.info('occupancy grid -> {}'.format(written[-1]))
            except Exception as error:
                self.log.error('occupancy grid export FAILED: {}'.format(error))

        tags = self.tags.as_list()
        report = {
            'mission_duration_s': self.clock.duration,
            'elapsed_s': round(self.clock.elapsed(), 1),
            'finished_within_deadline': self.clock.remaining() >= 0.0,
            # Positive when the deadline was blown, which is what the late
            # schedule in score_report.py is indexed by.
            'seconds_late': round(max(0.0, -self.clock.remaining()), 1),
            'stop_reason': self.stop_reason,
            'returned_home': self.returned_home,
            'home_pose': None if self.home is None else {
                'x': round(self.home[0], 3), 'y': round(self.home[1], 3),
                'yaw': round(self.home[2], 3)},
            'final_distance_from_home_m': (
                None if self.final_distance is None else round(self.final_distance, 3)),
            'unique_tags': len(tags),
            'tag_ids': [t['id'] for t in tags],
            'goals_sent': self.goals_sent,
            'goals_reached': self.goals_reached,
            'goals_failed': self.goals_failed,
            'map_jumps': self.map_jumps,
            'return_anchored_on': 'odometry' if self.map_jumps else 'map',
            'home_goal_used': None if self.home_goal() is None else {
                'x': round(self.home_goal()[0], 3),
                'y': round(self.home_goal()[1], 3)},
            'blacklisted_frontiers': len(
                self.selector.active_blacklist(self.clock.elapsed())),
            'final_pose': None if self.final_pose is None else {
                'x': round(self.final_pose[0], 3),
                'y': round(self.final_pose[1], 3),
                'yaw': round(self.final_pose[2], 3)},
            'map': None if grid is None else grid_statistics(grid),
        }

        try:
            written.extend(save_semantic_map(tags, directory, metadata=report))
            self.log.info('semantic map -> {}'.format(
                os.path.join(directory, 'semantic_map.{yaml,json,csv}')))
        except Exception as error:
            self.log.error('semantic map export FAILED: {}'.format(error))

        try:
            written.append(save_report(report, directory))
        except Exception as error:
            self.log.error('mission report export FAILED: {}'.format(error))

        # A picture of the grid with the tags, the start and the finish drawn on
        # it.  Not scored, but it is the fastest way for a human to see whether
        # a run went the way the numbers claim, and it costs nothing.
        try:
            overlay = None if grid is None else save_overlay(
                grid, tags, os.path.join(directory, 'mission_overlay.png'),
                home=self.home, final=self.final_pose)
            if overlay:
                written.append(overlay)
                self.log.info('overlay -> {}'.format(overlay))
        except Exception as error:
            self.log.warn('overlay export skipped: {}'.format(error))

        if self.map_jumps:
            self.log.warn(
                'SLAM re-optimised the map {} times during this run; the '
                'largest moved it {:.2f} m. The map and the tag positions in '
                'it are only as good as the last of those.'.format(
                    len(self.map_jumps),
                    max(j['shift_m'] for j in self.map_jumps)))

        self.log.info(
            '=== mission over: {} unique tags {}, {}, {:.0f}s of {:.0f}s, '
            '{}/{} goals reached ==='.format(
                report['unique_tags'], report['tag_ids'],
                'returned home' if self.returned_home else 'NOT home',
                report['elapsed_s'], self.clock.duration,
                self.goals_reached, self.goals_sent))
        return written

    # ------------------------------------------------------------------ #
    # The loop
    # ------------------------------------------------------------------ #

    def run(self):
        period = self.support.param('loop_period')
        try:
            if not self.initialise():
                return
            while self.state is State.EXPLORE and rclpy.ok():
                self.explore_step()
                time.sleep(period)
            if self.state is State.RETURN and rclpy.ok():
                self.go_home()
        except KeyboardInterrupt:
            self.stop_reason = 'interrupted'
            self.log.warn('interrupted; exporting what we have')
        finally:
            # 200 of the available points live in these files, so the export
            # runs whatever happened above.
            self.state = State.FINALIZE
            self.export()
            self.state = State.DONE


def main(args=None):
    rclpy.init(args=args)

    support = MissionSupport()
    use_sim_time = support.get_parameter('use_sim_time').value

    frontiers = FrontierMonitor()
    tags = TagManager()
    maps = MapRecorder()

    for node in (frontiers, tags, maps):
        node.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, bool(use_sim_time))])

    # BasicNavigator does not inherit use_sim_time, and a wall-clock stamp on a
    # goal sent into a sim-time stack is rejected by the BT navigator.
    navigator = BasicNavigator()
    planner = BasicNavigator(node_name='mission_path_query')
    for node in (navigator, planner):
        node.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, bool(use_sim_time))])

    # The return budget re-queries the planner every few seconds and
    # BasicNavigator logs "Getting path..." at info each time, which buries the
    # mission's own log under a line every three seconds for the whole run.
    # Warnings and errors from it still come through.
    planner.get_logger().set_level(LoggingSeverity.WARN)

    executor = MultiThreadedExecutor()
    for node in (support, frontiers, tags, maps):
        executor.add_node(node)

    import threading
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    mission = MissionControl(support, navigator, planner, frontiers, tags, maps)
    try:
        mission.run()
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        for node in (support, frontiers, tags, maps, navigator, planner):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
