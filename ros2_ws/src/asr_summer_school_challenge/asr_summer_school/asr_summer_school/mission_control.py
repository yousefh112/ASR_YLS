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
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from asr_summer_school.frontier_client import FrontierMonitor
from asr_summer_school.frontier_policy import FrontierPolicy
from asr_summer_school.map_export import (grid_statistics, save_occupancy_grid,
                                          save_overlay, save_report,
                                          save_semantic_map)
from asr_summer_school.map_recorder import MapRecorder
from asr_summer_school.coverage import coverage_summary, format_coverage
from asr_summer_school.patrol import candidate_points, choose_patrol_target
from asr_summer_school.mission_clock import MissionClock, path_length
from asr_summer_school.tag_manager import TagManager


# The challenge rule: the robot counts as returned when it stops inside a
# circle of this radius centred on its starting point.  `home_tolerance` is
# deliberately tighter than this - it is what the mission aims at, leaving the
# difference as margin for the drift between the estimated and the true pose.
RETURN_RULE_RADIUS_M = 0.50


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

        # Remembers the type each parameter was declared with, so param() can
        # hand back what the code expects however the value was written.
        self._declared_types = {}

        # Dynamically typed, because these three are launch arguments a human
        # types under time pressure.  rclpy rejects an int override for a
        # double-declared parameter, so `mission_duration:=150` killed the node
        # at construction while `150.0` worked - a distinction nobody should
        # have to remember while a battery drains.  param() coerces on the way
        # out, so the rest of the code still sees a float.
        self._declare('mission_duration', 600.0, dynamic=True)
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
        # Deliberately NOT return_speed, which goal_patience used to borrow.
        # That coupled two unrelated things: how fast the robot gets home, and
        # how long a frontier goal is allowed to run.  Tuning the return budget
        # then silently shortened every goal timeout, and a goal that times out
        # is not free - it blacklists the frontier for frontier_failure_ttl.
        self.declare_parameter('goal_speed', 0.15)
        self.declare_parameter('goal_timeout_min', 30.0)
        self.declare_parameter('goal_timeout_max', 90.0)
        self.declare_parameter('goal_timeout_factor', 2.0)
        # Seconds, not ticks.  The tick count this replaced expired after three
        # seconds of an empty frontier list, which is what a map looks like a
        # moment after startup, so the mission declared the arena explored
        # before the robot had moved.
        self.declare_parameter('no_frontier_timeout', 6.0)
        # Empty frontier list survives this many recovery spins before the
        # arena is accepted as explored.
        self.declare_parameter('recovery_spins', 1)
        # Finish as soon as this many unique tags are in hand.  0 disables it.
        # The rubric pays 50 a tag and nothing for finishing early, so this is
        # only safe when the true count is known; it exists because the
        # organisers may announce it, and because a run that ends the moment it
        # has everything is the one that wins a tie on time.
        self._declare('stop_after_tags', 0, dynamic=True)
        # A full turn on the spot before exploring.  One stationary scan gives
        # slam_toolbox a speckled map with unknown cells scattered through the
        # free space, and the detector then clusters that speckle into a single
        # centroid sitting on top of the robot.  A turn costs about fifteen
        # seconds and produces a map the frontier detector can actually use.
        self.declare_parameter('initial_spin', 6.28)
        self.declare_parameter('spin_time_allowance', 25.0)
        self.declare_parameter('loop_period', 0.4)
        self.declare_parameter('min_frontier_distance', 0.45)
        # A frontier goal exists to make unknown space known.  Once the
        # detector stops reporting a frontier anywhere near the target, that
        # has happened - the robot's own LiDAR filled it in on the way - and
        # standing on the exact centroid afterwards is pure cost.  A centroid
        # against a wall may also sit inside the inflation layer, where Nav2's
        # 25 cm goal tolerance can never be met and the goal burns its entire
        # timeout for nothing.
        #
        # Deliberately not a distance test.  Accepting a goal because the robot
        # is within some radius of it couples two thresholds that then have to
        # be kept in the right order: set the radius above the distance at
        # which a frontier is worth driving to and every goal completes the
        # instant it is dispatched, the robot never moves, and the run ends
        # with four "reached" goals and a two-metre map.  Asking whether the
        # frontier still exists has no such failure mode.
        self.declare_parameter('consumed_checks', 3)
        self.declare_parameter('blacklist_radius', 0.8)
        self.declare_parameter('frontier_max_attempts', 2)
        # Planner refusals are counted separately from drive failures and the
        # resulting ban lapses: early in a run the costmap between here and a
        # frontier is mostly unknown, and "no path" means "not yet".
        self.declare_parameter('frontier_unreachable_attempts', 3)
        self.declare_parameter('frontier_unreachable_ttl', 90.0)
        # A frontier the robot drove at and could not reach is set aside for
        # much longer than one the planner merely refused - but not forever.
        self.declare_parameter('frontier_failure_ttl', 240.0)
        # How many times the whole blacklist may be thrown away rather than
        # declare an arena explored while frontiers are still on the list.
        self.declare_parameter('blacklist_clears', 2)
        self.declare_parameter('turn_penalty', 0.6)
        self.declare_parameter('hysteresis', 1.35)
        # Ask the planner for the true cost of the top few candidates instead
        # of trusting euclidean distance, which is a poor proxy in a maze.
        self.declare_parameter('use_planner_costs', True)
        self.declare_parameter('candidates_to_plan', 5)
        # Below this much slack, stop starting long trips across the arena.
        self.declare_parameter('horizon_slack_threshold', 150.0)
        self.declare_parameter('min_horizon', 1.5)

        # Patrol: what the robot does when the map is finished but the clock is
        # not.  See patrol.py for why mapping the arena does not mean the tags
        # have been found.
        self.declare_parameter('patrol_spacing', 2.0)
        self.declare_parameter('patrol_clearance', 0.25)
        self.declare_parameter('patrol_travel_weight', 0.35)
        self.declare_parameter('patrol_max_failures', 4)
        # Below this the robot is standing everywhere it can already.
        self.declare_parameter('patrol_min_spacing', 0.5)

        # How many tags the arena is known to hold.  Used by the coverage
        # report to name which ids are missing, and by stop_after_tags.  0 means
        # unknown, and the report then only counts what was found.
        self.declare_parameter('expected_tags', 0)
        # Rotate on arrival at a frontier so the forward-facing camera sweeps
        # the whole area rather than only the direction of travel.  The camera
        # sees about 60 degrees, so a tag on a wall the robot drove past
        # without turning towards is simply never detected - and a tag is 50
        # points against the 30 the entire accuracy category is worth.
        self._declare('scan_rotation', 6.28, dynamic=True)
        # ...but only while there is time to spare.  A sweep is worth roughly
        # ten seconds; spending it when the return budget is thin trades 50
        # points for a 180 point swing.
        self.declare_parameter('scan_rotation_min_slack', 90.0)

        self.declare_parameter('startup_timeout', 120.0)

        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.listener = tf2_ros.TransformListener(self.buffer, self, spin_thread=False)

    def _declare(self, name, default, dynamic=False):
        descriptor = ParameterDescriptor(dynamic_typing=True) if dynamic else None
        if descriptor is None:
            self.declare_parameter(name, default)
        else:
            self.declare_parameter(name, default, descriptor)
        self._declared_types[name] = type(default)

    def param(self, name):
        """The parameter's value, coerced back to the type it was declared as.

        A dynamically typed parameter accepts whatever the caller wrote, which
        is the point - but the code downstream does arithmetic on it and should
        not have to care whether someone typed 150 or 150.0.
        """
        value = self.get_parameter(name).value
        expected = self._declared_types.get(name)
        if expected in (float, int) and value is not None and not isinstance(value, bool):
            try:
                return expected(value)
            except (TypeError, ValueError):
                return value
        return value

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
            unreachable_ttl=support.param('frontier_unreachable_ttl'),
            failure_ttl=support.param('frontier_failure_ttl'))

        self.home_odom = None
        self.map_to_odom = None
        self.map_jumps = []

        self.target = None
        self.target_kind = 'frontier'
        self.goal_started_at = None
        self.goal_patience_s = 60.0
        self.consumed_ticks = 0
        self.empty_since = None
        self.recoveries_used = 0
        self.blacklist_clears_used = 0
        self.last_budget_update = 0.0

        # Every position the camera has swept from.  Frontier exploration
        # finishes when the map is complete, which is not the same as the tags
        # being found: a 360 degree LiDAR maps a wall from any heading, a 55
        # degree camera only photographs it from one.  These are the places
        # already looked around from, so the patrol can go somewhere else.
        self.swept = []
        self.patrolling = False
        self.patrol_goals = 0
        self.patrol_failures = 0

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

    def target_is_consumed(self):
        """True once the detector no longer reports a frontier at the target.

        Centroids shift by a few cells every time the map updates, and a target
        can briefly fail to match one, so this has to agree with itself several
        ticks running before it is believed.
        """
        if self.target is None:
            return False
        # A patrol point is not a frontier, so there is no frontier there to be
        # consumed.  Without this the very first tick of every patrol goal sees
        # "no centroid within blacklist_radius", counts towards consumed_checks
        # and cancels the goal before the robot has gone anywhere.
        if self.target_kind != 'frontier':
            return False
        radius = self.support.param('blacklist_radius')
        live = any(math.hypot(c[0] - self.target[0], c[1] - self.target[1]) <= radius
                   for c in self.frontiers.centroids)
        self.consumed_ticks = 0 if live else self.consumed_ticks + 1
        return self.consumed_ticks >= int(self.support.param('consumed_checks'))

    def goal_patience(self, distance):
        """How long to let one frontier goal run, given how far away it is."""
        speed = max(0.05, self.support.param('goal_speed'))
        estimate = (distance / speed) * self.support.param('goal_timeout_factor')
        return min(max(estimate, self.support.param('goal_timeout_min')),
                   self.support.param('goal_timeout_max'))

    def dispatch(self, target, current, kind='frontier'):
        """Send the robot to a frontier, facing the way it travelled.

        `kind` separates a frontier goal from a patrol goal.  They are driven
        identically but judged differently: a patrol point is a place to stand
        and look, so it has no frontier to be consumed and it must never reach
        the frontier selector's blacklist, which reasons about unexplored space.
        """
        heading = math.atan2(target[1] - current[1], target[0] - current[0])
        goal = self.pose_stamped(target[0], target[1], heading)
        if not self.navigator.goToPose(goal):
            # goToPose returns False when the action server REJECTS the goal.
            # Discarding it used to leave self.target set while nothing was
            # driving, and BasicNavigator keeps the *previous* task's
            # result_future, so the next isTaskComplete()/getResult() pair
            # reported that stale result - a rejected goal counted as reached.
            self.log.warn('Nav2 rejected the {} goal at ({:.2f}, {:.2f})'
                          .format(kind, target[0], target[1]))
            if kind == 'frontier':
                self.selector.note_unreachable(target, now=self.clock.elapsed())
            return False
        self.target_kind = kind
        if kind == 'patrol':
            self.patrol_goals += 1
        self.target = target
        self.goal_started_at = self.clock.elapsed()
        self.consumed_ticks = 0
        distance = math.hypot(target[0] - current[0], target[1] - current[1])
        self.goal_patience_s = self.goal_patience(distance)
        self.goals_sent += 1
        self.log.info(
            '{} goal {}: ({:.2f}, {:.2f})  {:.1f} m away, {:.0f} s allowed  |  {}'
            .format(kind, self.goals_sent, target[0], target[1], distance,
                    self.goal_patience_s, self.clock.summary()))
        return True

    def finish_goal(self, reached, note):
        if self.target is not None:
            if reached:
                self.goals_reached += 1
                if self.target_kind == 'frontier':
                    self.selector.note_success(self.target)
            else:
                self.goals_failed += 1
                # Only frontiers go on the frontier blacklist.  A patrol point
                # the planner could not reach says nothing about unexplored
                # space, and banning it there would distort the selector that
                # the next round of exploring depends on.
                if self.target_kind == 'frontier':
                    # Two failures blacklist the frontier; one may just have
                    # been a transient costmap obstruction.
                    if self.selector.note_failure(self.target,
                                                  now=self.clock.elapsed()):
                        note += ', blacklisted'
                else:
                    self.patrol_failures += 1
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
        # Recorded before the spin, not after: the point of the list is "the
        # camera has covered this spot", and it has by the time the turn ends
        # whether or not Nav2 reports the Spin as SUCCEEDED.  A cancelled sweep
        # still photographed most of the circle.
        here = self.pose()
        if here is not None:
            self.swept.append((here[0], here[1]))
        self.spin_in_place(angle, reason='sweeping for tags')

    def patrol_target(self, current):
        """Somewhere in known free space the camera has not looked from yet.

        Only consulted once the frontier search is exhausted.  Returns None when
        the whole known map has been swept, which is the one honest reason to
        stop exploring early.
        """
        grid = self.maps.grid
        if grid is None:
            return None
        if self.patrol_failures >= int(self.support.param('patrol_max_failures')):
            return None

        spacing = self.support.param('patrol_spacing')
        try:
            points = candidate_points(
                grid.data, grid.info.width, grid.info.height,
                grid.info.resolution,
                grid.info.origin.position.x, grid.info.origin.position.y,
                stride_m=max(0.5, spacing / 2.0),
                clearance_m=self.support.param('patrol_clearance'))
        except (IndexError, ValueError, ZeroDivisionError) as error:
            self.log.warn('patrol scan of the grid failed: {}'.format(error))
            return None

        # The same horizon the frontier search obeys: late in the run a patrol
        # point across the arena is a trip the return budget cannot afford.
        horizon = self.frontier_horizon()
        travel_weight = self.support.param('patrol_travel_weight')
        # Clamped positive: the loop below halves the spacing toward this floor,
        # so a floor of 0 - or a negative one from a typo - would keep halving
        # until the value denormalised to zero, about a thousand passes over the
        # whole candidate list, inside the control loop.
        floor = max(0.1, self.support.param('patrol_min_spacing'))

        # Relax the spacing rather than give up.  At the full spacing the answer
        # goes to None as soon as the sweeps blanket the known map - 23 sweeps
        # claiming a 2 m disc each cover 289 m2, so a 126 m2 map is "all swept"
        # long before the arena is out of tags.  A measured run stopped there
        # with 124 s of the window left and six tags unfound, which is the exact
        # outcome patrolling exists to prevent.
        #
        # A closer look is worth less than a new one, but it is worth far more
        # than parking at the start: the camera sees 55 degrees, so a second
        # visit from 1 m away still photographs a different set of walls.  Only
        # a spacing below the floor means the robot is effectively standing
        # everywhere it can already, and that is the one honest stop.
        while spacing >= floor:
            target = choose_patrol_target(
                points, self.swept, (current[0], current[1]),
                min_spacing=spacing, travel_weight=travel_weight,
                max_range=horizon)
            if target is not None:
                if spacing < self.support.param('patrol_spacing'):
                    self.log.info(
                        'known space is all swept at {:.1f} m; looking again '
                        'from {:.1f} m spacing rather than going home early'
                        .format(self.support.param('patrol_spacing'), spacing))
                return target
            spacing /= 2.0
        return None

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
        # Once patrolling has started, the frontier list being empty is the
        # established state, not news.  Going back through the 6 s confirmation
        # wait before every patrol goal would spend it over and over for an
        # answer already known.
        if self.patrolling:
            current = self.pose()
            if current is not None:
                patrol = self.patrol_target(current)
                if patrol is not None and self.dispatch(patrol, current,
                                                        kind='patrol'):
                    return
            self.patrolling = False
            self.stop_reason = 'nowhere left unswept to look from'
            self.log.info(self.stop_reason)
            self.state = State.RETURN
            return

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
            # The costmap wipe is the part that can actually change the answer.
            # The turn cannot: slam_toolbox processes no scan while the robot is
            # not translating, so a stationary rotation leaves /map byte for
            # byte identical and the detector will return exactly what it
            # returned before.  It is kept only because it is another look for
            # the camera, and a tag is worth 50 points.
            try:
                self.navigator.clearAllCostmaps()
            except Exception as error:
                self.log.debug('costmap clear failed: {}'.format(error))
            self.spin_in_place(self.support.param('scan_rotation'),
                               reason='another look while the costmaps refill')
            self.empty_since = self.clock.elapsed()
            return

        # Last resort before going home early.  If the detector is still
        # publishing frontiers and the only reason none can be chosen is that
        # they are all on the blacklist, the blacklist is the thing that is
        # wrong.  Retrying a frontier that failed twenty minutes ago against a
        # map that has since been filled in is far cheaper than ending the run
        # with the arena half explored.
        if (self.frontiers.centroids
                and self.selector.active_blacklist(now)
                and self.blacklist_clears_used
                < int(self.support.param('blacklist_clears'))):
            self.blacklist_clears_used += 1
            dropped = self.selector.clear_blacklist()
            self.log.warn(
                'every remaining frontier is blacklisted; dropping all {} bans '
                'and trying again ({}/{})'.format(
                    dropped, self.blacklist_clears_used,
                    int(self.support.param('blacklist_clears'))))
            self.recoveries_used = 0
            self.empty_since = self.clock.elapsed()
            return

        # The map is finished.  The tag hunt is not, and the two are not the
        # same job: the LiDAR sees 360 degrees so the grid fills in from any
        # heading, while the camera sees 55, so a wall can be perfectly mapped
        # and never once photographed.  Going home now would hand back the rest
        # of the window with tags still in the arena - which is exactly what the
        # reference run did, standing at the start for 84 s with six of eleven
        # tags unfound.  Keep looking until the return budget says otherwise.
        current = self.pose()
        if current is not None:
            patrol = self.patrol_target(current)
            if patrol is not None:
                self.log.info(
                    'no frontiers left, but {:.0f} s of slack: sweeping for '
                    'tags from somewhere the camera has not looked'
                    .format(self.clock.slack()))
                if self.dispatch(patrol, current, kind='patrol'):
                    self.patrolling = True
                    self.empty_since = None
                    return

        self.stop_reason = (
            'exploration complete, no frontiers left and nowhere unswept '
            'to look from ({})'.format(self.selector.last_rejection))
        self.log.info(self.stop_reason)
        self.state = State.RETURN

    def explore_step(self):
        self.update_budget()
        now = self.clock.elapsed()

        target_tags = int(self.support.param('stop_after_tags'))
        if target_tags > 0 and self.tags.count >= target_tags:
            self.stop_reason = 'found all {} tags'.format(target_tags)
            self.log.info('{}; going home with {:.0f} s of the window unused'
                          .format(self.stop_reason, self.clock.remaining()))
            if self.target is not None:
                self.navigator.cancelTask()
                self.finish_goal(False, 'cancelled, all tags found')
            self.state = State.RETURN
            return

        if self.clock.must_return():
            self.stop_reason = 'return budget spent'
            if self.target is not None:
                self.navigator.cancelTask()
                self.finish_goal(False, 'cancelled for return')
            self.state = State.RETURN
            return

        if self.target is not None:
            # Once per tick, not twice.  isTaskComplete() spins the executor for
            # up to 0.10 s, so asking it twice made the 0.4 s loop_period a 0.6 s
            # one - and every tick-counted constant in this file, consumed_checks
            # among them, is written against the configured period.
            task_complete = self.navigator.isTaskComplete()

            if not task_complete and self.target_is_consumed():
                self.navigator.cancelTask()
                self.finish_goal(True, 'frontier already explored')
                self.look_around()
                return

            if task_complete:
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

        # A frontier was found, so whatever made the list go empty earlier is
        # over.  The recovery budget is per drought, not per run: it used to be
        # spent by the first transient - a costmap wipe, a gap between map
        # publications - and then never refunded, so the genuine drought later
        # in the run got no recovery at all and the mission ended early.
        self.empty_since = None
        self.recoveries_used = 0
        # Back to real exploring.  Patrolling drives, driving translates, and
        # translation is the one thing that gets slam_toolbox to process a scan
        # - so a patrol leg genuinely can open up new frontiers.  Dropping the
        # flag puts the next drought back through the full confirmation ladder,
        # so a transient empty list cannot divert the robot onto a patrol trip
        # when waiting six seconds would have produced a real frontier.
        self.patrolling = False
        self.dispatch(target, current)

    # ------------------------------------------------------------------ #
    # RETURN
    # ------------------------------------------------------------------ #

    def go_home(self):
        tolerance = self.support.param('home_tolerance')
        retries = int(self.support.param('return_retries'))
        grace = self.support.param('return_grace')

        self.log.info('heading home: {}'.format(self.clock.summary()))

        attempts_used = 0
        dispatches = 0
        # A re-aim after a mid-drive map jump is not a failed attempt, but the
        # total is still bounded so a burst of loop closures cannot loop here
        # until the deadline.
        max_dispatches = retries + 2

        while attempts_used < retries and dispatches < max_dispatches:
            dispatches += 1
            # The watchdog has to run here too.  check_for_map_jump used to be
            # reachable only through update_budget, which only explore_step
            # calls, so it was switched off for the whole return - and the
            # return leg is precisely when a false loop closure is most likely,
            # because the robot is re-observing corridors it has already seen.
            # A jump then moved the recorded start pose, the robot drove to
            # where the start used to be, and distance_home() measured against
            # that same moved coordinate and reported a perfect return.
            self.check_for_map_jump()
            jumps_at_dispatch = len(self.map_jumps)

            # Re-read each attempt: a jump between attempts moves the target.
            home = self.home_goal()
            goal = self.pose_stamped(home[0], home[1], home[2])
            if not self.navigator.goToPose(goal):
                self.log.warn('Nav2 rejected the home goal; retrying')
                attempts_used += 1
                time.sleep(1.0)
                continue

            jumped = False
            last_watch = self.clock.elapsed()
            while not self.navigator.isTaskComplete():
                if self.clock.remaining() < -grace:
                    self.log.error('past the deadline by more than {:.0f} s, giving up '
                                   'on the drive home'.format(grace))
                    self.navigator.cancelTask()
                    break
                now = self.clock.elapsed()
                if now - last_watch >= 1.0:
                    last_watch = now
                    # One TF lookup, deliberately NOT update_budget(): that
                    # calls BasicNavigator.getPath, which spins on a future with
                    # no timeout, and a stalled planner would freeze the
                    # deadline check directly above.
                    self.check_for_map_jump()
                    if len(self.map_jumps) > jumps_at_dispatch:
                        self.log.warn(
                            'SLAM re-optimised the map mid-return; the goal no '
                            'longer denotes the start. Re-aiming at the '
                            'odometry anchor.')
                        self.navigator.cancelTask()
                        jumped = True
                        break
                time.sleep(0.2)

            if jumped:
                continue

            attempts_used += 1
            attempt = attempts_used
            self.final_pose = self.pose()
            distance = self.distance_home()
            if distance is not None and distance <= tolerance:
                self.returned_home = True
                self.final_distance = distance
                # The rule is a 50 cm circle around the start.  We aim at
                # `tolerance` (tighter) so that SLAM drift between where TF
                # believes the robot is and where it physically stands still
                # leaves the robot inside the circle that scores.
                self.log.info(
                    'home: {:.2f} m from start, {:.2f} m of margin inside the '
                    '{:.2f} m rule, with {:.0f} s to spare'.format(
                        distance, RETURN_RULE_RADIUS_M - distance,
                        RETURN_RULE_RADIUS_M, self.clock.remaining()))
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
            'return_rule_radius_m': RETURN_RULE_RADIUS_M,
            'inside_return_rule': (
                None if self.final_distance is None
                else self.final_distance <= RETURN_RULE_RADIUS_M),
            'unique_tags': len(tags),
            'tag_ids': [t['id'] for t in tags],
            'goals_sent': self.goals_sent,
            'goals_reached': self.goals_reached,
            'goals_failed': self.goals_failed,
            # How much of the run was spent hunting tags after the map was
            # finished.  A high number here with no extra tags means the arena
            # was genuinely exhausted; zero means the frontier search never ran
            # dry and the deadline was the binding constraint.
            'patrol_goals': self.patrol_goals,
            'patrol_failures': self.patrol_failures,
            'camera_sweeps': len(self.swept),
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

        # The coverage report.  Printed into the log rather than only written to
        # a file, because on the robot the log is what comes back from a run,
        # and because it answers the question every other output leaves open:
        # was a missing tag missed because the robot never went there, or
        # because it went there and never looked?
        try:
            if grid is not None:
                summary = coverage_summary(
                    grid.data, grid.info.width, grid.info.height,
                    grid.info.resolution,
                    grid.info.origin.position.x, grid.info.origin.position.y,
                    self.swept,
                    # How far the camera could actually decode, which is
                    # tag_manager's business - a different node, so it is read
                    # as an attribute rather than as a parameter this node
                    # never declared.
                    self.tags.max_detection_range,
                    [t['id'] for t in tags],
                    int(self.support.param('expected_tags')),
                    full_sweep=self.support.param('scan_rotation') >= 5.0)
                report['coverage'] = summary
                text = format_coverage(summary)
                for line in text.splitlines():
                    self.log.info(line)
                path = os.path.join(directory, 'coverage_report.txt')
                with open(path, 'w') as handle:
                    handle.write(text + '\n')
                written.append(path)
                # Rewrite the report now that it carries the coverage block.
                save_report(report, directory)
        except Exception as error:
            self.log.warn('coverage report skipped: {}'.format(error))

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

        # One unmissable line naming the absolute directory.  The individual
        # files are logged as they are written, but scattered up the log among
        # everything else, and `output_directory` is usually given as a ~ path
        # that never appears expanded anywhere.  Someone who has just watched a
        # run finish should not have to go looking for its output.
        if written:
            self.log.info('=== {} files written to: {} ==='
                          .format(len(written), os.path.abspath(directory)))
            self.log.info('===   {} ==='
                          .format('  '.join(sorted(os.path.basename(f)
                                                   for f in written))))
        else:
            self.log.error('=== NO deliverables were written - see the errors '
                           'above.  The grid and the semantic map are worth '
                           '200 points. ===')
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
