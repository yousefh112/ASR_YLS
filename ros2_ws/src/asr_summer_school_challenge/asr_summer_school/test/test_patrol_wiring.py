#! /usr/bin/env python3
"""The patrol state transitions inside mission_control, without a ROS graph.

patrol.py's *decision* is covered by test_patrol.py. What is covered here is the
*wiring* around it, which is where the expensive mistakes live and which a
simulation run only exercises if the frontier search happens to run dry - in a
600 s run of the 20 x 20 m maze it never does, so this path would otherwise ship
having never executed.

`MissionControl` is a ROS node and cannot be constructed here, so these tests
bind its unbound methods to a stand-in carrying the same attributes. That keeps
the real code under test: `nothing_to_explore`, `dispatch` and `finish_goal` are
imported from the module, not reimplemented.
"""

import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from asr_summer_school import mission_control as mc               # noqa: E402

State = mc.State


class FakeNavigator:
    def __init__(self, accept=True):
        self.accept = accept
        self.goals = []
        self.cancelled = 0

    def goToPose(self, goal):
        self.goals.append(goal)
        return self.accept

    def cancelTask(self):
        self.cancelled += 1

    def clearAllCostmaps(self):
        pass


class FakeSelector:
    def __init__(self):
        self.last_rejection = 'none'
        self.failures = []
        self.unreachable = []
        self.successes = []

    def note_failure(self, point, now=0.0):
        self.failures.append(point)
        return False

    def note_unreachable(self, point, now=0.0):
        self.unreachable.append(point)

    def note_success(self, point):
        self.successes.append(point)

    def active_blacklist(self, now):
        return False

    def clear_blacklist(self):
        return 0


class FakeClock:
    def __init__(self, slack=400.0):
        self._slack = slack

    def elapsed(self):
        return 100.0

    def slack(self):
        return self._slack

    def summary(self):
        return 'fake clock'


class FakeLog:
    def info(self, *a):
        pass

    warn = debug = error = info


class Stub:
    """Enough of MissionControl for the patrol path to run."""

    DEFAULTS = {
        'patrol_spacing': 2.0,
        'patrol_clearance': 0.25,
        'patrol_travel_weight': 0.35,
        'patrol_max_failures': 4,
        'patrol_min_spacing': 0.5,
        'no_frontier_timeout': 6.0,
        'recovery_spins': 0,
        'blacklist_clears': 0,
        'scan_rotation': 5.30,
        'scan_rotation_min_slack': 20.0,
        'horizon_slack_threshold': 150.0,
        'min_horizon': 1.5,
        'return_speed': 0.18,
        'goal_speed': 0.15,
        'goal_timeout_factor': 2.0,
        'goal_timeout_min': 40.0,
        'goal_timeout_max': 90.0,
        'blacklist_radius': 0.8,
        'consumed_checks': 12,
        'map_frame': 'map',
    }

    def __init__(self, grid=None, pose=(0.0, 0.0, 0.0), accept=True):
        self.support = types.SimpleNamespace(
            param=lambda k: self.DEFAULTS[k],
            transform=lambda *a, **k: None)
        self.navigator = FakeNavigator(accept=accept)
        self.selector = FakeSelector()
        self.clock = FakeClock()
        self.log = FakeLog()
        self.maps = types.SimpleNamespace(grid=grid)
        self.tags = types.SimpleNamespace(count=0)
        self.frontiers = types.SimpleNamespace(centroids=[])
        self.state = State.EXPLORE
        self.stop_reason = None
        self.target = None
        self.target_kind = 'frontier'
        self.goal_started_at = None
        self.goal_patience_s = 60.0
        self.consumed_ticks = 0
        self.empty_since = None
        self.recoveries_used = 0
        self.blacklist_clears_used = 0
        self.swept = []
        self.patrolling = False
        self.patrol_goals = 0
        self.patrol_failures = 0
        self.goals_sent = 0
        self.goals_reached = 0
        self.goals_failed = 0
        self._pose = pose

    # Real code under test, bound to the stub.
    nothing_to_explore = mc.MissionControl.nothing_to_explore
    patrol_target = mc.MissionControl.patrol_target
    dispatch = mc.MissionControl.dispatch
    finish_goal = mc.MissionControl.finish_goal
    goal_patience = mc.MissionControl.goal_patience
    frontier_horizon = mc.MissionControl.frontier_horizon
    target_is_consumed = mc.MissionControl.target_is_consumed

    def pose(self):
        return self._pose

    def pose_stamped(self, x, y, yaw):
        return (x, y, yaw)


def open_grid(width=120, height=120, res=0.05):
    """A square of known free space with a safe margin of obstacles."""
    info = types.SimpleNamespace(
        width=width, height=height, resolution=res,
        origin=types.SimpleNamespace(
            position=types.SimpleNamespace(x=-3.0, y=-3.0)))
    return types.SimpleNamespace(data=[0] * (width * height), info=info)


# ------------------------------------------------------------------ tests #

def test_an_empty_frontier_list_patrols_instead_of_going_home():
    """The whole point: a finished map must not end the run while time remains."""
    stub = Stub(grid=open_grid())
    stub.empty_since = 0.0                      # drought already confirmed
    stub.nothing_to_explore(100.0)
    assert stub.state is State.EXPLORE, 'went home with slack left'
    assert stub.patrolling
    assert stub.patrol_goals == 1
    assert stub.target_kind == 'patrol'


def test_it_goes_home_when_there_is_genuinely_nowhere_left():
    """No grid means no candidates, and then RETURN is the honest answer."""
    stub = Stub(grid=None)
    stub.empty_since = 0.0
    stub.nothing_to_explore(100.0)
    assert stub.state is State.RETURN
    assert not stub.patrolling


def test_patrol_skips_the_confirmation_wait_once_established():
    """Re-paying 6 s per patrol goal for an answer already known is waste."""
    stub = Stub(grid=open_grid())
    stub.patrolling = True
    stub.empty_since = None
    stub.nothing_to_explore(100.0)
    assert stub.patrol_goals == 1
    assert stub.state is State.EXPLORE


def test_exhausted_patrol_ends_the_mission_rather_than_looping():
    stub = Stub(grid=open_grid())
    stub.patrolling = True
    stub.swept = [(x * 0.25 - 3.0, y * 0.25 - 3.0)
                  for x in range(30) for y in range(30)]
    stub.nothing_to_explore(100.0)
    assert stub.state is State.RETURN
    assert not stub.patrolling


def test_a_rejected_patrol_goal_does_not_leave_a_phantom_target():
    """goToPose returning False must not leave self.target set with nothing driving."""
    stub = Stub(grid=open_grid(), accept=False)
    stub.empty_since = 0.0
    stub.nothing_to_explore(100.0)
    assert stub.target is None
    assert stub.state is State.RETURN


def test_a_rejected_frontier_goal_is_marked_unreachable_not_reached():
    stub = Stub(accept=False)
    assert stub.dispatch((5.0, 5.0), (0.0, 0.0), kind='frontier') is False
    assert stub.selector.unreachable == [(5.0, 5.0)]
    assert stub.target is None
    assert stub.goals_sent == 0


def test_a_patrol_failure_never_blacklists_a_frontier():
    """The frontier selector reasons about unexplored space; a patrol point is not that."""
    stub = Stub()
    stub.dispatch((4.0, 0.0), (0.0, 0.0), kind='patrol')
    stub.finish_goal(False, 'failed')
    assert stub.selector.failures == [], 'patrol point reached the frontier blacklist'
    assert stub.patrol_failures == 1
    assert stub.goals_failed == 1


def test_a_frontier_failure_still_reaches_the_blacklist():
    stub = Stub()
    stub.dispatch((4.0, 0.0), (0.0, 0.0), kind='frontier')
    stub.finish_goal(False, 'failed')
    assert stub.selector.failures == [(4.0, 0.0)]
    assert stub.patrol_failures == 0


def test_a_patrol_success_is_counted_but_not_fed_to_the_selector():
    stub = Stub()
    stub.dispatch((4.0, 0.0), (0.0, 0.0), kind='patrol')
    stub.finish_goal(True, 'reached')
    assert stub.goals_reached == 1
    assert stub.selector.successes == []


def test_a_patrol_target_is_never_judged_consumed():
    """There is no frontier at a patrol point, so the consumed test must not fire.

    Without the guard, the first tick of every patrol goal sees no centroid
    nearby, counts toward consumed_checks and cancels the goal before the robot
    has moved.
    """
    stub = Stub()
    stub.dispatch((4.0, 0.0), (0.0, 0.0), kind='patrol')
    stub.frontiers.centroids = []
    for _ in range(int(Stub.DEFAULTS['consumed_checks']) + 2):
        assert stub.target_is_consumed() is False


def test_a_frontier_target_is_still_judged_consumed():
    stub = Stub()
    stub.dispatch((4.0, 0.0), (0.0, 0.0), kind='frontier')
    stub.frontiers.centroids = []
    fired = any(stub.target_is_consumed()
                for _ in range(int(Stub.DEFAULTS['consumed_checks']) + 2))
    assert fired


def test_patrol_stops_after_too_many_failures():
    stub = Stub(grid=open_grid())
    stub.patrol_failures = int(Stub.DEFAULTS['patrol_max_failures'])
    assert stub.patrol_target((0.0, 0.0)) is None


def test_patrol_respects_the_endgame_horizon():
    """With little slack left, a patrol point across the arena must be refused."""
    stub = Stub(grid=open_grid())
    stub.clock = FakeClock(slack=20.0)       # below horizon_slack_threshold
    stub.swept = [(-3.0, -3.0)]
    target = stub.patrol_target((0.0, 0.0))
    if target is not None:
        reach = stub.frontier_horizon()
        assert (target[0] ** 2 + target[1] ** 2) ** 0.5 <= reach + 1e-6


def test_patrol_relaxes_its_spacing_instead_of_going_home_early():
    """The failure this exists to stop, taken from a real run.

    23 sweeps claiming a 2 m disc each blanket 289 m2, so a 126 m2 map counts as
    "all swept" long before the arena is out of tags. The measured run stopped
    there with 124 s of its window unused and six of eleven tags unfound. With
    relaxation the robot keeps looking from closer spacings instead.
    """
    stub = Stub(grid=open_grid())
    # Swept on a 1.5 m lattice: nothing is 2.0 m from everything, so the full
    # spacing yields nothing, but 0.5 m still does.
    stub.swept = [(x * 1.5 - 3.0, y * 1.5 - 3.0)
                  for x in range(5) for y in range(5)]
    assert stub.patrol_target((0.0, 0.0)) is not None


def test_relaxation_still_stops_when_the_robot_is_standing_everywhere():
    """Below the floor there is genuinely nothing left, and RETURN is honest."""
    stub = Stub(grid=open_grid(width=40, height=40))
    stub.swept = [(x * 0.1 - 3.0, y * 0.1 - 3.0)
                  for x in range(60) for y in range(60)]
    assert stub.patrol_target((0.0, 0.0)) is None


def test_a_zero_min_spacing_cannot_spin_the_control_loop():
    """A floor of 0 would halve toward denormal - about a thousand passes over
    every candidate, inside the 0.4 s control loop. The floor is clamped."""
    import time
    stub = Stub(grid=open_grid())
    stub.DEFAULTS = dict(Stub.DEFAULTS, patrol_min_spacing=0.0)
    stub.support.param = lambda k: stub.DEFAULTS[k]
    stub.swept = [(x * 0.1 - 3.0, y * 0.1 - 3.0)
                  for x in range(60) for y in range(60)]
    started = time.time()
    stub.patrol_target((0.0, 0.0))
    assert time.time() - started < 1.0, 'relaxation did not terminate promptly'
