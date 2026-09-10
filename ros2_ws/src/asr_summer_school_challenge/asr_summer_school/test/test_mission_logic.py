"""Offline tests for the mission logic layer.

Everything under test here is deliberately free of rclpy, which is what lets the
exploration policy, the tag fusion, the deadline budget and the export formats
be checked on a laptop with nothing sourced and no simulator running:

    python3 -m pytest src/asr_summer_school_challenge/asr_summer_school/test -q
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from asr_summer_school import geometry as g              # noqa: E402
from asr_summer_school import map_export                 # noqa: E402
from asr_summer_school import score_report               # noqa: E402
from asr_summer_school.frontier_policy import FrontierPolicy   # noqa: E402
from asr_summer_school.mission_clock import MissionClock, path_length  # noqa: E402
from asr_summer_school.tag_map import TagMap             # noqa: E402


# --------------------------------------------------------------------------- #
# Fakes.  The message types are structural, so a namespace is enough and the
# tests do not need a ROS installation.
# --------------------------------------------------------------------------- #

class Bunch:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


def make_grid(width, height, values, resolution=0.05, origin=(0.0, 0.0)):
    return Bunch(
        info=Bunch(width=width, height=height, resolution=resolution,
                   origin=Bunch(position=Bunch(x=origin[0], y=origin[1], z=0.0),
                                orientation=Bunch(x=0.0, y=0.0, z=0.0, w=1.0))),
        data=values)


def make_path(points):
    return Bunch(poses=[Bunch(pose=Bunch(position=Bunch(x=x, y=y, z=0.0)))
                        for x, y in points])


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        clock = self

        class Stamp:
            def __init__(self, seconds):
                self.nanoseconds = seconds * 1e9

            def __sub__(self, other):
                return Bunch(nanoseconds=self.nanoseconds - other.nanoseconds)

        return Stamp(clock.t)


# --------------------------------------------------------------------------- #
# geometry
# --------------------------------------------------------------------------- #

def test_quaternion_matrix_round_trip():
    yaw = 0.83
    matrix = g.transform_matrix((1.0, -2.0, 0.3), g.yaw_quaternion(yaw))
    assert g.yaw_of(matrix) == pytest.approx(yaw)
    assert g.translation_of(matrix) == pytest.approx((1.0, -2.0, 0.3))


def test_invert_is_a_true_inverse():
    matrix = g.transform_matrix((3.0, 1.0, -0.5), g.yaw_quaternion(-1.2))
    product = g.invert(matrix) @ matrix
    for row in range(4):
        for column in range(4):
            assert product[row][column] == pytest.approx(
                1.0 if row == column else 0.0, abs=1e-9)


def test_optical_rotation_maps_camera_forward_to_body_forward():
    """Defect 2: a tag 2 m down the optical z axis is 2 m ahead of the robot."""
    corrected = g.optical_rotation() @ g.transform_matrix((0.0, 0.0, 2.0),
                                                          (0.0, 0.0, 0.0, 1.0))
    assert g.translation_of(corrected) == pytest.approx((2.0, 0.0, 0.0), abs=1e-9)


def test_optical_rotation_maps_right_and_down():
    # optical +x is to the right, which is body -y; optical +y is down, body -z.
    right = g.optical_rotation() @ g.transform_matrix((1.0, 0.0, 0.0), (0, 0, 0, 1))
    down = g.optical_rotation() @ g.transform_matrix((0.0, 1.0, 0.0), (0, 0, 0, 1))
    assert g.translation_of(right) == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)
    assert g.translation_of(down) == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)


# --------------------------------------------------------------------------- #
# tag fusion
# --------------------------------------------------------------------------- #

def test_first_observation_is_new_and_repeat_is_fused():
    tags = TagMap()
    assert tags.observe(4, 1.0, 1.0, 0.24, 0.0, range_m=1.0,
                        decision_margin=60.0) == TagMap.NEW
    assert tags.observe(4, 1.0, 1.0, 0.24, 1.0, range_m=1.0,
                        decision_margin=60.0) == TagMap.FUSED
    assert len(tags) == 1


def test_close_observations_outweigh_distant_ones():
    """1/r^2 weighting: the 1 m sighting must dominate the 3 m one."""
    tags = TagMap()
    tags.observe(1, 0.0, 0.0, 0.24, 0.0, range_m=1.0, decision_margin=60.0)
    tags.observe(1, 0.6, 0.0, 0.24, 1.0, range_m=3.0, decision_margin=60.0)
    assert tags[1].x < 0.3          # unweighted mean would be 0.30
    assert tags[1].x == pytest.approx(0.06, abs=0.01)


def test_quality_gates_discard_bad_detections():
    tags = TagMap(max_range=4.0, min_decision_margin=30.0)
    assert tags.observe(2, 0, 0, 0, 0.0, range_m=9.0,
                        decision_margin=60.0) == TagMap.DISCARDED
    assert tags.observe(2, 0, 0, 0, 0.0, range_m=1.0,
                        decision_margin=5.0) == TagMap.DISCARDED
    assert tags.observe(2, 0, 0, 0, 0.0, range_m=1.0, decision_margin=60.0,
                        hamming=2) == TagMap.DISCARDED
    assert tags.observe(2, float('nan'), 0, 0, 0.0, range_m=1.0,
                        decision_margin=60.0) == TagMap.DISCARDED
    assert len(tags) == 0


def test_outlier_is_gated_then_accepted_once_it_persists():
    tags = TagMap(gate_distance=0.75, gate_patience=3)
    tags.observe(5, 0.0, 0.0, 0.24, 0.0, range_m=1.0, decision_margin=60.0)
    assert tags.observe(5, 6.0, 6.0, 0.24, 1.0, range_m=1.0,
                        decision_margin=60.0) == TagMap.REJECTED
    assert tags[5].x == pytest.approx(0.0)
    tags.observe(5, 6.0, 6.0, 0.24, 2.0, range_m=1.0, decision_margin=60.0)
    # The third consecutive rejection concludes the stored estimate was wrong.
    assert tags.observe(5, 6.0, 6.0, 0.24, 3.0, range_m=1.0,
                        decision_margin=60.0) == TagMap.FUSED
    assert tags[5].x == pytest.approx(6.0)


def test_unique_ids_are_never_double_counted():
    tags = TagMap()
    for stamp in range(20):
        tags.observe(7, 2.0, 2.0, 0.24, float(stamp), range_m=1.5,
                     decision_margin=60.0)
    assert len(tags) == 1 and tags.ids == [7]
    assert tags[7].observations == 20


# --------------------------------------------------------------------------- #
# frontier policy
# --------------------------------------------------------------------------- #

def test_nearest_frontier_wins_when_heading_is_ignored():
    policy = FrontierPolicy()
    policy.update([(5.0, 0.0), (1.0, 0.0)])
    assert policy.select((0.0, 0.0)) == (1.0, 0.0)


def test_turn_penalty_prefers_the_candidate_ahead():
    """Two candidates the same distance away: take the one in front."""
    policy = FrontierPolicy(turn_penalty=0.6)
    policy.update([(2.0, 0.0), (-2.0, 0.0)])
    assert policy.select((0.0, 0.0), robot_yaw=0.0) == (2.0, 0.0)
    assert policy.select((0.0, 0.0), robot_yaw=math.pi) == (-2.0, 0.0)


def test_frontier_is_blacklisted_after_repeated_drive_failures():
    policy = FrontierPolicy(max_attempts=2)
    policy.update([(1.0, 0.0), (0.0, 4.0)])
    assert policy.note_failure((1.0, 0.0)) is False
    assert policy.note_failure((1.0, 0.0)) is True
    assert policy.is_blacklisted((1.0, 0.0))
    assert policy.select((0.0, 0.0), 0.0) == (0.0, 4.0)


def test_success_clears_the_failure_history():
    policy = FrontierPolicy(max_attempts=2)
    policy.update([(1.0, 0.0)])
    policy.note_failure((1.0, 0.0))
    policy.note_success((1.0, 0.0))
    assert policy.note_failure((1.0, 0.0)) is False   # counter restarted


def test_hysteresis_keeps_the_incumbent_goal():
    """A rival must be clearly better, or the robot dithers as centroids shift."""
    policy = FrontierPolicy(hysteresis=1.35, blacklist_radius=0.1)
    policy.update([(1.0, 0.0), (1.25, 0.0)])
    # The incumbent costs 1.25 against the leader's 1.00, inside the 1.35 ratio.
    assert policy.select((0.0, 0.0), 0.0, current_goal=(1.25, 0.0)) == (1.25, 0.0)


def test_a_shifted_centroid_is_still_the_same_frontier():
    """Centroids move a few cells on every map update.

    Anything within blacklist_radius of the incumbent counts as the same
    frontier, so the policy follows the centroid instead of treating the shift
    as a new candidate and re-dispatching.
    """
    policy = FrontierPolicy(blacklist_radius=0.8)
    policy.update([(1.05, 0.10)])
    assert policy.select((0.0, 0.0), 0.0, current_goal=(1.0, 0.0)) == (1.05, 0.10)


def test_hysteresis_yields_when_a_rival_is_much_better():
    policy = FrontierPolicy(hysteresis=1.2, blacklist_radius=0.3)
    policy.update([(1.0, 0.0), (8.0, 0.0)])
    assert policy.select((0.0, 0.0), 0.0, current_goal=(8.0, 0.0)) == (1.0, 0.0)


def test_a_single_planner_refusal_does_not_bury_a_frontier():
    """Early in a run "no path" means "the costmap in between is unknown".

    Blacklisting on the first refusal is how an exploration run stops after
    ninety seconds with most of the arena unvisited.
    """
    policy = FrontierPolicy(unreachable_attempts=3)
    policy.update([(1.0, 0.0), (4.0, 0.0)])
    refuse_near = lambda c: None if c == (1.0, 0.0) else 9.0   # noqa: E731

    assert policy.select((0.0, 0.0), 0.0, cost_fn=refuse_near) == (4.0, 0.0)
    assert not policy.is_blacklisted((1.0, 0.0))
    policy.select((0.0, 0.0), 0.0, cost_fn=refuse_near)
    assert not policy.is_blacklisted((1.0, 0.0))
    policy.select((0.0, 0.0), 0.0, cost_fn=refuse_near)
    assert policy.is_blacklisted((1.0, 0.0))


def test_a_drive_failure_outlasts_a_planner_refusal():
    """Both lapse, but they carry different weight: a refusal is a costmap that
    was not filled in yet, a failed approach is the robot's own evidence."""
    policy = FrontierPolicy(unreachable_attempts=1, unreachable_ttl=90.0,
                            max_attempts=1, failure_ttl=240.0)
    policy.note_unreachable((1.0, 0.0), now=0.0)
    policy.note_failure((3.0, 0.0), now=0.0)

    assert policy.is_blacklisted((1.0, 0.0), now=10.0)
    assert policy.is_blacklisted((3.0, 0.0), now=10.0)

    # The refusal has lapsed by now; the failed approach has not.
    assert not policy.is_blacklisted((1.0, 0.0), now=100.0)
    assert policy.is_blacklisted((3.0, 0.0), now=100.0)


def test_a_longer_ban_is_never_shortened_by_a_later_one():
    """Order of arrival must not decide how long a frontier stays set aside."""
    policy = FrontierPolicy(max_attempts=1, failure_ttl=240.0,
                            unreachable_attempts=1, unreachable_ttl=10.0)
    policy.note_failure((1.0, 0.0), now=0.0)
    policy.note_unreachable((1.0, 0.0), now=0.0)
    assert policy.is_blacklisted((1.0, 0.0), now=100.0)


def test_an_empty_selection_says_why():
    """"The arena is explored" and "the detector is quiet" look identical from
    a None return, and the mission ends on the difference."""
    policy = FrontierPolicy(min_distance=0.45)

    policy.update([])
    assert policy.select((0.0, 0.0), 0.0) is None
    assert 'no frontiers' in policy.last_rejection

    policy.update([(0.1, 0.0)])
    assert policy.select((0.0, 0.0), 0.0) is None
    assert 'within' in policy.last_rejection


def test_planner_costs_beat_euclidean_ranking_in_a_maze():
    """The near frontier is behind a wall; the far one is a straight run."""
    policy = FrontierPolicy()
    policy.update([(1.0, 0.0), (4.0, 0.0)])
    costs = {(1.0, 0.0): 30.0, (4.0, 0.0): 4.2}
    assert policy.select((0.0, 0.0), 0.0, cost_fn=costs.get) == (4.0, 0.0)


def test_exploration_reports_done_when_nothing_is_left():
    policy = FrontierPolicy()
    policy.update([])
    assert policy.select((0.0, 0.0), 0.0) is None


def test_frontier_underfoot_is_ignored():
    policy = FrontierPolicy(min_distance=0.45)
    policy.update([(0.1, 0.0)])
    assert policy.select((0.0, 0.0), 0.0) is None


# --------------------------------------------------------------------------- #
# mission clock
# --------------------------------------------------------------------------- #

def test_path_length_sums_segments():
    assert path_length(make_path([(0, 0), (3, 0), (3, 4)])) == pytest.approx(7.0)
    assert path_length(make_path([(0, 0)])) == 0.0
    assert path_length(None) == 0.0


def test_return_estimate_comes_from_the_planner_path():
    clock = FakeClock()
    mission = MissionClock(clock, duration=600.0, return_speed=0.15,
                           safety_factor=1.3, fixed_margin=20.0).start()
    estimate = mission.update_from_path(make_path([(0, 0), (15, 0)]))
    assert estimate == pytest.approx(15.0 / 0.15 * 1.3 + 20.0)
    assert mission.estimate_is_measured


def test_euclidean_fallback_is_inflated_by_the_detour_factor():
    mission = MissionClock(FakeClock(), duration=600.0, return_speed=0.15,
                           safety_factor=1.3, fixed_margin=20.0,
                           detour_factor=1.6).start()
    assert mission.update_from_distance(10.0) == pytest.approx(
        10.0 * 1.6 / 0.15 * 1.3 + 20.0)
    assert not mission.estimate_is_measured


def test_must_return_flips_exactly_when_the_reserve_is_spent():
    """The 180-point swing lives on this comparison."""
    clock = FakeClock()
    mission = MissionClock(clock, duration=600.0).start()
    mission.update_from_path(make_path([(0, 0), (10, 0)]))
    reserve = mission.return_estimate

    clock.t = 600.0 - reserve - 1.0
    assert not mission.must_return()
    assert mission.slack() == pytest.approx(1.0)

    clock.t = 600.0 - reserve + 0.5
    assert mission.must_return()


def test_clock_does_not_demand_a_return_before_the_mission_starts():
    assert MissionClock(FakeClock(), duration=600.0).must_return() is False


# --------------------------------------------------------------------------- #
# exports
# --------------------------------------------------------------------------- #

def test_pgm_is_valid_and_rows_are_flipped():
    # 2x2: bottom row free/occupied, top row unknown.
    grid = make_grid(2, 2, [0, 100, -1, -1])
    blob = map_export.grid_to_pgm_bytes(grid)
    header, pixels = blob.split(b'255\n', 1)
    assert header.startswith(b'P5\n')
    assert b'2 2' in header
    assert len(pixels) == 4
    # PGM starts at the top-left, the grid at the bottom-left, so the unknown
    # row must come out first.
    assert pixels[0] == 205 and pixels[1] == 205
    assert pixels[2] == 254 and pixels[3] == 0


def test_grid_yaml_matches_map_server_expectations():
    text = map_export.grid_to_yaml('map.pgm', make_grid(2, 2, [0] * 4,
                                                        origin=(-1.5, -2.5)))
    assert 'image: map.pgm' in text
    assert 'resolution: 0.050000' in text
    assert 'origin: [-1.500000, -2.500000, 0.000000]' in text
    assert 'negate: 0' in text


def test_grid_statistics_counts_cells():
    stats = map_export.grid_statistics(make_grid(2, 2, [0, 100, -1, 0]))
    assert stats['free_cells'] == 2
    assert stats['occupied_cells'] == 1
    assert stats['unknown_cells'] == 1
    assert stats['known_fraction'] == pytest.approx(0.75)


def test_semantic_map_yaml_uses_the_course_landmark_shape():
    tags = [{'id': 3, 'x': 1.0, 'y': 2.0, 'z': 0.24},
            {'id': 7, 'x': -1.0, 'y': 0.5, 'z': 0.24}]
    text = map_export.semantic_map_yaml(tags)
    assert 'id: [3, 7]' in text
    assert 'x: [1.0000, -1.0000]' in text
    import yaml
    parsed = yaml.safe_load(text)['landmarks']
    assert parsed['id'] == [3, 7] and parsed['frame_id'] == 'map'


def test_semantic_map_round_trips_through_the_scorer(tmp_path):
    tags = [{'id': 3, 'frame_id': 'map', 'x': 1.0, 'y': 2.0, 'z': 0.24,
             'observations': 4, 'best_range': 1.1}]
    map_export.save_semantic_map(tags, str(tmp_path))
    for name in ('semantic_map.yaml', 'semantic_map.json', 'semantic_map.csv'):
        assert (tmp_path / name).exists()
    assert score_report.load_estimates(str(tmp_path / 'semantic_map.json'))[3] \
        == pytest.approx((1.0, 2.0, 0.24))
    assert score_report.load_estimates(str(tmp_path / 'semantic_map.yaml'))[3] \
        == pytest.approx((1.0, 2.0, 0.24))


# --------------------------------------------------------------------------- #
# scoring
# --------------------------------------------------------------------------- #

def test_ground_truth_parses_all_eleven_tags():
    world = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'worlds', 'hard_maze_apriltag.world')
    truth = score_report.ground_truth_from_world(world)
    assert len(truth) == 11
    assert truth[0] == pytest.approx((-8.050, 9.994, 0.240))
    assert truth[10] == pytest.approx((9.700, -9.994, 0.240))


def test_baseline_run_with_no_tags_still_scores_450():
    """Map + semantic map + autonomy + on-time return, before any tag."""
    result = score_report.score({}, {0: (0.0, 0.0, 0.24)},
                                final=(0.1, 0.0), home=(0.0, 0.0),
                                seconds_late=0.0)
    assert result['total'] == 450


def test_a_minute_late_is_a_180_point_swing():
    on_time = score_report.score({}, {}, final=(0.1, 0.0), home=(0.0, 0.0),
                                 seconds_late=0.0)['total']
    late = score_report.score({}, {}, final=(0.1, 0.0), home=(0.0, 0.0),
                              seconds_late=61.0)['total']
    assert on_time - late == 180


def test_return_outside_the_radius_scores_nothing():
    result = score_report.score({}, {}, final=(0.9, 0.0), home=(0.0, 0.0))
    assert result['total'] == 300      # map + semantic map + autonomy only


def test_accuracy_band_is_capped_not_per_tag():
    truth = {i: (float(i), 0.0, 0.24) for i in range(4)}
    tight = {i: (float(i) + 0.05, 0.0, 0.24) for i in range(4)}
    loose = {i: (float(i) + 0.20, 0.0, 0.24) for i in range(4)}
    assert score_report.score(tight, truth)['breakdown'][-1] == \
        ('Localization accuracy', 30)
    assert score_report.score(loose, truth)['breakdown'][-1] == \
        ('Localization accuracy', 15)


def test_tags_not_in_the_arena_earn_nothing():
    result = score_report.score({99: (0.0, 0.0, 0.0)}, {0: (0.0, 0.0, 0.24)})
    assert result['matched'] == 0
    assert result['rows'][0]['status'] == 'not in arena'
    assert result['missed'] == [0]


# --------------------------------------------------------------------------- #
# config / code agreement
# --------------------------------------------------------------------------- #

def _package_root():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _declared_parameters(module_name):
    import re
    path = os.path.join(_package_root(), 'asr_summer_school', module_name)
    return set(re.findall(r"_?declare(?:_parameter)?\(\s*'([^']+)'", open(path).read()))


def _mission_yaml():
    import yaml
    return yaml.safe_load(
        open(os.path.join(_package_root(), 'config', 'param_mission.yaml')))


@pytest.mark.parametrize('section,module', [
    ('mission_control', 'mission_control.py'),
    ('tag_manager', 'tag_manager.py'),
])
def test_every_configured_parameter_is_declared_in_code(section, module):
    """A key no node declares is ignored in silence.

    That failure mode is invisible at runtime: the mission comes up, the setting
    simply has no effect, and the behaviour is whatever the code default was.
    """
    configured = set(_mission_yaml()[section]['ros__parameters'])
    declared = _declared_parameters(module) | {'use_sim_time'}
    assert configured - declared == set()


@pytest.mark.parametrize('section,module', [
    ('mission_control', 'mission_control.py'),
    ('tag_manager', 'tag_manager.py'),
])
def test_every_declared_parameter_is_configured(section, module):
    """The YAML is meant to be the single place these are tuned."""
    configured = set(_mission_yaml()[section]['ros__parameters'])
    assert _declared_parameters(module) - configured == set()


def test_launch_rewritten_keys_exist_in_the_yaml():
    """RewrittenYaml only substitutes keys already present in the file.

    mission.launch.py exposes these four as launch arguments; if the key is not
    in the YAML the argument is accepted on the command line and quietly does
    nothing.
    """
    document = _mission_yaml()
    flat = set()
    for section in document.values():
        flat |= set(section.get('ros__parameters', {}))
    for key in ('use_sim_time', 'mission_duration', 'output_directory',
                'optical_correction'):
        assert key in flat, key


def test_home_tolerance_leaves_margin_under_the_rule():
    """The rules say 50 cm; accepting at exactly 50 cm has no room for drift."""
    tolerance = _mission_yaml()['mission_control']['ros__parameters']['home_tolerance']
    assert 0.0 < tolerance < score_report.RETURN_RADIUS_M


# --------------------------------------------------------------------------- #
# scan preprocessing
#
# The rules encoded here were paid for with a run that drove six metres away
# from where it thought it was.  Karto reads two different point lists: the
# occupancy grid uses the range-filtered one, the scan matcher uses the
# unfiltered one.  A no-return ray has to land in the gap between them.
# --------------------------------------------------------------------------- #

from asr_summer_school.scan_preprocess import map_range          # noqa: E402


def test_a_no_return_ray_is_reported_far_away_not_just_out_of_range():
    """Karto stores an unfiltered point at the *reported* range for any ray
    outside its threshold, and the scan matcher correlates against those.  A
    value just past the threshold puts a ring of phantom obstacles around every
    scan pose; the correlation then peaks when the robot's estimated pose sits
    on an old one, and the estimate is dragged backwards."""
    horizon = 3.5
    assert map_range(float('inf'), horizon, 0.12, 20.0) == 20.0
    assert map_range(float('nan'), horizon, 0.12, 20.0) == 20.0
    assert map_range(3.5, horizon, 0.12, 20.0) == 20.0
    # Far enough to fall outside a correlation grid whose half-width is the
    # range threshold plus the search window.
    assert 20.0 > horizon * 2


def test_real_measurements_pass_through_untouched():
    for value in (0.13, 1.0, 2.5, 3.49):
        assert map_range(value, 3.5, 0.12, 20.0) == value


def test_readings_below_the_sensor_minimum_become_nan():
    """Something is there but the range is not trustworthy.  NaN is skipped by
    Karto and by Nav2; the no-return value would clear a real obstacle."""
    assert map_range(0.05, 3.5, 0.12, 20.0) != map_range(0.05, 3.5, 0.12, 20.0)


def test_a_zero_reading_is_a_no_return_not_a_near_obstacle():
    """The encoding a real LDS uses where the Gazebo model uses inf.

    Mapping 0.0 to NaN makes Karto discard every open direction, which is
    CLAUDE.md defect 1 exactly - the robot maps the wall beside it, sees
    frontiers in every direction at once and declares the arena explored
    without moving.  The simulation cannot catch this: Gazebo reports inf.
    """
    assert map_range(0.0, 3.5, 0.12, 20.0) == 20.0


def test_zero_can_be_treated_as_a_near_obstacle_if_a_driver_needs_it():
    result = map_range(0.0, 3.5, 0.12, 20.0, zero_is_no_return=False)
    assert result != result          # NaN


def test_genuinely_close_readings_are_still_nan_not_no_return():
    """Only exact zero is the no-return code; 5 cm is a real, untrusted return.

    Getting this wrong would raytrace straight through an obstacle touching the
    robot and clear it out of the costmap.
    """
    assert map_range(0.05, 3.5, 0.12, 20.0) != 20.0
    assert map_range(0.119, 3.5, 0.12, 20.0) != 20.0


def test_the_no_return_range_sits_inside_the_published_range_max():
    """`OccupancyGrid::AddScan` discards any reading >= range_max before it can
    trace anything, which is the original bug: `inf >= 3.5` meant open
    directions never became free space and the map never grew."""
    import re

    source = open(os.path.join(
        _package_root(), 'asr_summer_school', 'scan_preprocess.py')).read()
    no_return = float(re.search(
        r"declare_parameter\('no_return_range', ([\d.]+)\)", source).group(1))
    published_max = float(re.search(
        r"declare_parameter\('published_range_max', ([\d.]+)\)", source).group(1))
    assert no_return < published_max


def test_the_mapper_threshold_stays_below_the_no_return_range():
    """A no-return ray must be *above* max_laser_range so Karto traces it as
    free space instead of marking a phantom wall at its end."""
    import yaml

    with open(os.path.join(
            _package_root(), 'config', 'param_slam_toolbox.yaml')) as handle:
        params = yaml.safe_load(handle)['slam_toolbox']['ros__parameters']
    assert params['max_laser_range'] < 20.0
    assert params['scan_topic'] == '/scan_filtered'


# --------------------------------------------------------------------------- #
# Nav2 parameter coverage
#
# RewrittenYaml substitutes keys that already exist and never adds them, so a
# lifecycle node with no section in param_nav2.yaml silently keeps
# use_sim_time false under Gazebo.  For velocity_smoother, which sits in the
# cmd_vel path between the controller and the wheels, that also meant a generic
# robot's velocity limits instead of the burger's.
# --------------------------------------------------------------------------- #

# The nodes lifecycle_manager_navigation manages, from
# nav2_bringup/launch/navigation_launch.py.
NAV2_MANAGED_NODES = [
    'controller_server', 'smoother_server', 'planner_server', 'behavior_server',
    'bt_navigator', 'waypoint_follower', 'velocity_smoother',
]


def _nav2_yaml():
    import yaml
    return yaml.safe_load(
        open(os.path.join(_package_root(), 'config', 'param_nav2.yaml')))


@pytest.mark.parametrize('node', NAV2_MANAGED_NODES)
def test_every_managed_nav2_node_can_be_told_the_clock(node):
    document = _nav2_yaml()
    assert node in document, '{} has no section at all'.format(node)
    assert 'use_sim_time' in document[node]['ros__parameters']


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_costmaps_can_be_told_the_clock(costmap):
    params = _nav2_yaml()[costmap][costmap]['ros__parameters']
    assert 'use_sim_time' in params


def test_velocity_smoother_uses_the_burger_limits():
    """It clamps every cmd_vel on its way to the wheels."""
    limits = _nav2_yaml()['velocity_smoother']['ros__parameters']['max_velocity']
    assert limits[0] == pytest.approx(0.22), 'TurtleBot3 Burger tops out at 0.22 m/s'


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_inflation_leaves_a_clear_lane_in_a_one_metre_corridor(costmap):
    """Two walls a metre apart each inflating r overlap in the middle once
    2r > 1.0, leaving no zero-cost cell for the controller to steer down."""
    params = _nav2_yaml()[costmap][costmap]['ros__parameters']
    assert 2 * params['inflation_layer']['inflation_radius'] < 1.0
    assert params['robot_radius'] >= 0.105, 'the Burger is 0.105 m'


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_costmaps_read_the_same_scan_the_mapper_does(costmap):
    """The map and the costmaps disagreeing about obstacles is how a robot
    plans through a wall it has already mapped."""
    import yaml

    params = _nav2_yaml()[costmap][costmap]['ros__parameters']
    with open(os.path.join(_package_root(),
                           'config', 'param_slam_toolbox.yaml')) as handle:
        mapper_topic = yaml.safe_load(handle)['slam_toolbox']['ros__parameters']['scan_topic']
    for layer in params['plugins']:
        sources = params.get(layer, {}).get('observation_sources')
        if not sources:
            continue
        for source in sources.split():
            assert params[layer][source]['topic'] == mapper_topic


def test_a_drive_failure_ban_also_lapses():
    """A run that permanently banned its last frontier went home with the
    arena five percent explored.  Forty seconds re-approaching a frontier is
    cheaper than that by two orders of magnitude."""
    policy = FrontierPolicy(max_attempts=1, failure_ttl=240.0)
    policy.note_failure((1.0, 0.0), now=0.0)
    assert policy.is_blacklisted((1.0, 0.0), now=100.0)
    assert not policy.is_blacklisted((1.0, 0.0), now=300.0)


def test_clearing_the_blacklist_makes_everything_selectable_again():
    policy = FrontierPolicy(max_attempts=1)
    policy.update([(1.0, 0.0), (2.0, 0.0)])
    policy.note_failure((1.0, 0.0))
    policy.note_failure((2.0, 0.0))
    assert policy.select((0.0, 0.0), 0.0) is None
    assert policy.clear_blacklist() == 2
    assert policy.select((0.0, 0.0), 0.0) == (1.0, 0.0)


def test_a_frontier_is_worth_driving_to_before_nav2_would_call_it_reached():
    """Below Nav2's own goal tolerance there is nothing to drive to."""
    params = _mission_yaml()['mission_control']['ros__parameters']
    assert params['min_frontier_distance'] > 0.25


# --------------------------------------------------------------------------- #
# Node parameter hygiene
#
# rclpy raises ParameterNotDeclaredException on read, at construction time, so
# a parameter that is used but never declared kills the node the instant it
# starts.  Under a launch file that is one traceback in a log nobody is
# watching, and everything downstream then runs blind.
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize('module', [
    'mission_control.py', 'tag_manager.py', 'scan_preprocess.py',
    'frontier_client.py', 'map_recorder.py',
])
def test_every_parameter_read_is_also_declared(module):
    import re

    source = open(os.path.join(
        _package_root(), 'asr_summer_school', module)).read()
    declared = set(re.findall(r"_?declare(?:_parameter)?\(\s*'([^']+)'", source))
    # `param()` is mission_control's own accessor over get_parameter.
    read = set(re.findall(r"get_parameter\(\s*'([^']+)'", source))
    read |= set(re.findall(r"\bparam\(\s*'([^']+)'", source))
    read.discard('use_sim_time')          # declared by rclpy itself
    assert read - declared == set()


def test_the_mission_aims_inside_the_scored_return_circle():
    """The rule is a 50 cm circle.  Accepting arrival at exactly 50 cm puts the
    run on the boundary of a 150-point swing, with no room for the drift between
    where TF believes the robot is and where it physically stands."""
    tolerance = _mission_yaml()['mission_control']['ros__parameters']['home_tolerance']
    assert 0.0 < tolerance < score_report.RETURN_RADIUS_M
    # Enough margin left over to absorb the SLAM drift measured in a full run.
    assert score_report.RETURN_RADIUS_M - tolerance >= 0.10


def test_launch_exposed_numeric_parameters_accept_either_number_type():
    """rclpy refuses an int override for a double-declared parameter, and kills
    the node at construction when it gets one.  `mission_duration:=150` did
    exactly that while `150.0` worked — a distinction nobody should have to
    remember under time pressure, so these three are declared dynamically and
    coerced on the way out."""
    import re

    source = open(os.path.join(
        _package_root(), 'asr_summer_school', 'mission_control.py')).read()
    for name in ('mission_duration', 'scan_rotation', 'stop_after_tags'):
        assert re.search(
            r"_declare\(\s*'{}'\s*,[^)]*dynamic=True".format(name), source), name
    # ...and param() has to put the type back, or the arithmetic downstream
    # starts seeing ints where it expects seconds.
    assert 'self._declared_types' in source
