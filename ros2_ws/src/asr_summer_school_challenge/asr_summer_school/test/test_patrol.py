#! /usr/bin/env python3
"""Offline tests for the patrol selector.

Patrol is what keeps the robot hunting once the map is finished and the clock is
not, so its failure modes are all expensive in a way that looks like success:
proposing a point inside a wall wastes a whole goal cycle, proposing one the
camera has already swept wastes the trip, and returning None too eagerly ends
the mission with tags still in the arena.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from asr_summer_school.patrol import (candidate_points, choose_patrol_target,
                                      is_clear, nearest_distance)

FREE, OCC, UNK = 0, 100, -1


def grid(width, height, fill=FREE):
    return [fill] * (width * height)


def put(data, width, cx, cy, value):
    data[cy * width + cx] = value


# ---------------------------------------------------------------- is_clear #

def test_is_clear_accepts_open_space():
    assert is_clear(grid(20, 20), 20, 20, 10, 10, 2)


def test_is_clear_rejects_a_nearby_obstacle():
    data = grid(20, 20)
    put(data, 20, 11, 10, OCC)
    assert not is_clear(data, 20, 20, 10, 10, 2)


def test_is_clear_rejects_unknown_space():
    """Unknown is not free.  Standing in it means the planner has no costmap."""
    data = grid(20, 20)
    put(data, 20, 9, 10, UNK)
    assert not is_clear(data, 20, 20, 10, 10, 2)


def test_is_clear_rejects_the_map_edge():
    """A disc that runs off the grid cannot be verified, so it is not clear."""
    assert not is_clear(grid(20, 20), 20, 20, 0, 10, 2)
    assert not is_clear(grid(20, 20), 20, 20, 19, 10, 2)


def test_is_clear_ignores_obstacles_outside_the_radius():
    """The disc is a disc, not the bounding square."""
    data = grid(40, 40)
    # Corner of the 3-cell bounding box, distance 3*sqrt(2) = 4.24 > 3.
    put(data, 40, 23, 23, OCC)
    assert is_clear(data, 40, 40, 20, 20, 3)


# -------------------------------------------------------- candidate_points #

def test_candidates_are_inside_free_space_only():
    data = grid(40, 40)
    for cy in range(40):
        for cx in range(20, 40):
            put(data, 40, cx, cy, UNK)
    points = candidate_points(data, 40, 40, 0.05, 0.0, 0.0,
                              stride_m=0.2, clearance_m=0.1)
    assert points
    # Everything at or past x = 20 cells (1.0 m) is unknown.
    assert all(x < 1.0 for x, _ in points)


def test_candidates_respect_the_origin():
    """A grid whose origin is not zero must not shift the returned points."""
    points = candidate_points(grid(40, 40), 40, 40, 0.05, -5.0, 3.0,
                              stride_m=0.5, clearance_m=0.1)
    assert points
    assert all(-5.0 <= x <= -5.0 + 40 * 0.05 for x, _ in points)
    assert all(3.0 <= y <= 3.0 + 40 * 0.05 for _, y in points)


def test_candidates_survive_a_degenerate_grid():
    """slam_toolbox regrows its grid; a zero-sized one must not raise."""
    assert candidate_points([], 0, 0, 0.05, 0.0, 0.0) == []
    assert candidate_points(grid(10, 10), 10, 10, 0.0, 0.0, 0.0) == []


# ---------------------------------------------------- choose_patrol_target #

def test_nearest_distance_of_nothing_is_infinite():
    assert nearest_distance((0.0, 0.0), []) == float('inf')


def test_first_patrol_goes_somewhere_when_nothing_is_swept():
    points = [(1.0, 0.0), (5.0, 0.0)]
    assert choose_patrol_target(points, [], (0.0, 0.0)) is not None


def test_already_swept_ground_is_refused():
    """Every candidate sits inside a swept circle, so there is nothing to do."""
    points = [(0.5, 0.0), (1.0, 0.0), (1.5, 0.0)]
    swept = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert choose_patrol_target(points, swept, (0.0, 0.0), min_spacing=2.0) is None


def test_prefers_novel_ground_over_the_nearest_point():
    """Nearest-first would pick (2.1, 0); it is barely outside a swept circle."""
    swept = [(0.0, 0.0), (2.0, 0.0)]
    points = [(2.1, 0.0), (8.0, 0.0)]
    chosen = choose_patrol_target(points, swept, (0.0, 0.0), min_spacing=2.0)
    assert chosen == (8.0, 0.0)


def test_travel_weight_breaks_ties_towards_the_cheaper_trip():
    """Two equally novel points: take the near one rather than crossing the arena."""
    points = [(6.0, 0.0), (18.0, 0.0)]
    chosen = choose_patrol_target(points, [(0.0, 0.0)], (0.0, 0.0),
                                  min_spacing=2.0, travel_weight=0.35)
    assert chosen == (6.0, 0.0)


def test_max_range_refuses_a_trip_the_budget_cannot_afford():
    points = [(18.0, 0.0)]
    assert choose_patrol_target(points, [], (0.0, 0.0), min_spacing=2.0,
                                max_range=5.0) is None


def test_max_range_none_means_unlimited():
    points = [(18.0, 0.0)]
    assert choose_patrol_target(points, [], (0.0, 0.0), min_spacing=2.0,
                                max_range=None) == (18.0, 0.0)


def test_no_candidates_is_not_an_error():
    assert choose_patrol_target([], [], (0.0, 0.0)) is None


def test_the_patrol_walks_the_arena_rather_than_oscillating():
    """Feeding each choice back as swept must keep producing fresh ground.

    The failure this guards is a selector that alternates between two points
    forever, which burns the endgame driving back and forth and photographs
    nothing new.
    """
    points = [(float(x), 0.0) for x in range(0, 21, 2)]
    swept, chosen = [(0.0, 0.0)], []
    robot = (0.0, 0.0)
    for _ in range(6):
        target = choose_patrol_target(points, swept, robot, min_spacing=2.0)
        if target is None:
            break
        assert target not in chosen, 'patrol revisited {}'.format(target)
        chosen.append(target)
        swept.append(target)
        robot = target
    assert len(chosen) >= 4


def test_a_swept_point_is_never_returned_again():
    points = [(4.0, 0.0), (9.0, 0.0)]
    first = choose_patrol_target(points, [(0.0, 0.0)], (0.0, 0.0), min_spacing=2.0)
    second = choose_patrol_target(points, [(0.0, 0.0), first], first,
                                 min_spacing=2.0)
    assert second != first


def test_novelty_reward_is_capped_so_distance_still_matters():
    """Beyond a few spacings everything is simply 'new'; then take the cheap one."""
    points = [(20.0, 0.0), (40.0, 0.0)]
    chosen = choose_patrol_target(points, [(0.0, 0.0)], (0.0, 0.0),
                                  min_spacing=2.0, travel_weight=0.35)
    assert chosen == (20.0, 0.0)


def test_diagonal_novelty_uses_euclidean_distance():
    swept = [(0.0, 0.0)]
    # 1.5 m away on the diagonal is 2.12 m, outside a 2.0 m spacing.
    assert choose_patrol_target([(1.5, 1.5)], swept, (0.0, 0.0),
                                min_spacing=2.0) == (1.5, 1.5)
    # 1.4 m on the diagonal is 1.98 m, inside it.
    assert choose_patrol_target([(1.4, 1.4)], swept, (0.0, 0.0),
                                min_spacing=2.0) is None


def test_integration_grid_to_target_never_lands_in_a_wall():
    """End to end: a room with a block in it must never be patrolled into."""
    width = height = 60
    data = grid(width, height)
    for cy in range(25, 35):
        for cx in range(25, 35):
            put(data, width, cx, cy, OCC)
    points = candidate_points(data, width, height, 0.1, 0.0, 0.0,
                              stride_m=0.4, clearance_m=0.2)
    assert points
    for _ in range(10):
        target = choose_patrol_target(points, [], (0.0, 0.0), min_spacing=0.5)
        if target is None:
            break
        cx, cy = int(target[0] / 0.1), int(target[1] / 0.1)
        assert data[cy * width + cx] != OCC
        assert not (25 <= cx < 35 and 25 <= cy < 35), 'patrolled into the block'
        points.remove(target)
