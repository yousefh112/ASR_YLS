#! /usr/bin/env python3
"""Offline tests for the coverage report.

This report is the only output that distinguishes "never went there" from "went
there and never looked", and on the robot it is the only one at all - there is
no ground truth for a physical arena, so score_report cannot run. If it is wrong
it will be wrong quietly, in a direction that makes a run look better than it
was, so the occlusion and accounting rules are pinned here.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from asr_summer_school.coverage import (coverage_summary, format_coverage,
                                        grid_totals, open_frontier_cells,
                                        visible_from)

FREE, OCC, UNK = 0, 100, -1


def grid(w, h, fill=FREE):
    return [fill] * (w * h)


def put(data, w, cx, cy, v):
    data[cy * w + cx] = v


# ------------------------------------------------------------- grid totals #

def test_totals_add_up_to_the_whole_grid():
    data = grid(10, 10)
    for i in range(20):
        data[i] = OCC
    for i in range(20, 50):
        data[i] = UNK
    t = grid_totals(data, 10, 10, 0.1)
    assert abs(t['free_m2'] + t['occupied_m2'] + t['unknown_m2'] - 1.0) < 1e-9
    assert t['known_m2'] == round(70 * 0.01, 2)   # 50 free + 20 occupied
    assert t['known_fraction'] == 0.7


def test_a_fully_unknown_grid_is_zero_known():
    t = grid_totals(grid(10, 10, UNK), 10, 10, 0.05)
    assert t['known_m2'] == 0.0
    assert t['free_cells'] == 0


# ---------------------------------------------------------------- frontier #

def test_free_space_touching_unknown_is_open_frontier():
    data = grid(10, 10)
    for cy in range(10):
        put(data, 10, 9, cy, UNK)
    assert open_frontier_cells(data, 10, 10)


def test_free_space_touching_the_grid_edge_is_open_frontier():
    """What lies past the edge is unobserved by definition.

    This is the shape of CLAUDE.md defect 3: treating the boundary as closed
    made a robot in open space report the arena finished.
    """
    assert open_frontier_cells(grid(6, 6), 6, 6)


def test_a_sealed_room_has_no_open_frontier():
    """Free space fully ringed by walls: nothing left to explore."""
    data = grid(8, 8, OCC)
    for cy in range(1, 7):
        for cx in range(1, 7):
            put(data, 8, cx, cy, FREE)
    assert open_frontier_cells(data, 8, 8) == []


# -------------------------------------------------------------- visibility #

def test_an_empty_room_is_seen_from_the_middle():
    data = grid(40, 40)
    seen = visible_from(data, 40, 40, 0.1, 0.0, 0.0, [(2.0, 2.0)], 5.0)
    assert len(seen) > 1000


def test_a_wall_casts_a_shadow():
    """The whole point of raycasting rather than measuring distance."""
    w = h = 60
    data = grid(w, h)
    for cy in range(0, h):
        if 25 <= cy <= 35:
            continue
        put(data, w, 30, cy, OCC)          # wall with a gap in the middle
    seen = visible_from(data, w, h, 0.1, 0.0, 0.0, [(1.0, 1.0)], 12.0)
    # A cell well behind the solid part of the wall must not be counted.
    behind = 5 * w + 45
    assert behind not in seen


def test_the_wall_face_itself_counts_as_seen():
    w = h = 40
    data = grid(w, h)
    for cy in range(h):
        put(data, w, 20, cy, OCC)
    seen = visible_from(data, w, h, 0.1, 0.0, 0.0, [(1.0, 1.0)], 8.0)
    assert (10 * w + 20) in seen


def test_range_bounds_what_is_seen():
    data = grid(80, 80)
    near = visible_from(data, 80, 80, 0.1, 0.0, 0.0, [(4.0, 4.0)], 1.0)
    far = visible_from(data, 80, 80, 0.1, 0.0, 0.0, [(4.0, 4.0)], 4.0)
    assert len(near) < len(far)


def test_no_sweep_positions_means_nothing_seen():
    assert visible_from(grid(20, 20), 20, 20, 0.1, 0.0, 0.0, [], 5.0) == set()


def test_a_degenerate_resolution_does_not_raise():
    assert visible_from(grid(20, 20), 20, 20, 0.0, 0.0, 0.0, [(1.0, 1.0)], 5.0) == set()


# ----------------------------------------------------------------- summary #

def sealed_room(w=60, h=60):
    data = grid(w, h, OCC)
    for cy in range(1, h - 1):
        for cx in range(1, w - 1):
            put(data, w, cx, cy, FREE)
    return data


def test_a_swept_sealed_room_reports_nothing_left():
    data = sealed_room()
    s = coverage_summary(data, 60, 60, 0.1, 0.0, 0.0,
                         [(3.0, 3.0)], 12.0, [0, 1], 2)
    assert s['open_frontier_cells'] == 0
    assert s['camera_seen_fraction'] > 0.95
    assert s['tags_missing'] == []
    assert 'every expected tag was found' in format_coverage(s)


def test_an_unswept_pocket_is_found_and_located():
    """A room split by a wall, swept only on one side."""
    w = h = 60
    data = sealed_room(w, h)
    for cy in range(1, h - 1):
        if 28 <= cy <= 32:
            continue
        put(data, w, 30, cy, OCC)
    s = coverage_summary(data, w, h, 0.1, 0.0, 0.0,
                         [(1.5, 1.5)], 4.0, [], 3)
    assert s['unswept_m2'] > 1.0
    assert s['unswept_pocket_count'] >= 1
    text = format_coverage(s)
    assert 'never came into camera' in text


def test_missing_tag_ids_are_named():
    s = coverage_summary(sealed_room(), 60, 60, 0.1, 0.0, 0.0,
                         [(3.0, 3.0)], 12.0, [0, 2, 5], 6)
    assert s['tags_missing'] == [1, 3, 4]
    assert 'missing          1, 3, 4' in format_coverage(s)


def test_no_expected_count_still_reports_what_was_found():
    s = coverage_summary(sealed_room(), 60, 60, 0.1, 0.0, 0.0,
                         [(3.0, 3.0)], 12.0, [7, 9], 0)
    assert s['tags_missing'] == []
    assert 'expected count not configured' in format_coverage(s)


def test_unexplored_arena_is_called_out_separately_from_unswept():
    """The two failure modes must not be conflated - they have opposite fixes."""
    data = grid(60, 60)                      # open grid: boundary is frontier
    s = coverage_summary(data, 60, 60, 0.1, 0.0, 0.0,
                         [(3.0, 3.0)], 2.0, [], 4)
    assert s['open_frontier_cells'] > 0
    assert s['unswept_m2'] > 0
    text = format_coverage(s)
    assert 'Both causes are present' in text


def test_a_partial_sweep_is_flagged_as_optimistic():
    s = coverage_summary(sealed_room(), 60, 60, 0.1, 0.0, 0.0,
                         [(3.0, 3.0)], 12.0, [], 2, full_sweep=False)
    assert 'optimistic' in format_coverage(s)


def test_report_is_plain_ascii_so_it_survives_a_log():
    text = format_coverage(coverage_summary(
        sealed_room(), 60, 60, 0.1, 0.0, 0.0, [(3.0, 3.0)], 12.0, [1], 2))
    text.encode('ascii')          # raises if any non-ascii crept in
    assert len(text.splitlines()) > 15
