#! /usr/bin/env python3
"""The LiDAR resampler that keeps Karto from rejecting the real LDS-02's scans.

Karto fixes a sensor's reading count from the first scan it sees and rejects
every later scan whose count differs. The LDS-02 on nuc11 was measured emitting
206, 207, 208 and 209 readings per scan, with its angle metadata varying too, so
slam_toolbox discarded almost the whole stream and /map stayed empty. Gazebo's
LiDAR always emits exactly 360, so only these tests stand between that and a
simulation that looks perfect.

The metadata below is copied from four consecutive real scans on nuc11.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from asr_summer_school.scan_preprocess import resample   # noqa: E402

# (n, angle_min, angle_max, angle_increment) as measured on nuc11.
NUC11 = [
    (207, 0.02465, 6.27705, 0.0302776),
    (207, 0.00342, 6.25575, 0.0302245),
    (207, 0.01578, 6.26816, 0.0302093),
    (208, 0.00047, 6.25278, 0.0301714),
]


def karto_expected_count(angle_min, angle_max, angle_increment):
    """What Karto's LaserRangeFinder computes for a sensor's reading count."""
    return int(round((angle_max - angle_min) / angle_increment)) + 1


def grid_from_first(n, angle_min, angle_increment):
    return (n, angle_min, angle_increment)


def is_nan(v):
    return v != v


def test_the_real_metadata_is_inconsistent_which_is_the_whole_problem():
    """(max - min) / inc + 1 is not an integer, and it drifts scan to scan."""
    values = [(a1 - a0) / inc + 1 for _, a0, a1, inc in NUC11]
    assert any(abs(v - round(v)) > 0.2 for v in values)
    assert len({n for n, *_ in NUC11}) > 1


def test_every_real_scan_comes_out_the_same_length():
    n0, amin0, _, inc0 = NUC11[0]
    grid = grid_from_first(n0, amin0, inc0)
    for count in (206, 207, 208, 209):
        for _, amin, _, inc in NUC11:
            out = resample([1.0] * count, amin, inc, grid)
            assert len(out) == n0


def test_karto_expected_count_matches_on_every_published_scan():
    """The published angle_max is recomputed from the grid, so Karto agrees."""
    n0, amin0, _, inc0 = NUC11[0]
    amax_out = amin0 + (n0 - 1) * inc0
    assert karto_expected_count(amin0, amax_out, inc0) == n0


def test_resampling_loses_almost_nothing():
    """A count one or two off costs a beam at the edge, not the scan."""
    n0, amin0, _, inc0 = NUC11[0]
    grid = grid_from_first(n0, amin0, inc0)
    for count, (_, amin, _, inc) in zip((206, 207, 208, 209), NUC11):
        out = resample([2.0] * count, amin, inc, grid)
        assert sum(1 for v in out if is_nan(v)) <= 3


def test_a_reading_lands_at_its_own_bearing():
    """Nearest-neighbour by angle, not by index: a wall stays where it is."""
    n0, amin0, _, inc0 = NUC11[0]
    grid = grid_from_first(n0, amin0, inc0)
    _, amin, _, inc = NUC11[3]                   # a 208-reading scan
    values = [5.0] * 208
    marker = 100                                 # one distinctive reading
    values[marker] = 0.77
    bearing = amin + marker * inc
    out = resample(values, amin, inc, grid)
    hit = min(range(n0), key=lambda i: abs(amin0 + i * inc0 - bearing))
    assert out[hit] == 0.77


def test_identity_when_the_geometry_already_matches():
    """Gazebo's LiDAR is constant, so the simulation must be untouched."""
    n, amin, inc = 360, 0.0, 2 * math.pi / 360
    values = [0.5 + i * 0.001 for i in range(n)]
    assert resample(values, amin, inc, (n, amin, inc)) == values


def test_bearings_outside_the_input_are_nan_not_no_return():
    """A no-return value would trace free space the sensor never measured."""
    out = resample([3.0] * 10, 1.0, 0.1, (40, 0.0, 0.1))
    assert all(is_nan(v) for v in out[:9])       # 0.0 .. 0.8 rad: not covered
    assert out[10] == 3.0
    assert all(is_nan(v) for v in out[21:])      # past 1.9 rad: not covered


def test_degenerate_input_does_not_raise():
    assert all(is_nan(v) for v in resample([], 0.0, 0.03, (5, 0.0, 0.03)))
    assert all(is_nan(v) for v in resample([1.0] * 5, 0.0, 0.0, (5, 0.0, 0.03)))
    assert all(is_nan(v) for v in resample([1.0] * 5, 0.0, -0.1, (5, 0.0, 0.03)))


def test_nan_readings_survive_resampling():
    """A too-close reading is NaN on purpose, and must stay NaN."""
    values = [1.0, float('nan'), 1.0]
    out = resample(values, 0.0, 0.1, (3, 0.0, 0.1))
    assert is_nan(out[1]) and out[0] == 1.0 and out[2] == 1.0


def test_walled_arena_mode_drops_no_return_beams():
    """Final rehearsal: no-return beams through wall gaps made 95 m2 'known'
    around a ~20 m2 arena and put five of eight goals outside the walls."""
    from asr_summer_school.scan_preprocess import drop_no_return
    out = drop_no_return([1.0, 20.0, 2.5, 20.0], 20.0)
    assert out[0] == 1.0 and out[2] == 2.5          # real hits untouched
    assert is_nan(out[1]) and is_nan(out[3])       # leaks dropped, not traced
