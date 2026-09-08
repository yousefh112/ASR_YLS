#! /usr/bin/env python3
"""Score a finished run offline, against the arena's own ground truth.

The repo ships the arena as a Gazebo world with all eleven tags at known
coordinates and a `/ground_truth` pose topic, which means the stack can be
scored before Friday instead of guessed at.  This reads the artefacts a run
leaves behind and prints what the rubric would award.

Ground truth is parsed out of `worlds/hard_maze_apriltag.world` rather than
copied into a table here, so it cannot drift away from the world the robot is
actually driving in.  The physical arena will differ: this validates the
pipeline, it does not calibrate it.

Deliberately free of any ROS import, so it runs on a laptop with nothing
sourced:

    ./score_report.py --run ~/asr_mission/20260908-1530
    ./score_report.py --semantic-map semantic_map.json --final 0.12,-0.08
"""

import argparse
import json
import math
import os
import re
import sys

# Rubric, from the challenge handout.
POINTS_PER_TAG = 50
POINTS_MAP = 100
POINTS_SEMANTIC_MAP = 100
POINTS_EXPLORATION = 100
POINTS_RETURN_ON_TIME = 150
ACCURACY_TIGHT_M = 0.15
ACCURACY_TIGHT_POINTS = 30
ACCURACY_LOOSE_M = 0.30
ACCURACY_LOOSE_POINTS = 15
RETURN_RADIUS_M = 0.50

# Late-return schedule, (seconds past the deadline, points).
LATE_SCHEDULE = ((0.0, 150), (30.0, 0), (60.0, -30), (120.0, -70))

_TAG_MODEL = re.compile(
    r"<model\s+name=['\"]Apriltag36_11_(\d+)['\"]\s*>(.*?)</model>", re.S)
_POSE = re.compile(r"<pose>\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)")


def ground_truth_from_world(path):
    """{tag_id: (x, y, z)} parsed from the Gazebo world."""
    with open(path) as handle:
        text = handle.read()
    truth = {}
    for match in _TAG_MODEL.finditer(text):
        pose = _POSE.search(match.group(2))
        if pose:
            truth[int(match.group(1))] = tuple(float(v) for v in pose.groups())
    return truth


def ground_truth_from_yaml(path):
    """{tag_id: (x, y, z)} from a semantic-map YAML in the landmarks.yaml shape."""
    import yaml
    document = yaml.safe_load(open(path))
    landmarks = document.get('landmarks', document)
    ids = landmarks['id']
    return {int(i): (float(x), float(y), float(z))
            for i, x, y, z in zip(ids, landmarks['x'], landmarks['y'],
                                  landmarks.get('z', [0.0] * len(ids)))}


def _pose_xy(pose):
    """(x, y) from the {x, y, yaw} dicts the mission report writes."""
    if not pose:
        return None
    return (float(pose['x']), float(pose['y']))


def load_estimates(path):
    """{tag_id: (x, y, z)} from a semantic map written by the mission."""
    if path.endswith(('.yaml', '.yml')):
        return ground_truth_from_yaml(path)
    document = json.load(open(path))
    landmarks = document.get('landmarks', document)
    return {int(t['id']): (float(t['x']), float(t['y']), float(t.get('z', 0.0)))
            for t in landmarks}


# --------------------------------------------------------------------------- #
# Scoring
# --------------------------------------------------------------------------- #

def compare_tags(estimates, truth, planar=True):
    """Per-tag errors plus the false positives and the misses."""
    rows = []
    for tag_id in sorted(estimates):
        if tag_id not in truth:
            rows.append({'id': tag_id, 'error': None, 'status': 'not in arena'})
            continue
        estimate, actual = estimates[tag_id], truth[tag_id]
        if planar:
            error = math.hypot(estimate[0] - actual[0], estimate[1] - actual[1])
        else:
            error = math.dist(estimate, actual)
        rows.append({'id': tag_id, 'error': error, 'status': 'ok',
                     'estimate': estimate, 'truth': actual})
    missed = sorted(set(truth) - set(estimates))
    return rows, missed


def accuracy_points(errors):
    """The capped accuracy award, from the mean error over matched tags."""
    if not errors:
        return 0, None
    mean = sum(errors) / len(errors)
    if mean < ACCURACY_TIGHT_M:
        return ACCURACY_TIGHT_POINTS, mean
    if mean < ACCURACY_LOOSE_M:
        return ACCURACY_LOOSE_POINTS, mean
    return 0, mean


def return_points(distance, seconds_late):
    """Points for the return, or 0 if it never got inside the radius."""
    if distance is None or distance > RETURN_RADIUS_M:
        return 0
    if seconds_late is None:
        seconds_late = 0.0
    awarded = LATE_SCHEDULE[0][1]
    for threshold, points in LATE_SCHEDULE:
        if seconds_late > threshold:
            awarded = points
    return awarded


def score(estimates, truth, final=None, home=None, seconds_late=0.0,
          have_map=True, have_semantic_map=True, autonomous=True,
          collisions=0, interventions=0, planar=True, measured_distance=None):
    rows, missed = compare_tags(estimates, truth, planar)
    valid = [r for r in rows if r['status'] == 'ok']
    errors = [r['error'] for r in valid]

    accuracy, mean_error = accuracy_points(errors)

    distance = measured_distance
    if distance is None and final is not None and home is not None:
        distance = math.hypot(final[0] - home[0], final[1] - home[1])

    breakdown = [
        ('2D occupancy grid map', POINTS_MAP if have_map else 0),
        ('Semantic map', POINTS_SEMANTIC_MAP if have_semantic_map else 0),
        ('Autonomous exploration', POINTS_EXPLORATION if autonomous else 0),
        ('Return to start', return_points(distance, seconds_late)),
        ('Unique tags ({} x {})'.format(len(valid), POINTS_PER_TAG),
         len(valid) * POINTS_PER_TAG),
        ('Localization accuracy', accuracy),
    ]
    if collisions:
        breakdown.append(('Collisions ({})'.format(collisions), -20 * collisions))
    if interventions:
        breakdown.append(
            ('Manual interventions ({})'.format(interventions), -50 * interventions))

    return {
        'rows': rows,
        'missed': missed,
        'matched': len(valid),
        'mean_error': mean_error,
        'max_error': max(errors) if errors else None,
        'return_distance': distance,
        'breakdown': breakdown,
        'total': sum(points for _, points in breakdown),
    }


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #

def render(result, truth_count):
    out = []
    out.append('Tags')
    out.append('  {:<5} {:>9} {:>9} {:>9} {:>9} {:>9}'.format(
        'id', 'est x', 'est y', 'true x', 'true y', 'error'))
    for row in result['rows']:
        if row['status'] != 'ok':
            out.append('  {:<5} {:>49}'.format(row['id'], '! ' + row['status']))
            continue
        out.append('  {:<5} {:>9.3f} {:>9.3f} {:>9.3f} {:>9.3f} {:>8.3f}m'.format(
            row['id'], row['estimate'][0], row['estimate'][1],
            row['truth'][0], row['truth'][1], row['error']))

    out.append('')
    out.append('  matched {}/{}'.format(result['matched'], truth_count))
    if result['missed']:
        out.append('  missed  {}'.format(
            ', '.join(str(i) for i in result['missed'])))
    if result['mean_error'] is not None:
        out.append('  error   mean {:.3f} m, worst {:.3f} m'.format(
            result['mean_error'], result['max_error']))
    if result['return_distance'] is not None:
        verdict = 'inside' if result['return_distance'] <= RETURN_RADIUS_M else 'OUTSIDE'
        out.append('  return  {:.3f} m from start ({} the {:.2f} m radius)'.format(
            result['return_distance'], verdict, RETURN_RADIUS_M))

    out.append('')
    out.append('Score')
    for label, points in result['breakdown']:
        out.append('  {:<34} {:>+6d}'.format(label, points))
    out.append('  {:<34} {:>6d}'.format('TOTAL', result['total']))
    return '\n'.join(out)


def _find_world(name='hard_maze_apriltag.world'):
    """Locate the arena world without importing anything from ROS.

    The script is installed to `lib/asr_summer_school/` while the worlds land
    in `share/asr_summer_school/worlds/`, so walking up from __file__ alone
    finds nothing once the package is installed.  Both layouts are tried, plus
    the ament index, so the same script works from the source tree and from an
    install.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        # source tree: <pkg>/asr_summer_school/score_report.py
        os.path.join(os.path.dirname(here), 'worlds', name),
        # install tree: <prefix>/lib/asr_summer_school/score_report.py
        os.path.join(os.path.dirname(os.path.dirname(here)),
                     'share', 'asr_summer_school', 'worlds', name),
    ]
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            candidates.append(os.path.join(
                prefix, 'share', 'asr_summer_school', 'worlds', name))
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[0]


def _pair(text):
    parts = text.replace(',', ' ').split()
    return (float(parts[0]), float(parts[1]))


def main(argv=None):
    default_world = _find_world()

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--run', help='mission output directory')
    parser.add_argument('--semantic-map', help='semantic_map.json or .yaml')
    parser.add_argument('--world', default=default_world,
                        help='Gazebo world to read ground truth from')
    parser.add_argument('--truth-yaml', help='ground truth in landmarks.yaml shape')
    parser.add_argument('--home', type=_pair, help='start pose "x,y"')
    parser.add_argument('--final', type=_pair, help='final pose "x,y"')
    parser.add_argument('--late', type=float, default=0.0,
                        help='seconds past the deadline')
    parser.add_argument('--collisions', type=int, default=0)
    parser.add_argument('--interventions', type=int, default=0)
    parser.add_argument('--3d', dest='three_d', action='store_true',
                        help='include z in the error instead of planar distance')
    parser.add_argument('--json', action='store_true', help='machine-readable output')
    args = parser.parse_args(argv)

    semantic = args.semantic_map
    home, final, late = args.home, args.final, args.late
    have_map = True
    measured_distance = None

    if args.run:
        run = os.path.expanduser(args.run)
        if semantic is None:
            for candidate in ('semantic_map.json', 'semantic_map.yaml'):
                if os.path.exists(os.path.join(run, candidate)):
                    semantic = os.path.join(run, candidate)
                    break
        have_map = any(os.path.exists(os.path.join(run, n))
                       for n in ('map.pgm', 'map.yaml'))
        report_path = os.path.join(run, 'mission_report.json')
        if os.path.exists(report_path):
            report = json.load(open(report_path))
            home = home or _pose_xy(report.get('home_pose'))
            final = final or _pose_xy(report.get('final_pose'))
            if not late:
                late = max(0.0, float(report.get('seconds_late') or 0.0))
            # The mission measured this against the same TF the robot drove on;
            # prefer it to a distance recomputed from two rounded poses.
            if report.get('final_distance_from_home_m') is not None:
                measured_distance = float(report['final_distance_from_home_m'])

    if semantic is None or not os.path.exists(semantic):
        parser.error('no semantic map found; pass --semantic-map or --run')

    truth = (ground_truth_from_yaml(args.truth_yaml) if args.truth_yaml
             else ground_truth_from_world(args.world))
    if not truth:
        parser.error('no ground truth tags found in {}'.format(args.world))

    result = score(load_estimates(semantic), truth,
                   final=final, home=home, seconds_late=late,
                   have_map=have_map, have_semantic_map=True,
                   collisions=args.collisions, interventions=args.interventions,
                   planar=not args.three_d,
                   measured_distance=measured_distance)

    if args.json:
        printable = dict(result)
        printable['rows'] = [
            {k: v for k, v in row.items() if k != 'estimate'} for row in result['rows']]
        print(json.dumps(printable, indent=2, default=list))
    else:
        print(render(result, len(truth)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
