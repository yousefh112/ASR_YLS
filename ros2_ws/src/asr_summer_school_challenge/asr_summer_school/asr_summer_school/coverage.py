#! /usr/bin/env python3
"""What the run actually covered, and what it did not.

`score_report.py` answers "how well did we do" and needs the Gazebo world file
to do it, so it cannot run on the robot: there is no ground truth for a physical
arena. This answers a different and, on the day, more useful question - **where
did the robot look, and where did it not** - from the run's own output alone.
It therefore prints identically in simulation and on the robot.

The distinction that matters
----------------------------
There are two quite different ways to miss a tag, and they need opposite fixes:

  * the robot never went there.  Unexplored space, visible here as frontier
    cells still open when the run ended.  The fix is time, or exploration.
  * the robot drove past and never pointed the camera at it.  Mapped space that
    no camera sweep ever saw.  The fix is more sweeps, or better placed ones.

The second is invisible in every other output we produce. The occupancy grid
looks complete, the mission report says every goal succeeded, and the tags are
simply not there. Splitting the arena into "seen by the LiDAR" and "seen by the
camera" is the only way to tell the two apart.

How camera coverage is computed
-------------------------------
For each position the camera swept from, rays are cast outward at one-degree
steps until they hit an obstacle or reach `camera_range`. Cells along the way
are marked seen. That accounts for occlusion, which a plain distance test does
not - and occlusion is most of the story in a small arena full of blocks, where
a tag two metres away behind a pillar is as invisible as one twenty metres away.

The sweep is assumed to be a full turn, which is what `scan_rotation` does. A
partial sweep would need the yaw recorded too; if `scan_rotation` is ever set
below a full circle this becomes optimistic, and it says so in the report.

No rclpy import, for the same reason as frontier_policy and mission_clock: it
runs at a desk, in milliseconds, against a saved map.
"""

import math

UNKNOWN = -1
FREE_BELOW = 25
OCCUPIED_ABOVE = 65


def _classify(value):
    if value == UNKNOWN:
        return 'unknown'
    if value >= OCCUPIED_ABOVE:
        return 'occupied'
    if value <= FREE_BELOW:
        return 'free'
    return 'unknown'


def grid_totals(data, width, height, resolution):
    """Square metres of free, occupied and unknown inside the published grid."""
    cell = resolution * resolution
    counts = {'free': 0, 'occupied': 0, 'unknown': 0}
    for value in data:
        counts[_classify(value)] += 1
    return {
        'free_m2': round(counts['free'] * cell, 2),
        'occupied_m2': round(counts['occupied'] * cell, 2),
        'unknown_m2': round(counts['unknown'] * cell, 2),
        'known_m2': round((counts['free'] + counts['occupied']) * cell, 2),
        'known_fraction': round(
            (counts['free'] + counts['occupied']) / max(1, len(data)), 4),
        'free_cells': counts['free'],
    }


def open_frontier_cells(data, width, height):
    """Free cells that still touch unknown space, or the edge of the grid.

    Not the mission's frontier search - there is no flood fill and no
    reachability test here, because this is a report rather than a plan. It
    answers "was there anything left to explore", and it deliberately counts the
    grid boundary, since what lies past the edge is unobserved by definition.
    """
    open_cells = []
    for cy in range(height):
        for cx in range(width):
            if _classify(data[cy * width + cx]) != 'free':
                continue
            for dx, dy in ((0, 1), (1, 0), (0, -1), (-1, 0)):
                nx, ny = cx + dx, cy + dy
                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    open_cells.append((cx, cy))
                    break
                if data[ny * width + nx] == UNKNOWN:
                    open_cells.append((cx, cy))
                    break
    return open_cells


def visible_from(data, width, height, resolution, origin_x, origin_y,
                 poses, camera_range, step_deg=1.0):
    """Set of cell indices a camera sweeping a full turn at `poses` could see.

    Rays are cast at `step_deg` intervals and stop at the first occupied cell,
    so this respects walls. Unknown cells do not stop a ray - they are simply
    not counted as seen - because a ray reaching unknown space says nothing
    about what is behind it.
    """
    seen = set()
    if resolution <= 0.0:
        return seen
    max_cells = int(camera_range / resolution)
    angles = [math.radians(a * step_deg) for a in range(int(360 / step_deg))]
    for px, py in poses:
        cx0 = int((px - origin_x) / resolution)
        cy0 = int((py - origin_y) / resolution)
        for angle in angles:
            dx, dy = math.cos(angle), math.sin(angle)
            for r in range(max_cells):
                cx = int(cx0 + dx * r)
                cy = int(cy0 + dy * r)
                if cx < 0 or cx >= width or cy < 0 or cy >= height:
                    break
                index = cy * width + cx
                kind = _classify(data[index])
                if kind == 'occupied':
                    seen.add(index)      # the wall face itself is seen
                    break
                if kind == 'free':
                    seen.add(index)
    return seen


def _clusters(cells, width):
    """Connected components of a set of (cx, cy), 4-connected."""
    remaining = set(cells)
    out = []
    while remaining:
        seed = remaining.pop()
        stack, group = [seed], [seed]
        while stack:
            cx, cy = stack.pop()
            for dx, dy in ((0, 1), (1, 0), (0, -1), (-1, 0)):
                n = (cx + dx, cy + dy)
                if n in remaining:
                    remaining.remove(n)
                    stack.append(n)
                    group.append(n)
        out.append(group)
    return out


def coverage_summary(data, width, height, resolution, origin_x, origin_y,
                     swept, camera_range, found_ids, expected_tags,
                     full_sweep=True):
    """Everything the report needs, as plain numbers."""
    totals = grid_totals(data, width, height, resolution)
    cell = resolution * resolution

    frontier = open_frontier_cells(data, width, height)
    seen = visible_from(data, width, height, resolution, origin_x, origin_y,
                        swept, camera_range)

    unseen = []
    for cy in range(height):
        for cx in range(width):
            index = cy * width + cx
            if _classify(data[index]) == 'free' and index not in seen:
                unseen.append((cx, cy))

    # Every connected pocket, then split into the ones worth driving to and the
    # slivers.  Both halves are reported: quoting a total of 2.2 m2 and then
    # listing a single 0.3 m2 pocket, as an earlier version did, reads as an
    # arithmetic error and makes the whole report untrustworthy.
    WORTH_VISITING_M2 = 0.25       # smaller than the robot's own footprint
    all_pockets = []
    for group in _clusters(unseen, width):
        area = len(group) * cell
        mx = sum(c[0] for c in group) / len(group)
        my = sum(c[1] for c in group) / len(group)
        all_pockets.append({
            'area_m2': round(area, 2),
            'centre': (round(origin_x + mx * resolution, 2),
                       round(origin_y + my * resolution, 2)),
        })
    all_pockets.sort(key=lambda p: -p['area_m2'])
    pockets = [p for p in all_pockets if p['area_m2'] >= WORTH_VISITING_M2]
    pockets_m2 = round(sum(p['area_m2'] for p in pockets), 2)
    # Derived as the remainder rather than summed independently: hundreds of
    # slivers each rounded to 2 dp drift by a square metre or more against the
    # total, and three numbers that do not add up discredit the report.
    slivers_m2 = round(round(len(unseen) * cell, 2) - pockets_m2, 2)

    seen_free = sum(1 for i in seen if _classify(data[i]) == 'free')
    expected = list(range(expected_tags)) if expected_tags else []
    missing = [t for t in expected if t not in found_ids]

    return {
        'grid': totals,
        'extent_m': {
            'x': [round(origin_x, 2), round(origin_x + width * resolution, 2)],
            'y': [round(origin_y, 2), round(origin_y + height * resolution, 2)],
        },
        'sweep_positions': len(swept),
        'camera_range_m': camera_range,
        'full_sweep_assumed': full_sweep,
        'camera_seen_m2': round(seen_free * cell, 2),
        'camera_seen_fraction': round(seen_free / max(1, totals['free_cells']), 4),
        'unswept_m2': round(len(unseen) * cell, 2),
        'unswept_pockets': pockets[:5],
        'unswept_pocket_count': len(pockets),
        'unswept_pockets_m2': pockets_m2,
        'unswept_slivers_m2': slivers_m2,
        'open_frontier_cells': len(frontier),
        'open_frontier_m2': round(len(frontier) * cell, 2),
        'tags_found': sorted(found_ids),
        'tags_expected': expected_tags,
        'tags_missing': missing,
    }


def format_coverage(s):
    """The block printed into the mission log.  Plain text, no colour, no unicode."""
    g = s['grid']
    out = []
    add = out.append
    add('=' * 68)
    add('COVERAGE REPORT   -   what this run looked at, and what it did not')
    add('=' * 68)
    add('')
    add('MAPPED BY THE LIDAR')
    add('  known            {:8.2f} m2   (free {:.2f}, walls {:.2f})'
        .format(g['known_m2'], g['free_m2'], g['occupied_m2']))
    add('  unknown in grid  {:8.2f} m2   ({:.0f}% of the published grid is known)'
        .format(g['unknown_m2'], 100 * g['known_fraction']))
    add('  grid extent      x[{:.2f}, {:.2f}]  y[{:.2f}, {:.2f}]'
        .format(s['extent_m']['x'][0], s['extent_m']['x'][1],
                s['extent_m']['y'][0], s['extent_m']['y'][1]))
    add('')
    add('LEFT UNEXPLORED')
    if s['open_frontier_cells'] == 0:
        add('  none - no free cell still touched unknown space.  Everything')
        add('  reachable was mapped, so a missing tag was not missed for want')
        add('  of exploring.')
    else:
        add('  {:.2f} m2 of frontier was still open when the run ended.'
            .format(s['open_frontier_m2']))
        add('  The arena was NOT fully explored: there was somewhere left to go')
        add('  and the clock, not the map, ended the run.')
    add('')
    add('SEEN BY THE CAMERA')
    add('  sweep positions  {}'.format(s['sweep_positions']))
    add('  seen             {:8.2f} m2   ({:.0f}% of mapped free space, within '
        '{:.1f} m and in line of sight)'
        .format(s['camera_seen_m2'], 100 * s['camera_seen_fraction'],
                s['camera_range_m']))
    add('  never seen       {:8.2f} m2   total'.format(s['unswept_m2']))
    add('     of which      {:8.2f} m2   in {} pocket(s) big enough to drive to:'
        .format(s['unswept_pockets_m2'], s['unswept_pocket_count']))
    for p in s['unswept_pockets'][:5]:
        add('       {:6.2f} m2 around ({:.2f}, {:.2f})'
            .format(p['area_m2'], p['centre'][0], p['centre'][1]))
    if len(s['unswept_pockets']) > 5:
        add('       ... and {} more'.format(len(s['unswept_pockets']) - 5))
    if s['unswept_slivers_m2'] > 0:
        add('     and           {:8.2f} m2   in slivers smaller than the robot,'
            .format(s['unswept_slivers_m2']))
        add('                              mostly grazing angles along walls')
    if not s['full_sweep_assumed']:
        add('  NOTE: scan_rotation is less than a full turn, so the figures')
        add('        above are optimistic - they assume each stop looked all')
        add('        the way round.')
    add('')
    add('TAGS')
    if s['tags_expected']:
        add('  found            {} of {}'
            .format(len(s['tags_found']), s['tags_expected']))
        add('  ids              {}'.format(
            ', '.join(str(t) for t in s['tags_found']) or 'none'))
        add('  missing          {}'.format(
            ', '.join(str(t) for t in s['tags_missing']) or 'none'))
    else:
        add('  found            {}  (expected count not configured)'
            .format(len(s['tags_found'])))
        add('  ids              {}'.format(
            ', '.join(str(t) for t in s['tags_found']) or 'none'))
    add('')
    add('WHAT TO DO ABOUT IT')
    if s['tags_expected'] and not s['tags_missing']:
        add('  Nothing: every expected tag was found.')
    elif s['open_frontier_cells'] > 0 and s['unswept_m2'] > 1.0:
        add('  Both causes are present: {:.1f} m2 was never explored AND'
            .format(s['open_frontier_m2']))
        add('  {:.1f} m2 of what WAS mapped never came into camera view.'
            .format(s['unswept_m2']))
        add('  More time would help; so would more sweep positions.')
    elif s['open_frontier_cells'] > 0:
        add('  The arena was not fully explored - the run ran out of clock.')
        add('  A missing tag is most likely in the unexplored part, so the')
        add('  lever is exploration speed, not camera coverage.')
    elif s['unswept_m2'] > 1.0:
        add('  The map was finished but {:.1f} m2 of it never came into camera'
            .format(s['unswept_m2']))
        add('  view. A missing tag is most likely in a pocket listed above, so')
        add('  the lever is more or better placed sweeps - lower patrol_spacing')
        add('  - not more exploring.')
    else:
        add('  The arena was fully explored and essentially all of it was seen')
        add('  by the camera. A tag still missing was in view and not decoded:')
        add('  look at detection range, motion blur, or the tag facing away.')
    add('=' * 68)
    return '\n'.join(out)
