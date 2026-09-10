#! /usr/bin/env python3
"""Where to look next once the frontier search has run dry.

The problem this exists to solve
--------------------------------
Frontier exploration finishes when there is no boundary left between known free
space and unknown space.  That means the *map* is complete.  It does not mean
the *tags* have all been found, and the two are scored very differently: the
occupancy grid is worth 100 points once, while every unique tag is worth 50.

The gap between them is the camera.  The LiDAR is a 360 degree sensor, so a
single stop maps every direction at once and the grid fills in fast.  The RGB
camera sees about 55 degrees, so a tag is only detected if the robot happened to
be pointing at it.  Driving past a wall maps that wall completely and may
photograph none of it.  A run can therefore reach "exploration complete" with
the whole arena mapped and half the tags never once inside the frame.

The reference 600 s run ended with the arena 68% known, five of eleven tags
found, and 84 s of its window unused - it went home and stood there.  Those are
300 points of tags left in a room the robot had time to re-enter.

So when frontiers run out and the clock has not, the robot keeps working: it
drives to somewhere in known free space that it has not yet *looked around
from*, and sweeps the camera there.  Coverage of positions is not coverage of
viewing directions, and this closes the difference.

Why this file has no rclpy import
---------------------------------
Same reason as frontier_policy and mission_clock: it is the part that can be
tested at a desk, in milliseconds, without a graph.  The caller passes plain
numbers out of the OccupancyGrid message and gets a plain (x, y) back.
"""

import math

# OccupancyGrid conventions, from nav_msgs/OccupancyGrid.msg: -1 unknown,
# 0 free, 100 occupied, with everything in between a probability.
UNKNOWN = -1
FREE_BELOW = 25          # <= this and known is "free enough to stand in"
OCCUPIED_ABOVE = 65      # >= this is an obstacle


def _index(width, cx, cy):
    return cy * width + cx


def is_clear(data, width, height, cx, cy, radius_cells):
    """True when no cell within `radius_cells` of (cx, cy) is an obstacle.

    A patrol point has to be somewhere the robot can physically stand and, more
    to the point, somewhere Nav2's inflation layer will not have priced out of
    reach.  Checking a disc rather than the single cell is what stops the
    selector proposing the middle of a doorway or a spot flush against a wall,
    which the planner then refuses - and a refused goal costs a whole cycle.
    """
    for dy in range(-radius_cells, radius_cells + 1):
        ny = cy + dy
        if ny < 0 or ny >= height:
            return False
        for dx in range(-radius_cells, radius_cells + 1):
            nx = cx + dx
            if nx < 0 or nx >= width:
                return False
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            value = data[_index(width, nx, ny)]
            if value == UNKNOWN or value >= OCCUPIED_ABOVE:
                return False
    return True


def candidate_points(data, width, height, resolution, origin_x, origin_y,
                     stride_m=1.0, clearance_m=0.25):
    """Coarse lattice of standable points in known free space.

    Sampling on a lattice rather than testing every cell keeps this cheap: a
    20 x 20 m arena at 5 cm is 160 000 cells, but at a 1 m stride it is 400
    candidates, and a patrol point does not need centimetre placement.
    """
    if resolution <= 0.0 or width <= 0 or height <= 0:
        return []

    stride = max(1, int(round(stride_m / resolution)))
    clearance = max(1, int(round(clearance_m / resolution)))
    points = []
    for cy in range(clearance, height - clearance, stride):
        for cx in range(clearance, width - clearance, stride):
            value = data[_index(width, cx, cy)]
            if value == UNKNOWN or value > FREE_BELOW:
                continue
            if not is_clear(data, width, height, cx, cy, clearance):
                continue
            points.append((origin_x + (cx + 0.5) * resolution,
                           origin_y + (cy + 0.5) * resolution))
    return points


def nearest_distance(point, others):
    """Distance from `point` to the closest of `others`, or inf if empty."""
    best = float('inf')
    for other in others:
        d = math.hypot(point[0] - other[0], point[1] - other[1])
        if d < best:
            best = d
    return best


def choose_patrol_target(points, swept, robot, min_spacing=2.0,
                         travel_weight=0.35, max_range=None):
    """Pick the most valuable place to go and look around from.

    `points`  candidate standable positions, from `candidate_points`
    `swept`   positions the camera has already swept from
    `robot`   where the robot is now
    `min_spacing`
              a candidate closer than this to somewhere already swept is not
              worth the trip - the camera has covered that ground
    `travel_weight`
              metres of detour the robot will accept per metre of extra
              novelty.  Purely nearest-first re-sweeps the same corner
              repeatedly; purely farthest-first sends the robot across the
              arena and back for one look.
    `max_range`
              refuse anything further than this, so the endgame does not start
              a trip the return budget cannot afford

    Returns (x, y) or None when nothing is worth visiting.
    """
    if min_spacing <= 0.0:
        return None
    best, best_score = None, None
    for point in points:
        novelty = nearest_distance(point, swept)
        if novelty < min_spacing:
            continue
        travel = math.hypot(point[0] - robot[0], point[1] - robot[1])
        if max_range is not None and travel > max_range:
            continue
        # Unvisited arena is unbounded in novelty terms once it is far from
        # everything swept, so cap the reward: beyond a couple of spacings a
        # candidate is simply "somewhere new" and the tie-break should be
        # whichever is cheapest to reach.
        score = min(novelty, 3.0 * min_spacing) - travel_weight * travel
        if best_score is None or score > best_score:
            best, best_score = point, score
    return best
