"""Writing the two deliverables to disk.

The occupancy grid and the semantic map are +100 each, together worth more than
four tags, and both are lost if the export step fails at the end of a run.  So
the grid is written here from the raw `nav_msgs/OccupancyGrid` in the same
PGM + YAML format `nav2_map_server` produces, instead of depending on a service
call to a node that has to still be alive and responsive at minute twenty.
`slam_toolbox`'s own saver is still called when it is available, but only as a
bonus on top of a file that is already on disk.

The semantic map format is not specified in any of the provided material, so
every export writes the same content four ways and lets the organisers take
whichever they want:

  semantic_map.yaml   the id/x/y/z list shape the course's own
                      turtlebot3_perception/config/landmarks.yaml already uses
  semantic_map.json   the same tags plus per-tag provenance and mission metadata
  semantic_map.csv    one row per tag, for a spreadsheet
  semantic_map.png    the occupancy grid with the tags drawn on it

Only the PNG needs Pillow, and it is skipped rather than fatal if that import
fails.
"""

import csv
import io
import json
import math
import os

# map_server's trinary defaults; matching them keeps our PGM interchangeable
# with one written by `ros2 run nav2_map_server map_saver_cli`.
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.25

_FREE_PIXEL = 254
_UNKNOWN_PIXEL = 205
_OCCUPIED_PIXEL = 0


def ensure_directory(path):
    if path and not os.path.isdir(path):
        os.makedirs(path, exist_ok=True)
    return path


# --------------------------------------------------------------------------- #
# Occupancy grid
# --------------------------------------------------------------------------- #

def grid_to_pgm_bytes(grid,
                      occupied_thresh=OCCUPIED_THRESH,
                      free_thresh=FREE_THRESH):
    """Serialise a nav_msgs/OccupancyGrid to binary PGM (P5) bytes.

    The grid's first cell is the bottom-left corner of the map while PGM starts
    at the top-left, so the rows come out reversed.
    """
    width = int(grid.info.width)
    height = int(grid.info.height)
    data = grid.data

    occupied_cut = occupied_thresh * 100.0
    free_cut = free_thresh * 100.0

    lut = bytearray(202)
    for value in range(-1, 101):
        if value < 0:
            pixel = _UNKNOWN_PIXEL
        elif value >= occupied_cut:
            pixel = _OCCUPIED_PIXEL
        elif value <= free_cut:
            pixel = _FREE_PIXEL
        else:
            pixel = _UNKNOWN_PIXEL
        lut[value + 1] = pixel

    out = io.BytesIO()
    out.write(b'P5\n')
    out.write(b'# CREATOR: asr_summer_school map_export\n')
    out.write('{} {}\n255\n'.format(width, height).encode('ascii'))

    for row in range(height - 1, -1, -1):
        start = row * width
        chunk = data[start:start + width]
        out.write(bytes(lut[min(max(int(v), -1), 100) + 1] for v in chunk))

    return out.getvalue()


def grid_to_yaml(image_name, grid,
                 occupied_thresh=OCCUPIED_THRESH,
                 free_thresh=FREE_THRESH):
    """The map_server sidecar YAML for a grid written by grid_to_pgm_bytes."""
    origin = grid.info.origin
    yaw = math.atan2(
        2.0 * (origin.orientation.w * origin.orientation.z
               + origin.orientation.x * origin.orientation.y),
        1.0 - 2.0 * (origin.orientation.y ** 2 + origin.orientation.z ** 2))
    return (
        'image: {}\n'
        'mode: trinary\n'
        'resolution: {:.6f}\n'
        'origin: [{:.6f}, {:.6f}, {:.6f}]\n'
        'negate: 0\n'
        'occupied_thresh: {}\n'
        'free_thresh: {}\n'
    ).format(image_name, grid.info.resolution,
             origin.position.x, origin.position.y, yaw,
             occupied_thresh, free_thresh)


def save_occupancy_grid(grid, directory, stem='map',
                        occupied_thresh=OCCUPIED_THRESH,
                        free_thresh=FREE_THRESH):
    """Write <stem>.pgm and <stem>.yaml; returns both paths."""
    ensure_directory(directory)
    image_name = stem + '.pgm'
    pgm_path = os.path.join(directory, image_name)
    yaml_path = os.path.join(directory, stem + '.yaml')

    with open(pgm_path, 'wb') as handle:
        handle.write(grid_to_pgm_bytes(grid, occupied_thresh, free_thresh))
    with open(yaml_path, 'w') as handle:
        handle.write(grid_to_yaml(image_name, grid, occupied_thresh, free_thresh))

    return pgm_path, yaml_path


def grid_statistics(grid):
    """Cell counts and explored area, for the run report."""
    free = occupied = unknown = 0
    for value in grid.data:
        if value < 0:
            unknown += 1
        elif value >= OCCUPIED_THRESH * 100:
            occupied += 1
        else:
            free += 1
    cell_area = float(grid.info.resolution) ** 2
    total = free + occupied + unknown
    return {
        'width': int(grid.info.width),
        'height': int(grid.info.height),
        'resolution': float(grid.info.resolution),
        'free_cells': free,
        'occupied_cells': occupied,
        'unknown_cells': unknown,
        'explored_area_m2': round((free + occupied) * cell_area, 2),
        'known_fraction': round((free + occupied) / total, 4) if total else 0.0,
    }


# --------------------------------------------------------------------------- #
# Semantic map
# --------------------------------------------------------------------------- #

def semantic_map_yaml(tags, family='tag36h11'):
    """The id/x/y/z list shape used by the course's own landmarks.yaml."""
    lines = ['# Semantic map: AprilTag landmarks in the map frame.',
             '# Written by asr_summer_school, format mirrors',
             '# turtlebot3_perception/config/landmarks.yaml',
             'landmarks:',
             '  frame_id: map',
             "  family: '{}'".format(family)]
    for key, attribute in (('id', 'id'), ('x', 'x'), ('y', 'y'), ('z', 'z')):
        if attribute == 'id':
            values = ', '.join(str(t['id']) for t in tags)
        else:
            values = ', '.join('{:.4f}'.format(t[attribute]) for t in tags)
        lines.append('  {}: [{}]'.format(key, values))
    return '\n'.join(lines) + '\n'


def semantic_map_json(tags, metadata=None, family='tag36h11'):
    document = {
        'frame_id': 'map',
        'family': family,
        'tag_count': len(tags),
        'landmarks': tags,
    }
    if metadata:
        document['mission'] = metadata
    return json.dumps(document, indent=2, sort_keys=False) + '\n'


def semantic_map_csv(tags):
    fields = ['id', 'frame_id', 'x', 'y', 'z', 'observations', 'best_range']
    out = io.StringIO()
    writer = csv.DictWriter(out, fieldnames=fields, extrasaction='ignore')
    writer.writeheader()
    for tag in tags:
        writer.writerow(tag)
    return out.getvalue()


def save_semantic_map(tags, directory, stem='semantic_map',
                      metadata=None, family='tag36h11'):
    """Write the YAML, JSON and CSV renderings; returns the paths written."""
    ensure_directory(directory)
    written = []
    for suffix, content in (
            ('.yaml', semantic_map_yaml(tags, family)),
            ('.json', semantic_map_json(tags, metadata, family)),
            ('.csv', semantic_map_csv(tags))):
        path = os.path.join(directory, stem + suffix)
        with open(path, 'w') as handle:
            handle.write(content)
        written.append(path)
    return written


# --------------------------------------------------------------------------- #
# Overlay
# --------------------------------------------------------------------------- #

def _world_to_pixel(x, y, info):
    column = (x - info.origin.position.x) / info.resolution
    row = info.height - 1 - (y - info.origin.position.y) / info.resolution
    return int(round(column)), int(round(row))


def save_overlay(grid, tags, path, home=None, final=None, scale=3):
    """Draw the tags on the occupancy grid as a human-checkable PNG.

    Returns the path written, or None when Pillow is unavailable.
    """
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return None

    ensure_directory(os.path.dirname(path))

    width, height = int(grid.info.width), int(grid.info.height)
    grey = Image.frombytes(
        'L', (width, height),
        grid_to_pgm_bytes(grid).split(b'255\n', 1)[1])
    image = grey.convert('RGB').resize(
        (width * scale, height * scale), Image.NEAREST)
    draw = ImageDraw.Draw(image)

    def marker(point, colour, label=None, radius=5, label_dy=0):
        column, row = _world_to_pixel(point[0], point[1], grid.info)
        column, row = column * scale, row * scale
        draw.ellipse([column - radius, row - radius,
                      column + radius, row + radius],
                     fill=colour, outline=(0, 0, 0))
        if label is not None:
            draw.text((column + radius + 2, row - radius - 2 + label_dy),
                      label, fill=colour)

    # A good run ends where it started, which puts these two markers on top of
    # each other; the labels are offset in opposite directions so they stay
    # readable in exactly the case worth looking at.
    if home is not None:
        marker(home, (0, 140, 255), 'start', radius=6, label_dy=-9)
    if final is not None:
        marker(final, (0, 190, 90), 'end', radius=6, label_dy=9)
    for tag in tags:
        marker((tag['x'], tag['y']), (220, 30, 30), str(tag['id']))

    image.save(path)
    return path


# --------------------------------------------------------------------------- #
# Run report
# --------------------------------------------------------------------------- #

def save_report(report, directory, stem='mission_report'):
    """Write the machine-readable run summary next to the deliverables."""
    ensure_directory(directory)
    path = os.path.join(directory, stem + '.json')
    with open(path, 'w') as handle:
        handle.write(json.dumps(report, indent=2) + '\n')
    return path
