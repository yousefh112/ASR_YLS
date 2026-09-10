#!/usr/bin/env python3
"""Build the AprilTag mazes: the base maze worlds with AprilTags on their walls.

Two worlds come out of this, one per simulator, from the same tag placement:

    worlds/hard_maze_apriltag.world            Gazebo Classic, from hard_maze_base.world
    worlds/hard_maze_apriltag_ignition.world   Ignition/gz, from turtlebot3_ignition's
                                               worlds/hard_maze.world

Tag models follow the layout of koide3/gazebo_apriltag (https://github.com/koide3/gazebo_apriltag)
for Gazebo Classic and of rickarmstrong/gazebo_apriltag (harmonic branch,
https://github.com/rickarmstrong/gazebo_apriltag) for Ignition/gz; their textures come
from AprilRobotics/apriltag-imgs. Every model carries both flavours, so the same
models/ tree serves either simulator (see the emitters section below).

    # regenerate the tag textures too (needs opencv + a checkout of apriltag-imgs)
    git clone https://github.com/AprilRobotics/apriltag-imgs.git
    ./generate_apriltag_maze.py --apriltag-imgs apriltag-imgs

    # only re-emit the models and worlds, reusing the textures already in models/
    ./generate_apriltag_maze.py

Tags are placed only on wall faces that front open space, so a robot driving the
maze can actually see them. Every tag carries a distinct tag36h11 id.

NOTE ON SIZE: a tag36h11 image is 10x10 cells, of which the detectable
black-bordered tag is the inner 8x8. The detector's "tag size" is therefore
0.8 * the plate size set here: 0.80 m on the border walls, 0.40 m on the maze
obstacles.
"""
import argparse
import math
import os
import re
import shutil
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC_WORLD = os.path.join(PKG, "worlds", "hard_maze_base.world")
OUT_WORLD = os.path.join(PKG, "worlds", "hard_maze_apriltag.world")
# The Ignition/gz maze lives in the turtlebot3_ignition package, a sibling submodule.
IGN_SRC_WORLD = os.path.join(os.path.dirname(PKG), "turtlebot3_simulations",
                             "turtlebot3_ignition", "worlds", "hard_maze.world")
IGN_OUT_WORLD = os.path.join(PKG, "worlds", "hard_maze_apriltag_ignition.world")
MODELS = os.path.join(PKG, "models")

ARENA = 10.0          # inner face of the border walls, at +-ARENA on both axes

# Sized for turtlebot3_perception: apriltag.yaml declares one tag size (0.16 m) and
# camera.launch.py puts the RealSense at z=0.240 on base_footprint, looking level.

TAG = 0.16            # detectable tag edge, i.e. what apriltag.yaml's `size` must be
PLATE = TAG / 0.8     # 0.20 m -- the tag is the inner 8 of the image's 10 cells
TAG_Z = 0.24          # plate centre at camera height, so tags fill the frame vertically
STANDOFF = 0.006      # push the plate off the wall face to avoid z-fighting
N_TAGS = 12           # a sparse set to go and find, not a saturated arena, when
                      # models/ is empty; otherwise the shipped tags are reused
MIN_OPEN = 1.0        # only mount where the corridor in front runs at least this far
VIEW_RANGE = 3.0      # a tag needs VIEW_RANGE * PLATE of clear space in front of it,
                      # so the whole plate is framed from beyond the 0.3 m min range
SIDE_MARGIN = 0.10    # ... and that space must clear the plate edges by this much
TEXTURE_PX = 256      # must be a power of two; the source tag image is only 10x10 cells

# ---------------------------------------------------------------- geometry


def load_boxes():
    """Axis-aligned obstacle footprints as (xmin, xmax, ymin, ymax)."""
    world = ET.parse(SRC_WORLD).getroot().find("world")
    obst = next(m for m in world.findall("model") if m.get("name") == "obstacles")
    mp = [float(v) for v in obst.find("pose").text.split()]
    boxes = []
    for link in obst.findall("link"):
        lp = [float(v) for v in link.find("pose").text.split()]
        sx, sy, _ = [float(v) for v in link.find("collision/geometry/box/size").text.split()]
        cx, cy = mp[0] + lp[0], mp[1] + lp[1]
        boxes.append((cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2))
    return boxes


def free(x, y, boxes):
    if not (-ARENA < x < ARENA and -ARENA < y < ARENA):
        return False
    return not any(x0 < x < x1 and y0 < y < y1 for x0, x1, y0, y1 in boxes)


def visible(x, y, n, boxes):
    """True if the whole plate can be seen: the wedge in front of it, as wide as
    the plate and VIEW_RANGE * PLATE deep, must be clear of walls."""
    px, py = -n[1], n[0]                                   # along the wall face
    half = PLATE / 2 + SIDE_MARGIN
    depth = VIEW_RANGE * PLATE
    steps = max(3, int(depth / 0.25))
    for k in range(1, steps + 1):
        d = depth * k / steps
        for t in (-half, 0.0, half):
            if not free(x + n[0] * d + px * t, y + n[1] * d + py * t, boxes):
                return False
    return True


def open_depth(x, y, n, boxes, limit=4.0):
    """How far the corridor in front of a face runs before hitting something."""
    d = 0.0
    while d < limit:
        d += 0.1
        if not free(x + n[0] * d, y + n[1] * d, boxes):
            return d - 0.1
    return limit


def rpy_for(normal):
    """RPY that aims the plate's +Z along `normal` (a horizontal unit vector)
    with the tag image upright and unmirrored.

    The plate is textured on both faces; only the +Z one reads unmirrored.
    """
    # On the plate's +Z face the tag image runs "right" along local +Y and
    # "down" along local +X, so pointing local +X at the ground stands it upright.
    z = (normal[0], normal[1], 0.0)
    x = (0.0, 0.0, -1.0)
    y = (z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0])
    r = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]

    # Decompose as Rz(yaw) Ry(pitch) Rx(roll). Local +X points at the ground, so
    # pitch always lands on the +pi/2 singularity where roll and yaw are
    # degenerate; pin roll to zero and take yaw from the remaining terms.
    pitch = -math.asin(max(-1.0, min(1.0, r[2][0])))
    if abs(r[2][0]) > 1.0 - 1e-9:
        roll = 0.0
        yaw = -math.atan2(r[0][1], r[0][2]) * (1.0 if r[2][0] < 0 else -1.0)
    else:
        roll = math.atan2(r[2][1], r[2][2])
        yaw = math.atan2(r[1][0], r[0][0])
    return roll, pitch, yaw


def faces(boxes):
    """Every mountable wall face: the four border walls plus each obstacle side.

    Yields (normal, offset_of_face, (lo, hi) extent along the face, axis, kind).
    """
    yield ((1.0, 0.0), -ARENA, (-ARENA, ARENA), "y", "border")
    yield ((-1.0, 0.0), ARENA, (-ARENA, ARENA), "y", "border")
    yield ((0.0, -1.0), ARENA, (-ARENA, ARENA), "x", "border")
    yield ((0.0, 1.0), -ARENA, (-ARENA, ARENA), "x", "border")
    for x0, x1, y0, y1 in boxes:
        yield ((1.0, 0.0), x1, (y0, y1), "y", "obstacle")
        yield ((-1.0, 0.0), x0, (y0, y1), "y", "obstacle")
        yield ((0.0, 1.0), y1, (x0, x1), "x", "obstacle")
        yield ((0.0, -1.0), y0, (x0, x1), "x", "obstacle")


def place(boxes, n_tags=N_TAGS):
    """A small, well-spread set of tags.

    Every candidate face already fronts open space; among those we keep the ones
    with real depth in front, then pick by farthest-point sampling so the tags end
    up scattered around the maze and have to be explored for, rather than clustered.
    """
    cands = []
    for n, at, (lo, hi), axis, kind in faces(boxes):
        if hi - lo < PLATE + 0.2:
            continue
        edge = PLATE / 2 + 0.1
        t = lo + edge
        while t <= hi - edge + 1e-9:
            x, y = (at, t) if axis == "y" else (t, at)
            if visible(x, y, n, boxes):
                depth = open_depth(x, y, n, boxes)
                if depth >= MIN_OPEN:
                    cands.append((depth, x, y, n, kind))
            t += 0.25
    if not cands:
        raise SystemExit("no mountable faces found")

    chosen = [max(cands, key=lambda c: c[0])]          # seed: the most open face
    while len(chosen) < min(n_tags, len(cands)):
        best, best_d = None, -1.0
        for c in cands:
            d = min(math.hypot(c[1] - k[1], c[2] - k[2]) for k in chosen)
            if d > best_d:
                best, best_d = c, d
        chosen.append(best)

    chosen.sort(key=lambda c: (-c[2], c[1]))           # north-to-south, for readable ids
    return [(x + n[0] * STANDOFF, y + n[1] * STANDOFF, TAG_Z, n, kind)
            for _d, x, y, n, kind in chosen]


# ---------------------------------------------------------------- emitters

# Every tag ships in both flavours, so one models/ tree serves either simulator:
#
#   model.sdf      Gazebo Classic -- an Ogre material script (koide3/gazebo_apriltag)
#   model_gz.sdf   Ignition/gz    -- a PBR albedo map (rickarmstrong/gazebo_apriltag,
#                                    harmonic branch), which Ogre2 needs since it does
#                                    not read Classic's material scripts
#
# Both point at the same texture, and model.config offers each SDF version so
# sdformat hands every simulator the file it can parse: Classic (sdformat9, up to
# SDF 1.7) takes model.sdf, Ignition/gz (sdformat12+) takes model_gz.sdf.

MODEL_SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>1</static>
    <link name='main'>
      <pose>0 0 0 0 0 0</pose>
      <visual name='main_Visual'>
        <geometry>
          <box>
            <size>1.0 1.0 0.01</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>model://{name}/materials/scripts</uri>
            <uri>model://{name}/materials/textures</uri>
            <name>{name}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MODEL_SDF_GZ = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="main">
      <pose>0 0 0 0 0 0</pose>
      <visual name="main_Visual">
        <geometry>
          <box>
            <size>1.0 1.0 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <pbr>
            <metal>
              <albedo_map>materials/textures/{tag}.png</albedo_map>
              <roughness>0.5</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MODEL_CONFIG = """<?xml version="1.0" ?>
<model>
    <name>{name}</name>
    <version>1.0</version>
    <sdf version="1.6">model.sdf</sdf>
    <sdf version="1.9">model_gz.sdf</sdf>
    <author>
        <name>Kenji Koide, Rick Armstrong</name>
        <email>koide@dei.unipd.it, waitingfortheelectrician@gmail.com</email>
    </author>
    <description>AprilTag {tag} model, after koide3/gazebo_apriltag (model.sdf,
    Gazebo Classic) and rickarmstrong/gazebo_apriltag (model_gz.sdf, Ignition/gz).</description>
</model>
"""

MATERIAL = """material {name}
{{
  technique
  {{
    pass
    {{
      lighting off
      texture_unit
      {{
        texture {tag}.png
        filtering none none none
        scale 1.0 1.0
      }}
    }}
  }}
}}
"""

# The same two material flavours, for plates written inline into a world. The
# albedo map has to be a model:// URI here: a relative one would resolve against
# the world file rather than against a model directory.
MATERIAL_XML = {
    "classic": """                    <material>
                        <script>
                            <uri>model://{name}/materials/scripts</uri>
                            <uri>model://{name}/materials/textures</uri>
                            <name>{name}</name>
                        </script>
                    </material>
""",
    "ignition": """                    <material>
                        <ambient>1 1 1 1</ambient>
                        <diffuse>1 1 1 1</diffuse>
                        <specular>0.5 0.5 0.5 1</specular>
                        <pbr>
                            <metal>
                                <albedo_map>model://{name}/materials/textures/{tag}.png</albedo_map>
                                <roughness>0.5</roughness>
                                <metalness>0.0</metalness>
                            </metal>
                        </pbr>
                    </material>
""",
}


def names(i):
    """(texture/tag name, model name) of tag id `i`."""
    tag = "tag36_11_%05d" % i
    return tag, "April" + tag


def texture_path(i):
    tag, name = names(i)
    return os.path.join(MODELS, name, "materials", "textures", tag + ".png")


def installed_tags():
    """How many consecutive tag textures models/ already holds."""
    i = 0
    while os.path.isfile(texture_path(i)):
        i += 1
    return i


def generate_textures(ids, imgs_dir):
    """Redraw the tag textures from a checkout of AprilRobotics/apriltag-imgs."""
    import cv2
    for i in ids:
        tag, name = names(i)
        src = os.path.join(imgs_dir, "tag36h11", tag + ".png")
        img = cv2.imread(src, 0)
        if img is None:
            raise SystemExit("missing tag image: %s" % src)
        img = cv2.resize(img, (TEXTURE_PX, TEXTURE_PX), interpolation=cv2.INTER_NEAREST)
        shutil.rmtree(os.path.join(MODELS, name), ignore_errors=True)
        os.makedirs(os.path.dirname(texture_path(i)))
        cv2.imwrite(texture_path(i), img)
    print("wrote %d tag textures to %s" % (len(ids), MODELS))


CODEBOOK_C = os.path.join(os.path.dirname(os.path.dirname(PKG)),
                          "third_party", "apriltag", "tag36h11.c")


def generate_textures_from_codebook(ids):
    """Draw tag textures from the vendored apriltag codebook.

    The alternative is a checkout of AprilRobotics/apriltag-imgs, which needs
    network access we may not have in the lab and which is one more thing to
    have forgotten.  The codes are already in the tree, in the same library that
    does the detecting, so the textures can simply be drawn.

    A tag36h11 image is 10x10 cells: a one-cell white margin, an 8x8 black
    border (`width_at_border`), and the 36 data bits inside it at the (bit_x,
    bit_y) offsets the library lists.  Verified against the shipped textures for
    ids 0, 5 and 10 - the renderer reproduces them cell for cell.
    """
    import numpy as np
    from PIL import Image

    source = open(CODEBOOK_C).read()
    codes = [int(m, 16) for m in re.findall(r"0x([0-9a-fA-F]+)(?:UL)?L?\s*,", source)]
    bit_x = {int(i): int(v) for i, v in re.findall(r"bit_x\[(\d+)\]\s*=\s*(\d+)", source)}
    bit_y = {int(i): int(v) for i, v in re.findall(r"bit_y\[(\d+)\]\s*=\s*(\d+)", source)}
    nbits = len(bit_x)

    for i in ids:
        if i >= len(codes):
            raise SystemExit("tag36h11 has only %d ids; %d was asked for" % (len(codes), i))
        cells = np.full((10, 10), 255, dtype=np.uint8)
        cells[1:9, 1:9] = 0
        for b in range(nbits):
            bit = (codes[i] >> (nbits - 1 - b)) & 1
            cells[bit_y[b] + 1, bit_x[b] + 1] = 255 if bit else 0
        img = np.repeat(np.repeat(cells, TEXTURE_PX // 10, axis=0),
                        TEXTURE_PX // 10, axis=1)
        shutil.rmtree(os.path.join(MODELS, names(i)[1]), ignore_errors=True)
        os.makedirs(os.path.dirname(texture_path(i)), exist_ok=True)
        Image.fromarray(img).save(texture_path(i))
    print("drew %d tag textures from %s" % (len(ids), CODEBOOK_C))


def generate_models(ids):
    """Model descriptors for both simulators, around the textures already in place."""
    for i in ids:
        tag, name = names(i)
        root = os.path.join(MODELS, name)
        os.makedirs(os.path.join(root, "materials", "scripts"), exist_ok=True)
        for fname, template in (("model.sdf", MODEL_SDF),
                                ("model_gz.sdf", MODEL_SDF_GZ),
                                ("model.config", MODEL_CONFIG),
                                (os.path.join("materials", "scripts", "Apriltag.material"),
                                 MATERIAL)):
            with open(os.path.join(root, fname), "w") as f:
                f.write(template.format(name=name, tag=tag))
    print("wrote %d tag models to %s" % (len(ids), MODELS))


def tag_xml(i, spot, engine):
    """One wall-mounted plate. Written inline rather than <include>d so the plate
    carries the size this arena needs instead of the shipped model's 1 m."""
    x, y, z, n, _kind = spot
    roll, pitch, yaw = rpy_for(n)
    tag, name = names(i)
    return """        <model name='{name}'>
            <static>1</static>
            <pose>{x:.3f} {y:.3f} {z:.3f} {r:.6f} {p:.6f} {yw:.6f}</pose>
            <link name='main'>
                <visual name='main_Visual'>
                    <cast_shadows>0</cast_shadows>
                    <geometry>
                        <box>
                            <size>{s} {s} 0.005</size>
                        </box>
                    </geometry>
{material}                </visual>
            </link>
        </model>
""".format(name=name, x=x, y=y, z=z, r=roll, p=pitch, yw=yaw, s=round(PLATE, 4),
           material=MATERIAL_XML[engine].format(name=name, tag=tag))


def write_world(src, out, spots, engine):
    """Copy a base maze world, inserting the tag plates before its </world>."""
    with open(src) as f:
        base = f.read()
    tags = "".join(tag_xml(i, s, engine) for i, s in enumerate(spots))
    block = "\n        <!-- AprilTags (tag36h11), generated by scripts/generate_apriltag_maze.py -->\n" + tags
    if base.count("</world>") != 1:
        raise SystemExit("expected exactly one </world> in %s" % src)
    with open(out, "w") as f:
        f.write(base.replace("</world>", block + "    </world>"))
    print("wrote %s (%s)" % (out, engine))


def main():
    global SRC_WORLD, OUT_WORLD, ARENA
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--apriltag-imgs", metavar="DIR",
                    help="checkout of AprilRobotics/apriltag-imgs; redraws the tag textures")
    ap.add_argument("--tags", type=int, metavar="N",
                    help="how many tags to scatter through the maze; defaults to the number "
                         "already in models/ (%d, so a plain run reproduces the shipped worlds) "
                         "or to %d when there are none. Asking for more needs --apriltag-imgs, "
                         "to draw the textures the extra tags require."
                         % (installed_tags(), N_TAGS))
    ap.add_argument("--ignition-base", metavar="WORLD", default=IGN_SRC_WORLD,
                    help="turtlebot3_ignition maze to tag as well (default %(default)s); "
                         "skipped when it is missing")
    # The competition arena is a different size and shape from the practice
    # maze, so the base world, the output and the arena half-width all have to
    # move together.  See scripts/generate_competition_arena.py.
    ap.add_argument("--base", metavar="WORLD",
                    help="base world to place tags in (default %s)" % SRC_WORLD)
    ap.add_argument("--out", metavar="WORLD",
                    help="world to write (default %s)" % OUT_WORLD)
    ap.add_argument("--arena", type=float, metavar="M",
                    help="inner face of the border walls, at +-M on both axes "
                         "(default %g). Tag placement rejects anything outside "
                         "this, so it must match the base world or no tag is "
                         "placed at all." % ARENA)
    args = ap.parse_args()

    if args.base:
        SRC_WORLD = os.path.abspath(args.base)
    if args.out:
        OUT_WORLD = os.path.abspath(args.out)
    if args.arena:
        ARENA = args.arena

    boxes = load_boxes()
    spots = place(boxes, args.tags or installed_tags() or N_TAGS)
    ids = list(range(len(spots)))

    if args.apriltag_imgs:
        generate_textures(ids, args.apriltag_imgs)

    missing = [i for i in ids if not os.path.isfile(texture_path(i))]
    if missing:
        # No network, no checkout, no excuse: the codes are vendored.
        generate_textures_from_codebook(missing)
    generate_models(ids)

    write_world(SRC_WORLD, OUT_WORLD, spots, "classic")
    if args.base:
        # A custom base has no Ignition counterpart to mirror it into.
        return
    if os.path.isfile(args.ignition_base):
        write_world(args.ignition_base, IGN_OUT_WORLD, spots, "ignition")
    else:
        print("skipped the Ignition world: no %s" % args.ignition_base)

    n_border = sum(1 for s in spots if s[4] == "border")
    print("  %d tags of %.2f m (%.2f m plates) at z=%.2f -- %d on the border walls, %d on the obstacles"
          % (len(spots), TAG, PLATE, TAG_Z, n_border, len(spots) - n_border))


if __name__ == "__main__":
    main()
