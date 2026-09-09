# Vendored packages

These three directories were git submodules of this repository. They are now
checked in as plain sources, for the same reason `src/third_party/` is: a single
`git clone` of `ASR_YLS` has to produce a workspace that builds.

As submodules they did not. The parent repository recorded them by commit hash
only, with no working `.gitmodules` mapping at the level that mattered, so a
teammate cloning the project got empty directories and a `git submodule status`
that failed outright — and a `--recurse-submodules` anyone forgets on
competition day is a failure mode this project does not need.

| Directory | Upstream | Pinned at |
|---|---|---|
| `laser_filters` | https://github.com/ros-perception/laser_filters.git | `959c848` (2.0.10) |
| `turtlebot3_perception` | https://github.com/SESASR-Course/turtlebot3_perception.git | `c665ca9` (branch `asr`) |
| `turtlebot3_simulations` | https://github.com/SESASR-Course/turtlebot3_simulations.git | `d672661` (2.2.5-36-gd672661) |

**Nothing in them is modified, and nothing in them should be.** They are here to
be built, not edited; anything we write goes in `asr_summer_school/`. That rule
used to be enforced by the fact that edits were lost on the next `git pull`.
Now it is only a convention, so it is written down here.

To take an upstream update, re-clone the directory at the new commit and commit
the result on its own, so the diff is reviewable:

```bash
cd /tmp && git clone https://github.com/ros-perception/laser_filters.git
cd laser_filters && git checkout <new commit> && rm -rf .git
rsync -a --delete /tmp/laser_filters/ ~/ASR_YLS/ros2_ws/src/asr_summer_school_challenge/laser_filters/
```

`laser_filters` is no longer in the runtime pipeline — `scan_preprocess.py`
replaced it — but it stays because the workspace still builds it and the course
material refers to it.
