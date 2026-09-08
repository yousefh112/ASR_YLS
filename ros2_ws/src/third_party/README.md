# Vendored third-party packages

`apriltag`, `apriltag_msgs` and `apriltag_ros` are checked in here as plain
sources rather than pulled from apt, so the workspace builds and detects tags on
a machine where `ros-humble-apriltag-ros` is not installed and `sudo` is not
available. On the robot PC the system packages exist and are equivalent; this
overlay simply takes precedence.

| Package | Upstream | Version |
|---|---|---|
| `apriltag` | https://github.com/AprilRobotics/apriltag | 3.4.5 |
| `apriltag_msgs` | https://github.com/christianrauch/apriltag_msgs | 2.0.2 |
| `apriltag_ros` | https://github.com/christianrauch/apriltag_ros | 3.4.0 |

Nothing in here is modified. To refresh, delete a directory and re-clone it;
the `.git` directories were stripped so that this workspace is a single
self-contained checkout.
