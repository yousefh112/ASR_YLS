"""Mission deadline bookkeeping and return-home budgeting.

The scoring table makes the deadline the single most valuable thing to get
right: returning on time is +150, returning one minute late is -30.  That
180-point swing is worth 3.6 tags, so exploration is abandoned as soon as the
time left drops to the estimated cost of driving home.

The estimate is derived from the length of a path the Nav2 planner actually
produced, not from straight-line distance, so walls between the robot and the
start are accounted for.  A euclidean fallback with a detour factor covers the
case where the planner cannot answer.
"""

import math


def path_length(path):
    """Sum of segment lengths of a nav_msgs/Path, in meters."""
    if path is None or len(path.poses) < 2:
        return 0.0
    total = 0.0
    for previous, current in zip(path.poses, path.poses[1:]):
        total += math.hypot(
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y)
    return total


class MissionClock:
    """Tracks the mission deadline and decides when to abandon exploration.

    All times are seconds on the clock passed in, so the caller decides whether
    the mission runs on sim time or wall time by which node's clock it hands
    over.
    """

    def __init__(self, clock, duration,
                 return_speed=0.15,
                 safety_factor=1.3,
                 fixed_margin=20.0,
                 detour_factor=1.6):
        """
        clock          rclpy Clock the whole mission is timed against
        duration       mission length in seconds, from start() to deadline
        return_speed   effective drive speed home, m/s.  Well below the burger's
                       0.22 m/s top speed because the trip includes turning,
                       replanning and the occasional recovery.
        safety_factor  multiplier on the raw drive time
        fixed_margin   constant reserve for cancelling the current goal, the
                       final approach and the map export
        detour_factor  straight-line distance is multiplied by this when the
                       planner could not supply a path
        """
        self._clock = clock
        self.duration = float(duration)
        self.return_speed = float(return_speed)
        self.safety_factor = float(safety_factor)
        self.fixed_margin = float(fixed_margin)
        self.detour_factor = float(detour_factor)

        self._start = None
        self._return_estimate = float(fixed_margin)
        self._estimate_is_measured = False

    # ------------------------------------------------------------------ #
    # Clock
    # ------------------------------------------------------------------ #

    def start(self):
        """Mark the mission start.  Everything else is relative to this."""
        self._start = self._clock.now()
        return self

    @property
    def started(self):
        return self._start is not None

    def elapsed(self):
        """Seconds since start(), or 0.0 before the mission starts."""
        if self._start is None:
            return 0.0
        return (self._clock.now() - self._start).nanoseconds / 1e9

    def remaining(self):
        """Seconds left before the deadline.  Goes negative once blown."""
        return self.duration - self.elapsed()

    # ------------------------------------------------------------------ #
    # Return budget
    # ------------------------------------------------------------------ #

    @property
    def return_estimate(self):
        """Current estimate of the seconds needed to get home."""
        return self._return_estimate

    @property
    def estimate_is_measured(self):
        """True when the estimate came from a planner path, not the fallback."""
        return self._estimate_is_measured

    def update_from_path(self, path):
        """Re-estimate the return cost from a nav_msgs/Path to the start pose.

        Returns the new estimate, or None if the path was unusable so the
        caller knows to fall back.
        """
        if path is None or len(path.poses) < 2:
            return None
        length = path_length(path)
        self._return_estimate = self._cost(length)
        self._estimate_is_measured = True
        return self._return_estimate

    def update_from_distance(self, distance):
        """Re-estimate from a straight-line distance, inflated by the detour factor."""
        self._return_estimate = self._cost(distance * self.detour_factor)
        self._estimate_is_measured = False
        return self._return_estimate

    def _cost(self, length):
        if self.return_speed <= 0.0:
            return self.duration
        return length / self.return_speed * self.safety_factor + self.fixed_margin

    # ------------------------------------------------------------------ #
    # The decision
    # ------------------------------------------------------------------ #

    def must_return(self):
        """True once the time left has shrunk to the estimated cost of going home."""
        if self._start is None:
            return False
        return self.remaining() <= self._return_estimate

    def slack(self):
        """Seconds of exploration left before the return has to start."""
        return self.remaining() - self._return_estimate

    def summary(self):
        source = 'planner' if self._estimate_is_measured else 'estimate'
        return ('t+{:.0f}s  left {:.0f}s  return {:.0f}s ({})  slack {:.0f}s'
                .format(self.elapsed(), self.remaining(),
                        self._return_estimate, source, self.slack()))
