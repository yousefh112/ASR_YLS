"""Choosing which frontier to drive to, and giving up on the ones that fail.

`frontier_detection_node` publishes DBSCAN centroids and nothing in the provided
repo subscribes to them.  This is the consumer side: rank the candidates, hand
one to Nav2, and remember the ones that did not work out so the robot does not
spend the mission re-attempting the same unreachable corner.

Three behaviours matter more than the ranking function itself.

*Blacklisting* stops the classic exploration deadlock where a frontier behind a
wall is always the nearest one.

*Expiry* stops blacklisting from ending the mission.  A frontier the planner
refuses at minute two is often reachable at minute six, once the corridor
leading to it has been mapped: the global costmap is mostly unknown early on and
"no path" then means "not yet", not "never".  Even a frontier the robot drove at
and failed to reach is worth another try much later, from a different approach
and against a better map - a run that banned its only remaining frontier for
good ended with the arena five percent explored, which costs far more than the
forty seconds a retry would have wasted.  So every ban expires; drive failures
simply last much longer than planner refusals.

*Hysteresis* stops the robot dithering between two similar candidates every time
the map updates and the centroids shift a few cells.

No rclpy import, so the policy can be exercised offline.  Time is passed in as
`now` rather than read from a clock, so the expiry logic is testable and follows
sim time when the mission does.
"""

import math

_FOREVER = float('inf')


def _distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class FrontierPolicy:
    """Stateful frontier picker with an expiring blacklist and goal hysteresis."""

    def __init__(self,
                 blacklist_radius=0.8,
                 min_distance=0.45,
                 max_attempts=2,
                 turn_penalty=0.6,
                 hysteresis=1.35,
                 candidates_to_plan=5,
                 unreachable_attempts=3,
                 unreachable_ttl=90.0,
                 failure_ttl=240.0):
        """
        blacklist_radius     a candidate within this of a blacklisted point is
                             treated as the same frontier
        min_distance         candidates closer than this are ignored; the robot
                             is effectively already there and Nav2 will refuse
        max_attempts         drive failures at one frontier before it is set
                             aside
        turn_penalty         how much a candidate behind the robot is penalised,
                             as a fraction of its distance at 180 degrees.  Pure
                             nearest-frontier makes a differential robot spin on
                             the spot; this biases it to keep going forwards.
        hysteresis           the incumbent goal is kept unless a rival beats it
                             by this ratio.  Centroids move whenever the map
                             updates, and re-dispatching every cycle stalls the
                             robot.
        candidates_to_plan   how many of the top candidates get a real planner
                             query.  Euclidean ranking is a poor proxy in a
                             maze, but planning is not free, so only the
                             shortlist pays.
        unreachable_attempts planner refusals at one frontier before it is set
                             aside.  One refusal is usually an unknown costmap,
                             not a wall.
        unreachable_ttl      how long a planner-refused frontier stays set
                             aside.  After this it is offered again, because the
                             map it was refused against no longer exists.
        failure_ttl          how long a frontier the robot failed to *reach*
                             stays set aside.  Much longer than a planner
                             refusal, but not forever: the alternative is a run
                             that stops because its last frontier is banned.
        """
        self.blacklist_radius = float(blacklist_radius)
        self.min_distance = float(min_distance)
        self.max_attempts = int(max_attempts)
        self.turn_penalty = float(turn_penalty)
        self.hysteresis = float(hysteresis)
        self.candidates_to_plan = int(candidates_to_plan)
        self.unreachable_attempts = int(unreachable_attempts)
        self.unreachable_ttl = float(unreachable_ttl)
        self.failure_ttl = float(failure_ttl)

        self._centroids = []
        self._blacklist = []         # [(x, y), expires_at]
        self._attempts = []          # [(x, y), drive_failures, planner_refusals]

        # Why the last select() came back empty, for the caller's log.
        self.last_rejection = 'nothing yet'

    # ------------------------------------------------------------------ #
    # Incoming frontiers
    # ------------------------------------------------------------------ #

    def update(self, centroids):
        """Replace the candidate set with the latest detector output."""
        self._centroids = [(float(x), float(y)) for x, y in centroids]

    @property
    def centroids(self):
        return list(self._centroids)

    @property
    def blacklist(self):
        """Points currently banned.  Expired entries are not included."""
        return [point for point, expiry in self._blacklist]

    def active_blacklist(self, now=0.0):
        return [point for point, expiry in self._blacklist if expiry > now]

    def is_blacklisted(self, point, now=0.0):
        return any(expiry > now and _distance(point, banned) <= self.blacklist_radius
                   for banned, expiry in self._blacklist)

    def add_to_blacklist(self, point, now=0.0, ttl=None):
        """Ban a point.  ttl=None bans it for the rest of the mission."""
        expiry = _FOREVER if ttl is None else now + float(ttl)
        for entry in self._blacklist:
            if _distance(entry[0], point) <= self.blacklist_radius:
                # Never shorten an existing ban: a permanent one outranks a
                # temporary one no matter which arrived last.
                entry[1] = max(entry[1], expiry)
                return
        self._blacklist.append([(float(point[0]), float(point[1])), expiry])

    def prune(self, now):
        """Drop expired bans.  Purely housekeeping; is_blacklisted already
        ignores them."""
        self._blacklist = [e for e in self._blacklist if e[1] > now]

    def clear_blacklist(self):
        """Forget every ban and every attempt.

        The last resort before declaring an arena explored: if the only
        candidates left are ones already given up on, trying them again is
        strictly better than going home early with the map half empty.
        Returns how many bans were dropped.
        """
        dropped = len(self._blacklist)
        self._blacklist = []
        self._attempts = []
        return dropped

    # ------------------------------------------------------------------ #
    # Outcome of an attempt
    # ------------------------------------------------------------------ #

    def _attempt_entry(self, point):
        for entry in self._attempts:
            if _distance(entry[0], point) <= self.blacklist_radius:
                return entry
        entry = [(float(point[0]), float(point[1])), 0, 0]
        self._attempts.append(entry)
        return entry

    def note_failure(self, point, now=0.0):
        """The robot drove at this frontier and did not get there.

        Returns True when this failure set the frontier aside.  The ban lasts
        far longer than a planner refusal - this is evidence from the robot
        itself, not from a costmap that had not been filled in yet - but it
        still lapses, because a mission that bans its last frontier stops
        exploring, and that costs more than one wasted approach.
        """
        entry = self._attempt_entry(point)
        entry[1] += 1
        if entry[1] >= self.max_attempts:
            self.add_to_blacklist(entry[0], now=now, ttl=self.failure_ttl)
            entry[1] = 0
            return True
        return False

    def note_unreachable(self, point, now=0.0):
        """The planner could not produce a path to this frontier.

        Sets the frontier aside temporarily once it has been refused
        `unreachable_attempts` times.  Returns True when this refusal did so.
        """
        entry = self._attempt_entry(point)
        entry[2] += 1
        if entry[2] >= self.unreachable_attempts:
            self.add_to_blacklist(entry[0], now=now, ttl=self.unreachable_ttl)
            entry[2] = 0          # start counting again after the ban lapses
            return True
        return False

    def note_success(self, point):
        """A goal was reached: forget its failure history."""
        self._attempts = [e for e in self._attempts
                          if _distance(e[0], point) > self.blacklist_radius]

    # ------------------------------------------------------------------ #
    # Selection
    # ------------------------------------------------------------------ #

    def viable(self, robot_xy, now=0.0):
        """Candidates that are neither blacklisted nor already underfoot."""
        return [c for c in self._centroids
                if not self.is_blacklisted(c, now)
                and _distance(c, robot_xy) >= self.min_distance]

    def _heuristic(self, candidate, robot_xy, robot_yaw):
        distance = _distance(candidate, robot_xy)
        if robot_yaw is None:
            return distance
        bearing = math.atan2(candidate[1] - robot_xy[1], candidate[0] - robot_xy[0])
        turn = abs(math.atan2(math.sin(bearing - robot_yaw),
                              math.cos(bearing - robot_yaw)))
        return distance * (1.0 + self.turn_penalty * turn / math.pi)

    def select(self, robot_xy, robot_yaw=None, current_goal=None, cost_fn=None,
               now=0.0):
        """Pick the frontier to drive to next.

        cost_fn, when given, is asked for the true navigation cost of a
        shortlisted candidate and returns a positive length, or None if the
        planner could not reach it.  A refusal is counted rather than acted on
        immediately: three of them set the frontier aside for a while, because
        early in a run "no path" usually means "the costmap between here and
        there is still unknown".

        Returns (x, y) or None when nothing is worth driving to right now.
        `last_rejection` then says why, which is the difference between "the
        arena is explored" and "the planner is having a bad minute".
        """
        if not self._centroids:
            self.last_rejection = 'detector published no frontiers'
            return None

        candidates = self.viable(robot_xy, now)
        if not candidates:
            near = sum(1 for c in self._centroids
                       if _distance(c, robot_xy) < self.min_distance)
            self.last_rejection = (
                '{} frontiers, all rejected ({} within {:.2f} m of the robot, '
                '{} blacklisted)'.format(
                    len(self._centroids), near,
                    self.min_distance, len(self._centroids) - near))
            return None

        candidates.sort(key=lambda c: self._heuristic(c, robot_xy, robot_yaw))

        if cost_fn is None:
            scored = [(self._heuristic(c, robot_xy, robot_yaw), c)
                      for c in candidates]
        else:
            scored = []
            refused = []
            for candidate in candidates[:self.candidates_to_plan]:
                cost = cost_fn(candidate)
                if cost is None:
                    refused.append(candidate)
                    self.note_unreachable(candidate, now=now)
                    continue
                scored.append((float(cost), candidate))
            if not scored:
                # Every shortlisted candidate was refused.  Fall back to the
                # heuristic over whatever is still viable rather than declaring
                # the exploration finished on the strength of a planner hiccup;
                # driving at it is itself a way of finding out.
                remaining = self.viable(robot_xy, now)
                if not remaining:
                    self.last_rejection = (
                        'planner refused all {} shortlisted frontiers and they '
                        'are now set aside'.format(len(refused)))
                    return None
                self.last_rejection = 'planner refused the shortlist; using the heuristic'
                scored = [(self._heuristic(c, robot_xy, robot_yaw), c)
                          for c in remaining]

        scored.sort(key=lambda pair: pair[0])
        best_cost, best = scored[0]

        if current_goal is not None and not self.is_blacklisted(current_goal, now):
            for cost, candidate in scored:
                if _distance(candidate, current_goal) <= self.blacklist_radius:
                    if cost <= best_cost * self.hysteresis:
                        self.last_rejection = ''
                        return candidate
                    break

        self.last_rejection = ''
        return best

    def summary(self, now=0.0):
        return '{} frontiers, {} blacklisted'.format(
            len(self._centroids), len(self.active_blacklist(now)))
