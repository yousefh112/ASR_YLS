"""Unique-tag bookkeeping: deduplicate by ID and fuse repeat observations.

Scoring pays +50 for every *unique* ID, and a single capped +30 for accuracy, so
the job here is overwhelmingly "never lose an ID, never invent one" and only
secondarily "place it well".  The fusion is therefore a weighted running mean
with an outlier gate rather than anything that needs a covariance.

Observations are weighted by 1/range^2.  A tag seen from 0.8 m is far better
localised than the same tag seen from 3 m: the pixel error is the same, the
metric error is not.  The gate then protects the estimate from the one failure
mode that actually costs points, a bad detection dragging a good tag off its
true position.

Nothing here imports rclpy, so it is unit testable offline.
"""

import math


class TagEstimate:
    """Running estimate of one tag's position in the map frame."""

    def __init__(self, tag_id, x, y, z, weight, stamp, range_m):
        self.id = int(tag_id)
        self._wsum = float(weight)
        self._wx = float(x) * weight
        self._wy = float(y) * weight
        self._wz = float(z) * weight

        self.observations = 1
        self.rejected = 0
        self._consecutive_rejections = 0

        self.first_seen = stamp
        self.last_seen = stamp
        self.best_range = float(range_m)
        self.max_deviation = 0.0

    # ------------------------------------------------------------------ #
    # Position
    # ------------------------------------------------------------------ #

    @property
    def x(self):
        return self._wx / self._wsum

    @property
    def y(self):
        return self._wy / self._wsum

    @property
    def z(self):
        return self._wz / self._wsum

    @property
    def position(self):
        return (self.x, self.y, self.z)

    def distance_to(self, x, y, z=None):
        """Distance from the current estimate; planar unless z is given."""
        if z is None:
            return math.hypot(self.x - x, self.y - y)
        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2 + (self.z - z) ** 2)

    # ------------------------------------------------------------------ #
    # Fusion
    # ------------------------------------------------------------------ #

    def fuse(self, x, y, z, weight, stamp, range_m):
        deviation = self.distance_to(x, y, z)
        self._wsum += weight
        self._wx += x * weight
        self._wy += y * weight
        self._wz += z * weight
        self.observations += 1
        self.last_seen = stamp
        self.best_range = min(self.best_range, float(range_m))
        self.max_deviation = max(self.max_deviation, deviation)
        self._consecutive_rejections = 0

    def reject(self):
        self.rejected += 1
        self._consecutive_rejections += 1
        return self._consecutive_rejections

    def reset_to(self, x, y, z, weight, stamp, range_m):
        """Throw the history away and restart from this observation.

        Used when the gate has rejected so many observations in a row that the
        stored estimate, not the incoming ones, is the thing more likely wrong.
        """
        self._wsum = float(weight)
        self._wx = float(x) * weight
        self._wy = float(y) * weight
        self._wz = float(z) * weight
        self.observations = 1
        self.last_seen = stamp
        self.best_range = float(range_m)
        self.max_deviation = 0.0
        self._consecutive_rejections = 0

    # ------------------------------------------------------------------ #
    # Export
    # ------------------------------------------------------------------ #

    def to_dict(self):
        return {
            'id': self.id,
            'frame_id': 'map',
            'x': round(self.x, 4),
            'y': round(self.y, 4),
            'z': round(self.z, 4),
            'observations': self.observations,
            'rejected': self.rejected,
            'best_range': round(self.best_range, 3),
            'max_deviation': round(self.max_deviation, 4),
            'first_seen': round(self.first_seen, 2),
            'last_seen': round(self.last_seen, 2),
        }


class TagMap:
    """Every unique tag seen so far, keyed by ID."""

    NEW = 'new'
    FUSED = 'fused'
    REJECTED = 'rejected'
    DISCARDED = 'discarded'

    def __init__(self,
                 max_range=4.0,
                 min_range=0.25,
                 min_decision_margin=30.0,
                 max_hamming=0,
                 gate_distance=0.75,
                 gate_patience=5):
        """
        max_range            observations further away than this are dropped.
                             The sim LiDAR stops at 3.5 m and tag pose error
                             grows with the square of range, so far detections
                             cost accuracy without adding IDs.
        min_range            floor used when weighting, keeps 1/r^2 finite
        min_decision_margin  apriltag's own confidence score, below this the
                             detection is more likely a texture false positive
        max_hamming          corrected bits tolerated; 0 means exact decode
        gate_distance        an observation further than this from the running
                             estimate is rejected as an outlier
        gate_patience        consecutive rejections before the estimate itself
                             is assumed wrong and restarted
        """
        self.max_range = float(max_range)
        self.min_range = float(min_range)
        self.min_decision_margin = float(min_decision_margin)
        self.max_hamming = int(max_hamming)
        self.gate_distance = float(gate_distance)
        self.gate_patience = int(gate_patience)

        self._tags = {}
        self.discarded = 0

    # ------------------------------------------------------------------ #
    # Query
    # ------------------------------------------------------------------ #

    def __len__(self):
        return len(self._tags)

    def __contains__(self, tag_id):
        return int(tag_id) in self._tags

    def __getitem__(self, tag_id):
        return self._tags[int(tag_id)]

    @property
    def ids(self):
        return sorted(self._tags)

    def tags(self):
        """Estimates ordered by ID."""
        return [self._tags[i] for i in self.ids]

    # ------------------------------------------------------------------ #
    # Update
    # ------------------------------------------------------------------ #

    def acceptable(self, range_m, decision_margin=None, hamming=0):
        """Quality gate applied before an observation is allowed to fuse."""
        if not math.isfinite(range_m) or range_m > self.max_range:
            return False
        if hamming is not None and hamming > self.max_hamming:
            return False
        if decision_margin is not None and decision_margin < self.min_decision_margin:
            return False
        return True

    def observe(self, tag_id, x, y, z, stamp,
                range_m=1.0, decision_margin=None, hamming=0):
        """Fold one detection into the map.

        Returns NEW, FUSED, REJECTED or DISCARDED so the caller can log the
        interesting transitions without re-deriving them.
        """
        if not all(math.isfinite(v) for v in (x, y, z)):
            self.discarded += 1
            return self.DISCARDED
        if not self.acceptable(range_m, decision_margin, hamming):
            self.discarded += 1
            return self.DISCARDED

        weight = 1.0 / max(range_m, self.min_range) ** 2
        tag_id = int(tag_id)

        estimate = self._tags.get(tag_id)
        if estimate is None:
            self._tags[tag_id] = TagEstimate(tag_id, x, y, z, weight, stamp, range_m)
            return self.NEW

        if estimate.distance_to(x, y, z) > self.gate_distance:
            if estimate.reject() >= self.gate_patience:
                estimate.reset_to(x, y, z, weight, stamp, range_m)
                return self.FUSED
            return self.REJECTED

        estimate.fuse(x, y, z, weight, stamp, range_m)
        return self.FUSED

    # ------------------------------------------------------------------ #
    # Export
    # ------------------------------------------------------------------ #

    def to_list(self):
        return [tag.to_dict() for tag in self.tags()]

    def summary(self):
        if not self._tags:
            return 'no tags yet'
        return '{} unique: {}'.format(
            len(self._tags), ', '.join(str(i) for i in self.ids))
