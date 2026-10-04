"""Pure ROS-time ordering checks shared by the simulator bridge and audit tool."""


class SourceTimeGuard:
    def __init__(self, max_age_ns=500_000_000, reset_threshold_ns=100_000_000):
        if max_age_ns <= 0 or reset_threshold_ns <= 0:
            raise ValueError("time limits must be positive")
        self.max_age_ns = max_age_ns
        self.reset_threshold_ns = reset_threshold_ns
        self.epoch = 0
        self.last_clock_ns = 0
        self.last_source_by_stream = {}
        self.reset_barrier_by_stream = {}
        self.awaiting_reset_streams = set()

    def observe(self, stream, source_ns, clock_ns):
        """Return (accepted, epoch, reason), without refreshing rejected samples."""
        if clock_ns <= 0:
            return False, self.epoch, "invalid_clock"
        if self.last_clock_ns and clock_ns + self.reset_threshold_ns < self.last_clock_ns:
            self.epoch += 1
            self.reset_barrier_by_stream = dict(self.last_source_by_stream)
            self.awaiting_reset_streams = set(self.last_source_by_stream)
            self.last_source_by_stream.clear()
        elif self.last_clock_ns and clock_ns < self.last_clock_ns:
            return False, self.epoch, "clock_jitter"
        self.last_clock_ns = clock_ns

        if source_ns <= 0:
            return False, self.epoch, "invalid_source_time"
        if source_ns > clock_ns:
            return False, self.epoch, "future_source_time"
        if clock_ns - source_ns > self.max_age_ns:
            return False, self.epoch, "stale_source_time"
        if stream in self.awaiting_reset_streams:
            barrier = self.reset_barrier_by_stream[stream]
            # A callback can be delayed across the reset and arrive after the
            # restarted clock has advanced. Keep it blocked until this stream
            # passes the old high-water mark so old callbacks cannot seed a
            # new-epoch cache.
            if source_ns <= barrier:
                return False, self.epoch, "pre_reset_source_time"
            self.awaiting_reset_streams.remove(stream)
        previous = self.last_source_by_stream.get(stream, 0)
        if source_ns <= previous:
            return False, self.epoch, "duplicate_or_out_of_order"
        self.last_source_by_stream[stream] = source_ns
        return True, self.epoch, "accepted"
