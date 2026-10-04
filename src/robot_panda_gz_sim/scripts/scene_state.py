"""Small request/confirmation state machine for asynchronous MoveIt scene sync."""

from dataclasses import dataclass


@dataclass(frozen=True)
class SceneRequest:
    request_id: int
    version: int
    epoch: int
    attached: bool
    cleanup: bool
    submitted_at: float


class SceneReconciler:
    """Tracks desired scene state separately from service-confirmed state."""

    def __init__(self, response_timeout_s=5.0):
        if response_timeout_s <= 0:
            raise ValueError("response timeout must be positive")
        self.response_timeout_s = response_timeout_s
        self.epoch = 0
        self.version = 0
        self.desired_attached = False
        self.confirmed_attached = None
        self.confirmed_version = None
        self.in_flight = None
        self.dirty = True
        self.ready = False
        self.cleanup_pending = False
        self._next_request_id = 1
        self._timed_out_ids = set()

    def set_desired(self, attached):
        attached = bool(attached)
        if self.desired_attached != attached:
            self.desired_attached = attached
            self.version += 1
            self.dirty = True
            self.ready = False

    def mark_dirty(self):
        self.version += 1
        self.dirty = True
        self.ready = False

    def reset(self):
        self.epoch += 1
        self.version += 1
        self.desired_attached = False
        self.dirty = True
        self.ready = False
        self.cleanup_pending = True

    def submit(self, now, scene_ready):
        if self.in_flight is not None or not self.dirty or not scene_ready:
            self.ready = False
            return None
        request = SceneRequest(self._next_request_id, self.version, self.epoch,
                               self.desired_attached, self.cleanup_pending, now)
        self._next_request_id += 1
        self.in_flight = request
        self.dirty = False
        self.ready = False
        return request

    def check_timeout(self, now):
        if self.in_flight is None or self.in_flight.request_id in self._timed_out_ids:
            return False
        if now - self.in_flight.submitted_at < self.response_timeout_s:
            return False
        self._timed_out_ids.add(self.in_flight.request_id)
        self.dirty = True
        self.ready = False
        return True

    def complete(self, request, success):
        # Ignore duplicate or unrelated future callbacks. A timed-out request
        # remains serialized until its response arrives because the service may
        # still apply it after the caller's local deadline.
        if self.in_flight is None or request.request_id != self.in_flight.request_id:
            return False
        self.in_flight = None
        if success:
            self.confirmed_attached = request.attached
            self.confirmed_version = request.version
            if request.cleanup and request.epoch == self.epoch:
                self.cleanup_pending = False
        if not success or request.cleanup or request.epoch != self.epoch or request.version != self.version or \
                request.attached != self.desired_attached or self.cleanup_pending:
            self.dirty = True
            self.ready = False
        else:
            self.dirty = False
            self.ready = True
        return True
