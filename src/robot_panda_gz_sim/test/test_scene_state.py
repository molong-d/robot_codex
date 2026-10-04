#!/usr/bin/env python3
"""Regression tests for MoveIt scene request/confirmation interleavings."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from scene_state import SceneReconciler


def require(value, message):
    if not value:
        raise AssertionError(message)


state = SceneReconciler(response_timeout_s=1.0)
initial = state.submit(0.0, scene_ready=True)
require(initial is not None and not state.ready, "sent scene must not count as synchronized")
require(state.complete(initial, True) and state.ready, "successful current request was not confirmed")
require(state.submit(0.1, scene_ready=True) is None and state.ready,
        "idle timer tick cleared a confirmed scene")
require(state.submit(0.15, scene_ready=False) is None and not state.ready,
        "stale scene inputs did not revoke readiness")
refreshed = state.submit(0.2, scene_ready=True)
require(refreshed is not None and state.complete(refreshed, True) and state.ready,
        "fresh scene inputs did not trigger resynchronization")

state.set_desired(True)
attach = state.submit(0.3, scene_ready=True)
require(attach is not None and attach.attached, "attach snapshot was not captured")
state.set_desired(False)  # Contact was lost before the attach response arrived.
require(state.complete(attach, True), "attach response was not associated with its request")
require(state.confirmed_attached is True and state.dirty and not state.ready,
        "late attach response overwrote desired detach state")
detach = state.submit(0.5, scene_ready=True)
require(detach is not None and not detach.attached, "latest detach state was not resubmitted")
require(state.complete(detach, True) and state.ready and state.confirmed_attached is False,
        "detach was not confirmed")

state.set_desired(True)
attach_again = state.submit(0.7, scene_ready=True)
require(attach_again is not None, "second attach request missing")
state.reset()  # World reset while attach is in flight.
require(not state.ready and state.cleanup_pending and state.epoch == 1,
        "reset did not invalidate scene readiness and request cleanup")
require(state.complete(attach_again, True), "late pre-reset response was discarded without reconciliation")
require(state.dirty and not state.ready and state.confirmed_attached is True,
        "late old-epoch attach response was treated as current")
cleanup = state.submit(0.9, scene_ready=True)
require(cleanup is not None and cleanup.cleanup and not cleanup.attached,
        "reset did not submit an explicit attached-object cleanup")
require(state.complete(cleanup, True) and not state.cleanup_pending and state.dirty,
        "cleanup response should require a fresh post-reset world scene")
world = state.submit(1.1, scene_ready=True)
require(world is not None and not world.cleanup and not world.attached,
        "fresh world scene was not submitted after cleanup")
require(state.complete(world, True) and state.ready, "post-reset world scene did not synchronize")
require(state.submit(1.15, scene_ready=True) is None and state.ready,
        "idle timer tick cleared readiness after reset reconciliation")

state.mark_dirty()
failed = state.submit(1.3, scene_ready=True)
state.complete(failed, False)
require(not state.ready and state.dirty, "service failure falsely marked scene ready")
retry = state.submit(1.5, scene_ready=False)
require(retry is None and not state.ready, "scene sent without fresh scene inputs")
retry = state.submit(1.5, scene_ready=True)
require(retry is not None and state.check_timeout(2.5), "response timeout was not detected")
require(state.submit(2.6, scene_ready=True) is None and not state.ready,
        "timed-out request was overlapped or treated as applied")
require(state.complete(retry, True), "late service response did not reconcile")

print("scene state races, reset cleanup, failure, timeout, and readiness passed")
