#!/usr/bin/env python3
"""Independent audit of timestamped Panda Gazebo observations (audit v2)."""

import math

AUDIT_VERSION = "2.0.0"


def _pairs(sample):
    return [" ".join(pair).lower() for pair in sample.get("contacts", [])
            if isinstance(pair, list) and len(pair) == 2]


def _pair_matches(pairs, object_token, other_token):
    return any(object_token in pair and other_token in pair for pair in pairs)


def _finger_contact(pairs):
    return (_pair_matches(pairs, "workpiece", "leftfinger") or
            _pair_matches(pairs, "workpiece", "rightfinger"))


def _fresh(sample, final_wall_ns, max_age_ns):
    receipt = sample.get("receipt_monotonic_ns")
    return isinstance(receipt, int) and 0 <= final_wall_ns - receipt <= max_age_ns


def _valid_source_time(sample, max_age_ns):
    source = sample.get("source_stamp_ns")
    clock = sample.get("sim_time_ns")
    return (isinstance(source, int) and isinstance(clock, int) and source > 0 and
            source <= clock and clock - source <= max_age_ns)


def _ordered_valid_samples(samples, epoch, world_frame, max_age_ns, gazebo_world_name,
                           run_id, expected_object):
    last_source = {}
    last_sequence = {}
    valid = []
    for index, sample in enumerate(samples):
        stream = sample.get("stream")
        current_epoch = sample.get("epoch")
        source = sample.get("source_stamp_ns")
        sequence = sample.get("sequence")
        source_frame = sample.get("source_frame_id")
        if (sample.get("accepted") is not True or sample.get("run_id") != run_id or
                sample.get("frame_id") != world_frame or current_epoch != epoch or
                not _valid_source_time(sample, max_age_ns) or
                not isinstance(stream, str) or not isinstance(sequence, int) or sequence <= 0 or
                sample.get("object_id") != expected_object):
            continue
        if stream != "contact" and stream != f"pose:{expected_object}":
            continue
        allowed_source_frames = {world_frame, gazebo_world_name}
        if stream == "contact":
            allowed_source_frames.add("")
        if source_frame not in allowed_source_frames:
            continue
        key = (stream, current_epoch)
        if source <= last_source.get(key, 0) or sequence <= last_sequence.get(key, 0):
            continue
        last_source[key] = source
        last_sequence[key] = sequence
        valid.append((index, sample))
    return valid


def _nearest_unused(samples, source_ns, tolerance_ns, used):
    candidates = [(abs(sample["source_stamp_ns"] - source_ns), index, sample)
                  for index, sample in samples if index not in used]
    if not candidates:
        return None
    delta, index, sample = min(candidates, key=lambda item: item[0])
    return (index, sample) if delta <= tolerance_ns else None


def audit_physics(result, acceptance):
    """Compute physical success without reading runtime success/result flags."""
    max_age_ns = int(acceptance["evidence_max_age_ms"] * 1_000_000)
    max_gap_ns = int(acceptance["stability_max_gap_ms"] * 1_000_000)
    pair_tolerance_ns = int(acceptance["contact_pose_pairing_tolerance_ms"] * 1_000_000)
    xy_tolerance = float(acceptance["placement_xy_tolerance_m"])
    z_tolerance = float(acceptance["placement_z_tolerance_m"])
    stable_speed = float(acceptance["stable_speed_mps"])
    stable_window_ns = int(acceptance["verification_window_ms"] * 1_000_000)
    minimum_samples = int(acceptance["minimum_physical_samples"])
    support_z = float(acceptance["support_surface_z_m"])
    object_height = float(acceptance["object_height_m"])
    run_id = result.get("run_id")
    object_id = acceptance["object_id"]
    target_id = acceptance["target_id"]
    final_wall_ns = result.get("final_receipt_monotonic_ns", 0)
    final_sim_ns = result.get("final_sim_time_ns", 0)
    epoch = result.get("final_epoch")

    poses = result.get("pose_history", [])
    contacts = result.get("contact_samples", [])
    events = result.get("task_events", [])
    valid_object = _ordered_valid_samples(poses, epoch, acceptance["world_frame"], max_age_ns,
        acceptance["gazebo_world_name"], run_id, object_id)
    valid_target = _ordered_valid_samples(poses, epoch, acceptance["world_frame"], max_age_ns,
        acceptance["gazebo_world_name"], run_id, target_id)
    valid_contacts = _ordered_valid_samples(contacts, epoch, acceptance["world_frame"], max_age_ns,
        acceptance["gazebo_world_name"], run_id, object_id)
    object_poses = [(i, s) for i, s in valid_object if isinstance(s.get("xyz"), list) and len(s["xyz"]) == 3]
    target_poses = [(i, s) for i, s in valid_target if isinstance(s.get("xyz"), list) and len(s["xyz"]) == 3]

    release_events = [event for event in events if event.get("phase") == "release_confirmed" and
        event.get("run_id") == run_id and event.get("object_id") == object_id and
        event.get("target_id") == target_id and event.get("epoch") == epoch and
        isinstance(event.get("sim_time_ns"), int) and event.get("sim_time_ns") > 0 and
        isinstance(event.get("receipt_monotonic_ns"), int) and
        0 <= final_wall_ns - event["receipt_monotonic_ns"] and
        event["sim_time_ns"] <= final_sim_ns]
    release = release_events[-1] if release_events else None
    release_sim_ns = release["sim_time_ns"] if release else None

    dual = []
    for _, sample in valid_contacts:
        pairs = _pairs(sample)
        if (_pair_matches(pairs, "workpiece", "leftfinger") and
                _pair_matches(pairs, "workpiece", "rightfinger")):
            dual.append(sample)
    grasp_time_ns = dual[0]["source_stamp_ns"] if dual else None
    lift_height = support_z + object_height / 2.0 + float(acceptance["grasp_lift_m"])
    lifted = [sample for _, sample in object_poses if grasp_time_ns is not None and
              sample["source_stamp_ns"] > grasp_time_ns and sample["xyz"][2] >= lift_height]

    latest_object = object_poses[-1][1] if object_poses else None
    latest_target = target_poses[-1][1] if target_poses else None
    xy_error = z_error = None
    if latest_object and latest_target:
        xy_error = math.hypot(latest_object["xyz"][0] - latest_target["xyz"][0],
                              latest_object["xyz"][1] - latest_target["xyz"][1])
        z_error = abs(latest_object["xyz"][2] - (support_z + object_height / 2.0))

    # Join every post-release object pose to a distinct advancing target pose
    # and contact sample. Any missing pair, finger contact, unsupported object,
    # long gap, rewind, or excessive speed breaks the candidate window.
    stable = []
    post_release_none = []
    used_targets, used_contacts = set(), set()
    final_contact_for_pose = None
    if release_sim_ns is not None:
        post_release_targets = [(i, s) for i, s in target_poses if s["source_stamp_ns"] > release_sim_ns]
        post_release_contacts = [(i, s) for i, s in valid_contacts if s["source_stamp_ns"] > release_sim_ns]
        for _, obj in object_poses:
            if obj["source_stamp_ns"] <= release_sim_ns:
                continue
            target_match = _nearest_unused(post_release_targets, obj["source_stamp_ns"], pair_tolerance_ns, used_targets)
            contact_match = _nearest_unused(post_release_contacts, obj["source_stamp_ns"], pair_tolerance_ns, used_contacts)
            if target_match is None or contact_match is None:
                stable = []
                continue
            target_index, target = target_match
            contact_index, contact_sample = contact_match
            if (target["epoch"] != obj["epoch"] or contact_sample["epoch"] != obj["epoch"] or
                    target["source_stamp_ns"] <= release_sim_ns or
                    contact_sample["source_stamp_ns"] <= release_sim_ns):
                stable = []
                continue
            used_targets.add(target_index)
            used_contacts.add(contact_index)
            pairs = _pairs(contact_sample)
            has_finger = _finger_contact(pairs)
            has_support = _pair_matches(pairs, "workpiece", "tray_floor")
            if not has_finger:
                post_release_none.append(contact_sample)
            rel = (obj["xyz"][0] - target["xyz"][0],
                   obj["xyz"][1] - target["xyz"][1],
                   obj["xyz"][2] - (support_z + object_height / 2.0))
            inside = math.hypot(rel[0], rel[1]) <= xy_tolerance and abs(rel[2]) <= z_tolerance
            fresh_triplet = all(_valid_source_time(sample, max_age_ns)
                                for sample in (obj, target, contact_sample))
            valid_step = inside and has_support and not has_finger and fresh_triplet
            if stable:
                previous = stable[-1]
                dt_ns = obj["source_stamp_ns"] - previous["object"]["source_stamp_ns"]
                contact_dt = contact_sample["source_stamp_ns"] - previous["contact"]["source_stamp_ns"]
                speed = math.dist(rel, previous["relative_xyz"]) / (dt_ns * 1e-9) if dt_ns > 0 else math.inf
                obj_wall_gap = obj["receipt_monotonic_ns"] - previous["object"]["receipt_monotonic_ns"]
                target_wall_gap = target["receipt_monotonic_ns"] - previous["target"]["receipt_monotonic_ns"]
                contact_wall_gap = contact_sample["receipt_monotonic_ns"] - previous["contact"]["receipt_monotonic_ns"]
                valid_step = (valid_step and 0 < dt_ns <= max_gap_ns and 0 < contact_dt <= max_gap_ns and
                              0 < obj_wall_gap <= max_gap_ns and 0 < target_wall_gap <= max_gap_ns and
                              0 < contact_wall_gap <= max_gap_ns and speed <= stable_speed)
            if valid_step:
                stable.append({"object": obj, "target": target, "contact": contact_sample, "relative_xyz": rel})
                final_contact_for_pose = contact_sample
            else:
                stable = []

    stable_span_ns = (stable[-1]["object"]["source_stamp_ns"] - stable[0]["object"]["source_stamp_ns"]
                      if len(stable) >= 2 else 0)
    latest_contact = valid_contacts[-1][1] if valid_contacts else None
    latest_pairs = _pairs(latest_contact) if latest_contact else []
    latest_is_tray = bool(latest_contact and _pair_matches(latest_pairs, "workpiece", "tray_floor") and
                          not _finger_contact(latest_pairs))
    final_contact_pose_coherent = bool(latest_object and latest_contact and
        abs(latest_object["source_stamp_ns"] - latest_contact["source_stamp_ns"]) <= pair_tolerance_ns)
    stable_contacts_all_supported = bool(stable and all(
        _pair_matches(_pairs(item["contact"]), "workpiece", "tray_floor") and
        not _finger_contact(_pairs(item["contact"])) for item in stable))
    stable_all_fresh = bool(stable and all(_valid_source_time(item[key], max_age_ns)
        for item in stable for key in ("object", "target", "contact")))

    checks = {
        "same_epoch_world_poses": bool(latest_object and latest_target),
        "fresh_final_object_pose": bool(latest_object and _fresh(latest_object, final_wall_ns, max_age_ns)),
        "fresh_final_target_pose": bool(latest_target and _fresh(latest_target, final_wall_ns, max_age_ns)),
        "dual_finger_contact_same_sample": bool(dual),
        "object_lifted_after_grasp": bool(lifted),
        "release_identity_matches": release is not None,
        "fresh_post_release_no_finger_contact": bool(post_release_none and
            _fresh(post_release_none[-1], final_wall_ns, max_age_ns)),
        "final_tray_floor_contact": bool(latest_is_tray and latest_contact and
            _fresh(latest_contact, final_wall_ns, max_age_ns)),
        "final_contact_matches_pose_time": final_contact_pose_coherent,
        "tray_contact_during_stable_window": stable_contacts_all_supported,
        "placement_xy_within_tolerance": xy_error is not None and xy_error <= xy_tolerance,
        "placement_z_within_tolerance": z_error is not None and z_error <= z_tolerance,
        "stable_window_observed": len(stable) >= minimum_samples and stable_span_ns >= stable_window_ns,
        "stable_pose_samples_fresh": stable_all_fresh,
    }
    return {
        "audit_version": AUDIT_VERSION,
        "independently_valid_physical_outcome": all(checks.values()),
        "checks": checks,
        "placement_xy_error_m": xy_error,
        "placement_z_error_m": z_error,
        "stable_pose_sample_count": len(stable),
        "stable_window_observed_ms": stable_span_ns / 1_000_000,
        "dual_finger_contact_sample_count": len(dual),
        "post_release_no_contact_sample_count": len(post_release_none),
        "tray_floor_contact_sample_count": sum(
            _pair_matches(_pairs(sample), "workpiece", "tray_floor") for _, sample in valid_contacts),
        "final_workpiece_xyz_m": None if latest_object is None else latest_object["xyz"],
        "final_tray_xyz_m": None if latest_target is None else latest_target["xyz"],
        "release_event_sim_time_ns": release_sim_ns,
        "final_sim_time_ns": final_sim_ns,
        "final_epoch": epoch,
    }
