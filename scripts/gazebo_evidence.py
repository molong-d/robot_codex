#!/usr/bin/env python3
"""Independent, timestamped audit for Panda Gazebo physical observations."""

import math


def _pairs(sample):
    return [" ".join(pair).lower() for pair in sample.get("contacts", [])
            if isinstance(pair, list) and len(pair) == 2]


def _pair_matches(pairs, object_token, other_token):
    return any(object_token in pair and other_token in pair for pair in pairs)


def _fresh(sample, final_wall_ns, max_age_ns):
    receipt = sample.get("receipt_monotonic_ns")
    return isinstance(receipt, int) and 0 <= final_wall_ns - receipt <= max_age_ns


def _valid_source_time(sample, max_age_ns):
    source = sample.get("source_stamp_ns")
    clock = sample.get("sim_time_ns")
    return (isinstance(source, int) and isinstance(clock, int) and source > 0 and
            source <= clock and clock - source <= max_age_ns)


def _ordered_valid_samples(samples, epoch, world_frame, max_age_ns, gazebo_world_name,
                           expected_object=None):
    last_source = {}
    last_sequence = {}
    valid = set()
    for index, sample in enumerate(samples):
        stream = sample.get("stream")
        current_epoch = sample.get("epoch")
        source = sample.get("source_stamp_ns")
        sequence = sample.get("sequence")
        source_frame = sample.get("source_frame_id")
        if (sample.get("accepted") is not True or sample.get("frame_id") != world_frame or
                current_epoch != epoch or not _valid_source_time(sample, max_age_ns) or
                not isinstance(stream, str) or not isinstance(sequence, int) or sequence <= 0 or
                (expected_object is not None and sample.get("object_id") != expected_object)):
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
        valid.add(index)
    return valid


def audit_physics(result, acceptance):
    """Compute outcome from measured pose/contact histories, not task success."""
    max_age_ns = int(acceptance["evidence_max_age_ms"] * 1_000_000)
    xy_tolerance = float(acceptance["placement_xy_tolerance_m"])
    z_tolerance = float(acceptance["placement_z_tolerance_m"])
    stable_speed = float(acceptance["stable_speed_mps"])
    stable_window_ns = int(acceptance["verification_window_ms"] * 1_000_000)
    minimum_samples = int(acceptance["minimum_physical_samples"])
    final_wall_ns = result.get("final_receipt_monotonic_ns", 0)
    final_sim_ns = result.get("final_sim_time_ns", 0)
    epoch = result.get("final_epoch")

    poses = result.get("pose_history", [])
    contacts = result.get("contact_samples", [])
    events = result.get("task_events", [])
    valid_pose_indices = _ordered_valid_samples(
        poses, epoch, acceptance["world_frame"], max_age_ns, acceptance["gazebo_world_name"])
    accepted_poses = [s for i, s in enumerate(poses) if i in valid_pose_indices and
                      isinstance(s.get("xyz"), list) and len(s["xyz"]) == 3]
    object_poses = [s for s in accepted_poses if s.get("object_id") == acceptance["object_id"]]
    tray_poses = [s for s in accepted_poses if s.get("object_id") == acceptance["target_id"]]
    valid_contact_indices = _ordered_valid_samples(contacts, epoch, acceptance["world_frame"],
                                                    max_age_ns, acceptance["gazebo_world_name"],
                                                    acceptance["object_id"])
    accepted_contacts = [s for i, s in enumerate(contacts) if i in valid_contact_indices]

    release_events = [e for e in events if e.get("phase") == "release_confirmed" and
                      isinstance(e.get("sim_time_ns"), int)]
    release_sim_ns = release_events[-1]["sim_time_ns"] if release_events else None
    dual = []
    for sample in accepted_contacts:
        pairs = _pairs(sample)
        if (_pair_matches(pairs, "workpiece", "leftfinger") and
                _pair_matches(pairs, "workpiece", "rightfinger")):
            dual.append(sample)
    grasp_time_ns = dual[-1].get("source_stamp_ns") if dual else None
    lift_height = (float(acceptance["support_surface_z_m"]) +
                   float(acceptance["object_height_m"]) / 2.0 +
                   float(acceptance["grasp_lift_m"]))
    lifted = [s for s in object_poses if grasp_time_ns is not None and
              s["source_stamp_ns"] > grasp_time_ns and s["xyz"][2] >= lift_height]

    post_release_no_contact = []
    tray_floor = []
    for sample in accepted_contacts:
        if release_sim_ns is None or sample["source_stamp_ns"] <= release_sim_ns:
            continue
        pairs = _pairs(sample)
        finger_contact = (_pair_matches(pairs, "workpiece", "leftfinger") or
                          _pair_matches(pairs, "workpiece", "rightfinger"))
        if not finger_contact:
            post_release_no_contact.append(sample)
        if _pair_matches(pairs, "workpiece", "tray_floor") and not finger_contact:
            tray_floor.append(sample)

    final_object = object_poses[-1] if object_poses else None
    final_tray = tray_poses[-1] if tray_poses else None
    xy_error = z_error = None
    if final_object and final_tray:
        xy_error = math.hypot(final_object["xyz"][0] - final_tray["xyz"][0],
                              final_object["xyz"][1] - final_tray["xyz"][1])
        z_error = abs(final_object["xyz"][2] -
                      (float(acceptance["support_surface_z_m"]) + float(acceptance["object_height_m"]) / 2.0))

    stable = []
    if final_tray:
        for sample in object_poses:
            distance_xy = math.hypot(sample["xyz"][0] - final_tray["xyz"][0],
                                     sample["xyz"][1] - final_tray["xyz"][1])
            distance_z = abs(sample["xyz"][2] -
                             (float(acceptance["support_surface_z_m"]) +
                              float(acceptance["object_height_m"]) / 2.0))
            if distance_xy <= xy_tolerance and distance_z <= z_tolerance:
                if stable:
                    prev = stable[-1]
                    dt_ns = sample["source_stamp_ns"] - prev["source_stamp_ns"]
                    if dt_ns <= 0:
                        stable = []
                    else:
                        speed = math.dist(sample["xyz"], prev["xyz"]) / (dt_ns * 1e-9)
                        if (sample["epoch"] != prev["epoch"] or dt_ns > max_age_ns or
                                speed > stable_speed):
                            stable = [sample]
                        else:
                            stable.append(sample)
                else:
                    stable = [sample]
            else:
                stable = []
    stable_span_ns = (stable[-1]["source_stamp_ns"] - stable[0]["source_stamp_ns"]
                      if len(stable) >= 2 else 0)
    stable_contact = [s for s in tray_floor if stable and
                      stable[0]["source_stamp_ns"] <= s["source_stamp_ns"] <= stable[-1]["source_stamp_ns"]]
    latest_contact = accepted_contacts[-1] if accepted_contacts else None
    latest_is_tray = bool(latest_contact and
                          _pair_matches(_pairs(latest_contact), "workpiece", "tray_floor") and
                          not _pair_matches(_pairs(latest_contact), "workpiece", "leftfinger") and
                          not _pair_matches(_pairs(latest_contact), "workpiece", "rightfinger"))
    pair_tolerance_ns = int(acceptance["contact_pose_pairing_tolerance_ms"] * 1_000_000)
    final_contact_pose_coherent = bool(final_object and latest_contact and
        abs(final_object["source_stamp_ns"] - latest_contact["source_stamp_ns"]) <= pair_tolerance_ns)

    checks = {
        "same_epoch_world_poses": bool(final_object and final_tray),
        "fresh_final_object_pose": bool(final_object and _fresh(final_object, final_wall_ns, max_age_ns)),
        "fresh_final_target_pose": bool(final_tray and _fresh(final_tray, final_wall_ns, max_age_ns)),
        "dual_finger_contact_same_sample": bool(dual),
        "object_lifted_after_grasp": bool(lifted),
        "release_event_observed": release_sim_ns is not None,
        "fresh_post_release_no_finger_contact": bool(post_release_no_contact and
            _fresh(post_release_no_contact[-1], final_wall_ns, max_age_ns)),
        "final_tray_floor_contact": bool(latest_is_tray and latest_contact and
            _fresh(latest_contact, final_wall_ns, max_age_ns)),
        "final_contact_matches_pose_time": final_contact_pose_coherent,
        "tray_contact_during_stable_window": bool(stable_contact),
        "placement_xy_within_tolerance": xy_error is not None and xy_error <= xy_tolerance,
        "placement_z_within_tolerance": z_error is not None and z_error <= z_tolerance,
        "stable_window_observed": len(stable) >= minimum_samples and stable_span_ns >= stable_window_ns,
        "stable_pose_samples_fresh": bool(stable and all(
            _fresh(s, final_wall_ns, max_age_ns) for s in stable[-minimum_samples:])),
    }
    return {
        "independently_valid_physical_outcome": all(checks.values()),
        "checks": checks,
        "placement_xy_error_m": xy_error,
        "placement_z_error_m": z_error,
        "stable_pose_sample_count": len(stable),
        "stable_window_observed_ms": stable_span_ns / 1_000_000,
        "dual_finger_contact_sample_count": len(dual),
        "post_release_no_contact_sample_count": len(post_release_no_contact),
        "tray_floor_contact_sample_count": len(tray_floor),
        "final_workpiece_xyz_m": None if final_object is None else final_object["xyz"],
        "final_tray_xyz_m": None if final_tray is None else final_tray["xyz"],
        "release_event_sim_time_ns": release_sim_ns,
        "final_sim_time_ns": final_sim_ns,
        "final_epoch": epoch,
    }
