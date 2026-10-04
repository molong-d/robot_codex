"""Pure contact-state rules shared by simulator synchronization and tests."""

from enum import IntEnum


class ContactState(IntEnum):
    UNKNOWN = 0
    NONE = 1
    LEFT_ONLY = 2
    RIGHT_ONLY = 3
    BOTH = 4


def classify_contacts(contacts, object_token="workpiece"):
    left = right = False
    for pair in contacts:
        if not isinstance(pair, (tuple, list)) or len(pair) != 2:
            continue
        names = (str(pair[0]).lower(), str(pair[1]).lower())
        if not any(object_token.lower() in name for name in names):
            continue
        left = left or any("leftfinger" in name for name in names)
        right = right or any("rightfinger" in name for name in names)
    if left and right:
        return ContactState.BOTH
    if left:
        return ContactState.LEFT_ONLY
    if right:
        return ContactState.RIGHT_ONLY
    return ContactState.NONE


def attachment_after_contact(currently_attached, state):
    """Only bilateral evidence changes planning-scene attachment state."""
    if state == ContactState.BOTH:
        return True
    if state == ContactState.NONE:
        return False
    return bool(currently_attached)


def release_confirmed(sample, expected_epoch, after_source_ns, last_sequence):
    """Return NONE only for a new, fresh, post-action sample in this epoch."""
    if not isinstance(sample, dict) or sample.get("fresh") is not True:
        return ContactState.UNKNOWN
    if (sample.get("epoch") != expected_epoch or
            not isinstance(sample.get("sequence"), int) or
            sample["sequence"] <= last_sequence or
            not isinstance(sample.get("source_ns"), int) or
            sample["source_ns"] <= after_source_ns):
        return ContactState.UNKNOWN
    try:
        state = ContactState[sample["state"].upper()]
    except (KeyError, AttributeError):
        return ContactState.UNKNOWN
    return state
