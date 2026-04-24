"""
Trajectory helpers for the AL5B arm.

Two styles are supported:

1. ``move_to_pose`` / ``follow_waypoints`` for Cartesian IK-driven moves.
2. ``move_to_pulses`` / ``follow_pulse_steps`` for fixed servo-pulse scripts.

The current pick-and-place demo uses a fixed pulse script because the pickup
and drop poses have been tuned directly on the real arm.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

from ssc32_driver import (
    SSC32,
    CH_BASE, CH_SHOULDER, CH_ELBOW, CH_WRIST, CH_GRIPPER,
)
from al5b_kinematics import (
    inverse_kinematics,
    angles_to_pulses,
    GRIPPER_OPEN_US,
)


@dataclass
class Waypoint:
    """A target pose for the tool tip."""
    x: float
    y: float
    z: float
    pitch: float = 0.0           # radians
    gripper_us: Optional[int] = None  # None = leave as-is
    duration_ms: int = 1500      # how long to take reaching this waypoint


@dataclass
class PulseStep:
    """A fixed per-channel pulse target for scripted motions."""
    name: str
    pulses: Dict[int, int]
    duration_ms: int = 1500
    settle_s: float = 0.25


def move_to_pose(arm: SSC32,
                 x: float, y: float, z: float, pitch: float = 0.0,
                 duration_ms: int = 1500,
                 gripper_us: Optional[int] = None,
                 wait: bool = True) -> bool:
    """Move the tip to (x, y, z, pitch). Returns False if IK fails."""
    sol = inverse_kinematics(x, y, z, pitch)
    if sol is None:
        return False
    targets = angles_to_pulses(sol)
    if gripper_us is not None:
        targets[CH_GRIPPER] = gripper_us
    arm.move_many(targets, time_ms=duration_ms)
    if wait:
        # A small buffer: SSC-32 reports idle slightly before the full T elapses.
        time.sleep(duration_ms / 1000.0)
    return True


def _sleep_interruptible(total_s: float, stop_event=None) -> bool:
    """Sleep in short slices so scripted motions can stop promptly."""
    deadline = time.monotonic() + total_s
    while time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            return False
        remaining = deadline - time.monotonic()
        time.sleep(min(0.05, remaining))
    return True


def move_to_pulses(arm: SSC32, pulses: Dict[int, int], duration_ms: int = 1500,
                   settle_s: float = 0.25, stop_event=None) -> bool:
    """Move directly to a known-good set of servo pulses."""
    arm.move_many(pulses, time_ms=duration_ms)
    return _sleep_interruptible(duration_ms / 1000.0 + settle_s, stop_event)


def follow_waypoints(arm: SSC32, waypoints: Sequence[Waypoint],
                     stop_event=None) -> List[bool]:
    """Run through a list of waypoints. Set stop_event to interrupt cleanly."""
    results = []
    for wp in waypoints:
        if stop_event is not None and stop_event.is_set():
            break
        ok = move_to_pose(arm, wp.x, wp.y, wp.z, wp.pitch,
                          duration_ms=wp.duration_ms,
                          gripper_us=wp.gripper_us,
                          wait=True)
        results.append(ok)
    return results


def follow_pulse_steps(arm: SSC32, steps: Sequence[PulseStep],
                       stop_event=None) -> List[str]:
    """Run through a list of pulse-script steps."""
    executed = []
    for step in steps:
        if stop_event is not None and stop_event.is_set():
            break
        move_to_pulses(arm, step.pulses, duration_ms=step.duration_ms,
                       settle_s=step.settle_s, stop_event=stop_event)
        executed.append(step.name)
    return executed


# ---------------------------------------------------------------------------
# Named poses — useful starting points for scripted demos.
# Coordinates are in millimetres; the world frame has +X forward from the arm,
# +Y to the arm's left, +Z up. The arm base sits at the origin.
# ---------------------------------------------------------------------------
HOME_POSE = Waypoint(x=250.0, y=0.0, z=200.0, pitch=0.0,
                     gripper_us=GRIPPER_OPEN_US, duration_ms=2000)

PICK_POSE_PULSES = {
    CH_BASE: 1690,
    CH_SHOULDER: 1120,
    CH_ELBOW: 1280,
    CH_WRIST: 970,
    CH_GRIPPER: 2000,
}

PLACE_POSE_HOLD_PULSES = {
    CH_BASE: 1090,
    CH_SHOULDER: 1310,
    CH_ELBOW: 1470,
    CH_WRIST: 940,
    CH_GRIPPER: 2000,
}

PLACE_POSE_DROP_PULSES = {
    CH_BASE: 1090,
    CH_SHOULDER: 1310,
    CH_ELBOW: 1470,
    CH_WRIST: 940,
    CH_GRIPPER: 1000,
}


def pick_and_place(arm: SSC32, stop_event=None) -> List[str]:
    """Run the tuned pickup/drop script using fixed servo pulses.

    Sequence:
        1. Try to move to HOME_POSE using IK.
        2. Move to the tuned pickup pulses and grip there (CH4=2000).
        3. Move to the tuned place pose while still gripping.
        4. Release at the place pose by switching CH4 to 1000.
    """
    if stop_event is not None and stop_event.is_set():
        return []

    home_ok = move_to_pose(
        arm,
        HOME_POSE.x,
        HOME_POSE.y,
        HOME_POSE.z,
        HOME_POSE.pitch,
        duration_ms=HOME_POSE.duration_ms,
        gripper_us=HOME_POSE.gripper_us,
        wait=False,
    )
    if not home_ok:
        print("[demo] home pose unreachable; continuing with tuned pulse sequence")
    elif not _sleep_interruptible(HOME_POSE.duration_ms / 1000.0 + 0.25, stop_event):
        return ["home"]

    seq = [
        PulseStep("pick", PICK_POSE_PULSES, duration_ms=1700, settle_s=0.50),
        PulseStep("carry_to_place", PLACE_POSE_HOLD_PULSES, duration_ms=1700, settle_s=0.40),
        PulseStep("drop", PLACE_POSE_DROP_PULSES, duration_ms=500, settle_s=0.60),
    ]
    executed = []
    if home_ok:
        executed.append("home")
    executed.extend(follow_pulse_steps(arm, seq, stop_event=stop_event))
    return executed


if __name__ == "__main__":
    # Dry-run the pick-and-place on the simulator.
    from ssc32_driver import FakeSSC32
    arm = FakeSSC32(verbose=False)
    arm.open()
    steps = pick_and_place(arm)
    print("Executed", len(steps), "script steps on fake arm")
    for i, step in enumerate(steps):
        print("  {:2d} {}".format(i, step))
