"""
Cartesian trajectory helpers for the AL5B arm.

Two levels of planning are provided:

1. ``move_to_pose`` — single-segment Cartesian move with trapezoidal-ish
   timing. Internally this is just a synchronised multi-servo SSC-32 "T<ms>"
   command, so the SSC-32 itself handles the per-servo interpolation.

2. ``follow_waypoints`` — a sequence of (x,y,z,pitch) waypoints and per-segment
   durations. Useful for scripted demos like pick-and-place.

For richer motion (straight-line Cartesian paths, velocity profiles) you would
subdivide segments into sub-waypoints here and push each one to the SSC-32 at
fixed intervals.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

from ssc32_driver import SSC32, CH_GRIPPER
from al5b_kinematics import (
    JointAngles,
    inverse_kinematics,
    angles_to_pulses,
    GRIPPER_OPEN_US,
    GRIPPER_CLOSE_US,
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


# ---------------------------------------------------------------------------
# Named poses — useful starting points for scripted demos.
# Coordinates are in millimetres; the world frame has +X forward from the arm,
# +Y to the arm's left, +Z up. The arm base sits at the origin.
# ---------------------------------------------------------------------------
HOME_POSE = Waypoint(x=250.0, y=0.0, z=200.0, pitch=0.0,
                     gripper_us=GRIPPER_OPEN_US, duration_ms=2000)


def pick_and_place(arm: SSC32,
                   pick_xy=(280.0, 80.0),
                   place_xy=(280.0, -80.0),
                   table_z: float = 30.0,
                   hover_z: float = 120.0,
                   pitch: float = -1.2,   # ~-70 deg: tool angled down
                   stop_event=None) -> List[Waypoint]:
    """A simple pick-and-place demo.

    The arm:
        1. goes home (arm up, gripper open)
        2. moves above the pick point
        3. descends to table height
        4. closes gripper
        5. ascends to hover height
        6. traverses to above the place point
        7. descends to table height
        8. opens gripper
        9. ascends and returns home

    Returns the waypoint list (also useful for dry-run visualisation).
    """
    px, py = pick_xy
    qx, qy = place_xy

    seq = [
        HOME_POSE,
        Waypoint(px, py, hover_z, pitch, GRIPPER_OPEN_US, 1500),
        Waypoint(px, py, table_z, pitch, GRIPPER_OPEN_US, 1200),
        Waypoint(px, py, table_z, pitch, GRIPPER_CLOSE_US, 600),   # close gripper
        Waypoint(px, py, hover_z, pitch, GRIPPER_CLOSE_US, 1200),
        Waypoint(qx, qy, hover_z, pitch, GRIPPER_CLOSE_US, 1800),
        Waypoint(qx, qy, table_z, pitch, GRIPPER_CLOSE_US, 1200),
        Waypoint(qx, qy, table_z, pitch, GRIPPER_OPEN_US, 600),    # release
        Waypoint(qx, qy, hover_z, pitch, GRIPPER_OPEN_US, 1200),
        HOME_POSE,
    ]
    follow_waypoints(arm, seq, stop_event=stop_event)
    return list(seq)


if __name__ == "__main__":
    # Dry-run the pick-and-place on the simulator.
    from ssc32_driver import FakeSSC32
    arm = FakeSSC32(verbose=False)
    arm.open()
    wps = pick_and_place(arm)
    print("Executed", len(wps), "waypoints on fake arm")
    for i, w in enumerate(wps):
        print("  {:2d} x={:+6.1f} y={:+6.1f} z={:+6.1f} pitch={:+.2f} grip={} t={}ms"
              .format(i, w.x, w.y, w.z, w.pitch, w.gripper_us, w.duration_ms))
