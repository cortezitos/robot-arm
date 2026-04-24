"""
Forward and inverse kinematics for the Lynxmotion AL5B arm (5-channel wiring).

The AL5B here is a 4-DOF positioning arm plus a gripper:
    J0 - base rotation about +Z (world yaw)   -> SSC-32 CH0
    J1 - shoulder pitch                        -> SSC-32 CH1
    J2 - elbow pitch                           -> SSC-32 CH2
    J3 - wrist pitch                           -> SSC-32 CH3
    gripper (open/close only)                  -> SSC-32 CH4

After peeling off the base rotation J0, the shoulder/elbow/wrist are all
pitches about a shared horizontal axis -> IK is a planar 3R problem in the
(r, z) half-plane.

                           z
                           ^
                           |                * tip
                           |               /
                           |              * wrist
                           |             /
                           |   L2 ______/   (elbow joint)
                           |  /
                           | /  L1
                           |/
                 base  ----+---------> r  (horizontal distance from base axis)
                   (shoulder joint sits BASE_H above the base plate)

LINK LENGTHS (millimetres). Nominal Lynxmotion AL5B values from the wiki.
Measure your own arm and edit these before trusting Cartesian moves.
    https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-v1/ses-v1-robots/ses-v1-arms/al5b/
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from ssc32_driver import (
    CH_BASE, CH_SHOULDER, CH_ELBOW, CH_WRIST, CH_GRIPPER,
)


# ---------------------------------------------------------------------------
# Geometry (millimetres). Edit after measuring the actual arm.
# ---------------------------------------------------------------------------
BASE_H = 70.0
L1 = 146.0     # shoulder -> elbow
L2 = 187.0     # elbow -> wrist
L3 = 100.0     # wrist -> gripper tip (tool length)


# ---------------------------------------------------------------------------
# Servo-angle mapping.
#   zero_pulse: SSC-32 pulse (us) for joint angle = 0 rad.
#   us_per_rad: signed scale mapping angle (rad) to pulse delta.
# Signs depend on how each servo is mounted. Verify with slider tests first.
# ---------------------------------------------------------------------------
@dataclass
class JointCal:
    channel: int
    zero_pulse: int
    us_per_rad: float   # positive = CCW when viewed from +axis of rotation


DEFAULT_CAL: Dict[str, JointCal] = {
    "base":     JointCal(CH_BASE,     1500,  600.0 / math.radians(90)),
    "shoulder": JointCal(CH_SHOULDER, 1500, -700.0 / math.radians(90)),
    "elbow":    JointCal(CH_ELBOW,    1500, -700.0 / math.radians(90)),
    "wrist":    JointCal(CH_WRIST,    1500,  700.0 / math.radians(90)),
}

GRIPPER_OPEN_US = 2000
GRIPPER_CLOSE_US = 1200


# ---------------------------------------------------------------------------
# Joint-angle <-> pulse conversion
# ---------------------------------------------------------------------------
def angle_to_pulse(joint: str, angle_rad: float,
                   cal: Dict[str, JointCal] = DEFAULT_CAL) -> int:
    c = cal[joint]
    return int(round(c.zero_pulse + c.us_per_rad * angle_rad))


def pulse_to_angle(joint: str, pulse: int,
                   cal: Dict[str, JointCal] = DEFAULT_CAL) -> float:
    c = cal[joint]
    return (pulse - c.zero_pulse) / c.us_per_rad


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------
@dataclass
class JointAngles:
    base: float = 0.0
    shoulder: float = 0.0
    elbow: float = 0.0
    wrist: float = 0.0

    def as_dict(self) -> Dict[str, float]:
        return dict(base=self.base, shoulder=self.shoulder,
                    elbow=self.elbow, wrist=self.wrist)


def forward_kinematics(ja: JointAngles,
                       l1: float = L1, l2: float = L2, l3: float = L3,
                       base_h: float = BASE_H) -> Tuple[float, float, float, float]:
    """Return (x, y, z, pitch) of the gripper tip in mm / rad.

    ``pitch`` is the absolute angle of the tool axis relative to the horizontal
    plane (positive = tool pointing up).
    """
    q1, q2, q3, q4 = ja.shoulder, ja.elbow, ja.wrist, ja.base
    r = (l1 * math.cos(q1)
         + l2 * math.cos(q1 + q2)
         + l3 * math.cos(q1 + q2 + q3))
    z = (base_h
         + l1 * math.sin(q1)
         + l2 * math.sin(q1 + q2)
         + l3 * math.sin(q1 + q2 + q3))
    x = r * math.cos(q4)
    y = r * math.sin(q4)
    pitch = q1 + q2 + q3
    return x, y, z, pitch


# ---------------------------------------------------------------------------
# Inverse kinematics (geometric, closed-form)
# ---------------------------------------------------------------------------
def inverse_kinematics(x: float, y: float, z: float, pitch: float,
                       elbow_up: bool = True,
                       l1: float = L1, l2: float = L2, l3: float = L3,
                       base_h: float = BASE_H) -> Optional[JointAngles]:
    """Solve IK for target tip pose (x, y, z, tool_pitch).

    Returns None if the target is unreachable.
    """
    # 1. Base yaw.
    q_base = math.atan2(y, x)
    r = math.hypot(x, y)

    # 2. Wrist position in the (r, z) plane, stepping back from the tool tip.
    r_w = r - l3 * math.cos(pitch)
    z_w = z - base_h - l3 * math.sin(pitch)

    # 3. 2-link planar IK for shoulder + elbow.
    d_sq = r_w * r_w + z_w * z_w
    d = math.sqrt(d_sq)
    eps = 1e-6
    if d > (l1 + l2) + eps or d < abs(l1 - l2) - eps:
        return None

    cos_elbow = (d_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    sin_elbow = math.sqrt(1.0 - cos_elbow * cos_elbow)
    if not elbow_up:
        sin_elbow = -sin_elbow
    q_elbow = math.atan2(sin_elbow, cos_elbow)

    q_shoulder = (math.atan2(z_w, r_w)
                  - math.atan2(l2 * sin_elbow, l1 + l2 * cos_elbow))

    # 4. Wrist closes the chain so the tool lands at the requested pitch.
    q_wrist = pitch - q_shoulder - q_elbow

    return JointAngles(base=q_base, shoulder=q_shoulder,
                       elbow=q_elbow, wrist=q_wrist)


# ---------------------------------------------------------------------------
# Convenience: angles -> {channel: pulse} for SSC32.move_many()
# ---------------------------------------------------------------------------
def angles_to_pulses(ja: JointAngles,
                     cal: Dict[str, JointCal] = DEFAULT_CAL) -> Dict[int, int]:
    return {
        cal["base"].channel:     angle_to_pulse("base",     ja.base,     cal),
        cal["shoulder"].channel: angle_to_pulse("shoulder", ja.shoulder, cal),
        cal["elbow"].channel:    angle_to_pulse("elbow",    ja.elbow,    cal),
        cal["wrist"].channel:    angle_to_pulse("wrist",    ja.wrist,    cal),
    }


def reachable(x: float, y: float, z: float, pitch: float) -> bool:
    return inverse_kinematics(x, y, z, pitch) is not None


if __name__ == "__main__":
    # Round-trip self-check.
    import itertools
    print("AL5B  L1=" + str(L1) + "  L2=" + str(L2)
          + "  L3=" + str(L3) + "  base_h=" + str(BASE_H))
    for q1, q2, q3 in itertools.product([-0.3, 0.0, 0.4], repeat=3):
        ja = JointAngles(base=0.2, shoulder=q1, elbow=q2, wrist=q3)
        x, y, z, pitch = forward_kinematics(ja)
        sol = inverse_kinematics(x, y, z, pitch)
        if sol is None:
            print("  unreachable q=" + str((q1, q2, q3)))
            continue
        x2, y2, z2, _ = forward_kinematics(sol)
        err = ((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2) ** 0.5
        line = "  q=({:+.2f},{:+.2f},{:+.2f})  tip=({:+7.1f},{:+7.1f},{:+7.1f})  err={:.3f} mm"
        print(line.format(q1, q2, q3, x, y, z, err))
