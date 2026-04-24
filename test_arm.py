"""
Standalone arm sanity-check — no Flask, no IK, no kinematics.

Use this BEFORE the webapp to prove:
    - the serial port is right
    - every channel is wired correctly
    - each servo actually moves when commanded
    - nothing crashes into itself

Workflow:

    python test_arm.py --port COM4          # Windows
    python test_arm.py --port /dev/ttyUSB0  # Linux

Modes:
    --center        send every channel to 1500 us and stop
    --wiggle        center, then nudge each servo +/- 80 us one at a time
    --sweep CH      slowly sweep one channel between 1300 and 1700 us
    --set CH PULSE  set a single channel to a specific pulse

Examples:
    python test_arm.py --port COM4 --center
    python test_arm.py --port COM4 --wiggle
    python test_arm.py --port COM4 --sweep 0
    python test_arm.py --port COM4 --set 5 2000
"""

from __future__ import annotations

import argparse
import sys
import time

from ssc32_driver import SSC32


CHANNEL_NAMES = {
    0: "base",
    1: "shoulder",
    2: "elbow",
    3: "wrist",
    4: "gripper",
}


def center_all(arm: SSC32, time_ms: int = 2000) -> None:
    print("[test] centering all channels at 1500 us ...")
    neutral = {ch: 1500 for ch in range(5)}
    arm.move_many(neutral, time_ms=time_ms)
    time.sleep(time_ms / 1000.0 + 0.3)
    print("[test] done. Arm should be in neutral pose.")


def wiggle(arm: SSC32, delta: int = 80, hold_s: float = 0.5) -> None:
    center_all(arm, time_ms=1800)
    for ch in range(5):
        name = CHANNEL_NAMES.get(ch, f"ch{ch}")
        print(f"[test] channel {ch} ({name}):  1500 -> {1500 + delta} -> {1500 - delta} -> 1500")
        arm.move(ch, 1500 + delta, time_ms=500)
        time.sleep(hold_s + 0.5)
        arm.move(ch, 1500 - delta, time_ms=500)
        time.sleep(hold_s + 0.5)
        arm.move(ch, 1500, time_ms=500)
        time.sleep(hold_s + 0.5)
    print("[test] wiggle complete. All channels returned to 1500.")


def sweep(arm: SSC32, ch: int, low: int = 1300, high: int = 1700,
          step: int = 20, dwell_s: float = 0.05) -> None:
    name = CHANNEL_NAMES.get(ch, f"ch{ch}")
    print(f"[test] sweep channel {ch} ({name}) from {low} to {high} us ...")
    arm.move(ch, 1500, time_ms=500)
    time.sleep(0.6)
    try:
        # low -> high
        for p in range(low, high + 1, step):
            arm.move(ch, p, time_ms=60)
            time.sleep(dwell_s)
        # high -> low
        for p in range(high, low - 1, -step):
            arm.move(ch, p, time_ms=60)
            time.sleep(dwell_s)
    finally:
        arm.move(ch, 1500, time_ms=500)
        time.sleep(0.6)
    print("[test] sweep done. Channel returned to 1500.")


def set_one(arm: SSC32, ch: int, pulse: int, time_ms: int = 800) -> None:
    name = CHANNEL_NAMES.get(ch, f"ch{ch}")
    print(f"[test] channel {ch} ({name}) -> {pulse} us over {time_ms} ms")
    arm.move(ch, pulse, time_ms=time_ms)
    time.sleep(time_ms / 1000.0 + 0.3)


def main():
    ap = argparse.ArgumentParser(description="AL5B arm sanity-check script")
    ap.add_argument("--port", required=True,
                    help="Serial port (COM4 on Windows, /dev/ttyUSB0 on Linux)")
    ap.add_argument("--baud", type=int, default=115200)
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--center", action="store_true", help="center every channel at 1500 us")
    group.add_argument("--wiggle", action="store_true", help="center, then nudge every channel +/- 80 us")
    group.add_argument("--sweep", type=int, metavar="CH", help="sweep one channel 1300<->1700 us")
    group.add_argument("--set", nargs=2, type=int, metavar=("CH", "PULSE"),
                       help="send one channel to a specific pulse")
    args = ap.parse_args()

    arm = SSC32(port=args.port, baudrate=args.baud)
    try:
        arm.open()
    except Exception as e:
        print(f"ERROR: could not open {args.port} at {args.baud} baud: {e}")
        sys.exit(1)

    try:
        if args.center:
            center_all(arm)
        elif args.wiggle:
            wiggle(arm)
        elif args.sweep is not None:
            if not (0 <= args.sweep <= 4):
                print("channel must be 0..4")
                sys.exit(1)
            sweep(arm, args.sweep)
        elif args.set is not None:
            ch, pulse = args.set
            if not (0 <= ch <= 4):
                print("channel must be 0..4")
                sys.exit(1)
            if not (500 <= pulse <= 2500):
                print("pulse must be 500..2500 us")
                sys.exit(1)
            set_one(arm, ch, pulse)
    finally:
        arm.close()


if __name__ == "__main__":
    main()
