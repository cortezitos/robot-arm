"""
SSC-32 serial driver for the Lynxmotion AL5B robotic arm.

This build targets a 5-channel AL5B wiring:
    CH0 - base rotation
    CH1 - shoulder pitch
    CH2 - elbow pitch
    CH3 - wrist pitch
    CH4 - gripper

SSC-32 protocol (reference: Lynxmotion SSC-32U User Guide):
    Move servo:          #<ch> P<pulse> [S<speed>] [T<time_ms>] <CR>
    Multi-servo move:    #0 P1500 #1 P1800 ... T2000 <CR>
    Query position:      QP <ch> <CR>     -> single byte, pulse/10
    Query movement:      Q <CR>           -> '.' idle, '+' still moving
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

import serial  # pyserial


# ---------------------------------------------------------------------------
# AL5B channel map. Edit here if your team wired the servos differently.
# ---------------------------------------------------------------------------
CH_BASE = 0
CH_SHOULDER = 1
CH_ELBOW = 2
CH_WRIST = 3
CH_GRIPPER = 4

DEFAULT_CHANNELS = (CH_BASE, CH_SHOULDER, CH_ELBOW, CH_WRIST, CH_GRIPPER)

# Safe pulse window for hobby servos in microseconds.
PULSE_MIN = 700
PULSE_MAX = 2300
PULSE_CENTER = 1500


@dataclass
class ServoLimit:
    """Per-channel safe pulse range and centre."""
    ch: int
    pmin: int = PULSE_MIN
    pmax: int = PULSE_MAX
    center: int = PULSE_CENTER

    def clamp(self, pulse: int) -> int:
        return max(self.pmin, min(self.pmax, int(pulse)))


# Default per-channel limits. Tighten these during calibration.
DEFAULT_LIMITS: Dict[int, ServoLimit] = {
    CH_BASE:     ServoLimit(CH_BASE,      700, 2300, 1500),
    CH_SHOULDER: ServoLimit(CH_SHOULDER,  900, 2100, 1500),
    CH_ELBOW:    ServoLimit(CH_ELBOW,     700, 2300, 1500),
    CH_WRIST:    ServoLimit(CH_WRIST,     700, 2300, 1500),
    CH_GRIPPER:  ServoLimit(CH_GRIPPER,  1000, 2000, 1500),  # 1000=closed, 2000=open (tune)
}


class SSC32:
    """Thread-safe serial interface to an SSC-32 servo controller."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.5,
        limits: Optional[Dict[int, ServoLimit]] = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.limits = limits if limits is not None else dict(DEFAULT_LIMITS)
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._timeout = timeout

    # ---- connection -------------------------------------------------------

    def open(self) -> None:
        self._ser = serial.Serial(
            self.port, self.baudrate, timeout=self._timeout, write_timeout=self._timeout,
        )
        time.sleep(0.1)

    def close(self) -> None:
        if self._ser is not None:
            try:
                self.stop_all()
            except Exception:
                pass
            self._ser.close()
            self._ser = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # ---- raw IO -----------------------------------------------------------

    def _write(self, cmd: str) -> None:
        if self._ser is None:
            raise RuntimeError("Serial port is not open. Call open() first.")
        if not cmd.endswith("\r"):
            cmd += "\r"
        with self._lock:
            self._ser.write(cmd.encode("ascii"))
            self._ser.flush()

    def _read_until(self, terminator: bytes = b"\r", max_bytes: int = 64) -> bytes:
        if self._ser is None:
            raise RuntimeError("Serial port is not open.")
        buf = bytearray()
        deadline = time.time() + self._timeout
        while time.time() < deadline and len(buf) < max_bytes:
            b = self._ser.read(1)
            if not b:
                continue
            buf += b
            if terminator and b == terminator:
                break
        return bytes(buf)

    # ---- high level API ---------------------------------------------------

    def move(self, ch: int, pulse: int, time_ms: Optional[int] = None,
             speed_us_per_s: Optional[int] = None) -> None:
        limit = self.limits.get(ch, ServoLimit(ch))
        pulse = limit.clamp(pulse)
        cmd = "#" + str(ch) + " P" + str(pulse)
        if speed_us_per_s is not None:
            cmd += " S" + str(int(speed_us_per_s))
        if time_ms is not None:
            cmd += " T" + str(int(time_ms))
        self._write(cmd)

    def move_many(self, targets: Dict[int, int], time_ms: int = 1000,
                  speed_us_per_s: Optional[int] = None) -> None:
        if not targets:
            return
        parts = []
        for ch, pulse in targets.items():
            limit = self.limits.get(ch, ServoLimit(ch))
            parts.append("#" + str(ch) + " P" + str(limit.clamp(pulse)))
            if speed_us_per_s is not None:
                parts.append("S" + str(int(speed_us_per_s)))
        parts.append("T" + str(int(time_ms)))
        self._write(" ".join(parts))

    def center_all(self, channels: Iterable[int] = DEFAULT_CHANNELS,
                   time_ms: int = 2000) -> None:
        self.move_many({ch: self.limits.get(ch, ServoLimit(ch)).center
                        for ch in channels}, time_ms=time_ms)

    def stop_all(self, channels: Iterable[int] = DEFAULT_CHANNELS) -> None:
        """Cancel any in-progress move and HOLD each servo where it currently is.

        Uses the SSC-32 native ``STOP <ch>`` command, which is distinct from
        re-issuing the last target: STOP leaves the servo at its current
        interpolated position without snapping anywhere.

        Do NOT use ``move_many(..., time_ms=1)`` here — that tells the
        servo "go to the commanded target in 1 ms" and causes a violent
        snap to the target, which is the opposite of "stop where you are".
        """
        for ch in channels:
            try:
                self._write("STOP " + str(ch))
            except Exception:
                pass

    def is_moving(self) -> bool:
        self._write("Q")
        resp = self._read_until(b"", max_bytes=1)
        if not resp:
            return False
        return resp[:1] == b"+"

    def query_pulse(self, ch: int) -> Optional[int]:
        self._write("QP" + str(ch))
        resp = self._read_until(b"", max_bytes=1)
        if not resp:
            return None
        return int(resp[0]) * 10

    def wait_until_idle(self, poll_s: float = 0.05, timeout_s: float = 10.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if not self.is_moving():
                return True
            time.sleep(poll_s)
        return False


# ---------------------------------------------------------------------------
# Simulation shim — lets the UI and kinematics run without the arm plugged in.
# ---------------------------------------------------------------------------
class FakeSSC32(SSC32):
    """In-memory stand-in that accepts the same API but prints commands."""

    def __init__(self, limits: Optional[Dict[int, ServoLimit]] = None, verbose: bool = True):
        self.port = "<fake>"
        self.baudrate = 0
        self.limits = limits if limits is not None else dict(DEFAULT_LIMITS)
        self._state: Dict[int, int] = {ch: lim.center for ch, lim in self.limits.items()}
        self._verbose = verbose

    def open(self): return None
    def close(self): return None

    def _write(self, cmd: str) -> None:
        if self._verbose:
            print("[FakeSSC32] " + cmd.strip())

    def move(self, ch, pulse, time_ms=None, speed_us_per_s=None):
        limit = self.limits.get(ch, ServoLimit(ch))
        self._state[ch] = limit.clamp(pulse)
        if self._verbose:
            extra = ""
            if time_ms is not None: extra += " T" + str(time_ms)
            if speed_us_per_s is not None: extra += " S" + str(speed_us_per_s)
            print("[FakeSSC32] #" + str(ch) + " P" + str(self._state[ch]) + extra)

    def move_many(self, targets, time_ms=1000, speed_us_per_s=None):
        for ch, p in targets.items():
            limit = self.limits.get(ch, ServoLimit(ch))
            self._state[ch] = limit.clamp(p)
        if self._verbose:
            parts = " ".join("#" + str(c) + " P" + str(self._state[c]) for c in targets)
            print("[FakeSSC32] " + parts + " T" + str(time_ms))

 