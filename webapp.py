"""
Flask web UI for the AL5B arm — Raspberry Pi build.

This is the Pi port of the PC/Debian/Windows version. The Python is identical
to al5b_project/webapp.py; the only differences are:

  * The default --port is /dev/ttyUSB0 (the SSC-32 always shows up there on
    a stock Raspberry Pi OS install with exactly one USB serial device).
  * The default --host is 0.0.0.0 so a phone on the same Wi-Fi as the Pi
    can open http://<pi-ip>:5000/.

Everything else — FakeSSC32 simulator, sliders-only UI, pick-and-place demo
thread, STOP-in-place command — is unchanged.

Endpoints:
    GET  /                -> HTML page (per-joint sliders + buttons)
    GET  /api/state       -> current last-commanded pulse per channel + busy flag
    POST /api/joint       -> {channel, pulse, time_ms}   direct single-servo move
    POST /api/center      -> centre every channel at 1500 us
    POST /api/home        -> IK-based home pose (uses kinematics)
    POST /api/demo        -> kick off pick-and-place in a background thread
    POST /api/stop        -> emergency freeze (SSC-32 native STOP per channel)

Threads:
    - Flask request threads (one per HTTP request).
    - Demo thread (spawned on-demand for pick-and-place).

Run on the Pi:
    python3 webapp.py                            # uses /dev/ttyUSB0
    python3 webapp.py --port /dev/ttyUSB1        # override if needed
    python3 webapp.py --fake                     # simulator, no hardware
"""

from __future__ import annotations

import argparse
import threading
import time
from typing import Dict, Optional

from flask import Flask, jsonify, render_template, request

from ssc32_driver import (
    SSC32, FakeSSC32,
    CH_BASE, CH_SHOULDER, CH_ELBOW, CH_WRIST, CH_GRIPPER,
    DEFAULT_CHANNELS,
)
from al5b_kinematics import (
    inverse_kinematics, angles_to_pulses,
)
from trajectory import pick_and_place, HOME_POSE


# ---------------------------------------------------------------------------
# Controller — thin shared state for the demo thread + status endpoint.
# ---------------------------------------------------------------------------
class ArmController:
    def __init__(self, arm: SSC32):
        self.arm = arm
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._busy = threading.Event()
        # Track last-commanded pulse per channel (for the state endpoint).
        self.last_pulses: Dict[int, int] = {ch: 1500 for ch in DEFAULT_CHANNELS}

    # ---- basic motion ----------------------------------------------------

    def move_channel(self, ch: int, pulse: int, time_ms: int = 300) -> None:
        with self._lock:
            self.arm.move(ch, pulse, time_ms=time_ms)
            self.last_pulses[ch] = pulse

    def move_many(self, targets: Dict[int, int], time_ms: int = 1000) -> None:
        with self._lock:
            self.arm.move_many(targets, time_ms=time_ms)
            for ch, p in targets.items():
                self.last_pulses[ch] = p

    def center_all(self, time_ms: int = 1500) -> None:
        self.move_many({ch: 1500 for ch in DEFAULT_CHANNELS}, time_ms=time_ms)

    def emergency_stop(self) -> None:
        self._stop.set()
        try:
            self.arm.stop_all()
        except Exception as e:
            print("[stop] error:", e)

    # ---- kinematics-based helpers ---------------------------------------

    def move_to_cartesian(self, x: float, y: float, z: float, pitch: float,
                          gripper_us: Optional[int], time_ms: int = 2000) -> bool:
        sol = inverse_kinematics(x, y, z, pitch)
        if sol is None:
            return False
        targets = angles_to_pulses(sol)
        if gripper_us is not None:
            targets[CH_GRIPPER] = int(gripper_us)
        self.move_many(targets, time_ms=time_ms)
        return True

    # ---- demo thread -----------------------------------------------------

    def start_pick_and_place(self) -> bool:
        if self._busy.is_set():
            return False
        t = threading.Thread(target=self._run_pick_and_place, daemon=True)
        t.start()
        return True

    def _run_pick_and_place(self):
        self._busy.set()
        self._stop.clear()
        try:
            pick_and_place(self.arm, stop_event=self._stop)
        finally:
            self._busy.clear()


# ---------------------------------------------------------------------------
# Flask app
# ---------------------------------------------------------------------------
def create_app(controller: ArmController) -> Flask:
    app = Flask(__name__, template_folder="templates", static_folder="static")

    @app.get("/")
    def index():
        html = render_template("index.html")
        # Tell the browser not to cache the UI — otherwise a stale tab keeps
        # firing POSTs to endpoints we've removed (e.g. /api/cmd from the
        # previous joystick build) and spams 404s in the log.
        resp = app.make_response(html)
        resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
        resp.headers["Pragma"] = "no-cache"
        resp.headers["Expires"] = "0"
        return resp

    @app.get("/favicon.ico")
    def favicon():
        # Silence the browser's automatic favicon probe.
        return ("", 204)

    @app.post("/api/cmd")
    def api_cmd_stub():
        # Legacy endpoint from the joystick build. Kept as a no-op so any
        # stale browser tab doesn't flood the log with 404s. Modern UI
        # uses /api/joint for per-channel control.
        return ("", 204)

    @app.get("/api/state")
    def api_state():
        return jsonify({
            "busy": controller._busy.is_set(),
            "pulses": controller.last_pulses,
        })

    @app.post("/api/joint")
    def api_joint():
        """Direct per-joint control.

        JSON: {"channel": int, "pulse": int, "time_ms": int}
        """
        data = request.get_json(force=True, silent=True) or {}
        try:
            ch = int(data["channel"])
            pulse = int(data["pulse"])
        except (KeyError, ValueError, TypeError):
            return jsonify({"error": "channel and pulse required"}), 400
        t_ms = int(data.get("time_ms", 300))
        controller.move_channel(ch, pulse, time_ms=t_ms)
        return ("", 204)

    @app.post("/api/center")
    def api_center():
        controller.center_all(time_ms=1500)
        return ("", 204)

    @app.post("/api/home")
    def api_home():
        """Move to an IK-based home pose (uses calibration)."""
        ok = controller.move_to_cartesian(HOME_POSE.x, HOME_POSE.y, HOME_POSE.z,
                                          HOME_POSE.pitch, HOME_POSE.gripper_us,
                                          time_ms=2000)
        return jsonify({"ok": ok})

    @app.post("/api/demo")
    def api_demo():
        started = controller.start_pick_and_place()
        return jsonify({"started": started})

    @app.post("/api/stop")
    def api_stop():
        controller.emergency_stop()
        return ("", 204)

    return app


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="Serial port (default /dev/ttyUSB0 on the Pi)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--host", default="0.0.0.0",
                    help="HTTP bind host (0.0.0.0 lets phones on the LAN connect)")
    ap.add_argument("--http-port", type=int, default=5000)
    ap.add_argument("--fake", action="store_true",
                    help="Use simulator instead of real hardware")
    ap.add_argument("--no-wiggle", action="store_true",
                    help="Skip the wake-up wiggle on startup")
    args = ap.parse_args()

    if args.fake:
        print("Using FakeSSC32 simulator. Drop --fake to talk to the real arm.")
        arm = FakeSSC32(verbose=False)
    else:
        print("[startup] opening " + args.port + " @ " + str(args.baud) + " baud ...")
        arm = SSC32(port=args.port, baudrate=args.baud)
    arm.open()

    # ---- Safe startup ----------------------------------------------------
    # Step 1: centre every channel at 1500 us (no IK).
    print("[startup] centering all servos at 1500 us ...")
    try:
        neutral = {ch: 1500 for ch in DEFAULT_CHANNELS}
        arm.move_many(neutral, time_ms=2000)
        time.sleep(2.2)
    except Exception as e:
        print("[startup] centre-all error:", e)

    # Step 2: optional wake-up wiggle.
    if not args.no_wiggle:
        print("[startup] wake-up wiggle: each channel 1500 -> 1580 -> 1500 ...")
        try:
            for ch in DEFAULT_CHANNELS:
                arm.move(ch, 1580, time_ms=400)
                time.sleep(0.55)
                arm.move(ch, 1500, time_ms=400)
                time.sleep(0.55)
        except Exception as e:
            print("[startup] wiggle error:", e)

    listen_host = args.host
    if listen_host == "0.0.0.0":
        listen_host = "<pi-ip>"
    print("[startup] ready. Open http://" + listen_host + ":" + str(args.http_port) + "/ in a browser.")
    print("[startup] the arm will HOLD at 1500 until you move a slider.")

    controller = ArmController(arm)
    app = create_app(controller)
    try:
        app.run(host=args.host, port=args.http_port, debug=False, threaded=True)
    finally:
        controller.emergency_stop()
        arm.close()


if __name__ == "__main__":
    main()
