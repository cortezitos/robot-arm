# AL5B Arm Project — Raspberry Pi Build

Control of the Lynxmotion **AL5B** robotic arm via the **SSC-32 / SSC-32U**
servo controller from a **Raspberry Pi** (Pi 3 / Pi 4 / Pi 5 / Pi Zero 2 W,
anything running Raspberry Pi OS / Debian 11 or 12).

This folder is the **Pi port** of `../al5b_project/`. The Python code is
identical — pyserial behaves the same on the Pi as on a PC — so the only
real differences are a few Pi-flavoured defaults, helper shell scripts, and
a systemd unit file for auto-start on boot.

## What's new compared to the PC build

| File                | Purpose                                                                      |
|---------------------|------------------------------------------------------------------------------|
| `setup_pi.sh`       | one-command installer (pip deps, `dialout` group, udev rule for `/dev/ssc32`)|
| `run_pi.sh`         | launcher that auto-picks `/dev/ssc32` or `/dev/ttyUSB0`                      |
| `al5b.service`      | systemd unit for auto-start on boot                                          |
| `webapp.py`         | default `--port` is `/dev/ttyUSB0`, default `--host` is `0.0.0.0`            |

## Hardware wiring (5 channels)

| SSC-32 Channel | Joint          | Notes                                   |
|:--------------:|:---------------|:----------------------------------------|
| CH0            | Base rotation  |                                         |
| CH1            | Shoulder pitch |                                         |
| CH2            | Elbow pitch    |                                         |
| CH3            | Wrist pitch    | Single wrist (no wrist rotate on AL5B)  |
| CH4            | Gripper        | 1200 us ~ closed, 2000 us ~ open (tune) |

## Power (read this before plugging in)

**Do NOT power the servos from the Pi's 5 V rail.** Five servos on a single
arm can peak well past 2 A when they move together, which will brown-out the
Pi and corrupt the SD card. Power the SSC-32's **VS1 / VS2** servo rail from
a dedicated **6 V supply** (the XL4015 buck converter in the parts kit works;
set it to 6.0 V with a multimeter BEFORE wiring it up). The SSC-32's **VL**
logic rail can be fed from the Pi's 5 V pin or from its own USB-powered
regulator — just don't bridge VL to VS.

## One-command install

On a fresh Raspberry Pi OS install:

    git clone <your-repo-url> ~/al5b_pi_project
    cd ~/al5b_pi_project
    bash setup_pi.sh

This installs pip + Flask + pyserial, adds your user to the `dialout` group
(required to read/write `/dev/ttyUSB0`), and drops a udev rule so the SSC-32
always appears as `/dev/ssc32` no matter how many other USB-serial adapters
are plugged in. Log out and back in afterwards so the group change applies.

## Recommended startup workflow

### Step 1 — sanity-check every channel BEFORE touching the webapp

    python3 test_arm.py --port /dev/ttyUSB0 --center
    python3 test_arm.py --port /dev/ttyUSB0 --wiggle

`--wiggle` steps through CH0..CH4 and moves each one
`1500 -> 1580 -> 1420 -> 1500`. Watch which physical joint responds on each
channel — that proves your wiring.

### Step 2 — run the web UI

    ./run_pi.sh                         # on the arm
    ./run_pi.sh --fake                  # on a Pi with no arm plugged in
    python3 webapp.py --port /dev/ssc32 # direct, equivalent to run_pi.sh

Startup sequence (safe by design):

1. Opens the serial port.
2. Centres every channel at **1500 us** and holds for 2 seconds.
3. Does a wake-up wiggle (`--no-wiggle` disables this).
4. Holds at 1500 until you move a slider in the browser.

Find the Pi's IP:

    hostname -I

Then on your phone, open `http://<pi-ip>:5000/`. The `--host 0.0.0.0`
default lets any device on the same Wi-Fi reach the webapp.

### Step 3 — calibrate

Before the IK-based **Home** and **Pick & Place** buttons work correctly:

1. **Servo directions.** Move each slider. Flip the sign of `us_per_rad` in
   `al5b_kinematics.py → DEFAULT_CAL` for any joint that moves the wrong way.
2. **Zero positions.** With each slider at 1500 us, the arm should be in a
   canonical pose (base centred, shoulder vertical, elbow horizontal forward,
   wrist horizontal). Update `zero_pulse` for any joint that isn't.
3. **Link lengths.** Measure `L1`, `L2`, `L3`, `BASE_H` on the physical arm.
   Edit the constants at the top of `al5b_kinematics.py`.
4. **Gripper range.** Close the gripper slowly, find the pulse where the jaws
   just meet without binding. Open fully, record that pulse. Update
   `GRIPPER_OPEN_US` / `GRIPPER_CLOSE_US`.

Until calibration matches your arm, the **per-joint sliders** are the safe
interface. The IK-based buttons may drive the arm to physically wrong poses.

## Auto-start on boot (optional)

Once you're happy with the calibration and want the webapp to come up
automatically every time the Pi boots:

    sudo cp al5b.service /etc/systemd/system/al5b.service
    # edit User= and WorkingDirectory= if your user/path differs from /home/pi
    sudo systemctl daemon-reload
    sudo systemctl enable al5b
    sudo systemctl start al5b

Check it's running:

    systemctl status al5b
    journalctl -u al5b -f        # live log

Disable:

    sudo systemctl disable al5b
    sudo systemctl stop al5b

## UI buttons

- **Center all @ 1500** — re-issues 1500 us on every channel. Always safe.
- **Home (IK)** — uses inverse kinematics to reach the `HOME_POSE` defined in
  `trajectory.py`. Requires calibration to be correct.
- **Pick & Place** — runs the scripted pick-and-place trajectory from
  `trajectory.pick_and_place()`. Requires calibration.
- **STOP** — sends the SSC-32 native `STOP <ch>` per channel. The arm freezes
  **at its current position** — no snap back to the last target.

## Troubleshooting

**`/dev/ttyUSB0: Permission denied`**

You're not in the `dialout` group yet. Run `groups` — if `dialout` isn't in
the output, `sudo usermod -a -G dialout $USER` then log out and back in.

**`/dev/ttyUSB0` doesn't exist**

Unplug and re-plug the SSC-32 USB cable, then `dmesg | tail` — you should
see something like `ftdi_sio: new full speed USB device`. If you see a
different tty name (e.g. `ttyUSB1`), run `setup_pi.sh` once, which drops a
udev rule that pins the SSC-32 to `/dev/ssc32` regardless of plug order.

**Webapp works on `localhost:5000` but phone can't reach it**

Make sure the Pi and your phone are on the same Wi-Fi SSID (not the guest
network), and that the firewall on the Pi isn't blocking port 5000
(`sudo ufw status`). The `--host 0.0.0.0` default already binds the server
on all interfaces; you don't need to change anything in the code.

**Arm contracts or jerks on startup**

This was an uncalibrated-IK bug in an older build. In the current version
the webapp centres every channel at 1500 us before anything else, then waits
for you to move a slider — the IK never runs until you click **Home (IK)**
or **Pick & Place**, by which time your calibration edits have taken effect.

**STOP button slams the arm back to the last position**

You're on an old version. The fixed driver sends the SSC-32 native
`STOP <ch>` command, which freezes each servo at its current interpolated
position. If you're still seeing a snap, clear the Python bytecode cache
once (`rm -rf __pycache__`) and restart `webapp.py`.

## Files

- `ssc32_driver.py` — thin SSC-32 serial wrapper (thread-safe, with simulator)
- `al5b_kinematics.py` — FK + closed-form IK for the 4-DOF AL5B chain
- `trajectory.py` — waypoint follower and pick-and-place sequence
- `webapp.py` — Flask web server (sliders-only UI)
- `templates/index.html` — per-joint sliders + Home / Pick-and-Place / STOP
- `test_arm.py` — standalone per-channel sanity checker
- `requirements.txt` — Python dependencies
- `setup_pi.sh` — one-command Pi installer
- `run_pi.sh` — convenience launcher
- `al5b.service` — systemd unit for auto-start
- `docs/report_outline.md` — report skeleton

## Moving back to a PC

The code in this folder is byte-for-byte the same logic as `../al5b_project/`,
just with Pi-friendly defaults. If you want the PC build, use that folder
instead — the only meaningful difference is `--port COM4` vs `/dev/ttyUSB0`.
