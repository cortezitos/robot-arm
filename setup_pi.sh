#!/usr/bin/env bash
#
# One-command installer for the AL5B Pi build.
#
#   curl -sS https://.../setup_pi.sh | bash
#   # or, after cloning the repo:
#   cd al5b_pi_project && bash setup_pi.sh
#
# What it does:
#   1. apt-installs python3-pip and python3-venv if missing
#   2. pip-installs the Python deps (Flask, pyserial)
#   3. adds the current user to the 'dialout' group (serial port access)
#   4. drops a udev rule so the SSC-32 is always /dev/ttyUSB0
#   5. prints next steps (log out and back in; then run ./run_pi.sh)
#
# This script is idempotent — you can re-run it safely.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"

echo "==> AL5B Pi setup"
echo "    working from: $HERE"
echo

# ---- 1. System packages --------------------------------------------------
if ! command -v pip3 >/dev/null 2>&1; then
    echo "==> installing python3-pip"
    sudo apt-get update
    sudo apt-get install -y python3-pip
else
    echo "==> python3-pip already installed"
fi

# ---- 2. Python deps ------------------------------------------------------
echo "==> installing Python dependencies"
pip3 install --user -r "$HERE/requirements.txt"

# ---- 3. Serial permissions ----------------------------------------------
if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    echo "==> $USER is already in 'dialout' group"
else
    echo "==> adding $USER to 'dialout' group (needed for /dev/ttyUSB0)"
    sudo usermod -a -G dialout "$USER"
    NEEDS_RELOG=1
fi

# ---- 4. udev rule (optional but nice) -----------------------------------
# The SSC-32 reports VID:PID 0403:6001 (FTDI). Pinning it to /dev/ttyUSB0
# means the default --port in webapp.py always hits the right device, even
# if you plug other FTDI adapters in later (they'll get ttyUSB1, ttyUSB2, ...).
UDEV_RULE="/etc/udev/rules.d/99-ssc32.rules"
if [ ! -f "$UDEV_RULE" ]; then
    echo "==> installing udev rule -> /dev/ssc32 symlink"
    echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ssc32"' \
        | sudo tee "$UDEV_RULE" >/dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
else
    echo "==> udev rule already present"
fi

echo
echo "==> done."
if [ "${NEEDS_RELOG:-0}" = "1" ]; then
    echo "    >>> log out and back in (or reboot) so the dialout group applies. <<<"
fi
echo "    then:  cd $HERE && ./run_pi.sh"
