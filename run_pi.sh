#!/usr/bin/env bash
#
# Convenience launcher for the AL5B webapp on the Raspberry Pi.
#
# Picks the right serial device automatically:
#   * prefers /dev/ssc32  (udev symlink, installed by setup_pi.sh)
#   * falls back to /dev/ttyUSB0
#   * any extra args are forwarded to webapp.py (e.g. --no-wiggle, --fake)
#
# Examples:
#   ./run_pi.sh
#   ./run_pi.sh --no-wiggle
#   ./run_pi.sh --fake
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

if [ -e /dev/ssc32 ]; then
    PORT=/dev/ssc32
elif [ -e /dev/ttyUSB0 ]; then
    PORT=/dev/ttyUSB0
else
    echo "ERROR: no SSC-32 serial device found." >&2
    echo "       Plug in the USB cable and run: ls /dev/ttyUSB*" >&2
    echo "       Or run with --fake to use the simulator." >&2
    exit 1
fi

echo "[run_pi] using port $PORT"
exec python3 webapp.py --port "$PORT" "$@"
