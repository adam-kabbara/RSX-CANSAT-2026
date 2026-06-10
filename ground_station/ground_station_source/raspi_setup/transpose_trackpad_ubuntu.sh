#!/bin/bash

# Add this line to `crontab -e` to run this script at startup:
# @reboot sleep 30 && /home/rsx/ground_station_source/raspi_setup/transpose_trackpad_ubuntu.sh

# Run the trackpad orientation script after the graphical session is available.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="$SCRIPT_DIR/transpose_trackpad.py"
LOGFILE="/home/rsx/transpose_trackpad.log"

echo "[$(date)] transpose_trackpad_ubuntu.sh starting" >> "$LOGFILE"

# Use the current X11 session if available. The Python script depends on xinput.
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/rsx/.Xauthority}"

# Wait for the desktop session to be ready before calling xinput through Python.
for _ in $(seq 1 30); do
    if command -v xinput >/dev/null 2>&1 && xinput list >/dev/null 2>&1; then
        break
    fi
    sleep 2
done

if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "[$(date)] Python script not found: $PYTHON_SCRIPT" >> "$LOGFILE"
    exit 1
fi

python3 "$PYTHON_SCRIPT" >> "$LOGFILE" 2>&1
EXIT_CODE=$?

echo "[$(date)] transpose_trackpad_ubuntu.sh finished with exit code $EXIT_CODE" >> "$LOGFILE"
exit "$EXIT_CODE"