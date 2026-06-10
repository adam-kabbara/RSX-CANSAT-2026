#!/bin/bash

# Add this line to `crontab -e` to run this script at startup:
# @reboot sleep 30 && /home/rsx/cansat_gui_ubuntu.sh

# Cron starts with a minimal environment, so set the basics explicitly.
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# Source the virtual environment if your Python dependencies are installed there.
source ~/RSX-CANSAT-2026/Software/ground_station_source/genv/bin/activate

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Change this to the Python script you want to run at boot.
PYTHON_SCRIPT="${PYTHON_SCRIPT:-$SCRIPT_DIR/RSX-CANSAT-2026/Software/ground_station_source/code/main.py}"
LOGFILE="${LOGFILE:-/home/rsx/cansat_gui_ubuntu.log}"
PYTHON_BIN="${PYTHON_BIN:-$(command -v python3 2>/dev/null)}"

{
    echo "[$(date)] cansat_gui_ubuntu.sh starting"

    if [ -z "$PYTHON_BIN" ]; then
        echo "[$(date)] python3 not found in PATH"
        exit 1
    fi

    if [ ! -f "$PYTHON_SCRIPT" ]; then
        echo "[$(date)] Python script not found: $PYTHON_SCRIPT"
        exit 1
    fi

    # Use the current X11 session if the script needs desktop access.
    export DISPLAY="${DISPLAY:-:0}"
    export XAUTHORITY="${XAUTHORITY:-/home/rsx/.Xauthority}"

    # Wait for the graphical session to be ready before launching Python.
    for _ in $(seq 1 30); do
        if command -v xinput >/dev/null 2>&1 && xinput list >/dev/null 2>&1; then
            break
        fi
        sleep 2
    done

    if ! command -v xinput >/dev/null 2>&1; then
        echo "[$(date)] xinput not found"
        exit 1
    fi

    if ! xinput list >/dev/null 2>&1; then
        echo "[$(date)] X11 session not ready for xinput"
        exit 1
    fi

    "$PYTHON_BIN" "$PYTHON_SCRIPT"
    EXIT_CODE=$?

    echo "[$(date)] cansat_gui_ubuntu.sh finished with exit code $EXIT_CODE"
    exit "$EXIT_CODE"
} >> "$LOGFILE" 2>&1
