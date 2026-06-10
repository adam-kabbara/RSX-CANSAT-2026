#!/bin/bash

# Add this line to `crontab -e` to run this script at startup:
# @reboot sleep 30 && /home/rsx/lcd_ubuntu.sh

export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_ACTIVATE="${VENV_ACTIVATE:-/home/rsx/code/temp/bin/activate}"
PYTHON_SCRIPT="${PYTHON_SCRIPT:-$SCRIPT_DIR/code/lcd.py}"
LOGFILE="${LOGFILE:-/home/rsx/lcd_ubuntu.log}"

{
    echo "[$(date)] lcd_ubuntu.sh starting"

    if [ ! -f "$VENV_ACTIVATE" ]; then
        echo "[$(date)] virtualenv activate file not found: $VENV_ACTIVATE"
        exit 1
    fi

    if [ ! -f "$PYTHON_SCRIPT" ]; then
        echo "[$(date)] Python script not found: $PYTHON_SCRIPT"
        exit 1
    fi

    # shellcheck disable=SC1090
    source "$VENV_ACTIVATE"

    python "$PYTHON_SCRIPT"
    EXIT_CODE=$?

    echo "[$(date)] lcd_ubuntu.sh finished with exit code $EXIT_CODE"
    exit "$EXIT_CODE"
} >> "$LOGFILE" 2>&1
