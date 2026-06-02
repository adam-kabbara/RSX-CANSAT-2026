#!/bin/bash

# Log file so we can debug if needed
LOGFILE=/home/rsx/force_resolution.log

echo "[$(date)] force_resolution.sh starting" >> "$LOGFILE"

# Use the same DISPLAY as your desktop. If in doubt, check `echo $DISPLAY` in a terminal.
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=/home/rsx/.Xauthority

HDMI_OUTPUT=""

# Wait up to ~90 seconds for ANY HDMI output to show as connected
for i in {1..45}; do
    HDMI_OUTPUT=$(xrandr | awk '/HDMI-[0-9]+ connected/ {print $1; exit}')
    if [ -n "$HDMI_OUTPUT" ]; then
        echo "[$(date)] Detected connected HDMI output: $HDMI_OUTPUT" >> "$LOGFILE"
        break
    fi
    echo "[$(date)] No HDMI connected yet, retry $i" >> "$LOGFILE"
    sleep 2
done

if [ -z "$HDMI_OUTPUT" ]; then
    echo "[$(date)] No connected HDMI output found, giving up." >> "$LOGFILE"
    exit 1
fi

# define custom mode 
xrandr --newmode "1280x960_60.00"  102.25  1280 1376 1488 1728  960 963 968 992 -hsync +vsync>>"$LOGFILE"
# add it to HDMI
xrandr --addmode "$HDMI_OUTPUT" "1280x960_60.00" 2>/dev/null
xrandr --output "$HDMI_OUTPUT" --mode "1280x960_60.00" --primary
