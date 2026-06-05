#!/bin/bash

# Add this line to `crontab -e` to run this script at startup:
# @reboot sleep 30 && /home/rsx/ground_station_source/raspi_setup/fix_touchscreen_ubuntu.sh

export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/rsx/.Xauthority}"

LOGFILE="${LOGFILE:-/home/rsx/fix_touchscreen.log}"

echo "[$(date)] fix_touchscreen_ubuntu.sh starting" >> "$LOGFILE"

for _ in $(seq 1 30); do
	if command -v xinput >/dev/null 2>&1 && xinput list >/dev/null 2>&1; then
		break
	fi
	sleep 2
done

if ! command -v xinput >/dev/null 2>&1; then
	echo "[$(date)] xinput not found" >> "$LOGFILE"
	exit 1
fi

if ! xinput list >/dev/null 2>&1; then
	echo "[$(date)] X11 session not ready for xinput" >> "$LOGFILE"
	exit 1
fi

xinput map-to-output "wch.cn USB2IIC_CTP_CONTROL" HDMI-2 >> "$LOGFILE" 2>&1
EXIT_CODE=$?

echo "[$(date)] fix_touchscreen_ubuntu.sh finished with exit code $EXIT_CODE" >> "$LOGFILE"
exit "$EXIT_CODE"