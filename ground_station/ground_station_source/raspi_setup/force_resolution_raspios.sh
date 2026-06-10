#!/bin/bash
# In order to get the pi to work with a decent resolution on the monitor setup we have
# we must force the resolution. This script is setup to run on startup after the 
# graphical environment is loaded. 
# It is important to ensure that the monitor is plugged in before the pi boots
# otherwise the pi will not detect the monitor and will not be able to set the resolution

export XDG_RUNTIME_DIR=/run/user/$(id -u)
WAYLAND_DISPLAY="wayland-0" wlr-randr
WAYLAND_DISPLAY="wayland-0" wlr-randr --output HDMI-A-1 --custom-mode 1600x900@65Hz