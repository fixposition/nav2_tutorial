#!/bin/bash
set -e
modprobe gs_usb can-utils
ip link set can0 up type can bitrate 500000
