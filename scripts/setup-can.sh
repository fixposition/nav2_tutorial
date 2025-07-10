#!/usr/bin/env bash
#
# start_can.sh — load USB-CAN module, reset can0, configure at 500 kbps, and dump CAN frames

set -euo pipefail

# Ensure script is run as root
if (( EUID != 0 )); then
  echo "ERROR: this script must be run as root or via sudo." >&2
  exit 1
fi

# Load the USB-CAN kernel module if needed
if ! lsmod | grep -q '^gs_usb'; then
  modprobe gs_usb
  echo "Loaded gs_usb module."
else
  echo "gs_usb module already loaded."
fi

# Tear down can0 if it exists/in use (ignore errors)
ip link set can0 down 2>/dev/null || true

# Bring up can0 at 500 000 bit/s
if ip link set can0 up type can bitrate 500000; then
  echo "can0 is up @ 500 kbps."
else
  echo "ERROR: failed to set up can0." >&2
  exit 1
fi

echo "Starting candump on can0…"
exec candump can0
