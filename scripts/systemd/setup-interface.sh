#!/usr/bin/env bash
set -e

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root or with sudo."
  exit 1
fi

# install the executable
install -Dm755 can-interface.sh \
  /usr/local/bin/can-interface.sh

# install the systemd unit
install -Dm644 can-interface.service \
  /etc/systemd/system/can-interface.service

# reload and start
systemctl daemon-reload
systemctl enable can-interface.service
systemctl start  can-interface.service

echo "Successfully installed and started the can-interface."
