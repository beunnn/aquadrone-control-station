#!/usr/bin/env bash
# Install the hub mavlink-router config and restart the service (brief MAVLink gap while it restarts).
# Run on the Pi from the repo's hub folder: sudo ./setup_mavlink_router.sh
set -euo pipefail
cd "$(dirname "$0")"

if [ -f /etc/mavlink-router/main.conf ] && ! cmp -s mavlink-router/main.conf /etc/mavlink-router/main.conf; then
    cp /etc/mavlink-router/main.conf "/etc/mavlink-router/main.conf.$(date +%Y%m%d-%H%M%S).bak"
fi
install -m 0644 mavlink-router/main.conf /etc/mavlink-router/main.conf
systemctl restart mavlink-router
sleep 1
systemctl is-active mavlink-router
