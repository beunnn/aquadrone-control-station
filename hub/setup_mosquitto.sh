#!/usr/bin/env bash
# Install Mosquitto on the Pi hub with password auth and the AquaDrone ACL.
# Run on the Pi from the repo root: sudo hub/setup_mosquitto.sh  (prompts for each user's password)
set -euo pipefail
cd "$(dirname "$0")"

apt-get install -y mosquitto mosquitto-clients
install -m 0644 mosquitto/aquadrone.conf /etc/mosquitto/conf.d/aquadrone.conf
install -m 0600 -o mosquitto -g mosquitto mosquitto/aquadrone.acl /etc/mosquitto/aquadrone.acl

if [ ! -f /etc/mosquitto/passwd ]; then
    install -m 0600 -o mosquitto -g mosquitto /dev/null /etc/mosquitto/passwd
fi
for u in tab5 ctrlbox filter operator; do
    echo "Password for MQTT user '$u':"
    mosquitto_passwd /etc/mosquitto/passwd "$u"
done
# Owner as the broker (2.0.21) wants it; mosquitto_passwd's "owner is not root" warning is harmless.
chown mosquitto:mosquitto /etc/mosquitto/passwd
chmod 0600 /etc/mosquitto/passwd

systemctl enable mosquitto
systemctl restart mosquitto
systemctl --no-pager status mosquitto | head -n 5
