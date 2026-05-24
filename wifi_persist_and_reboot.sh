#!/bin/sh
# Pre-create the NetworkManager profile so auto-connect fires during the
# brief healthy window right after boot.
set -e
SSID="I'm a 5 star LAN!"
PSK="9182603083"
NAME="5star"

# Delete any prior profile with same name
nmcli con delete "$NAME" 2>/dev/null || true

# Create profile in disconnected state with autoconnect enabled
nmcli con add type wifi ifname wlan0 con-name "$NAME" ssid "$SSID" \
  -- wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK" \
  connection.autoconnect yes connection.autoconnect-priority 100

echo "=== profile created ==="
nmcli -t con show "$NAME" | grep -E '^(connection.id|connection.autoconnect|802-11-wireless.ssid|802-11-wireless-security.key-mgmt):'

echo "=== rebooting ==="
sync
sleep 1
reboot
