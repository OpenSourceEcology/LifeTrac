#!/bin/sh
# Pre-create the NetworkManager profile so auto-connect fires during the
# brief healthy window right after boot.
# WiFi credentials come ONLY from the environment -- never commit an SSID or
# password to this repo:
#   LIFETRAC_WIFI_SSID  network name    LIFETRAC_WIFI_PSK  WPA passphrase
# sudo drops the caller's environment, so pass them through, e.g. on the board:
#   sudo env LIFETRAC_WIFI_SSID="<ssid>" LIFETRAC_WIFI_PSK="<psk>" sh /tmp/wifi_persist_and_reboot.sh
# Bench note: the bench tractor runs with WiFi disabled on purpose
# (LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/
# bench_tools/BENCH_BOARDS.md); do not re-enable it during radio legs.
set -e
: "${LIFETRAC_WIFI_SSID:?set LIFETRAC_WIFI_SSID (WiFi network name; never commit it)}"
: "${LIFETRAC_WIFI_PSK:?set LIFETRAC_WIFI_PSK (WiFi passphrase; never commit it)}"
SSID="$LIFETRAC_WIFI_SSID"
PSK="$LIFETRAC_WIFI_PSK"
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
