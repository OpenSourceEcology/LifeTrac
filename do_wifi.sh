#!/bin/sh
# WiFi credentials come ONLY from the environment -- never commit an SSID or
# password to this repo:
#   LIFETRAC_WIFI_SSID  network name    LIFETRAC_WIFI_PSK  WPA passphrase
# sudo drops the caller's environment, so pass them through, e.g. on the board:
#   sudo env LIFETRAC_WIFI_SSID="<ssid>" LIFETRAC_WIFI_PSK="<psk>" sh /tmp/do_wifi.sh
# Bench note: the bench tractor runs with WiFi disabled on purpose
# (LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/
# bench_tools/BENCH_BOARDS.md); do not re-enable it during radio legs.
: "${LIFETRAC_WIFI_SSID:?set LIFETRAC_WIFI_SSID (WiFi network name; never commit it)}"
: "${LIFETRAC_WIFI_PSK:?set LIFETRAC_WIFI_PSK (WiFi passphrase; never commit it)}"
nmcli con delete 5star 2>/dev/null || true
nmcli con delete "$LIFETRAC_WIFI_SSID" 2>/dev/null || true
nmcli con add type wifi con-name 5star ifname wlan0 ssid "$LIFETRAC_WIFI_SSID" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$LIFETRAC_WIFI_PSK" connection.autoconnect yes
nmcli con up 5star
ip -4 addr show wlan0
