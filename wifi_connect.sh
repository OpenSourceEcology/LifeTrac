#!/bin/sh
# WiFi credentials come ONLY from the environment -- never commit an SSID or
# password to this repo:
#   LIFETRAC_WIFI_SSID  network name    LIFETRAC_WIFI_PSK  WPA passphrase
# Run it by hand in an interactive board shell (adb shell / ssh) -- never from a
# VS Code task or a PC command line, which echo or log the passphrase:
#   read -r LIFETRAC_WIFI_SSID; stty -echo; read -r LIFETRAC_WIFI_PSK; stty echo
#   export LIFETRAC_WIFI_SSID LIFETRAC_WIFI_PSK
#   sudo --preserve-env=LIFETRAC_WIFI_SSID,LIFETRAC_WIFI_PSK sh /tmp/wifi_connect.sh
# (`sudo env LIFETRAC_WIFI_PSK=...` would show the passphrase in ps and in
# sudo's journal entry.)
# Bench note: the bench tractor runs with WiFi disabled on purpose
# (LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/
# bench_tools/BENCH_BOARDS.md); do not re-enable it during radio legs.
set -e
: "${LIFETRAC_WIFI_SSID:?set LIFETRAC_WIFI_SSID (WiFi network name; never commit it)}"
: "${LIFETRAC_WIFI_PSK:?set LIFETRAC_WIFI_PSK (WiFi passphrase; never commit it)}"
SSID="$LIFETRAC_WIFI_SSID"
PSK="$LIFETRAC_WIFI_PSK"
nmcli radio wifi on
nmcli dev wifi rescan 2>/dev/null || true
sleep 3
nmcli dev wifi connect "$SSID" password "$PSK" ifname wlan0
sleep 2
echo "--- connection status ---"
nmcli -t -f NAME,DEVICE,STATE con show --active
echo "--- ip ---"
ip -4 addr show wlan0
echo "--- ping ---"
ping -c 2 -W 2 8.8.8.8 || true
