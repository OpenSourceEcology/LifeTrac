#!/bin/sh
# Wait for brcmfmac firmware load to complete, then connect FAST
# WiFi credentials come ONLY from the environment -- never commit an SSID or
# password to this repo:
#   LIFETRAC_WIFI_SSID  network name    LIFETRAC_WIFI_PSK  WPA passphrase
# Run it by hand in an interactive board shell (adb shell / ssh) -- never from a
# VS Code task or a PC command line, which echo or log the passphrase:
#   read -r LIFETRAC_WIFI_SSID; stty -echo; read -r LIFETRAC_WIFI_PSK; stty echo
#   export LIFETRAC_WIFI_SSID LIFETRAC_WIFI_PSK
#   sudo --preserve-env=LIFETRAC_WIFI_SSID,LIFETRAC_WIFI_PSK sh /tmp/wifi_connect_fast.sh
# (`sudo env LIFETRAC_WIFI_PSK=...` would show the passphrase in ps and in
# sudo's journal entry.)
# Bench note: the bench tractor runs with WiFi disabled on purpose
# (LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/
# bench_tools/BENCH_BOARDS.md); do not re-enable it during radio legs.
set -e
: "${LIFETRAC_WIFI_SSID:?set LIFETRAC_WIFI_SSID (WiFi network name; never commit it)}"
: "${LIFETRAC_WIFI_PSK:?set LIFETRAC_WIFI_PSK (WiFi passphrase; never commit it)}"
echo "=== waiting for wlan0 to appear ==="
for i in 1 2 3 4 5 6 7 8 9 10 15 20; do
  if ip link show wlan0 >/dev/null 2>&1; then
    echo "wlan0 ready at $i"
    break
  fi
  sleep 1
done
echo "=== rfkill unblock all ==="
rfkill unblock all
echo "=== nmcli wifi on ==="
nmcli radio wifi on
echo "=== rescan ==="
nmcli dev wifi rescan ifname wlan0 2>/dev/null || true
sleep 4
echo "=== connect ==="
nmcli dev wifi connect "$LIFETRAC_WIFI_SSID" password "$LIFETRAC_WIFI_PSK" ifname wlan0
sleep 3
echo "=== status ==="
nmcli -t -f NAME,DEVICE,STATE con show --active
ip -4 addr show wlan0
echo "=== ping ==="
ping -c 3 -W 2 8.8.8.8 || true
