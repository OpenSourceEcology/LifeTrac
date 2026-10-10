#!/bin/sh
# Wait for brcmfmac firmware load to complete, then connect FAST
# WiFi credentials come ONLY from the environment -- never commit an SSID or
# password to this repo:
#   LIFETRAC_WIFI_SSID  network name    LIFETRAC_WIFI_PSK  WPA passphrase
# sudo drops the caller's environment, so pass them through, e.g. on the board:
#   sudo env LIFETRAC_WIFI_SSID="<ssid>" LIFETRAC_WIFI_PSK="<psk>" sh /tmp/wifi_connect_fast.sh
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
