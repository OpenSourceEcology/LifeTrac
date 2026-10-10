#!/bin/sh
# WiFi credentials come ONLY from the environment -- never commit an SSID or
# password to this repo:
#   LIFETRAC_WIFI_SSID  network name    LIFETRAC_WIFI_PSK  WPA passphrase
# sudo drops the caller's environment, so pass them through, e.g. on the board:
#   sudo env LIFETRAC_WIFI_SSID="<ssid>" LIFETRAC_WIFI_PSK="<psk>" sh /tmp/wifi_rescan_connect.sh
# Bench note: the bench tractor runs with WiFi disabled on purpose
# (LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/
# bench_tools/BENCH_BOARDS.md); do not re-enable it during radio legs.
: "${LIFETRAC_WIFI_SSID:?set LIFETRAC_WIFI_SSID (WiFi network name; never commit it)}"
: "${LIFETRAC_WIFI_PSK:?set LIFETRAC_WIFI_PSK (WiFi passphrase; never commit it)}"
echo "=== rescan (12s) ==="
nmcli dev wifi rescan ifname wlan0 2>&1 || true
sleep 12
echo "=== visible nets ==="
nmcli -t -f SSID,SIGNAL dev wifi list ifname wlan0 | head -n 15
echo "=== connect ==="
nmcli dev wifi connect "$LIFETRAC_WIFI_SSID" password "$LIFETRAC_WIFI_PSK" ifname wlan0
sleep 4
echo "=== active connections ==="
nmcli -t -f NAME,DEVICE,STATE con show --active
echo "=== ip ==="
ip -4 addr show wlan0
echo "=== ping ==="
ping -c 3 -W 2 8.8.8.8 || true
echo "=== dmesg tail ==="
dmesg | tail -n 10
