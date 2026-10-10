#!/bin/sh
# WiFi credentials come ONLY from the environment -- never commit an SSID or
# password to this repo:
#   LIFETRAC_WIFI_SSID  network name    LIFETRAC_WIFI_PSK  WPA passphrase
# sudo drops the caller's environment, so pass them through, e.g. on the board:
#   sudo env LIFETRAC_WIFI_SSID="<ssid>" LIFETRAC_WIFI_PSK="<psk>" sh /tmp/connect_wpa.sh
# Bench note: the bench tractor runs with WiFi disabled on purpose
# (LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/
# bench_tools/BENCH_BOARDS.md); do not re-enable it during radio legs.
: "${LIFETRAC_WIFI_SSID:?set LIFETRAC_WIFI_SSID (WiFi network name; never commit it)}"
: "${LIFETRAC_WIFI_PSK:?set LIFETRAC_WIFI_PSK (WiFi passphrase; never commit it)}"
# The config holds the passphrase: keep it root-only. (A '"' in the SSID or
# passphrase would need escaping for wpa_supplicant.conf.)
(
umask 077
cat > /tmp/wpa.conf <<EOF
network={
    ssid="${LIFETRAC_WIFI_SSID}"
    psk="${LIFETRAC_WIFI_PSK}"
}
EOF
)
killall wpa_supplicant 2>/dev/null || true
wpa_supplicant -B -i wlan0 -c /tmp/wpa.conf
sleep 3
udhcpc -i wlan0 -n -q 2>/dev/null || dhclient wlan0 2>/dev/null || true
ip -4 addr show wlan0
