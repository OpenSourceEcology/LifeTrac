#!/bin/sh
set -e
echo "=== Disabling Wi-Fi permanently ==="
nmcli radio wifi off || true
nmcli connection delete MyWifi 2>/dev/null || true
rfkill block wifi || true
mkdir -p /etc/NetworkManager/conf.d
cat > /etc/NetworkManager/conf.d/99-disable-wifi.conf <<EOF
[device-wifi-disable]
match-device=interface-name:wlan0
managed=false
EOF
echo "--- 99-disable-wifi.conf ---"
cat /etc/NetworkManager/conf.d/99-disable-wifi.conf
systemctl restart NetworkManager
sleep 3
echo "=== nmcli radio ==="
nmcli radio
echo "=== rfkill list ==="
rfkill list
echo "=== wlan0 ip ==="
ip -o addr show wlan0 2>/dev/null || echo "wlan0 not present"
echo "=== active connections ==="
nmcli -t -f NAME,DEVICE,STATE connection show --active
echo "=== iw dev ==="
iw dev 2>/dev/null || echo "iw not available"
echo "=== done ==="
