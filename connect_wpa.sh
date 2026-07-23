#!/bin/sh
cat > /tmp/wpa.conf <<'EOF'
network={
    ssid="I'm a 5 star LAN!"
    psk="9182603083"
}
EOF
killall wpa_supplicant 2>/dev/null || true
wpa_supplicant -B -i wlan0 -c /tmp/wpa.conf
sleep 3
udhcpc -i wlan0 -n -q 2>/dev/null || dhclient wlan0 2>/dev/null || true
ip -4 addr show wlan0
