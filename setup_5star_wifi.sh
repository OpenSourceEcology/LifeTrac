#!/bin/sh
nmcli con delete 5star 2>/dev/null || true
nmcli con add type wifi ifname wlan0 con-name 5star ssid "I'm a 5 star LAN!" -- wifi-sec.key-mgmt wpa-psk wifi-sec.psk 9182603083 connection.autoconnect yes
nmcli con up 5star
ip -4 addr show wlan0
