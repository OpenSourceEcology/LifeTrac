#!/bin/sh
# Survey what the tractor X8 needs to start producing image frames.
echo "=== HOSTNAME / IP ==="
hostname
ip -o -4 addr show | awk '{print $2, $4}'
echo
echo "=== USB CAMERAS (/dev/video*) ==="
ls -l /dev/video* 2>&1 || echo "no /dev/video*"
echo
echo "=== v4l2 camera NAMES ==="
for n in /sys/class/video4linux/video*/name; do
  [ -e "$n" ] && printf "%s: %s\n" "$n" "$(cat $n)"
done
echo
echo "=== LoRa serial nodes (ttymxc3 + USB-CDC) ==="
ls -l /dev/ttymxc* /dev/ttyACM* /dev/ttyUSB* 2>&1 | grep -v 'No such'
echo
echo "=== ALREADY-RUNNING lifetrac services ==="
systemctl list-units 'lifetrac-*' --all --no-legend 2>&1 | head -20
echo
echo "=== DOCKER containers ==="
echo fio | sudo -S -p '' docker ps --format '{{.Names}}\t{{.Image}}\t{{.Status}}' 2>&1
echo
echo "=== /opt/lifetrac present? ==="
ls -la /opt/lifetrac 2>&1 | head -5
echo
echo "=== ffmpeg binary present? ==="
which ffmpeg; ls -l /tmp/ffmpeg 2>&1 | head -1
echo
echo "=== mosquitto local? (lora_bridge would expect MQTT) ==="
ss -ltn 2>/dev/null | grep -E '1883|8080' || netstat -ltn 2>/dev/null | grep -E '1883|8080' || echo "no ss/netstat"
echo
echo "=== Connectivity to BASE X8 (192.168.1.117) ==="
ping -c 2 -W 1 192.168.1.117 2>&1 | tail -3
echo
echo "=== M7 co-MCU sanity: is /dev/ttymxc3 readable + opened by anyone? ==="
echo fio | sudo -S -p '' lsof /dev/ttymxc3 2>&1 | head -5
echo
echo "=== Disk free ==="
df -h / | tail -1
echo
echo "=== Done ==="
