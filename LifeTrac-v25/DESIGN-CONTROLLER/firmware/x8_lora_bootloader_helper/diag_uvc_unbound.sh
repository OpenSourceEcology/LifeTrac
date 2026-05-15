#!/bin/sh
echo "=== lsmod ==="
lsmod | grep -E "uvc|snd_usb|usbcore"
echo "=== uvcvideo driver dir ==="
ls -l /sys/bus/usb/drivers/uvcvideo/ 2>&1
echo "=== /dev/video* ==="
ls -l /dev/video* 2>&1
echo "=== modinfo uvcvideo ==="
modinfo uvcvideo 2>&1 | head -5
echo "=== modprobe.d ==="
cat /etc/modprobe.d/lifetrac-no-usb-audio.conf 2>&1
echo "=== modprobe -n uvcvideo ==="
modprobe -n -v uvcvideo 2>&1
echo "=== udevadm info -a 1-1.2.3:1.0 ==="
udevadm info -a -p /sys/bus/usb/devices/1-1.2.3:1.0 2>&1 | head -25
echo "=== udevadm test ==="
udevadm test /sys/bus/usb/devices/1-1.2.3:1.0 2>&1 | tail -25
echo "=== dmesg uvc/c2 ==="
dmesg | grep -E -i "uvc|kurokesu|c2|1-1\.2\.3" | tail -40
