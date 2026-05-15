#!/bin/bash
# W2-01 C2 Bench / Field Provisioning Script
# This lays down the production software state required for the camera USB fixes.

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Must run as root. Re-run with sudo."
    exit 1
fi

echo "Installing modprobe configuration..."
cp lifetrac-no-usb-audio.conf /etc/modprobe.d/

echo "Removing currently loaded USB audio driver if present..."
modprobe -r snd_usb_audio snd_usbmidi_lib snd_hwdep 2>/dev/null || true

echo "Installing udev rules..."
cp 99-w2-01-c2.rules /etc/udev/rules.d/

echo "Installing systemd service..."
cp lifetrac-camera.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable lifetrac-camera.service

echo "Reloading udev and re-triggering C2 rules..."
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=usb
udevadm trigger --action=add --subsystem-match=video4linux || true
udevadm settle --timeout=5 || true

if [ -e /dev/lifetrac-c2 ]; then
    systemctl restart lifetrac-camera.service || true
fi

echo "Enabling video group for user 'fio'..."
# the LmP image default user might be fio
usermod -aG video fio || echo "User fio not found, ignoring..."

echo "Provisioning complete. Recommended to restart or verify the camera node."
