#!/bin/sh
# Phase 1: verify tractor image has needed deps; show what's running.
set -e
echo "=== lifetrac-tractor-x8 image deps ==="
echo fio | sudo -S docker run --rm --entrypoint sh lifetrac-tractor-x8:latest -c "python3 -c 'import serial,paho.mqtt.client; print(\"ok pyserial=\"+serial.__version__)'"
echo
echo "=== tractor-camera container ==="
echo fio | sudo -S docker ps --filter name=tractor-camera --format "{{.Names}}	{{.Status}}"
