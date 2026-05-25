#!/bin/sh
echo "=== ls /dev/ttymxc3 ==="
ls -la /dev/ttymxc3 2>&1
echo "=== python stdlib + paho ==="
python3 -c "import paho.mqtt.client, logging, dataclasses, queue, json, hashlib, struct, threading; print('ALL_OK')" 2>&1
echo "=== which python3 ==="
which python3
python3 --version
