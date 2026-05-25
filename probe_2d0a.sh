#!/bin/sh
echo "=== docker ps ==="
echo fio | sudo -S -p '' docker ps -a --format '{{.Names}} {{.Image}}'
echo "=== docker images ==="
echo fio | sudo -S -p '' docker images --format '{{.Repository}}:{{.Tag}}'
echo "=== /dev/ttymxc3 ==="
ls -la /dev/ttymxc3
