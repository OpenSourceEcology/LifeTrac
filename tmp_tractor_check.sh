#!/bin/sh
sleep 30
docker logs --tail 30 tractor-image-tx-v2 2>&1 | tail -30
echo --- camera ---
docker logs --tail 10 tractor-camera-v2 2>&1 | tail -10
