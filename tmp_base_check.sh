#!/bin/sh
docker ps --format '{{.Names}}\t{{.Status}}' | grep -E vtest
echo --- rx logs ---
docker logs --tail 60 lifetrac-vtest-image_rx-1 2>&1 | tail -60
