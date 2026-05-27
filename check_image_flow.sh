#!/bin/sh
echo "=== lora_bridge logs (last 50) ==="
docker logs --tail 50 design-controller-lora_bridge-1 2>&1
echo ""
echo "=== mosquitto sub on tile_delta (5s sample) ==="
timeout 5 docker exec design-controller-mosquitto-1 mosquitto_sub -h 127.0.0.1 -t 'lifetrac/v25/#' -v -W 5 2>&1 | head -20 || echo "(no messages or timeout)"
echo ""
echo "=== ttymxc3 counters (basestation) ==="
grep '30A60000' /proc/tty/driver/IMX-uart 2>/dev/null
echo ""
echo "=== L072 detection ==="
ls -la /dev/ttymxc3 2>&1
