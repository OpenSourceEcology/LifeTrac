#!/bin/bash
set -e
echo "=== current state ==="
grep -nE "LIFETRAC_REG_PROFILE|LIFETRAC_FHSS_WIDE_MASK|LIFETRAC_FORCE_FRF_HZ" /opt/lifetrac/video-test/docker-compose.yml || true
echo "=== run flip ==="
echo fio | sudo -S python3 /tmp/_flip_tractor.py
echo "=== new state ==="
grep -nE "LIFETRAC_REG_PROFILE|LIFETRAC_FHSS_WIDE_MASK|LIFETRAC_FORCE_FRF_HZ" /opt/lifetrac/video-test/docker-compose.yml || true
echo "=== restart image_tx ==="
echo fio | sudo -S bash -lc 'cd /opt/lifetrac/video-test && docker compose up -d --force-recreate image_tx 2>&1 | tail -10'
sleep 3
echo "=== containers ==="
echo fio | sudo -S docker ps --format 'table {{.Names}}\t{{.Status}}'
