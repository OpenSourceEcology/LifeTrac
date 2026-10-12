#!/bin/bash
# DO NOT RUN on the bench without an explicit radio GO: runs
# /tmp/_flip_tractor.py on /opt/lifetrac/video-test/docker-compose.yml, then
# re-creates image_tx (compose project video-test), which opens /dev/ttymxc3 =
# transmits.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units".
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
