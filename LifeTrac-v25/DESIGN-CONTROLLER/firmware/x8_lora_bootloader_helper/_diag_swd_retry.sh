#!/bin/bash
# _diag_swd_retry.sh — clean up stale openocd/cat, then probe whether
# SWD can reach H7 DP IDR before launching the full contract.
set -u
pkill -9 openocd 2>/dev/null || true
pkill -9 -f 'cat /dev/ttymxc3' 2>/dev/null || true
sleep 1

echo "=== minimal SWD probe (init only, no halt) ==="
timeout 10 openocd \
  -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
  -c "init" -c "shutdown" 2>&1 | tail -25
echo "OCD_PROBE_RC=$?"

echo ""
echo "=== retry contract ==="
bash /tmp/lifetrac_p0c/run_stage1_standard_contract.sh \
     /tmp/lifetrac_p0c/firmware.bin 2D0A1209DABC240B
