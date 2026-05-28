#!/bin/bash
# run_flash_l072.sh — orchestrate the full flash:
#   1) launch openocd holding BOOT0 high, NRST pulsed, target halted (600s window)
#   2) wait 5s for ROM-bootloader entry
#   3) check fuser /dev/ttymxc3 (bail if held)
#   4) run stm32_an3155_flasher.py
#   5) wait for openocd to release at 600s mark (or kill it early on success)
#
# Run from /tmp/lifetrac_p0c/ on the X8.

set -u
TOOLDIR=/tmp/lifetrac_p0c
CFG=$TOOLDIR/07_assert_pa11_pf4_long.cfg
FLASHER=$TOOLDIR/stm32_an3155_flasher.py
IMAGE=${1:-$TOOLDIR/mlm32l07x01.bin}
DEV=/dev/ttymxc3
LOG=$TOOLDIR/flash_run.log
OCD_LOG=$TOOLDIR/flash_ocd.log

: > $LOG
: > $OCD_LOG

echo "=== sanity ===" | tee -a $LOG
ls -la $CFG $FLASHER $IMAGE 2>&1 | tee -a $LOG
HOLDERS=$(fuser $DEV 2>&1 || true)
echo "fuser $DEV (pre-evict): $HOLDERS" | tee -a $LOG

# 2026-05-26 Blocker B3 hardening: stop any lingering owners of
# /dev/ttymxc3 (camera_tx/rx daemons, prior flasher zombies) and any
# stale openocd that may still hold the H7 SWD GPIOs from a previous
# cycle. Without this, the next openocd attach can race the prior one
# and SWD DPIDR comes back 0xdeadbeef. The header comment promised
# this behavior; the code never enforced it. Now it does.
# 2026-05-26 ROOT-CAUSE FIX: also stop lifetrac-camera.service (the
# systemd unit that supervises the camera docker stack which holds
# /dev/ttymxc3 with continuous tx traffic). Without this stop, the
# i.MX8 UART4 tx FIFO never drains for our Python flasher and the
# first os.write returns BlockingIOError; the L072 also sees garbage
# during its boot window and never enters ROM. The unit is stopped
# for the duration of the flash; restart is the caller's job.
systemctl stop lifetrac-camera.service 2>/dev/null || true
pkill -9 -f camera_service 2>/dev/null || true
pkill -9 -f camera_tx_daemon 2>/dev/null || true
pkill -9 -f camera_rx_daemon 2>/dev/null || true
pkill -9 openocd 2>/dev/null || true
fuser -k $DEV 2>/dev/null || true
sleep 0.5
HOLDERS_POST=$(fuser $DEV 2>&1 || true)
echo "fuser $DEV (post-evict): $HOLDERS_POST" | tee -a $LOG
if [ -n "$HOLDERS_POST" ]; then
    echo "ERROR: $DEV still held after evict; refusing to flash." | tee -a $LOG
    exit 90
fi

# Diagnostic 0: BASELINE passive listen BEFORE any openocd / NRST.
# The murata_l072 firmware emits "LT_BOOT_HEARTBEAT stage=..." on
# host UART (USART2 PA2/PA3) at boot and may emit periodic frames.
# If /dev/ttymxc3 is correctly wired to the L072 firmware UART AND
# the chip is currently running firmware, we will see traffic. If
# we see nothing here, then EITHER (a) /dev/ttymxc3 is not wired
# to the firmware UART, OR (b) the chip is not running firmware
# (dead / wedged / stuck in ROM already with no host activity).
# Configure stty first so we read at the right frame format.
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo 2>/dev/null || true
echo "" | tee -a $LOG
echo "=== diag 0: BASELINE passive listen 3.0s (no openocd, no NRST) ===" | tee -a $LOG
rm -f $TOOLDIR/rxbase.bin
timeout 3 dd if=$DEV of=$TOOLDIR/rxbase.bin bs=1 count=512 2>/dev/null || true
echo "baseline_bytes=$(stat -c%s $TOOLDIR/rxbase.bin 2>/dev/null || echo 0)" | tee -a $LOG
if [ -s $TOOLDIR/rxbase.bin ]; then
    echo "baseline_hex_first128:" | tee -a $LOG
    xxd $TOOLDIR/rxbase.bin | head -8 | tee -a $LOG
    echo "baseline_ascii_first128:" | tee -a $LOG
    tr -c '[:print:]\n' '.' < $TOOLDIR/rxbase.bin | head -c 128 | tee -a $LOG
    echo "" | tee -a $LOG
fi

echo "" | tee -a $LOG
echo "=== launching openocd (600s hold) ===" | tee -a $LOG
# Per-cycle gpio preflight: ensure gpio8/10/15 are exported and gpio10 (H7 NRST)
# is driven high and given time to settle BEFORE openocd attaches. Then UNEXPORT
# them so openocd's imx_gpio mmap driver has uncontested access (sysfs-owned gpio
# pins can collide with openocd's direct register writes via /dev/mem mmap).
# Symptom of the collision on the RX X8 (OpenOCD 0.11.0-dirty 2025-07-14):
# SWD DPIDR returns 0xdeadbeef and OCD never reaches `init` / Phase A.
for n in 8 10 15; do
    [ -d /sys/class/gpio/gpio$n ] || echo $n > /sys/class/gpio/export 2>/dev/null
done
echo out > /sys/class/gpio/gpio10/direction 2>/dev/null
echo 1   > /sys/class/gpio/gpio10/value     2>/dev/null
sleep 1
echo "gpio10_value_before_unexport=$(cat /sys/class/gpio/gpio10/value 2>/dev/null)" | tee -a $LOG
for n in 8 10 15; do
    [ -d /sys/class/gpio/gpio$n ] && echo $n > /sys/class/gpio/unexport 2>/dev/null
done

nohup openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
              -f $CFG \
              > $OCD_LOG 2>&1 < /dev/null &
OCD_PID=$!
echo "OCD_PID=$OCD_PID" | tee -a $LOG

# Poll for READY banner up to 25s. The newer X8 OpenOCD build
# (0.11.0-dirty 2025-07-14) on the RX X8 (e.g. 2D0A1209DABC240B) can take
# 8-12s between gpio10-high settle and SWD-attach completion, vs <5s on
# the older bundled OpenOCD on the TX X8. The previous fixed `sleep 5`
# raced this and falsely reported "openocd did not reach READY phase".
READY_DEADLINE=$(( $(date +%s) + 25 ))
while [ "$(date +%s)" -lt "$READY_DEADLINE" ]; do
    if grep -q "READY: L072 in STM32 ROM bootloader" "$OCD_LOG"; then
        break
    fi
    sleep 1
done

echo "" | tee -a $LOG
echo "=== openocd output so far ===" | tee -a $LOG
cat $OCD_LOG | tee -a $LOG

if ! grep -q "READY: L072 in STM32 ROM bootloader" $OCD_LOG ; then
    echo "" | tee -a $LOG
    echo "ERROR: openocd did not reach READY phase. Aborting flash." | tee -a $LOG
    kill -9 $OCD_PID 2>/dev/null
    exit 2
fi

echo "" | tee -a $LOG
echo "=== configure UART 19200 8E1 raw ===" | tee -a $LOG
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo
echo "--- stty -a (post-config) ---" | tee -a $LOG
stty -F $DEV -a 2>&1 | tee -a $LOG
echo "--- end stty ---" | tee -a $LOG

# Diagnostic A: passive listen. If the L072 is running user firmware
# (i.e. BOOT0 was NOT honored at NRST rising edge), it will emit
# host_cmd serialrpc traffic on /dev/ttymxc3 at its own baud rate.
# 1s passive read should be silent if the chip is correctly in ROM
# bootloader (ROM waits for the host to speak first). Any bytes here
# = strong signal that user firmware is running, not ROM.
echo "" | tee -a $LOG
echo "=== diag A: passive listen 3.0s post-NRST (expect EMPTY in ROM; if firmware runs and BOOT0 ignored, we should see LT_BOOT_HEARTBEAT here) ===" | tee -a $LOG
rm -f $TOOLDIR/rxpre.bin
timeout 3 dd if=$DEV of=$TOOLDIR/rxpre.bin bs=1 count=512 2>/dev/null || true
echo "passive_bytes=$(stat -c%s $TOOLDIR/rxpre.bin 2>/dev/null || echo 0)" | tee -a $LOG
if [ -s $TOOLDIR/rxpre.bin ]; then
    echo "passive_hex_first128:" | tee -a $LOG
    xxd $TOOLDIR/rxpre.bin | head -8 | tee -a $LOG
    echo "passive_ascii_first128:" | tee -a $LOG
    tr -c '[:print:]\n' '.' < $TOOLDIR/rxpre.bin | head -c 128 | tee -a $LOG
    echo "" | tee -a $LOG
fi
# Diagnostic B: independent raw 0x7F probe. Bypasses the Python
# flasher entirely. Writes a single 0x7F via /bin/printf and reads up
# to 4 bytes with a 2s timeout. ACK=0x79 means the L072 ROM IS alive
# and the silence is a Python/flasher-layer issue. No response means
# the chip itself is not in ROM (or our UART path is broken).
#
# 2026-05-26: DISABLED in production. Confirmed via this probe that
# the ROM IS alive (ACK=0x79). Leaving the probe ENABLED corrupts
# the autobaud state: the L072 ROM v3.1 autobauds on the FIRST 0x7F
# it sees and from then on treats every byte as a command. Our
# probe locks autobaud, then the Python flasher's own 0x7F is
# treated as an invalid command, the subsequent GET (0x00 0xFF) is
# NACK'd, and the flash fails with FAIL_ID (NACK on GET). Toggle
# via FLASH_DIAG_RAW_PROBE=1 if you need to re-confirm the chip is
# alive without running the full flasher.
echo "" | tee -a $LOG
if [ "${FLASH_DIAG_RAW_PROBE:-0}" = "1" ]; then
    echo "=== diag B: raw 0x7F probe (bypasses python) ===" | tee -a $LOG
    rm -f $TOOLDIR/rxprobe.bin
    ( timeout 2 dd if=$DEV of=$TOOLDIR/rxprobe.bin bs=1 count=4 2>/dev/null || true ) &
    DD_PID=$!
    sleep 0.1
    printf '\x7f' > $DEV
    wait $DD_PID 2>/dev/null || true
    echo "probe_bytes=$(stat -c%s $TOOLDIR/rxprobe.bin 2>/dev/null || echo 0)" | tee -a $LOG
    if [ -s $TOOLDIR/rxprobe.bin ]; then
        echo "probe_hex:" | tee -a $LOG
        xxd $TOOLDIR/rxprobe.bin | tee -a $LOG
    else
        echo "probe_hex: <silence>" | tee -a $LOG
    fi
    echo "WARN: raw 0x7F probe was ENABLED; autobaud locked, flasher will FAIL_ID. Set FLASH_DIAG_RAW_PROBE=0 for real flashes." | tee -a $LOG
else
    echo "=== diag B: raw 0x7F probe SKIPPED (FLASH_DIAG_RAW_PROBE!=1; would corrupt autobaud) ===" | tee -a $LOG
fi

# Drain any residual byte before the Python flasher runs.
timeout 0.3 dd if=$DEV of=/dev/null bs=1 count=64 2>/dev/null || true

echo "" | tee -a $LOG
if [ "${FLASH_VERIFY_ONLY:-0}" = "1" ]; then
    echo "=== running flasher (readback verify only) ===" | tee -a $LOG
    python3 -u $FLASHER $IMAGE --verify-only 2>&1 | tee -a $LOG
else
    echo "=== running flasher (with --verify) ===" | tee -a $LOG
    python3 -u $FLASHER $IMAGE --verify 2>&1 | tee -a $LOG
fi
RC=${PIPESTATUS[0]}
echo "flasher exit code = $RC" | tee -a $LOG

echo "" | tee -a $LOG
echo "=== killing openocd ($OCD_PID) ===" | tee -a $LOG
kill -9 $OCD_PID 2>/dev/null
sleep 1

echo "" | tee -a $LOG
echo "=== final openocd log ===" | tee -a $LOG
tail -30 $OCD_LOG | tee -a $LOG

echo "" | tee -a $LOG
echo "=== overall result: flasher RC=$RC ===" | tee -a $LOG
exit $RC
