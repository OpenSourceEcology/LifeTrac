#!/bin/sh
# test_swd_bypass_probe.sh — V1: run bypass launcher in background, probe
# /dev/ttymxc3 for STM32 ROM ACK 0x79. Mirrors the probe sequence from
# run_pa11_pf4_test.sh, but uses the SWD-free launcher.
#
# Outputs to /tmp/swd_bypass_probe.log and /tmp/swd_bypass_rom_${i}.bin.
#
# Usage: test_swd_bypass_probe.sh
# Env:   SUDO_PASS=fio HOLD_S=30
set -u
SUDO_PASS="${SUDO_PASS:-fio}"
HOLD_S="${HOLD_S:-30}"
ATTEMPTS="${ATTEMPTS:-5}"
LAUNCHER="${LAUNCHER:-/tmp/swd_bypass_pa11_pf4_launcher.sh}"
DEV=/dev/ttymxc3
LOG=/tmp/swd_bypass_probe.log

# Re-exec as root if we aren't already (so ttymxc3, sysfs writes all just work).
if [ "$(id -u)" != "0" ]; then
  exec sh -c "echo '$SUDO_PASS' | sudo -S -p '' env SUDO_PASS='$SUDO_PASS' HOLD_S='$HOLD_S' ATTEMPTS='$ATTEMPTS' LAUNCHER='$LAUNCHER' sh '$0' \"\$@\"" -- "$@"
fi

: > "$LOG"

log() { printf '[probe %s] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG"; }

log "=== launching bypass launcher in background (HOLD_S=$HOLD_S) ==="
HOLD_S="$HOLD_S" SUDO_PASS="$SUDO_PASS" nohup "$LAUNCHER" \
  > /tmp/swd_bypass_launcher.log 2>&1 < /dev/null &
LAUNCHER_PID=$!
log "launcher pid=$LAUNCHER_PID"

# Wait for launcher to drive PA11 high + pulse PF4 + reach hold (~500 ms).
sleep 2

log "=== launcher state so far ==="
tail -20 /tmp/swd_bypass_launcher.log | tee -a "$LOG"

log "=== configuring $DEV for 19200 8E1 ==="
stty -F "$DEV" 19200 cs8 parenb -parodd -cstopb raw -echo time 5 min 0 2>&1 | tee -a "$LOG"

ACK_HIT=0
i=1
while [ "$i" -le "$ATTEMPTS" ]; do
  log "--- attempt $i ---"
  rm -f "/tmp/swd_bypass_rom_${i}.bin"
  cat "$DEV" > "/tmp/swd_bypass_rom_${i}.bin" &
  CATPID=$!
  sleep 0.3
  printf '\x7f' > "$DEV"
  sleep 0.8
  kill -9 "$CATPID" 2>/dev/null || true
  wait "$CATPID" 2>/dev/null || true
  SZ=$(stat -c %s "/tmp/swd_bypass_rom_${i}.bin" 2>/dev/null || echo 0)
  HEX=$(xxd -p "/tmp/swd_bypass_rom_${i}.bin" 2>/dev/null | tr -d '\n' | head -c 32)
  log "  size=$SZ hex=${HEX}"
  if [ "$SZ" -gt 0 ]; then
    FIRST=$(head -c 1 "/tmp/swd_bypass_rom_${i}.bin" | xxd -p | head -c 2)
    if [ "$FIRST" = "79" ]; then
      log "  *** GOT 0x79 ACK — L072 IS IN STM32 ROM BOOTLOADER (via x8h7 bridge) ***"
      ACK_HIT=1
      break
    fi
  fi
  sleep 0.5
  i=$((i + 1))
done

log "=== ACK_HIT=$ACK_HIT ==="

log "=== killing launcher ==="
kill -TERM "$LAUNCHER_PID" 2>/dev/null || true
wait "$LAUNCHER_PID" 2>/dev/null || true

log "=== final launcher log tail ==="
tail -30 /tmp/swd_bypass_launcher.log | tee -a "$LOG"

if [ "$ACK_HIT" -eq 1 ]; then
  log "RESULT: PASS"
  exit 0
else
  log "RESULT: FAIL"
  exit 10
fi
