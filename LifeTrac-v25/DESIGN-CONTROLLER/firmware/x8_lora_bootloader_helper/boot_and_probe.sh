#!/bin/bash
# boot_and_probe.sh — release BOOT0, reset L072, probe AT response.
#
# Run AFTER successful flash. Sources 08_boot_user_app.cfg via openocd to
# drive PA_11=LOW + pulse PF_4, then issues AT+VER? at 19200 8N1 and prints
# the response. Expect a real version string instead of '+ERR_RX'.
#
# 2026-05-11: Restricted DEV_LIST to /dev/ttymxc3 only. Including ttymxc0
# (the X8 console UART, no termios) caused the writes to loop back via the
# kernel tty discipline, yielding a non-zero "AT response size" without the
# L072 actually replying — a false-positive PASS in the standard contract.
# Override with LIFETRAC_UART_DEV_LIST only when intentionally probing
# alternate carrier wirings; never re-add /dev/ttymxc0.

set -e
LOG=/tmp/lifetrac_p0c/boot_probe.log
DEV_LIST_DEFAULT="/dev/ttymxc3"
DEV_LIST="${LIFETRAC_UART_DEV_LIST:-$DEV_LIST_DEFAULT}"
HELPER_DIR=/tmp/lifetrac_p0c
mkdir -p $HELPER_DIR
: > $LOG

echo "=== killing any lingering openocd ===" | tee -a $LOG
for dev in $DEV_LIST; do
  fuser -k "$dev" 2>/dev/null || true
done
pkill -9 openocd 2>/dev/null || true
sleep 1

echo "" | tee -a $LOG
echo "=== running 08_boot_user_app.cfg (release BOOT0 + pulse NRST) ===" | tee -a $LOG
openocd \
  -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
  -f $HELPER_DIR/08_boot_user_app.cfg 2>&1 | tee -a $LOG

# Give the L072 user firmware time to boot and start its UART
sleep 1.5

BAUD_LIST="19200 9600 38400"
ATTEMPTS_PER_BAUD=3
BEST_SZ=0
BEST_BAUD=0
BEST_ATTEMPT=0
BEST_DEV=""

wait_for_uart_dev() {
  local dev="$1"
  local tries="${2:-5}"
  local delay_s="${3:-0.2}"
  local attempt=0

  while [ "$attempt" -lt "$tries" ]; do
    if [ -e "$dev" ]; then
      return 0
    fi

    attempt=$((attempt + 1))
    sleep "$delay_s"
  done

  return 1
}

probe_at_once() {
  local dev="$1"
  local baud="$2"
  local attempt="$3"
  local passive_file="/tmp/boot_passive_$(basename "$dev")_${baud}_${attempt}.bin"
  local passive_err_file="/tmp/boot_passive_$(basename "$dev")_${baud}_${attempt}.err"
  local resp_file="/tmp/at_resp_$(basename "$dev")_${baud}_${attempt}.bin"
  local err_file="/tmp/at_resp_$(basename "$dev")_${baud}_${attempt}.err"
  local stty_log="/tmp/stty_$(basename "$dev")_${baud}_${attempt}.log"
  local sz=0
  local passive_sz=0

  rm -f "$passive_file" "$passive_err_file" "$resp_file" "$err_file" "$stty_log"

  echo "" | tee -a $LOG
  echo "=== UART configure ${dev} ${baud} 8N1 raw ===" | tee -a $LOG
  if ! wait_for_uart_dev "$dev" 6 0.2; then
    echo "WARN: skipping dev=${dev} baud=${baud} attempt=${attempt} (device unavailable)" | tee -a "$LOG"
    return 1
  fi

  stty -F "$dev" "$baud" cs8 -parenb -cstopb raw -echo -ixon -ixoff time 5 min 0 > "$stty_log" 2>&1
  local stty_rc=$?
  if [ -s "$stty_log" ]; then
    cat "$stty_log" | tee -a "$LOG"
  fi

  if [ "$stty_rc" -ne 0 ]; then
    if grep -qi "Inappropriate ioctl for device" "$stty_log" 2>/dev/null; then
      echo "NOTE: continuing with existing line settings for dev=${dev} baud=${baud} attempt=${attempt}" | tee -a "$LOG"
    else
      echo "WARN: skipping dev=${dev} baud=${baud} attempt=${attempt} (stty failed)" | tee -a "$LOG"
      return 1
    fi
  fi

  # First capture any spontaneous boot bytes before driving AT traffic.
  cat "$dev" > "$passive_file" 2> "$passive_err_file" &
  local passpid=$!
  sleep 0.8
  kill -9 "$passpid" 2>/dev/null || true
  wait "$passpid" 2>/dev/null || true

  if [ -s "$passive_err_file" ]; then
    echo "WARN: UART passive-read stderr (baud=${baud}, attempt=${attempt}):" | tee -a "$LOG"
    head -20 "$passive_err_file" | tee -a "$LOG"
  fi

  if [ -f "$passive_file" ]; then
    passive_sz=$(stat -c %s "$passive_file" 2>/dev/null || echo 0)
  fi

  echo "Passive probe result: dev=${dev} baud=${baud} attempt=${attempt} size=${passive_sz} bytes" | tee -a "$LOG"

  cat "$dev" > "$resp_file" 2> "$err_file" &
  local catpid=$!
  sleep 0.3

  if ! printf 'AT\r\n' > "$dev"; then
    echo "WARN: write failed for AT (dev=${dev}, baud=${baud}, attempt=${attempt})" | tee -a "$LOG"
  fi
  sleep 0.4
  if ! printf 'AT+VER?\r\n' > "$dev"; then
    echo "WARN: write failed for AT+VER? (dev=${dev}, baud=${baud}, attempt=${attempt})" | tee -a "$LOG"
  fi
  sleep 0.8

  kill -9 "$catpid" 2>/dev/null || true
  wait "$catpid" 2>/dev/null || true

  if [ -s "$err_file" ]; then
    echo "WARN: UART read stderr (baud=${baud}, attempt=${attempt}):" | tee -a "$LOG"
    head -20 "$err_file" | tee -a "$LOG"
  fi

  if [ -f "$resp_file" ]; then
    sz=$(stat -c %s "$resp_file" 2>/dev/null || echo 0)
  fi

  # Reject pure-echo: if every byte in the response also appears in the
  # request bytes ("AT\r\nAT+VER?\r\n"), classify as loopback (size=0).
  # This is a defence-in-depth check on top of the DEV_LIST restriction.
  local nonecho_sz=0
  if [ "$sz" -gt 0 ]; then
    nonecho_sz=$(python3 -c 'import sys
req=set(b"AT\r\nAT+VER?\r\n")
data=open(sys.argv[1],"rb").read()
print(sum(1 for b in data if b not in req))' "$resp_file" 2>/dev/null || echo 0)
    if [ "$nonecho_sz" -eq 0 ]; then
      echo "NOTE: dev=${dev} baud=${baud} attempt=${attempt} response is pure echo of request bytes — treating as loopback (sz=0)" | tee -a "$LOG"
      sz=0
    fi
  fi

  echo "AT probe result: dev=${dev} baud=${baud} attempt=${attempt} size=${sz} bytes (passive=${passive_sz}, nonecho=${nonecho_sz})" | tee -a "$LOG"

  if [ "$sz" -gt "$BEST_SZ" ]; then
    BEST_SZ="$sz"
    BEST_BAUD="$baud"
    BEST_ATTEMPT="$attempt"
    BEST_DEV="$dev"
    cp -f "$resp_file" /tmp/at_resp.bin
  fi

  if [ "$BEST_SZ" -gt 0 ]; then
    return 0
  fi

  return 1
}

echo "" | tee -a $LOG
echo "=== AT probe sweep (8N1) ===" | tee -a $LOG
echo "Device sweep: $DEV_LIST" | tee -a $LOG

for dev in $DEV_LIST; do
  for baud in $BAUD_LIST; do
    for attempt in $(seq 1 "$ATTEMPTS_PER_BAUD"); do
      if probe_at_once "$dev" "$baud" "$attempt"; then
        break 3
      fi
    done
  done
done

echo "AT response size = $BEST_SZ bytes" | tee -a $LOG
if [ "$BEST_SZ" -gt 0 ]; then
  echo "AT response selected from dev=$BEST_DEV baud=$BEST_BAUD attempt=$BEST_ATTEMPT" | tee -a $LOG
  xxd /tmp/at_resp.bin | head -20 | tee -a $LOG
fi

echo "" | tee -a $LOG
echo "=== done ===" | tee -a $LOG
