#!/bin/sh
# swd_bypass_pa11_pf4_launcher.sh
#
# Drive the L072's BOOT0 (= H7 PA11) and NRST (= H7 PF4) via the x8h7 SPI
# bridge instead of via SWD/OpenOCD. Used to enter L072 ROM bootloader on
# X8 boards whose SWD bus is dead (e.g. Board 1 — DPIDR reads 0xffffffff).
#
# Functional equivalent of 07_assert_pa11_pf4_long.cfg, but uses:
#   - /sys/class/pwm/pwmchip0/pwm4   (x8h7_pwm channel 4 -> PWM_pinmap[4] = PA11)
#   - /sys/class/gpio/gpio163        (x8h7_gpio offset 3 + base 160 -> PF4)
#
# libgpiod tools (gpioset/gpioget) are NOT installed on the LmP image, so
# this uses the legacy sysfs GPIO interface.
#
# See: LifeTrac-v25/AI NOTES/2026-05-13_W2-01_SWD_Bypass_Launcher_Attempt_v1_0.md
#
# Environment knobs:
#   HOLD_S            seconds to hold L072 in bootloader (default 600)
#   NRST_LOW_MS       NRST low-pulse duration in ms (default 250)
#   POST_RST_HOLD_MS  delay after NRST release before returning (default 200)
#   SUDO_PASS         password for sudo -S (default 'fio' = bench convention)
#   UNEXPORT_GPIO=1   unexport gpio163 on cleanup (default keep exported)
#
# Exit codes: 0 ok, 1 env err, 2 PWM fail, 3 GPIO fail.

set -u

# ---------- config ----------
HOLD_S="${HOLD_S:-600}"
NRST_LOW_MS="${NRST_LOW_MS:-250}"
POST_RST_HOLD_MS="${POST_RST_HOLD_MS:-200}"
SUDO_PASS="${SUDO_PASS:-fio}"

PWMCHIP_DIR="/sys/class/pwm/pwmchip0"
PWM_CH=4                                 # H7 PA11 = PWM_pinmap[4]
PWM_DIR="${PWMCHIP_DIR}/pwm${PWM_CH}"
PWM_PERIOD_NS=1000000                    # 1 ms
PWM_DUTY_NS=1000000                      # 100% duty -> constant high

GPIO_BASE=160                            # x8h7_gpio base
NRST_OFFSET=3                            # H7 PF4 = GPIO_pinmap[3]
NRST_GPIO=$((GPIO_BASE + NRST_OFFSET))   # = 163
NRST_DIR="/sys/class/gpio/gpio${NRST_GPIO}"

# ---------- helpers ----------
log() { printf '[swd-bypass %s] %s\n' "$(date +%H:%M:%S)" "$*" >&2; }

sudo_() {
  if [ "$(id -u)" = "0" ]; then
    "$@"
  else
    echo "$SUDO_PASS" | sudo -S -p '' "$@"
  fi
}

# Write a literal value to a sysfs path. When non-root, sudo's password
# consumption must NOT eat the data, so password is sent as the first stdin
# line and the value as the second; sudo forwards the rest to tee.
sudo_write() {  # sudo_write VALUE PATH
  v="$1"; p="$2"
  if [ "$(id -u)" = "0" ]; then
    printf '%s\n' "$v" > "$p"
  else
    printf '%s\n%s\n' "$SUDO_PASS" "$v" | sudo -S -p '' tee "$p" >/dev/null
  fi
}

cleanup() {
  log "cleanup: disabling PWM, releasing NRST"
  if [ -d "$PWM_DIR" ]; then
    sudo_write 0 "$PWM_DIR/enable" 2>/dev/null || true
  fi
  if [ -d "$NRST_DIR" ]; then
    sudo_write 1 "$NRST_DIR/value" 2>/dev/null || true
    if [ "${UNEXPORT_GPIO:-0}" = "1" ]; then
      sudo_write "$NRST_GPIO" /sys/class/gpio/unexport 2>/dev/null || true
    fi
  fi
}
trap cleanup EXIT INT TERM

ms_sleep() {
  s="$1"
  awk_s=$(awk "BEGIN{printf \"%.3f\", $s/1000}")
  sleep "$awk_s" 2>/dev/null || sleep 1
}

# ---------- preflight ----------
[ -d "$PWMCHIP_DIR" ] || { log "FATAL: $PWMCHIP_DIR missing -- is x8h7_pwm loaded?"; exit 2; }
[ -d /sys/class/gpio ] || { log "FATAL: /sys/class/gpio missing"; exit 3; }

log "config: HOLD_S=$HOLD_S NRST_LOW_MS=$NRST_LOW_MS POST_RST_HOLD_MS=$POST_RST_HOLD_MS"
log "PWM: $PWM_DIR (period=$PWM_PERIOD_NS duty=$PWM_DUTY_NS)"
log "NRST: gpio${NRST_GPIO} (= base ${GPIO_BASE} + offset ${NRST_OFFSET} = PF4)"

sudo_ true 2>/dev/null || { log "FATAL: sudo failed (check SUDO_PASS)"; exit 1; }

# ---------- step 1: drive PA11 (BOOT0) HIGH via PWM 100% duty ----------
if [ ! -d "$PWM_DIR" ]; then
  log "exporting PWM channel $PWM_CH"
  sudo_write "$PWM_CH" "$PWMCHIP_DIR/export" || { log "FATAL: pwm export failed"; exit 2; }
  for _ in 1 2 3 4 5; do
    [ -d "$PWM_DIR" ] && break
    ms_sleep 100
  done
  [ -d "$PWM_DIR" ] || { log "FATAL: $PWM_DIR did not appear"; exit 2; }
fi

# Read current period; if 0 (freshly exported) we MUST set period before any
# duty / enable write, otherwise pwm-core returns EINVAL.
CUR_PERIOD=$(cat "$PWM_DIR/period" 2>/dev/null || echo 0)
if [ "$CUR_PERIOD" -eq 0 ]; then
  log "fresh PWM channel (period=0); setting period first"
  sudo_write "$PWM_PERIOD_NS" "$PWM_DIR/period"
else
  # already configured: disable first so we can rewrite duty/period in any order
  sudo_write 0 "$PWM_DIR/enable" 2>/dev/null || true
  sudo_write 0 "$PWM_DIR/duty_cycle" 2>/dev/null || true
  sudo_write "$PWM_PERIOD_NS" "$PWM_DIR/period"
fi
# duty must be <= period; use period-1 to satisfy strict-less-than checks.
DUTY_SAFE=$((PWM_DUTY_NS < PWM_PERIOD_NS ? PWM_DUTY_NS : PWM_PERIOD_NS - 1))
sudo_write "$DUTY_SAFE"  "$PWM_DIR/duty_cycle"
sudo_write normal        "$PWM_DIR/polarity"   2>/dev/null || true
sudo_write 1             "$PWM_DIR/enable"

log "PA11 driven HIGH (BOOT0 asserted; period=$PWM_PERIOD_NS duty=$DUTY_SAFE)"
ms_sleep 50

# ---------- step 2: export PF4 GPIO and pulse LOW -> HIGH ----------
if [ ! -d "$NRST_DIR" ]; then
  log "exporting gpio${NRST_GPIO} (PF4)"
  # may EINVAL with 'already exported' on retries -- tolerate, then verify dir.
  sudo_write "$NRST_GPIO" /sys/class/gpio/export 2>/dev/null || true
  for _ in 1 2 3 4 5; do
    [ -d "$NRST_DIR" ] && break
    ms_sleep 100
  done
  [ -d "$NRST_DIR" ] || { log "FATAL: $NRST_DIR did not appear"; exit 3; }
  ms_sleep 100
fi

# Drive low (sets direction=out and value=0 atomically)
if ! sudo_write low "$NRST_DIR/direction" 2>/dev/null; then
  log "direction=low not supported, falling back to out + value"
  sudo_write out "$NRST_DIR/direction"
  sudo_write 0   "$NRST_DIR/value"
fi

log "NRST asserted low for ${NRST_LOW_MS} ms"
ms_sleep "$NRST_LOW_MS"

sudo_write 1 "$NRST_DIR/value"
log "NRST released high"

ms_sleep "$POST_RST_HOLD_MS"

# ---------- step 3: hold for HOLD_S ----------
log "holding bootloader for ${HOLD_S} s"
log "ready: PA11=HIGH PF4=HIGH -- run flasher / probe ttymxc3 now"

i=0
while [ "$i" -lt "$HOLD_S" ]; do
  sleep 1
  i=$((i + 1))
done

log "hold window expired -- cleaning up"
exit 0
