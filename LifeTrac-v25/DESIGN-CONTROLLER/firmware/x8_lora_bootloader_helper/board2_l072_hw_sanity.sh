#!/bin/sh
# X8-side sanity checks for the Board 2 Murata L072 control path.
# Run on the Portenta X8 as root: sh /tmp/board2_l072_hw_sanity.sh
set -u

PWMCHIP=/sys/class/pwm/pwmchip0
PWM_DIR=$PWMCHIP/pwm4
NRST_GPIO=163
NRST_DIR=/sys/class/gpio/gpio$NRST_GPIO
DEV=/dev/ttymxc3
PERIOD=1000000

say() { printf '%s\n' "$*"; }

ensure_controls() {
    [ -d "$PWM_DIR" ] || echo 4 > "$PWMCHIP/export"
    sleep 0.1
    echo 0 > "$PWM_DIR/enable" 2>/dev/null || true
    echo "$PERIOD" > "$PWM_DIR/period"

    [ -d "$NRST_DIR" ] || echo "$NRST_GPIO" > /sys/class/gpio/export
    sleep 0.1
    echo out > "$NRST_DIR/direction"
}

boot0_low() {
    echo 0 > "$PWM_DIR/enable" 2>/dev/null || true
    echo "$PERIOD" > "$PWM_DIR/period"
    echo 0 > "$PWM_DIR/duty_cycle"
    echo 1 > "$PWM_DIR/enable"
}

boot0_high() {
    echo 0 > "$PWM_DIR/enable" 2>/dev/null || true
    echo "$PERIOD" > "$PWM_DIR/period"
    echo "$PERIOD" > "$PWM_DIR/duty_cycle"
    echo 1 > "$PWM_DIR/enable"
}

pulse_nrst() {
    echo 0 > "$NRST_DIR/value"
    sleep "$1"
    echo 1 > "$NRST_DIR/value"
}

print_state() {
    say "--- control state ---"
    printf 'BOOT0 PWM enable='; cat "$PWM_DIR/enable" 2>/dev/null || true
    printf 'BOOT0 PWM period='; cat "$PWM_DIR/period" 2>/dev/null || true
    printf 'BOOT0 PWM duty_cycle='; cat "$PWM_DIR/duty_cycle" 2>/dev/null || true
    printf 'NRST gpio direction='; cat "$NRST_DIR/direction" 2>/dev/null || true
    printf 'NRST gpio value='; cat "$NRST_DIR/value" 2>/dev/null || true
    ls -l "$DEV" 2>/dev/null || true
}

listen_user_fw() {
    OUT=/tmp/_sanity_user921k.bin
    rm -f "$OUT"
    boot0_low
    pulse_nrst 0.25
    sleep 0.5
    stty -F "$DEV" 921600 raw -echo -parenb cs8 -cstopb
    timeout 5 cat "$DEV" > "$OUT" 2>/dev/null
    printf 'USER_FW_921600_BYTES='
    wc -c < "$OUT"
    xxd "$OUT" | head -8
}

probe_rom() {
    OUT=/tmp/_sanity_rom19200.bin
    rm -f "$OUT"
    boot0_high
    pulse_nrst 0.5
    sleep 0.2
    stty -F "$DEV" 19200 raw -echo parenb -parodd cs8 -cstopb
    cat "$DEV" > "$OUT" 2>/dev/null &
    READER=$!
    sleep 0.1
    printf '\177\000\377' > "$DEV"
    sleep 1.2
    kill "$READER" 2>/dev/null || true
    wait "$READER" 2>/dev/null || true
    printf 'ROM_19200_8E1_BYTES='
    wc -c < "$OUT"
    xxd "$OUT" | head -8
}

say "=== Board 2 L072 X8-side hardware sanity ==="
date || true
uname -a || true
uptime || true
ensure_controls
print_state

say "=== reset pulse visibility check ==="
echo 1 > "$NRST_DIR/value"
printf 'NRST before pulse='; cat "$NRST_DIR/value"
echo 0 > "$NRST_DIR/value"
printf 'NRST while held low='; cat "$NRST_DIR/value"
sleep 0.25
echo 1 > "$NRST_DIR/value"
printf 'NRST after release='; cat "$NRST_DIR/value"

say "=== BOOT0 LOW / user firmware listen ==="
listen_user_fw
print_state

say "=== BOOT0 HIGH / ROM autobaud probe ==="
probe_rom
print_state

say "=== leave board in BOOT0 LOW, NRST HIGH ==="
boot0_low
echo 1 > "$NRST_DIR/value"
print_state
say "=== done ==="