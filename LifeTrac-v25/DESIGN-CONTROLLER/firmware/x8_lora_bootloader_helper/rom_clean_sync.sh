#!/bin/bash
# rom_clean_sync.sh — pulse PF4 NRST (with PWM4 BOOT0=high held by launcher
# state), then send a SINGLE 0x7F sync at 19200 8E1 and read the first byte.
# Result decoder:
#   0x79 = ACK   → ROM alive, autobauded cleanly. Ready for commands.
#   0x1F = NACK  → ROM alive but already locked / saw garbage first.
#   0xFE = parity error on host side → wrong framing.
#   <none>       → no L072 response (PA11 not high or USART2 dead).
#
# IMPORTANT: assumes BOOT0 (PA11) was already driven HIGH by an earlier run of
# swd_bypass_pa11_pf4_launcher.sh. We only re-pulse NRST here.

set -u
DEV=/dev/ttymxc3
LOG=/tmp/lifetrac_p0c/rom_clean_sync.log
mkdir -p /tmp/lifetrac_p0c
: > "$LOG"

GPIO=163

# 1. Force-assert PWM4 HIGH (BOOT0 high before NRST release).
PWMCHIP=/sys/class/pwm/pwmchip0
if ! [ -d $PWMCHIP/pwm4 ]; then
    echo 4 > $PWMCHIP/export 2>/dev/null || true
fi
PWM=$PWMCHIP/pwm4
echo 0 > $PWM/enable 2>/dev/null || true
echo 1000000  > $PWM/period
echo 999999   > $PWM/duty_cycle
echo 1        > $PWM/enable
EN=$(cat $PWM/enable); DUTY=$(cat $PWM/duty_cycle); PER=$(cat $PWM/period)
echo "PWM4: enable=$EN period=$PER duty=$DUTY  (BOOT0=HIGH)" | tee -a "$LOG"
sleep 0.05

# 2. Verify gpio-163 (PF4/NRST) is exported.
if ! [ -d /sys/class/gpio/gpio$GPIO ]; then
    echo "$GPIO" > /sys/class/gpio/export 2>/dev/null || true
fi
echo "out" > /sys/class/gpio/gpio$GPIO/direction
echo "1"   > /sys/class/gpio/gpio$GPIO/value

# 3. Configure UART: 19200 8E1 (ROM expects EVEN parity).
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo -ixon -ixoff -inpck -istrip 2>&1 | tee -a "$LOG"

# 4. Drain anything pending.
rm -f /tmp/rcs_drain.bin
timeout 1 cat $DEV > /tmp/rcs_drain.bin 2>/dev/null || true
echo "pre-drain bytes=$(stat -c %s /tmp/rcs_drain.bin)" | tee -a "$LOG"

# 5. Pulse NRST low for 50ms.
echo "0" > /sys/class/gpio/gpio$GPIO/value
sleep 0.05
echo "1" > /sys/class/gpio/gpio$GPIO/value
# 6. Wait ~80ms for ROM to start its autobaud loop.
sleep 0.08

# 7. Drain again — there may be reset glitch bytes.
rm -f /tmp/rcs_drain2.bin
timeout 0.3 cat $DEV > /tmp/rcs_drain2.bin 2>/dev/null || true
echo "post-reset drain bytes=$(stat -c %s /tmp/rcs_drain2.bin)" | tee -a "$LOG"
xxd /tmp/rcs_drain2.bin | head -4 | tee -a "$LOG"

# 8. Start receiver, send ONE 0x7F, wait 0.5s.
rm -f /tmp/rcs_resp.bin
cat $DEV > /tmp/rcs_resp.bin &
PID=$!
sleep 0.1
printf '\x7f' > $DEV
sleep 0.5
kill -9 $PID 2>/dev/null; wait $PID 2>/dev/null

SZ=$(stat -c %s /tmp/rcs_resp.bin)
echo "" | tee -a "$LOG"
echo "=== response to single 0x7F ===" | tee -a "$LOG"
echo "size=$SZ" | tee -a "$LOG"
xxd /tmp/rcs_resp.bin | tee -a "$LOG"

if [ "$SZ" -gt 0 ]; then
    FIRST=$(xxd -p -l 1 /tmp/rcs_resp.bin)
    case "$FIRST" in
        79) echo "RESULT: ACK (0x79) — ROM bootloader alive and ready." | tee -a "$LOG"; exit 0 ;;
        1f) echo "RESULT: NACK (0x1F) — ROM alive but rejected sync (already autobauded?)." | tee -a "$LOG"; exit 10 ;;
        fe) echo "RESULT: 0xFE — host saw parity/framing error." | tee -a "$LOG"; exit 11 ;;
        *)  echo "RESULT: unexpected first byte 0x$FIRST." | tee -a "$LOG"; exit 12 ;;
    esac
else
    echo "RESULT: NO RESPONSE — PA11 not really high, or USART2 dead." | tee -a "$LOG"
    exit 20
fi
