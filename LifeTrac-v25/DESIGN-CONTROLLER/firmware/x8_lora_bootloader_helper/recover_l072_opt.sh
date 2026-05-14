#!/bin/bash
# recover_l072_opt.sh — recover STM32L072 from corrupted Option Bytes via
# AN3155 ROM bootloader on /dev/ttymxc3.
#
# Background: see LifeTrac-v25/AI NOTES/2026-05-14_W2-02_Board2_L072_OPT_Bytes_Root_Cause_Copilot_v1_0.md
#
# Procedure (matches stm32flash semantics):
#   1) PA11=HIGH, NRST pulse → ROM
#   2) Read OPT bytes at 0x1FF80000 (baseline)
#   3) Send AN3155 cmd 0x73 (WRITE_UNPROTECT) — clears WRPROT1/2, chip auto-resets
#   4) PA11 still HIGH, re-pulse NRST → ROM (so we can verify)
#   5) Read OPT bytes again — verify WRP cleared
#   6) If --runprot or USER byte still bad: send AN3155 cmd 0x92 (READOUT_UNPROTECT)
#      — RDP→AA + mass erase + OPT defaults + reset
#   7) Read OPT bytes a final time to confirm
#
# WARNING: cmd 0x92 mass-erases user firmware.
# WARNING: DO NOT use cmd 0x73 (wunprot). On L072 bootloader v3.1, it corrupts
#          the Option Byte complements and permanently bricks the core!
#
# Usage:
#   recover_l072_opt.sh              # Defaults to runprot (destructive, but safe for hardware)
set -u

DEV=/dev/ttymxc3
FLASHER=/tmp/flash_l072_via_uart.py
LOG=/tmp/lifetrac_p0c/recover_l072_opt.log
mkdir -p /tmp/lifetrac_p0c
: > "$LOG"

DO_WUNPROT=0 # DEPRECATED: Cmd 0x73 Bricks STM32L072 (See AI Notes)
DO_RUNPROT=1 # Always default to 0x92 (destructive mass-erase) for safe recovery
for arg in "$@"; do
    case "$arg" in
        --force-wunprot) DO_WUNPROT=1 ;; # Hidden flag if ever strictly needed
        --runprot|--runprot-only) DO_RUNPROT=1 ;;
        *) echo "unknown arg: $arg" >&2; exit 2 ;;
    esac
done

GPIO=163
PWMCHIP=/sys/class/pwm/pwmchip0
PWM=$PWMCHIP/pwm4
PERIOD=1000000

ensure_pwm() {
    [ -d $PWM ] || echo 4 > $PWMCHIP/export
    echo 0 > $PWM/enable 2>/dev/null || true
    echo $PERIOD > $PWM/period
}
boot0_high() { ensure_pwm; echo $PERIOD > $PWM/duty_cycle; echo 1 > $PWM/enable; }

ensure_nrst() {
    [ -d /sys/class/gpio/gpio$GPIO ] || echo "$GPIO" > /sys/class/gpio/export 2>/dev/null
    echo "out" > /sys/class/gpio/gpio$GPIO/direction
    echo "1"   > /sys/class/gpio/gpio$GPIO/value
}
pulse_nrst() {
    ensure_nrst
    echo "0" > /sys/class/gpio/gpio$GPIO/value
    sleep 0.05
    echo "1" > /sys/class/gpio/gpio$GPIO/value
}

enter_rom() {
    boot0_high
    pulse_nrst
    sleep 0.3
}

read_opt() {
    python3 $FLASHER read 0x1FF80000 32 2>&1 | tee -a "$LOG"
}

echo "=== STEP 1: enter ROM (PA11 HIGH + NRST pulse) ===" | tee -a "$LOG"
enter_rom

echo "=== STEP 2: baseline OPT bytes ===" | tee -a "$LOG"
read_opt

if [ "$DO_WUNPROT" = "1" ]; then
    echo "=== STEP 3: WRITE_UNPROTECT (cmd 0x73) ===" | tee -a "$LOG"
    enter_rom  # fresh probe so cmd state is clean
    python3 $FLASHER wunprot 2>&1 | tee -a "$LOG"
    sleep 1.0  # chip is auto-resetting

    echo "=== STEP 4: re-enter ROM and verify OPT bytes ===" | tee -a "$LOG"
    enter_rom
    read_opt
fi

if [ "$DO_RUNPROT" = "1" ]; then
    echo "=== STEP 5: READOUT_UNPROTECT (cmd 0x92) — DESTRUCTIVE mass erase ===" | tee -a "$LOG"
    enter_rom
    python3 $FLASHER runprot 2>&1 | tee -a "$LOG"
    sleep 2.0  # mass erase + reset

    echo "=== STEP 6: re-enter ROM and verify OPT bytes ===" | tee -a "$LOG"
    enter_rom
    read_opt
fi

echo "=== DONE — log: $LOG ===" | tee -a "$LOG"
echo "Expected post-recovery OPT bytes:" | tee -a "$LOG"
echo "  1ff80000  aa 00 ff 00 00 00 ff ff 00 00 ff ff 00 00 ff ff" | tee -a "$LOG"
echo "  (RDP=AA, USER=FF, WRPROT1/2=FFFF)" | tee -a "$LOG"
