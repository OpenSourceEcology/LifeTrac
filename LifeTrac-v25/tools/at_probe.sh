#!/bin/sh
# at_probe.sh — identify the Murata CMWX1ZZABZ-078 AT firmware on /dev/ttymxc3
# Run as root.  Tries 19200 8N1, 19200 8N2, 115200 8N1.
# Reset line: GPIO163 (gpiochip5 line 3).
set -u

TTY=/dev/ttymxc3
GPIO=163
TMPOUT=/tmp/at_probe_capture.txt

reset_modem() {
    if [ ! -d /sys/class/gpio/gpio${GPIO} ]; then
        echo ${GPIO} > /sys/class/gpio/export 2>/dev/null || true
    fi
    echo out > /sys/class/gpio/gpio${GPIO}/direction 2>/dev/null || true
    echo 0 > /sys/class/gpio/gpio${GPIO}/value 2>/dev/null || true
    sleep 0.2
    echo 1 > /sys/class/gpio/gpio${GPIO}/value 2>/dev/null || true
    sleep 1
}

probe_at() {
    BAUD=$1
    STOP=$2  # "" for 8N1, "cstopb" for 8N2
    echo "================================================================"
    echo "Probe: ${BAUD} 8N${STOP:+2}${STOP:+ (cstopb)}${STOP:- (8N1)}"
    echo "================================================================"

    reset_modem
    stty -F ${TTY} ${BAUD} cs8 -parenb -parodd ${STOP:--cstopb} \
        -ixon -ixoff -crtscts -icrnl -ocrnl -opost -isig -icanon -echo -echoe \
        min 0 time 5 2>/dev/null

    : > ${TMPOUT}
    # Background reader for ~1.5s
    timeout 1.5 cat ${TTY} >> ${TMPOUT} &
    READER=$!
    sleep 0.1

    for CMD in "AT" "AT" "AT+VER?" "AT+VER" "ATI" "AT+DEV?" "AT+DEV" \
               "AT+APPEUI?" "AT+APPEUI" "AT+CGMI" "AT+CGMM" "AT+CGMR" \
               "AT+MODE=?" "AT+CLASS=?" "AT+REG=?"; do
        printf "%s\r\n" "${CMD}" > ${TTY}
        sleep 0.15
    done

    sleep 0.3
    wait ${READER} 2>/dev/null
    echo "--- captured ${BAUD} ---"
    od -c ${TMPOUT} | head -40
    echo "--- printable ---"
    tr -d '\000' < ${TMPOUT} | strings -n 2
    echo
}

probe_at 19200 ""
probe_at 19200 "cstopb"
probe_at 115200 ""
probe_at 57600 ""

echo "DONE"
