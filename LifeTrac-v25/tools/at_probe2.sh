#!/bin/sh
# at_probe2.sh — disambiguation probe for the Murata UART firmware
# Findings from probe1: alive @19200 8N1, emitting "Error when receiving\n+ERR_RX\r" continuously.
# Goal: determine whether host->modem TX path works at all.
set -u
TTY=/dev/ttymxc3
GPIO=163
TMP=/tmp/at_probe2_capture.txt

reset_modem() {
    [ -d /sys/class/gpio/gpio${GPIO} ] || echo ${GPIO} > /sys/class/gpio/export 2>/dev/null
    echo out > /sys/class/gpio/gpio${GPIO}/direction 2>/dev/null
    echo 0 > /sys/class/gpio/gpio${GPIO}/value 2>/dev/null
    sleep 0.2
    echo 1 > /sys/class/gpio/gpio${GPIO}/value 2>/dev/null
    sleep 1
}

set_19200_8n1() {
    stty -F ${TTY} 19200 cs8 -parenb -parodd -cstopb \
        -ixon -ixoff -crtscts -icrnl -ocrnl -opost -isig -icanon -echo -echoe \
        min 0 time 5 2>/dev/null
}

trial() {
    LABEL="$1"; shift
    echo "==== TRIAL: ${LABEL} ===="
    reset_modem
    set_19200_8n1
    : > ${TMP}
    timeout 3 cat ${TTY} >> ${TMP} &
    R=$!
    sleep 0.2
    "$@"
    sleep 0.5
    wait $R 2>/dev/null
    echo "-- captured (last ~30 lines) --"
    tr -d '\000' < ${TMP} | tail -c 800 | strings -n 2
    echo "-- byte count: $(wc -c < ${TMP}) --"
    echo
}

# Trial A: do nothing — baseline URC rate
trial "A_baseline_silent" sh -c "true"

# Trial B: send AT with CRLF
trial "B_AT_crlf" sh -c "printf 'AT\r\n' > ${TTY}"

# Trial C: send AT with CR only (Hayes classic)
trial "C_AT_cr" sh -c "printf 'AT\r' > ${TTY}"

# Trial D: send +++ then AT (Hayes escape)
trial "D_plusplusplus_AT" sh -c "printf '+++' > ${TTY}; sleep 1.2; printf 'AT\r\n' > ${TTY}"

# Trial E: send the Arduino MKRWAN P2P command set candidates
trial "E_mkrwan_p2p" sh -c "
for c in 'AT' 'ATI' 'AT+VER' 'AT+VER?' 'AT+P2P_VER' 'AT+P2P_VER?' \\
         'AT+RFCFG?' 'AT+RFCFG=868000000,7,125,1,8,14,1,0,0,1,0,0,3000,0,0' \\
         'AT+SEND=hello' 'AT+TX=hello' 'AT+UTX=5' 'AT+RX=1' 'AT+MODE?' 'AT+P2P?' ; do
  printf '%s\r\n' \"\$c\" > ${TTY}
  sleep 0.15
done"

# Trial F: send a single byte 0x00 to test framing (some modems use length-prefix binary)
trial "F_null_byte" sh -c "printf '\\0' > ${TTY}"

# Trial G: send the Semtech AT_Slave canonical 'AT?'
trial "G_at_question" sh -c "printf 'AT?\r\n' > ${TTY}"

# Trial H: hold the line at break (some bootloaders enter on BREAK)
trial "H_break" sh -c "stty -F ${TTY} 0; sleep 0.4; stty -F ${TTY} 19200 cs8 -parenb -cstopb -echo; printf 'AT\r\n' > ${TTY}"

echo "DONE_PROBE2"
