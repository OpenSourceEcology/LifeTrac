# === FCC-B2-b ARTIFACT HEADER BEGIN (v1) ===
# firmware_git_sha: d4dfcb86cfd99bbcbd227844940a1f905336b356
# firmware_git_sha_short: d4dfcb86cfd9
# build_timestamp_utc: 2026-05-20T07:49:56Z
# profile_enum: 0
# profile_string: REG_PROFILE_BENCH_ONLY_FIXED_915
# rfco_summary_schema_ver: 1
# rfco_pertx_schema_ver: 1
# header_schema_ver: 1
# === FCC-B2-b ARTIFACT HEADER END ===

#!/bin/bash
set +e
pkill -9 -f 'cat /dev/ttymxc3' 2>/dev/null
pkill -9 openocd 2>/dev/null
sleep 0.3
cd /tmp/lifetrac_p0c
echo '=== reset L072 ==='
openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -f 08_boot_user_app.cfg 2>&1 | tail -5
stty -F /dev/ttymxc3 raw 921600 cs8 -parenb -cstopb -ixon -ixoff -crtscts
( timeout 5 cat /dev/ttymxc3 > /tmp/rx_test_capture.bin ) &
CATPID=$!
sleep 1.0
echo '=== bursts ==='
printf '\x00\x03\x01\x01\xab\xcd\x00' > /dev/ttymxc3
sleep 0.4
printf 'AT\r\n' > /dev/ttymxc3
sleep 0.4
printf '\x00\x03\x01\x01\xab\xcd\x00' > /dev/ttymxc3
wait $CATPID 2>/dev/null
echo '=== size ==='
wc -c /tmp/rx_test_capture.bin
echo '=== ASCII ==='
strings /tmp/rx_test_capture.bin | head -25
echo '=== hex ==='
xxd /tmp/rx_test_capture.bin
