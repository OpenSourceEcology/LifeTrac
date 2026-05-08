#!/bin/sh
# verify_l072_rom.sh — run on X8 in a SECOND shell while openocd holds H7 halted
# with PG_7 asserted. Confirms whether the L072 entered ROM bootloader.
#
# (Same script as the earlier %TEMP% version, copied here so it travels with
# the openocd toolkit when this directory is pushed to /tmp/lifetrac_p0c.)
set -e
DEV=/dev/ttymxc3

echo "--- AT probe at 19200 8N1 (ROM bootloader will be silent) ---"
stty -F $DEV 19200 cs8 -parenb -cstopb raw -echo -echoe -echok -ixon -ixoff -icrnl -onlcr time 5 min 0
( cat $DEV > /tmp/at_resp.bin & ) ; CATPID=$!
sleep 0.2
printf 'AT\r\n' > $DEV
sleep 1
kill $CATPID 2>/dev/null || true
wait 2>/dev/null || true
SZ=$(stat -c %s /tmp/at_resp.bin 2>/dev/null || echo 0)
echo "AT_RESP_SIZE=$SZ"
[ "$SZ" -gt 0 ] && xxd /tmp/at_resp.bin | head -5

echo
echo "--- ROM bootloader sync at 19200 8E1 (expect 0x79 ACK) ---"
stty -F $DEV 19200 cs8 parenb -parodd -cstopb raw -echo -echoe -echok -ixon -ixoff -icrnl -onlcr time 5 min 0
( cat $DEV > /tmp/rom_resp.bin & ) ; CATPID=$!
sleep 0.2
printf '\x7f' > $DEV
sleep 1
kill $CATPID 2>/dev/null || true
wait 2>/dev/null || true
SZ=$(stat -c %s /tmp/rom_resp.bin 2>/dev/null || echo 0)
echo "ROM_RESP_SIZE=$SZ"
[ "$SZ" -gt 0 ] && xxd /tmp/rom_resp.bin | head -5

echo
echo "--- verdict ---"
if [ "$SZ" -gt 0 ]; then
  FIRST=$(head -c 1 /tmp/rom_resp.bin | xxd -p)
  echo "First byte after 0x7F: 0x$FIRST"
  case "$FIRST" in
    79) echo "RESULT: ROM bootloader ACK (0x79) - L072 IS in System Memory bootloader." ;;
    1f) echo "RESULT: ROM bootloader NACK (0x1F) - bootloader present but rejected sync." ;;
    *)  echo "RESULT: Unexpected response - review hex above." ;;
  esac
else
  echo "RESULT: silent on 0x7F - BOOT0 not actually asserted on the L072 net."
fi
