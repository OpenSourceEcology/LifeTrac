#!/bin/sh
# Comprehensive X8 + Max Carrier health check (read-only).
echo "==== UPTIME ===="
uptime
echo "==== KERNEL ===="
uname -a
echo "==== OS-RELEASE ===="
grep -E '^(NAME|VERSION|PRETTY_NAME)=' /etc/os-release 2>/dev/null
echo "==== UPTIME-PROC ===="
cat /proc/uptime
echo "==== DMESG (errors/warnings, last boot) ===="
dmesg -T -l err,warn 2>/dev/null | tail -40
echo "==== X8H7 MODULES ===="
lsmod | grep -E '^x8h7|^Module' | sort
echo "==== X8H7 FW VERSION (5s timeout) ===="
timeout 5 cat /sys/kernel/x8h7_firmware/version 2>&1 || echo "<read failed/timeout>"
echo "==== M4_PROXY ===="
systemctl is-active m4-proxy.service 2>/dev/null
systemctl is-active stm32h7-program.service 2>/dev/null
echo "==== GPIO CHIPS ===="
ls /sys/class/gpio/ | grep -E '^gpiochip' | sort
echo "==== EXPORTED GPIOS ===="
ls /sys/class/gpio/ | grep -E '^gpio[0-9]+$' | sort
echo "==== SWD-RELEVANT GPIOS (8/10/15) STATE ===="
for g in 8 10 15; do
  if [ -d /sys/class/gpio/gpio$g ]; then
    d=$(cat /sys/class/gpio/gpio$g/direction 2>/dev/null)
    v=$(cat /sys/class/gpio/gpio$g/value 2>/dev/null)
    echo "gpio$g dir=$d val=$v"
  else
    echo "gpio$g not exported"
  fi
done
echo "==== /dev/ttymxc3 PRESENT ===="
ls -la /dev/ttymxc3 2>&1
echo "==== USB DEVICES ===="
ls /sys/bus/usb/devices/ 2>/dev/null | sort
echo "==== USB IDs ===="
for d in /sys/bus/usb/devices/*/idVendor; do
  vid=$(cat "$d" 2>/dev/null)
  pid=$(cat "$(dirname $d)/idProduct" 2>/dev/null)
  prod=$(cat "$(dirname $d)/product" 2>/dev/null)
  echo "$(basename $(dirname $d)) ${vid}:${pid} $prod"
done
echo "==== WATCHDOG ===="
ls /dev/watchdog* 2>&1
echo "==== DISK ===="
df -h / /var 2>/dev/null
echo "==== MEMORY ===="
free -m
echo "==== LOAD ===="
cat /proc/loadavg
echo "==== L072 ROM-MODE PROBE (non-destructive) ===="
# 2026-05-13 discovery: ROM auto-bauds on the FIRST byte after NRST release.
# A glitch byte (most likely from cs42l52 i2c retry traffic) consumes the
# auto-baud slot, so sending 0x7F NACKs (0x1F). Skip 0x7F and send GET
# directly. NOTE: only meaningful if PA11 (BOOT0) was driven high before
# NRST release — which we DO NOT do here. We just look at the existing
# state. Three outcomes are interesting:
#   (a) ACK + payload → chip already in ROM mode (BOOT0 was somehow high)
#   (b) NACK 0x1F     → chip in ROM but auto-baud already consumed
#   (c) silence       → chip is running user firmware (expected normal state)
if [ -c /dev/ttymxc3 ]; then
  stty -F /dev/ttymxc3 19200 cs8 parenb -parodd -cstopb raw -echo -ixon -ixoff -inpck -istrip 2>/dev/null
  rm -f /tmp/_hc_l072_rom.bin
  cat /dev/ttymxc3 > /tmp/_hc_l072_rom.bin 2>/dev/null &
  HCPID=$!
  sleep 0.1
  printf '\x00\xff' > /dev/ttymxc3 2>/dev/null
  sleep 0.4
  kill -9 $HCPID 2>/dev/null; wait $HCPID 2>/dev/null
  SZ=$(stat -c %s /tmp/_hc_l072_rom.bin 2>/dev/null || echo 0)
  if [ "$SZ" = "0" ]; then
    echo "L072 GET probe: silent (chip running user firmware — normal)"
  else
    FIRST=$(xxd -p -l 1 /tmp/_hc_l072_rom.bin 2>/dev/null)
    case "$FIRST" in
      79) echo "L072 GET probe: ACK ($SZ bytes) — chip is in ROM bootloader mode!" ;;
      1f) echo "L072 GET probe: NACK 0x1F — chip in ROM but auto-baud consumed (rerun via flash_l072_via_uart.py)" ;;
      *)  echo "L072 GET probe: unexpected first byte 0x$FIRST ($SZ bytes)" ;;
    esac
    xxd /tmp/_hc_l072_rom.bin | head -2
  fi
else
  echo "/dev/ttymxc3 not present — skipping L072 probe"
fi
echo "==== L072 OPT BYTES (read-only; only meaningful if ROM is reachable) ===="
# 2026-05-14: Board 2 was found booting silently with corrupted OPT bytes
# (USER=0x55, WRPROT1=0x8070). Capturing OPT bytes here surfaces the same
# regression on any board automatically. Requires PA11 HIGH + NRST to enter
# ROM — we DO NOT do that here (read-only). If the GET probe above already
# returned ACK, the chip is in ROM and this read will succeed; otherwise
# it will time out and we'll print SKIP.
if [ -c /dev/ttymxc3 ] && [ -f /tmp/flash_l072_via_uart.py ]; then
  if [ "$FIRST" = "79" ] 2>/dev/null; then
    timeout 5 python3 /tmp/flash_l072_via_uart.py read 0x1FF80000 32 2>&1 | head -10
    echo "Expected default: 1ff80000  aa 00 ff 00 00 00 ff ff 00 00 ff ff 00 00 ff ff"
  else
    echo "SKIP: chip not in ROM mode (run recover_l072_opt.sh to enter ROM + read OPT)"
  fi
else
  echo "SKIP: /tmp/flash_l072_via_uart.py not present"
fi
echo "==== END ===="
