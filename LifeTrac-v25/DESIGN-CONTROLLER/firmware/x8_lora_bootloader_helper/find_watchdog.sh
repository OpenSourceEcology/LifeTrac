#!/bin/sh
echo "=== fuser watchdogs ==="
fuser /dev/watchdog0 /dev/watchdog 2>&1
echo "=== systemd watchdog config ==="
grep -E "RuntimeWatchdog|ShutdownWatchdog" /etc/systemd/system.conf 2>/dev/null
systemctl show -p RuntimeWatchdogUSec -p RebootWatchdogUSec 2>/dev/null
echo "=== anything with 'watchdog' in cmdline ==="
ps -ef | grep -i watchdog | grep -v grep
echo "=== known X8 services touching x8h7 ==="
systemctl list-units --type=service --state=running --no-pager 2>/dev/null | grep -iE "m4|stm32|x8h7|monitor|forwarder|arduino"
echo "=== monitor-m4-elf-file path unit ==="
systemctl status monitor-m4-elf-file.path --no-pager 2>&1 | head -15
echo "=== programs holding /dev/ttyX0 (x8h7_uart) ==="
fuser /dev/ttyX0 2>&1
