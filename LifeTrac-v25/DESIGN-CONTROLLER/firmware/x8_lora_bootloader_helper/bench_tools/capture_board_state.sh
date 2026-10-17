#!/bin/sh
# capture_board_state.sh -- READ-ONLY inventory of one LifeTrac bench Portenta X8.
#
# First run on the base 2026-10-04 (bench-evidence/board_state_2026-10-04/). Run as root:
#   sudo -n sh /tmp/capture_board_state.sh <base|tractor>
#
# It writes ONLY under $OUT (default: tmpfs /tmp, gone at the next reboot) and prints
# CAPTURE_TGZ=<path> at the end for the PC to pull.
#
# It NEVER does any of these:
#   - systemctl start/stop/restart/enable/disable/mask
#   - docker run/start/stop/rm/pull/build/save
#   - openocd, or anything that opens /dev/ttymxc3 (the L072 radio UART)
#   - nmcli/rfkill changes, fw_setenv, modprobe/rmmod
#   - opening /dev/watchdog0 (section 5b only stats it: fuser reads /proc/*/fd, ls -l)
#   - writing /proc/sysrq-trigger or any /proc/sys value (section 5b only reads them)
#   - i2cget/i2cset/i2cdump or any other bus access to the PMIC (5b reads the device tree)
# Secrets are not copied: NetworkManager connection files, /var/sota keys, ssh host
# keys, shadow, the base's secrets/ dir and .env values are listed (name and size only)
# or redacted.
# shellcheck disable=SC2016 # run() takes sh -c strings; their $ expands in the child on purpose
set -u
PATH=/usr/sbin:/sbin:/usr/bin:/bin:$PATH
export PATH
TAG=${1:-board}
TS=$(date -u +%Y%m%dT%H%M%SZ)
OUT=${OUT:-/tmp/board_state_${TAG}_${TS}}
mkdir -p "$OUT/files" || exit 1
cd "$OUT" || exit 1

# run <name> <shell command...> -> $OUT/<name>.txt with the command, output and rc
run() {
  n=$1; shift
  { echo "\$ $*"; sh -c "$*" 2>&1; echo "[rc=$?]"; } >> "$OUT/$n.txt"
}
# keep a copy of a file under files/ with its absolute path (regular, readable, < 512 KB)
keep() {
  for f in "$@"; do
    [ -f "$f" ] && [ -r "$f" ] || continue
    sz=$(wc -c < "$f" 2>/dev/null || echo 0)
    [ "$sz" -lt 524288 ] || { echo "SKIP_BIG $f $sz" >> "$OUT/skipped.txt"; continue; }
    mkdir -p "$OUT/files$(dirname "$f")" && cp -p "$f" "$OUT/files$f"
  done
}
REDACT='s/\(\(KEY\|PASS\|PASSWORD\|TOKEN\|SECRET\|PIN\|PSK\)[A-Za-z0-9_]*=\)[^",]*/\1<redacted>/g'
DENY='shadow|gshadow|ssh_host_|_host_key|/dropbear/|machine-id|docker/key.json|bash_history|system-connections|wpa_supplicant.*\.conf|/sota/|\.pem$|\.key$|/secrets/|/\.env$|/\.ssh/'

echo "capture start $TS tag=$TAG" > "$OUT/00_meta.txt"

# ---- 1. identity, OS image, boot chain ----------------------------------------------
run 01_identity 'hostname; cat /etc/hostname; uname -a; uptime; date -u'
run 01_identity 'cat /etc/os-release'
run 01_identity 'cat /proc/cmdline'
run 01_identity 'cat /sys/devices/soc0/soc_id /sys/devices/soc0/revision /sys/devices/soc0/serial_number 2>/dev/null'
run 02_ostree   'ostree admin status'
run 02_ostree   'ostree admin config-diff'          # every /etc file changed vs the image default
run 03_bootenv  'cat /etc/fw_env.config; fw_printenv'  # read-only; shows bootargs edits (W2-01 Option A)
run 03_bootenv  'ls -la /boot/loader/entries 2>/dev/null; cat /boot/loader/entries/*.conf 2>/dev/null'
run 04_versions 'python3 --version; openocd --version; docker version; docker compose version'
run 04_versions 'df -h; free -m; cat /usr/lib/tmpfiles.d/tmp.conf 2>/dev/null'

# ---- 2. systemd: what is installed, enabled, masked ---------------------------------
run 10_systemd 'systemctl list-unit-files --no-pager'
run 10_systemd 'systemctl list-units --all --no-pager'
run 10_systemd 'systemctl --failed --no-pager; systemctl list-timers --all --no-pager'
run 11_masked  'find /etc/systemd/system -maxdepth 2 -lname /dev/null -printf "%p -> %l\n"'
run 11_masked  'ls -laR /etc/systemd/system'
for u in 'lifetrac-*' stm32h7-program.service m4-proxy.service monitor-m4-elf-file.path \
         monitor-m4-elf-file.service 'compose-apps-early-start*' aktualizr-lite.service \
         wpa_supplicant.service disable-wifi.service docker.service NetworkManager.service \
         adbd.service 'systemd-tmpfiles-clean.timer'; do
  run 12_unit_cat "systemctl cat --no-pager '$u'"
  run 12_unit_state "systemctl is-enabled '$u'; systemctl is-active '$u'"
done

# ---- 3. /etc changes (allowlisted copies; everything else name/size only) ------------
keep /etc/systemd/system/*.service /etc/systemd/system/*.path /etc/systemd/system/*.timer \
     /etc/systemd/system/*/*.conf /etc/udev/rules.d/* /etc/modprobe.d/* /etc/modules-load.d/* \
     /etc/sudoers.d/* /etc/docker/daemon.json /etc/NetworkManager/NetworkManager.conf \
     /etc/NetworkManager/conf.d/* /etc/tmpfiles.d/* /etc/sysctl.d/* /etc/fw_env.config \
     /etc/hosts /etc/hostname /etc/ssh/sshd_config /etc/systemd/system.conf /etc/systemd/journald.conf
ostree admin config-diff 2>/dev/null | awk '{print $2}' | while read -r p; do
  f=/etc/$p
  [ -e "$f" ] || { echo "DELETED $f" >> "$OUT/13_etc_changed.txt"; continue; }
  if echo "$f" | grep -Eq "$DENY"; then
    echo "SECRET_NOT_COPIED $(ls -ld "$f")" >> "$OUT/13_etc_changed.txt"
  else
    echo "CHANGED $(ls -ld "$f")" >> "$OUT/13_etc_changed.txt"
    [ -f "$f" ] && keep "$f"
  fi
done

# ---- 4. network, radios, users -------------------------------------------------------
run 20_network 'ip -br addr; ip route; ss -tlnp 2>/dev/null || netstat -tlnp'
run 20_network 'cat /sys/class/net/eth0/speed /sys/class/net/eth0/duplex 2>/dev/null; ethtool eth0 2>/dev/null'
run 20_network 'nmcli -t general status; nmcli -t device status; nmcli -t -f NAME,UUID,TYPE,DEVICE,AUTOCONNECT con show'
nmcli -t -f NAME,TYPE con show 2>/dev/null | while IFS=: read -r c t; do
  case "$t" in *ethernet*) run 20_network "nmcli -g connection.id,802-3-ethernet.speed,802-3-ethernet.duplex,802-3-ethernet.auto-negotiate con show '$c'";; esac
  case "$t" in *wireless*) run 20_network "nmcli -g connection.id,connection.autoconnect,802-11-wireless.ssid con show '$c'";; esac   # no --show-secrets
done
run 21_radios  'rfkill list; nmcli radio all; ls -la /etc/NetworkManager/system-connections 2>/dev/null'
run 22_users   'id fio; getent group docker video dialout sudo wheel'
run 22_users   'ls -la /home/fio/.ssh 2>/dev/null; ssh-keygen -lf /home/fio/.ssh/authorized_keys 2>/dev/null'   # fingerprints + comments only
run 22_users   'crontab -l -u fio 2>/dev/null; crontab -l 2>/dev/null; ls -la /etc/cron* 2>/dev/null'

# ---- 5. kernel, devices, USB ---------------------------------------------------------
run 30_kernel  'lsmod; cat /sys/module/usbcore/parameters/autosuspend'
run 30_kernel  'ls -la /dev/ttymxc* /dev/video* /dev/lifetrac-c2 /dev/watchdog* 2>/dev/null'
run 30_kernel  'for d in /sys/bus/usb/devices/*/idVendor; do p=$(dirname $d); echo "$(basename $p) $(cat $d):$(cat $p/idProduct) $(cat $p/product 2>/dev/null)"; done'
run 30_kernel  'dmesg -T 2>/dev/null | tail -400'
run 31_journal 'journalctl --list-boots --no-pager 2>/dev/null | tail -20'

# ---- 5b. watchdog owner, panic/sysrq, PMIC power key, OSTree trial boot ---------------
# 2026-10-17: the facts X8_HEALTH_AND_RECOVERY/REBOOT_AND_POWER_CYCLE.md L2/L3 rest on.
# All reads. Nothing here opens /dev/watchdog0 (fuser and ls stat it), writes sysrq or
# a sysctl, or talks to the PMIC over i2c. /proc/cmdline is already in 01_identity.
# effective watchdog config: who owns the dog (systemd with firmware/x8_recovery's L2
# drop-in, else nobody: the kernel keepalive) and what bounds a hung reboot.
# systemd 250 has no WatchdogDevice D-Bus property: the configured device shows only
# in cat-config below and the journal's "Using hardware watchdog ..., device ..." line.
run 32_watchdog 'systemctl show -p RuntimeWatchdogUSec -p RebootWatchdogUSec -p KExecWatchdogUSec'
run 32_watchdog 'systemctl show -p JobTimeoutUSec -p JobTimeoutAction reboot.target; systemctl cat --no-pager reboot.target'
run 32_watchdog 'systemd-analyze cat-config systemd/system.conf'
run 32_watchdog 'ls -la /etc/systemd/system.conf.d /usr/lib/systemd/system.conf.d /etc/systemd/system/reboot.target.d'
run 32_watchdog 'fuser -v /dev/watchdog0 /dev/watchdog'
run 32_watchdog 'for f in /sys/class/watchdog/*/*; do [ -f "$f" ] && echo "$f=$(cat "$f" 2>&1)"; done'
run 32_watchdog 'dmesg 2>/dev/null | grep -i -E "watchdog|wdog|imx2"; journalctl -b --no-pager 2>/dev/null | grep -i watchdog | tail -40'
# panic -> reboot path: kernel.panic decides whether a panic reboots by itself or hangs
# until WDOG1 bites (L2). No LifeTrac rung crashes the kernel on purpose (never sysrq c);
# CONFIG_MAGIC_SYSRQ and /proc/sysrq-trigger say whether L3's s,u,b rung can work at all
# (kernel.sysrq masks only the keyboard, not root's writes to the trigger file).
run 33_panic_sysrq 'for k in panic panic_on_oops sysrq hung_task_panic softlockup_panic; do echo "kernel.$k=$(cat /proc/sys/kernel/$k 2>&1)"; done'
run 33_panic_sysrq 'ls -l /proc/sysrq-trigger'
run 33_panic_sysrq '{ zcat /proc/config.gz 2>/dev/null || cat "/boot/config-$(uname -r)"; } | grep -E "MAGIC_SYSRQ|PANIC_TIMEOUT|PANIC_ON_OOPS|IMX2_WDT|WATCHDOG_NOWAYOUT|WATCHDOG_HANDLE_BOOT|WATCHDOG_OPEN_TIMEOUT|LOCKUP|HUNG_TASK"'
# BD71847 power key (Max Carrier PB1 -> PWRON): press timings from the device tree, and
# how the key reaches userspace (input device, logind)
run 34_pmic_powerkey 'for n in $(find /proc/device-tree/ \( -name "pmic@4b" -o -name "bd718*@4b" \) 2>/dev/null); do echo "node $n"; ls "$n" | grep "^rohm,"; for p in rohm,short-press-ms rohm,long-press-ms; do [ -f "$n/$p" ] && { echo "$p:"; hexdump -C "$n/$p" 2>/dev/null || od -An -tx1 "$n/$p"; }; done; done'
run 34_pmic_powerkey 'grep -B2 -A8 -i pwrkey /proc/bus/input/devices'
run 34_pmic_powerkey 'systemd-analyze cat-config systemd/logind.conf | grep -i -E "^# /|HandlePowerKey"'
# OSTree/u-boot trial-boot state: watchdog power cycles during a trial boot count
# toward bootlimit and can roll the board back to the previous deployment
run 35_trial_boot 'fw_printenv bootcount bootlimit upgrade_available rollback 2>&1'

# ---- 6. STM32H747: stock x8h7 (M7) image on disk, M4 sketch traces --------------------
# Reads files and journals only. It does NOT read the H7's flash (that needs openocd and halts the H7).
run 40_h7 'ls -la /usr/arduino /usr/arduino/extra /usr/arduino/m4 /tmp/arduino 2>/dev/null'
run 40_h7 'sha256sum /usr/arduino/extra/* 2>/dev/null'
run 40_h7 'timeout 5 cat /sys/kernel/x8h7_firmware/version'          # HC-02 item 11: a timeout is benign
run 40_h7 'journalctl -u stm32h7-program.service -u monitor-m4-elf-file.service -u m4-proxy.service --no-pager 2>/dev/null | tail -200'
keep /usr/arduino/extra/*.sh /usr/arduino/extra/*.cfg

# ---- 7. Foundries / Arduino OOTB: update agent and compose-apps -----------------------
run 50_foundries 'systemctl is-enabled aktualizr-lite 2>/dev/null; systemctl is-active aktualizr-lite 2>/dev/null'
run 50_foundries 'ls -la /var/sota /var/sota/compose-apps /var/sota/reset-apps 2>/dev/null'   # names only, no key contents
for y in /var/sota/compose-apps/*/docker-compose.yml; do [ -f "$y" ] && keep "$y"; done

# ---- 8. docker: images, containers, compose projects ---------------------------------
run 60_docker 'docker info 2>/dev/null | head -40'
run 60_docker 'docker images --digests --no-trunc'
run 60_docker 'docker image inspect --format "{{.Id}} tags={{.RepoTags}} digests={{.RepoDigests}} created={{.Created}} size={{.Size}}" $(docker images -q | sort -u)'
run 60_docker 'docker ps -a --no-trunc --format "{{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Command}}"'
run 60_docker 'docker compose ls -a; docker volume ls; docker network ls'
for c in $(docker ps -aq 2>/dev/null); do
  docker inspect "$c" 2>/dev/null | sed "$REDACT" > "$OUT/61_inspect_$(docker inspect -f '{{.Name}}' "$c" | tr -d /).json"
  for y in $(docker inspect -f '{{index .Config.Labels "com.docker.compose.project.config_files"}}' "$c" 2>/dev/null | tr ',' ' '); do keep "$y"; done
done

# ---- 9. /opt/lifetrac (deployed trees) and /home/fio --------------------------------
# manifest: path, size, sha256 (sha256 withheld for secrets and .env); copy small text files
manifest() {
  root=$1; out=$2
  find "$root" -xdev -type f 2>/dev/null | grep -v '/__pycache__/' | while read -r f; do
    sz=$(wc -c < "$f")
    if echo "$f" | grep -Eq "$DENY"; then
      echo "$f $sz <secret: hash withheld>"
    else
      echo "$f $sz $(sha256sum "$f" | cut -d' ' -f1)"
      case "$f" in
        *.yml|*.yaml|*.conf|*.txt|*.py|*.sh|*.service|*.cfg|*.rules|*/Dockerfile|*DEPLOYED_FROM*|*.log|*.pid) keep "$f";;
      esac
    fi
  done > "$OUT/$out"
}
for r in /var/rootdirs/opt/lifetrac /opt/lifetrac; do [ -d "$r" ] && { manifest "$r" 70_opt_lifetrac_manifest.txt; break; }; done
for r in /var/rootdirs/home/fio /home/fio; do [ -d "$r" ] && { manifest "$r" 71_home_fio_manifest.txt; break; }; done
for e in /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER/.env /opt/lifetrac/DESIGN-CONTROLLER/.env; do
  [ -f "$e" ] && { sed 's/=.*/=<redacted>/' "$e" > "$OUT/72_env_keys_only.txt"; break; }
done
for b in /opt/lifetrac/bin/ffmpeg /var/rootdirs/opt/lifetrac/bin/ffmpeg; do
  [ -x "$b" ] && run 73_ffmpeg "$b -version | head -2; sha256sum $b"
done

# ---- done ----------------------------------------------------------------------------
echo "capture end $(date -u +%Y%m%dT%H%M%SZ)" >> "$OUT/00_meta.txt"
tar -czf "$OUT.tgz" -C "$(dirname "$OUT")" "$(basename "$OUT")"
sha256sum "$OUT.tgz"
echo "CAPTURE_TGZ=$OUT.tgz"
