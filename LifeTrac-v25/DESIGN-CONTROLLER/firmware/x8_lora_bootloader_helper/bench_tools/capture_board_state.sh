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
#   - opening /dev/watchdog0
# Secrets are not copied: NetworkManager connection files, /var/sota keys, ssh host
# keys, shadow, the base's secrets/ dir and .env values are listed (name and size only)
# or redacted.
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
DENY='shadow|gshadow|ssh_host_|_host_key|/dropbear/|machine-id|system-connections|wpa_supplicant.*\.conf|/sota/|\.pem$|\.key$|/secrets/|/\.env$|/\.ssh/'

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
