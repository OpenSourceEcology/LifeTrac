#!/usr/bin/env bash
# provision_bench_board.sh -- bring one freshly imaged Portenta X8 (on a Max
# Carrier) to the LifeTrac bench state, or report how far it is from it.
#
#   bash provision_bench_board.sh base    [--check] [--ssh] [--nic-100]
#   bash provision_bench_board.sh tractor [--check] [--mosquitto-from-base | --mosquitto-tar FILE]
#   bash provision_bench_board.sh --help
#
# Runs on the bench PC (Git Bash on Windows; any bash with adb works). Talks to
# one board over adb (the base can use ssh with --ssh). Every step checks first
# and changes only what is missing, so re-running it is safe. --check changes
# nothing and exits 3 if something is still to do.
#
# What it applies (BENCH_SETUP.md explains each step):
#   both     sudoers drop-in (../install_lifetrac_nopasswd.sh), docker running,
#            the probe image hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44
#            (repo-root foundries_python.tar; radio_park.py / radio_state.py /
#            rs116_health_probe.py run in it), the old boot units retired
#            (see below)
#   base     compose-apps-early-start{,-recovery}.service masked, the PC's ssh
#            key in fio's authorized_keys, eclipse-mosquitto:2 pulled,
#            optional eth0 100BASE-TX full pin (--nic-100)
#   tractor  WiFi off for good (units/disable-wifi.service enabled,
#            wpa_supplicant masked, nmcli radio wifi off, stored WiFi profiles
#            deleted after a root-only backup tarball in /root on the board),
#            camera USB provisioning (../provision_x8.sh),
#            eclipse-mosquitto:2 loaded from the base or a tarball; the
#            compose-apps mask too, unless the board still runs the factory
#            674 image (where that unit succeeds)
#
# Retired boot units (BENCH_BOARDS.md; retired by hand on the original pair on
# 2026-10-12): lifetrac-base.service and lifetrac-base-compose.service on the
# base, lifetrac-tractor-compose.service on the tractor. If one of them ever
# succeeds at boot it starts compose services that map /dev/ttymxc3, without
# a GO. The step passes only when systemd cannot load any of them: for each,
# 'systemctl is-enabled' finds no unit file and nothing named <unit> or
# <unit>.d is left in /etc/systemd/system, or the unit is masked by a symlink
# to /dev/null there; and it is inactive or failed with no job queued. A board
# that never had the units passes. Otherwise, and only while every listed unit
# is failed or inactive with no start or stop job queued, it runs
# 'systemctl disable' (never --now) and 'systemctl reset-failed', moves the
# unit file (or a symlink there that is not a mask) and its <unit>.d directory
# from /etc/systemd/system into a new /etc/lifetrac-retired/<PC UTC time>/
# (never an existing directory), writes a README.txt there, and runs
# 'systemctl daemon-reload'. A unit that is already disabled but still has its
# file there gets the same treatment. If any listed unit is active, starting
# or stopping, has a job queued (early in a boot an enabled unit reads
# inactive while its start job waits), or is in a state a disable and a move
# cannot retire (is-enabled reads a -runtime form, generated, transient, alias
# or indirect, or a word that does not match what is in /etc/systemd/system),
# the step fails, changes nothing and prints what to do. The step never
# starts, stops or masks a unit, never runs docker compose and never stops or
# removes a container (the base broker design-controller-mosquitto-1
# included). The leftover video-test containers and the tractor's
# /opt/lifetrac/video-test/docker-compose.yml are only reported; their manual,
# ordered cleanup is in BENCH_BOARDS.md.
#
# Radio safety: this script never opens /dev/ttymxc3 (the L072 radio UART),
# never starts or stops lifetrac-base.service, lifetrac-base-compose.service
# or lifetrac-tractor-compose.service, and on the tractor it leaves
# lifetrac-camera.service and the tractor-camera container stopped (both map
# /dev/ttymxc3; they come back at the next boot).
#
# Settings come from bench.env next to this script ($BENCH_ENV; copy
# bench.env.example), loaded by lib/env_file.sh like every bench script: CRLF is
# tolerated and a NON-EMPTY variable already in the environment wins over the
# file. Anything still unset gets these defaults (the original bench pair):
#   BASE_SERIAL=2D0A1209DABC240B  TRACTOR_SERIAL=2E2C1209DABC240B
#   BASE_HOST=192.168.1.117       BENCH_SUDO_PW=fio (used once, to install sudoers)
#   BASE_SSH_KEY=~/.ssh/lifetrac_base_ed25519  BASE_SSH_USER=fio (old name
#   BASE_USER)  ADB=adb
# The password is never printed.

set -u

usage() {
  sed -n '2,/^$/{s/^# \{0,1\}//;p}' "$0"
  cat <<'EOF'
Options:
  --check                 report only; change nothing (exit 3 if anything is missing)
  --serial SERIAL         adb serial of the board (default: BASE_SERIAL / TRACTOR_SERIAL)
  --ssh                   base only: use ssh to BASE_HOST instead of adb
  --nic-100               base only: pin 'Wired connection 1' to 100BASE-TX full
                          (workaround for a marginal cable; leave it off on a good cable)
  --mosquitto-from-base   tractor only: docker save eclipse-mosquitto:2 on the base,
                          copy it through the PC and docker load it on the tractor
  --mosquitto-tar FILE    tractor only: docker load eclipse-mosquitto:2 from FILE (a docker save tarball)
  --mask-compose          tractor only: mask compose-apps-early-start{,-recovery} even on the 674 image
  --keep-wifi-profiles    tractor only: do not delete stored NetworkManager WiFi profiles
                          (without it they are deleted after a root-only backup on the board)
  --allow-camera-restart  tractor only: allow provision_x8.sh to run although a camera compose
                          app is installed (it restarts lifetrac-camera.service, which maps the
                          radio UART; this script stops it again right after)
  -h, --help              this text
Exit codes: 0 done / all present, 1 a step failed, 2 usage or board unreachable, 3 --check found work to do.
EOF
}

# ---------------------------------------------------------------- settings --
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
HELPER_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../../../.." && pwd)

BENCH_ENV=${BENCH_ENV:-${BENCH_ENV_FILE:-$SCRIPT_DIR/bench.env}}
# shellcheck source=lib/env_file.sh
. "$SCRIPT_DIR/lib/env_file.sh" || exit 2
if [ -f "$BENCH_ENV" ]; then
  # CR stripped (a bench.env saved by a Windows editor); the environment wins
  bench_load_env_file "$BENCH_ENV" || exit 2
else
  echo "WARN: no $BENCH_ENV -- the original bench's serials and address apply unless set in the environment (cp bench.env.example bench.env; BENCH_SETUP.md 5.3)" >&2
fi
BASE_SERIAL=${BASE_SERIAL:-2D0A1209DABC240B}
TRACTOR_SERIAL=${TRACTOR_SERIAL:-2E2C1209DABC240B}
BASE_HOST=${BASE_HOST:-192.168.1.117}
BENCH_SUDO_PW=${BENCH_SUDO_PW:-fio}
BASE_SSH_KEY=${BASE_SSH_KEY:-$HOME/.ssh/lifetrac_base_ed25519}
BASE_USER=${BASE_SSH_USER:-${BASE_USER:-fio}}
ADB=${ADB:-adb}

FOUNDRIES_IMG=hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44
FOUNDRIES_TAR=$REPO_ROOT/foundries_python.tar
MOSQ_IMG=eclipse-mosquitto:2
NIC_CON='Wired connection 1'
RSTAGE=/tmp/lifetrac_provision          # on the board (tmpfs; gone after a reboot)
UNIT_WIFI=$SCRIPT_DIR/units/disable-wifi.service

# --------------------------------------------------------------- arguments --
ROLE=""; CHECK_ONLY=0; SERIAL_ARG=""; USE_SSH=0; NIC_100=0
MOSQ_FROM_BASE=0; MOSQ_TAR=""; MASK_COMPOSE=0; KEEP_WIFI=0; ALLOW_CAMERA_RESTART=0
while [ $# -gt 0 ]; do
  case "$1" in
    base|tractor)           ROLE=$1 ;;
    --check)                CHECK_ONLY=1 ;;
    --serial|--mosquitto-tar)
      case "${2:-}" in ''|-*) echo "$1 needs a value" >&2; exit 2 ;; esac
      if [ "$1" = --serial ]; then SERIAL_ARG=$2; else MOSQ_TAR=$2; fi
      shift ;;
    --ssh)                  USE_SSH=1 ;;
    --nic-100)              NIC_100=1 ;;
    --mosquitto-from-base)  MOSQ_FROM_BASE=1 ;;
    --mask-compose)         MASK_COMPOSE=1 ;;
    --keep-wifi-profiles)   KEEP_WIFI=1 ;;
    --allow-camera-restart) ALLOW_CAMERA_RESTART=1 ;;
    -h|--help)              usage; exit 0 ;;
    *) echo "unknown argument: $1 (see --help)" >&2; exit 2 ;;
  esac
  shift
done
[ -n "$ROLE" ] || { usage; exit 2; }
if [ "$ROLE" = tractor ] && [ "$USE_SSH" = 1 ]; then
  echo "--ssh is for the base only; the tractor is reached over adb (its WiFi stays off)" >&2; exit 2
fi
if [ "$ROLE" = base ] && { [ "$MOSQ_FROM_BASE" = 1 ] || [ -n "$MOSQ_TAR" ]; }; then
  echo "--mosquitto-* are for the tractor; the base pulls $MOSQ_IMG itself" >&2; exit 2
fi

if [ "$USE_SSH" = 1 ]; then
  TRANSPORT=ssh; TARGET=$BASE_HOST
else
  TRANSPORT=adb
  if [ -n "$SERIAL_ARG" ]; then TARGET=$SERIAL_ARG
  elif [ "$ROLE" = base ]; then TARGET=$BASE_SERIAL
  else TARGET=$TRACTOR_SERIAL; fi
fi

# ----------------------------------------------------------------- helpers --
say() { printf '%s\n' "$*"; }
die() { printf 'ERROR: %s\n' "$2" >&2; exit "$1"; }
indent() { sed 's/^/      /'; }

winpath() {  # adb.exe wants C:/... paths; elsewhere the path is used as is
  if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else printf '%s' "$1"; fi
}
unixpath() {  # scp reads C:/... as host "C": give it /c/... (e.g. --mosquitto-tar C:/x.tar)
  if command -v cygpath >/dev/null 2>&1; then cygpath -u "$1"; else printf '%s' "$1"; fi
}

# on <adb|ssh> <serial|host> <command>: run a shell command on a board, CR stripped.
# LogLevel=ERROR: stderr is merged into the output, and a client warning (OpenSSH 10
# prints three "post-quantum key exchange" lines per connection) would otherwise be
# read as command output. Connection errors still print.
on() {
  if [ "$1" = ssh ]; then
    ssh -i "$BASE_SSH_KEY" -o BatchMode=yes -o ConnectTimeout=10 -o LogLevel=ERROR "$BASE_USER@$2" "$3" 2>&1 | tr -d '\r'
  else
    MSYS_NO_PATHCONV=1 "$ADB" -s "$2" shell "$3" 2>&1 | tr -d '\r'
  fi
}
push_to() {    # push_to <adb|ssh> <serial|host> <local file> <remote path>
  if [ "$1" = ssh ]; then
    scp -q -i "$BASE_SSH_KEY" -o BatchMode=yes "$(unixpath "$3")" "$BASE_USER@$2:$4"
  else
    MSYS_NO_PATHCONV=1 "$ADB" -s "$2" push "$(winpath "$3")" "$4" >/dev/null
  fi
}
pull_from() {  # pull_from <adb|ssh> <serial|host> <remote path> <local file>
  if [ "$1" = ssh ]; then
    scp -q -i "$BASE_SSH_KEY" -o BatchMode=yes "$BASE_USER@$2:$3" "$(unixpath "$4")"
  else
    MSYS_NO_PATHCONV=1 "$ADB" -s "$2" pull "$3" "$(winpath "$4")" >/dev/null
  fi
}
rsh()   { on "$TRANSPORT" "$TARGET" "$1"; }
# rtest '<shell condition>': true when it succeeds on the board (does not rely on
# adb propagating exit codes, which older adbd builds do not)
rtest() { [ "$(rsh "if { $1; } >/dev/null 2>&1; then echo __YES__; else echo __NO__; fi" | tail -n 1)" = __YES__ ]; }
rval()  { rsh "$1" | tail -n 1; }

LSTAGE=$(mktemp -d "${TMPDIR:-/tmp}/lifetrac_provision.XXXXXX") || die 2 "mktemp failed"
trap 'rm -rf "$LSTAGE"' EXIT

stage_text() {  # stage_text <local text file> <remote dir>: strip CR, then push
  local name; name=$(basename "$1")
  sed 's/\r$//' "$1" > "$LSTAGE/$name" || return 1
  push_to "$TRANSPORT" "$TARGET" "$LSTAGE/$name" "$2/$name"
}
lsha() { sed 's/\r$//' "$1" | sha256sum | awk '{print $1}'; }
rsha() { rsh "sha256sum '$1' 2>/dev/null" | awk 'NF==2 {print $1}' | tail -n 1; }

RESULTS=(); N_TODO=0; N_FAIL=0
record() {  # record <status> <label>
  RESULTS+=("$(printf '%-9s %s' "[$1]" "$2")")
  printf '  %-9s %s\n' "[$1]" "$2"
  case "$1" in TODO) N_TODO=$((N_TODO + 1)) ;; FAILED) N_FAIL=$((N_FAIL + 1)) ;; esac
}
info() { record info "$1"; }
# item <label> <check command> <apply command>: check; if missing and not
# --check, apply and check again. Commands are this script's own functions.
item() {
  if eval "$2"; then record ok "$1"; return 0; fi
  if [ "$CHECK_ONLY" = 1 ]; then record TODO "$1"; return 1; fi
  say "  ->        applying: $1"
  eval "$3"
  if eval "$2"; then record applied "$1"; return 0; fi
  record FAILED "$1"; return 1
}

# ------------------------------------------------------------------- steps --
# sudo without a password: everything after this uses 'sudo -n'
chk_sudo() { rtest 'sudo -n test -f /etc/sudoers.d/99-lifetrac-bench-nopasswd'; }
app_sudo() {
  local pw
  rsh "mkdir -p $RSTAGE" >/dev/null
  stage_text "$HELPER_DIR/install_lifetrac_nopasswd.sh" "$RSTAGE" || return 1
  pw=$(printf '%s' "$BENCH_SUDO_PW" | sed "s/'/'\\\\''/g")
  rsh "printf '%s\n' '$pw' | sudo -S -p '' sh $RSTAGE/install_lifetrac_nopasswd.sh" | grep -v '^$' | indent
}

chk_masked() { [ "$(rval "systemctl is-enabled $1 2>/dev/null")" = masked ]; }
app_mask()   { rsh "sudo -n systemctl mask --now $1" | indent; }

chk_docker() { rtest 'sudo -n docker info'; }
app_docker() { rsh 'sudo -n systemctl start docker.service' | indent; }

chk_image() { rtest "sudo -n docker image inspect $1"; }
load_tar() {  # load_tar <local tarball> <remote file name>
  rsh "mkdir -p $RSTAGE" >/dev/null
  say "      pushing $(basename "$1") ($(du -k "$1" | awk '{print $1}') KiB) ..."
  push_to "$TRANSPORT" "$TARGET" "$1" "$RSTAGE/$2" || return 1
  rsh "sudo -n docker load -i $RSTAGE/$2; rm -f $RSTAGE/$2" | indent
}
app_foundries() {
  [ -f "$FOUNDRIES_TAR" ] || { say "      $FOUNDRIES_TAR is missing (it is tracked at the repo root)"; return 1; }
  load_tar "$FOUNDRIES_TAR" foundries_python.tar
}
app_mosq_pull() { rsh "sudo -n docker pull $MOSQ_IMG" | tail -n 3 | indent; }

fetch_mosq_from_base() {  # base -> PC: ssh when the key exists, else adb
  local btr btg
  for btr in ssh adb; do
    if [ "$btr" = ssh ]; then
      { [ -f "$BASE_SSH_KEY" ] && [ -n "$BASE_HOST" ]; } || continue
      btg=$BASE_HOST
    else
      btg=$BASE_SERIAL
    fi
    say "      docker save $MOSQ_IMG on the base ($btr $btg) ..."
    if on "$btr" "$btg" "sudo -n docker image inspect $MOSQ_IMG >/dev/null 2>&1 || sudo -n docker pull $MOSQ_IMG >/dev/null; sudo -n docker save -o /tmp/eclipse-mosquitto_2.tar $MOSQ_IMG && sudo -n chmod 0644 /tmp/eclipse-mosquitto_2.tar && echo __SAVED__" | grep -q __SAVED__ \
       && pull_from "$btr" "$btg" /tmp/eclipse-mosquitto_2.tar "$LSTAGE/eclipse-mosquitto_2.tar"; then
      on "$btr" "$btg" "sudo -n rm -f /tmp/eclipse-mosquitto_2.tar" >/dev/null
      return 0
    fi
  done
  say "      could not get $MOSQ_IMG from the base (provision it first: provision_bench_board.sh base)"
  return 1
}
app_mosq_tractor() {
  local tar
  if [ -n "$MOSQ_TAR" ]; then
    tar=$MOSQ_TAR
  elif [ "$MOSQ_FROM_BASE" = 1 ]; then
    fetch_mosq_from_base || return 1
    tar=$LSTAGE/eclipse-mosquitto_2.tar
  else
    say "      the tractor has no network; re-run with --mosquitto-from-base (base provisioned and reachable)"
    say "      or --mosquitto-tar FILE (a 'docker save $MOSQ_IMG' tarball on this PC)"
    return 1
  fi
  [ -f "$tar" ] || { say "      no such file: $tar"; return 1; }
  load_tar "$tar" eclipse-mosquitto_2.tar
}

nic_val() { rval "nmcli -g $1 connection show '$NIC_CON' 2>/dev/null"; }
chk_nic() { [ "$(nic_val 802-3-ethernet.speed)" = 100 ] && [ "$(nic_val 802-3-ethernet.duplex)" = full ]; }
app_nic() {
  rtest "nmcli -g connection.id connection show '$NIC_CON'" \
    || { say "      no NetworkManager profile '$NIC_CON' (is the ethernet cable plugged in?)"; return 1; }
  # auto-negotiate stays on: NetworkManager then advertises only 100/full, which is
  # what the bench base runs (capture 20_network.txt) and avoids a duplex mismatch
  rsh "sudo -n nmcli connection modify '$NIC_CON' 802-3-ethernet.speed 100 802-3-ethernet.duplex full 802-3-ethernet.auto-negotiate yes" | indent
  if [ "$TRANSPORT" = adb ]; then
    rsh "sudo -n nmcli connection up '$NIC_CON'" | indent
  else
    say "      saved; it takes effect at the next 'nmcli connection up' or reboot (not re-activated over ssh)"
  fi
}

PUBKEY=$BASE_SSH_KEY.pub
pubkey_id() { sed 's/\r$//' "$PUBKEY" | head -n 1 | awk '{print $1" "$2}'; }
chk_sshkey() { [ -f "$PUBKEY" ] && rtest "grep -qF '$(pubkey_id)' ~/.ssh/authorized_keys"; }
app_sshkey() {
  if [ ! -f "$PUBKEY" ]; then
    say "      no public key at $PUBKEY; create the pair on this PC first:"
    say "      ssh-keygen -t ed25519 -f ~/.ssh/lifetrac_base_ed25519 -C lifetrac-bench-pc"
    return 1
  fi
  rsh "mkdir -p $RSTAGE" >/dev/null
  sed 's/\r$//' "$PUBKEY" | head -n 1 > "$LSTAGE/pc_key.pub"
  push_to "$TRANSPORT" "$TARGET" "$LSTAGE/pc_key.pub" "$RSTAGE/pc_key.pub" || return 1
  rsh "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat $RSTAGE/pc_key.pub >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys; rm -f $RSTAGE/pc_key.pub" | indent
}

# tractor: WiFi off for good
chk_wifi_unit() { [ "$(rsha /etc/systemd/system/disable-wifi.service)" = "$(lsha "$UNIT_WIFI")" ]; }
app_wifi_unit() {
  rsh "mkdir -p $RSTAGE" >/dev/null
  stage_text "$UNIT_WIFI" "$RSTAGE" || return 1
  rsh "sudo -n cp $RSTAGE/disable-wifi.service /etc/systemd/system/disable-wifi.service && sudo -n chown root:root /etc/systemd/system/disable-wifi.service && sudo -n chmod 0644 /etc/systemd/system/disable-wifi.service && sudo -n systemctl daemon-reload" | indent
}
chk_wifi_enabled() { [ "$(rval 'systemctl is-enabled disable-wifi.service 2>/dev/null')" = enabled ]; }
app_wifi_enabled() { rsh 'sudo -n systemctl enable disable-wifi.service' | indent; }
# Remote commands below avoid embedded double quotes (they cross the Windows
# command line on the way to adb.exe), and the negative checks first prove the
# query itself works, so a missing 'sudo -n' reads as TODO, not as ok.
chk_rfkill() { rtest "sudo -n /usr/sbin/rfkill list wifi >/dev/null && ! sudo -n /usr/sbin/rfkill list wifi | grep -q 'Soft blocked: no'"; }
app_rfkill() { rsh 'sudo -n systemctl start disable-wifi.service; sudo -n /usr/sbin/rfkill block wifi' | indent; }
chk_nm_wifi() { [ "$(rval 'nmcli radio wifi 2>/dev/null')" = disabled ]; }
app_nm_wifi() { rsh 'sudo -n nmcli radio wifi off' | indent; }

NM_LIST="sudo -n nmcli -t -f UUID,TYPE connection show"
WIFI_UUIDS="$NM_LIST 2>/dev/null | grep ':802-11-wireless\$' | cut -d: -f1"
chk_wifi_profiles() { rtest "$NM_LIST >/dev/null && ! $NM_LIST | grep -q ':802-11-wireless\$'"; }
list_wifi_profiles() {  # profile names only; secrets are never requested
  rsh "for u in \$($WIFI_UUIDS); do nmcli -g connection.id connection show uuid \$u 2>/dev/null; done"
}
app_wifi_profiles() {
  local bk
  # Back the stored profiles up first, root-only (0600): the board may hold the only
  # copy of a WiFi credential. Nothing from the backup is printed.
  bk=$(rval "sudo -n sh -c 'umask 077; d=/etc/NetworkManager/system-connections; f=/root/lifetrac_nm_profiles_\$(date -u +%Y%m%dT%H%M%SZ).tgz; if [ -d \$d ] && tar -czf \$f -C \$d . && chmod 600 \$f; then echo \$f; else echo NONE; fi' 2>/dev/null")
  case "$bk" in
    /root/lifetrac_nm_profiles_*.tgz)
      say "      stored profiles backed up on the board (root-only): $bk"
      say "      restore: sudo tar -xzf $bk -C /etc/NetworkManager/system-connections && sudo nmcli connection reload" ;;
    *)
      say "      could not back up /etc/NetworkManager/system-connections, so the WiFi profiles were NOT deleted"
      say "      (re-run with --keep-wifi-profiles to skip this step)"
      return 1 ;;
  esac
  rsh "for u in \$($WIFI_UUIDS); do n=\$(nmcli -g connection.id connection show uuid \$u 2>/dev/null); if sudo -n nmcli connection delete uuid \$u >/dev/null 2>&1; then echo deleted WiFi profile: \$n; else echo FAILED to delete WiFi profile: \$n; fi; done" | indent
}

# tractor: camera USB provisioning, by running the repo's provision_x8.sh unchanged
CAM_FILES="lifetrac-no-usb-audio.conf:/etc/modprobe.d/lifetrac-no-usb-audio.conf 99-w2-01-c2.rules:/etc/udev/rules.d/99-w2-01-c2.rules lifetrac-camera.service:/etc/systemd/system/lifetrac-camera.service"
chk_camera() {
  local pair
  for pair in $CAM_FILES; do
    [ "$(rsha "${pair#*:}")" = "$(lsha "$HELPER_DIR/${pair%%:*}")" ] || return 1
  done
  rtest 'id -nG fio | grep -qw video'
}
app_camera() {
  local f
  if [ "$ALLOW_CAMERA_RESTART" != 1 ] && rtest 'test -e /dev/lifetrac-c2 && test -f /opt/lifetrac/compose-apps/lifetrac-camera/docker-compose.yml'; then
    say "      not run: a camera compose app is installed and the C2 is attached, so provision_x8.sh"
    say "      would restart lifetrac-camera.service, whose container maps /dev/ttymxc3 (the radio UART)."
    say "      Re-run with --allow-camera-restart to accept that (it is stopped again right after)."
    return 1
  fi
  rsh "mkdir -p $RSTAGE/camera" >/dev/null
  for f in provision_x8.sh lifetrac-no-usb-audio.conf 99-w2-01-c2.rules lifetrac-camera.service; do
    stage_text "$HELPER_DIR/$f" "$RSTAGE/camera" || return 1
  done
  rsh "cd $RSTAGE/camera && sudo -n bash provision_x8.sh" | indent
  app_camera_stopped
}

# tractor: lifetrac-camera.service and the tractor-camera container map /dev/ttymxc3.
# The unit's state is read as text and only inactive or failed passes:
# 'activating' (its 2 s auto-restart window) is not stopped, and neither is a
# query that did not answer.
chk_camera_stopped() {
  case "$(rval 'systemctl is-active lifetrac-camera.service 2>/dev/null')" in inactive|failed) ;; *) return 1 ;; esac
  rtest "sudo -n docker ps >/dev/null && ! sudo -n docker ps --format '{{.Names}}' | grep -qx tractor-camera"
}
app_camera_stopped() {
  rsh 'sudo -n systemctl stop lifetrac-camera.service 2>/dev/null; sudo -n docker stop tractor-camera >/dev/null 2>&1; true' >/dev/null
}

# The old boot units: lifetrac-base.service and lifetrac-base-compose.service on
# the base, lifetrac-tractor-compose.service on the tractor. The original pair
# had them enabled, and they failed at every boot that was captured; they were
# retired, not repaired, on 2026-10-12 (BENCH_BOARDS.md), because that failure
# was all that kept a radio daemon from starting at boot without a GO. This
# step leaves a board in the same state. systemd runs ExecStop only after a
# successful start, so a failed or inactive unit needs no stop and is disabled
# WITHOUT --now. 'systemctl mask' is not used: the real unit file sits where
# the mask symlink would go. Nothing here starts or stops a unit, runs docker
# compose, or stops or removes a container.
UNIT_DIR=/etc/systemd/system
RETIRE_ROOT=/etc/lifetrac-retired
RETIRE_ERR=""   # set when a retirement only half worked, so the re-check fails

# What 'systemctl is-enabled' says for a unit that has no unit file anywhere:
# on both bench boards (systemd 250 on the tractor) no word, only this error on
# stderr, as captured on 2026-10-12; later systemd versions print not-found.
enoent_msg() { printf 'Failed to get unit file state for %s: No such file or directory' "$1"; }
unit_state() {  # prints <is-enabled>/<is-active>; is-enabled reads not-found when there is no unit file of that name
  rval "e=\$(systemctl is-enabled $1 2>&1); case \$e in '$(enoent_msg "$1")') e=not-found ;; esac; echo \$e/\$(systemctl is-active $1 2>/dev/null || true)"
}
unit_job() {  # prints "job=" when systemd has no start or stop job queued for the unit, else "job=<id>"
  rval "j=\$(systemctl show -p Job --value $1 2>/dev/null) && echo job=\$j"
}
# unit_path_kind <name>: prints mask, link, file or none for $UNIT_DIR/<name>.
# mask = a symlink to /dev/null, as 'systemctl mask' makes it; link = any other
# symlink, dangling or not; file = a real file or directory.
unit_path_kind() {
  rval "if test -L $UNIT_DIR/$1; then case \$(readlink $UNIT_DIR/$1) in /dev/null) echo mask ;; *) echo link ;; esac; elif test -e $UNIT_DIR/$1; then echo file; else echo none; fi"
}
# Retired means systemd cannot load the unit from a file: is-enabled reads
# not-found and nothing named <unit> is left in $UNIT_DIR, or it reads masked
# and $UNIT_DIR/<unit> is a symlink to /dev/null. Every other is-enabled word
# (systemctl(1)) names a unit file systemd can still load, and start by hand,
# from udev or as another unit's dependency: enabled, linked (a 'systemctl
# link' symlink), alias, static, indirect, disabled, generated, transient, and
# the -runtime forms, which /run keeps until the next boot. The unit must also
# be inactive or failed, with no <unit>.d left and no job queued: systemd keeps
# a loaded unit until a daemon-reload, and a queued start of it still runs.
# Only these exact replies pass, so a query that did not work reads as TODO,
# not as ok. The file test is what lets a re-run finish a retirement that
# stopped after the disable.
chk_units_retired() {
  local u st
  [ -z "$RETIRE_ERR" ] || return 1
  for u in "$@"; do
    st=$(unit_state "$u")
    case "${st##*/}" in inactive|failed) ;; *) return 1 ;; esac
    case "${st%/*}/$(unit_path_kind "$u")" in not-found/none|masked/mask) ;; *) return 1 ;; esac
    [ "$(unit_path_kind "$u.d")" = none ] || return 1
    [ "$(unit_job "$u")" = job= ] || return 1
  done
  return 0
}
# unit_job_msg <unit> <unit_job reply>: returns 0 when no job is queued; else
# prints why the unit is not touched and returns 1. Early in a boot an enabled
# unit reads inactive while its start job waits for docker.service, and
# 'systemctl disable' does not cancel a queued job.
unit_job_msg() {
  case "$2" in
    job=) return 0 ;;
    job=*[!0-9]*|'') ;;
    job=*)
      say "      $1 does not read active, but systemd has a job queued for it (job ${2#job=}):"
      say "      the board is still booting, or the unit has just been started or stopped."
      say "      Not touched: 'systemctl disable' does not cancel a queued start. Wait until"
      say "      'systemctl list-jobs' no longer lists the unit, then re-run."
      return 1 ;;
  esac
  say "      $1: could not read whether systemd has a job queued for it (reply '${2:-none}'). Not touched."
  return 1
}
# unit_enabled_msg <unit> <is-enabled word>: returns 0 when 'systemctl disable'
# and a move out of $UNIT_DIR can retire a unit that reads that word, and what
# is at $UNIT_DIR/<unit> agrees with the word; else prints why the unit is not
# touched and returns 1.
unit_enabled_msg() {
  local k
  case "$2" in
    not-found|enabled|disabled|static|linked|masked)
      k=$(unit_path_kind "$1")
      case "$2/$k" in
        not-found/none|not-found/link|masked/mask|linked/link) return 0 ;;
        enabled/file|enabled/link|disabled/file|disabled/link|static/file|static/link) return 0 ;;
        masked/none) say "      $1 is masked, but not by a symlink to /dev/null in $UNIT_DIR." ;;
        */none)
          say "      $1 is $2, but its unit file is not in $UNIT_DIR (systemctl cat $1"
          say "      shows where), and this step moves files from there only." ;;
        */file) say "      $1 is $2, but $UNIT_DIR/$1 is a file: the two do not agree." ;;
        */link) say "      $1 is $2, but $UNIT_DIR/$1 is a symlink: the two do not agree." ;;
        */mask) say "      $1 is $2, but $UNIT_DIR/$1 is a symlink to /dev/null: the two do not agree." ;;
        *) say "      $1: could not look at $UNIT_DIR/$1." ;;
      esac ;;
    *-runtime)
      say "      $1 is $2: /run holds that state until the next boot, and a plain"
      say "      'systemctl disable' does not change it." ;;
    alias)
      say "      $1 is an alias, a symlink to another unit file: 'systemctl disable'"
      say "      would act on that unit, and moving the alias does not retire it." ;;
    indirect)
      say "      $1 is indirect: its [Install] section names other units (Also=), or it"
      say "      is enabled under another name; 'systemctl disable' would change those too." ;;
    generated|transient)
      say "      $1 is $2: systemd makes it at run time (a generator, or the runtime"
      say "      API as systemd-run uses it), so there is no file of it in $UNIT_DIR to move." ;;
    *)
      say "      $1: is-enabled returned '${2:-nothing}', not a state this step knows." ;;
  esac
  say "      Not touched. Look at it by hand (systemctl status $1), then re-run."
  return 1
}
unit_blocked_msg() {  # unit_blocked_msg <unit> <is-active>: why it is not touched, and what to do
  local d
  case "$2" in
    activating|reloading|deactivating)
      say "      $1 is $2: a start or stop job is still running. Not touched. Wait until it"
      say "      has failed or is inactive, then re-run. Do not stop it and do not interrupt a build." ;;
    active)
      case "$1" in
        lifetrac-base.service)         d="lora_bridge and web_ui" ;;
        lifetrac-base-compose.service) d="image_rx and web_ui" ;;
        *)                             d="image_tx and camera" ;;
      esac
      say "      $1 is active: its start succeeded, so its compose services were started"
      say "      and may be running ($d; /dev/ttymxc3 is mapped). Not touched:"
      say "      this step stops no unit and no container. Stop those containers by name"
      say "      (sudo docker ps; sudo docker stop <container>), then"
      say "      'sudo systemctl stop $1' and re-run this script before the next boot."
      if [ "$1" = lifetrac-base.service ]; then
        say "      That stop runs the unit's ExecStop, a bare 'docker compose down' in DESIGN-CONTROLLER:"
        say "      it also removes the broker container design-controller-mosquitto-1 (its volume stays)."
        say "      Recreate the broker with deploy_base.sh. Never run that bare 'down' by hand."
      fi ;;
    *)
      say "      $1: is-active returned '${2:-nothing}' (expected inactive or failed). Not touched." ;;
  esac
}
retire_readme() {  # retire_readme <timestamp> <moved paths> <retire dir>
  rsh "mkdir -p $RSTAGE" >/dev/null
  printf '%s\n' \
    "Retired $1 (PC UTC; a board clock may not be wall time) by bench_tools/provision_bench_board.sh." \
    "Moved here from:$2" \
    "Why: on a LifeTrac bench board these units must not run at boot. If one of them succeeds it" \
    "starts compose services that map /dev/ttymxc3 (the L072 radio UART) without a GO. The bench" \
    "retired them on 2026-10-12: see bench_tools/BENCH_BOARDS.md in the LifeTrac repo." \
    "Do not reinstall or re-enable them on a bench board." \
    "Undo: mv the files back to $UNIT_DIR/, systemctl daemon-reload, systemctl enable <unit>." \
    > "$LSTAGE/README.txt" || return 1
  push_to "$TRANSPORT" "$TARGET" "$LSTAGE/README.txt" "$RSTAGE/README.txt" || return 1
  rtest "sudo -n cp $RSTAGE/README.txt $3/README.txt && sudo -n chown root:root $3/README.txt && sudo -n chmod 0644 $3/README.txt; rm -f $RSTAGE/README.txt; test -f $3/README.txt"
}
app_units_retired() {
  local u f st out ts n dir="" files="" blocked=0
  # Guard first, as on 2026-10-12: nothing changes unless every unit is failed or
  # inactive. Beyond that run's guard, none may have a job queued, and each must
  # read an is-enabled word that a disable and a move can retire, and that
  # agrees with what is at $UNIT_DIR/<unit>.
  for u in "$@"; do
    st=$(unit_state "$u")
    case "${st##*/}" in
      inactive|failed)
        unit_enabled_msg "$u" "${st%/*}" || blocked=1
        unit_job_msg "$u" "$(unit_job "$u")" || blocked=1 ;;
      *) unit_blocked_msg "$u" "${st##*/}"; blocked=1 ;;
    esac
  done
  if [ "$blocked" = 1 ]; then say "      nothing was changed"; return 1; fi

  for u in "$@"; do
    st=$(unit_state "$u")
    case "$st" in
      not-found/inactive|not-found/failed)
        # No unit file of that name anywhere systemd looks: nothing to disable. A
        # symlink left in $UNIT_DIR (a dangling one) is moved below; a file or a
        # mask there contradicts the reply, so it cannot be trusted.
        case "$(unit_path_kind "$u")" in
          none|link) ;;
          *) say "      $u: is-enabled does not know it, but $UNIT_DIR/$u is there; nothing was moved"
             RETIRE_ERR=1 ;;
        esac
        continue ;;
      masked/inactive|masked/failed) continue ;;   # nothing to disable; the mask stays, a <unit>.d is moved below
      enabled/*|disabled/*|static/*|linked/*) ;;
      *) say "      $u: its state changed or could not be read again (reply '${st:-none}'); nothing was moved"
         RETIRE_ERR=1; continue ;;
    esac
    # The board reads is-active and the job once more, right before the change,
    # and is-enabled after it: only a word that names no enablement counts.
    # 'disable' removes a 'systemctl link' symlink, so the unit may read not-found
    # then; a symlink it leaves is moved below. reset-failed complains when the
    # unit is not loaded; that is fine.
    out=$(rsh "a=\$(systemctl is-active $u 2>/dev/null); j=\$(systemctl show -p Job --value $u 2>/dev/null) || j=unknown; case \$a/\$j in inactive/|failed/) sudo -n systemctl disable $u; sudo -n systemctl reset-failed $u 2>/dev/null; e=\$(systemctl is-enabled $u 2>&1); case \$e in disabled|static|masked|linked|not-found|'$(enoent_msg "$u")') echo __DISABLED__ ;; *) echo is-enabled after disable: \${e:-no reply} ;; esac ;; *) echo is-active: \$a, queued job: \$j ;; esac")
    printf '%s\n' "$out" | grep -v -e '^__DISABLED__$' -e '^$' | indent
    if ! printf '%s\n' "$out" | grep -q '^__DISABLED__$'; then
      say "      $u was NOT disabled (see above); no unit file was moved"
      RETIRE_ERR=1
    fi
  done

  for u in "$@"; do
    for f in "$u" "$u.d"; do
      # A mask of the unit (a symlink to /dev/null) stays. Any other symlink is
      # moved like a file: mv moves the link, not what it points to.
      case "$f:$(unit_path_kind "$f")" in
        "$u:mask"|*:none) ;;
        *:file|*:link|*:mask) files="$files $UNIT_DIR/$f" ;;
        *) say "      could not look at $UNIT_DIR/$f; nothing was moved"; RETIRE_ERR=1 ;;
      esac
    done
  done
  if [ -n "$files" ] && [ -z "$RETIRE_ERR" ]; then
    ts=$(date -u +%Y%m%dT%H%M%SZ)   # the PC's clock: the tractor's is not wall time
    for n in "" -2 -3 -4 -5; do
      # plain mkdir fails on an existing directory: an earlier retirement is never reused
      if rtest "sudo -n mkdir -p $RETIRE_ROOT && sudo -n mkdir $RETIRE_ROOT/$ts$n"; then dir=$RETIRE_ROOT/$ts$n; break; fi
    done
    if [ -z "$dir" ]; then
      say "      could not create a new directory under $RETIRE_ROOT, so nothing was moved:$files"
      RETIRE_ERR=1
    else
      out=$(rsh "sudo -n mv$files $dir/ && echo __MOVED__")
      printf '%s\n' "$out" | grep -v -e '^__MOVED__$' -e '^$' | indent
      if printf '%s\n' "$out" | grep -q '^__MOVED__$'; then
        say "      moved to $dir:$files"
        say "      undo: sudo mv them back to $UNIT_DIR/, sudo systemctl daemon-reload, sudo systemctl enable <unit>"
        if ! retire_readme "$ts" "$files" "$dir"; then
          say "      could not write $dir/README.txt: the units are retired, but the directory has no note."
          say "      Write one by hand (a re-run reads ok: the check does not look for the note)."
          RETIRE_ERR=1
        fi
      else
        say "      the move to $dir failed; part of the paths may be there already, without a README.txt."
        say "      Check $UNIT_DIR and $dir on the board, then re-run: what is left goes to a new directory."
        RETIRE_ERR=1
      fi
    fi
  fi
  rsh 'sudo -n systemctl daemon-reload' | indent
  # 'disable' and 'daemon-reload' make systemd re-read its units. On the tractor
  # the camera step must follow (it is the next step); the 2026-10-12 run looked
  # again 5 s after the reload, so give systemd that time first.
  if [ "$ROLE" = tractor ]; then sleep 5; fi
  [ -z "$RETIRE_ERR" ]
}
retired_report() {  # retired_report <1 when the step passed, else 0> <unit>...: info only, changes nothing
  local ok=$1 u f st w j pat names en=0 act=0 left n0=${#RESULTS[@]}
  shift
  for u in "$@"; do
    st=$(unit_state "$u"); w=${st%/*}
    left=1   # 1: the unit's state alone would pass, so a leftover path gets its own line
    case "$w" in enabled*) en=1 ;; esac
    case "$st" in
      ''|/*|*/|*[!a-z/-]*) info "$u: its state could not be read" ;;
      not-found/inactive|not-found/failed|masked/inactive|masked/failed) ;;   # the path lines below say the rest
      enabled/inactive|enabled/failed|disabled/inactive|disabled/failed|static/inactive|static/failed|linked/inactive|linked/failed)
        left=0
        if [ "$CHECK_ONLY" = 1 ]; then
          info "$u is $st: systemd can still load it, so it is not retired; without --check the step disables it and moves its files out of $UNIT_DIR, or says why it cannot"
        else
          info "$u is $st: systemd can still load it, so it is not retired (see the step's messages above)"
        fi ;;
      */inactive|*/failed)
        left=0
        info "$u is $st: the step does not retire a unit in that state (without --check it says why); look at it by hand" ;;
      *)
        left=0
        case "${st##*/}" in active) act=1 ;; esac
        if [ "$CHECK_ONLY" = 1 ]; then
          info "$u is $st: a unit in this state is not retired; without --check the step prints what to do"
        else
          info "$u is $st: not retired; the step's message above says what to do"
        fi ;;
    esac
    j=$(unit_job "$u")
    case "$j" in
      job=) ;;
      job=*[!0-9]*) info "$u: could not read whether systemd has a job queued for it" ;;
      job=*) info "$u: systemd has a job queued for it (job ${j#job=}): not retired until the job is gone" ;;
      *) info "$u: could not read whether systemd has a job queued for it" ;;
    esac
    for f in "$u" "$u.d"; do
      case "$f:$(unit_path_kind "$f")" in
        *:none) [ "$f:$w" != "$u:masked" ] || info "$u is masked, but not by a symlink to /dev/null in $UNIT_DIR: look at it by hand (systemctl status $u)" ;;
        "$u:mask")
          if [ "$w" = masked ]; then info "$UNIT_DIR/$f is a mask (a symlink to /dev/null): left alone"
          else info "$UNIT_DIR/$f is a symlink to /dev/null, but is-enabled reads '$w': look at it by hand"; fi ;;
        *:file) [ "$ok" = 1 ] || [ "$left" = 0 ] || info "$UNIT_DIR/$f is still there: not retired until it is moved to $RETIRE_ROOT" ;;
        *:link|*:mask) [ "$ok" = 1 ] || [ "$left" = 0 ] || info "$UNIT_DIR/$f is a symlink, not a mask: not retired until it is moved to $RETIRE_ROOT" ;;
        *) info "$UNIT_DIR/$f: could not look at it" ;;
      esac
    done
  done
  if [ "$ok" != 1 ] && [ -z "$RETIRE_ERR" ] && [ "${#RESULTS[@]}" = "$n0" ]; then
    info "no unit state or leftover file explains that result: a query to the board may have failed; run it again"
  fi
  # What the retired units left behind. Never removed here: on the tractor the old
  # containers' names are what makes an enabled unit fail at boot, so the order matters.
  if [ "$ROLE" = base ]; then pat='^lifetrac-vtest-'; else pat='^tractor-.*-v2$'; fi
  names=$(rsh "sudo -n docker ps -a --format '{{.Names}}' 2>/dev/null" | grep -E "$pat" | tr '\n' ' ' | sed 's/ $//')
  if [ -n "$names" ]; then
    if [ "$ok" = 1 ]; then
      info "old video-test containers still exist: $names -- not removed here; manual cleanup, in order: BENCH_BOARDS.md"
    elif [ "$act" = 1 ]; then
      info "video-test containers exist: $names -- a unit is active, so these may be its running containers; do not remove them (BENCH_BOARDS.md, rule 3)"
    elif [ "$ROLE" = tractor ] && [ "$en" = 1 ]; then
      info "WARNING: do NOT remove $names while lifetrac-tractor-compose.service is enabled: their names are what makes it fail at boot (BENCH_BOARDS.md)"
    else
      info "old video-test containers exist: $names -- leave them until the units are retired (BENCH_BOARDS.md)"
    fi
  fi
  if [ "$ROLE" = tractor ] && rtest 'test -f /opt/lifetrac/video-test/docker-compose.yml'; then
    info "/opt/lifetrac/video-test/docker-compose.yml still exists: 'docker compose up' in that folder would start the image_tx container on /dev/ttymxc3 -- manual cleanup, in order: BENCH_BOARDS.md"
  fi
}
image_report() {  # image ids for the session record
  local id
  id=$(rval "sudo -n docker image inspect --format '{{.Id}}' $FOUNDRIES_IMG 2>/dev/null" | sed 's/^sha256://' | cut -c1-12)
  [ -n "$id" ] && info "probe image id $id (foundries_python.tar loads 9a454afe48f7)"
  id=$(rval "sudo -n docker image inspect --format '{{.Id}}' $MOSQ_IMG 2>/dev/null" | sed 's/^sha256://' | cut -c1-12)
  [ -n "$id" ] && info "$MOSQ_IMG image id $id (a moving tag: note the id in the session evidence)"
  return 0
}
uart_report() {
  local out h
  if ! rtest 'command -v fuser'; then info "/dev/ttymxc3 holders: fuser not available on this image"; return; fi
  # The marker tells "nobody holds it" from "the query never ran" (as lib/bench_env.sh does).
  out=$(rsh 'sudo -n fuser /dev/ttymxc3 2>/dev/null; echo; echo __UART_CHK_DONE__')
  case $out in
    *__UART_CHK_DONE__*) ;;
    *) info "WARNING: /dev/ttymxc3 (radio UART) holders could not be read (sudo or transport)"; return ;;
  esac
  h=$(printf '%s\n' "$out" | grep -v '__UART_CHK_DONE__' | tr -s ' \n' ' ' | sed 's/^ //; s/ $//')
  if [ -z "$h" ]; then info "/dev/ttymxc3 (radio UART) is free"
  else info "WARNING: /dev/ttymxc3 (radio UART) is held by pid(s) $h"; fi
}

# --------------------------------------------------------------- preflight --
if [ "$TRANSPORT" = adb ]; then
  command -v "$ADB" >/dev/null 2>&1 || die 2 "adb not found; install Android platform-tools (PC_SETUP.md) or set ADB=/path/to/adb"
  state=$(MSYS_NO_PATHCONV=1 "$ADB" -s "$TARGET" get-state 2>/dev/null | tr -d '\r')
  if [ "$state" != device ]; then
    say "adb devices:"; MSYS_NO_PATHCONV=1 "$ADB" devices -l | tr -d '\r' | indent
    die 2 "board $TARGET is not listed as 'device' (state: ${state:-absent}). Set BASE_SERIAL / TRACTOR_SERIAL in $BENCH_ENV or pass --serial. Do not run 'adb kill-server' on a first-boot board."
  fi
else
  [ -f "$BASE_SSH_KEY" ] || die 2 "no ssh key at $BASE_SSH_KEY (first run: use adb; the base step installs the key)"
fi
[ "$(rval 'echo __OK__')" = __OK__ ] || die 2 "cannot run a shell on $TARGET over $TRANSPORT"

MODE=apply; [ "$CHECK_ONLY" = 1 ] && MODE=check
IMAGE_VERSION=$(rval '. /etc/os-release 2>/dev/null; echo ${IMAGE_VERSION:-unknown}')
say "== provision_bench_board.sh: $ROLE on $TARGET over $TRANSPORT ($MODE)"
info "soc serial $(rval 'cat /sys/devices/soc0/serial_number 2>/dev/null'), $(rval '. /etc/os-release 2>/dev/null; echo LmP ${VERSION:-unknown}'), kernel $(rval 'uname -r'), shell user $(rval 'id -un')"
if [ "$IMAGE_VERSION" != 934 ] && [ "$IMAGE_VERSION" != 674 ]; then
  info "WARNING: image $IMAGE_VERSION has never run on this bench (934 = base, 674 = tractor); see BENCH_SETUP.md"
fi

# ------------------------------------------------------------------- steps --
item "sudo without a password (sudoers.d/99-lifetrac-bench-nopasswd)" chk_sudo app_sudo
if ! chk_sudo; then
  [ "$CHECK_ONLY" = 1 ] || die 1 "the sudoers drop-in did not install; is BENCH_SUDO_PW right for this board?"
  say "  note: 'sudo -n' does not work yet, so the root-only checks below read as TODO"
fi

need_compose_mask=0
if [ "$ROLE" = base ] || [ "$MASK_COMPOSE" = 1 ] || [ "$IMAGE_VERSION" != 674 ]; then need_compose_mask=1; fi
if [ "$need_compose_mask" = 1 ]; then
  # On the 934 image compose-apps-early-start.service fails at boot; its OnFailure
  # unit compose-apps-early-start-recovery.service then stops docker and deletes
  # /var/lib/docker, about every 60 s. Mask both before relying on docker.
  item "compose-apps-early-start-recovery.service masked (it wipes /var/lib/docker on 934)" \
       "chk_masked compose-apps-early-start-recovery.service" "app_mask compose-apps-early-start-recovery.service"
  item "compose-apps-early-start.service masked (its failure triggers the recovery unit)" \
       "chk_masked compose-apps-early-start.service" "app_mask compose-apps-early-start.service"
else
  info "image 674: compose-apps-early-start.service succeeds here, left as is (--mask-compose to mask anyway)"
fi

if [ "$ROLE" = base ]; then
  if [ "$NIC_100" = 1 ]; then
    item "eth0 '$NIC_CON' pinned to 100BASE-TX full" chk_nic app_nic
  else
    info "eth0 '$NIC_CON': speed=$(nic_val 802-3-ethernet.speed) duplex=$(nic_val 802-3-ethernet.duplex) auto-negotiate=$(nic_val 802-3-ethernet.auto-negotiate) (--nic-100 pins 100/full)"
  fi
  item "PC ssh key ($PUBKEY) in fio's authorized_keys" chk_sshkey app_sshkey
fi

if [ "$ROLE" = tractor ]; then
  item "units/disable-wifi.service installed in /etc/systemd/system" chk_wifi_unit app_wifi_unit
  item "disable-wifi.service enabled (rfkill block wifi at every boot)" chk_wifi_enabled app_wifi_enabled
  item "wpa_supplicant.service masked" "chk_masked wpa_supplicant.service" "app_mask wpa_supplicant.service"
  item "WiFi soft-blocked now (rfkill)" chk_rfkill app_rfkill
  item "NetworkManager WiFi radio off" chk_nm_wifi app_nm_wifi
  if [ "$KEEP_WIFI" = 1 ]; then
    names=$(list_wifi_profiles | tr '\n' ',' | sed 's/,$//; s/,/, /g')
    info "stored WiFi profiles kept (--keep-wifi-profiles): ${names:-none}"
  else
    names=$(list_wifi_profiles | tr '\n' ',' | sed 's/,$//; s/,/, /g')
    [ -n "$names" ] && say "  stored WiFi profiles found: $names"
    item "no stored NetworkManager WiFi profiles" chk_wifi_profiles app_wifi_profiles
  fi
  item "camera USB provisioning (provision_x8.sh: modprobe, udev, lifetrac-camera.service, fio in video)" chk_camera app_camera
fi

item "docker running" chk_docker app_docker
item "probe image $FOUNDRIES_IMG" "chk_image $FOUNDRIES_IMG" app_foundries
if [ "$ROLE" = base ]; then
  item "$MOSQ_IMG (the base broker image)" "chk_image $MOSQ_IMG" app_mosq_pull
  RETIRE_UNITS="lifetrac-base.service lifetrac-base-compose.service"
else
  item "$MOSQ_IMG (the tractor's local broker; it cannot pull)" "chk_image $MOSQ_IMG" app_mosq_tractor
  RETIRE_UNITS="lifetrac-tractor-compose.service"
fi
# On the tractor this step stays BEFORE the camera step: its 'systemctl disable'
# and 'daemon-reload' make systemd re-read its units, and lifetrac-camera.service
# (started by udev, maps /dev/ttymxc3) is checked, and stopped, after that.
retired_ok=1
item "old boot units retired (no unit file systemd can load, not active, no job queued): $RETIRE_UNITS" \
     "chk_units_retired $RETIRE_UNITS" "app_units_retired $RETIRE_UNITS" || retired_ok=0
if [ "$ROLE" = tractor ]; then
  item "radio UART free: lifetrac-camera.service and tractor-camera stopped" chk_camera_stopped app_camera_stopped
fi
# shellcheck disable=SC2086
retired_report "$retired_ok" $RETIRE_UNITS
image_report
uart_report

# ----------------------------------------------------------------- summary --
say ""
say "== summary: $ROLE $TARGET ($MODE)"
for r in "${RESULTS[@]}"; do say "  $r"; done
say ""
say "Still to do by hand (BENCH_SETUP.md):"
say "  [ ] verify: HC-01 / HC-02 health checks, then '$(basename "$0") $ROLE --check' shows no TODO"
say "  [ ] L072 bench firmware (build md5 0c1bb0a9573f813137f941dfa47177d0): ../FLASH_RUNBOOK.md"
if [ "$ROLE" = base ]; then
  say "  [ ] deploy the base tree, image lifetrac-v25 and the broker: DEPLOY.md (deploy_base.sh)"
  say "  [ ] power: the base boots again after 'systemctl poweroff'; remove power to keep it down"
else
  say "  [ ] tractor image lifetrac-tractor-x8 (built on the base): DEPLOY.md (build_tractor_image.sh)"
  say "  [ ] after every boot: stop lifetrac-camera.service and tractor-camera before any probe;"
  say "      they come back at boot and on camera hotplug and grab /dev/ttymxc3"
fi
say "  [ ] radios: the LifeTrac L072 firmware boots listening (RXCONT); park it with radio_park.py"
say "      if the radios must stay off, and confirm later with radio_state.py"

if [ "$N_FAIL" -gt 0 ]; then exit 1; fi
if [ "$CHECK_ONLY" = 1 ] && [ "$N_TODO" -gt 0 ]; then exit 3; fi
exit 0
