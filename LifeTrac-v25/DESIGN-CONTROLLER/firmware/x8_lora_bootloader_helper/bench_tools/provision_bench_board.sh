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
#            rs116_health_probe.py run in it)
#   base     compose-apps-early-start{,-recovery}.service masked, the PC's ssh
#            key in fio's authorized_keys, eclipse-mosquitto:2 pulled,
#            optional eth0 100BASE-TX full pin (--nic-100)
#   tractor  WiFi off for good (units/disable-wifi.service enabled,
#            wpa_supplicant masked, nmcli radio wifi off, stored WiFi profiles
#            deleted), camera USB provisioning (../provision_x8.sh),
#            eclipse-mosquitto:2 loaded from the base or a tarball; the
#            compose-apps mask too, unless the board still runs the factory
#            674 image (where that unit succeeds)
#
# Radio safety: this script never opens /dev/ttymxc3 (the L072 radio UART),
# never starts lifetrac-base.service, lifetrac-base-compose.service or
# lifetrac-tractor-compose.service, and on the tractor it leaves
# lifetrac-camera.service and the tractor-camera container stopped (both map
# /dev/ttymxc3; they come back at the next boot).
#
# Settings come from bench.env next to this script when it exists (copy
# bench.env.example), else these defaults (the original bench pair):
#   BASE_SERIAL=2D0A1209DABC240B  TRACTOR_SERIAL=2E2C1209DABC240B
#   BASE_HOST=192.168.1.117       BENCH_SUDO_PW=fio (used once, to install sudoers)
#   BASE_SSH_KEY=~/.ssh/lifetrac_base_ed25519  BASE_USER=fio  ADB=adb
# Values already in bench.env win over the defaults. The password is never printed.

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
if [ -f "$BENCH_ENV" ]; then
  # strip CR so a bench.env saved by a Windows editor does not put \r into serials
  # shellcheck disable=SC1090
  . <(sed 's/\r$//' "$BENCH_ENV")
fi
BASE_SERIAL=${BASE_SERIAL:-2D0A1209DABC240B}
TRACTOR_SERIAL=${TRACTOR_SERIAL:-2E2C1209DABC240B}
BASE_HOST=${BASE_HOST:-192.168.1.117}
BENCH_SUDO_PW=${BENCH_SUDO_PW:-fio}
BASE_SSH_KEY=${BASE_SSH_KEY:-$HOME/.ssh/lifetrac_base_ed25519}
BASE_USER=${BASE_USER:-${BASE_SSH_USER:-fio}}
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

# on <adb|ssh> <serial|host> <command>: run a shell command on a board, CR stripped
on() {
  if [ "$1" = ssh ]; then
    ssh -i "$BASE_SSH_KEY" -o BatchMode=yes -o ConnectTimeout=10 "$BASE_USER@$2" "$3" 2>&1 | tr -d '\r'
  else
    MSYS_NO_PATHCONV=1 "$ADB" -s "$2" shell "$3" 2>&1 | tr -d '\r'
  fi
}
push_to() {    # push_to <adb|ssh> <serial|host> <local file> <remote path>
  if [ "$1" = ssh ]; then
    scp -q -i "$BASE_SSH_KEY" -o BatchMode=yes "$3" "$BASE_USER@$2:$4"
  else
    MSYS_NO_PATHCONV=1 "$ADB" -s "$2" push "$(winpath "$3")" "$4" >/dev/null
  fi
}
pull_from() {  # pull_from <adb|ssh> <serial|host> <remote path> <local file>
  if [ "$1" = ssh ]; then
    scp -q -i "$BASE_SSH_KEY" -o BatchMode=yes "$BASE_USER@$2:$3" "$4"
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

# tractor: lifetrac-camera.service and the tractor-camera container map /dev/ttymxc3
chk_camera_stopped() {
  ! rtest 'systemctl is-active -q lifetrac-camera.service' \
    && rtest "sudo -n docker ps >/dev/null && ! sudo -n docker ps --format '{{.Names}}' | grep -qx tractor-camera"
}
app_camera_stopped() {
  rsh 'sudo -n systemctl stop lifetrac-camera.service 2>/dev/null; sudo -n docker stop tractor-camera >/dev/null 2>&1; true' >/dev/null
}

unit_report() {  # never started or changed here
  local u st
  for u in "$@"; do
    st=$(rval "echo \$(systemctl is-enabled $u 2>/dev/null || true)/\$(systemctl is-active $u 2>/dev/null || true)")
    case "$st" in
      /*|not-found/*|/inactive) ;;   # not installed: nothing to say
      *) info "$u is $st -- left alone; do not start it on the bench (it would run the radio stack)" ;;
    esac
  done
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
  local h
  if ! rtest 'command -v fuser'; then info "/dev/ttymxc3 holders: fuser not available on this image"; return; fi
  h=$(rsh 'sudo -n fuser /dev/ttymxc3 2>/dev/null' | tr -s ' \n' ' ' | sed 's/^ //; s/ $//')
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
  unit_report lifetrac-base.service lifetrac-base-compose.service
else
  item "$MOSQ_IMG (the tractor's local broker; it cannot pull)" "chk_image $MOSQ_IMG" app_mosq_tractor
  item "radio UART free: lifetrac-camera.service and tractor-camera stopped" chk_camera_stopped app_camera_stopped
  unit_report lifetrac-tractor-compose.service
fi
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
