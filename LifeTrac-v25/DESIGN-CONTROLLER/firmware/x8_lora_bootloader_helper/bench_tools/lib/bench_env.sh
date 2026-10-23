# shellcheck shell=bash
# lib/bench_env.sh -- shared settings and helpers for the bench scripts. SOURCE it,
# do not run it:
#
#     . "$(dirname "${BASH_SOURCE[0]}")/../lib/bench_env.sh"
#
# What it does when sourced:
#   * finds the repo root from THIS file's location (git rev-parse --show-toplevel,
#     falling back to the fixed depth below bench_tools/), so the scripts work from
#     any checkout and any working directory;
#   * loads bench_tools/bench.env if it exists, else bench_tools/bench.env.example
#     with a one-line WARN on stderr (or $BENCH_ENV_FILE), through lib/env_file.sh,
#     the loader every bench script shares: a NON-EMPTY variable already in the
#     environment wins over the file; empty values get the computed defaults
#     documented in the example file;
#   * exports MSYS_NO_PATHCONV=1 so Git Bash does not rewrite /tmp/... board paths;
#   * defines the helpers listed below.
# Sourcing it touches nothing on the boards; it only creates $BENCH_SCRATCH.
#
# Paths: on Windows every PC-side path it sets is in mixed form (C:/Users/...),
# which Git Bash, adb, py and PowerShell all accept. Hand Python tools and
# `adb pull` that form, never /c/... (RS-13.1 A6). The one exception is scp
# (BASE_TRANSPORT=ssh), which reads "C:/..." as host "C": board_push and
# board_pull hand it the /c/... form themselves.
#
# Helpers:
#   win_path P                 P in C:/... form (cygpath -m); unchanged off Windows
#   bench_py ARGS...           py -3 (Windows launcher) or python3
#   stamp MSG...               "HH:MM:SSZ MSG" (UTC); also appended to $STAMP_LOG if set
#   die MSG...                 print to stderr, exit 1
#   bench_usage                print the calling script's header comment (for -h)
#   bench_need CMD...          die unless every CMD is on PATH
#   board_role WHO             base | tractor | other   (WHO = role name or adb serial)
#   board_serial WHO           adb serial for WHO
#   board_via WHO              adb | ssh (ssh only for the base with BASE_TRANSPORT=ssh)
#   board_sh WHO CMD...        run CMD in the board's shell; CR stripped; returns its rc.
#                              Put "$SUDO" in front of each command that needs root.
#   board_sudo WHO CMD...      board_sh WHO "$SUDO CMD" (only CMD's first command runs as root)
#   board_present WHO          true when WHO answers (adb device state, or ssh true)
#   board_push WHO SRC DST     copy a file/dir to the board (adb push / scp -r)
#   board_push_lf WHO SRC DST  copy a text file LF-clean (shell, openocd cfg, conf)
#   board_pull WHO SRC DST     copy a board file to the PC (adb pull / scp)
#                              (scp gets the PC path in /c/... form: it reads C:/... as host C)
#   board_image WHO            the probe image for WHO (BASE_IMAGE / TRACTOR_PROBE_IMAGE)
#   board_has_image WHO IMG    true when IMG exists on the board
#   board_uart_holders WHO     pids holding /dev/ttymxc3 (the L072 radio UART), nothing when
#                              free, or "unknown(...)" when the check could not run (no
#                              fuser, sudo or transport failure) -- callers treat any
#                              non-empty answer as busy, so the gate fails closed
#   board_containers WHO       names of the running containers, space separated
#   strip_to_gzip FILE         drop the login-shell preamble adb exec-out puts before a gzip
#   bench_require_dts_carrier  return 1 (with the reason) unless DTS_CARRIER_HZ may be flown
#   bench_check_name WHAT VAL  die unless VAL matches ^[A-Za-z0-9_.-]+$ (leg tags, scene
#                              names: they end up in file names and board shell commands)
#   bench_evidence_guard [--warn-only]
#                              check $EVIDENCE_DIR before anything is written there (below);
#                              returns 1 on a refusal. Messages go to stderr and into
#                              BENCH_EVIDENCE_MSG (call it directly, not in $(...), to keep
#                              that variable for a record started after the check)
#   bench_evidence_dir [--warn-only]
#                              bench_evidence_guard, then mkdir -p $EVIDENCE_DIR and print it
#   bench_evidence_tracked DIR number of files tracked by git in DIR (0 outside a checkout;
#                              'error: <git message>' when git cannot answer)
#   bench_git_desc             "<branch> @ <short sha>[ +dirty]" of this checkout
#   bench_show_env             print the effective settings (password masked)
#
# Constants: BOARD_STAGE=/tmp/lifetrac_strict (the harness hardcodes it),
# BOARD_FLASH_STAGE=/tmp/lifetrac_p0c, SUDO (sudo prefix with the password piped),
# PROBE_RUN (docker run prefix WITH the radio UART mapped -- radio probes only),
# WORK_RUN (the same without any device).
#
# Committed evidence (2026-10-22): bench_evidence_guard REFUSES an EVIDENCE_DIR that
# already holds files tracked by git (in this checkout, or in whichever checkout holds
# that folder): it is a committed evidence record, and a stale bench.env would append a
# new session's files to it (the 10-22 review found the flight bench.env still pointing
# at RS_13_range_edge_2026-10-19/legs). A folder that does not exist yet, holds only
# untracked files or lies outside any checkout passes. When git cannot answer (git
# missing, 'dubious ownership', a broken worktree link) the guard refuses as for a
# committed folder: it never guesses 'not committed'. For a deliberate append, set
# BENCH_EVIDENCE_APPEND=1 in the ENVIRONMENT for that run (prefix the command, or export
# it in that Git Bash window). EVERY script of the leg needs it -- leg2d_switch.sh and
# link_sample.sh refuse the folder without it while the harness is already on the air --
# so leg_prep.sh prints its follow-up lines with the prefix. It is not a bench.env
# setting: a BENCH_EVIDENCE_APPEND line in the settings file is ignored with a WARN, so
# it cannot be left on. --warn-only turns the refusal into a WARNING, for the one
# caller that must not be stopped (leg_post.sh: its park is safety-critical). The guard
# also WARNS when the last YYYY-MM-DD in the EVIDENCE_DIR path is not today (local or
# UTC date). Each warning prints once per process tree (BENCH_EVIDENCE_WARNED is
# exported after a direct call); a refusal prints every time.

[ -n "${BASH_VERSION:-}" ] || { echo "bench_env.sh needs bash" >&2; return 1 2>/dev/null || exit 1; }

export MSYS_NO_PATHCONV=1

win_path() {
  if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else printf '%s\n' "$1"; fi
}

bench_py() {
  if command -v py >/dev/null 2>&1; then py -3 "$@"; else python3 "$@"; fi
}

stamp() {
  local line
  line="$(date -u +%H:%M:%SZ) $*"
  printf '%s\n' "$line"
  if [ -n "${STAMP_LOG:-}" ]; then printf '%s\n' "$line" >> "$STAMP_LOG"; fi
}

die() { echo "$*" >&2; exit 1; }

bench_usage() {                          # print the calling script's header comment
  awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"
}

bench_need() {
  local c
  for c in "$@"; do command -v "$c" >/dev/null 2>&1 || die "needs '$c' on PATH (PC_SETUP.md)"; done
}

# --- locate the checkout ------------------------------------------------------
BENCH_LIB_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
_bench_bt=$(cd "$BENCH_LIB_DIR/.." && pwd)
REPO_ROOT=$(git -C "$_bench_bt" rev-parse --show-toplevel 2>/dev/null || true)
if [ -z "$REPO_ROOT" ] || [ ! -f "$REPO_ROOT/LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/bench_tools/lib/bench_env.sh" ]; then
  REPO_ROOT=$(cd "$_bench_bt/../../../../.." && pwd)          # bench_tools is 5 levels below the root
fi
REPO_ROOT=$(win_path "$REPO_ROOT")
DC="$REPO_ROOT/LifeTrac-v25/DESIGN-CONTROLLER"
HELPER_DIR="$DC/firmware/x8_lora_bootloader_helper"
BT_DIR="$HELPER_DIR/bench_tools"
LEGS_DIR="$BT_DIR/legs"
[ -f "$HELPER_DIR/run_live_radio_monitor.ps1" ] || { echo "bench_env: cannot find the LifeTrac tree (REPO_ROOT=$REPO_ROOT)" >&2; return 1 2>/dev/null || exit 1; }
unset _bench_bt

# --- load bench.env (or the example) -----------------------------------------
_BENCH_VARS="BASE_SERIAL TRACTOR_SERIAL BASE_TRANSPORT BASE_HOST BASE_SSH_USER BASE_SSH_KEY BENCH_SUDO_PW
PC_HOST BENCH_SCRATCH EVIDENCE_DIR ARCHIVE_DIR DTS_CARRIER_HZ DTS_CARRIER_DATE YOUTUBE_URL
FIREFOX_PROFILE_DIR FIREFOX_EXE BASE_IMAGE TRACTOR_PROBE_IMAGE TRACTOR_APP_IMAGE BASE_BROKER_CONTAINER"
# BENCH_ENV is the name provision_bench_board.sh, deploy_base.sh, build_tractor_image.sh
# and flash_l072.sh use for the same thing; accept either.
if [ -z "${BENCH_ENV_FILE:-}" ] && [ -n "${BENCH_ENV:-}" ]; then BENCH_ENV_FILE=$BENCH_ENV; fi
if [ -z "${BENCH_ENV_FILE:-}" ]; then
  if [ -f "$BT_DIR/bench.env" ]; then
    BENCH_ENV_FILE="$BT_DIR/bench.env"
  else
    BENCH_ENV_FILE="$BT_DIR/bench.env.example"
    if [ -z "${BENCH_ENV_EXAMPLE_WARNED:-}" ]; then        # once per process tree
      echo "bench_env: WARN: no bench_tools/bench.env -- using bench.env.example (the original OSE bench's serials and addresses); cp bench.env.example bench.env and edit it (BENCH_SETUP.md 5.3)" >&2
      export BENCH_ENV_EXAMPLE_WARNED=1
    fi
  fi
fi
[ -f "$BENCH_ENV_FILE" ] || { echo "bench_env: settings file not found: $BENCH_ENV_FILE" >&2; return 1 2>/dev/null || exit 1; }
# shellcheck source=env_file.sh
. "$BENCH_LIB_DIR/env_file.sh" || { echo "bench_env: cannot load $BENCH_LIB_DIR/env_file.sh" >&2; return 1 2>/dev/null || exit 1; }
_bench_append=${BENCH_EVIDENCE_APPEND-}         # environment only (2026-10-22): the file may not set it
bench_load_env_file "$BENCH_ENV_FILE" || { return 1 2>/dev/null || exit 1; }   # CRLF-tolerant; the environment wins
if [ "${BENCH_EVIDENCE_APPEND-}" != "$_bench_append" ]; then
  echo "bench_env: WARN: BENCH_EVIDENCE_APPEND in $BENCH_ENV_FILE is ignored -- it is a one-run override for the environment (BENCH_EVIDENCE_APPEND=1 bash <script>, on every script of the leg), never a settings-file value" >&2
  BENCH_EVIDENCE_APPEND=$_bench_append
fi
unset _bench_append

: "${BASE_SERIAL:=2D0A1209DABC240B}"
: "${TRACTOR_SERIAL:=2E2C1209DABC240B}"
: "${BASE_TRANSPORT:=adb}"
: "${BASE_HOST:=192.168.1.117}"
: "${BASE_SSH_USER:=${BASE_USER:-fio}}"          # BASE_USER: the name the setup/deploy scripts use
BASE_USER=$BASE_SSH_USER
: "${BASE_SSH_KEY:=$HOME/.ssh/lifetrac_base_ed25519}"
: "${BENCH_SUDO_PW:=fio}"
: "${PC_HOST:=}"
: "${BENCH_SCRATCH:=$(win_path "${TMPDIR:-${TEMP:-/tmp}}")/lifetrac-bench}"
: "${EVIDENCE_DIR:=$DC/bench-evidence/RS_13_vector_scene_$(date -u +%Y-%m-%d)/legs}"
: "${ARCHIVE_DIR:=${BENCH_ARCHIVE_DIR:-$HOME/Documents/LifeTrac-bench-archive}}"
BENCH_ARCHIVE_DIR=$ARCHIVE_DIR
: "${DTS_CARRIER_HZ:=}"
: "${DTS_CARRIER_DATE:=}"
: "${YOUTUBE_URL:=https://www.youtube.com/watch?v=B1yUQwpNhJA}"
: "${FIREFOX_PROFILE_DIR:=$BENCH_SCRATCH/ff_bench_youtube}"
: "${FIREFOX_EXE:=}"
: "${BASE_IMAGE:=lifetrac-v25:latest}"
: "${TRACTOR_PROBE_IMAGE:=hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44}"
: "${TRACTOR_APP_IMAGE:=lifetrac-tractor-x8:latest}"
: "${BASE_BROKER_CONTAINER:=design-controller-mosquitto-1}"
case $BASE_TRANSPORT in adb|ssh) ;; *) echo "bench_env: BASE_TRANSPORT must be adb or ssh, not '$BASE_TRANSPORT'" >&2; return 1 2>/dev/null || exit 1;; esac
case $BENCH_SUDO_PW in *"'"*) echo "bench_env: BENCH_SUDO_PW must not contain a single quote" >&2; return 1 2>/dev/null || exit 1;; esac

BASE_SSH_KEY=$(win_path "$BASE_SSH_KEY")
BENCH_SCRATCH=$(win_path "$BENCH_SCRATCH")
EVIDENCE_DIR=$(win_path "$EVIDENCE_DIR")
ARCHIVE_DIR=$(win_path "$ARCHIVE_DIR")
FIREFOX_PROFILE_DIR=$(win_path "$FIREFOX_PROFILE_DIR")
mkdir -p "$BENCH_SCRATCH"

# Everything but the password is exported, so nested scripts, PowerShell and the
# Python tools see the same values.
export REPO_ROOT DC HELPER_DIR BT_DIR LEGS_DIR BENCH_ENV_FILE
export BASE_SERIAL TRACTOR_SERIAL BASE_TRANSPORT BASE_HOST BASE_SSH_USER BASE_SSH_KEY PC_HOST BENCH_SCRATCH \
       EVIDENCE_DIR ARCHIVE_DIR DTS_CARRIER_HZ DTS_CARRIER_DATE YOUTUBE_URL FIREFOX_PROFILE_DIR FIREFOX_EXE \
       BASE_IMAGE TRACTOR_PROBE_IMAGE TRACTOR_APP_IMAGE BASE_BROKER_CONTAINER

# --- constants ------------------------------------------------------------------
BOARD_STAGE=/tmp/lifetrac_strict          # run_live_radio_monitor.ps1 mounts exactly this as /work
BOARD_FLASH_STAGE=/tmp/lifetrac_p0c       # flash tooling + the harness's openocd reset cfg
SUDO="echo '$BENCH_SUDO_PW' | sudo -S -p ''"
PROBE_RUN="docker run --rm --network=host --entrypoint python3 --device=/dev/ttymxc3 -v $BOARD_STAGE:/work -w /work -e PYTHONPATH=/work:/work/paho"
WORK_RUN="docker run --rm --network=host --entrypoint python3 -v $BOARD_STAGE:/work -w /work -e PYTHONPATH=/work:/work/paho"
# accept-new: a PC without a known_hosts entry for the base would otherwise fail every
# BatchMode call (deploy_base.sh does the same); a CHANGED host key still fails.
BENCH_SSH_OPTS=(-i "$BASE_SSH_KEY" -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new)

# --- board helpers ----------------------------------------------------------------
board_role() {
  case "$1" in
    base|"$BASE_SERIAL") echo base ;;
    tractor|"$TRACTOR_SERIAL") echo tractor ;;
    *) echo other ;;
  esac
}

board_serial() {
  case "$1" in
    base) echo "$BASE_SERIAL" ;;
    tractor) echo "$TRACTOR_SERIAL" ;;
    *) echo "$1" ;;
  esac
}

board_via() {
  if [ "$(board_role "$1")" = base ] && [ "$BASE_TRANSPORT" = ssh ]; then echo ssh; else echo adb; fi
}

board_sh() {
  local who=$1 ser
  shift
  ser=$(board_serial "$who")
  if [ "$(board_via "$who")" = ssh ]; then
    ssh "${BENCH_SSH_OPTS[@]}" "$BASE_SSH_USER@$BASE_HOST" "$*" < /dev/null | tr -d '\r'
    return "${PIPESTATUS[0]}"
  fi
  adb -s "$ser" shell "$*" < /dev/null | tr -d '\r'
  return "${PIPESTATUS[0]}"
}

board_sudo() {                           # board_sh with the password piped into sudo for the first command
  local who=$1
  shift
  board_sh "$who" "$SUDO $*"
}

board_present() {
  local who=$1 ser
  ser=$(board_serial "$who")
  if [ "$(board_via "$who")" = ssh ]; then
    ssh "${BENCH_SSH_OPTS[@]}" "$BASE_SSH_USER@$BASE_HOST" true < /dev/null > /dev/null 2>&1
    return
  fi
  adb devices 2>/dev/null | tr -d '\r' | awk 'NR>1 && $2=="device"{print $1}' | grep -qx "$ser" \
    && adb -s "$ser" shell true < /dev/null > /dev/null 2>&1
}

board_push() {
  local who=$1 src dst=$3 ser
  src=$(win_path "$2")
  ser=$(board_serial "$who")
  [ -e "$src" ] || { echo "board_push: no such file: $src" >&2; return 1; }
  if [ "$(board_via "$who")" = ssh ]; then
    if command -v cygpath >/dev/null 2>&1; then src=$(cygpath -u "$src"); fi   # not C:/... (host "C")
    scp -q -r "${BENCH_SSH_OPTS[@]}" "$src" "$BASE_SSH_USER@$BASE_HOST:$dst" < /dev/null
  else
    adb -s "$ser" push "$src" "$dst" < /dev/null > /dev/null
  fi
}

board_push_lf() {                         # CRLF trap: board shell scripts / cfg / conf must be LF
  local tmp
  mkdir -p "$BENCH_SCRATCH/lf"
  tmp="$BENCH_SCRATCH/lf/$(basename "$2")"
  tr -d '\r' < "$2" > "$tmp" || return 1
  board_push "$1" "$tmp" "$3"
}

board_pull() {
  local who=$1 src=$2 dst ser
  dst=$(win_path "$3")
  ser=$(board_serial "$who")
  if [ "$(board_via "$who")" = ssh ]; then
    if command -v cygpath >/dev/null 2>&1; then dst=$(cygpath -u "$dst"); fi   # not C:/... (host "C")
    scp -q -r "${BENCH_SSH_OPTS[@]}" "$BASE_SSH_USER@$BASE_HOST:$src" "$dst" < /dev/null
  else
    adb -s "$ser" pull "$src" "$dst" < /dev/null > /dev/null
  fi
}

board_image() {
  case "$(board_role "$1")" in
    base) echo "$BASE_IMAGE" ;;
    *) echo "$TRACTOR_PROBE_IMAGE" ;;
  esac
}

board_has_image() {
  [ "$(board_sh "$1" "$SUDO docker image inspect -f ok $2 2>/dev/null")" = ok ]
}

board_uart_holders() {
  # The markers tell "nobody holds it" apart from "the check never ran" (fuser missing,
  # sudo refused, transport down): the latter answers unknown(...), never "free".
  local out
  out=$(board_sh "$1" "$SUDO sh -c 'if command -v fuser >/dev/null 2>&1; then fuser /dev/ttymxc3 2>/dev/null; echo; echo __UART_CHK_DONE__; else echo __UART_NO_FUSER__; fi'")
  case $out in
    *__UART_NO_FUSER__*) echo "unknown(no fuser on the board)"; return 0 ;;
    *__UART_CHK_DONE__*) ;;
    *) echo "unknown(check failed: sudo or transport)"; return 0 ;;
  esac
  printf '%s\n' "$out" | grep -v '__UART_CHK_DONE__' | tr -s ' \n' ' ' | sed 's/^ //; s/ $//'
}

board_containers() {
  board_sh "$1" "$SUDO docker ps --format '{{.Names}}'" | tr '\n' ' ' | sed 's/ $//'
}

strip_to_gzip() {
  bench_py -c "import sys;p=sys.argv[1];d=open(p,'rb').read();i=d.find(b'\x1f\x8b\x08');assert 0<=i<64,i;open(p,'wb').write(d[i:])" "$1"
}

bench_require_dts_carrier() {
  local f=${DTS_CARRIER_HZ:-} today
  today=$(date +%Y-%m-%d)
  if [ -z "$f" ]; then
    echo "REFUSED: DTS_CARRIER_HZ is empty. Run a receive-only channel spot-check today with the tractor parked" >&2
    echo "         (RS13_VECTOR_LEG.md 'Pin the DTS carrier'), then set DTS_CARRIER_HZ and DTS_CARRIER_DATE in $BT_DIR/bench.env." >&2
    return 1
  fi
  case $f in *[!0-9]*) echo "REFUSED: DTS_CARRIER_HZ='$f' is not an integer number of Hz" >&2; return 1 ;; esac
  if [ "$f" -lt 902250000 ] || [ "$f" -gt 927750000 ]; then
    echo "REFUSED: DTS_CARRIER_HZ=$f is outside 902250000..927750000 (the 500 kHz channel must stay inside 902-928 MHz)" >&2
    return 1
  fi
  if [ "$f" = 915000000 ] && [ "${DTS_ALLOW_915:-0}" != 1 ]; then
    echo "REFUSED: 915000000 is the channel of the known RS-11.6 external emitter (RS-13.1 A19). Pick another carrier, or set DTS_ALLOW_915=1 to fly there on purpose." >&2
    return 1
  fi
  if [ -n "${DTS_CARRIER_DATE:-}" ]; then
    if [ "$DTS_CARRIER_DATE" != "$today" ]; then
      echo "REFUSED: the spot-check behind DTS_CARRIER_HZ=$f is dated $DTS_CARRIER_DATE, not today ($today). The band changes from day to day: re-check." >&2
      return 1
    fi
  else
    echo "WARN: DTS_CARRIER_DATE is empty -- make sure the spot-check for $f Hz was made today." >&2
  fi
  return 0
}

bench_check_name() {                     # bench_check_name WHAT VALUE
  case $2 in
    ''|*[!A-Za-z0-9_.-]*) die "$1 '$2' is not allowed: use only letters, digits, '_', '.' and '-' (it goes into file names and board shell commands)" ;;
  esac
}

bench_evidence_tracked() {               # bench_evidence_tracked DIR: files git tracks in DIR
  # A folder not made yet is asked about from its nearest existing parent. git runs in
  # that folder, so the checkout that holds it answers (C:/... works from Git Bash with
  # MSYS_NO_PATHCONV=1); outside any checkout the answer is 0. Any other git failure
  # (git missing, "dubious ownership", a broken worktree link) prints "error: <git's
  # message>" instead of a count, so the guard never passes a folder it could not check.
  local d=$1 rel="" p err out
  while [ "${#d}" -gt 1 ] && [ "${d%/}" != "$d" ]; do d=${d%/}; done
  while [ ! -d "$d" ]; do
    p=$(dirname "$d")
    [ "$p" != "$d" ] || { echo 0; return 0; }
    rel="$(basename "$d")${rel:+/$rel}"
    d=$p
  done
  command -v git >/dev/null 2>&1 || { echo "error: git is not on PATH"; return 0; }
  if ! err=$(git -C "$d" rev-parse --is-inside-work-tree 2>&1 >/dev/null); then
    # Only git's "outside any checkout" message means 0; a broken .git link says
    # "not a git repository: <path>" without the "(or any" part.
    case $err in *"not a git repository (or any"*) echo 0; return 0 ;; esac
    echo "error: $(printf '%s\n' "$err" | head -n 1)"; return 0
  fi
  if ! out=$(git --literal-pathspecs -C "$d" ls-files -- "${rel:-.}" 2>&1); then
    echo "error: $(printf '%s\n' "$out" | head -n 1)"; return 0
  fi
  printf '%s\n' "$out" | grep -c .
}

bench_evidence_guard() {                 # [--warn-only]; the rules are in the header
  local warn_only=0 n last today_l today_u msg=""
  [ "${1:-}" = --warn-only ] && warn_only=1
  BENCH_EVIDENCE_MSG=""
  n=$(bench_evidence_tracked "$EVIDENCE_DIR")
  case $n in
    error:*)      # git could not answer: refuse unless asked, never guess "not committed"
      if [ "${BENCH_EVIDENCE_APPEND:-}" = 1 ]; then
        msg="NOTE: BENCH_EVIDENCE_APPEND=1 -- git could not say whether EVIDENCE_DIR=$EVIDENCE_DIR holds committed files (${n#error: }); writing there on purpose"
      elif [ $warn_only = 1 ]; then
        msg="WARNING: git could not say whether EVIDENCE_DIR=$EVIDENCE_DIR holds committed files (${n#error: }). This step must not be stopped, so its files go there anyway: check that folder before the next run."
      else
        BENCH_EVIDENCE_MSG="REFUSED: git could not say whether EVIDENCE_DIR=$EVIDENCE_DIR holds committed files (${n#error: }).
         Fix git in that checkout (for 'dubious ownership': git config --global --add safe.directory <repo>),
         or, to write there anyway, run this and every other script of the leg with BENCH_EVIDENCE_APPEND=1 in the environment (not in bench.env)."
        printf '%s\n' "$BENCH_EVIDENCE_MSG" >&2
        return 1
      fi
      n=0 ;;
    ''|*[!0-9]*) n=0 ;;
  esac
  if [ "$n" -gt 0 ]; then
    if [ "${BENCH_EVIDENCE_APPEND:-}" = 1 ]; then
      msg="NOTE: BENCH_EVIDENCE_APPEND=1 -- appending on purpose to EVIDENCE_DIR=$EVIDENCE_DIR, a committed evidence record ($n file(s) tracked by git)"
    elif [ $warn_only = 1 ]; then
      msg="WARNING: EVIDENCE_DIR=$EVIDENCE_DIR already holds $n file(s) tracked by git (a committed evidence record). This step must not be stopped, so its files go there anyway: move them out (or commit them on purpose) and point EVIDENCE_DIR in $BENCH_ENV_FILE at a new folder before the next run. (If this append was deliberate, run every script of the leg with BENCH_EVIDENCE_APPEND=1: this becomes a NOTE.)"
    else
      BENCH_EVIDENCE_MSG="REFUSED: EVIDENCE_DIR=$EVIDENCE_DIR already holds $n file(s) tracked by git: it is a committed evidence record, and this run would add its files to it.
         Point EVIDENCE_DIR in $BENCH_ENV_FILE at a new folder for this session (e.g. bench-evidence/<campaign>_$(date +%Y-%m-%d)/legs),
         or, to append to that record on purpose, run this and every other script of the leg with BENCH_EVIDENCE_APPEND=1 in the environment (not in bench.env)."
      printf '%s\n' "$BENCH_EVIDENCE_MSG" >&2
      return 1
    fi
  fi
  if [[ $EVIDENCE_DIR =~ ^.*([0-9]{4}-[0-9]{2}-[0-9]{2}) ]]; then     # the last date in the path
    last=${BASH_REMATCH[1]}
    today_l=$(date +%Y-%m-%d); today_u=$(date -u +%Y-%m-%d)
    if [ "$last" != "$today_l" ] && [ "$last" != "$today_u" ]; then
      msg="${msg:+$msg
}WARNING: EVIDENCE_DIR=$EVIDENCE_DIR is dated $last, not today ($today_l): is it this session's folder? (EVIDENCE_DIR in $BENCH_ENV_FILE)"
    fi
  fi
  if [ -n "$msg" ] && [ "${BENCH_EVIDENCE_WARNED:-}" != "$EVIDENCE_DIR" ]; then
    BENCH_EVIDENCE_MSG=$msg
    printf '%s\n' "$msg" >&2
    export BENCH_EVIDENCE_WARNED=$EVIDENCE_DIR      # once per process tree (from a direct call)
  fi
  return 0
}

bench_evidence_dir() {                   # [--warn-only]
  bench_evidence_guard "$@" || return 1
  mkdir -p "$EVIDENCE_DIR" || return 1
  printf '%s\n' "$EVIDENCE_DIR"
}

bench_git_desc() {
  local b s d=""
  b=$(git -C "$REPO_ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null || echo '?')
  s=$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo '?')
  git -C "$REPO_ROOT" diff --quiet HEAD -- 2>/dev/null || d=" +dirty"
  echo "$b @ $s$d"
}

bench_show_env() {
  local v
  echo "settings file: $BENCH_ENV_FILE"
  echo "repo:          $REPO_ROOT ($(bench_git_desc))"
  for v in $_BENCH_VARS; do
    [ "$v" = BENCH_SUDO_PW ] && { echo "BENCH_SUDO_PW=(set, not shown)"; continue; }
    echo "$v=${!v}"
  done
  [ -z "${BENCH_EVIDENCE_APPEND:-}" ] || echo "BENCH_EVIDENCE_APPEND=$BENCH_EVIDENCE_APPEND (environment only; 1 = appends to a committed EVIDENCE_DIR allowed)"
}
