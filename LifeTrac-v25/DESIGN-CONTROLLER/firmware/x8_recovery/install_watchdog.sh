#!/bin/sh
# install_watchdog.sh -- install, remove or check the L2 hardware-watchdog drop-ins
# on one Portenta X8 (LmP, systemd 250). L2 of
# X8_HEALTH_AND_RECOVERY/REBOOT_AND_POWER_CYCLE.md; TODO.md PWR-2.
#
#   sudo sh install_watchdog.sh check
#   sudo sh install_watchdog.sh install --role base
#   sudo sh install_watchdog.sh install --role bench --bench-opt-in
#   sudo sh install_watchdog.sh install --role tractor --i-accept-tractor-prereqs
#   sudo sh install_watchdog.sh remove
#
# Push the whole firmware/x8_recovery folder with LF endings (e.g. to
# /tmp/x8_recovery): the drop-ins are copied from systemd/ next to this script,
# so the repo files stay the only copy of their content. POSIX sh and coreutils
# only; the LmP host python is not used.
#
# What it writes (2026-10-17), nothing else:
#   $LIFETRAC_ETC_SYSTEMD/system.conf.d/90-lifetrac-watchdog.conf
#       RuntimeWatchdogSec=30s, RebootWatchdogSec=2min: PID 1 owns WDOG1, and a
#       hung PID 1 or kernel ends in a PMIC power cycle about 30 s later.
#   $LIFETRAC_ETC_SYSTEMD/system/reboot.target.d/90-lifetrac-timeout.conf
#       JobTimeoutSec=3min, JobTimeoutAction=reboot-force: a reboot stuck
#       stopping units is forced after 3 min instead of 30.
#   ($LIFETRAC_ETC_SYSTEMD defaults to /etc/systemd; the SIL test points it at
#   a temp dir.) Each file is copied to a dot-name in the target directory,
#   then renamed over the old one and synced; systemd ignores the dot-name
#   because it does not end in .conf. /etc on LmP is writable; the files
#   survive OTA updates through OSTree's 3-way /etc merge and show up in
#   `ostree admin config-diff` (capture_board_state.sh records it).
#
# When it goes live -- treat 'install' as arming, 'remove' as disarming:
#   systemd 250 re-reads system.conf.d at EVERY manager reload, not only at
#   daemon-reexec or boot (v250 src/core/main.c MANAGER_RELOAD ->
#   parse_configuration() -> set_manager_settings() -> manager_set_watchdog()
#   -> watchdog_setup(); read in the source 2026-10-17, not yet seen on a
#   board). A reload is `systemctl daemon-reload`, and also any `systemctl
#   enable`, `disable`, `reenable`, `preset`, `link`, `mask`, `unmask`,
#   `revert` or `edit` run without --no-reload (v250 systemctl-enable.c and
#   systemctl-edit.c reload after the change; 2026-10-17 review). So the 30 s
#   runtime watchdog arms at the next reload or daemon-reexec caused by
#   ANYONE -- x8_recovery/README.md's supervisor install, provision_x8.sh,
#   bench_tools/provision_bench_board.sh (`systemctl mask`, `enable`,
#   `disable`) and any unit install all cause one -- or at the next boot.
#   `check` (RuntimeWatchdogUSec) is the only reliable reading of the armed
#   state. After 'remove' the next one closes /dev/watchdog0 with 'V' and
#   disarms it. This script runs neither; it prints the command.
#
# What it never does:
#   - systemctl daemon-reexec / daemon-reload / reboot (see above).
#   - open, write or close /dev/watchdog0 ('check' runs `fuser`, which reads
#     /proc/*/fd and never opens the device). A stray close without 'V' is a
#     power cycle 60 s later (FLASH_RUNBOOK.md section 3).
#   - touch /proc/sysrq-trigger, the PMIC, u-boot env, or any other file.
#
# Role gate ('install' needs --role; check and remove do not):
#   base     production base station: installs, and says that production
#            arming waits on the PWR-1 shutdown-timing and staged-OTA reboot
#            checks (systemd/reboot.target.d/90-lifetrac-timeout.conf: the
#            3 min bound can cut ostree-finalize-staged short).
#   bench    a bench board (one carrying the bench marker below), whichever
#            role it plays: needs --bench-opt-in. Refused on a board without
#            the marker (2026-10-17 review: --role bench on a production
#            tractor skipped the tractor gate).
#            An unattended reboot on the bench wipes the /tmp staging and
#            re-triggers the tractor's lifetrac-camera.service, which grabs
#            /dev/ttymxc3 (the L072 UART); on the bench tractor it also
#            reflashes H747 bank 1 and resets the H747 (BENCH_BOARDS.md).
#            It arms at PID 1's next reload, whoever causes it (see above).
#   tractor  refused unless --i-accept-tractor-prereqs: every reboot, watchdog
#            power cycles included, reflashes and resets the H747 until the
#            TODO.md PWR prerequisites land (PWR-8: no per-boot bank-1 erase by
#            stm32h7-program, X8OS-3; PWR-9: H747 nonce RNG, H7-13; PWR-11:
#            what the Opta's engine-kill latch does after an H747 reset is
#            settled).
#   The role is the operator's word, so the bench marker overrides it
#   (2026-10-17 review): a board carrying the bench-only sudoers drop-in
#   ($LIFETRAC_BENCH_MARKER, default /etc/sudoers.d/99-lifetrac-bench-nopasswd,
#   from install_lifetrac_nopasswd.sh via provision_bench_board.sh) is a bench
#   board whatever --role says -- the only base X8 today is bench board 2D0A --
#   so base and tractor there need --bench-opt-in too and get the bench warning.
#
# Exit: 0 done (check always exits 0 and prints L2_DROPINS=<state>),
#       1 usage, 2 a file operation failed, 3 refused by the role gate.
set -u
PATH=$PATH:/usr/sbin:/sbin   # appended, so a PATH-prepended fake (SIL test) still wins
export PATH

# follow a symlinked invocation to the real script, so systemd/ is found next
# to it (readlink -f: coreutils and busybox both have it; else $0 as given)
SELF=$(readlink -f "$0" 2>/dev/null) || SELF=""
[ -n "$SELF" ] || SELF=$0
SRC_DIR=$(cd "$(dirname "$SELF")" && pwd) || exit 2
ETC=${LIFETRAC_ETC_SYSTEMD:-/etc/systemd}
BENCH_MARKER=${LIFETRAC_BENCH_MARKER:-/etc/sudoers.d/99-lifetrac-bench-nopasswd}
WDT_DEV=${WDT_DEV:-/dev/watchdog0}
PET_PIDFILE=${WDT_PET_DIR:-/tmp/lifetrac_p0c}/wdt_pet.pid   # wdt_pet.sh's
WD_REL=system.conf.d/90-lifetrac-watchdog.conf
RT_REL=system/reboot.target.d/90-lifetrac-timeout.conf
WD_SRC=$SRC_DIR/systemd/system.conf.d/90-lifetrac-watchdog.conf
RT_SRC=$SRC_DIR/systemd/reboot.target.d/90-lifetrac-timeout.conf

usage() {
  cat >&2 <<'EOF'
usage: sh install_watchdog.sh check
       sh install_watchdog.sh install --role base
       sh install_watchdog.sh install --role bench --bench-opt-in
       sh install_watchdog.sh install --role tractor --i-accept-tractor-prereqs
       sh install_watchdog.sh remove
EOF
  exit "${1:-1}"
}

# run a read-only query with a 5 s cap: systemctl blocks if PID 1 is wedged
q() {
  if command -v timeout >/dev/null 2>&1; then timeout 5 "$@"; else "$@"; fi
}

same_file() {   # same_file <a> <b>: identical bytes (cksum: coreutils, no cmp on LmP)
  [ -f "$1" ] && [ -f "$2" ] && [ "$(cksum < "$1")" = "$(cksum < "$2")" ]
}

put() {   # put <src> <dst>: atomic copy, mode 0644
  dir=$(dirname "$2")
  if same_file "$1" "$2"; then
    echo "unchanged  $2"
    return 0
  fi
  mkdir -p "$dir" || return 2
  tmp="$dir/.$(basename "$2").tmp.$$"
  if cp "$1" "$tmp" && chmod 0644 "$tmp" && mv -f "$tmp" "$2"; then
    echo "installed  $2"
    return 0
  fi
  rm -f "$tmp"
  echo "install_watchdog: FAILED to write $2" >&2
  return 2
}

holders() {   # who has $WDT_DEV open, by pid and command name
  command -v fuser >/dev/null 2>&1 || { echo "fuser not installed; holder unknown"; return 0; }
  who=""
  for p in $(fuser "$WDT_DEV" 2>/dev/null); do   # pids on stdout, "dev:" label on stderr
    p=${p%%[!0-9]*}
    [ -n "$p" ] && who="$who $p($(cat "/proc/$p/comm" 2>/dev/null))"
  done
  echo "$WDT_DEV held by:${who:- nobody visible (the kernel keepalive pets it; non-root sees only its own fds)}"
}

bench_warning() {
  cat <<'EOF'
BENCH WARNING: with the runtime watchdog armed, a PID 1 or kernel stall becomes an
unattended PMIC power cycle about 30 s later. On the bench that wipes the /tmp staging
(/tmp/lifetrac_p0c, /tmp/lifetrac_strict) and re-triggers the tractor's
lifetrac-camera.service, which grabs /dev/ttymxc3 (the L072 UART): stop it again before
any radio work. On the bench tractor each power cycle also runs stm32h7-program.service,
which erases and reflashes H747 bank 1 and resets the H747 (TODO.md PWR-8). It arms at
PID 1's next reload, whoever causes it: `systemctl daemon-reload`, or any `systemctl
enable/disable/mask/unmask/preset/revert/edit` run without --no-reload. When the session
that needed it is over: remove, then daemon-reload.
EOF
}

tractor_refusal() {
  cat <<'EOF'
TRACTOR: refused. Every reboot, a watchdog power cycle included, runs
stm32h7-program.service, which erases and reflashes H747 bank 1 and resets the H747.
In production that trips the engine-kill latch through the Opta, and each reset
restarts the H747's nonce state. Arm the tractor only after the TODO.md PWR
prerequisites land: PWR-8, no per-boot bank-1 erase by stm32h7-program (X8OS-3);
PWR-9, H747 nonce seeds from its RNG (H7-13); and PWR-11, the Opta latch semantics
after an H747 reset settled. When they have, re-run with --i-accept-tractor-prereqs
(PID 1's next reload, anyone's, then arms it).
EOF
}

next_steps() {
  cat <<'EOF'
Not armed by this script, but it arms at PID 1's next reload or at the next boot. A
reload is `systemctl daemon-reload` or `daemon-reexec`, whoever runs it (any other
installer's too, such as the supervisor install in x8_recovery/README.md), and also any
`systemctl enable/disable/mask/unmask/preset/revert/edit` run without --no-reload. From
then on PID 1 owns /dev/watchdog0 at 30 s and a 3 min reboot.target timeout applies.
To arm it now:
    sudo systemctl daemon-reload
First run `bash wdt_pet.sh status`; if a petter is running, `bash wdt_pet.sh stop` it (or
let the flash finish) before any reload. While a petter holds the device systemd's open
gets EBUSY and it retries: on a PID 1 that never owned the device that only delays the
open, but after an earlier arm and disarm in the same boot PID 1 retries in a busy loop,
stalling 1 s at a time, until the petter closes (v250 source; not seen on a board).
Then verify:
    sudo sh install_watchdog.sh check   (RuntimeWatchdogUSec=30s, held by 1(systemd))
EOF
}

base_note() {
  cat <<'EOF'
BASE: install for the PWR-1 checks. Production arming of the base, where OTAs run, waits
on PWR-1 timing a normal `systemctl reboot` on this board and one reboot with a staged
OTA deployment: the 3 min reboot.target bound can cut ostree-finalize-staged short and
the update is then silently not applied (systemd/reboot.target.d/90-lifetrac-timeout.conf).
EOF
}

cmd=${1:-}
[ $# -gt 0 ] && shift
ROLE=""
BENCH_OK=0
TRACTOR_OK=0
while [ $# -gt 0 ]; do
  case "$1" in
    --role)   [ $# -ge 2 ] || usage 1; ROLE=$2; shift 2 ;;
    --role=*) ROLE=${1#--role=}; shift ;;
    --bench-opt-in) BENCH_OK=1; shift ;;
    --i-accept-tractor-prereqs) TRACTOR_OK=1; shift ;;
    -h|--help) usage 0 ;;
    *) echo "install_watchdog: unknown argument '$1'" >&2; usage 1 ;;
  esac
done

case "$cmd" in
  install)
    case "$ROLE" in
      base|bench|tractor) ;;
      "") echo "install_watchdog: install needs --role base|tractor|bench" >&2; usage 1 ;;
      *)  echo "install_watchdog: unknown role '$ROLE' (base|tractor|bench)" >&2; usage 1 ;;
    esac
    if [ "$ROLE" = tractor ] && [ "$TRACTOR_OK" -ne 1 ]; then
      tractor_refusal >&2
      exit 3
    fi
    # bench is a statement about the board, so the marker must back it: a
    # production tractor installed as "bench" would skip the tractor gate
    if [ "$ROLE" = bench ] && [ ! -e "$BENCH_MARKER" ]; then
      echo "install_watchdog: refused: not a bench board (no $BENCH_MARKER): use --role base or --role tractor" >&2
      exit 3
    fi
    # bench gate: --role bench, or any role on a board with the bench marker
    if [ "$ROLE" = bench ] || [ -e "$BENCH_MARKER" ]; then
      bench_warning
      if [ "$BENCH_OK" -ne 1 ]; then
        if [ "$ROLE" = bench ]; then
          echo "install_watchdog: refused: --role bench needs --bench-opt-in" >&2
        else
          echo "install_watchdog: refused: $BENCH_MARKER marks this as a bench board; --role $ROLE here needs --bench-opt-in too" >&2
        fi
        exit 3
      fi
    fi
    [ "$ROLE" = tractor ] && echo "TRACTOR: installing on the operator's word that the PWR prerequisites have landed."
    [ "$ROLE" = base ] && base_note
    for f in "$WD_SRC" "$RT_SRC"; do
      [ -f "$f" ] || { echo "install_watchdog: missing $f (push the whole x8_recovery folder)" >&2; exit 2; }
    done
    rc=0
    put "$WD_SRC" "$ETC/$WD_REL" || rc=2
    put "$RT_SRC" "$ETC/$RT_REL" || rc=2
    sync
    [ "$rc" -eq 0 ] && next_steps
    # any later reload races a live petter, and that is not harmless (a busy
    # PID 1 after an earlier arm/disarm in this boot, see next_steps): say so
    if [ "$rc" -eq 0 ] && [ -f "$PET_PIDFILE" ] && kill -0 "$(cat "$PET_PIDFILE" 2>/dev/null)" 2>/dev/null; then
      echo "NOTE: wdt_pet.sh's petter (pid $(cat "$PET_PIDFILE")) holds $WDT_DEV now: run 'bash wdt_pet.sh stop' (or let the flash finish) before any reload, as above."
    fi
    exit "$rc"
    ;;

  remove)
    rc=0
    for rel in "$WD_REL" "$RT_REL"; do
      if [ -e "$ETC/$rel" ]; then
        if rm -f "$ETC/$rel"; then echo "removed    $ETC/$rel"; else rc=2; fi
      else
        echo "absent     $ETC/$rel"
      fi
    done
    sync
    cat <<'EOF'
Not disarmed by this script. At PID 1's next reload (`systemctl daemon-reload` or
`daemon-reexec`, whoever runs it, or any
`systemctl enable/disable/mask/unmask/preset/revert/edit` without --no-reload) or the
next boot, PID 1 closes /dev/watchdog0 with the magic 'V' and the kernel keepalive takes
over again (the hardware timeout stays 30 s until u-boot re-arms 60 s at the next boot
-- inferred). To disarm it now:
    sudo systemctl daemon-reload
EOF
    exit "$rc"
    ;;

  check)
    n_inst=0
    n_stale=0
    n_unver=0
    echo "== L2 drop-ins under $ETC =="
    for pair in "$WD_REL:$WD_SRC" "$RT_REL:$RT_SRC"; do
      rel=${pair%%:*}
      src=${pair#*:}
      if [ ! -e "$ETC/$rel" ]; then
        echo "absent     $ETC/$rel"
      elif [ ! -f "$src" ]; then
        # only this script was pushed: no repo copy to compare against, which
        # says nothing about the installed file
        echo "present    $ETC/$rel (repo copy not found at $src; cannot compare)"
        n_inst=$((n_inst + 1))
        n_unver=$((n_unver + 1))
      elif same_file "$src" "$ETC/$rel"; then
        echo "installed  $ETC/$rel"
        n_inst=$((n_inst + 1))
      else
        echo "DIFFERS    $ETC/$rel (not the repo copy at $src)"
        n_inst=$((n_inst + 1))
        n_stale=$((n_stale + 1))
      fi
    done
    if [ "$n_inst" -eq 0 ]; then state=absent
    elif [ "$n_inst" -eq 1 ]; then state=partial
    elif [ "$n_stale" -gt 0 ]; then state=differs
    elif [ "$n_unver" -gt 0 ]; then state=present-unverified
    else state=installed
    fi
    echo "L2_DROPINS=$state"

    echo "== effective settings (read-only) =="
    if command -v systemctl >/dev/null 2>&1; then
      # one query, parsed below: a wedged PID 1 costs one 5 s timeout, not three
      out=$(q systemctl show -p RuntimeWatchdogUSec -p RebootWatchdogUSec 2>&1)
      [ -n "$out" ] && echo "$out"
      rwd=$(echo "$out" | sed -n 's/^RuntimeWatchdogUSec=//p' | head -n 1)
      case "$rwd" in
        "")
          # an empty answer is NOT "off": systemctl failed, timed out or this is
          # not a systemd boot. Don't steer anyone into reloading a wedged PID 1.
          echo "NOTE: systemctl gave no RuntimeWatchdogUSec within 5 s (PID 1 wedged, or not systemd); armed state unknown" ;;
        0|0s|off)
          [ "$state" = absent ] || echo "NOTE: drop-in present but PID 1 has not re-read it: its next reload (anyone's daemon-reload, or a systemctl enable/disable/mask without --no-reload) or boot arms it; now: sudo systemctl daemon-reload" ;;
        *)
          [ "$state" = absent ] && echo "NOTE: armed (RuntimeWatchdogUSec=$rwd) without our drop-in: another system.conf file sets it (below), or 'remove' ran and no daemon-reload since" ;;
      esac
      [ -n "$rwd" ] && q systemctl show -p JobTimeoutUSec -p JobTimeoutAction reboot.target 2>&1 | sed 's/^/reboot.target /'
    else
      echo "systemctl not found"
    fi
    if command -v systemd-analyze >/dev/null 2>&1; then
      # every system.conf file in the order PID 1 reads it (later wins); a later
      # drop-in that sets RuntimeWatchdogSec would silently override ours
      echo "-- system.conf files and their watchdog keys (systemd-analyze cat-config) --"
      q systemd-analyze cat-config systemd/system.conf 2>&1 | grep -E '^# /|^[[:space:]]*[A-Za-z]*Watchdog'
    fi

    echo "== $WDT_DEV holder (fuser; the device is not opened) =="
    holders
    exit 0
    ;;

  -h|--help) usage 0 ;;
  *) usage 1 ;;
esac
