#!/bin/sh
# lifetrac-supervisor.sh -- L3 host health supervisor for the Portenta X8
# (base and tractor): health checks with a bounded escalation ladder.
# DISARMED BY DEFAULT. Part of L3 in
# X8_HEALTH_AND_RECOVERY/REBOOT_AND_POWER_CYCLE.md (TODO.md PWR-4).
#
# Usage:
#   lifetrac-supervisor.sh run         # loop forever (lifetrac-supervisor.service)
#   lifetrac-supervisor.sh once        # one pass, then exit (tests; manual use)
#   lifetrac-supervisor.sh status      # print the state files, change nothing
#   lifetrac-supervisor.sh clear-hold  # leave HELD and zero the reboot counter
#
# `once` takes the same actions as `run` when the board is armed. Do not run it
# by hand on an armed board while the service is active (they share state, and
# on a tractor one instance's start can withdraw the other's motion-hold
# request; the look right before `systemctl reboot` catches that, but there is
# no lock between instances).
#
# Why it is shaped this way (2026-10-17, from the adversarial review of the
# reboot plan):
#   - POSIX sh, no Python. The tractor's LmP host Python 3.10 has no json,
#     select, socket or subprocess-capable modules. Every external command
#     runs under `timeout N`, so a wedged dockerd cannot hang the loop. Not
#     bounded: a file write, rename or `sync` stuck on wedged storage. A
#     process in uninterruptible I/O cannot be killed by `timeout` or by the
#     unit watchdog (VERIFY on the bench), and nothing on the X8 recovers a
#     supervisor stuck that way while PID 1 is healthy: L2's hardware
#     watchdog bites only if PID 1 or the kernel stalls too (2026-10-17
#     review). The backstop is L4 (design only), which stops getting the L3
#     heartbeat.
#   - No wall-clock time anywhere. The tractor has no battery-backed RTC (its
#     clock is not wall time) and the base clock steps when it syncs at boot.
#     Every rate limit, grace period and staleness test is in seconds of
#     uptime (/proc/uptime) inside one boot (/proc/sys/kernel/random/boot_id).
#     A heartbeat file is stale when its mtime VALUE has not changed for more
#     than max_age seconds of uptime, never "now minus mtime".
#   - Bench rule: no supervisor-initiated reboot or container restart on a
#     bench board without an explicit GO. So nothing is restarted or rebooted
#     unless $LIFETRAC_SUPERVISOR_ARMED_FILE (/etc/lifetrac/supervisor.armed)
#     exists. Disarmed, every check still runs and each action it would have
#     taken is logged as "WOULD <action>".
#   - Tractor: any X8 reboot resets the H747 (stm32h7-program.service
#     reflashes it at every tractor boot; a PMIC cycle resets it too). In the
#     production design that trips the Opta's 200 ms Modbus watchdog, drops
#     the PSR and kills the engine behind a latch that needs a person at the
#     machine. So with ROLE=tractor every reboot rung is OFF unless
#     TRACTOR_REBOOT_ENABLED=1; a reboot that would be needed is logged and put
#     in the status file, and nothing else happens. TODO.md PWR-8, PWR-9,
#     PWR-11, PWR-12 and PWR-13 gate turning it on.
#   - Tractor, idle only (DECISIONS.md D-PWR1, Dorkmo 2026-10-18). Even with
#     TRACTOR_REBOOT_ENABLED=1 a tractor reboots only while the H747 reports
#     idle (engine stopped, every valve output off) and has latched a motion
#     hold for this reboot, so the tractor cannot be moved between the check
#     and the reset (an orderly shutdown can take minutes). The H747 talks to
#     Linux through a link service (TODO.md PWR-13, after X8OS-3 picks the
#     link) and the two files below; no such service exists yet, so today
#     this gate blocks every tractor reboot. Contract:
#       $IDLE_FILE (/run/lifetrac/tractor_idle), written by the link service
#       at least once a second, tmp + rename, in a directory the link service
#       owns. First line:
#         busy <seq>          not idle, or the H747 does not know
#         idle <seq>          engine stopped and every valve output off
#         locked <seq> <id>   idle, and the H747 holds motion off for
#                             reboot request <id>
#       <seq> is a token from the H747's own status frames (1-32 letters or
#       digits) that changes at least once a second; the link service copies
#       it and never makes one up. State, <seq> and <id> come from one H747
#       frame. A file whose <seq> does not change while the supervisor
#       watches is not live and never reads as idle; a stopped link service
#       leaves its last line in place. Until PWR-12 the H747 reports busy
#       while the key is off. Anything other than busy, idle or locked reads
#       as "invalid"; the file is read at most 256 bytes at a time, by an
#       open that refuses a symlink and does not block (GNU dd
#       iflag=nofollow,nonblock; a dd without them fails, which reads as a
#       missing file: VERIFY on LmP under PWR-1).
#       $REQ_FILE (/run/lifetrac-supervisor/tractor_reboot_request), in a
#       root-only directory the supervisor creates (0755, never
#       RuntimeDirectory=, which would delete it when the shutdown stops
#       us); the link service only reads it. First line <id>: 1-40 letters,
#       digits or "-" (the supervisor writes <boot id prefix>-<uptime>). The
#       link service mirrors it to the H747 continuously. The H747 latches
#       the hold when it sees a request while idle, keeps it while busy, and
#       releases it only when it resets or the link service reports that no
#       request file exists. A new <id> while it holds keeps the hold and
#       re-latches it for the new <id>. A link service that stops (shutdown)
#       leaves the hold in place.
#     The supervisor waits up to TRACTOR_IDLE_WAIT_S polls (one a second) for
#     a live "idle", writes the request, waits again for a live
#     "locked <seq> <id>", and looks once more right before it issues the
#     reset. Before that point it withdraws the request (deletes the file)
#     on any path where the reboot does not happen. Once the reset is issued
#     ($RESET_FILE, next to the request, records it for this boot) the
#     request stays until the reset, whatever happens next: an accepted
#     `systemctl reboot` can still land after this process gives up on it.
#     Never overridden, OTA probation included.
#   - ROLE has no permissive default (2026-10-17 review). A missing config,
#     a file that does not set ROLE, a ROLE line the shell could not run
#     (`ROLE = tractor`, a UTF-8 BOM), an invalid value, or a file that
#     fails to load all mean ROLE=tractor. Only
#     an explicit ROLE=base in a cleanly loaded file turns the reboot rung on
#     without TRACTOR_REBOOT_ENABLED=1, so a typo on a tractor cannot.
#   - Docker never restarts a container just for being "unhealthy" (restart
#     policies react to exit only), so this is the piece that acts on the
#     compose healthcheck status (PWR-3) of the containers in CONTAINERS.
#     That allowlist ships empty and no LifeTrac container is safe on it yet
#     (2026-10-17 review): a broker restart silently drops the subscriptions
#     of clients that never resubscribe (lora_bridge's cmd/estop on the base),
#     web_ui republishes retained radio settings, and the image daemons reset
#     the L072. See supervisor.conf.example.
#
# Each pass (CHECK_INTERVAL_S, default 30 s):
#   1. `docker info` (CHECK_DOCKER=1).
#   2. `docker inspect` Name, State.Status and State.Health.Status of each
#      container in the CONTAINERS allowlist. Unhealthy = not "running", or
#      health "unhealthy", or missing. After FAIL_THRESHOLD bad passes in a
#      row: `docker restart`, at most CONTAINER_RESTART_MAX times per
#      container per boot. A container whose restarts are used up, or whose
#      next restart cannot be recorded (state not writable), is a node-level
#      fault and is not restarted. Docker resolves a name it cannot find as an ID
#      prefix, so hex-only names are refused at config time and a container
#      whose inspected Name is not "/<name>" is never restarted.
#   3. HEARTBEATS (path:max_age_s): stale as described above. A missing file
#      is stale once max_age has passed since this boot's supervisor started
#      watching it.
#   4. Node-level faults (docker down, stale heartbeat, container restarts used
#      up) present FAIL_THRESHOLD passes in a row -> the reboot rung, behind
#      these gates, applied in this order and each logged with a reason code:
#        grace    no reboot before REBOOT_GRACE_S of uptime (default 600 s).
#        tractor  ROLE=tractor and TRACTOR_REBOOT_ENABLED!=1. Never overridden.
#        HELD     $STATE/held exists: no reboots until `clear-hold`
#                 (container restarts continue). Logged every pass.
#        cap      $STATE/consecutive_reboots >= cap -> enter HELD. cap is
#                 REBOOT_MAX_CONSECUTIVE, raised to max(cap, bootlimit + 1)
#                 while `fw_printenv -n upgrade_available` is 1 (OTA trial
#                 boot).
#        idle     ROLE=tractor: a live "idle" or "locked" in $IDLE_FILE
#                 (D-PWR1, above). Read-only, so it runs disarmed too. Never
#                 overridden.
#        armed    the armed file; disarmed -> "WOULD reboot".
#        lock     ROLE=tractor: the request is written and the H747 must
#                 answer with a live "locked <seq> <id>"; otherwise the
#                 request is withdrawn and the reboot blocked. Checked again
#                 (request and idle file, one read) right before
#                 `systemctl reboot`.
#      OTA trial boot (review finding, 2026-10-17): raising the cap alone did
#      not let u-boot's rollback fire when the counter carried over from
#      before the update, or when a held file was left from before it. So
#      while upgrade_available=1 and `fw_printenv -n bootcount` <= bootlimit
#      (rollback not due yet), neither HELD nor the cap stops the reboot; u-boot
#      bounds the loop (its rollback_setup clears upgrade_available). The
#      override also needs u-boot's bootcount to have risen after the
#      supervisor's own probation reboot in the previous boot, in case the env
#      copy is stale (images that keep the flag in RPMB, fiovb.*: VERIFY
#      during the first OTA trial). Bench boot
#      env: bootlimit=5, bootcount and upgrade_available in /mnt/boot/uboot.env,
#      no fiovb_rpmb (bench-evidence/board_state_2026-10-04/base/03_bootenv.txt).
#      The counter resets to 0 once the node has been healthy (every check
#      passing) for HEALTHY_RESET_S seconds of uptime in one boot. HELD stays
#      until a person clears it.
#
# Reboot rung (review finding C3): persist consecutive_reboots+1, write the
# reset-reason record $STATE/last_action (tmp file, `timeout 10 sync`, mv),
# then `timeout 30 systemctl reboot`. If that fails or times out, the forced
# path runs at once. If it is accepted and this process is still alive
# FORCE_REBOOT_AFTER_S later (a shutdown job hung before it stopped us; once it
# has stopped us, the reboot.target JobTimeoutSec drop-in in
# systemd/reboot.target.d bounds the hang where L2 is installed, and on a
# tractor, which has no L2 under D-PWR1, upstream's 30 min reboot.target
# bound does, with motion held), the forced path runs only if the
# gates still pass (armed, not HELD, tractor gate, and on a tractor the
# motion hold, below), a node fault is still present and the pending marker
# is not stale (older than
# FORCE_REBOOT_AFTER_S + 2 x CHECK_INTERVAL_S, i.e. the supervisor was not
# running meanwhile); otherwise the marker is dropped, logged, and the normal
# ladder resumes. "Later" counts from the uptime read right after systemd
# accepted the reboot, not from the start of the pass (2026-10-17 review:
# the rung's own syncs and fw_printenv calls could eat the whole window, so
# the marker read as stale and a second counted reboot was issued instead of
# the forced path). The forced path records its intent in last_action, then
# writes s, u, b to /proc/sysrq-trigger as three separate writes
# SYSRQ_SETTLE_S apart, then `systemctl reboot -ff` as the last resort (also
# the only step when the sysrq file does not exist). Never sysrq 'c': a plain
# reboot on the i.MX8MM most likely already asserts WDOG_B -> PMIC power cycle
# (TF-A IMX_WDOG_B_RESET; VERIFY on the bench, TODO.md PWR-1), so a crash rung
# buys nothing and loses the sync.
# On a tractor the forced path first looks at the motion hold: it forces when
# $IDLE_FILE shows a live "locked <seq> <id>" for the request on disk, or
# when the link service is not live (stopped by the shutdown) and its last
# line is that same "locked <seq> <id>" (the H747 keeps the hold until it
# resets). It does not force when the request file is gone, the idle file is
# missing, the live link does not show the hold, or the frozen last line is
# anything else (busy, idle, a hold for another request). It looks again
# (request still ours, idle file still showing that hold) after writing
# last_action, after the s settle (before u, which remounts the disks
# read-only and cannot be undone, so from u on it goes ahead), and without
# sysrq after a bounded sync right before `systemctl reboot -ff`, which syncs
# again before it resets. If the hold is gone it gives up; when no reset was
# issued earlier in this boot, the request is withdrawn.
#
# Reset reason (no telemetry route exists yet; a container can read the file):
# at startup in a new boot, if $STATE/last_action names the boot id stored in
# $STATE/boot_id by the previous run, $STATE/last_reset_reason becomes
# "supervisor:<action>:<reason>", otherwise "external_or_unknown". <action> is
# reboot or force_reboot, or reboot_abandoned: a supervisor reboot that was
# requested but then did not happen and will not be forced (pending marker
# dropped by a gate, as stale or because the node is healthy again, or a
# forced path that returned). The record is rewritten, not deleted, at that
# point (2026-10-17 review): the board stays up in that boot with no time
# limit, and a later external reset used to read as a supervisor reboot.
# reboot_abandoned says the cause is uncertain; it is also what a late
# completion of that systemd reboot reads as. Limitation: a power cut between
# writing last_action and the reboot reads as supervisor.
#
# NEVER-DO list. Enforced by not implementing any of it; do not add it here.
# Each is a bench tool with its own runbook under X8_HEALTH_AND_RECOVERY/
# recovery/, and none is safe unattended:
#   - touch a GPIO (gpio10 = H7 NRST, gpio163 = L072 NRST) or run openocd;
#   - rmmod / insmod / modprobe any x8h7_* module;
#   - restart stm32h7-program.service or docker.service;
#   - open or write /dev/ttymxc3 (the L072 UART);
#   - call x8_max_carrier_recovery.sh or any L072 flash / unprotect / AN3155
#     tool;
#   - restart a container that is not in the CONTAINERS allowlist;
#   - write sysrq 'c' (crash).
#
# Files (STATE = ${LIFETRAC_SUPERVISOR_STATE:-/var/lib/lifetrac/supervisor}):
#   status, supervisor.log (+ .1 past 256 KiB), boot_id, last_action,
#   last_reset_reason, consecutive_reboots, held, probation_reboot; per_boot/
#   holds this boot's streaks, restart counts, the previous boot id and a
#   pending reboot, and is wiped when the boot id changes. Tractor only, on
#   tmpfs: $IDLE_FILE (read) and $REQ_FILE (written, see D-PWR1 above), and
#   $RESET_FILE (boot id, uptime and request id of a reset issued in this
#   boot). A request left from an earlier run is withdrawn at start unless
#   a reboot is pending or a reset was issued in this boot.
#   To release a kept hold by hand: `systemctl cancel` the queued reboot, then
#   rm $REQ_FILE. The next rung then treats that reset as cancelled (a request
#   only a person can have removed) and clears $RESET_FILE.
# Config: ${LIFETRAC_SUPERVISOR_CONF:-/etc/lifetrac/supervisor.conf}, see
# supervisor.conf.example. Test overrides: LIFETRAC_SUPERVISOR_ARMED_FILE,
# LIFETRAC_SYSRQ, LIFETRAC_PROC_UPTIME, LIFETRAC_BOOT_ID_FILE,
# LIFETRAC_TRACTOR_IDLE_FILE, LIFETRAC_TRACTOR_REBOOT_REQUEST,
# LIFETRAC_IDLE_POLL (sleep between idle-file polls, default 1).
# SIL tests: base_station/tests/test_x8_supervisor_sil.py.

set -u
set -f   # never glob: container names and paths are expanded unquoted on purpose

CONF=${LIFETRAC_SUPERVISOR_CONF:-/etc/lifetrac/supervisor.conf}
STATE=${LIFETRAC_SUPERVISOR_STATE:-/var/lib/lifetrac/supervisor}
ARMED_FILE=${LIFETRAC_SUPERVISOR_ARMED_FILE:-/etc/lifetrac/supervisor.armed}
SYSRQ=${LIFETRAC_SYSRQ:-/proc/sysrq-trigger}
PROC_UPTIME=${LIFETRAC_PROC_UPTIME:-/proc/uptime}
BOOT_ID_FILE=${LIFETRAC_BOOT_ID_FILE:-/proc/sys/kernel/random/boot_id}
IDLE_FILE=${LIFETRAC_TRACTOR_IDLE_FILE:-/run/lifetrac/tractor_idle}
REQ_FILE=${LIFETRAC_TRACTOR_REBOOT_REQUEST:-/run/lifetrac-supervisor/tractor_reboot_request}
RESET_FILE=${REQ_FILE%/*}/reset_issued
IDLE_POLL=${LIFETRAC_IDLE_POLL:-1}
case $IDLE_POLL in
  ''|.*|*.|*[!0-9.]*|*.*.*) IDLE_POLL=1 ;;
esac
case $IDLE_POLL in
  *[1-9]*) ;;
  *) IDLE_POLL=1 ;;   # 0 would make the liveness window too short to see
esac

RUN=$STATE/per_boot
LOG=$STATE/supervisor.log
LOG_MAX_BYTES=262144
NL='
'
CR=$(printf '\r')
TAB=$(printf '\t')
CONF_KEYS="ROLE CHECK_INTERVAL_S FAIL_THRESHOLD CONTAINERS HEARTBEATS CHECK_DOCKER CONTAINER_RESTART_MAX REBOOT_MAX_CONSECUTIVE HEALTHY_RESET_S REBOOT_GRACE_S FORCE_REBOOT_AFTER_S SYSRQ_SETTLE_S TRACTOR_REBOOT_ENABLED TRACTOR_IDLE_WAIT_S"
INSPECT_FMT='{{.Name}} {{.State.Status}} {{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}'

UP=
BOOT_ID=unknown
BOOT8=unknown
STATE_OK=0
ARMED=0
HELD=0
CONF_STATE=defaults
CAP=0
PROBATION=0
PROBATION_BC=
PENDING_DUE=0
PENDING_EL=
PENDING_RSN=
IDLE_WHY=
IDLE_LIVE=0
IDLE_ST=
IDLE_SEQ=
IDLE_RID=
IDLE_SEEN=
TF_ID=
RESET_ISSUED=0

set_defaults() {
  ROLE=tractor   # fail-safe, see the header: only an explicit ROLE=base is base
  CHECK_INTERVAL_S=30
  FAIL_THRESHOLD=3
  CONTAINERS=""
  HEARTBEATS=""
  CHECK_DOCKER=1
  CONTAINER_RESTART_MAX=3
  REBOOT_MAX_CONSECUTIVE=3
  HEALTHY_RESET_S=1800
  REBOOT_GRACE_S=600
  FORCE_REBOOT_AFTER_S=180
  SYSRQ_SETTLE_S=3
  TRACTOR_REBOOT_ENABLED=0
  TRACTOR_IDLE_WAIT_S=10
}

# ------------------------------------------------------------------ helpers
# Helpers use their own variable prefixes: POSIX sh has no `local`.
# Redirections put 2>/dev/null FIRST: they are processed left to right, so in
# `printf x > "$f" 2>/dev/null` a failing `> "$f"` still reports on the
# journal's stderr, every pass, when the state dir is unusable (2026-10-17).

# to_int VALUE: set INT to VALUE without leading zeros, if VALUE is 1-9
# decimal digits; fail otherwise. dash and bash both read "010" as octal and
# choke on "08", and fw_printenv / a hand-edited config can hand us either.
to_int() {
  case $1 in ''|*[!0-9]*) return 1 ;; esac
  [ ${#1} -le 9 ] || return 1
  _ti=${1#"${1%%[!0]*}"}
  INT=${_ti:-0}
}

# rc_note RC: set RCNOTE to a readable suffix for timeout/exec exit codes.
rc_note() {
  case $1 in
    124) RCNOTE="(timeout)" ;;
    125) RCNOTE="(timeout itself failed)" ;;
    126) RCNOTE="(not executable)" ;;
    127) RCNOTE="(command not found)" ;;
    *) RCNOTE="" ;;
  esac
}

log() {
  _lg="boot=$BOOT8 up=${UP:-?} $*"
  printf '%s\n' "$_lg"
  if [ "$STATE_OK" = 1 ]; then
    printf '%s\n' "$_lg" 2>/dev/null >> "$LOG"
  fi
  return 0
}

rotate_log() {
  [ "$STATE_OK" = 1 ] || return 0
  [ -f "$LOG" ] || return 0
  _rl=$(timeout 5 stat -c %s "$LOG" 2>/dev/null) || return 0
  to_int "$_rl" || return 0
  if [ "$INT" -gt "$LOG_MAX_BYTES" ]; then
    mv -f "$LOG" "$LOG.1" 2>/dev/null
  fi
  return 0
}

# kv_get FILE DEFAULT: set KV to the first line of FILE, or DEFAULT.
kv_get() {
  KV=$2
  [ -f "$1" ] || return 0
  _kg=
  { read -r _kg < "$1"; } 2>/dev/null || [ -n "$_kg" ] || return 0
  KV=$_kg
}

# kv_int FILE DEFAULT: set INT from FILE, or DEFAULT when absent or garbage.
kv_int() {
  kv_get "$1" "$2"
  to_int "$KV" || INT=$2
}

# kv_put FILE VALUE: per-pass scratch state. Written only when it changes
# (eMMC wear), tmp + rename, no sync: losing it in a crash costs one boot's
# streak counts, and per_boot/ is discarded at the next boot anyway.
kv_put() {
  kv_get "$1" ""
  if [ -f "$1" ] && [ "$KV" = "$2" ]; then
    return 0
  fi
  if printf '%s\n' "$2" 2>/dev/null > "$1.tmp" && mv -f "$1.tmp" "$1" 2>/dev/null; then
    return 0
  fi
  rm -f "$1.tmp" 2>/dev/null
  return 1
}

# write_record FILE CONTENT: a record that must survive the reboot it
# precedes. Tmp file, best-effort sync, rename, then sync again so the rename
# itself is on disk (`systemctl reboot -ff` syncs too, but a sysrq b does not).
# `timeout 10 sync` bounds a slow sync, not one stuck in uninterruptible I/O
# on wedged storage (see the header).
write_record() {
  if ! printf '%s\n' "$2" 2>/dev/null > "$1.tmp"; then
    rm -f "$1.tmp" 2>/dev/null
    log "ERROR cannot write $1.tmp"
    return 1
  fi
  timeout 10 sync >/dev/null 2>&1
  if ! mv -f "$1.tmp" "$1" 2>/dev/null; then
    rm -f "$1.tmp" 2>/dev/null
    log "ERROR cannot rename $1.tmp"
    return 1
  fi
  timeout 10 sync >/dev/null 2>&1
  return 0
}

read_uptime() {
  UP=
  _ru=
  { read -r _ru _ru_idle < "$PROC_UPTIME"; } 2>/dev/null || [ -n "$_ru" ] || return 1
  to_int "${_ru%%.*}" || return 1
  UP=$INT
}

read_boot_id() {
  BOOT_ID=unknown
  _rb=
  { read -r _rb < "$BOOT_ID_FILE"; } 2>/dev/null || [ -n "$_rb" ] || _rb=
  case $_rb in
    ''|*[!0-9A-Fa-f-]*) ;;
    *) BOOT_ID=$_rb ;;
  esac
  if [ ${#BOOT_ID} -le 8 ]; then
    BOOT8=$BOOT_ID
  else
    BOOT8=${BOOT_ID%"${BOOT_ID#????????}"}
  fi
}

notify_watchdog() {
  [ -n "${NOTIFY_SOCKET:-}" ] || return 0
  command -v systemd-notify >/dev/null 2>&1 || return 0
  # Blocking form on purpose (systemd >= 246 waits for PID 1 to take the
  # message): with --no-block the short-lived sender can exit before PID 1
  # maps its PID to our cgroup and the ping is dropped. NotifyAccess=all in
  # the unit lets a child of the main process send it.
  timeout 10 systemd-notify WATCHDOG=1 >/dev/null 2>&1 || {
    _nw=$?
    log "WARN systemd-notify WATCHDOG=1 failed rc=$_nw"
  }
  return 0
}

# ------------------------------------------------------------------- config
# The config runs in a throw-away subshell, so a syntax error, an `exit` or a
# stray function in it cannot touch the supervisor. Only the known keys come
# back, whitespace-collapsed, and are then validated one by one.
#
# The file is rejected as a whole (every key takes its default, ROLE=tractor)
# when it does not parse, exits, its last command fails, or it writes ANYTHING
# to stderr. The last check is the one that matters (review finding,
# 2026-10-17): `.` only reports the last command's status, so `ROLE = tractor`
# (INI-style), a UTF-8 BOM before ROLE=, or an unquoted `CONTAINERS=a b` on
# any other line ran as "not found" commands, were silently skipped, and the
# file still read as loaded. Each such line writes "not found" to stderr.
#
# CR is stripped from every line first and also counts as whitespace in
# values: supervisor.conf.example is not covered by the repo's .gitattributes
# LF rules, so a Windows checkout copies it with CRLF, and a blank CRLF line
# would otherwise run "\r" as a command.
load_config() {
  set_defaults
  if [ ! -e "$CONF" ]; then
    CONF_STATE="missing(defaults)"
    log "CONFIG $CONF missing: every key takes its default, including ROLE=tractor (reboot rungs gated off unless the file sets ROLE=base)"
    return 0
  fi
  _lc_dump=$(
    exec 2>&1 </dev/null   # the file's diagnostics come back ahead of the marker
    # shellcheck disable=SC2086
    unset $CONF_KEYS
    _lc_src=
    _lc_unk=
    while IFS= read -r _lc_l || [ -n "$_lc_l" ]; do
      _lc_l=${_lc_l%"$CR"}
      _lc_src=$_lc_src$_lc_l$NL
      # Warn about KEY= lines the supervisor does not know (role=, typos).
      _lc_t=${_lc_l#"${_lc_l%%[! "$TAB"]*}"}
      case $_lc_t in
        *=*)
          _lc_k=${_lc_t%%=*}
          case $_lc_k in
            ''|[0-9]*|*[!A-Za-z0-9_]*) ;;
            *)
              case " $CONF_KEYS " in
                *" $_lc_k "*) ;;
                *) _lc_unk="$_lc_unk $_lc_k" ;;
              esac ;;
          esac ;;
      esac
    done < "$CONF" || exit 4
    set +u   # a config line like X="$X more" must not abort the load
    eval "$_lc_src" >/dev/null || exit 3
    set -u
    echo __SOURCED__
    IFS=" $TAB$CR$NL"
    for _lc_k in $CONF_KEYS; do
      eval "_lc_set=\${$_lc_k+x}"
      [ -n "$_lc_set" ] || continue
      eval "_lc_v=\$$_lc_k"
      # shellcheck disable=SC2086
      set -- $_lc_v
      printf '%s=%s\n' "$_lc_k" "$*"
    done
    for _lc_k in $_lc_unk; do
      printf '__UNKNOWN__=%s\n' "$_lc_k"
    done
    echo __LOADED__
  )
  _lc_rc=$?
  case $_lc_dump in
    *__LOADED__) ;;
    *) [ "$_lc_rc" -ne 0 ] || _lc_rc=1 ;;
  esac
  _lc_diag=${_lc_dump%%__SOURCED__*}
  if [ "$_lc_rc" -eq 0 ] && [ -n "$_lc_diag" ]; then
    _lc_rc=5
  fi
  if [ "$_lc_rc" -ne 0 ]; then
    # Every key keeps its default, ROLE=tractor included: the file that
    # failed may well have said ROLE=tractor (same safe side as an invalid
    # ROLE value below).
    CONF_STATE="unloadable(rc=$_lc_rc,defaults,role=tractor)"
    _lc_n=0
    while IFS= read -r _lc_l; do
      [ -n "$_lc_l" ] || continue
      _lc_n=$((_lc_n + 1))
      if [ "$_lc_n" -le 3 ]; then
        log "CONFIG $CONF: $_lc_l"
      fi
    done <<EOF
$_lc_diag
EOF
    log "CONFIG $CONF failed to load (rc=$_lc_rc: a line the shell could not run, a syntax error, exit, or a non-zero last command); using ALL defaults, ROLE=tractor (reboot rungs gated off: the file may have named a tractor)"
    return 0
  fi
  CONF_STATE=loaded
  _lc_role=0
  while IFS= read -r _lc_line; do
    case $_lc_line in
      __UNKNOWN__=*)
        log "CONFIG unknown key '${_lc_line#__UNKNOWN__=}' in $CONF ignored (keys are upper case; see supervisor.conf.example)"
        CONF_STATE="loaded(with-warnings)"
        continue ;;
    esac
    _lc_k=${_lc_line%%=*}
    _lc_v=${_lc_line#*=}
    case " $CONF_KEYS " in
      *" $_lc_k "*)
        eval "$_lc_k=\$_lc_v"
        if [ "$_lc_k" = ROLE ]; then
          _lc_role=1
        fi ;;
    esac
  done <<EOF
$_lc_dump
EOF
  if [ "$_lc_role" = 0 ]; then
    log "CONFIG ROLE not set in $CONF: using the default ROLE=tractor (reboot rungs gated off unless TRACTOR_REBOOT_ENABLED=1); the base station needs ROLE=base"
    CONF_STATE="loaded(with-defaults)"
  fi
  validate_config
}

# cfg_num KEY DEFAULT MIN [MAX]
cfg_num() {
  _cn=
  eval "_cn=\${$1}"
  if to_int "$_cn" && [ "$INT" -ge "$3" ] && { [ -z "${4:-}" ] || [ "$INT" -le "$4" ]; }; then
    eval "$1=\$INT"
    return 0
  fi
  if [ -n "${4:-}" ]; then
    log "CONFIG $1='$_cn' invalid (want a whole number $3..$4); using default $2"
  else
    log "CONFIG $1='$_cn' invalid (want a whole number >= $3); using default $2"
  fi
  CONF_STATE="loaded(with-defaults)"
  eval "$1=\$2"
}

# cfg_bool KEY DEFAULT
cfg_bool() {
  _cb=
  eval "_cb=\${$1}"
  case $_cb in
    0|1) return 0 ;;
  esac
  log "CONFIG $1='$_cb' invalid (want 0 or 1); using default $2"
  CONF_STATE="loaded(with-defaults)"
  eval "$1=\$2"
}

validate_config() {
  # Upper bounds keep the unit's WatchdogSec=300 satisfiable: the longest
  # stretch without a WATCHDOG=1 is one sleep plus the checks (120 + ~100 s
  # with three allowlisted containers, + 20 s when a dropped pending reboot
  # rewrites last_action), or the reboot rung on its own after its own ping
  # (three fw_printenv, five synced records, `systemctl reboot`, the forced
  # path with 2 x SYSRQ_SETTLE_S, or a 10 s sync without sysrq, and
  # `reboot -ff`: ~210 s), not counting file
  # writes on wedged storage (see the header). A tractor adds up to two
  # TRACTOR_IDLE_WAIT_S probes before the rung pings again, and one more
  # inside the forced path.
  cfg_num CHECK_INTERVAL_S 30 1 120
  cfg_num FAIL_THRESHOLD 3 1
  cfg_bool CHECK_DOCKER 1
  cfg_num CONTAINER_RESTART_MAX 3 0
  cfg_num REBOOT_MAX_CONSECUTIVE 3 0
  cfg_num HEALTHY_RESET_S 1800 0
  cfg_num REBOOT_GRACE_S 600 0
  cfg_num FORCE_REBOOT_AFTER_S 180 1
  cfg_num SYSRQ_SETTLE_S 3 0 10
  cfg_bool TRACTOR_REBOOT_ENABLED 0
  cfg_num TRACTOR_IDLE_WAIT_S 10 2 15
  case $ROLE in
    base|tractor) ;;
    *)
      # Deliberately the tractor default: a typo on a tractor must not turn
      # its reboot rungs on.
      log "CONFIG ROLE='$ROLE' invalid (want base or tractor); treating as ROLE=tractor (reboots gated off) as the safe fallback"
      CONF_STATE="loaded(with-defaults)"
      ROLE=tractor
      ;;
  esac
  _vc=
  for _vc_c in $CONTAINERS; do
    case $_vc_c in
      [!A-Za-z0-9]*|*[!A-Za-z0-9_.-]*)
        log "CONFIG CONTAINERS: dropping invalid container name '$_vc_c'"
        continue ;;
      *[!0-9A-Fa-f]*) ;;
      *)
        # Docker looks a CONTAINER argument up as full ID, then name, then
        # unique ID prefix: an absent hex-only name could inspect and restart
        # some other container (never-do: nothing outside the allowlist).
        log "CONFIG CONTAINERS: dropping '$_vc_c': hex digits only, docker could resolve it as another container's ID prefix"
        continue ;;
    esac
    case " $_vc " in
      *" $_vc_c "*)
        # Listed twice it would be inspected twice per pass and restarted
        # after half the FAIL_THRESHOLD passes.
        log "CONFIG CONTAINERS: ignoring duplicate '$_vc_c'"
        continue ;;
    esac
    _vc="$_vc $_vc_c"
  done
  CONTAINERS=${_vc# }
  _vc=
  for _vc_h in $HEARTBEATS; do
    _vc_p=${_vc_h%:*}
    _vc_a=${_vc_h##*:}
    case $_vc_p in
      /?*)
        if [ "$_vc_p" != "$_vc_h" ] && to_int "$_vc_a" && [ "$INT" -ge 1 ]; then
          _vc_dup=0
          for _vc_e in $_vc; do
            if [ "${_vc_e%:*}" = "$_vc_p" ]; then
              _vc_dup=1
            fi
          done
          if [ "$_vc_dup" = 1 ]; then
            log "CONFIG HEARTBEATS: ignoring duplicate path '$_vc_p' ('$_vc_h')"
          else
            _vc="$_vc $_vc_p:$INT"
          fi
          continue
        fi ;;
    esac
    log "CONFIG HEARTBEATS: dropping invalid entry '$_vc_h' (want /abs/path:max_age_s)"
  done
  HEARTBEATS=${_vc# }
}

# -------------------------------------------------------------------- state
init_state() {
  case $STATE in
    /?*) ;;
    *) printf '%s\n' "lifetrac-supervisor: STATE '$STATE' must be an absolute path" >&2; exit 2 ;;
  esac
  STATE_OK=0
  if mkdir -p "$STATE" 2>/dev/null && [ -d "$STATE" ] && [ -w "$STATE" ]; then
    STATE_OK=1
  fi
  rotate_log
}

# Runs at every start. A new boot id means the previous boot ended: classify
# how, then reset the per-boot state.
boot_init() {
  [ "$STATE_OK" = 1 ] || return 0
  kv_get "$STATE/boot_id" ""
  _bi_prev=$KV
  _bi_newprev=
  if [ "$_bi_prev" != "$BOOT_ID" ]; then
    _bi_why=external_or_unknown
    if [ -n "$_bi_prev" ] && [ -f "$STATE/last_action" ]; then
      _bi_boot=''
      _bi_act=''
      _bi_rsn=''
      while IFS= read -r _bi_l; do
        case $_bi_l in
          boot_id=*) _bi_boot=${_bi_l#boot_id=} ;;
          action=*) _bi_act=${_bi_l#action=} ;;
          reason=*) _bi_rsn=${_bi_l#reason=} ;;
        esac
      done < "$STATE/last_action"
      if [ "$_bi_boot" = "$_bi_prev" ]; then
        _bi_why="supervisor:$_bi_act:$_bi_rsn"
      fi
    fi
    write_record "$STATE/last_reset_reason" "$_bi_why"
    write_record "$STATE/boot_id" "$BOOT_ID"
    log "BOOT new boot (previous=${_bi_prev:-none}) last_reset_reason=$_bi_why"
    _bi_newprev=${_bi_prev:-none}
  fi
  kv_get "$RUN/id" ""
  if [ "$KV" != "$BOOT_ID" ]; then
    rm -rf "$RUN" 2>/dev/null
    mkdir -p "$RUN" 2>/dev/null && kv_put "$RUN/id" "$BOOT_ID"
  fi
  [ -d "$RUN" ] || STATE_OK=0
  # The OTA probation check compares u-boot's bootcount with the one recorded
  # at a supervisor reboot in the boot just before this one.
  if [ -n "$_bi_newprev" ]; then
    kv_put "$RUN/prev_boot_id" "$_bi_newprev"
  fi
}

startup() {
  read_boot_id
  read_uptime || UP=
  init_state
  load_config
  boot_init
  if [ "$STATE_OK" != 1 ]; then
    log "ERROR state dir $STATE unusable: running DISARMED (the boot-loop guard cannot persist)"
  fi
  # A request with no pending reboot behind it (this process died between
  # writing it and systemd taking the reboot, or the state was lost) must not
  # keep the tractor's motion hold on, unless a reset was issued in this boot
  # (withdraw_lock keeps it then).
  if [ -e "$REQ_FILE" ] && [ ! -f "$RUN/reboot_pending" ]; then
    withdraw_lock "left from an earlier run with no pending reboot"
  fi
}

# ------------------------------------------------------------------- checks
add_fault() { FAULTS="${FAULTS:+$FAULTS,}$1"; }
add_decision() { DECISION="${DECISION:+$DECISION }$1"; }

check_docker() {
  DOCKER_OK=1
  [ "$CHECK_DOCKER" = 1 ] || return 0
  timeout 20 docker info >/dev/null 2>&1
  _cd_rc=$?
  kv_int "$RUN/docker_streak" 0
  _cd_s=$INT
  if [ "$_cd_rc" -eq 0 ]; then
    if [ "$_cd_s" -gt 0 ]; then
      log "CHECK docker OK again after $_cd_s failing pass(es)"
      kv_put "$RUN/docker_streak" 0
    fi
    return 0
  fi
  DOCKER_OK=0
  ALL_OK=0
  _cd_s=$((_cd_s + 1))
  kv_put "$RUN/docker_streak" "$_cd_s"
  rc_note "$_cd_rc"
  log "CHECK docker FAIL rc=$_cd_rc$RCNOTE streak=$_cd_s"
  add_fault docker_down
}

check_containers() {
  [ -n "$CONTAINERS" ] || return 0
  if [ "$DOCKER_OK" = 0 ]; then
    # Counted as docker_down above; inspecting or restarting now would only
    # stack 20 s timeouts and burn restart budget on a daemon fault.
    ALL_OK=0
    log "CHECK containers skipped: docker down"
    return 0
  fi
  for _cc in $CONTAINERS; do
    check_container "$_cc"
  done
}

check_container() {
  _ct=$1
  _ct_out=$(timeout 20 docker inspect --type container -f "$INSPECT_FMT" "$_ct" 2>&1)
  _ct_rc=$?
  _ct_line=${_ct_out##*"$NL"}   # the -f line is last; a stderr warning may precede it
  _ct_nm=${_ct_line%% *}
  _ct_rest=${_ct_line#* }
  _ct_st=${_ct_rest%% *}
  _ct_hl=${_ct_rest#* }
  # Docker answered for a container whose name is not ours (an ID-prefix
  # match): report it, never restart it.
  _ct_foreign=0
  if [ "$_ct_rc" -eq 0 ] && [ "$_ct_nm" != "/$_ct" ]; then
    _ct_foreign=1
  fi
  if [ "$_ct_rc" -eq 0 ] && [ "$_ct_foreign" = 0 ] && [ "$_ct_st" = running ] && [ "$_ct_hl" != unhealthy ]; then
    kv_int "$RUN/c.$_ct.streak" 0
    if [ "$INT" -gt 0 ]; then
      log "CHECK container=$_ct OK again ($_ct_st/$_ct_hl) after $INT failing pass(es)"
      kv_put "$RUN/c.$_ct.streak" 0
    fi
    return 0
  fi
  ALL_OK=0
  if [ "$_ct_foreign" = 1 ]; then
    _ct_why="resolves_to=$_ct_nm"
  elif [ "$_ct_rc" -eq 0 ]; then
    _ct_why="state=$_ct_st/health=$_ct_hl"
  else
    case $_ct_out in
      *"No such"*) _ct_why=missing ;;
      *) rc_note "$_ct_rc"; _ct_why="inspect_failed_rc=$_ct_rc$RCNOTE" ;;
    esac
  fi
  kv_int "$RUN/c.$_ct.streak" 0
  _ct_s=$((INT + 1))
  kv_put "$RUN/c.$_ct.streak" "$_ct_s"
  # Disarmed runs count their would-be restarts separately, so the dry run
  # walks the whole ladder but arming mid-boot starts from a full budget.
  if [ "$ARMED" = 1 ]; then
    _ct_cf=$RUN/c.$_ct.restarts
  else
    _ct_cf=$RUN/c.$_ct.would_restarts
  fi
  kv_int "$_ct_cf" 0
  _ct_n=$INT
  if [ "$_ct_n" -ge "$CONTAINER_RESTART_MAX" ]; then
    log "CHECK container=$_ct FAIL $_ct_why: restarts used up ($_ct_n/$CONTAINER_RESTART_MAX this boot) -> node-level fault"
    add_fault "container_exhausted:$_ct"
    return 0
  fi
  if [ "$_ct_s" -lt "$FAIL_THRESHOLD" ]; then
    log "CHECK container=$_ct FAIL $_ct_why streak=$_ct_s/$FAIL_THRESHOLD"
    return 0
  fi
  _ct_n=$((_ct_n + 1))
  # No restart that the per-boot budget cannot record (2026-10-17 review): on
  # full or read-only storage the old count would stay on disk and every
  # later pass would restart again, past CONTAINER_RESTART_MAX. Treat it like
  # an exhausted budget; the reboot rung refuses too while it cannot persist
  # its own counter. A disarmed run only logs, so it carries on (an unusable
  # state dir already forces DISARMED).
  if ! kv_put "$_ct_cf" "$_ct_n" && [ "$ARMED" = 1 ]; then
    log "REFUSED restart container=$_ct: cannot record restart $_ct_n/$CONTAINER_RESTART_MAX in $_ct_cf (state not writable); nothing restarted -> node-level fault"
    add_decision "restart_unrecorded:$_ct"
    add_fault "container_unrecorded:$_ct"
    return 0
  fi
  kv_put "$RUN/c.$_ct.streak" 0
  if [ "$_ct_foreign" = 1 ]; then
    # Counted like a missing container, so the ladder ends the same way.
    log "REFUSED restart container=$_ct: docker resolves the name to '$_ct_nm', not the allowlisted container; nothing restarted (restart=$_ct_n/$CONTAINER_RESTART_MAX counted)"
    add_decision "restart_refused:$_ct"
    return 0
  fi
  if [ "$ARMED" != 1 ]; then
    log "WOULD restart container=$_ct reason=$_ct_why streak=$_ct_s restart=$_ct_n/$CONTAINER_RESTART_MAX (disarmed: $ARMED_FILE absent)"
    add_decision "would_restart:$_ct"
    return 0
  fi
  log "ACTION restart container=$_ct reason=$_ct_why streak=$_ct_s restart=$_ct_n/$CONTAINER_RESTART_MAX"
  timeout 60 docker restart "$_ct" >/dev/null 2>&1
  _ct_rc=$?
  rc_note "$_ct_rc"
  log "ACTION restart container=$_ct done rc=$_ct_rc$RCNOTE"
  add_decision "restart:$_ct"
  # A restart can take most of a minute; keep the unit watchdog fed (each
  # step is bounded, so this cannot mask a wedged supervisor).
  notify_watchdog
}

check_heartbeats() {
  [ -n "$HEARTBEATS" ] || return 0
  _hb_f=$RUN/heartbeats
  _hb_new=
  for _hb in $HEARTBEATS; do
    _hb_p=${_hb%:*}
    _hb_max=${_hb##*:}
    _hb_m=$(timeout 5 stat -c %Y "$_hb_p" 2>/dev/null) || _hb_m=
    [ -n "$_hb_m" ] || _hb_m=missing
    _hb_pm=
    _hb_pc=
    if [ -f "$_hb_f" ]; then
      while read -r _hb_rp _hb_rm _hb_rc; do
        if [ "$_hb_rp" = "$_hb_p" ]; then
          _hb_pm=$_hb_rm
          _hb_pc=$_hb_rc
          break
        fi
      done < "$_hb_f"
    fi
    # Remember the mtime VALUE and the uptime at which it last changed. A
    # first sighting (or a disappearing file) starts the clock now.
    if ! to_int "$_hb_pc"; then
      _hb_pc=$UP
    elif [ "$_hb_m" != "$_hb_pm" ]; then
      _hb_pc=$UP
    else
      _hb_pc=$INT
    fi
    _hb_new="$_hb_new$_hb_p $_hb_m $_hb_pc$NL"
    _hb_age=$((UP - _hb_pc))
    if [ "$_hb_age" -gt "$_hb_max" ]; then
      ALL_OK=0
      log "CHECK heartbeat=$_hb_p STALE: mtime=$_hb_m unchanged for ${_hb_age}s of uptime (max ${_hb_max}s)"
      add_fault "heartbeat_stale:$_hb_p"
    fi
  done
  if printf '%s' "$_hb_new" 2>/dev/null > "$_hb_f.tmp"; then
    mv -f "$_hb_f.tmp" "$_hb_f" 2>/dev/null
  fi
}

# -------------------------------------------------------------- reboot rung
# compute_cap: set CAP, and PROBATION=1 (with PROBATION_BC) when an OTA trial
# boot's rollback is not due yet, so HELD and the cap must not stop a reboot.
compute_cap() {
  CAP=$REBOOT_MAX_CONSECUTIVE
  PROBATION=0
  PROBATION_BC=
  command -v fw_printenv >/dev/null 2>&1 || return 0
  _cp_ua=$(timeout 10 fw_printenv -n upgrade_available 2>/dev/null) || _cp_ua=0
  [ "$_cp_ua" = 1 ] || return 0
  _cp_bl=$(timeout 10 fw_printenv -n bootlimit 2>/dev/null) || _cp_bl=
  if to_int "$_cp_bl"; then
    _cp_bl=$INT
  else
    _cp_bl=5
  fi
  if [ $((_cp_bl + 1)) -gt "$CAP" ]; then
    CAP=$((_cp_bl + 1))
  fi
  _cp_bc=$(timeout 10 fw_printenv -n bootcount 2>/dev/null) || _cp_bc=
  if ! to_int "$_cp_bc"; then
    _cp_note="bootcount unreadable: the cap applies, HELD is not overridden"
  elif [ "$INT" -gt "$_cp_bl" ]; then
    _cp_note="bootcount=$INT > bootlimit: rollback already due, the cap applies"
  else
    _cp_bc=$INT
    # Self-check against a stale env copy: after a probation reboot by the
    # supervisor in the previous boot, u-boot must have counted that boot.
    kv_get "$RUN/prev_boot_id" ""
    _cp_prev=$KV
    _cp_rb=
    _cp_rn=
    if [ -n "$_cp_prev" ] && [ -f "$STATE/probation_reboot" ]; then
      while IFS= read -r _cp_l; do
        case $_cp_l in
          boot_id=*) _cp_rb=${_cp_l#boot_id=} ;;
          bootcount=*) _cp_rn=${_cp_l#bootcount=} ;;
        esac
      done < "$STATE/probation_reboot"
    fi
    if [ "$_cp_rb" = "$_cp_prev" ] && [ -n "$_cp_rb" ] && to_int "$_cp_rn" && [ "$_cp_bc" -le "$INT" ]; then
      _cp_note="bootcount=$_cp_bc did not rise after the supervisor's probation reboot in the previous boot (was $INT): u-boot env copy not live? the cap applies, HELD is not overridden"
    else
      PROBATION=1
      PROBATION_BC=$_cp_bc
      _cp_note="bootcount=$_cp_bc <= bootlimit: rollback not due yet, so neither HELD nor the cap stops this reboot"
    fi
  fi
  log "PROBATION upgrade_available=1 (OTA trial boot): bootlimit=$_cp_bl, consecutive-reboot cap = max($REBOOT_MAX_CONSECUTIVE, $_cp_bl+1) = $CAP so u-boot's bootcount rollback can fire first; $_cp_note"
}

# ------------------------------------------------- tractor idle gate (D-PWR1)
# idle_read: set IDLE_ST, IDLE_SEQ and IDLE_RID from the first line of
# $IDLE_FILE; fail if it is missing, empty, not a regular file or a symlink
# (the link service's directory is not root's), or the read does not finish
# in 2 s. At most 256 bytes are read. The open itself refuses a symlink and
# does not block (O_NOFOLLOW, O_NONBLOCK), so one swapped in after the
# checks is not followed and a FIFO cannot hang the probe. IDLE_ST is busy, idle, locked or
# "invalid". A <seq> that is not 1-32 letters or digits reads as empty, which
# never counts as a change; an <id> that is not 1-40 letters, digits or "-"
# reads as empty, which never matches a request. Nothing else from the file
# reaches the log or the status file.
idle_read() {
  IDLE_ST=
  IDLE_SEQ=
  IDLE_RID=
  if [ ! -f "$IDLE_FILE" ] || [ -L "$IDLE_FILE" ]; then
    return 1
  fi
  _ir=$(timeout 2 dd if="$IDLE_FILE" bs=256 count=1 iflag=nofollow,nonblock status=none 2>/dev/null) || return 1
  _ir=${_ir%%"$NL"*}
  _ir=${_ir%"$CR"}
  [ -n "$_ir" ] || return 1
  _ir_ifs=$IFS
  IFS=" $TAB"
  # shellcheck disable=SC2086
  set -- $_ir
  IFS=$_ir_ifs
  case ${1:-} in
    busy|idle|locked) IDLE_ST=$1 ;;
    *) IDLE_ST=invalid ;;
  esac
  IDLE_SEQ=${2:-}
  case $IDLE_SEQ in
    ''|*[!0-9A-Za-z]*) IDLE_SEQ= ;;
  esac
  if [ ${#IDLE_SEQ} -gt 32 ]; then
    IDLE_SEQ=
  fi
  IDLE_RID=${3:-}
  case $IDLE_RID in
    ''|*[!0-9A-Za-z-]*) IDLE_RID= ;;
  esac
  if [ ${#IDLE_RID} -gt 40 ]; then
    IDLE_RID=
  fi
  return 0
}

# idle_seen WANT: set IDLE_SEEN to what the last read showed, for logs:
# IDLE_ST, or "locked:other_id" for a hold on a request other than WANT.
idle_seen() {
  IDLE_SEEN=${IDLE_ST:-none}
  if [ "$IDLE_ST" = locked ] && [ "$1" != idle ] && [ "$IDLE_RID" != "$1" ]; then
    IDLE_SEEN=locked:other_id
  fi
}

# idle_probe WANT: poll $IDLE_FILE every $IDLE_POLL for up to
# TRACTOR_IDLE_WAIT_S polls. WANT "idle" passes on a live idle or locked (any
# request); any other WANT is a request id and passes only on a live
# "locked <seq> WANT". Live = <seq> differs from the first read, so a file
# left behind by a dead link service never passes. Sets IDLE_LIVE (1 once
# <seq> changed) and IDLE_WHY; IDLE_ST and IDLE_RID keep the last read.
idle_probe() {
  IDLE_LIVE=0
  if ! idle_read; then
    IDLE_WHY="missing:$IDLE_FILE"
    return 1
  fi
  _ip_seq0=$IDLE_SEQ
  _ip_n=0
  while [ "$_ip_n" -lt "$TRACTOR_IDLE_WAIT_S" ]; do
    _ip_n=$((_ip_n + 1))
    sleep "$IDLE_POLL"
    if ! idle_read; then
      IDLE_WHY="missing:$IDLE_FILE"
      return 1
    fi
    if [ "$IDLE_LIVE" = 0 ]; then
      if [ -z "$IDLE_SEQ" ] || [ "$IDLE_SEQ" = "$_ip_seq0" ]; then
        continue
      fi
      IDLE_LIVE=1
    fi
    if [ "$1" = idle ]; then
      case $IDLE_ST in
        idle|locked) IDLE_WHY="state=$IDLE_ST"; return 0 ;;
        *) IDLE_WHY="state=$IDLE_ST"; return 1 ;;
      esac
    fi
    case $IDLE_ST in
      locked)
        if [ "$IDLE_RID" = "$1" ]; then
          IDLE_WHY="locked=$1"
          return 0
        fi ;;
      idle) ;;
      *) IDLE_WHY="state=$IDLE_ST"; return 1 ;;
    esac
  done
  idle_seen "$1"
  if [ "$IDLE_LIVE" = 0 ]; then
    IDLE_WHY="not_live:seq_unchanged_for_${TRACTOR_IDLE_WAIT_S}_polls:last=$IDLE_SEEN"
  else
    IDLE_WHY="no_lock:state=$IDLE_SEEN"
  fi
  return 1
}

# tractor_lock ID: ask the H747 to hold motion off for reboot request ID and
# wait for it. On failure the request is withdrawn and the reboot blocked.
# The request's directory is root-only and made here (never by the link
# service, never RuntimeDirectory=), so no other user can plant or swap a
# name in it.
tractor_lock() {
  # Only a person removes a request while a reset is recorded (withdraw_lock
  # keeps it, the link service only reads it), and the documented release
  # cancels the queued reboot first: that reset no longer counts.
  if [ ! -e "$REQ_FILE" ] && reset_issued; then
    log "LOCK released by hand: $REQ_FILE was removed after a reset was issued in this boot, so that reset no longer keeps a request"
    clear_reset_issued
  fi
  if [ ! -d "${REQ_FILE%/*}" ]; then
    mkdir -m 0755 "${REQ_FILE%/*}" 2>/dev/null
  fi
  if ! kv_put "$REQ_FILE" "$1"; then
    log "REFUSED reboot: cannot write the motion-hold request $REQ_FILE reason=$_rr"
    add_decision reboot_refused:lock_request
    withdraw_lock "request not writable"
    return 1
  fi
  log "LOCK requested id=$1: waiting up to $TRACTOR_IDLE_WAIT_S polls for the H747 to hold motion off"
  if idle_probe "$1"; then
    log "LOCK held id=$1 ($IDLE_WHY)"
    return 0
  fi
  withdraw_lock "not granted: $IDLE_WHY"
  log "BLOCKED reboot gate=tractor_lock ($IDLE_WHY): the H747 did not confirm a motion hold for request $1 reason=$_rr"
  add_decision reboot_blocked:tractor_lock
  BLOCKED=tractor_lock
  return 1
}

# lock_still_held ID: one more look right before the reset is issued (the
# records written after tractor_lock can take a minute): the request on disk
# is still ID and the idle file's first line is still "locked <seq> ID". One
# read, no liveness wait: tractor_lock saw the link live moments ago, and a
# link stopped since then leaves the H747 holding.
lock_still_held() {
  kv_get "$REQ_FILE" ""
  if [ "$KV" != "$1" ]; then
    IDLE_WHY="request_gone_or_changed"
    return 1
  fi
  if ! idle_read; then
    IDLE_WHY="missing:$IDLE_FILE"
    return 1
  fi
  if [ "$IDLE_ST" != locked ] || [ "$IDLE_RID" != "$1" ]; then
    idle_seen "$1"
    IDLE_WHY="no_lock:state=$IDLE_SEEN"
    return 1
  fi
  return 0
}

# reset_issued: did this boot issue a tractor reset (an accepted, timed-out
# or signal-killed `systemctl reboot`, or the forced path)? Then the request stays until the
# reset. $RESET_FILE is on tmpfs, so it goes with the reboot; one naming
# another boot (only where it is not tmpfs, e.g. the SIL tests) does not
# count. An unreadable one does.
reset_issued() {
  [ "$RESET_ISSUED" = 1 ] && return 0
  [ -e "$RESET_FILE" ] || return 1
  kv_get "$RESET_FILE" ""
  _ri=${KV%% *}
  if [ -n "$_ri" ] && [ "$_ri" != "$BOOT_ID" ]; then
    return 1
  fi
  return 0
}

# mark_reset_issued ID: record that a reset for request ID is about to be
# issued in this boot. Fails if the record cannot be written.
mark_reset_issued() {
  RESET_ISSUED=1
  kv_put "$RESET_FILE" "$BOOT_ID $UP $1"
}

# clear_reset_issued: the reset this rung was about to issue was not issued
# (refused, or `systemctl reboot` failed outright, not a timeout or a
# signal, so systemd queued nothing), and none was issued earlier in this boot.
clear_reset_issued() {
  RESET_ISSUED=0
  rm -f "$RESET_FILE" 2>/dev/null
  if [ -e "$RESET_FILE" ]; then
    RESET_ISSUED=1
    log "ERROR cannot remove $RESET_FILE: the motion-hold request stays until the reset or a person removes $REQ_FILE"
  fi
}

# withdraw_lock WHY: delete the request, so the link service tells the H747
# to release its motion hold. Called on every path where the reboot does not
# happen; a no-op once a reset has been issued in this boot (reset_issued):
# systemd may still carry the reboot out, so the hold stays until it does.
withdraw_lock() {
  [ -e "$REQ_FILE" ] || return 0
  if reset_issued; then
    log "LOCK kept ($1): a reset was issued in this boot ($RESET_FILE), so $REQ_FILE stays and the H747 keeps holding motion off until the reset; to release it by hand, systemctl cancel the queued reboot, then remove $REQ_FILE"
    return 0
  fi
  rm -f "$REQ_FILE" 2>/dev/null
  if [ -e "$REQ_FILE" ]; then
    log "ERROR cannot remove $REQ_FILE ($1): the H747 keeps holding motion off until it resets or the file is removed"
  else
    log "LOCK withdrawn ($1): $REQ_FILE removed, so the H747 can release its motion hold"
  fi
  return 0
}

# tractor_force_ok: may the forced path reset a tractor now? Sets TF_ID to the
# request on disk. Yes if the hold for it is live, or if the link service is
# not live (the shutdown stopped it) and its last line shows that hold (the
# H747 keeps it until it resets). No if the request is gone, the idle file is
# missing, the live link does not show the hold, or the frozen last line is
# anything else.
tractor_force_ok() {
  kv_get "$REQ_FILE" ""
  TF_ID=$KV
  if [ -z "$TF_ID" ]; then
    log "BLOCKED forced reboot gate=tractor_lock: no motion-hold request at $REQ_FILE, so the H747 may not be holding motion off"
    return 1
  fi
  if idle_probe "$TF_ID"; then
    log "LOCK still held id=$TF_ID ($IDLE_WHY): forcing"
    return 0
  fi
  if [ "$IDLE_LIVE" = 1 ]; then
    log "BLOCKED forced reboot gate=tractor_lock: the H747 link is live but not holding request $TF_ID ($IDLE_WHY)"
    return 1
  fi
  if [ "$IDLE_ST" = locked ] && [ "$IDLE_RID" = "$TF_ID" ]; then
    log "LOCK id=$TF_ID: the H747 link is not live ($IDLE_WHY; stopped by the shutdown?) and its last report shows the hold, which the H747 keeps until it resets, so forcing"
    return 0
  fi
  log "BLOCKED forced reboot gate=tractor_lock: the H747 link is not live ($IDLE_WHY) and its last report does not show a hold for request $TF_ID"
  return 1
}

write_last_action() {  # ACTION REASON CONSECUTIVE [DETAIL]
  write_record "$STATE/last_action" "action=$1
reason=$2
boot_id=$BOOT_ID
uptime_s=$UP
consecutive=$3
role=$ROLE
detail=${4:-none}"
}

# uncount_reboot: a tractor rung stopped after persisting consecutive_reboots
# but before issuing the reset (the hold went away at the last look, or the
# reset record could not be written). Like the other tractor gates, that
# attempt is not a reboot and must not move the boot-loop cap.
uncount_reboot() {
  write_record "$STATE/consecutive_reboots" "$((_rr_n - 1))"
}

reboot_rung() {
  _rr=$1
  if [ "$UP" -lt "$REBOOT_GRACE_S" ]; then
    log "BLOCKED reboot gate=grace up=${UP}s<${REBOOT_GRACE_S}s reason=$_rr"
    add_decision reboot_blocked:grace
    BLOCKED=grace
    return 0
  fi
  if [ "$ROLE" = tractor ] && [ "$TRACTOR_REBOOT_ENABLED" != 1 ]; then
    log "BLOCKED reboot gate=tractor_reboot_disabled (ROLE=tractor, TRACTOR_REBOOT_ENABLED=0: an X8 reboot resets the H747 and, in production, trips the Opta watchdog and the engine-kill latch) reason=$_rr"
    add_decision reboot_blocked:tractor_reboot_disabled
    BLOCKED=tractor_reboot_disabled
    return 0
  fi
  compute_cap
  kv_int "$STATE/consecutive_reboots" 0
  _rr_n=$INT
  if [ "$HELD" = 1 ]; then
    if [ "$PROBATION" != 1 ]; then
      log "HELD reboot suppressed reason=$_rr"
      add_decision reboot_suppressed:held
      return 0
    fi
    log "PROBATION overrides HELD (bootcount=$PROBATION_BC): the held file stays and still needs clear-hold after the trial reason=$_rr"
    add_decision held_overridden:probation
  fi
  if [ "$_rr_n" -ge "$CAP" ]; then
    if [ "$PROBATION" = 1 ]; then
      log "PROBATION overrides cap: consecutive_reboots=$_rr_n >= cap=$CAP but u-boot's rollback is not due yet (bootcount=$PROBATION_BC) reason=$_rr"
      add_decision cap_overridden:probation
    elif [ "$ARMED" != 1 ]; then
      log "WOULD hold (disarmed): consecutive_reboots=$_rr_n reached cap=$CAP reason=$_rr"
      add_decision would_hold
      kv_put "$RUN/node_streak" 0
      return 0
    else
      write_record "$STATE/held" "reason=$_rr
consecutive=$_rr_n
cap=$CAP
boot_id=$BOOT_ID
uptime_s=$UP"
      HELD=1
      log "HELD entering hold: consecutive_reboots=$_rr_n reached cap=$CAP; no more supervisor reboots until '$0 clear-hold' reason=$_rr"
      add_decision held
      return 0
    fi
  fi
  _rr_n=$((_rr_n + 1))
  # D-PWR1: a tractor reboots only while the H747 reports idle. The node
  # streak is kept, so the next pass with the fault asks again.
  if [ "$ROLE" = tractor ] && ! idle_probe idle; then
    log "BLOCKED reboot gate=tractor_not_idle ($IDLE_WHY): a tractor reboots only while the H747 reports the engine stopped and every valve output off (D-PWR1) reason=$_rr"
    add_decision reboot_blocked:tractor_not_idle
    BLOCKED=tractor_not_idle
    return 0
  fi
  if [ "$ARMED" != 1 ]; then
    if [ "$ROLE" = tractor ]; then
      log "WOULD reboot reason=$_rr consecutive=$_rr_n/$CAP (disarmed: $ARMED_FILE absent; the H747 reports idle, so the next step would be the motion-hold request)"
    else
      log "WOULD reboot reason=$_rr consecutive=$_rr_n/$CAP (disarmed: $ARMED_FILE absent)"
    fi
    add_decision would_reboot
    kv_put "$RUN/node_streak" 0
    return 0
  fi
  _rr_id=$BOOT8-$UP
  if [ "$ROLE" = tractor ]; then
    tractor_lock "$_rr_id" || return 0
    # The probes took up to 2 x TRACTOR_IDLE_WAIT_S; the rest of the rung
    # starts with a fresh unit-watchdog budget (each step is bounded).
    notify_watchdog
  fi
  if ! write_record "$STATE/consecutive_reboots" "$_rr_n"; then
    log "ERROR cannot persist consecutive_reboots: refusing to reboot (boot-loop guard would be blind)"
    add_decision reboot_refused:state
    withdraw_lock "reboot refused"
    return 0
  fi
  if [ "$PROBATION" = 1 ]; then
    # Evidence for compute_cap's self-check in the next boot. If it cannot be
    # written the next boot has no evidence and trusts u-boot's bootcount.
    write_record "$STATE/probation_reboot" "boot_id=$BOOT_ID
bootcount=$PROBATION_BC"
  fi
  log "ACTION reboot reason=$_rr consecutive=$_rr_n/$CAP"
  if ! write_last_action reboot "$_rr" "$_rr_n"; then
    log "ERROR cannot write last_action: refusing to reboot without a reset-reason record"
    add_decision reboot_refused:state
    withdraw_lock "reboot refused"
    return 0
  fi
  _rr_was=0
  if [ "$ROLE" = tractor ]; then
    # A reset issued earlier in this boot (abandoned since) can still land,
    # so its record is never cleared below.
    if reset_issued; then
      _rr_was=1
    fi
    # Recorded before systemd can take the reboot: from here on no path in
    # this boot, a restarted supervisor included, withdraws the request.
    if ! mark_reset_issued "$_rr_id"; then
      if [ "$_rr_was" != 1 ]; then
        clear_reset_issued
      fi
      log "REFUSED reboot: cannot write $RESET_FILE, so a later pass or restart could release the motion hold while the reboot is queued (not counted) reason=$_rr"
      add_decision reboot_refused:reset_marker
      uncount_reboot
      abandon_reboot "$_rr" reboot_refused:reset_marker
      return 0
    fi
    if ! lock_still_held "$_rr_id"; then
      if [ "$_rr_was" != 1 ]; then
        clear_reset_issued
      fi
      log "BLOCKED reboot gate=tractor_lock ($IDLE_WHY): the motion hold for request $_rr_id is no longer shown right before the reboot (not counted) reason=$_rr"
      add_decision reboot_blocked:tractor_lock_lost
      BLOCKED=tractor_lock
      uncount_reboot
      abandon_reboot "$_rr" reboot_blocked:tractor_lock_lost
      return 0
    fi
  fi
  add_decision reboot
  timeout 30 systemctl reboot >/dev/null 2>&1
  _rr_rc=$?
  if [ "$_rr_rc" -eq 0 ]; then
    # The forced-path clock starts now, when systemd took the reboot, not at
    # the pass-start $UP (2026-10-17 review): the syncs, fw_printenv calls and
    # `systemctl reboot` above can take minutes. Keep the old value if the
    # uptime file cannot be read.
    _rr_up=$UP
    read_uptime || UP=$_rr_up
    if ! kv_put "$RUN/reboot_pending" "$UP $PROBATION $_rr"; then
      log "ERROR cannot write $RUN/reboot_pending: no forced path for this reboot"
    fi
    log "ACTION reboot: systemctl reboot accepted; forced path if still alive after ${FORCE_REBOOT_AFTER_S}s"
    return 0
  fi
  # rc 124: `systemctl reboot` timed out; 128 and above: it was killed by a
  # signal. Either way systemd may still have queued the reboot, so a
  # tractor keeps its reset record. Any other failure (systemctl's own
  # error, or it never ran) queued nothing.
  if [ "$ROLE" = tractor ] && [ "$_rr_rc" -ne 124 ] && [ "$_rr_rc" -lt 128 ] && [ "$_rr_was" != 1 ]; then
    clear_reset_issued
  fi
  rc_note "$_rr_rc"
  force_reboot "$_rr" "systemctl_reboot_rc_$_rr_rc"
}

# Still alive after an accepted `systemctl reboot` (review finding,
# 2026-10-17: the forced path used to check ARMED only, never dropped its
# marker when disarmed, and stopped every health check for the rest of the
# boot). Returns 0 when this pass should stop here, inside the
# FORCE_REBOOT_AFTER_S window. Otherwise returns 1 so the checks run, with
# PENDING_DUE=1 when the forced path is due; pass() forces only if a node
# fault is still present.
check_pending_reboot() {
  PENDING_DUE=0
  [ -f "$RUN/reboot_pending" ] || return 1
  kv_get "$RUN/reboot_pending" ""
  _pr_up=${KV%% *}
  _pr_rest=${KV#* }
  _pr_ovr=${_pr_rest%% *}
  _pr_rsn=${_pr_rest#* }
  if ! to_int "$_pr_up"; then
    rm -f "$RUN/reboot_pending" 2>/dev/null
    log "PENDING marker unreadable: dropped, not forcing"
    add_decision force_skipped:unreadable
    abandon_reboot unknown force_skipped:pending_marker_unreadable
    return 1
  fi
  _pr_el=$((UP - INT))
  # The same gates as reboot_rung (grace cannot apply within a boot), read
  # again now: the board may have been disarmed or held, or the service
  # restarted with the tractor gate closed, since the reboot was accepted.
  _pr_gate=
  if [ "$ARMED" != 1 ]; then
    _pr_gate=disarmed
  elif [ "$HELD" = 1 ] && [ "$_pr_ovr" != 1 ]; then
    _pr_gate=held
  elif [ "$ROLE" = tractor ] && [ "$TRACTOR_REBOOT_ENABLED" != 1 ]; then
    _pr_gate=tractor_reboot_disabled
    BLOCKED=tractor_reboot_disabled
  fi
  if [ -n "$_pr_gate" ]; then
    rm -f "$RUN/reboot_pending" 2>/dev/null
    log "BLOCKED forced reboot gate=$_pr_gate: systemctl reboot was accepted ${_pr_el}s ago (reason=$_pr_rsn) and will not be forced; pending marker dropped, checks resume"
    add_decision "force_blocked:$_pr_gate"
    abandon_reboot "$_pr_rsn" "force_blocked:$_pr_gate"
    return 1
  fi
  if [ "$_pr_el" -lt "$FORCE_REBOOT_AFTER_S" ]; then
    log "PENDING reboot requested ${_pr_el}s ago reason=$_pr_rsn; waiting for shutdown"
    add_decision reboot_pending
    return 0
  fi
  # One forced attempt per accepted reboot, whatever happens below. If we are
  # somehow still here afterwards, the counted ladder resumes.
  rm -f "$RUN/reboot_pending" 2>/dev/null
  _pr_late=$((FORCE_REBOOT_AFTER_S + 2 * CHECK_INTERVAL_S))
  if [ "$_pr_el" -gt "$_pr_late" ]; then
    log "PENDING marker stale: systemctl reboot accepted ${_pr_el}s ago (> ${_pr_late}s, so the supervisor was not running meanwhile) reason=$_pr_rsn; dropped, not forcing"
    add_decision force_skipped:stale
    abandon_reboot "$_pr_rsn" force_skipped:stale
    return 1
  fi
  PENDING_DUE=1
  PENDING_EL=$_pr_el
  PENDING_RSN=$_pr_rsn
  return 1
}

sysrq_write() {  # KEY [quiet]
  # Append, one character per open: the kernel acts on each write, and a
  # regular test file records the order. quiet: no log line (the caller
  # logs the outcome later).
  if printf '%s' "$1" 2>/dev/null >> "$SYSRQ"; then
    [ "${2:-}" = quiet ] || log "SYSRQ wrote '$1'"
    return 0
  fi
  [ "${2:-}" = quiet ] || log "SYSRQ write '$1' to $SYSRQ failed"
  return 1
}

sysrq_settle() {
  if [ "$SYSRQ_SETTLE_S" -gt 0 ]; then
    sleep "$SYSRQ_SETTLE_S"
  fi
}

force_reboot() {  # REASON WHY
  if [ "$ROLE" = tractor ]; then
    if ! tractor_force_ok; then
      add_decision force_blocked:tractor_lock
      BLOCKED=tractor_lock
      abandon_reboot "$1" "force_blocked:tractor_lock:$2"
      return 0
    fi
    # Was a reset already issued in this boot (an accepted, timed-out or
    # killed `systemctl reboot`)? If not, and this path gives up before its
    # own reset, force_hold_ok clears the record again.
    _fr_was=0
    if reset_issued; then
      _fr_was=1
    fi
    # A reset can land at any moment from here, and the request stays until
    # it does, even if this path returns.
    if ! mark_reset_issued "$TF_ID"; then
      log "ERROR cannot write $RESET_FILE: this process keeps the motion-hold request, a restarted one would not"
    fi
  fi
  kv_int "$STATE/consecutive_reboots" 0
  write_last_action force_reboot "$1" "$INT" "$2"
  # The record's syncs take up to 20 s: look again before anything else.
  force_hold_ok "$1" "$2" "before the forced reset" || return 0
  add_decision force_reboot
  log "ACTION force_reboot reason=$1 why=$2"
  _fr_u=0
  if [ -e "$SYSRQ" ]; then
    sysrq_write s
    sysrq_settle
    # Last look, after the s settle and before u: u remounts every
    # block-device filesystem read-only and cannot be undone, so once it is
    # written this path goes on to b (one settle later) and `reboot -ff`.
    force_hold_ok "$1" "$2" "before sysrq u" || return 0
    # Nothing but the settle from this look to b: u is logged after b, as
    # an append to the log on the eMMC can block while u's remount runs.
    if sysrq_write u quiet; then
      _fr_u=1
    fi
    sysrq_settle
    sysrq_write b
    if [ "$_fr_u" = 1 ]; then
      log "SYSRQ wrote 'u' (logged after b)"
    else
      log "SYSRQ write 'u' to $SYSRQ failed"
    fi
    log "ACTION force_reboot: still alive after sysrq b"
  else
    log "ACTION force_reboot: $SYSRQ missing, skipping sysrq"
  fi
  if [ "$_fr_u" != 1 ]; then
    # `systemctl reboot -ff` syncs before it resets: flush now (bounded),
    # then look once more, so its own sync has little left to do.
    timeout 10 sync >/dev/null 2>&1
    force_hold_ok "$1" "$2" "before systemctl reboot -ff" || return 0
  fi
  timeout 30 systemctl reboot -ff >/dev/null 2>&1
  _fr_rc=$?
  rc_note "$_fr_rc"
  log "ACTION force_reboot: still alive after systemctl reboot -ff rc=$_fr_rc$RCNOTE"
  abandon_reboot "$1" "forced_path_returned:$2"
}

# force_hold_ok REASON WHY WHERE: on a tractor, one more look
# (lock_still_held) at the request and hold the forced path saw, at WHERE in
# it: the record syncs and sysrq settles take seconds, and a person may
# cancel the reboot and remove the request, or the hold may go, meanwhile.
# Lost: the forced attempt is abandoned (BLOCKED gate=tractor_lock). Only
# called before the forced path's own reset (and before sysrq u), so if no
# reset was issued earlier in this boot the record is cleared and the
# request withdrawn.
force_hold_ok() {
  [ "$ROLE" = tractor ] || return 0
  if lock_still_held "$TF_ID"; then
    return 0
  fi
  log "BLOCKED forced reboot gate=tractor_lock ($IDLE_WHY) $3: the motion hold for request $TF_ID is no longer shown"
  add_decision force_blocked:tractor_lock_lost
  BLOCKED=tractor_lock
  if [ "$_fr_was" != 1 ]; then
    clear_reset_issued
  fi
  abandon_reboot "$1" "force_blocked:tractor_lock_lost:$2"
  return 1
}

# abandon_reboot REASON WHY: the reboot that last_action announces did not
# happen and will not be forced in this boot (2026-10-17 review). Rewrite the
# record with action=reboot_abandoned, same boot id and reason, so a later
# reset in this boot (power cut, brown-out, L2 bite, or the accepted reboot
# completing late) reads as supervisor:reboot_abandoned:<reason> instead of a
# plain supervisor reboot. Not deleted: that late completion stays possible,
# which is also why a tractor keeps its motion-hold request once the reset
# was issued (withdraw_lock).
abandon_reboot() {
  kv_int "$STATE/consecutive_reboots" 0
  if write_last_action reboot_abandoned "$1" "$INT" "$2"; then
    log "ABANDONED reboot reason=$1 why=$2: last_action rewritten, so a later reset in this boot is not read as a supervisor reboot"
  else
    log "ABANDONED reboot reason=$1 why=$2: ERROR last_action could not be rewritten, so a later reset in this boot may be read as the supervisor reboot it still names"
  fi
  withdraw_lock "reboot abandoned"
}

# --------------------------------------------------------------------- pass
track_healthy() {
  if [ "$ALL_OK" != 1 ] || [ -n "$FAULTS" ]; then
    rm -f "$RUN/healthy_since" 2>/dev/null
    return 0
  fi
  kv_get "$RUN/healthy_since" ""
  if to_int "$KV"; then
    _th_since=$INT
  else
    _th_since=$UP
    kv_put "$RUN/healthy_since" "$UP"
  fi
  kv_int "$STATE/consecutive_reboots" 0
  _th_n=$INT
  _th_for=$((UP - _th_since))
  if [ "$_th_n" -gt 0 ] && [ "$_th_for" -ge "$HEALTHY_RESET_S" ]; then
    write_record "$STATE/consecutive_reboots" 0
    if [ "$HELD" = 1 ]; then
      log "HEALTHY for ${_th_for}s of uptime: consecutive_reboots $_th_n -> 0 (HELD stays until clear-hold)"
    else
      log "HEALTHY for ${_th_for}s of uptime: consecutive_reboots $_th_n -> 0"
    fi
  fi
}

write_status() {
  [ "$STATE_OK" = 1 ] || return 0
  kv_int "$STATE/consecutive_reboots" 0
  _ws_n=$INT
  kv_int "$RUN/node_streak" 0
  _ws_s=$INT
  printf '%s\n' "updated_uptime_s=${UP:-unknown}
boot_id=$BOOT_ID
role=$ROLE
armed=$ARMED
held=$HELD
consecutive_reboots=$_ws_n
node_fault_streak=$_ws_s
fail_threshold=$FAIL_THRESHOLD
faults=${FAULTS:-none}
decision=${DECISION:-none}
reboot_blocked=${BLOCKED:-none}
tractor_idle=${IDLE_WHY:-not_checked}
config=$CONF_STATE" 2>/dev/null > "$STATE/status.tmp" && mv -f "$STATE/status.tmp" "$STATE/status" 2>/dev/null
}

pass() {
  FAULTS=
  DECISION=
  BLOCKED=
  IDLE_WHY=
  ALL_OK=1
  if ! read_uptime; then
    UP=
    log "ERROR cannot read uptime from $PROC_UPTIME: pass skipped (no wall-clock fallback by design)"
    return 0
  fi
  rotate_log
  ARMED=0
  if [ "$STATE_OK" = 1 ] && [ -e "$ARMED_FILE" ]; then
    ARMED=1
  fi
  kv_get "$RUN/armed" ""
  if [ "$STATE_OK" = 1 ] && [ "$KV" != "$ARMED" ]; then
    log "ARMED state ${KV:-unset} -> $ARMED ($ARMED_FILE $( [ "$ARMED" = 1 ] && echo present || echo absent))"
    kv_put "$RUN/armed" "$ARMED"
  fi
  HELD=0
  if [ -f "$STATE/held" ]; then
    HELD=1
    _ps_h=
    while IFS= read -r _ps_l; do
      case $_ps_l in reason=*|consecutive=*|cap=*) _ps_h="$_ps_h ${_ps_l}" ;; esac
    done < "$STATE/held"
    log "HELD: supervisor reboots suppressed until '$0 clear-hold' (held:$_ps_h); container restarts continue; an OTA trial boot overrides it (PROBATION)"
  fi

  if check_pending_reboot; then
    write_status
    return 0
  fi

  check_docker
  check_containers
  check_heartbeats

  kv_int "$RUN/node_streak" 0
  _ps_s=$INT
  if [ -n "$FAULTS" ]; then
    _ps_s=$((_ps_s + 1))
    kv_put "$RUN/node_streak" "$_ps_s"
    log "NODE fault=$FAULTS streak=$_ps_s/$FAIL_THRESHOLD"
    if [ "$PENDING_DUE" = 1 ]; then
      # Fresh unit-watchdog budget for the forced path (syncs, sysrq
      # settles, `reboot -ff`).
      notify_watchdog
      force_reboot "$PENDING_RSN" "alive_${PENDING_EL}s_after_systemctl_reboot"
    elif [ "$_ps_s" -ge "$FAIL_THRESHOLD" ]; then
      # The rung can take ~210 s on its own (fw_printenv, syncs, systemctl,
      # the forced path); start it with a fresh unit-watchdog budget.
      notify_watchdog
      reboot_rung "$FAULTS"
    fi
  else
    if [ "$PENDING_DUE" = 1 ]; then
      log "PENDING systemctl reboot accepted ${PENDING_EL}s ago (reason=$PENDING_RSN) but no node fault now: not forcing"
      add_decision force_skipped:healthy
      abandon_reboot "$PENDING_RSN" force_skipped:healthy
    fi
    if [ "$_ps_s" -gt 0 ]; then
      log "NODE fault cleared after $_ps_s pass(es)"
      kv_put "$RUN/node_streak" 0
    fi
  fi
  track_healthy
  write_status
}

# ---------------------------------------------------------------- commands
show_file() {
  printf '%s\n' "--- $1"
  if [ -f "$STATE/$1" ]; then
    while IFS= read -r _sf || [ -n "$_sf" ]; do
      printf '%s\n' "$_sf"
    done < "$STATE/$1"
  else
    printf '%s\n' "(none)"
  fi
}

cmd_status() {
  printf '%s\n' "state_dir=$STATE"
  if [ -e "$ARMED_FILE" ]; then
    printf '%s\n' "armed_file=$ARMED_FILE (present: ARMED)"
  else
    printf '%s\n' "armed_file=$ARMED_FILE (absent: disarmed, WOULD-only)"
  fi
  for _cs in status held consecutive_reboots last_reset_reason last_action boot_id probation_reboot; do
    show_file "$_cs"
  done
}

cmd_clear_hold() {
  read_boot_id
  read_uptime || UP=
  init_state
  if [ "$STATE_OK" != 1 ]; then
    log "ERROR clear-hold: state dir $STATE unusable"
    return 1
  fi
  _ch=absent
  [ -f "$STATE/held" ] && _ch=present
  rm -f "$STATE/held" 2>/dev/null
  write_record "$STATE/consecutive_reboots" 0 || return 1
  log "CLEAR-HOLD by operator: held file was $_ch; removed; consecutive_reboots -> 0"
}

usage() {
  printf '%s\n' "usage: $0 run|once|status|clear-hold" >&2
}

case ${1:-} in
  run|once) ;;
  status) cmd_status; exit $? ;;
  clear-hold) cmd_clear_hold; exit $? ;;
  *) usage; exit 2 ;;
esac

startup
if [ "$1" = once ]; then
  pass
  notify_watchdog
  exit 0
fi
log "START role=$ROLE config=$CONF_STATE armed_file=$ARMED_FILE containers='$CONTAINERS' heartbeats='$HEARTBEATS' interval=${CHECK_INTERVAL_S}s threshold=$FAIL_THRESHOLD grace=${REBOOT_GRACE_S}s tractor_reboot=$TRACTOR_REBOOT_ENABLED tractor_idle_file=$IDLE_FILE"
while :; do
  pass
  notify_watchdog
  sleep "$CHECK_INTERVAL_S"
done
