# x8_recovery — reboot and watchdog recovery for the Portenta X8

Host-side pieces of the reboot / power-cycle plan in
[X8_HEALTH_AND_RECOVERY/REBOOT_AND_POWER_CYCLE.md](../../X8_HEALTH_AND_RECOVERY/REBOOT_AND_POWER_CYCLE.md)
(TODO.md **PWR-2** and **PWR-4**). Targets: Portenta X8 on a Max Carrier,
Arduino LmP Linux, systemd 250, Docker 20.10 (tractor) / 24.0 (base).

**Status (2026-10-18): built, installed nowhere, disarmed.** The supervisor
reboots the tractor only when idle (D-PWR1); that gate is built and blocks
every tractor reboot until the H747 side exists (PWR-13). Nothing here has
run on a bench board. Valve safety stays with the M4 and Opta 200 ms
watchdogs; nothing in this folder may gate or wait on them.

## Files

| File | Layer | What it is |
|---|---|---|
| `systemd/system.conf.d/90-lifetrac-watchdog.conf` | L2 | systemd drop-in: PID 1 owns the i.MX hardware watchdog (`RuntimeWatchdogSec`, `RebootWatchdogSec`). |
| `systemd/reboot.target.d/90-lifetrac-timeout.conf` | L2 | systemd drop-in: bounds a hung `systemctl reboot` (`JobTimeoutSec`, then `reboot-force`). |
| `install_watchdog.sh` | L2 | Installs, removes or checks the two drop-ins above, with a role gate (base / bench / tractor; the tractor does not get L2 under D-PWR1). See its header. |
| `lifetrac-supervisor.sh` | L3 | Host health supervisor: checks, allowlisted container restarts, guarded reboot rung. POSIX sh. |
| `lifetrac-supervisor.service` | L3 | systemd unit for the supervisor (`run` loop, `WatchdogSec=300`). Not installed by any script yet. |
| `supervisor.conf.example` | L3 | Annotated config; copy to `/etc/lifetrac/supervisor.conf`. Ships with an empty container allowlist. |

SIL tests, run from `base_station/` with `python -m pytest tests -q`:
`tests/test_x8_supervisor_sil.py` (L3 supervisor) and
`tests/test_x8_watchdog_ownership_sil.py` (L2).

## L3 — `lifetrac-supervisor`

### What one pass does (every `CHECK_INTERVAL_S`, default 30 s)

1. `timeout 20 docker info` (when `CHECK_DOCKER=1`).
2. For each name in the `CONTAINERS` allowlist:
   `timeout 20 docker inspect --type container -f '{{.Name}} {{.State.Status}} {{if .State.Health}}…'`.
   Unhealthy = not `running`, health `unhealthy`, or no such container.
   After `FAIL_THRESHOLD` bad passes in a row: `timeout 60 docker restart`,
   at most `CONTAINER_RESTART_MAX` times per container per boot. Docker never
   restarts a container just for being unhealthy, so this is what acts on the
   compose healthchecks. Docker looks an unknown name up as a container ID
   prefix, so hex-only names are refused at config time, and a container
   whose inspected name is not exactly `/<name>` is never restarted
   (`REFUSED restart`). Duplicate entries are ignored.
3. `HEARTBEATS` files: stale when the mtime **value** has not changed for more
   than `max_age` seconds of **uptime** (missing file: since the supervisor
   started watching it).
4. A node-level fault (docker down, stale heartbeat, a container whose
   restarts are used up or whose next restart cannot be recorded because the
   state directory is not writable) in `FAIL_THRESHOLD` passes in a row goes to the
   reboot rung, through these gates in order:

| Gate | Blocks the reboot when | Logged as |
|---|---|---|
| grace | uptime < `REBOOT_GRACE_S` (600 s) | `BLOCKED reboot gate=grace` |
| tractor | `ROLE=tractor` and `TRACTOR_REBOOT_ENABLED` is not 1. Never overridden. | `BLOCKED reboot gate=tractor_reboot_disabled`, also `reboot_blocked=` in `status` |
| HELD | `$STATE/held` exists (unless the OTA override below applies) | `HELD reboot suppressed` (plus a `HELD:` line every pass) |
| cap | `consecutive_reboots` ≥ cap → writes `held` (unless the OTA override applies) | `HELD entering hold` |
| idle | `ROLE=tractor` and the H747 does not report a live `idle` (D-PWR1, below). Read-only, so it runs disarmed too. Never overridden. | `BLOCKED reboot gate=tractor_not_idle`, `tractor_idle=` in `status` |
| armed | `/etc/lifetrac/supervisor.armed` is absent | `WOULD reboot` |
| lock | `ROLE=tractor` and the H747 does not confirm a motion hold for this reboot within `TRACTOR_IDLE_WAIT_S` polls; the request is withdrawn | `BLOCKED reboot gate=tractor_lock` |

**ROLE defaults to tractor** (2026-10-17 review). Only an explicit
`ROLE=base` in a config that loads cleanly makes a node a base. A missing
config, a file without a `ROLE` line (or with
lower-case `role=`), an invalid value, or a rejected file all read as
tractor, so a typo cannot turn a tractor's reboot rungs on. The whole config
file is rejected (all defaults, `ROLE=tractor`, logged with the shell's own
message) when any line writes to stderr, for example `ROLE = tractor` with
spaces, a UTF-8 byte-order mark, or an unquoted `CONTAINERS=a b` anywhere in
the file. Before this, `.` only checked the last line, so such a line was
skipped silently.

The cap is `REBOOT_MAX_CONSECUTIVE` (3). While `fw_printenv -n
upgrade_available` prints 1 (an OTA trial boot) it becomes
`max(REBOOT_MAX_CONSECUTIVE, bootlimit + 1)`. That alone did not stop a
counter carried over from before the update, or a `held` file left from
before it, from blocking the rollback. So while `fw_printenv -n bootcount`
is ≤ `bootlimit` (rollback not due yet), neither HELD nor the cap stops a
reboot (`PROBATION overrides ...`). u-boot bounds that loop: its
`rollback_setup` clears `upgrade_available`. In case the env copy is stale
(images that keep the flag in RPMB), the override stops when bootcount did
not rise after the supervisor's own probation reboot in the previous boot
(`$STATE/probation_reboot`). The counter goes back to 0 after
`HEALTHY_RESET_S` (1800 s) of continuous health in one boot; HELD stays
until a person runs `clear-hold`.

The reboot itself: persist the counter, write `$STATE/last_action` (tmp file,
`sync`, rename, `sync`), then `timeout 30 systemctl reboot`. If that fails,
it writes `s`, `u`, `b` to `/proc/sysrq-trigger` (three writes,
`SYSRQ_SETTLE_S` apart), then tries `systemctl reboot -ff`. Never sysrq `c`.
If `systemctl reboot` was accepted but the supervisor is still alive
`FORCE_REBOOT_AFTER_S` (180 s) later, the same forced path runs only when
the board is still armed, not HELD, past the tractor gate, a node fault is
still present, and the request is not stale (older than
`FORCE_REBOOT_AFTER_S + 2 × CHECK_INTERVAL_S`). Both are counted from the
uptime read right after `systemctl reboot` was accepted, not from the start
of the pass. Otherwise the pending marker is dropped (`BLOCKED forced reboot
gate=...`, `PENDING marker stale`, or `not forcing`) and the normal checks
resume. A dropped marker, or a forced path that returns, rewrites
`last_action` as `reboot_abandoned` (`ABANDONED reboot` in the log), because
the board then stays up in that boot.

### Why it is shaped this way

- **POSIX sh only.** The tractor's LmP host Python has no `json`, `select`,
  `socket` or `subprocess`. Every external command runs under `timeout`, so a
  wedged dockerd cannot hang the loop. Not bounded: file writes, renames and
  `sync` on wedged storage. A process in uninterruptible I/O cannot be killed
  by `timeout` or by the unit watchdog (VERIFY on the bench).
- **Uptime, never wall-clock time.** No battery-backed RTC on the tractor;
  the base clock steps at boot. Rate limits, grace and staleness all use
  `/proc/uptime` inside one `boot_id`.
- **Disarmed by default (bench rule).** No supervisor-initiated restart or
  reboot on a bench board without an explicit GO. Disarmed, every check runs
  and the actions it would take are logged as `WOULD ...`; the dry run counts
  its would-be restarts separately, so arming mid-boot starts from a full
  budget.
- **Tractor reboots off.** Any X8 reboot resets the H747 (reflashed by
  `stm32h7-program.service` at every tractor boot), which in the production
  design trips the Opta's 200 ms watchdog, drops the PSR and kills the engine
  behind a latch that needs a person at the machine. TODO.md PWR-8, PWR-9,
  PWR-11, PWR-12 and PWR-13 gate `TRACTOR_REBOOT_ENABLED=1`. Because the role is not
  always known for sure, an unknown role counts as tractor (see above).
- **Tractor reboots only when idle** (Dorkmo, 2026-10-18,
  [DECISIONS.md D-PWR1](../../DECISIONS.md#d-pwr1--tractor-automatic-reboots-idle-only)).
  The H747 decides what idle is (engine stopped, every valve output off) and
  must hold motion off until the reset, because an orderly shutdown can take
  minutes. The two talk through two tmpfs files and a link service on the
  X8 that does not exist yet (TODO.md PWR-13), so today every tractor reboot
  is blocked:
  - `/run/lifetrac/tractor_idle`, written by the link service at least once
    a second (tmp + rename) in a directory it owns, first line
    `busy <seq>`, `idle <seq>` or `locked <seq> <id>`, all from one H747
    frame. `<seq>` (1-32 letters or digits) comes from the H747's own
    status frames and changes at least once a second; a file whose `<seq>`
    does not change while the supervisor watches is not live and never
    reads as idle, and a line whose `<seq>` is malformed is invalid, never
    idle or a hold, even from a link already seen live. The supervisor reads at most 256 bytes, with an open
    that refuses a symlink and does not block (GNU `dd`
    `iflag=nofollow,nonblock`; VERIFY on LmP under PWR-1, and a `dd`
    without them reads as a missing file, so it fails closed), and logs
    only `busy`, `idle`, `locked` or `invalid` from it.
  - `/run/lifetrac-supervisor/tractor_reboot_request`, in a root-only
    directory the supervisor creates (never `RuntimeDirectory=`, which
    would delete it when the shutdown stops the supervisor): first line
    `<id>` (`<boot id prefix>-<uptime>`; 1-40 letters, digits or `-`). The
    link service only reads it and mirrors it to the H747, which latches
    the hold when it sees a request while idle, keeps it while busy,
    re-latches it for a new `<id>`, and releases it only when it resets or
    the link reports no request file. A link service stopped by the
    shutdown leaves the hold in place.
  The rung waits for a live `idle`, writes the request, waits for a live
  `locked <seq> <id>` (`TRACTOR_IDLE_WAIT_S` polls of one second each, 10
  by default, 2..15), persists its records, records the reset in
  `/run/lifetrac-supervisor/reset_issued`, looks once more (request still
  ours, idle file still `locked <seq> <id>`) and only then calls
  `systemctl reboot`. Before that point every path where the reboot does
  not happen (refused, blocked) deletes the request, and a request left
  from an earlier run is deleted at start. Once the reset is issued
  (`systemctl reboot` accepted, timed out or killed by a signal, or the
  forced path) the request stays until the reset, even when the supervisor abandons the
  reboot: systemd can still carry out the reboot it accepted (a stop job
  that hangs, then upstream's 30 min `reboot-force`), and the tractor
  must not be movable when it lands. To release a kept hold by hand,
  `systemctl cancel` the queued reboot, then remove the request file; the
  next rung treats that reset as cancelled. A rung that stops at the last
  look does not count toward the boot-loop cap. The forced path on a tractor runs only while the
  link still shows the hold, or when the link is no longer live (stopped
  by the shutdown) and its last line shows the hold; never when the
  request or the idle file is gone or anything else is shown. It looks
  again after writing its last_action record, again after the `s` settle
  (before `u`, which remounts the disks read-only and cannot be undone,
  so from `u` on it goes ahead with `b`, one settle later), and, when `u`
  was not written (no sysrq file, or its writes failed), after a bounded
  sync right before `systemctl reboot -ff` (which syncs again before it
  resets). If the hold is gone by then it gives up, and if no reset was
  issued earlier in this boot it withdraws the request. It writes its
  reset record right before the last of these looks (and drops it again
  when `u` and `b` both failed, until the look before `reboot -ff`), so a
  supervisor killed outright during the record syncs, settles or that
  sync (the unit watchdog, SIGKILL) leaves none, and the restart
  withdraws the request. It is refused when its reset record cannot be
  written and no earlier reset in this boot recorded one. When `u` was not
  written and `systemctl reboot -ff` fails too (its own error, or it never
  ran), nothing was sent, so the record goes and the request is withdrawn;
  a timeout or a signal keeps them, as for `systemctl reboot`. Stopping the service (`systemctl stop`, a kill
  by hand) withdraws a request that has no reset behind it, because an
  explicit stop does not restart the service and the startup cleanup
  would not run. That includes a stop after the reset record is written
  but before the reset is sent (the rung's last look, or the forced
  path's after a `systemctl reboot` that failed outright).
  A stop once the reset was sent (`systemctl reboot` called, `reboot -ff`,
  or an earlier reset in this boot), such as the shutdown of that reboot,
  keeps it. Once the forced path has written `u` (the disks are read-only
  from then on), a stop waits until `b` and `reboot -ff` are sent, then
  keeps the request. Not handled yet (TODO PWR-14): a supervisor killed
  outright between `u` and `b` leaves the disks read-only with no reset
  queued; the restart keeps the motion hold but runs disarmed, so the X8
  waits for a person to cycle power. No OTA
  probation override. The systemd-owned hardware watchdog (L2) is not
  installed on the tractor, because a watchdog reset cannot wait for idle,
  so a shutdown that hangs after the supervisor has stopped waits for
  upstream's 30 min `reboot.target` bound, with motion held.
- **Empty allowlist; no LifeTrac container is safe on it yet** (2026-10-17
  review). Restarting a broker silently drops the subscriptions of clients
  that subscribe once after `connect()`, with a clean session and no
  `on_connect`: on the base `lora_bridge` (`cmd/estop`, `cmd/control`) and
  `audit_tail`, on the tractor `camera_service`, so E-stop forwarding and
  the audit trail stop with nothing logged (the compose files say the same;
  TODO.md PWR-3 tracks the resubscribe fix). Restarting `web_ui` republishes
  retained radio settings (a restart once caused a live on-air profile
  switch, TODO.md RS-0.6); restarting `image_tx` / `image_rx` sends the L072
  a `RESET_REQ`; `lora_bridge` is obsolete (RS-5.11). The example config has
  no commented entry to uncomment.
- **Never-do list** (header of the script): no GPIOs (gpio10, gpio163), no
  openocd, no `x8h7_*` module loads/unloads, no restart of
  `stm32h7-program.service` or `docker.service`, nothing on `/dev/ttymxc3`, no
  L072 flash / unprotect / AN3155 tools, nothing outside the allowlist, no
  sysrq `c`. The SIL test checks the script's code lines for these.

### Commands

```sh
lifetrac-supervisor.sh run          # the service loop
lifetrac-supervisor.sh once         # one pass; acts like `run` when armed
lifetrac-supervisor.sh status       # print the state files; changes nothing
lifetrac-supervisor.sh clear-hold   # remove held, zero consecutive_reboots
```

Do not run `once` by hand on an armed board while the service is active:
both share the state directory.

### State (`/var/lib/lifetrac/supervisor`)

| File | Content |
|---|---|
| `status` | key=value snapshot of the last pass: armed, held, faults, decision, `reboot_blocked`. |
| `supervisor.log` (+ `.1`) | `boot=<8 chars of boot_id> up=<uptime> <event>`, one line per event; rotated past 256 KiB. The same lines go to the journal. |
| `last_action` | Written before every reboot action: `action`, `reason`, `boot_id`, `uptime_s`, `consecutive`, `role`, `detail`. Rewritten with `action=reboot_abandoned` when a requested reboot did not happen and will not be forced. |
| `last_reset_reason` | Set once per boot: `supervisor:reboot:<reason>` or `supervisor:force_reboot:<reason>` if the previous boot ended in a supervisor reboot; `supervisor:reboot_abandoned:<reason>` if a supervisor reboot was requested in that boot but abandoned, so the boot ended some other way later (or that reboot completed late: the cause is uncertain); else `external_or_unknown`. No telemetry route yet; a container can read this file. |
| `boot_id` | Boot id of the last run, for the classification above. |
| `consecutive_reboots`, `held` | Boot-loop guard. |
| `probation_reboot` | Boot id and u-boot `bootcount` at the last reboot issued under the OTA override; read in the next boot. |
| `per_boot/` | Streaks, restart counts, heartbeat tracking, previous boot id, pending reboot. Wiped when the boot id changes. |

### Manual install (no script does this yet)

As root on the X8 (`/opt` is `/var/rootdirs/opt` on LmP and survives OSTree
updates):

```sh
mkdir -p /opt/lifetrac/x8_recovery /etc/lifetrac
cp lifetrac-supervisor.sh /opt/lifetrac/x8_recovery/ && chmod 755 /opt/lifetrac/x8_recovery/lifetrac-supervisor.sh
cp supervisor.conf.example /etc/lifetrac/supervisor.conf      # ships ROLE=tractor; base: set ROLE=base
cp lifetrac-supervisor.service /etc/systemd/system/
systemctl daemon-reload && systemctl enable --now lifetrac-supervisor.service
journalctl -u lifetrac-supervisor -f                           # watch the WOULD lines
```

`supervisor.conf.example` is not in the repo's `.gitattributes` LF list, so a
Windows checkout may copy it with CRLF; the supervisor strips CR from each
line before it runs the config, so such a copy still loads.

Arming is a separate, deliberate step: `touch /etc/lifetrac/supervisor.armed`
(not on a bench board without a GO; reboots additionally need `ROLE=base`,
or on a tractor `TRACTOR_REBOOT_ENABLED=1`). Remove the file to disarm; the
next pass logs the change. Arming also arms the forced-reboot rung (sysrq
`s`,`u`,`b`, then `systemctl reboot -ff`), which has no switch of its own and
can land mid-write on `/mnt/boot` (FAT, the only u-boot env copy), so arming
any production node waits on TODO.md PWR-7 (crash-only storage) as well.

The unit has no dependency on `docker.service` at all (no `Requires=`,
`Wants=` or `After=`): starting or restarting the supervisor never starts a
stopped `docker.service` (which would start every container with a restart
policy, radio daemons included), stopping docker never stops the
supervisor, and a dockerd that hangs at boot before it reports ready
(LmP's unit is `Type=notify` with `TimeoutSec=0`) cannot keep the
supervisor from starting. The first passes may find docker still starting;
the 600 s reboot grace covers that. LmP's `multi-user.target` already
starts docker at boot.

### Open items

- **VERIFY on the bench (PWR-1):** that a plain reboot on the i.MX8MM asserts
  `WDOG_B` (PMIC cycle), which is why there is no sysrq `c` rung; and which
  copy of `upgrade_available` and `bootcount` LmP uses on these boards
  (`fw_printenv` u-boot env vs `fiovb_printenv` RPMB; the base bench env
  shows both at 0, `bootlimit=5`, no `fiovb_rpmb`).
- **Decided 2026-10-17 after review:** `ROLE` defaults to tractor, not
  base, so a missing or broken config can never turn tractor reboots on; and
  the unit has no dependency on `docker.service` (see above).
- An install step in provisioning (an `install_supervisor` script or part of
  X8OS-4), merging with OTA_STRATEGY.md's `lifetrac-health`, and publishing
  `last_reset_reason` in telemetry.
- The plan dropped the L072/H7 NRST rungs (§L3 "Removed from the earlier
  draft"); they stay on the never-do list above.
