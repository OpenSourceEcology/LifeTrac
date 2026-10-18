# Reboot and power cycle — X8 + Max Carrier

**Status: revised 2026-10-17** after an adversarial review (hardware facts,
repo consistency, safety/security, buildability; 74 findings survived
verification) and Copilot's review of PR #153. The software layers that can
be built without board access are in the repo but **installed nowhere and
disarmed by default**: [`firmware/x8_recovery/`](../firmware/x8_recovery/README.md)
(watchdog drop-ins, `install_watchdog.sh`, `lifetrac-supervisor`) and the
compose healthchecks. The hardware layer (L4) and remote restart (L5) are
design only. Nothing here has been run on a board. **VERIFY** marks claims
inferred from datasheets, schematics or source code that need a bench check
(collected in §6 / TODO PWR-1). Tracked as TODO.md **PWR-1..PWR-13**.

**Decided 2026-10-18 (Dorkmo, [DECISIONS.md D-PWR1](../DECISIONS.md#d-pwr1--tractor-automatic-reboots-idle-only)):
the supervisor reboots the tractor only when idle**, meaning the engine is
stopped and every valve output is off when the reboot is requested, and the
H747 then holds motion off until the reset. The supervisor enforces this
(L3 guards below); the H747 side is TODO PWR-13, after PWR-12. Two automatic
resets stay outside the rule: a WDOG1 bite on a kernel hang (kept on
purpose) and LmP's `docker.service` `FailureAction=reboot` until PWR-13
turns it off on the tractor.

Question this answers: *how can we reboot or power-cycle the Portenta X8 and
the Max Carrier, which of those do we already use, and how do we build them
into the system so a wedged node recovers on its own or on command?*

Companions: [recovery/SOFT_RESET_INDEX.md](recovery/SOFT_RESET_INDEX.md),
[recovery/T2_cold_power_cycle.md](recovery/T2_cold_power_cycle.md),
[../POWER_MANAGEMENT.md](../POWER_MANAGEMENT.md),
[../SAFETY_CASE.md](../SAFETY_CASE.md),
[../CYBERSECURITY_CASE.md](../CYBERSECURITY_CASE.md) §4.4,
[../OTA_STRATEGY.md](../OTA_STRATEGY.md).

---

## 1. The short version

1. **A plain reboot is probably already a full PMIC power cycle of the X8
   module.** The kernel reboots through PSCI `SYSTEM_RESET`; NXP's TF-A for
   the i.MX8M Mini is built with `IMX_WDOG_B_RESET` and asserts `WDOG_B`,
   which is wired to the BD71847 PMIC. The X8 device tree also sets
   `fsl,ext-reset-output` on `wdog1`. Inferred from source; **VERIFY** with
   `wdog_regs.py` (WRSR `POR=1` vs `SFTW=1`) after the next reboot. The old
   "reboot is a warm reset, only a watchdog bite is a POR" idea is dropped,
   and with it the deliberate kernel-crash rung.
2. **The hardware watchdog exists but nothing in userspace owns it**, so a
   hung systemd never recovers. Fix: systemd `RuntimeWatchdogSec=30s` (L2).
3. **On the tractor, every module reset is also an engine stop.** In the
   production design the H747 is the Modbus master; any X8 reset resets it
   (tractor: `stm32h7-program` halts and reflashes it at every boot), the
   Opta trips at 200 ms, the PSR drops, the engine stops, and the latch needs
   a person at the machine. Automatic recovery restores comms and telemetry,
   **never motion**. That is why tractor reboots are off by default in
   everything built here, and when turned on they happen only while the
   tractor is idle (D-PWR1).
4. **The carrier cannot be power-cycled by anything on the carrier.** The
   only full cold cycle is cutting the 12 V input (and the 18650, and the X8
   USB-C). That needs an external, independently time-limited switch (L4),
   which has real hardware decisions behind it.

## 2. What a reset actually reaches

| Method | i.MX Linux | H747 (M7+M4) | Carrier +3V3 = X8 VCC (L072, USB hub, microSD) | Carrier +5V (USB-A VBUS, mPCIe) / +3V8 (modem) | Tractor safety effect (production design) |
|---|---|---|---|---|---|
| `systemctl reboot` | PSCI `SYSTEM_RESET` → TF-A asserts `WDOG_B` → PMIC to SNVS and back (**VERIFY**) | Tractor: held and reflashed by `stm32h7-program` at boot (~13 s). Base: script says "No Update", so reset only if the reboot is a PMIC cycle (**VERIFY**) | follows the PMIC 3.3 V rail if it is a PMIC cycle (**VERIFY** drop time) | no | Opta trip → PSR → engine stop, latched |
| `reboot -ff` / sysrq `b` | same path as above, no clean shutdown | same | same | no | same |
| i.MX WDOG1 timeout | PMIC power cycle (bench: WRSR `POR=1`, 2026-09-12) | power cycled (H747 on PMIC `BUCK6` VDD_3V3, **VERIFY**) | yes (**VERIFY** drop time) | no | same |
| `systemctl poweroff` / `halt` | not a final state: base (934/6.1) booted again ~4.5 min after poweroff; a halted kernel stops petting and `imx2_wdt` re-arms to 128 s at shutdown, so a halt ends in a PMIC reset (inferred) | follows | follows | no | same |
| H7 NRST (`gpio10`, openocd `reset run`) | no | hard reset; does **not** revive the Linux x8h7 drivers, and reloading them OOPSes the 6.1 kernel | no | no | same (M7 RAM E-stop latch is cleared) |
| L072 NRST (`gpio163`) | no | no | L072 only; needs a live x8h7 bridge; dead on the tractor today (RS-5.2) | no | stop-by-absence, no latch |
| L072 `RESET_REQ` (0x03) over `/dev/ttymxc3` | no | no | L072 soft reset; sent by the radio daemons at every link open | no | stop-by-absence, no latch |
| Max Carrier PB1 / X8 module PB1 (`PWRON`, HDC J2-05) | short press: PMIC IRQ → key event → systemd-logind default `HandlePowerKey=poweroff`; long press (≥3 s by DT, 10 ms OTP before the driver loads): PMIC off/on (**VERIFY** both) | follows | follows | no | same as reboot |
| Max Carrier PB2 RESET, HDC J1-73 `RESET` | target **VERIFY** (repo notes say i.MX nRST; J1-73 is driven only by the on-carrier debugger, so it is not a field path) | ? | ? | no | ? |
| Cut 12 V input, no 18650, no X8 USB-C | full cold start | yes | yes | yes | same |
| Cut 12 V with an 18650 fitted | depends on the BQ24195: the cell reaches +5V (and the X8) only in boost/OTG mode (REG01[5:4]=10); the reset default is charge mode. **VERIFY** | depends | depends | +3V8 modem stays up on the cell via BATFET | depends |

Hardware facts behind the table:

- **PMIC:** ROHM BD71847. `WDOG` pin ← i.MX `WDOG_B` (R3, 0 Ω); `POR` →
  i.MX `POR_B`; `PWRON` ← both PB1 buttons. The X8 DT sets
  `rohm,reset-snvs-powered`, so a WDOG/power-key reset goes to SNVS, not to
  full off: every rail except the SNVS domain cycles. Sources:
  [ABX00049 schematics](https://docs.arduino.cc/resources/schematics/ABX00049-schematics.pdf),
  [BD71847 DT binding](https://www.kernel.org/doc/Documentation/devicetree/bindings/mfd/rohm%2Cbd71847-pmic.yaml).
- **Reboot path:** Linux arm64 `machine_restart` → PSCI `SYSTEM_RESET`
  (priority 129 beats `imx2_wdt`'s 128); NXP imx-atf
  `plat/imx/imx8m/imx8mm/include/platform_def.h` defines
  `IMX_WDOG_B_RESET` on every branch checked, and `imx_wdog_restart(true)`
  clears WDA "to assert WDOG_B". Arduino's TF-A patch set does not touch
  that file. A `reboot=warm` command line would take PSCI `SYSTEM_RESET2`
  instead; LifeTrac must not set it.
- **WDOG1** is armed by u-boot on both bench boards: `WCR=0x773d`, 60 s,
  `WDT=1`. A timeout reads `POR` in WRSR, so WRSR cannot tell a watchdog
  reset from a power-on ([wdog_regs.py](../firmware/x8_lora_bootloader_helper/bench_tools/wdog_regs.py),
  [FLASH_RUNBOOK §3](../firmware/x8_lora_bootloader_helper/FLASH_RUNBOOK.md)).
  The PMIC's own `RESETSRC` register (0x29: `RWDOG`, `RPWRON`, `RSWRST`,
  `RVSYS_2P7`, ...) is cleared only by UVLO and could tell them apart
  (**VERIFY**).
- **Nothing in userspace owns the watchdog today.** `system.conf` has only
  the commented default `#RuntimeWatchdogSec=off` and no `/etc` drop-in; the
  direct proof is that `wdt_pet.sh` opened `/dev/watchdog0` without `EBUSY`
  on both boards (2026-09-12). Vendor drop-ins under `/usr/lib` were never
  captured; `capture_board_state.sh` now records the effective value.
- **Early boot is uncovered:** from `imx2_wdt` probe until PID 1 opens the
  device, the kernel pets WDOG1 forever (`watchdog.open_timeout=0`), so a
  hang in the initramfs or early systemd never resets. The kernel argument
  `watchdog.open_timeout=` fixes that, but only together with L2 (otherwise
  every boot resets) — X8OS-4.
- **Max Carrier power** ([ABX00043 datasheet](https://docs.arduino.cc/resources/datasheets/ABX00043-datasheet.pdf),
  [pinout](https://docs.arduino.cc/resources/pinouts/ABX00043-full-pinout.pdf)):
  jack 6–36 V → MPM3550 (enable tied to input) → +5V; 18650 via BQ24195
  ("can be used as backup power source if the external power supply fails";
  "Priority is with the jack delivered power"). Nothing powers the whole
  carrier off. Sub-rails are switchable through x8h7 GPIOs: SARA modem
  `PWR_ON`/`RESET`, the mPCIe 3V3 buck `EN`, USB-A VBUS via the USB2514B
  port-power pins. Cellular is archived, so the modem can simply be held off.
- **X8 module input:** VIN 4.5–5.5 V from the carrier, or its own USB-C;
  either keeps the module alive, which is why T2 unplugs both.

## 3. What LifeTrac already uses

| Layer | Mechanism | Where | State |
|---|---|---|---|
| Containers | `restart: unless-stopped` | [`docker-compose.yml`](../docker-compose.yml), [`firmware/tractor_x8/docker-compose.yml`](../firmware/tractor_x8/docker-compose.yml) | Declared, but no production stack runs on either board: the base runs only the bench broker; `lifetrac-base.service` is retired and not installable (RS-5.11); the tractor camera app starts only through the udev unit and is stopped at every bench power-up. |
| systemd units | `Restart=on-failure` | `firmware/x8_lora_bootloader_helper/lifetrac-camera.service` (tractor, udev-started `docker compose up` wrapper), `bench_tools/units/disable-wifi.service` | The only LifeTrac units installed. `firmware/tractor_x8/systemd/*.service` are OBSOLETE and installed nowhere (RS-5.16). No `WatchdogSec=` anywhere. |
| LmP auto-reboot | `docker.service` `FailureAction=reboot` (tractor) / `StartLimitAction=reboot` (base) after 3 failed starts in 60 s; `aktualizr-lite` `StartLimitAction=reboot` (inactive: no `sota.toml`) | LmP image | Live. Restarting `docker.service` in a loop can reboot the board; the supervisor never does it. On the tractor this reboot does not wait for idle, so D-PWR1 turns it off there (PWR-13). |
| OTA rollback | u-boot `bootcount` / `bootlimit=5`, `upgrade_available` in `/mnt/boot/uboot.env` (FAT, single copy) | LmP image | Every reboot, WDOG bite or power cut during a trial boot counts toward rollback. |
| Linux reboot | `systemctl reboot` | flash pipeline `REVIVE_MODE=reboot` | Bench; boards back in 12–24 s. |
| i.MX watchdog | u-boot arms WDOG1, kernel keepalive pets; `wdt_pet.sh` holds it during L072 flashes and hands it back with magic close `'V'` | `firmware/x8_lora_bootloader_helper/wdt_pet.sh` | Bench. Closing without `'V'` power-cycles the board 60 s later (tractor, 2026-09-12). |
| H7 reset | `gpio10` NRST, openocd `reset run` | `pulse_h7_nrst.sh`, `revive_bridge.sh` | Bench only. |
| L072 reset | `RESET_REQ` (0x03) at every radio link open; `gpio163` NRST and openocd BOOT0/NRST | `image_tx_daemon.py`, `image_rx_daemon.py`; bench scripts | `RESET_REQ` is in the production daemons. There is no `AT+REBOOT` in the LifeTrac L072 firmware. The L072 IWDG is designed but never started (RS-4.10). Under the H7↔L072 contract the H7 owns NRST and its own throttled recovery ([04 §5](../DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md)). |
| Valve safety | M4 200 ms software watchdog → PSR; Opta 200 ms Modbus watchdog | `tractor_h7_m4`, `tractor_opta` | Designed. They make valves safe; they restart nothing. The draft Opta firmware latches `WATCHDOG_TRIPPED` until the Opta itself is power-cycled, which contradicts TRACTOR_NODE.md's "re-enables after ≥10 advances" (PWR-11). |
| Full power cycle | Unplug 12 V + USB-C by hand (T2) | [T2](recovery/T2_cold_power_cycle.md) | Manual only. |
| Key-off power | Self-holding relay held by an H747 output | [POWER_MANAGEMENT.md](../POWER_MANAGEMENT.md) | Design draft. The hold drops whenever the H747 resets (PWR-12). |

## 4. Safety and security consequences (read before arming anything)

- **Tractor resets stop the engine.** Every rung that restarts the H747 (any
  reboot, WDOG bite, L4 cut, H7 NRST) stops the Modbus master, trips the
  Opta within 200 ms and drops the PSR. The M7's E-stop latch lives in RAM
  and is cleared by the reset, so the **Opta and PSR latches are the
  authoritative ones and must survive every rung**; the M7 must read the
  Opta `safety_state` at boot and stay latched if it is not `NORMAL`
  (PWR-11). Recovery restores comms and telemetry, not motion. The base node
  has no such effect. Since D-PWR1 a supervisor-initiated tractor reboot
  also needs the tractor idle and the H747 holding motion off, so it never
  lands mid-work. Two automatic resets still can: LmP's `docker.service`
  `FailureAction=reboot` (until PWR-13 turns it off) and a WDOG1 bite on a
  kernel hang (kept by D-PWR1).
- **Reboot loops wear the valve controller's flash.** On the tractor every
  boot erases and reprograms H747 bank 1 (~13 s). Any automatic reboot path
  on the tractor waits on PWR-8 (program only on mismatch, or X8OS-3).
- **More boots weaken the tractor's GCM nonces.** The H7 builds nonces from
  `seq`, seconds-since-boot (both restart at 0) and 5 bytes from an xorshift
  seeded once from UID ⊕ millis ⊕ micros. More reboots raise the odds of a
  (key, nonce) collision, which would leak the GHASH key. Tractor arming
  waits on PWR-9 (hardware RNG). The base uses `os.urandom` and is not
  affected.
- **Unplanned cuts need crash-only storage.** A WDOG bite, a forced reboot
  and an L4 cut can land mid-write, including on `/mnt/boot` (FAT, no
  journal, the only u-boot env copy; the tractor already logs "Volume was
  not properly unmounted"). Production arming of the forced-reboot rung and
  of L4 waits on PWR-7.
- **An L072 reset wipes the FCC dwell accountant** (rolling 10 s / 20 s
  windows in RAM). Any reset rung, and the planned IWDG, must be followed by
  a TX hold-off of one dwell window (PWR-10, L072 firmware; SIL only, no air
  time).
- **New attack surfaces** (remote restart opcode, Opta power-request
  register, a root supervisor, a network-controlled supply relay) are listed
  in [CYBERSECURITY_CASE.md §4.4](../CYBERSECURITY_CASE.md).

## 5. Design

Each layer catches what the one above it cannot. None of it may delay or
override the valve watchdogs.

### L1 — Health status for containers (built)

- Compose `healthcheck:` on **mosquitto** (base and tractor) and **web_ui**
  (base). Status only: Docker never restarts a container for being
  unhealthy; the L3 supervisor acts on the status.
- Not on `lora_bridge` (KISS, obsolete, replaced under RS-5.11),
  `audit_tail` (idle by design), or the radio daemons until RS-1.2 gives
  them a heartbeat written from the worker loop. Never as a Dockerfile
  `HEALTHCHECK`: the bench harness runs the same images with bare
  `docker run` and must keep failing loudly (RS-4.8).
- The tractor compose now mounts a repo `mosquitto.conf` with an anonymous
  listener on 1883. `eclipse-mosquitto:2` is a moving tag: 2.0.x images
  shipped a listener-less config (local-only, which refuses the camera), but
  the 2.1.2 image on both boards already listens anonymously on 1883 and
  also serves an unauthenticated `http_api` dashboard on 9883. The repo file
  pins the 1883 behaviour across tag bumps and drops the 9883 listener.
- **Restart side effects:** restarting `web_ui` republishes retained radio
  settings (once switched a live profile on air); restarting a radio daemon
  sends `RESET_REQ`. Neither belongs on the supervisor's restart allowlist.
  Nor does a broker yet: `lora_bridge` (`cmd/estop`, `cmd/control`),
  `audit_tail` and the tractor's `camera_service` subscribe once after
  `connect()` with a clean session and no `on_connect`, so after a broker
  restart they reconnect with no subscriptions and E-stop forwarding, the
  audit trail and the camera's keyframe requests stop with nothing logged.
  A broker goes on the allowlist only after they resubscribe in
  `on_connect` or are retired (TODO PWR-3). The supervisor's example
  config ships with no container listed or one uncomment away.
- Blocked for production on a deploy path: RS-5.11 (base) and a scripted
  install of the tractor compose app.

### L2 — systemd owns the hardware watchdog (built, not installed)

- `firmware/x8_recovery/systemd/system.conf.d/90-lifetrac-watchdog.conf`:
  `RuntimeWatchdogSec=30s`, `RebootWatchdogSec=2min`. If PID 1 or the
  kernel stops scheduling, WDOG1 bites and the PMIC cycles the module.
  `RebootWatchdogSec` covers only the systemd-shutdown phase; `imx2_wdt`
  re-arms to 128 s at device shutdown.
- `systemd/reboot.target.d/90-lifetrac-timeout.conf`: `JobTimeoutSec=3min`,
  `JobTimeoutAction=reboot-force`, so a shutdown stuck on a D-state process
  (wedged camera USB) cannot hold a reboot for upstream's 30 min.
- `install_watchdog.sh check|install|remove --role base|tractor|bench`:
  base: install for PWR-1 under the bench opt-in (the only base X8 is a
  bench board); production arming after the PWR-1 shutdown-timing and
  staged-OTA reboot checks, because the 3 min `reboot.target` bound can cut
  `ostree-finalize-staged` short (the installer prints this for
  `--role base`); tractor: **not installed** (D-PWR1: a watchdog reset
  cannot wait for idle, so it could stop the engine mid-work). The installer
  still refuses `--role tractor` without an explicit prerequisites flag
  (PWR-8, PWR-9, PWR-11), kept only for a revisited decision. The tractor
  keeps what it has today: u-boot arms WDOG1 and the kernel pets it, so only
  a kernel hang resets it, and its shutdowns keep upstream's 30 min
  `reboot.target` bound. The supervisor's forced path covers only a reboot
  that stalls before the shutdown stops the supervisor; a stop job that
  hangs after that holds the tractor, with motion held, for up to those
  30 min. Bench needs an explicit opt-in,
  is refused on a board without the bench marker, and is only for the PWR-1
  checks.
- Coexistence: once systemd owns `/dev/watchdog0`, `wdt_pet.sh start` sees
  that and relies on systemd instead of reporting a false failure. The
  runtime timeout becomes 30 s, and the close-without-`'V'` trick
  (SOFT_RESET_INDEX 4.3) should fail with `EBUSY` (inferred from the
  single-open driver, **VERIFY** under PWR-1).
- Follow-ups: `watchdog.open_timeout=` (X8OS-4); decide `HandlePowerKey`
  (a stray PB1 short press is a clean poweroff today: inferred, **VERIFY**
  under PWR-1).

### L3 — `lifetrac-supervisor` (built, disarmed)

POSIX sh (the tractor's host Python lacks `json`, `select`, `socket` and
working `subprocess`), every external command under `timeout`. Source and
details: [`firmware/x8_recovery/`](../firmware/x8_recovery/README.md).

Ladder, cheapest first:

1. restart an **allowlisted** container that is unhealthy for N passes
   (per-boot cap, then it counts as a node fault);
2. `systemctl reboot`, bounded by the L2 `reboot.target` timeout where L2
   is installed (not the tractor: upstream's 30 min there);
3. if the reboot command fails or the supervisor is still alive after a
   timeout: `sysrq` `s`, `u`, `b` (sync, read-only remount, immediate
   reboot), falling back to `systemctl reboot -ff`.

Rung 3 has no switch of its own: arming the supervisor arms it. So the PWR-7
gate on the forced-reboot rung (§4) is enforced by not arming the supervisor
on any production node before PWR-7; the README, unit file and example
config say so.

Removed from the earlier draft, with reasons:

- **L072 NRST** — the H7 owns NRST under the H7↔L072 contract, `gpio163`
  needs a live bridge and is dead on the tractor (RS-5.2), and restarting a
  radio container already sends `RESET_REQ`. L072 recovery belongs to the
  radio owner (H7 or daemon) and RS-4.10's IWDG.
- **H7 NRST** — does not revive the Linux x8h7 drivers, the module reload
  OOPSes the 6.1 kernel, and a reboot resets the H747 anyway.
- **Deliberate kernel crash (`sysrq c`)** — no gain if a reboot is already
  a PMIC cycle (§1), and it skips the sync.

Guards:

| Guard | Rule |
|---|---|
| Arming | Acts only if `/etc/lifetrac/supervisor.armed` exists; otherwise logs `WOULD …` and does nothing. Bench boards never get the file without a GO. |
| Tractor | Reboot rungs off unless `TRACTOR_REBOOT_ENABLED=1` (§4: engine stop, flash wear, nonces). `ROLE` defaults to tractor: only an explicit `ROLE=base` in a config that loads cleanly allows base reboots, so a missing or broken config never turns tractor reboots on. |
| Tractor idle (D-PWR1) | Even with `TRACTOR_REBOOT_ENABLED=1`, a tractor reboots only after the H747 reports a live `idle` (engine stopped, every valve output off) through `/run/lifetrac/tractor_idle`, the supervisor writes a reboot request (`/run/lifetrac-supervisor/tractor_reboot_request`, a root-only directory), and the H747 answers `locked <seq> <id>`: it holds motion off for that request until it resets or the request is withdrawn, so an orderly shutdown that takes minutes cannot end with the tractor moving. Liveness = the H747's `<seq>` token changes while the supervisor watches. The request and the hold are checked once more right before `systemctl reboot`. Not idle, no answer within `TRACTOR_IDLE_WAIT_S` polls, or the hold gone at that last look: the request is withdrawn and the reboot blocked (`reboot_blocked=tractor_not_idle` / `tractor_lock`); the next pass with the fault asks again. Once the reboot is issued the request stays until the reset, whatever happens next, because systemd can still carry out an accepted reboot after the supervisor gives up on it (a person releases it by cancelling the queued reboot and removing the file). A rung stopped at the last look does not count toward the boot-loop cap. The forced path runs only while the hold stands, or when the link service was stopped by the shutdown and its last line shows the hold, and it looks again after its record, before sysrq `u` (which cannot be undone) and, without sysrq, right before `systemctl reboot -ff`; it is refused when it cannot record its reset. Stopping the supervisor withdraws a request with no reset behind it; once the forced path has written sysrq `u`, a stop waits until `b` and `reboot -ff` are sent. Never overridden, OTA probation included. No link service exists yet, so this blocks every tractor reboot until PWR-13. |
| Grace | No reboot before 600 s uptime. |
| Boot loop | Consecutive supervisor reboots persisted; at the cap (default 3) it enters HELD and stops rebooting until `clear-hold`, except during an OTA trial boot (next row). The count resets after 30 min of continuous health. |
| Clock | No wall clock anywhere (no RTC; the tractor clock is not wall time): uptime and `boot_id` only. Heartbeat staleness = mtime unchanged for N seconds of uptime. |
| OTA trial boot | While `upgrade_available=1`, the cap is raised to `bootlimit + 1`, and while u-boot's `bootcount` ≤ `bootlimit` (rollback not due yet) neither HELD nor the cap stops a reboot, so a counter or `held` file carried over from before the update cannot block the rollback. u-boot bounds that loop (its rollback clears `upgrade_available`), and the override also needs `bootcount` to have risen after the supervisor's own probation reboot in the previous boot (guard against a stale env copy). The tractor gate is never overridden. |
| Reason | Writes an atomic `last_action` record before every reboot, rewritten as `reboot_abandoned` when a requested reboot does not happen and will not be forced; on the next start classifies the previous reset as `supervisor:<action>:<reason>` or `external_or_unknown`. |
| Self-liveness | `WatchdogSec=300` with `systemd-notify WATCHDOG=1` after each pass. |
| Never | GPIOs, openocd, x8h7 modules, `stm32h7-program`, `docker.service`, `/dev/ttymxc3`, L072 flash/unprotect tools, `sysrq c`. |

Trigger definitions: `docker info` failing (node fault), allowlisted
container `status != running` or `health == unhealthy`, heartbeat files
stale. Radio and bridge liveness come only from the owning daemon's
heartbeat file, never from a second opener of `/dev/ttymxc3` or from
`/sys/kernel/x8h7_firmware/version` (which times out on healthy boards).

Open items: install path in provisioning; merging with OTA_STRATEGY's
`lifetrac-health` as its probation mode (one service, one set of checks);
publishing the reset reason (base: local broker; tractor: through the H747
once X8OS-3 lands); the H747 side of the tractor idle gate (PWR-13).

### L4 — Independent hard power cycle (design only)

Needed for what L2/L3 cannot fix: carrier +5V/+3V8 devices, a module that
will not boot, a Linux that hangs before L3 starts. It must cut the
carrier's input from outside the X8. Requirements:

1. **Trigger from node health, monitored outside Linux.** The H747's
   `watchdog_counter` advances on its own 20 Hz loop regardless of Linux, so
   it cannot see a Linux hang or a boot failure. L4 needs (a) a
   **supervisor heartbeat** that originates in Linux (from L3, only after a
   healthy pass) and a separate **boot-complete timeout** after every
   power-up, both watched by the L4 device; (b) the H747 counter only as an
   extra "H747 dead" signal. Keep the valve-safety counter independent of
   Linux.
2. **Hardware-bounded off pulse.** A controller that hangs or a shorted SSR
   must not keep the carrier off. The off time is limited by hardware (a
   one-shot or timer relay, or a capacitor-coupled coil drive), and no
   single switch can hold the cut (two switches in series). Flyback diode on
   the coil; carrier-side 12 V fed back to an input to detect a stuck cut.
3. **Contact.** An SPDT relay used on its normally-closed contact, rated for
   low-level loads (gold-clad; the carrier draws only ~0.3–0.5 A, below the
   wetting current of silver power contacts) and for tractor vibration,
   mounted in the isolated enclosure. Not a plain high-side switch, which
   fails off.
4. **Battery.** For autonomous recovery the 18650 must be cut by the same
   independent device (second contact in the cell lead), or the bridge store
   must sit upstream of the contact. BQ24195 ship mode (BATFET off; TI:
   "can be turned on by plugging in adapter") works only for planned cycles
   while Linux is alive, with a guard that re-enables it if the cut never
   comes.
5. **Arming and inhibits.** Disarmed by default (provisioned flag or
   jumper); disarmed while ignition I1 is low, so a parked tractor is never
   cycled; a timed maintenance/flash inhibit (L072 flash and H7 debug hold
   the H747 for 45–165 s); a "planned off" flag set before any deliberate
   shutdown; never during an OTA confirm window.
6. **State machine.** Grace after every power-up and every re-close, at
   least the measured cold boot plus the H747 reflash; back-off between
   cycles; give up after K consecutive cycles without recovery until the
   next key cycle; status reported in telemetry.
7. **Key-off hold.** POWER_MANAGEMENT's self-holding relay must survive an
   H747 reset (an Opta OR leg, or a hardware hold timer); until then L3 and
   L4 refuse to act while I1 is low. For L3 the H747 enforces this by
   reporting busy while I1 is low (PWR-13), so the idle gate blocks.

Where it lives:

- **Recommended: a standalone node-watchdog board** (small MCU, DPDT relay,
  hardware off-time limit, ignition input, maintenance jumper) fed by the
  L3 heartbeat on a carrier GPIO. It keeps power supervision out of the
  safety PLC, sees Linux health directly, and does not wait on X8OS-3 or on
  the Opta latch rework.
- **Alternative: the Opta.** Use EMR R4 (MASTER_PLAN §8.18 "spare", after
  the §8.18 coil rework) or, interim, D1608S channel 7 (0-based; the only
  channel the firmware leaves free; the "4 spare" text in TRACTOR_NODE.md
  and HARDWARE_BOM.md is wrong). It needs latch-exempt registers (0x0007
  request with magic + nonce, a heartbeat register), its own timer outside
  `all_coils_off()` and the boot self-test, a coil feed that the PSR does
  not switch, and X8OS-3 to forward the Linux heartbeat.

### L5 — Remote restart and bench/base power control (design only)

- **"Restart tractor" from the base UI.** Requirements: name the transport
  and put the command inside the GCM envelope (never plaintext 0xFB); bind
  it to the tractor's current boot with a fresh per-boot challenge
  published in authenticated telemetry (the replay window is RAM-only and
  unprimed at boot); share the persistent rate limit with L3; the H747
  checks at execution time that valves are neutral, no E-stop is latched and
  no handheld is the active source; the UI says that motion needs a reset at
  the machine.
- **Base station.** It runs from a 12 V supply through a DC mini-UPS, so a
  mains smart plug cuts nothing. Put a normally-closed contact of a
  network- or USB-controlled DC relay between the UPS output and the Max
  Carrier jack, driven from a host that does not depend on the base X8. No
  18650 and no X8 USB-C on the base, or the cut is not a cold cycle.
- **Bench (Toughbook).** Same DC relay on each board's 12 V feed, plus an
  inline USB VBUS switch with data pass-through on a direct PC port (the
  bench rule is no hub for adb or reflash). Keep the carrier's debug
  micro-USB (J10) unplugged during automated cycles. Leg launchers compare
  `/proc/sys/kernel/random/boot_id` at the start and end of each leg so an
  unattended reboot is caught.

## 6. Bench checks (PWR-1)

None of these asks the L072 to transmit, but each reboots or power-cycles a
board, so each follows the bench power-up procedure: the first boot of each
board after 2026-10-12 starts with the checks in BENCH_BOARDS "Not yet
observed: the next boot"; arm `legs/power_up_guard.sh --wait-power-cycle
--no-park` before every test reboot or cut (it stops `lifetrac-camera` and
`tractor-camera` and restages `/tmp`); park both radios at the end. Each
check needs Dorkmo's OK for bench time.

Read-only first (no reboot): run `capture_board_state.sh`, which now records
the effective watchdog owner, `kernel.panic`, sysrq availability, the PMIC
power-key DT properties and input device, `HandlePowerKey`, and the
u-boot trial-boot variables.

Then, in this order:

- [ ] **Does `systemctl reboot` go through `WDOG_B`?** Free: after the next
  `REVIVE_MODE=reboot` flash, run `wdog_regs.py`. `POR=1` → PMIC path;
  `SFTW=1` → internal reset. Do it on both boards (the base's H7 is not
  reflashed at boot).
- [ ] **Does a reboot reset the L072?** Compare `uptime` in the periodic
  `RFCO_SUMMARY` URC (payload offset 8, every 60 s, no RF needed) before and
  after, and record elapsed time so continued uptime is distinguishable
  from a reset. (FAULT_URC is event-driven and may not repeat.)
- [ ] **WDOG1 bite** (open `/dev/watchdog0` and close it without `'V'`;
  only before L2 is installed): same two readings, plus PMIC `RESETSRC`.
- [ ] **L2 installed** (bench opt-in on the board under test, removed
  afterwards): `systemctl show -p RuntimeWatchdogUSec`, `wdt_pet.sh start`
  prints that it relies on systemd, `wdog_regs.py` shows the 30 s timeout.
- [ ] **Shutdown timing and a staged OTA under L2** (same install): time a
  normal `systemctl reboot` on each board against `reboot.target`'s 3 min
  `JobTimeoutSec`, and do one reboot with a staged deployment to see
  `ostree-finalize-staged` finish inside it. Production arming of L2 where
  OTAs run (the base) waits on this.
- [ ] **PB1 short and long press, PB2, J1-73:** answer from the DT and the
  input device first; press only with an OK.
- [ ] **18650 behaviour:** with a cell fitted, does a 12 V cut keep the X8
  up? Read BQ24195 REG01 mode first.
- [ ] **`systemctl poweroff` / `halt`:** does each board stay off? Time the
  base's restart against 128 s.
- [ ] **Does a module-only reset clear the camera USB-host wedge / Linux
  soft hang** (T2 rows 2026-05-12 and 2026-05-13 manual unplug)? Out of
  scope: the Board 1 SWD wedge, which survived T2 and a T3a reflash.

## 7. Not recommended

- `sysrq c` or any deliberate kernel crash as a recovery rung.
- `kexec` as recovery (no rescue kernel; truly non-POR).
- Restarting `docker.service` in a loop (LmP reboots after 3 failed starts).
- L072 or H7 resets from the Linux supervisor (owned by the H7 / reboot).
- Dockerfile `HEALTHCHECK`, or an autoheal sidecar with the Docker socket.
- A supply relay that must be actively held on to power the node, or a
  high-side switch standing in for a normally-closed contact.
- A mains smart plug upstream of the base's DC UPS.
- Folding power-cycle logic into the 200 ms valve watchdogs.
