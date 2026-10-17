# Reboot and power cycle — X8 + Max Carrier

**Status: research + design proposal, 2026-10-17.** Nothing here has been
run on the bench yet. Items marked **VERIFY** are inferred from datasheets
or schematics and need a bench check before anyone relies on them. Tracked
as TODO.md items **PWR-1..PWR-6** (Phase 1, after X8OS).

Question this answers: *how can we reboot or power-cycle the Portenta X8 and
the Max Carrier, which of those do we already use, and how do we build them
into the system so a wedged node recovers on its own or on command?*

Companions: [recovery/SOFT_RESET_INDEX.md](recovery/SOFT_RESET_INDEX.md)
(every keyboard reset, layer by layer), [recovery/T2_cold_power_cycle.md](recovery/T2_cold_power_cycle.md),
[../POWER_MANAGEMENT.md](../POWER_MANAGEMENT.md) (key-off shutdown and the
self-holding power relay), [../SAFETY_CASE.md](../SAFETY_CASE.md) (valve
watchdog chain).

---

## 1. What a "reset" actually reaches

| Method | i.MX Linux | H747 (M7+M4) | L072 LoRa | Carrier 5 V domain (modem, Coral, USB hub) | Survives an 18650 in the carrier |
|---|---|---|---|---|---|
| `systemctl reboot` / `reboot -f` | warm reset | reflashed + restarted by `stm32h7-program.service` on boot | not reset (**VERIFY**) | no | n/a |
| i.MX WDOG1 timeout (`WDOG_B` → PMIC) | PMIC power cycle | power cycled (**VERIFY**) | power cycled if fed from the X8 3V3 out (**VERIFY**) | no | n/a |
| H7 NRST (`gpio10`, openocd `reset run`) | no | hard reset | no | no | n/a |
| L072 NRST (`gpio163`) | no | no | hard reset | no | n/a |
| Max Carrier PWRON button PB1 (HDC J2-05 `POWER_ON_REQ`) | PMIC power key: off/on per BD71847 config (**VERIFY**) | follows PMIC | follows PMIC | no | n/a |
| Cut carrier 12 V input (barrel / screw terminal) | full cold cycle **only if** no battery and no X8 USB-C | yes | yes | yes | **no** — the BQ24195 switches to the battery |
| Cut 12 V **and** X8 USB-C, no battery | full cold cycle (= T2) | yes | yes | yes | — |

Hardware facts behind the table:

- **X8 PMIC** is a ROHM BD71847. Its `WDOG` input is tied (R3, 0 Ω) to the
  i.MX `WDOG_B` net, its `POR` output drives i.MX `POR_B`, and it has a
  `PWRON` power-key input ([ABX00049 schematics](https://docs.arduino.cc/resources/schematics/ABX00049-schematics.pdf)).
- **WDOG1 is already armed by u-boot** on both bench boards: `WCR=0x773d`,
  60 s, `WDT=1` (assert `WDOG_B`). A timeout power-cycles the module through
  the PMIC, and `WRSR` then reads POR, so the register cannot tell a watchdog
  reboot from a power-on ([wdog_regs.py](../firmware/x8_lora_bootloader_helper/bench_tools/wdog_regs.py),
  [FLASH_RUNBOOK §3](../firmware/x8_lora_bootloader_helper/FLASH_RUNBOOK.md)).
- **Nothing in userspace owns the watchdog.** `RuntimeWatchdogSec=off` in
  `/etc/systemd/system.conf` on both boards (captured in
  `bench-evidence/board_state_2026-10-04/base` and `board_state_2026-10-10/tractor`).
  The kernel's boot keepalive pets it forever, so a hung systemd or a
  userspace lockup never trips it.
- **Max Carrier power** ([ABX00043 datasheet](https://docs.arduino.cc/resources/datasheets/ABX00043-datasheet.pdf),
  [full pinout](https://docs.arduino.cc/resources/pinouts/ABX00043-full-pinout.pdf)):
  jack input 6–36 V; an 18650 cell on J16/J18 through a BQ24195 charger, jack
  has priority, battery takes over when the jack drops. **PB1 "PWRON"** goes
  to HDC J2-05 (`POWER_ON_REQ`). **HDC J1-73 `RESET`** is driven by the
  carrier's on-board debugger ("JTAG RESET"); on the X8 its target (H7 NRST
  vs. i.MX) is **VERIFY**. No documented carrier-side power-enable signal.
- **X8 module input**: VIN 4.5–5.5 V from the carrier, or its own USB-C. Either
  one keeps the module alive, which is why T2 unplugs both.

## 2. What LifeTrac already uses

| Layer | Mechanism | Where | State |
|---|---|---|---|
| Containers | `restart: unless-stopped` on all compose services | [`docker-compose.yml`](../docker-compose.yml), tractor compose files | In use. No `healthcheck:`; a hung-but-running container is never restarted. Image tx/rx daemons run via bare `docker run` and are unsupervised (TODO RS-4.8). |
| systemd services | `Restart=on-failure` | `firmware/tractor_x8/systemd/lifetrac-*.service` | In use for time, camera, params, logger. No `WatchdogSec=`. |
| Linux reboot | `systemctl reboot` | `full_flash_pipeline.sh` / `run_flash_bench.sh` `REVIVE_MODE=reboot` | In use on the bench; boards back in 12–24 s. |
| i.MX hardware watchdog | u-boot arms WDOG1, kernel pets; `wdt_pet.sh` holds it during L072 flashes and hands it back with magic close `'V'` | `firmware/x8_lora_bootloader_helper/wdt_pet.sh` | In use on the bench only. Proven 2026-09-12 that closing without `'V'` power-cycles the board 60 s later. |
| H7 reset | `gpio10` NRST pulse, openocd `reset run` | `pulse_h7_nrst.sh`, `revive_bridge.sh` | Bench scripts only. |
| L072 reset | `gpio163` NRST, openocd BOOT0/NRST, `AT+REBOOT` | `revive_bridge.sh`, flash pipeline | Bench scripts only. L072 IWDG designed but never started (TODO RS-4.10). |
| Valve safety | M4 200 ms software watchdog → PSR relay; Opta 200 ms Modbus watchdog | `tractor_h7_m4`, `tractor_opta` | Designed. These make valves safe; they do **not** restart anything. |
| Full power cycle | Unplug 12 V + USB-C by hand (T2) | [T2_cold_power_cycle.md](recovery/T2_cold_power_cycle.md) | Manual only. TODO bench idea (2026-08-22): smart plug / relay on the bench supplies. |
| Key-off power | Self-holding relay held by an H747 output | [POWER_MANAGEMENT.md](../POWER_MANAGEMENT.md) | Design draft, not built. |

Correction to the soft-reset index: rows 1.13 and 4.3 say a watchdog-driven
SoC reset is "not reachable" from userspace. The 2026-09-12 finding above
shows it is: open `/dev/watchdog0`, then close it without `'V'` (or stop
petting), and WDOG1 fires a PMIC power cycle at its timeout.

## 3. Recommended design — five layers, cheapest first

Each layer catches what the one above it cannot. All of it stays out of the
valve-safety path: the M4 and Opta watchdogs make the hydraulics safe within
200 ms no matter what this design does, and nothing here may wait on or
override them.

### L1 — Service self-healing (software, both nodes)

- Add `healthcheck:` to the radio, bridge and web containers, and an
  autoheal policy (a small sidecar or the L3 supervisor) that restarts
  containers reported `unhealthy`. Promote the image tx/rx daemons to compose
  services per RS-4.8, after the RS-1.2 heartbeat counter lands.
- `WatchdogSec=` + `sd_notify` on the LifeTrac systemd units that can hang
  rather than exit.

### L2 — Kernel/PID 1 hang → hardware watchdog (software config, both nodes)

- Set `RuntimeWatchdogSec=30s` and `RebootWatchdogSec=2min` in a
  `/etc/systemd/system.conf.d/lifetrac-watchdog.conf` drop-in. systemd then
  owns `/dev/watchdog0`; if PID 1 or the kernel stops scheduling, WDOG1 fires
  and the PMIC power-cycles the module. This is the only layer that recovers
  a frozen Linux with no operator.
- **Bench impact:** `wdt_pet.sh start` will get `EBUSY` once systemd owns the
  device. The flash pipeline must detect that and skip its own petter
  (systemd already pets every 15 s). Land the drop-in only together with that
  script change.
- Ship it through the OSTree/LmP image (X8OS-4) or a provisioning step; on
  the bench, provision with `provision_bench_board.sh`.

### L3 — `lifetrac-supervisor`: health checks with an escalation ladder (software, both nodes)

A small systemd service (host, not a container, so it survives Docker
trouble) that runs the HC-02-style checks every few seconds and escalates
only as far as needed:

1. restart the failing container or unit;
2. L072 NRST (`gpio163`) if the radio stops answering;
3. H7 NRST (`gpio10` / openocd `reset run`) if the x8h7 bridge is dead;
4. `systemctl reboot`;
5. **deliberate PMIC power cycle**: `echo c > /proc/sysrq-trigger`. With L2
   in place, the crashed kernel stops petting and WDOG1 power-cycles the
   module within `RuntimeWatchdogSec`. (Writing `/proc/sysrq-trigger` as root
   works whatever `kernel.sysrq` is set to.) This is the soft path to a POR,
   which a plain reboot is not.

Guards: a reboot counter in persistent storage (`/var/lib/lifetrac`), at most
N escalations to step 4/5 per hour, then stop escalating and stay up for
diagnosis (no boot loops). Log every action with a reason code before acting,
fsync'd, and publish the last reset reason in telemetry after boot. Default
**disarmed on the bench** (same gating rule as `power_mgmt_enabled` in
POWER_MANAGEMENT.md), so bench sessions never get surprise reboots.

### L4 — Independent hard power cycle of the whole node (hardware, tractor)

Needed for the cases L2/L3 cannot fix: the carrier 5 V domain (modem,
Coral, USB hub), a wedge that survives a module-only PMIC cycle, or a
module that will not boot. Requires something outside the X8 to cut the
carrier's input.

- **Tractor: Opta-driven interrupt relay.** The Opta is powered separately,
  already sees the H747's `watchdog_counter` (0x0004), and can see Modbus
  silence. Put a **normally-closed** relay contact (automotive 12 V relay, or
  a high-side load switch) in series with the Max Carrier 12 V feed, driven by
  one of the 4 spare D1608S SSR channels ([TRACTOR_NODE.md](../TRACTOR_NODE.md)). Opta logic: if the H747 counter has not moved
  for a long window (proposal 120 s, never the 200 ms valve window), and not
  in the first 180 s after Opta boot, open the contact for 15 s, then close
  it; at most 3 cycles per hour. Also expose a Modbus register so the H747
  can *request* a cycle (from the L3 supervisor after step 5 fails, or from an
  operator command, below).
  Normally-closed means a dead Opta leaves the carrier powered, so this can
  only ever add a recovery, never cause an outage by failing.
- **Battery interaction:** with the 18650 UPS cell fitted, cutting 12 V does
  nothing, because the BQ24195 runs the carrier from the cell. Options:
  (a) cut on the battery side too (second contact in the cell lead);
  (b) before the cut, have Linux put the BQ24195 in ship mode (BATFET off over
  I²C) so the cell disconnects, then the 12 V cut drops everything (**VERIFY**
  the BATFET re-enables when input returns); (c) replace the cell with the
  supercap bridge POWER_MANAGEMENT.md already prefers, sized so it drains
  within the 15 s off window. Pick together with the POWER_MANAGEMENT battery
  decision.
- **Fit with POWER_MANAGEMENT.md's self-holding relay:** that relay cannot
  power-cycle while the key is on, because the ignition leg re-energizes the
  coil at once. The interrupt contact therefore sits downstream of it, in the
  carrier feed only. The H747 hold output stays the owner of key-off.
- **Alternative if no Opta channel is free:** a standalone watchdog timer
  board (heartbeat input from a carrier GPIO, timed NC relay output).

### L5 — Remote command (both nodes)

- **Operator "Restart tractor" button** on the base web UI → an
  authenticated LoRa command → H747 → either `SHUTDOWN_REQ`-style clean
  reboot (L3 step 4) or a power-cycle request to the Opta (L4). Must wait for
  the AEAD work (RS-8); an unauthenticated reboot opcode is a denial-of-service
  hole (add to CYBERSECURITY_CASE.md). Refused while the tractor reports
  motion or valves active.
- **Base station** (mains + UPS, reachable by ssh): L1–L3 as above; for a hard
  cycle, a network smart plug or outlet-switched UPS on the base's 12 V supply,
  on a network path that does not depend on the X8.
- **Bench (Toughbook):** put both bench boards' 12 V supplies on a smart plug
  or USB relay **and** the X8 USB-C cables on a hub with per-port power
  switching (Windows CLI support needed, so check the hub before buying).
  This turns T2 into a command for the Toughbook agent. 18650s stay out of
  bench carriers, or T2 is not a cold cycle.

## 4. Bench checks before building (no RF transmit needed)

These touch only power and resets; none keys the radio. Each still needs
Dorkmo's OK for bench time.

- [ ] Does `systemctl reboot` reset the L072? Compare L072 `uptime_ms`
  (FAULT_URC offset 20) before and after.
- [ ] Does a WDOG1 timeout (close `/dev/watchdog0` without `'V'`) also reset
  the L072 and the H7? Same `uptime_ms` check; H7 via its boot banner.
- [ ] Does `echo c > /proc/sysrq-trigger` with `RuntimeWatchdogSec=30s`
  power-cycle the board in ~30 s and come back clean?
- [ ] What does a short and a long press of Max Carrier PB1 do on the X8
  (BD71847 PWRON config)? What does HDC J1-73 `RESET` reset on the X8?
- [ ] With an 18650 fitted, confirm a 12 V cut keeps the board up, and test
  the BQ24195 ship-mode path.
- [ ] Does any module-only reset (reboot, WDOG1, sysrq) clear the
  camera/IOMUXC class of wedge (T2 history), or is the L4 input cut required?

## 5. Not recommended

- `kexec` as a recovery path: same non-POR limits as reboot, no known-good
  rescue kernel (SOFT_RESET_INDEX 4.2).
- Folding power-cycle logic into the 200 ms valve watchdogs. Different time
  scale and different failure direction; keep them separate.
- Powering the carrier through a normally-open relay that must be actively
  held by the supervisor: a supervisor fault would then take the node down.
