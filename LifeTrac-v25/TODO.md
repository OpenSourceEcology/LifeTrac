# LifeTrac v25 — TODO

> **⚡ Controller architecture update (2026):** The primary controller design
> is now the three-tier Portenta Max Carrier + MKR WAN 1310 system documented
> in [DESIGN-CONTROLLER/ARCHITECTURE.md](DESIGN-CONTROLLER/ARCHITECTURE.md).
> All hardware-purchase, firmware, and bring-up tasks for that design live in
> [DESIGN-CONTROLLER/TODO.md](DESIGN-CONTROLLER/TODO.md).
>
> The Opta / ESP32 / Raspberry Pi prototype code referenced below has been
> moved to
> [DESIGN-CONTROLLER/RESEARCH-CONTROLLER/](DESIGN-CONTROLLER/RESEARCH-CONTROLLER/)
> (see its [README](DESIGN-CONTROLLER/RESEARCH-CONTROLLER/README.md)).
> The safety bug-fixes listed in this file are still worth completing on
> the prototype code if it gets used for any further bench tests, but the
> equivalent safety logic must be implemented from the start in the new
> Portenta firmware (tracked in [DESIGN-CONTROLLER/TODO.md § Phase 4](DESIGN-CONTROLLER/TODO.md#phase-4--tractor-firmware)).
> Path links below refer to the historical locations; prepend
> `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/` for the current location of any
> Opta / ESP32 / `raspberry_pi_web_controller` files.

## 2026-04-28 controller code-review implementation plan

Four code reviews of the DESIGN-CONTROLLER stack (Claude Opus 4.7, GitHub
Copilot v1.0, GPT-5.3-Codex v1.0, Gemini 3.1 Pro) have been merged into a
single actionable plan with stable IDs (`IP-001` … `IP-309`) and severity
tags. **All work tracked there:**

➡️ **[AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md)**

**Test inventory & per-IP traceability:** the canonical index of every
SIL test, every Arduino compile gate, every HIL bench item, and which
IP each one verifies lives in
➡️ **[MASTER_TEST_PROGRAM.md](MASTER_TEST_PROGRAM.md)** —
update it in the same PR as any test/code change (see §7 of that file
for the protocol).

---

## Pre-field-deployment checklist (open items as of 2026-05-04)

> **Purpose.** A single consolidated view of what still has to happen
> before a real LifeTrac v25 can be powered on at a real work site.
> Every line links to the authoritative phase plan in
> [DESIGN-CONTROLLER/TODO.md](DESIGN-CONTROLLER/TODO.md) (or the
> structural / hydraulic equivalents). **All software work that does
> not need bench hardware is landed** (810 SIL tests / 60 files); what
> remains is **hardware, integration, and field-validation work that
> requires physical parts**.
>
> Status legend: 🟥 = blocker (must finish before Phase 8 field tests)
> · 🟨 = required for Phase 9 release · 🟩 = stretch / nice-to-have.

### A. Hardware procurement (lead-time blocker — start first)

🟥 Production procurement remains open. Two bench Portenta X8 + Max
Carrier stacks are on hand for bring-up, but the rest of the tractor,
base, handheld, hydraulic, and dev-gear BOM still blocks full HIL and
field testing.

- [ ] 🟥 **Tractor node** — Portenta Max Carrier, Portenta X8, LoRa +
  cellular antennas, NEO-M9N GPS, IP65 enclosure, Opta WiFi + D1608S
  + A0602 expansions, hydraulic pressure sensors, engine-kill relay,
  Phoenix PSR safety relay, USB UVC webcam + MCP2221A + BNO086 IMU.
  Full BOM in
  [DESIGN-CONTROLLER/TODO.md § Phase 0 / Tractor node hardware](DESIGN-CONTROLLER/TODO.md#phase-0--hardware-procurement--shop-setup).
- [ ] 🟥 **Base station** — second Max Carrier + X8, 8 dBi mast
  antenna, LMR-400 coax, lightning arrestor, mast + ground rod,
  indoor PSU + UPS. Coral Mini PCIe pending the Phase 1 validation
  spike. See [DESIGN-CONTROLLER/TODO.md § Base station hardware](DESIGN-CONTROLLER/TODO.md#base-station-hardware).
- [ ] 🟥 **Handheld** — MKR WAN 1310, dual joysticks, latching E-stop,
  SSD1306 OLED, IP54 enclosure, custom joystick PCB (KiCad design +
  fab). [DESIGN-CONTROLLER/TODO.md § Handheld hardware](DESIGN-CONTROLLER/TODO.md#handheld-hardware).
- [ ] 🟨 **Spares** — 2× of each major board + antennas + joysticks +
  OLEDs. Cuts the bring-up loop in half if anything DOAs.
- [ ] 🟨 **Dev gear** — RTL-SDR, Saleae Logic 8 (or clone), bench
  PSU, 50 Ω SMA dummy loads, spectrum-analyzer rental for FCC EIRP.
- [ ] 🟥 **Mechanical / hydraulic BOM** — frame tube stock, pivot
  pins, lift cylinders, valves, hoses, fittings. Tracked in
  [DESIGN-STRUCTURAL/](DESIGN-STRUCTURAL/) and
  [DESIGN-HYDRAULIC/](DESIGN-HYDRAULIC/). The chassis has to exist
  before the controller can move anything.

### B. Bench bring-up (Phase 1)

🟥 Mostly blocked on Section A. Current bench exception: two Portenta X8
and Max Carrier stacks are available and have produced partial W4-pre
M7 firmware-liveness evidence; the full production procurement list
remains open.

- [ ] 🟥 First-power-on smoke tests for each of the three nodes
  (tractor X8 + Max Carrier H7, base X8 + Max Carrier H7, handheld
  MKR WAN 1310). Verify USB enumeration, serial console, LED blink.
  [DESIGN-CONTROLLER/TODO.md § Phase 1](DESIGN-CONTROLLER/TODO.md#phase-1--bench-bring-up).
  **2026-05-04 partial:** Board 1 (`2D0A1209DABC240B`) and Board 2
  (`2E2C1209DABC240B`) both flashed `tractor_h7` M7 at `0x08040000`,
  reached `loop()`, advanced SRAM4 liveness for roughly 60 s, and had
  CFSR/HFSR = 0. Formal USB-CDC, rail, blink/echo, and stock
  dual-core handshake captures are still open.
- [ ] 🟥 RadioLib + SX1276 (or Murata SiP) hello-world: send a packet
  base ↔ tractor at 1 m, decode RSSI/SNR.
  **2026-05-04 update:** The physical architecture has been definitively confirmed. The Murata `CMWX1ZZABZ-078` on the Max Carrier acts as a standalone AT modem and its raw SPI pins are **not** routed to the Portenta high-density connectors. Raw SPI communication (i.e. `RadioLib` P2P) is physically impossible. All LoRa control MUST be rewritten to use AT commands over a serial port (`Serial3` on H7, or `/dev/ttymxc3` on X8 Linux).
  **2026-05-04 bench probe update:** Live UART probe on both X8 boards (see [AI NOTES/2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md](AI%20NOTES/2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md)) confirmed the modem UART is alive at **19200 8N1**, reset via `gpio163` works, and host→modem TX is acknowledged. **However**, the firmware currently flashed on the Murata does *not* respond to any known AT command set (Hayes / MKRWAN / RAK / Semtech AT_Slave / RUI3 all rejected with `Error when receiving / +ERR_RX`), and a board1↔board2 over-air smoke test produced zero received bytes. Method comparison in [AI NOTES/2026-05-04_LoRa_AT_vs_SPI_Method_Comparison_Copilot_v1_0.md](AI%20NOTES/2026-05-04_LoRa_AT_vs_SPI_Method_Comparison_Copilot_v1_0.md).
  **2026-05-04 Arduino-doc reading update:** Per [Arduino's Max Carrier TTN tutorial §2.1](https://docs.arduino.cc/tutorials/portenta-max-carrier/connecting-to-ttn) and the [official LoRa modem firmware update guide](https://support.arduino.cc/hc/en-us/articles/4405107258130-How-to-update-the-LoRa-modem-firmware), the Max Carrier ships with a **stale stock Murata firmware that must be updated before first use** via the `MKRWANFWUpdate_standalone` example sketch from the `MKRWAN` library, with `#define PORTENTA_CARRIER` added before `#include <MKRWAN.h>`. The sketch reflashes the modem **over UART from the host H7** using the STM32L072 system bootloader — **no SWD required**. The `+ERR_RX` URC observed on our boards matches the documented "stale firmware" failure mode.
  **2026-05-04 Method G analysis update:** Per user question "could we implement [SPI-era features] with custom firmware?" — yes, fully. The Murata SiP's SX1276 SPI bus is internal to the package and directly wired to the on-die STM32L072 (192 KB Flash / 20 KB RAM, Cortex-M0+ @ 32 MHz). Custom firmware on the L072 recovers per-frame FHSS, adaptive SF, three-profile per-frame PHY swap, P0 preempt, AES-GCM, custom 16 B ControlFrame, no-ACK semantics, with sub-millisecond host↔radio latency over a binary COBS UART. Cost: 4–6 weeks of focused L072 firmware work using [`hardwario/lora-modem`](https://github.com/hardwario/lora-modem) (MIT) as the fork point. Brick risk near-zero if BOOT0/NRST control + a UART safe-mode command are designed in. Full analysis in [AI NOTES/2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md](AI%20NOTES/2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md); summary appended as §11 of the comparison note.
  **2026-05-04 Method G commitment update:** Per project lead direction "we are not in a hurry, we want to get the most performance out of LoRa as possible. Lets skip external modems or boards in method E/F, and focus exclusively on method G" — Methods A, B, C, D, E, F are now superseded. Method G (custom firmware on the L072) is committed as the only LoRa path forward. New design folder [DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/) created to hold the plans and analysis: [README](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/README.md), [00 Method-G commitment & decision record](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md), [01 capabilities analysis](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md) (12 R-XX recovered features + 34 N-XX new capabilities including LBT, channel-quality-aware FHSS, deep-sleep handheld, autonomous emergency beacon, signed firmware, A/B field updates), [02 firmware architecture](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) (source layout, bare-metal cooperative scheduler, ~92 KB / ~14 KB resource budget, Make + PlatformIO), [03 bring-up roadmap](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md) (8 phases, gated on capability not calendar), [04 hardware interface & recovery](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) (host UART spec + 5-layer brick recovery).
  Status: Blocked on Phase 0 of the bring-up roadmap — running `MKRWANFWUpdate_standalone` (with `#define PORTENTA_CARRIER`) on Board2 first, then Board1, to qualify the H7→L072 bootloader pipeline that all custom-firmware development will rely on. Bench utilities to verify the post-flash result are checked in under [LifeTrac-v25/tools/](tools/) (`at_probe.sh`, `at_probe2.sh`, `rx_listen.sh`, `tx_burst.sh`). After Phase 0 green, Phase 1 (hello-world + brick-resistance + prove SiP-internal SPI works from our binary) is the next gate. Per [DECISION 00](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md), Methods E/F are dropped — no external SX1276 module to order, no second antenna to plan.
- [ ] 🟥 Coral Mini PCIe **2-day validation spike** — `lspci`
  enumeration, `gasket`/`apex` driver against the X8 Yocto kernel,
  30-min sustained inference without thermal throttle. On failure,
  swap to Coral USB Accelerator; on second failure, ship CPU-only
  per [MASTER_PLAN.md §8.19](DESIGN-CONTROLLER/MASTER_PLAN.md).
- [ ] 🟥 HIL bench rig: 8× LED dummy coils, 8× 1 kΩ trimpot dummy
  pressure transducers, 2× DMM on the Burkert 0–10 V outputs, bench
  PSU with current meter. Documented bring-up procedure in
  [HIL_RUNBOOK.md](DESIGN-CONTROLLER/HIL_RUNBOOK.md).

### C. Firmware that needs hardware to validate (Phases 2–4.5)

🟥 SIL coverage is in place; on-target compile + bench-run is not.

- [ ] 🟥 Compile-gate **all three Arduino sketches** under the
  Arduino CI matrix on real boards (currently CI-only, no hardware
  pinned): tractor M7 + M4, handheld MKR WAN 1310, Opta valve
  controller. See
  [ARDUINO_CI.md](DESIGN-CONTROLLER/ARDUINO_CI.md) and the
  IP-traceability rows still marked as compile-gate-only in
  [MASTER_TEST_PROGRAM.md](MASTER_TEST_PROGRAM.md).
- [ ] 🟥 Wire the **`CMD_LINK_TUNE` reciprocal handler** on the
  handheld and base receivers — currently only the tractor M7 sends.
  See note in
  [DESIGN-CONTROLLER/TODO.md § Phase 2 line 166](DESIGN-CONTROLLER/TODO.md#phase-2--common-firmware-shared-by-all-three-nodes).
- [ ] 🟥 Wire the **`pick_csma_hop()` caller** into the RadioLib
  `scanChannel` TX path on tractor and handheld and the base SPI TX
  path. Helper landed; integration pending.
- [ ] 🟥 Land the **C-side nonce generator mirror** in
  [`firmware/common/lora_proto/crypto_stub.c`](DESIGN-CONTROLLER/firmware/common/lora_proto/crypto_stub.c)
  to match `build_nonce()` in the Python mirror.
- [ ] 🟥 Phase 4.5 **Opta Modbus slave firmware** — bring-up against
  D1608S + A0602 + Burkert 8605, verify register map matches
  [TRACTOR_NODE.md Modbus map](DESIGN-CONTROLLER/TRACTOR_NODE.md#modbus-rtu-register-map-max-carrier--opta).
- [ ] 🟥 Phase 5 **base-station Linux services** — Docker compose
  bring-up on real X8 hardware (Yocto image), end-to-end web UI
  reachable on LAN, audit log writing to disk.

### D. Cross-cutting firmware/security work (still required for Phase 7)

- [ ] 🟥 **Pairing flow + persistent AES-GCM nonce counter in flash**
  — survives reboot to close the post-power-cycle replay window. QR
  bootstrap on tractor X8 OLED / web UI. See
  [DESIGN-CONTROLLER/TODO.md § Cross-cutting / Device pairing](DESIGN-CONTROLLER/TODO.md#device-pairing--key-provisioning).
- [ ] 🟥 **Code-signing pipeline (Ed25519)** for OTA images; X8
  verifies before flashing the H7 / MKR / Opta. See
  [FIRMWARE_UPDATES.md](DESIGN-CONTROLLER/FIRMWARE_UPDATES.md).
- [ ] 🟨 **Time sync chain** — base X8 NTP-over-cellular, LoRa beacon
  every 1 s, tractor X8 disciplines RTC from beacon → GPS PPS →
  free-run fallback.
- [ ] 🟨 **WireGuard / Tailscale tunnel** for remote base-station web
  UI access (no public HTTP surface). Defence-in-depth: HTTPS +
  basic auth on the LAN-only listener.
- [ ] 🟥 **Web-UI ramp-state heatmap painter** (consumes the
  Round-53 K-E2 JSON from
  [base_station/ramp_heatmap.py](DESIGN-CONTROLLER/base_station/ramp_heatmap.py)).
  Data model + 29 SIL tests already shipped; only the JS render
  + a `/diagnostics/heatmap` route remain.

### E. Phase 6 mast install (base station site work)

- [ ] 🟥 Site survey for clear LoS to typical work area.
- [ ] 🟥 Drive ≥ 2.5 m ground rod; erect mast (concrete or guyed);
  mount 8 dBi omni; route LMR-400 in conduit; install lightning
  arrestor at base.
- [ ] 🟥 VNA / NanoVNA: confirm SWR < 2:1 across 902–928 MHz.

### F. Phase 7 integration testing (bench, all-three-nodes)

🟥 All blocked on B + C. Each item below is a discrete pass/fail.

- [ ] 🟥 Bench: all three nodes powered, exchange frames at 1 m.
- [ ] 🟥 Single-source: handheld-only → tractor follows; base-only
  → tractor follows; both active → handheld wins (priority).
- [ ] 🟥 Handover: handheld release → 30 s latch + 500 ms timeout
  → base takes over.
- [ ] 🟥 TAKE CONTROL: physical button on handheld pre-empts an
  active base session immediately.
- [ ] 🟥 Failsafe: power-off the active source mid-frame → tractor
  reaches neutral within 500 ms (M4 watchdog gate).
- [ ] 🟥 Replay attack: capture frame, retransmit later → rejected.
- [ ] 🟥 Tamper: flip a bit in a captured frame, retransmit →
  rejected by AES-GCM tag check.
- [ ] 🟥 Latency: handheld stick → tractor valve, target ≤ 150 ms
  median; base UI stick → valve, ≤ 250 ms median. Saleae trace.

### G. Phase 8 field testing (gates Phase 9 release)

- [ ] 🟥 LoS range from base mast: 1 km / 5 km / 10 km / 15 km.
- [ ] 🟥 Light-foliage range: 1 km / 3 km.
- [ ] 🟥 Handheld range: 100 m / 500 m / 1 km / 2 km.
- [ ] 🟥 Vibration soak: drive over rough ground; no spurious
  failsafes; enclosure intact.
- [ ] 🟥 Cellular fallback: pull LoRa antenna at tractor →
  telemetry continues over cellular.
- [ ] 🟥 Engine-crank brown-out: cold-start engine while controller
  is up; LiPo backup carries through; no MCU reset.
- [ ] 🟥 IP rating: garden-hose spray test of all three enclosures.
- [ ] 🟥 24 h shop soak; then 7-day on-site soak with full event log.

### H. Documentation gaps still to close before Phase 9 release

- [ ] 🟨 [`NON_ARDUINO_BOM.md`](DESIGN-CONTROLLER/NON_ARDUINO_BOM.md)
  — DigiKey / Mouser / L-com / Phoenix / Burkert / McMaster
  consolidated order list. (Stub exists; needs filling.)
- [ ] 🟨 [`CALIBRATION.md`](DESIGN-CONTROLLER/CALIBRATION.md) —
  joystick deadband, flow-valve 0–10 V → GPM curve, pressure-sensor
  zero, GPS antenna offset.
- [ ] 🟨 [`FIELD_SERVICE.md`](DESIGN-CONTROLLER/FIELD_SERVICE.md) —
  diagnostic flowcharts, fuse map, common failure modes,
  spare-parts kit contents.
- [ ] 🟨 [`OPERATIONS_MANUAL.md`](DESIGN-CONTROLLER/OPERATIONS_MANUAL.md)
  — operator-facing power-on, pairing, take-control, E-stop,
  charging the handheld.
- [ ] 🟨 Hookup guide consolidating
  [TRACTOR_NODE.md](DESIGN-CONTROLLER/TRACTOR_NODE.md),
  [BASE_STATION.md](DESIGN-CONTROLLER/BASE_STATION.md),
  [HANDHELD_REMOTE.md](DESIGN-CONTROLLER/HANDHELD_REMOTE.md).

### I. Regulatory + release (Phase 9 — before any public deployment)

- [ ] 🟥 **FCC §15.247 EIRP verification** with spectrum analyzer.
  Targets: handheld +14 dBm, tractor +20 dBm, base +20 dBm + 8 dBi
  = +26.3 dBm EIRP, all under +36 dBm limit.
- [ ] 🟥 **Safety case sign-off** —
  [SAFETY_CASE.md](DESIGN-CONTROLLER/SAFETY_CASE.md) HAZOP-lite,
  ISO 13849 PL=c claim on the E-stop chain, Phoenix PSR wiring as
  the safety function.
- [ ] 🟨 Open-source licence pass: firmware GPLv3, web UI AGPLv3,
  public release tag `controller-v1.0.0`.
- [ ] 🟨 Add controller hardware to the v25 main BOM and update
  [LifeTrac-v25/README.md](README.md).

### J. Mechanical / hydraulic / structural (parallel track)

🟥 The controller is useless without a chassis. These are tracked
outside DESIGN-CONTROLLER but block field deployment equally.

- [ ] 🟥 Frame fabrication —
  [DESIGN-STRUCTURAL/](DESIGN-STRUCTURAL/) and
  [BUILD-STRUCTURE/](BUILD-STRUCTURE/).
- [ ] 🟥 Hydraulic plumbing + valve manifold —
  [DESIGN-HYDRAULIC/](DESIGN-HYDRAULIC/) and
  [BUILD-HYDRAULIC/](BUILD-HYDRAULIC/).
- [ ] 🟥 Tracks / drive sprockets / final drive (UTU-v25 chain
  reuse) — see
  [AI NOTES/2026-02-15_UTU_v25_Track_Chain_Implementation.md](AI%20NOTES/2026-02-15_UTU_v25_Track_Chain_Implementation.md).
- [ ] 🟥 Lift cylinder mount + pivot geometry —
  [AI NOTES/2026-01-25_Pivot_Mount_Assembly.md](AI%20NOTES/2026-01-25_Pivot_Mount_Assembly.md)
  +
  [AI NOTES/2026-01-25_Lift_Cylinder_Parametric_Formula.md](AI%20NOTES/2026-01-25_Lift_Cylinder_Parametric_Formula.md).

---

## Recently completed (running log — newest first)

**Implementation status (2026-04-29, through Round 53):**
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md)
— every plan item achievable without bench hardware is now landed
(Wave 0 8/8, Wave 1 8/8, Wave 2 9/9, Wave 3 9/9). **810 base_station
tests pass / 2 skipped / 2364 subtests / 60 files**, and **every "—"
cell with a natural SIL surface in the
[MASTER_TEST_PROGRAM.md](MASTER_TEST_PROGRAM.md) §5 IP-traceability
table is now closed.** **Round 53** lands K-E2 — web UI ramp-state
heatmap data model. New module
[base_station/ramp_heatmap.py](DESIGN-CONTROLLER/base_station/ramp_heatmap.py)
exposes a 5-state per-tick classifier (``idle`` / ``matched`` /
``reversal`` / ``decay`` / ``mushy``) over (raw, effective) axis
pairs so the operator-visible "why did my stop feel mushy" overlay
can paint cells without recomputing ramp state. Precedence ladder:
``idle`` (both within deadband) > ``reversal`` (opposite-sign while
both active — wins over ``mushy`` because BC-22 brake lag is
*expected*) > ``decay`` (raw idle, effective still asserting) >
``mushy`` (same-sign ``|lag| > mushy_threshold``) > ``matched``
(otherwise). ``DEFAULT_DEADBAND`` mirrors the BC-29 ``ui.axis_deadband``
config default; ``DEFAULT_MUSHY_THRESHOLD = 40`` (~31% of int8 full
scale) is the documented starting point. Public API:
``HeatmapSample``, ``classify_state()``, ``lag()``,
``build_heatmap_row()``, ``build_heatmap()`` — all JSON-serializable
output for the painter. The new
[base_station/tests/test_ramp_heatmap_sil.py](DESIGN-CONTROLLER/base_station/tests/test_ramp_heatmap_sil.py)
adds 29 tests across RH-A..RH-F: state-vocabulary pin, signed lag
math, full classification ladder including precedence + boundary +
threshold validation, row shape, top-level heatmap shape with
stream-order preservation and threshold propagation, and a firmware-
identity gate that pins ``DEFAULT_DEADBAND`` to ``cfg.ui.axis_deadband``.
[ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) flips the K-E2 row to
``✓ LANDED Round 53`` and adds the Round 53 sequencing line. Painter
wiring (writing the JSON into a web view) deferred. **Suite at 810
tests / 2 skipped / 60 files** (781 → 810 from Round 53).
**Round 52** lands BC-29 — configurable axis deadband
deadband (cheap-win K-D3). New schema int leaf
``[ui].axis_deadband`` ∈ ``[0, 32]``, default ``13`` (≈10% of int8
full scale = byte-for-byte identity vs the pre-Round-52 hard-coded
firmware constant). Firmware sources the constant from the codegen-
emitted header: ``static const int8_t AXIS_DEADBAND =
(int8_t)LIFETRAC_UI_AXIS_DEADBAND;``. All four use sites
(``axis_active()``, the per-coil activation block, BC-24 spin-turn
detection, the flow-set-point computation) continue to read the same
compile-time constant so the four call sites stay in sync. The new
[base_station/tests/test_axis_deadband_sil.py](DESIGN-CONTROLLER/base_station/tests/test_axis_deadband_sil.py)
adds 23 tests across DB-A..DB-E: DB-A schema validation (required-
list, in-range 0/1/13/20/32 accepted, out-of-range low/high + wrong
type + float-where-int rejected); DB-B default-value identity
(default toml authors 13, loader yields int 13, codegen emits
``LIFETRAC_UI_AXIS_DEADBAND 13``, ``reload_class = restart_required``);
DB-C override round-trip for 5 / 0 / 32; DB-D firmware tripwires
(BC-29 marker, exact macro init line, no legacy hard-coded literal,
constant still consumed in all four use sites with the precise
patterns); DB-E docs tripwires (schema bounds, default-toml leaf +
doc comment, CAPABILITY_INVENTORY row).
[ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) gains a new BC-29 entry,
adds a K-D3 row to the cheap-wins table marked ``✓ LANDED Round 52``,
and adds the Round 52 sequencing line. **Suite at 781 tests / 2
skipped / 59 files** (758 → 781 from Round 52).
**Round 51** lands BC-28 — operator profile
preset (cheap-win K-D1, minimal version). New schema enum leaf
``[ui].operator_profile`` ∈ ``{normal, gentle, sport}``, default
``"normal"`` (byte-for-byte identity — individual operator-feel
leaves stand as authored). ``"gentle"`` overrides
``ui.confined_space_mode_enabled = true`` AND
``hydraulic.ramp_shape = "scurve"`` for tight-quarters work.
``"sport"`` overrides ``ui.confined_space_mode_enabled = false``,
``hydraulic.ramp_shape = "linear"`` AND ``ui.stick_curve_exponent
= 1.0`` for crisp control. Overrides are applied at config-load time
inside ``build_config.load()`` via a new
``_apply_operator_profile_overrides()`` helper that mutates the
parsed TOML dict in place after schema validation but before the
hydraulic-compatibility cross-check. Both Python consumers and the
codegen-emitted firmware header automatically pick up the post-
override state because both flow through the same loader. Bundles
intentionally only touch operator-feel leaves with ``reload_class
= restart_required``; safety / hydraulic-topology / network leaves
are NEVER overridden by a profile. The new
[base_station/tests/test_operator_profile_sil.py](DESIGN-CONTROLLER/base_station/tests/test_operator_profile_sil.py)
adds 21 tests across OP-A..OP-E: OP-A pure helper math (normal no-op
even with bundle-shaped authored values, gentle / sport flip exactly
their bundle leaves and leave sentinels untouched, unknown profile
defensive no-op, public ``OPERATOR_PROFILE_OVERRIDES`` table pinned);
OP-B loader integration via temp-TOML + ``LIFETRAC_BUILD_CONFIG_PATH``
(default toml yields normal + identity, gentle/sport tomls override
authored values, ``cfg.raw`` reflects post-override state so
``config_sha256`` is consumer-consistent); OP-C codegen integration
(gentle emits ``..._CONFINED_SPACE_MODE_ENABLED 1`` +
``..._RAMP_SHAPE_SCURVE 1`` + profile macro/side-macro; sport emits
``..._CONFINED_SPACE_MODE_ENABLED 0`` + ``..._RAMP_SHAPE_LINEAR 1`` +
``..._STICK_CURVE_EXPONENT 1.0f``; normal default toml is byte-
identity to authored); OP-D schema rejection of unknown enum value;
OP-E source/config tripwires (``BC-28`` marker,
``_apply_operator_profile_overrides`` symbol +
``OPERATOR_PROFILE_OVERRIDES`` table, schema enum,
``operator_profile = "normal"`` line in ``build.default.toml``,
CAPABILITY_INVENTORY row).
[ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) gains a new BC-28 entry,
marks K-D1 cheap-win as ``✓ LANDED Round 51 (minimal)``, and adds
the Round 51 sequencing line. **Suite at 758 tests / 2 skipped /
58 files** (737 → 758 from Round 51).
**Round 50** lands BC-27 — confined-space
mode (cheap-win K-D2, minimal version). New schema leaf
``[ui].confined_space_mode_enabled`` (bool, default ``false`` =
byte-for-byte identity — zero behaviour change unless a build opts
in). When ``true``, ``ramp_duration_ms()`` multiplies its base
ladder result by ``3 / 2`` (integer-exact for every ladder value:
250→375, 500→750, 1000→1500, 2000→3000) so release ramps,
BC-22 reversal-decay ramps, and the K-A4 forced-coordinated track
duration all stretch by 1.5× — the tractor stops more gently in
tight quarters at the cost of a slightly softer feel. Composes
orthogonally with BC-25 stick curve and BC-26 scurve ramp shape
(operators who already opted in keep their preferences). Minimal
version: 1.5× ramp duration only — no curve / flow-cap change
(deferred until real-world testing motivates them). The new
[base_station/tests/test_confined_space_sil.py](DESIGN-CONTROLLER/base_station/tests/test_confined_space_sil.py)
adds 18 tests across CS-A..CS-G: CS-A pure ladder math at
``confined=False/True`` for both track and arm magnitudes plus
sign-absolute handling; CS-B default-identity at ``FourAxisArbiter``;
CS-C end-to-end release ramp from ``lhy=127`` (baseline last-non-zero
~tick 38–40, confined ~tick 58–60, ≥18-tick stretch); CS-D K-A4
asymmetric-magnitude coordination preserved at the deadline tick
(left=127 / right=25 → both 0 at deadline 40 baseline / 60 confined);
CS-E composition with BC-25 ``stick_curve_exponent=2.0`` (curve
compresses 64→32 AND ramp stretches); CS-F composition with BC-26
``ramp_shape="scurve"`` (smoothstep over the longer 3 s ramp); CS-G
firmware/source tripwires (``BC-27`` marker,
``LIFETRAC_UI_CONFINED_SPACE_MODE_ENABLED`` macro consumption,
``base * 3u`` / ``) / 2u`` multiplier expression, schema leaf,
CAPABILITY_INVENTORY row). The ``test_axis_ramp_sil.py`` Python
mirror gained a ``confined_space`` parameter on ``ramp_duration_ms()``
and ``step_axis_ramp()``, plumbed through ``FourAxisArbiter`` to
both per-axis ramps and the K-A4 forced-track-duration computation;
the default ``confined_space=False`` keeps every pre-Round-50 ramp
test byte-identical. [ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md)
gains a new BC-27 entry, marks K-D2 cheap-win as ``✓ LANDED
Round 50 (minimal)``, and adds the Round 50 sequencing line.
**Suite at 737 tests / 2 skipped / 57 files** (719 → 737 from
Round 50).
**Round 49** lands BC-26 — S-curve ramp shape
selector (cheap-win K-A3). New schema leaf
``[hydraulic].ramp_shape`` ∈ ``{linear, scurve}``, default ``linear``
(byte-for-byte identity — zero behaviour change unless a build opts
in). Firmware factors the prior inline interpolation in
``step_axis_ramp()`` into a ``ramp_interpolate(start, elapsed,
duration_ms)`` helper. When the build defines
``LIFETRAC_HYDRAULIC_RAMP_SHAPE_SCURVE`` the helper substitutes a
half-cosine smoothstep ``shape(t) = 0.5 × (1 + cos(π × t))`` whose
derivative is zero at both endpoints, cutting P95 jerk roughly in
half for the same total stop distance. Pinned closed-form quarter
points: smoothstep at ``t=0.25 → ~0.854 × start`` (vs linear ``0.75``),
``t=0.5 → ~0.5 × start`` (matches linear), ``t=0.75 → ~0.146 × start``
(vs linear ``0.25``).
The new
[base_station/tests/test_ramp_shape_sil.py](DESIGN-CONTROLLER/base_station/tests/test_ramp_shape_sil.py)
adds 21 tests across RS-A..RS-G: RS-A linear-default byte-identity to
the legacy truncation formula across positive / negative starts and
quarter / half / three-quarter / endpoint elapsed, plus implicit
default-arg identity; RS-B scurve endpoints (`elapsed=0 → start`,
``elapsed≥duration → 0``, ``duration=0 → 0``, monotonic non-increasing);
RS-C scurve quarter-point closed-form pinning that distinguishes from
linear; RS-D sign preservation + magnitude symmetry within 1 LSB;
RS-E end-to-end through ``FourAxisArbiter(ramp_shape="scurve")``
(release ramp from full forward, scurve strictly higher at the
quarter-point sample, both shapes reach 0 at the K-A4-coordinated
2 s deadline); RS-F default-identity contract for ``FourAxisArbiter()``
release trajectory; RS-G firmware/source tripwires (BC-26 marker,
``ramp_interpolate`` symbol, ``LIFETRAC_HYDRAULIC_RAMP_SHAPE_SCURVE``
macro consumption, helper call site, schema enum, CAPABILITY_INVENTORY
row).
The ``test_axis_ramp_sil.py`` Python mirror gained an
``_ramp_interpolate(start, elapsed, duration_ms, shape)`` helper and
``step_axis_ramp()`` / ``FourAxisArbiter`` now route through it; the
default ``shape="linear"`` keeps every pre-Round-49 ramp test
byte-identical. [ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) gains a
new BC-26 entry, marks K-A3 cheap-win as ``✓ LANDED Round 49``, and
adds the Round 49 sequencing line. **Suite at 719 tests / 2 skipped
/ 56 files** (698 → 719 from Round 49).
**Round 48** lands BC-25 — per-stick response
curve exponent (cheap-win K-A2). New schema leaf
``[ui].stick_curve_exponent`` ∈ ``{1.0, 1.5, 2.0}``, default ``1.0``
(byte-for-byte identity — zero behaviour change unless a build opts in).
Firmware precomputes a 128-entry uint8_t LUT in ``setup()`` from
``LIFETRAC_UI_STICK_CURVE_EXPONENT`` and applies
``effective = sign(x) × LUT[|x|]`` to every raw stick axis (``lhx``,
``lhy``, ``rhx``, ``rhy``) **post-deadband, pre-mixing**. At ``n = 2.0``
a 50%-stick input compresses to ~32 (vs 64 linear) for finer creep
precision; pegged sticks still reach ±127 at every supported exponent.
The new
[base_station/tests/test_stick_curve_sil.py](DESIGN-CONTROLLER/base_station/tests/test_stick_curve_sil.py)
adds 20 tests across SC-A..SC-F: SC-A pure curve math (identity, zero
fixed point, full-stick preserved, square-law at n=2.0, monotonic, sign
preservation, OOB clamping); SC-B default-identity contract for
``FourAxisArbiter()``; SC-C K-A2 motivating low-stick compression; SC-D
top-end authority across exponents; SC-E pre-mixing application
(``lhx=64`` at n=2.0 → tracks ``(32, -32)`` not ``(16, -16)``); SC-F
firmware source / config tripwires (BC-25 marker, ``apply_stick_curve``
symbol, ``init_stick_curve_lut`` setup-time call, four call sites
wrapping ``cf.axis_*``, schema enum, CAPABILITY_INVENTORY row).
The ``test_axis_ramp_sil.py`` Python mirror grew an
``_apply_stick_curve(v, exponent)`` helper and ``FourAxisArbiter``
gained a ``stick_curve_exponent`` ctor parameter (default 1.0 → legacy
fixtures unaffected). [INPUT_MAPPING.md](DESIGN-KINEMATICS/INPUT_MAPPING.md)
§ "Stick curve" rewritten to past tense; pending-list item ticked off.
[ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) gains a new BC-25 entry,
marks K-A2 cheap-win as ``✓ LANDED Round 48``, and adds the Round 48
sequencing line. **Suite at 698 tests / 2 skipped / 55 files** (678 →
698 from Round 48).
**Round 47** is a doc-debt cleanup + SIL traceability round: closes the two ``STUB`` kinematics docs that were
originally scheduled for Round 44.
[INPUT_MAPPING.md](DESIGN-KINEMATICS/INPUT_MAPPING.md) and
[MOTION_PRIMITIVES.md](DESIGN-KINEMATICS/MOTION_PRIMITIVES.md) are now
marked landed, with their pending sections pointing at the deferred
schema leaves (``[ui].stick_curve_exponent``, BUILD_VARIANT_MOTION_MATRIX
promotion). Stale BC-22 "New design" entry in
[ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) corrected to ``✓ LANDED Round
45``. The new
[base_station/tests/test_motion_primitives_sil.py](DESIGN-CONTROLLER/base_station/tests/test_motion_primitives_sil.py)
adds 15 tests (1 skipped — MP-12 float, build-gated by ``hydraulic.spool_type``)
that pin the post-mix logical-axis signature of every primitive
MP-01..MP-12 through ``FourAxisArbiter``: drive forward / reverse,
skid turn, pivot turn (one track exactly 0), spin in place
(opposite-sign tracks + BC-24 full-flow boost), arms raise / lower,
bucket curl / dump, drive + arms (max-flow over arms), turn + bucket
(both pivot+bucket and spin+bucket combinations), and a cross-cutting
sub-deadband test (all four sticks at v=5 → every logical axis 0,
flow_sp 0). The doc-anchored test file means any future regression to
operator-visible motion vocabulary fails loudly with the primitive
number. **Suite at 678 tests / 2 skipped / 54 files** (663 → 678 from
Round 47).
**Round 46** lands BC-23 — preserve-steering proportional scale-down on saturation — plus BC-24 — spin-turn flow
boost. The naive ``clip_to_int8(lhy + lhx)`` per-side clip is replaced
in ``apply_control()`` by ``mix_tracks_preserve_steering(left_intent,
right_intent)``: when either intent magnitude exceeds ±127, BOTH are
scaled by ``127 / max_mag`` so the differential (steering) ratio is
preserved at the cost of throttle authority. Concrete example: ``lhy=120,
lhx=80`` previously clipped to ``(127, 40)`` — differential 43, vs
operator-commanded 80 — now scales to ``(127, 25)`` — differential 51.
BC-24 augments the flow set-point: when both tracks are active in opposite
directions (a pure or near-pure spin-turn), the track contribution becomes
``min(127, |left_track| + |right_track|)`` instead of
``max(|left_track|, |right_track|)``, so the proportional valve doesn't
under-budget flow and stall the spin. Same-sign motion (forward, reverse,
smooth-curve turn) continues to use max. Arms / bucket continue to use
max in both cases since they're independent valve banks. The new
[base_station/tests/test_steering_priority_sil.py](DESIGN-CONTROLLER/base_station/tests/test_steering_priority_sil.py)
adds 24 tests across SP-A..SP-E: SP-A helper math (8 cases incl. boundary
and mixed-sign saturation), SP-B end-to-end steering-ratio invariant
(post-BC-23 diff is no further from operator intent than pre-BC-23
baseline), SP-C spin-turn boost (pure spin reaches 10000 mV; partial spin
uses sum; large spin clamps), SP-D non-spin cases (forward / reverse /
smooth-curve / arms-only must use max), SP-E firmware source tripwires.
The ``test_axis_ramp_sil.py`` Python mirror grew the new
``_mix_tracks_preserve_steering`` helper and ``FourAxisArbiter`` now
applies it. ``test_track_mix_ramp_sil.py`` RT-A / RT-C / RT-D expectations
updated for BC-23 scaling; behavioural invariants (mixed-mode skip, K-A4
coordinated stop, smooth-curve no-step) preserved. Schema-driven
policy selectors (``[hydraulic].steering_priority`` enum,
``[hydraulic].spin_turn_boost_enabled`` bool) deferred to a future round.
[DIFFERENTIAL_MIXING.md](DESIGN-KINEMATICS/DIFFERENTIAL_MIXING.md) and
[FLOW_BUDGETING.md](DESIGN-KINEMATICS/FLOW_BUDGETING.md) promoted from
stub to landed; [ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) marks BC-23
and BC-24 as ``✓ LANDED Round 46``.
**Suite at 663 tests / 1 skipped / 53 files** (639 → 663 from Round 46).
**Round 45** lands BC-22 — reversal brake.
``step_axis_ramp()`` now detects a same-tick sign-flip on a still-
energised axis (``was_active && is_now && opposite signs``), converts
the operator's reversal command into a decay-to-zero ramp using the
standard release ladder, and on completion holds the axis at zero for
``REVERSAL_BRAKE_MS = 100`` ms (= 2 × ``RAMP_TICK_MS``) before allowing
the new direction to pass through. Two new fields on ``struct AxisRamp``
(``reversal_pending`` and ``brake_until_ms``); a decay-shield invariant
ensures the operator's still-held opposite-sign input cannot cancel the
in-flight reversal ramp via the snap-on-activation path — the decay+
settle sequence runs to completion deterministically, which is the
central hydraulic-safety reason for BC-22 (prevents spool slam and the
cavitation/pressure-spike pair documented in
[DESIGN-KINEMATICS/REVERSAL_HANDLING.md](DESIGN-KINEMATICS/REVERSAL_HANDLING.md)).
The new
[base_station/tests/test_reversal_brake_sil.py](DESIGN-CONTROLLER/base_station/tests/test_reversal_brake_sil.py)
adds 21 tests across BR-A..BR-F: BR-A decay phase (no-snap, release-ladder
duration, monotonic non-increasing magnitude, brake-arm at deadline),
BR-B settle window (holds zero through the full window, ignores input
changes, 2-tick width invariant), BR-C post-brake resumption (active
resumes immediately, zero stays zero), BR-D non-reversal cases (same-sign
jump, plain release, sub-deadband flicker, mixed-mode-skip), BR-E arm
reversals share settle window, BR-F firmware source tripwires. The
``test_axis_ramp_sil.py`` Python mirror grew the same two fields and the
shield logic; the W4-05 / W4-06 invariants stay green. K-A1 / K-A2 /
K-A3 ramping refinements deferred (semantics ambiguous against the
current snap-on-activation contract; need an operator-feel design pass).
[REVERSAL_HANDLING.md](DESIGN-KINEMATICS/REVERSAL_HANDLING.md) promoted
from stub to landed; [ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) marks
BC-22 as ``✓ LANDED Round 45``.
**Suite at 639 tests / 1 skipped / 52 files** (618 → 639 from Round 45).
**Round 44** lands BC-21 — mix-then-ramp +
K-A4 coordinated bilateral track stop. The four ramps in
`apply_control()` now operate on **logical** motion axes
(`g_ramp_left_track`, `g_ramp_right_track`, `g_ramp_arms`,
`g_ramp_bucket`) AFTER differential mixing of raw stick channels
(`leftTrack = clip(lhy + lhx, ±127)`, mirror for right). Pre-BC-21,
ramping on raw stick axes meant a smooth-curve turn (forward-pinned
`lhy`, ramping `lhx` in) stepped the right track because the ramp
short-circuit fired on a stick channel that no track ramp watched.
Three independent code reviews flagged this. As an incidental fix the
coil mapping moved from `lhy`-only OR'd pairs (`if lhy>db: coils |=
LF | RF`) to per-side independent (`if leftTrack>db: coils |= LF`),
which also fixes a second pre-BC-21 bug: pure spin-turns
(`lhy=0, lhx=full`) produced ZERO drive coil activation. K-A4 ships
alongside via a new optional `forced_duration_ms` parameter on
`step_axis_ramp`: when both tracks transition active→released in the
same tick, both ramps share a duration computed from the larger
starting magnitude so the tractor doesn't pivot during release. The
new
[base_station/tests/test_track_mix_ramp_sil.py](DESIGN-CONTROLLER/base_station/tests/test_track_mix_ramp_sil.py)
adds 25 tests across RT-A..RT-G: RT-A mixing math (8 cases incl.
saturation), RT-B spin-turn coil activation (3 cases), RT-C
smooth-curve no-step (the central correctness claim), RT-D K-A4
(symmetric, asymmetric same-tick zero-crossing), RT-F single-track
release, RT-G firmware source tripwire (BC-21 / K-A4 markers,
old-globals-removed, per-side coil mapping). Existing
[test_axis_ramp_sil.py](DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py)
W4-05 / W4-06 tests refactored to logical-axis semantics
(behavioural invariants preserved). New top-level folder
[`DESIGN-KINEMATICS/`](DESIGN-KINEMATICS/README.md) created in the
same round as the canonical home for motion-command semantics
(input mapping, mixing, ramp profiles, reversal handling, flow
budgeting, build-variant motion matrix); [ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md)
captures BC-22 / BC-23 / BC-24 and 13 follow-on K-* ideas.
**Suite at 618 tests / 1 skipped / 51 files** (593 → 618 from Round
44). **Round 43** lands BC-19 — hydraulic build-
variant configurability. Three new `[hydraulic]` schema leaves
(`spool_type` enum tandem/float/closed/open, `load_holding` enum
spool_inherent/po_check/counterbalance/none, `valve_settling_ms`
uint 0..250) extend the canonical TOML, the `HydraulicConfig`
dataclass, the schema-driven C-header codegen (which auto-emits 11
new `LIFETRAC_HYDRAULIC_*` macros incl. enum side-flags), and
`CAPABILITY_INVENTORY.md`. A new in-loader cross-leaf validator
`_validate_hydraulic_compatibility` rejects three contradictory
combinations (tandem/closed + load_holding=none; float/open +
load_holding=spool_inherent; float/open + non-zero
valve_settling_ms) at `build_config.load()` time, before any
consumer touches the config. The new
[base_station/tests/test_hydraulic_compatibility_sil.py](DESIGN-CONTROLLER/base_station/tests/test_hydraulic_compatibility_sil.py)
adds 13 tests across BC19_A..BC19_D: BC19_A canonical default
passes the validator; BC19_B four documented reference builds load
(OSE-legacy float+po_check, v25-canonical tandem+inherent, high-
performance open+counterbalance, closed+counterbalance redundant);
BC19_C six invalid pairings rejected (each with diagnostic naming
the offending leaves); BC19_D source tripwire pins the validator
call site so a refactor cannot silently disable the gate. The
existing variant-matrix / loader / codegen SIL gates (BC-07 / BC-01
/ BC-03) exercise the three new leaves at canonical values via
their schema-driven sweeps without per-leaf edits. BUILD_CONFIG.md
gets a new "Common pitfalls" entry pointing to the validator hook
for future cross-leaf rules. **Suite at 593 tests / 1 skipped / 50
files** (581 → 593 from Round 43). **Round 42** lands BC-15 — a machine-readable
JSON form for the BC-10 ``lifetrac-config diff`` subcommand so non-
Python consumers (notably ``hil/dispatch.ps1`` and ad-hoc bench-laptop
shell pipelines) can decide whether a candidate config requires a
service restart or a firmware reflash without re-implementing the
schema-aware comparator. ``--format text`` (default) is byte-identical
to the pre-BC-15 output; ``--format json`` emits the canonical sorted-
keys compact form (consistent with ``dump-json``) carrying
``changed`` / ``classes`` / ``worst`` / ``is_empty`` /
``restart_required`` / ``firmware_required``. The new
[base_station/tests/test_config_diff_cli_sil.py](DESIGN-CONTROLLER/base_station/tests/test_config_diff_cli_sil.py)
adds 10 tests across BC15_A..BC15_C pinning the contract: BC15_A text
output unchanged (no-diff, live-class change, explicit ``--format
text`` matches default byte-for-byte); BC15_B JSON shape (no-diff
empty payload, live-only / restart_required / firmware_required
classification, ``worst`` promotion across multiple changes,
canonical sorted-keys compact output); BC15_C argparse rejects
unknown ``--format`` values with exit 2. BUILD_CONFIG.md ``diff`` row
updated; doc-coverage gate auto-validates. **Suite at 581 tests / 1
skipped / 49 files** (571 → 581 from Round 42). **Round 41** lands
BC-12C — the bench-side
CLI half of BC-12 — by composing Round 39's ``boot_self_test``
comparator with Round 37's ``lifetrac-config`` CLI. The new
``self-test`` subcommand takes a TOML config and a JSON-encoded
``HardwareInventory`` (whatever the operator captured from the M4 on
the bench), runs ``run_self_test`` in-process, and prints a structured
pass/fail report as text (default) or JSON. Exit code is non-zero on
any ``error`` finding (matches the boot-time gate); ``warning``-only
findings keep exit code 0. The new
[base_station/tests/test_config_self_test_cli_sil.py](DESIGN-CONTROLLER/base_station/tests/test_config_self_test_cli_sil.py)
adds 10 tests across BC12C_A..BC12C_E pinning the contract: BC12C_A
matching inventory passes (text + JSON formats); BC12C_B
error-mismatches exit non-zero with stable codes (AXIS_COUNT_TRACK,
AXIS_COUNT_ARM); BC12C_C warning-only mismatches (camera count) keep
exit 0; BC12C_D bad inputs are rejected with diagnostic stderr
(missing file, non-object JSON, missing fields, unparseable JSON);
BC12C_E source-grep tripwire pins the subparser registration.
BUILD_CONFIG.md synopsis bumped eight→nine subcommands; the BC-08
doc-coverage gate auto-validates. **Round 40** lands BC-14B — a follow-up
to Round 37/BC-14 that closes the gap noted in the Round 37 memo:
the ``config_loaded`` audit emitters in
[base_station/web_ui.py](DESIGN-CONTROLLER/base_station/web_ui.py)
and
[base_station/lora_bridge.py](DESIGN-CONTROLLER/base_station/lora_bridge.py)
now include ``schema_version`` (was: missing, leaving the BC-14
inventory column always empty). The new
[base_station/tests/test_config_loaded_schema_version_sil.py](DESIGN-CONTROLLER/base_station/tests/test_config_loaded_schema_version_sil.py)
adds 5 tests across BC14B_A..BC14B_D pinning the contract: BC14B_A
web_ui's ``_audit_config_loaded`` writes ``schema_version`` as an
``int`` from ``BUILD.schema_version``; BC14B_B the lora_bridge inline
block writes the same field (exercised against the same paho-mocked
pattern as BC4_A); BC14B_C end-to-end — fixture audit lines
that carry ``schema_version`` flow through the BC-14 aggregator and
populate the CSV column, and back-compat is preserved (legacy lines
without the field still aggregate cleanly with empty cells); BC14B_D
source greps both emitter call-sites for ``schema_version=`` so a
future refactor that drops the field fails the gate even if a test
happens to mock around it. Suite at **561 tests / 1 skipped / 47
files** (556 → 561 from Round 40). **Round 39** lands the SIL half of BC-12 —
new
[base_station/boot_self_test.py](DESIGN-CONTROLLER/base_station/boot_self_test.py)
module that compares a loaded ``BuildConfig`` against an injected
``HardwareInventory`` (the M4-side Modbus probe will live in the HIL
half, deferred until bench rig) and produces a structured
``SelfTestReport`` with a stable finding-code catalogue (12 codes:
AXIS_COUNT_TRACK / AXIS_COUNT_ARM / PROPORTIONAL_FLOW_UNAVAILABLE /
PRESSURE_SENSOR_COUNT / IMU_PRESENCE / GPS_PRESENCE /
CAMERA_COUNT_SHORT / CAMERA_COUNT_EXTRA / LORA_REGION_MISMATCH /
AUX_PORT_COUNT / AUX_COUPLER_TYPE / AUX_CASE_DRAIN). Severity model:
safety-significant mismatches (axis counts, proportional flow when
commanded, pressure-sensor count, IMU/GPS presence, LoRa region, aux
plumbing) are ``error`` and set ``ok=False``; cosmetic camera-count
mismatches degrade to ``warning`` and ``ok`` stays True. The
``emit_audit()`` helper writes one ``boot_self_test`` JSONL event per
boot with the canonical ``{component, unit_id, config_sha256, ok,
error_count, warning_count, findings[], started_ts, finished_ts}``
shape so jq / Pandas can pivot on ``code`` without nested-object
handling. The new
[base_station/tests/test_boot_self_test_sil.py](DESIGN-CONTROLLER/base_station/tests/test_boot_self_test_sil.py)
adds 15 tests across 8 classes BC12_A..BC12_H pinning every branch
plus the audit-event shape and the finding-code catalogue (catalogue
renames are caught here so dashboards / runbooks don't break
silently). The HIL half (real M4 Modbus probe + tractor-side
integration) remains deferred. Suite at **556 tests / 1 skipped /
46 files** (541 → 556 from Round 39). **Round 38** lands BC-13 — a new
``$9 Build-config attack surface`` section in
[CYBERSECURITY_CASE.md](DESIGN-CONTROLLER/CYBERSECURITY_CASE.md)
that treats the build-config subsystem (TOML files, installer
bundles, ``lifetrac-config`` CLI, push daemon, ``config_loaded``
audit events, generated firmware header) as an explicit asset class
with safety significance, with four sub-sections: 9.1 Z-CONFIG zone
enumeration, 9.2 STRIDE-per-asset table (9 rows), 9.3
capability-altering leaves are safety-significant (the defence-in-depth
stack that ties together BC-09 BOM cross-reference, ``codegen --check``
drift gate, BC-14 inventory visibility, and the three-watchdog
safe-state), and 9.4 self-reference to the enforcing SIL drift gate.
The new
[base_station/tests/test_cybersecurity_buildconfig_xref_sil.py](DESIGN-CONTROLLER/base_station/tests/test_cybersecurity_buildconfig_xref_sil.py)
adds 5 tests across BC13_A..BC13_E pinning the prose against the rest
of the codebase: BC13_A four-subheading structure; BC13_B every
named ``lifetrac-config <subcommand>`` resolves through the live
_build_parser() (catches typos and stale subcommand names); BC13_C
every relative Markdown link target exists on disk; BC13_D the
STRIDE table covers each Z-CONFIG asset row from $9.1; BC13_E $9
self-references its enforcing test file (so renaming the test fails
the gate). Suite at **541 tests / 1 skipped / 45 files** (536 → 541
from Round 38). **Round 37** lands BC-14 — a new
``lifetrac-config inventory`` subcommand that aggregates
``config_loaded`` audit-log events across one or more ``audit.jsonl``
files into a CSV / Markdown fleet inventory, deduplicated by
``(unit_id, config_sha256)`` so a unit that has been reflashed
shows one row per distinct build it has booted (with first-seen,
last-seen, boot-count, and which components emitted the record).
Directory arguments expand to every ``audit*.jsonl*`` sibling so
log-rotation files (``audit.jsonl.1`` etc.) are picked up
automatically. The new
[base_station/tests/test_config_inventory_sil.py](DESIGN-CONTROLLER/base_station/tests/test_config_inventory_sil.py)
adds 14 tests across 5 classes BC14_A..BC14_E pinning the parser
(BC14_A — only ``config_loaded`` records pass; malformed lines /
non-dict records / missing-field records / missing files all
skipped silently), aggregator (BC14_B — unit/SHA dedup;
reflash produces two rows; sort order), renderers (BC14_C — CSV
header matches ``INVENTORY_FIELDS``; CSV byte-identical across two
runs; Markdown header + separator shape), command integration
(BC14_D — directory glob picks up rotated siblings + skips stray
non-audit files; ``--format markdown`` switches the renderer), and
timestamp formatting (BC14_E — known-epoch ISO-8601 UTC; zero /
negative renders empty). The BC-08 doc gate then auto-validates the
new subcommand row in BUILD_CONFIG.md (synopsis bumped from seven
to eight subcommands). Suite at **536 tests / 1 skipped / 44 files**
(522 → 536 from Round 37). **Round 36** lands BC-09 — a new
`## Capability cross-reference` appendix in
[DESIGN-CONTROLLER/HARDWARE_BOM.md](DESIGN-CONTROLLER/HARDWARE_BOM.md)
that maps every **physical-shape** build-config leaf (the 15 leaves
whose value materially changes which parts you order) to the BOM
section + part(s) that physically realise it. Pure-software / tunable
leaves (`unit_id`, `schema_version`, `*_ramp_seconds`, every
`safety.*` threshold, every `ui.*`, every `net.*`) are deliberately
omitted because they do not change the BOM. The new
[base_station/tests/test_hardware_bom_xref_sil.py](DESIGN-CONTROLLER/base_station/tests/test_hardware_bom_xref_sil.py)
adds 6 tests across 5 classes BC09_A..BC09_E pinning the table to
reality: every appendix capability ID exists in the schema (BC09_A
— no orphan IDs); every entry in the curated
``_PHYSICAL_CAPABILITIES`` set appears in the appendix AND every
curated entry is itself a real schema property (BC09_B —
bidirectional); every row's evidence cell carries a ``Tier 1`` /
``Tier 2`` / ``Tier 3`` token OR a ``planned`` marker so unrealised
capabilities (e.g. the three Round-33 ``aux.*`` leaves) stay self-
documenting (BC09_C); every relative link in the appendix resolves
on disk (BC09_D); the appendix self-references its enforcing test
file so the next reader can find the contract that pins it (BC09_E).
Adding a new physical-shape capability is now a five-place edit
(schema, default TOML, loader dataclass, codegen ``_SECTIONS``, AND
the BOM xref row) instead of four; the gate fails loudly if step
five is forgotten. Suite at **522 tests / 1 skipped / 43 files**
(516 → 522 from Round 36). **Round 35** lands BC-08 — a single
operator-facing onboarding doc
[DESIGN-CONTROLLER/BUILD_CONFIG.md](DESIGN-CONTROLLER/BUILD_CONFIG.md)
that ties every piece of the per-unit build-config system together
for the next person who is handed a LifeTrac v25 to bring up. Nine
short sections cover what a build-config is, where the files live
(schema / default / per-unit / loader / codegen / CLI), the eight
required sections (with a one-line rationale + common-override-
reason for each), the three reload classes (``live`` /
``restart_required`` / ``firmware_required`` with the operator
action each demands), the seven CLI subcommands (``validate`` /
``bundle`` / ``verify`` / ``diff`` / ``push`` / ``codegen`` /
``dump-json``), the deterministic ``config_sha256`` identity, the
``config_loaded`` audit trail, the four common pitfalls (UTF-8
BOMs trip ``tomllib``, section ordering matters for tests, a new
schema leaf needs four edits, ``firmware_required`` changes need a
reflash), and a where-to-read-next pointer table. The new
[base_station/tests/test_build_config_doc_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_doc_sil.py)
adds 5 tests across 5 classes BC08_A..BC08_E pinning the doc to
reality: every required schema section is named (BC08_A); every
``sub.add_parser`` registered subcommand is named AND no orphan
subcommands appear in the synopsis table (BC08_B); every
``reload_class`` value the schema declares is named, and the
schema's vocabulary is exactly the documented three (BC08_C);
every relative-path link target resolves on disk (BC08_D); every
``estop_topology`` enum value appears verbatim (BC08_E —
representative drift gate so a new topology can't be added without
updating the doc). Suite at **516 tests / 1 skipped / 42 files**
(511 → 516 from Round 35). **Round 34** lands BC-06 — the Wave-4
HIL dispatcher is now build-config aware. Per-gate applicability
rules live in the new
[DESIGN-CONTROLLER/hil/gate_applicability.json](DESIGN-CONTROLLER/hil/gate_applicability.json)
file (one source of truth shared between PowerShell and Python
consumers); ``hil/dispatch.ps1`` gains ``-ConfigPath`` and
``-NoApplicability`` parameters and shells out to a new
``lifetrac-config dump-json`` subcommand to load the validated
config as canonical JSON, then evaluates the rules table to flag
gates the active fleet shape can't exercise as ``N/A`` instead of
``NOT-STARTED``. The recommended-next-gate line skips ``N/A`` rows.
The rules cover every Wave-4 gate (BC06_A enforces parity between
the rules file and ``$Script:GateTargets``): radio gates W4-01 /
W4-02 are N/A when ``comm.handheld_present == false``; W4-05 is
N/A when ``hydraulic.proportional_flow == false``; W4-06 is N/A
when ``hydraulic.arm_axis_count == 0``; W4-08 is N/A when
``cameras.count == 0``; the rest always apply. The new
[base_station/tests/test_hil_dispatch_applicability_sil.py](DESIGN-CONTROLLER/base_station/tests/test_hil_dispatch_applicability_sil.py)
adds 13 tests across 7 classes covering rules-file shape +
dispatcher-vs-rules parity (BC06_A), canonical default fully
applicable (BC06_B), four representative N/A scenarios (BC06_C
through BC06_F), and ``dump-json`` determinism + payload contract
(BC06_G). The applicability evaluator is a pure-Python translation
of the four ops the rules file declares (``eq`` / ``gt`` / ``gte`` /
``truthy``) so the SIL gate exercises the rules without ever
subprocessing ``powershell.exe`` from CI; the PowerShell evaluator
in ``dispatch.ps1`` is the trivial mechanical mirror of the same
vocabulary. Suite at **511 tests / 1 skipped / 41 files**
(498 → 511 from Round 34). **Round 33** lands BC-11 — first-class auxiliary
attachment ports. The schema gains an ``[aux]`` top-level section
(``port_count`` 0..2, ``coupler_type`` ``iso_5675``/``flat_face``/``none``,
``case_drain_present``); every leaf carries
``reload_class = restart_required`` because aux ports are a hardware
capability that requires re-init of the M4 PWM channels and the
attachment-permit gate. The canonical default is the conservative
shape (``port_count = 0``, ``coupler_type = "none"``,
``case_drain_present = false``) so existing v25 builds without aux
plumbing keep getting the safe answer; per-unit ``build.<unit_id>.toml``
files can opt in. The Round 31 codegen walks ``aux`` automatically
(now in ``_SECTIONS``) and emits ``LIFETRAC_AUX_PORT_COUNT``,
``LIFETRAC_AUX_COUPLER_TYPE`` (with enum side-macros
``LIFETRAC_AUX_COUPLER_TYPE_NONE`` etc.), and
``LIFETRAC_AUX_CASE_DRAIN_PRESENT`` macros so M4 firmware can
``#if LIFETRAC_AUX_PORT_COUNT > 0`` to gate aux-PWM init. The on-disk
[firmware/common/lifetrac_build_config.h](DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h)
is regenerated; new SHA ``f961d9fd924b52ec46074c6cd26e6d1efcac035f8993e676f6003beb84b1c280``
replaces the Round 31 ``63a15fcb...``. The new
[base_station/tests/test_build_config_aux_section_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_aux_section_sil.py)
adds 13 tests across BC11_A (schema declares aux + every leaf
``restart_required`` + coupler_type enum includes ``none``), BC11_B
(``AuxConfig`` dataclass + canonical default conservative + loader
rejects missing section), BC11_C (codegen emits every leaf macro +
enum side-macros), BC11_D (``iter_reload_classes`` covers every aux
leaf + ``diff_reload_classes`` classifies aux changes as
``restart_required``), and BC11_E (committed on-disk header contains
the aux block — the drift gate that fails when an aux schema edit
forgets the regen). [CAPABILITY_INVENTORY.md](DESIGN-CONTROLLER/CAPABILITY_INVENTORY.md)
gains an "Auxiliary attachment ports" section so the schema-vs-doc
parity gate stays green. Suite at **498 tests / 1 skipped /
40 files** (485 → 498 from Round 33). **Round 32** lands BC-07 — the variant-matrix
SIL. Four representative fleet shapes (canonical / no-camera / no-IMU+GPS /
single-axis) are synthesised inline from the canonical default TOML
and round-tripped through the loader, ``web_ui`` module-import
(``_CAMERA_IDS`` filter + ``MAX_CONTROL_SUBSCRIBERS`` substitution),
the Round 31 codegen (header values reflect the variant), and the
BC-10 reload-class diff helper (camera-only diffs classify as
``live``, axis-count diffs bubble to ``restart_required``). The four
variants produce pairwise-distinct ``config_sha256`` values so a
single ``config_loaded`` audit-log line disambiguates the fleet shape
that booted. The new
[base_station/tests/test_build_config_variant_matrix_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_variant_matrix_sil.py)
adds 14 tests across BC07_A (loader accepts every variant; canonical
fixture byte-identical to the default), BC07_B (web_ui consumes the
variant: empty camera table when ``cameras.count == 0``, default
camera table when canonical, ``MAX_CONTROL_SUBSCRIBERS`` reads from
``BUILD.ui`` even on single-axis), BC07_C (codegen for every variant:
present flags zero on no-camera and no-IMU+GPS, model strings persist
so re-enable just flips a bool, single-axis emits the smaller numbers
verbatim with the ``f`` suffix on the float), BC07_D (canonical
self-diff empty; no-camera diff stays inside the cameras section and
bubbles to ``live``; single-axis diff bubbles to ``restart_required``
because ``track_axis_count`` and ``arm_axis_count`` carry that
reload class), and BC07_E (pairwise-distinct SHAs across the four
variants + canonical SHA equals a fresh load of
``build.default.toml``). Suite at **485 tests / 1 skipped /
39 files** (471 → 485 from Round 32). **Round 31** lands BC-03 — firmware codegen.
New [base_station/build_config_codegen.py](DESIGN-CONTROLLER/base_station/build_config_codegen.py)
walks the validated `BuildConfig` and emits a deterministic C header
([firmware/common/lifetrac_build_config.h](DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h))
that the M4/M7 sketches `#include` in place of the hand-edited
`#define` blocks they carry today (existing
`LIFETRAC_M4_WATCHDOG_MS` is preserved as a legacy alias of the
generated `LIFETRAC_SAFETY_M4_WATCHDOG_MS` so the firmware can
migrate without a flag-day rename). Every scalar leaf the JSON Schema
declares becomes a `#define` (~30 macros across hydraulic / safety /
cameras / sensors / comm / ui / net), with type-safe formatting
(`int` → bare literal, `bool` → `1`/`0`, `float` → `2.0f`-suffixed,
`str` → double-quoted) and enum side-macros
(`LIFETRAC_SAFETY_ESTOP_TOPOLOGY_PSR_MONITORED_DUAL` etc.) so
sketches can `#if` against the canonical truth. The header is
ASCII-only, LF-terminated, deterministic (no timestamps in the body),
and carries `LIFETRAC_UNIT_ID` + `LIFETRAC_SCHEMA_VERSION` +
`LIFETRAC_CONFIG_SHA256_HEX` for runtime cross-checks against the
`config_loaded` audit entry. The `tools/lifetrac-config` CLI grows
a `codegen` subcommand with `--check` mode (zero-exit when the
on-disk header matches the canonical emission, non-zero on drift) so
CI catches a schema or default-TOML edit that didn't also regenerate
the firmware header. The new
[base_station/tests/test_build_config_codegen_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_codegen_sil.py)
adds 22 tests across BC03_A (header-shape: guard, identity macros,
ASCII / LF / no trailing whitespace), BC03_B (leaf-parity: every
schema leaf appears as a `#define`), BC03_C (type formatting: int /
bool / float / string / enum side-macros), BC03_D (determinism + the
CI gate that fails when the on-disk firmware header drifts from the
default TOML), BC03_E (legacy aliases resolvable + ordered correctly),
BC03_F (CLI write byte-identical to direct `emit_header`, `--check`
clean / drift / canonical), and BC03_G (source tripwires for the
module + CLI + on-disk header). Suite at **471 tests / 1 skipped /
38 files** (449 → 471 from Round 31). **Round 30** lands BC-05 — the operator-facing
build-config admin form on the base UI. New page at `/config`
([web/config.html](DESIGN-CONTROLLER/base_station/web/config.html))
is a thin client over four PIN-gated sibling routes:
`GET /api/build_config/source` seeds the editor with the verbatim
active TOML body, `POST /api/build_config/preview-diff` schema-
validates a candidate body and returns the would-be reload-class
classification (`live` / `restart_required` / `firmware_required`)
without writing, `POST /api/build_config/upload` schema-validates,
refuses `firmware_required` diffs (re-flash via bench), atomically
rewrites `BUILD.source_path` via `installer_daemon._atomic_write`,
and emits a `config_upload` audit entry; the existing watcher then
picks up the change on its next poll and routes it through the same
BC-10 contract the X8 installer uses (`live` → swap immediately;
`restart_required` → set sticky `restart_pending` flag, surface in
the watcher state). The page itself shows running unit_id, schema
version, SHA, source path, and watcher pill, plus a Preview-Diff
button that lists every changed leaf with its reload class before
the operator commits. New SIL gate
[base_station/tests/test_build_config_admin_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_admin_sil.py)
adds 16 tests across BC05_A (page routing + widget IDs), BC05_B
(source endpoint verbatim + auth gate), BC05_C (preview-diff:
identical / live / restart-required / schema-violation + no-write
tripwire), BC05_D (upload: identical / live atomic-write / schema-
rejected / unit_id-mismatch / firmware-required-rejected + auth
gate), and BC05_E (source + HTML tripwires). Suite at
**449 tests / 1 skipped / 37 files** (433 → 449 from Round 30).
**Round 29b-beta** lands the tractor-side half
of the BC-10 delivery surface and closes the loop opened by 29b-alpha.
New [base_station/installer_daemon.py](DESIGN-CONTROLLER/base_station/installer_daemon.py)
verifies a USB-stick bundle against this X8's identity at three layers
(filename `unit_id`, header `unit_id`, body `unit_id`), schema-validates
the body, classifies the diff against the in-memory current config
(rejecting `firmware_required` outright since a re-flash isn't a
USB-stick operation), and atomically replaces the active TOML via
`os.replace` so a partial write can't leave a half-applied config
behind. Every outcome — `applied` / `rejected` / `noop` /
`deferred` — produces an `InstallResult` that gets serialised to
`lifetrac-config-result.json` next to the bundle on the stick, so an
operator unplugging the stick has a record of what happened.
[base_station/feedback.py](DESIGN-CONTROLLER/base_station/feedback.py)
pins the operator-facing contract: a 21-character OLED line and a
blink pattern keyed off the install status (3 long green = applied,
3 short red = rejected, amber slow = deferred, single short green =
no-op). The pattern table is locked verbatim in the SIL gate so
changing it is a deliberate contract bump.
[base_station/config_watcher.py](DESIGN-CONTROLLER/base_station/config_watcher.py)
is the watch-and-reload helper consumed by both the base `web_ui`
(via the new `/api/build_config/state` endpoint) and the tractor-side
`lora_bridge`: it polls the source-path `mtime`+`size`, reloads on
change, runs `diff_reload_classes` + `evaluate_quiescence` (Round 29
helpers), and emits one of six `WatchEvent` kinds (`noop` /
`applied` / `deferred` / `restart_pending` / `firmware_required` /
`rejected`) so the caller does no decision-making of its own. The
`tools/lifetrac-config` CLI grows a `push` subcommand (path 1: local
copy into a USB-stick mount or LAN drop directory with optional
`--apply` that invokes the daemon directly; path 5: scp+ssh-trigger
plan, dry-run by default, `--execute` to actually run). The
base web UI gains `/api/build_config/state` (PIN-gated, returns
`{unit_id, sha256, schema_version, restart_pending, last_event,
last_event_reason, changed_leaves, worst_reload_class}`) and emits a
`config_watch_event` audit entry on every non-noop poll. The new
[base_station/tests/test_build_config_installer_daemon_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_installer_daemon_sil.py)
adds 30 tests across BC10c_A (daemon: apply / discover / result-file),
BC10c_B (feedback: pinned LED + OLED for every status), BC10c_C
(watcher: noop / applied-when-quiet / deferred-when-busy /
restart-pending / rejected-on-schema-violation + `/api/build_config/state`
integration), and BC10c_D (push: local copy / local --apply / ssh
dry-run + module-export tripwires for `installer_daemon`, `feedback`,
`config_watcher`, the web route, the audit verb, and the CLI
subcommand). The X8 systemd unit + udev rule that wraps
`installer_daemon.process_mount` in a polling loop on USB-stick mount
is a bench operation queued for the next on-tractor checkout. BC-03,
BC-05, BC-06, BC-07, BC-08, BC-09 remain queued. **Round 29b-alpha**
lands the base-side half of
the BC-10 delivery surface: the installer-bundle envelope, a four-
subcommand laptop CLI (`tools/lifetrac-config validate / bundle /
verify / diff`), and a PIN-gated `/config/download` route on the base
web UI that emits a named, SHA-stamped installer file an operator can
drop on a USB stick. Bundle format is plain UTF-8 text with a header
block (`# bundle_version`, `# unit_id`, `# sha256`, `# generator`,
`# created`) terminated by a `# --- body ---` sentinel and the
validated TOML verbatim, so an operator can `cat` the file in the
field and see exactly what's about to be applied; the SHA covers body
bytes only (no chicken-and-egg with the header that carries it) and
the filename `lifetrac-config-<unit_id>-<sha8>.toml` carries the same
`unit_id` + first-8-of-SHA so a USB stick with multiple bundles is
self-disambiguating. The new
[base_station/config_bundle.py](DESIGN-CONTROLLER/base_station/config_bundle.py)
is the single source of truth for the format; both the CLI and the
web route use `make_bundle` / `serialise` / `parse` /
`verify_filename_matches` so byte-for-byte agreement between
"download from base UI" and "build with the CLI" is enforced by
construction. The CLI exits 0 on success and 2 on validation /
verification failure (reserves 1 for unexpected exceptions so CI can
distinguish "config bad" from "tool bad"); `validate` and `verify`
schema-validate the body too, so a malformed TOML can never reach
the X8-side installer (deferred to 29b-beta). The bundle is gated by
[`base_station/tests/test_build_config_installer_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_build_config_installer_sil.py)
(24 tests across 4 classes BC10b_A/B/C/D) which pins the envelope
(round-trip, filename pattern, body-tamper breaks SHA, unsupported
bundle_version refused, bad unit_id pattern refused, filename/header
mismatch refused, missing sentinel refused), the CLI (`validate`
passes canonical / fails out-of-range; `bundle` writes a file whose
on-disk SHA matches the embedded SHA; `verify` accepts bundler output
/ rejects tampered body; `diff` reports identical / classifies live /
classifies restart_required), the download route (401 without
session, 200 with session, returns a parseable bundle with the
documented Content-Disposition filename, emits a `config_download`
audit entry carrying the SHA), and source tripwires (CLI script
exists with the four subcommand handlers; web_ui carries the route
and the `config_download` audit verb; `config_bundle.py` exports the
documented surface). The X8-side installer daemon (USB-stick watch,
filename + unit-id + SHA verify, atomic-rename apply, OLED + LED
feedback, `lifetrac-config-result.json` write-back) and the daemon
watch-and-reload loop in `web_ui` + `lora_bridge` are deferred to
**Round 29b-beta** so this PR stays a clean base-side round; the CLI
and the download route are immediately useful for fleet-onboarding
even without the X8 installer in place. BC-03, BC-05, BC-06, BC-07,
BC-08, BC-09 remain queued. **Round 29** lands the schema + library
half of
BC-10 — the hot-reload contract that future config delivery (USB-cable
CLI, base-UI installer bundle) is built on. Every leaf in
[build_config.schema.json](DESIGN-CONTROLLER/base_station/config/build_config.schema.json)
now carries a `reload_class` annotation (`live`, `restart_required`, or
`firmware_required`); a missing or out-of-enum annotation is an error,
not a default. The loader gains three pure helpers in
[build_config.py](DESIGN-CONTROLLER/base_station/build_config.py):
`iter_reload_classes(schema)` returns the dotted-path → class map;
`diff_reload_classes(old, new)` returns a `ReloadDiff` with `changed`
paths, per-path classes, and the strictest `worst` class so the
installer can decide apply vs defer vs reject in one call;
`evaluate_quiescence(state)` checks the four preconditions any live
reload depends on (parked ≥ 30 s, no `/ws/control` subscribers, M7 TX
queue empty, engine off or idle-low) and returns an operator-facing
reason string when any fails. `firmware_required` is currently
restricted to `schema_version` and `safety.m4_watchdog_ms`; the SIL
gate pins this set so any future demotion or expansion is a deliberate
edit. The bundle is gated by
[`base_station/tests/test_build_config_delivery_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_build_config_delivery_sil.py)
(19 tests across 5 classes BC10_A/B/C/D/E) which pins reload-class
annotations (every leaf, in-enum, firmware set exact, missing/unknown
raises), diff classification (identical → empty; live-only; restart-
required; firmware-required wins over both; undeclared leaf raises),
quiescence (all four preconditions block independently with their own
operator-facing reason; threshold configurable; OK only when all hold),
leaf coverage (no schema/loader drift in either direction), and source
tripwires (loader contains the BC-10 helper names; schema uses the
`reload_class` keyword and every class token). The actual delivery
landing (laptop USB-cable `lifetrac-config push` CLI, base-UI installer
bundle download route, X8-side installer daemon with OLED + LED
feedback, atomic-rename + result-file write-back) is deferred to
**Round 29b** so this PR stays a clean schema/library round; the
contract pinned here is what 29b builds against. BC-03, BC-05, BC-06,
BC-07, BC-08, BC-09 remain queued. **Round 28** lands BC-04 — the first round in
which the BC-XX BuildConfig actually changes runtime behaviour rather
than just sitting on disk. Both
[`web_ui.py`](DESIGN-CONTROLLER/base_station/web_ui.py) and
[`lora_bridge.py`](DESIGN-CONTROLLER/base_station/lora_bridge.py) now
load the active config at boot (env
`LIFETRAC_UNIT_ID` selects the per-unit override) and write a
`config_loaded` audit-log line carrying `unit_id` / `source_path` /
`config_sha256` so post-mortems can cross-reference behaviour against
the exact file that was running. The web UI's `_CAMERA_IDS` table is
filtered against `BUILD.cameras.{front,rear,implement,crop_health}_present`
— a build with `cameras.count == 0` advertises an empty table (UI tile
and `/api/camera/select` API both refuse the absent positions); the
default canonical build (front camera only) advertises just `auto` +
`front`; a fully-loaded variant build advertises all five. The
`MAX_CONTROL_SUBSCRIBERS` constant is now read from
`BUILD.ui.max_control_subscribers` instead of being hard-coded; the
historical default of 4 is the dev-checkout fallback when the loader
fails. Loader failures are non-fatal — the daemons log a warning and
continue with built-in defaults so a dev checkout without a config
file still boots a degraded-but-running console rather than crash-
looping (BC-XX rationale: observe-the-failure first, then BC-05/BC-10
tighten enforcement). The bundle is gated by
[`base_station/tests/test_build_config_consumption_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_build_config_consumption_sil.py)
(11 tests across 5 classes BC4_A/B/C/D/E) which pins the boot audit
contract (both daemons emit `config_loaded` with the `unit_id` /
`source_path` / `config_sha256` field set, and the helper is a no-op
when `BUILD is None`), camera gating (canonical = `auto + front`,
full-loadout = all five, `count == 0` collapses), parameter
substitution (`MAX_CONTROL_SUBSCRIBERS` reads from BuildConfig at
import time, falls back to 4 when missing), graceful degradation
(missing TOML does not break module import; full hard-coded camera
table falls through), and source tripwires (both files greppably
reference the BC-04 audit call). BC-03, BC-05, BC-06, BC-07, BC-08,
BC-09, BC-10 remain queued. **Round 27** opens the BC-XX build-configuration
initiative with BC-01 + BC-02: a markdown
[`CAPABILITY_INVENTORY.md`](DESIGN-CONTROLLER/CAPABILITY_INVENTORY.md)
enumerating every optional / parameterised capability in the v25
BOM (axis count, E-stop topology, camera count, IMU/GPS presence,
LoRa region, etc.); a draft-07 JSON Schema at
[`base_station/config/build_config.schema.json`](DESIGN-CONTROLLER/base_station/config/build_config.schema.json)
that encodes the inventory machine-readably with strict
`additionalProperties: false` on every section; a canonical
[`build.default.toml`](DESIGN-CONTROLLER/base_station/config/build.default.toml)
for the canonical-BOM build; and a stdlib-only loader
[`base_station/build_config.py`](DESIGN-CONTROLLER/base_station/build_config.py)
(`load(unit_id) -> BuildConfig`) with frozen-dataclass nested
sections, fallback chain `LIFETRAC_BUILD_CONFIG_PATH` env →
`build.<unit_id>.toml` → `build.default.toml`, hand-rolled JSON
Schema validator (no third-party deps), and a deterministic
`config_sha256` derived from canonical-JSON of the validated
dict — what every boot will record into the audit log per BC-04.
The whole bundle is gated by
[`base_station/tests/test_build_config_loader_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_build_config_loader_sil.py)
(20 tests across 5 classes BC_A/BC_B/BC_C/BC_D/BC_E) which pins
schema well-formedness (every section is a strict object), loader
fallback-chain precedence (env wins over per-unit wins over default,
missing env path raises), strict validation (unknown top-level /
nested keys, out-of-range integers, wrong types, bad enum tokens,
pattern-violating `unit_id`, missing required section all raise
`BuildConfigError`), deterministic SHA-256 (matches the explicit
canonical-JSON formula, invariant under TOML whitespace + section
re-ordering, changes when any leaf changes), and **inventory
parity** (every schema property has a backticked `id` row in
`CAPABILITY_INVENTORY.md` and vice-versa — no third-place drift
between loader, schema, and doc). BC-03..BC-10 (firmware codegen +
`#if LIFETRAC_HAS_*` guards, web_ui consumption, admin `/config`
route, HIL `N/A` skipping, variant-matrix SIL, onboarding doc, BOM
cross-reference) remain queued. **Round 26** lands the Wave-4 HIL harness
toolchain under
[`DESIGN-CONTROLLER/hil/`](DESIGN-CONTROLLER/hil/): one PowerShell
harness skeleton per W4-XX gate (10 files) that wraps each
[HIL_RUNBOOK.md](DESIGN-CONTROLLER/HIL_RUNBOOK.md) procedure with
the §0 setup checklist, the per-gate prompts, the auto pass/fail
flip on threshold violation, and a uniform JSONL result-line
writer; a shared
[`_common.ps1`](DESIGN-CONTROLLER/hil/_common.ps1) helper exposing
`Write-GateHeader` / `Assert-Section0-Ready` / `Write-HilResult`
/ `New-RunId`; a [`results_schema.json`](DESIGN-CONTROLLER/hil/results_schema.json)
JSON Schema that locks the JSONL contract; and a
[`dispatch.ps1`](DESIGN-CONTROLLER/hil/dispatch.ps1) that reads
every `bench-evidence/W4-XX/results.jsonl`, prints a
status-table (`PASS/Target` per gate), and recommends the next
gate to run. The whole bundle is gated by the new
[`base_station/tests/test_hil_harness_completeness_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_hil_harness_completeness_sil.py)
(13 tests, 4 classes HC_A/HC_B/HC_C/HC_D) which pins file
existence, harness contract (dot-source `_common`, mandatory
`-Operator`, `Write-GateHeader` + `Assert-Section0-Ready` +
`Write-HilResult` + HIL_RUNBOOK back-reference), schema invariants
(W4-01..W4-10 gate-id pattern, PASS/FAIL/SKIP/ABORT enum, 5-key
firmware-SHA bundle), and dispatcher coverage (every gate has a
positive `Target`, no orphans either way). Day-1 with hardware is
now "edit three COM ports in `_common.ps1`, run `pwsh
./dispatch.ps1`, run the recommended harness" rather than "write
the harness from scratch." **Round 25** lands the IP-201 MQTT
retry/backoff fake-clock SIL
[`base_station/tests/test_mqtt_retry_backoff_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_mqtt_retry_backoff_sil.py)
(13 tests across 4 classes). The test reloads `web_ui` per-test with
`paho.mqtt.client.Client` stubbed so the module-import-time
`_connect_mqtt_with_retry()` call succeeds, then patches
`web_ui.mqtt_client.connect` with a scripted side-effect sequence
and replaces `web_ui.time.monotonic` / `web_ui.time.sleep` with a
`_FakeClock` that advances only on `sleep()`. This makes the
entire retry contract deterministic: env-var (`LIFETRAC_MQTT_HOST`)
honoured with `localhost` default; first-try success path observes
zero sleeps; one-retry-then-success observes exactly `[0.5]`; full
six-failure schedule observes `[0.5, 1.0, 2.0, 4.0, 5.0, 5.0]`
(geometric doubling, cap at 5.0); deadline path raises the
underlying exception verbatim once monotonic time crosses
`+30.0` s; late-success-near-deadline does NOT raise spuriously;
the deadline uses `time.monotonic` (NOT wall-clock `time.time`,
which can jump backwards under boot-time NTP slew). A source-grep
tripwire pins the 30 s deadline literal, the 0.5 s initial
backoff, the `min(backoff * 2, 5.0)` cap, the env-var lookup, and
the `(OSError, ConnectionError)` exception filter against future
refactors. **Round 24** lands the IP-102 nonce-seq thread-through SIL
[`base_station/tests/test_nonce_seq_threadthrough_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_nonce_seq_threadthrough_sil.py)
(13 tests across 4 classes). The test pins the bridge invariant that
every `_on_mqtt_message` arm reserves exactly one seq via
`_reserve_tx_seq()`, stamps it into the cleartext header (via
`_restamp_control` for the `cmd/control` arm or `pack_command(seq, ...)`
for `cmd/estop` / `cmd/camera_select` / `cmd/req_keyframe`), and
threads that *same* seq into `_tx(..., nonce_seq=seq)` so the GCM
nonce bytes (`build_nonce` offsets 1..3) match the cleartext header
seq bytes (`pt[3:5]`) byte-for-byte. The acceptance criterion from
the IP-102 spec ("100 control messages — AEAD nonce seq == cleartext
header seq for every frame; no replay-window false-rejects") is
driven verbatim. Negative tests confirm malformed payloads and
invalid camera-id values are rejected BEFORE `_reserve_tx_seq` so
garbage on the LAN side cannot accelerate `NonceStore` wear or
advance the persistent counter. A cross-arm interleave test pins
that all four arms share ONE counter (so two arms cannot reserve
the same value), and a 16-bit wrap test pins `tx_seq` masking so
the GCM nonce's 2-byte seq slot never desynchronises from the
cleartext header at wrap-around. A source-grep tripwire asserts
every `self._tx(` call inside `_on_mqtt_message` carries a
`nonce_seq=` keyword — the original IP-102 bug surface.
**Round 23** lands the Wave-3 polish constants SIL
[`base_station/tests/test_protocol_constants_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_protocol_constants_sil.py)
(12 tests across three IPs): IP-306 pins
`lora_proto.TELEM_MAX_PAYLOAD == 118` against the C-side
`TelemetryFrame.payload[120]` storage and the `payload_len = 0..118`
header comment so future buffer drift trips loudly; IP-303 pins
`REG_AUX_OUTPUTS = 0x0003` on both the Opta slave (`#define`,
`AUX_OUTPUTS_VALID_MASK = 0x0007`, and a `case REG_AUX_OUTPUTS:`
write-handler arm that masks reserved bits so a typo'd bitfield
cannot energise the valve-manifold SSR channels) and on the M7
Modbus master enum so the slot can never become orphaned;
IP-301 lands the one-line `s_btn_change_ms = millis();` anchor
at the end of `handheld_mkr.ino` `setup()` (after `Serial.begin` /
`pinMode` / `radio.begin` / OLED init) so the very first
`read_buttons()` call after boot can never satisfy
`(now - s_btn_change_ms) >= DEBOUNCE_MS` against the
uninitialised-zero default and commit a spurious debounced state.
**Round 22** lands the last two HIL-companion SILs:
W4-08 (camera back-channel keyframe round-trip / IP-103, IP-104)
via
[`base_station/tests/test_keyframe_round_trip_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_keyframe_round_trip_sil.py)
(17 tests) and W4-10 (fleet-key provisioning sanity / IP-008) via
[`base_station/tests/test_fleet_key_provisioning_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_fleet_key_provisioning_sil.py)
(24 tests). The keyframe SIL exercises the `/cmd/req_keyframe` →
`pack_command(seq, CMD_REQ_KEYFRAME)` translation, the on-air
encrypt + KISS encode + KISS decode + decrypt + parse round-trip
(including 0xC0 escape-path coverage), the M7 `send_cmd_to_x8()`
`[X8_CMD_TOPIC, opcode]` framing → KISS UART → `dispatch_back_channel`
keyframe-event firing, and a pure-Python time-on-air model that
proves the LADDER[0] (SF7/BW250) airtime fits the < 100 ms LoRa
leg with margin and the modeled end-to-end (REQ TOA + worst-case
in-flight CTRL TOA + UART KISS @ 115200 + encode-loop wake-up)
fits the < 200 ms HIL acceptance budget. The fleet-key SIL parses
both `handheld_mkr.ino` and `tractor_h7.ino` to assert the
`#ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY` guard exists in `setup()`,
calls `fleet_key_is_zero()`, halts in a forever-loop with
`delay()` inside, renders the canonical handheld OLED text
`"FLEET KEY NOT\nPROVISIONED\nHALT (IP-008)"` byte-for-byte, and
writes `LIFETRAC_ESTOP_MAGIC` (= 0xA5A5A5A5) to
`SHARED->estop_request` on the tractor side so the M4 watchdog
covered by `test_m4_safety_sil.py` drops the safety relay; the
Python half drives `lora_bridge._load_fleet_key()` through every
failure mode (missing env, all-zero hex, all-zero file, wrong
length, invalid hex, unreadable file) and verifies the
`LIFETRAC_ALLOW_UNCONFIGURED_KEY=1` bypass lives ONLY in the
module-level try/except, not inside the loader, so explicit
production calls always fail-closed. **Round 21** added
[`MASTER_TEST_PROGRAM.md`](MASTER_TEST_PROGRAM.md), the
canonical index of all SIL tests, compile gates, and HIL items
with per-IP traceability. **Round 20** lands the W4-07 (boot-PHY first-frame
decode / IP-006) SIL via
[`base_station/tests/test_boot_phy_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_boot_phy_sil.py)
(11 tests). The test parses the three relevant firmware sources
(`handheld_mkr.ino`, `tractor_h7.ino`, `tractor_x8/params_service.py`)
and asserts the `LADDER[3]` tables on the two LoRa nodes are
byte-identical, every `radio.begin(...)` references `LADDER[0]`
symbolically (regression-trapping numeric-literal drift), `LADDER[0]`
matches DECISIONS.md D-A2 (SF7/BW250/CR4-5/bw_code=1), the sync word
is `0x12`, and the Pi-side `control_phy` string agrees. **Round 19** ports the M7 adaptive-SF ladder state
machine (`try_step_ladder()`, `poll_link_ladder()`, the HB ingress
hook, and `apply_phy_rung()`) from
[`firmware/tractor_h7/tractor_h7.ino`](DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
to pure-Python in
[`base_station/tests/test_link_tune_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_link_tune_sil.py)
(16 tests). W4-02 (link-tune walk-down) moves from greenfield bench
pass to verification pass against a documented model: the SF7→SF8→SF9
bad-window walk-down (R-8 hysteresis: 3 bad of 5 s windows), the
SF9→SF8→SF7 good-window walk-back (6 good of 5 s windows), SF9 +
SF7 saturation gates, twice-back-to-back announce invariant
(one TX at OLD PHY, one TX at NEW PHY, same target & reason byte),
500 ms revert deadline with commit / revert / edge-case coverage,
stale-active-source pessimism rule, and the verification-window
gate that prevents the in-flight tune window from being
double-counted as bad. Only the < 1% packet-loss metric still
needs an HIL bench RF attenuator. **Round 18** ports `step_axis_ramp()` and the four-axis
`apply_control` arbitration shape from
[`firmware/tractor_h7/tractor_h7.ino`](DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
to pure-Python in
[`base_station/tests/test_axis_ramp_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py)
(10 tests). W4-05 (proportional valve ramp-out) and W4-06 (mixed-mode
skip) move from greenfield bench passes to verification passes
against a documented model: the track 2/1/0.5 s ladder, arm 1/0.5/0.25 s
ladder, linear-interpolation midpoint, coil-stays-engaged-through-ramp
invariant, flow-setpoint monotonicity, and the four mixed-mode skip
variants (including the subtle one where `other_active` is computed
from *raw* inputs so a stale ramping sibling can't block a fresh
release's ramp/skip decision) are all now CI-gated. **Round 17** flips the last best-effort Arduino compile
gate (`firmware-compile-tractor-h7`) to blocking and adds a parallel
`firmware-compile-tractor-m4` job. The M4 watchdog firmware was
moved out of the M7 sketch folder into its own
[`firmware/tractor_h7_m4/tractor_h7_m4.ino`](DESIGN-CONTROLLER/firmware/tractor_h7_m4/tractor_h7_m4.ino).
A `#if defined(CORE_CM4)` guard inside the original `tractor_m4.cpp`
fixed the M7 `multiple definition of 'setup'` link error, but the M4
build still failed because arduino-cli's library auto-discovery
scans the .ino's `#include` directives regardless of `target_core`,
pulling RadioLib + Modbus into the M4 build where they don't
compile against the M4 variant headers. Two separate sketch folders
is the canonical Portenta-dual-core layout (matches Arduino IDE's
stock `PortentaDualCore` example). M7 builds at 207888 B / 9% flash;
M4 at 87032 B. The Portenta core does not expose a separate
`envie_m4` FQBN — the M4 build uses the `target_core=cm4` board
menu option (`arduino:mbed_portenta:envie_m7:target_core=cm4`).
**Round 16** flips the second of three Arduino compile
gates (`firmware-compile-opta`) from `continue-on-error: true` to
blocking after a local arduino-cli replay exposed two issues: the
.ino base name didn't match its parent folder (renamed
`opta_modbus_slave.ino` → `tractor_opta.ino`) and the
`Arduino_Opta_Blueprint` library (provider of `OptaBlue.h`) was
missing from the workflow's install step. tractor_opta now compiles
clean (171520 B / 8% flash). **Round 15** turned the Arduino compile
gate from `continue-on-error: true` into a real blocking job for
`firmware/handheld_mkr` after a local arduino-cli replay exposed
three independent bugs that had been hiding behind that flag since
IP-007: stale `#include "../common/..."` paths that don't resolve
in arduino-cli's temp build dir; `--build-property build.extra_flags=...`
clobbering the MKR WAN 1310's `-DUSE_BQ24195L_PMIC` (which gates
`LORA_IRQ`); and `crypto_stub.c` requiring an explicit
`-DLIFETRAC_ALLOW_STUB_CRYPTO` opt-in. Both stub (55932 B) and real-crypto
(59940 B) handheld builds are now green locally and in
[`.github/workflows/arduino-ci.yml`](../.github/workflows/arduino-ci.yml).
A reproducible local staging script
[`DESIGN-CONTROLLER/tools/stage_firmware_sketches.ps1`](DESIGN-CONTROLLER/tools/stage_firmware_sketches.ps1)
mirrors the CI staging steps (extended in Round 17 to also stage
shared_mem.h into the new tractor_h7_m4 sketch).
**Round 14** resolves the TR-I cumulative-vs-consecutive
divergence pinned in Round 10: the M7's
`s_modbus_fail_count` in
[`firmware/tractor_h7/tractor_h7.ino`](DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
now resets to 0 on every successful Modbus poll, so the 10-strike
E-stop latch counts *consecutive* failures and matches the W4-04
runbook prose. The SIL test suite was flipped accordingly: the old
`test_fail_count_is_cumulative_not_consecutive` (which pinned the
firmware bug) is replaced by `test_fail_count_is_consecutive` plus
two new pathological-pattern tests
(`test_intermittent_failures_never_latch`,
`test_nine_then_one_then_nine_does_not_latch`) — 1000-cycle
alternating fail/success and 9-1-9 patterns that *would* have tripped
under cumulative semantics but must not under consecutive. This
prevents the "noisy bus is a slow-burn nuisance trip" failure mode
where a single bad poll every few hours would eventually latch the
tractor down. Test count: 198 → 200 (+1 cumulative test removed,
+3 consecutive tests added).
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md)
— every plan item achievable without bench hardware is now landed
(Wave 0 8/8, Wave 1 8/8, Wave 2 9/9, Wave 3 9/9). 198 base_station
tests pass. **Round 13** completes the third and final fragmenter
fuzz: the on-air TileDeltaFrame chunker in
[`base_station/image_pipeline/reassemble.py`](DESIGN-CONTROLLER/base_station/image_pipeline/reassemble.py),
exercised via
[`base_station/tests/test_image_reassembly_fuzz.py`](DESIGN-CONTROLLER/base_station/tests/test_image_reassembly_fuzz.py).
13 tests cover IF-A through IF-L: round-trip identity for arbitrary
frame sizes split N ways, fragment-header invariants (magic /
constant `frag_seq` / monotonic idx / shared `total_minus1`),
bad-magic passthrough that must not disturb an in-flight assembly,
truncated-header-with-magic decode-error path, `frag_idx >= total`
rejection, reordered delivery (20 PRNG trials), duplicate handling
with `2*(N-1)` duplicate-counter check, missing-middle timeout +
GC, two-stream interleave by `frag_seq`, seq-wrap 255 → 0 across an
intervening GC, mid-stream `total` change recovery, and a 200-trial
PRNG-seeded randomised stress with optional shuffle + duplicate.
The system now has property-fuzz coverage on **all three** binary
parsers — IP-108 command frame (operator → tractor),
`telemetry_fragmentation_fuzz` (M7 → base, Round 11), and the
TileDeltaFrame chunker (camera → base, Round 13).
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md)
— every plan item achievable without bench hardware is now landed
(Wave 0 8/8, Wave 1 8/8, Wave 2 9/9, Wave 3 9/9). 185 base_station
tests pass. **Round 12** added a WS subscriber-concurrency stress
suite
([`base_station/tests/test_ws_subscriber_concurrency.py`](DESIGN-CONTROLLER/base_station/tests/test_ws_subscriber_concurrency.py))
that drives `_admit_ws`, `_discard_subscriber`, `_snapshot_subscribers`,
and `_on_mqtt_message` together against a real asyncio loop running on
a background thread — the same topology the production gateway
executes. 6 tests cover: cap honoured under N=4×cap concurrent admits
(WC-A), 200 admit/discard cycles with no leak (WC-B), 20 000 snapshot
iterations under 4-thread churn with zero `RuntimeError: Set changed
size during iteration` (WC-C), 500 `_on_mqtt_message` fan-outs while
3 churners mutate the pool (WC-D), per-pool cap independence (WC-E),
drain-and-refill counter integrity (WC-F), and 4429-on-overflow
close-code verification piggybacked on WC-A (WC-G). This converts
Round 9 §B/§C from "code looks right by inspection" to a CI-enforced
property.
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md)
— every plan item achievable without bench hardware is now landed
(Wave 0 8/8, Wave 1 8/8, Wave 2 9/9, Wave 3 9/9). 179 base_station
tests pass. **Round 11** added the property/fuzz suite for the
telemetry fragmentation path
([`base_station/tests/test_telemetry_fragmentation_fuzz.py`](DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation_fuzz.py)),
symmetric to the IP-108 command-frame fuzz: 15 tests cover round-trip
identity across every length × packable profile, header invariants
(magic, monotonic idx, shared seq/total), 25 ms airtime cap per
fragment, the > 256-fragment oversize rejection, reordered and
duplicated delivery, missing-middle timeout & GC, multi-source/topic
isolation, seq-wrap across GC boundaries, non-magic passthrough,
mid-stream `total` change recovery, and a 200-trial PRNG-seeded
randomised stress. **Round 10** added a SIL model of the Opta Modbus
slave + M7 `apply_control()` writer
([`base_station/tests/test_modbus_slave_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_modbus_slave_sil.py)),
which converts W4-04 (Modbus failure → E-stop) from a HIL-only gate
into a CI verification — 13 tests cover the 200 ms Opta watchdog,
the 10-strike fail-count latch, the fail-closed block shape, the
REG_ARM_ESTOP/external-loop trip paths, the AUX-mask guard, and the
W4-04 < 1 s budget. The suite also pins a documented divergence: the
current firmware fail-count is *cumulative* not *consecutive*, so the
W4-04 runbook prose and the firmware disagree (firmware wins until a
deliberate fix lands). **Round 9** addressed five operational-hardening
findings that surfaced in the parallel GPT-5.3-Codex and Gemini
second-pass reviews: **§B** `/ws/control` admission cap
(`MAX_CONTROL_SUBSCRIBERS=4` + `_admit_ws()` parity with
telemetry/image/state); **§C** `_subscribers_lock` guarding all WS-pool
mutations + `_snapshot_subscribers()` for the MQTT fan-out thread;
**§D** unconditional all-zero fleet-key refusal in both firmware
sketches (was previously gated only on `LIFETRAC_USE_REAL_CRYPTO`,
now requires explicit `LIFETRAC_ALLOW_UNCONFIGURED_KEY` opt-in);
**§E** `AuditLog` promoted to a module-level singleton via
`_get_audit_log()` so PIN failures stop opening a new file handle per
event; **§F** cross-language
`LP_REPLAY_WINDOW_BITS == REPLAY_WINDOW_BITS == 64` invariant test
parsing the C header. Remaining work is the HIL gate set
below — bench validation of W4-01…W4-10 and the **§A** CI compile-gate
flip from best-effort to blocking once the three `arduino-cli compile`
jobs go green on a real Actions runner.

Source reviews:

- [Claude Opus 4.7 (primary, with cross-review addendum)](AI%20NOTES/2026-04-28_Controller_Code_Pipeline_Review_ClaudeOpus4_7_v1_0.md)
- [GitHub Copilot v1.0](AI%20NOTES/2026-04-28_Controller_Code_Pipeline_Review_GitHub_Copilot_v1_0.md)
- [GPT-5.3-Codex v1.0](AI%20NOTES/2026-04-28_Controller_Code_Pipeline_Review_GPT-5.3-Codex_v1_0.md)
- [Gemini 3.1 Pro v1.0](AI%20NOTES/2026-04-28_Controller_Code_Review_Gemini_3.1_Pro_v1.0.md)

Wave 0 (IP-001 … IP-008) is the BLOCKER set — nothing else is testable
until those land. Wave 4 lists the gates that must pass before any wet
hydraulic test.

## Open initiative — Build-configuration / hardware-variant editor (BC-XX)

> **Status:** planning only — not yet scheduled. Captured 2026-04-29
> after the user observed that not every builder will procure every
> BOM line item and the software must not break when an optional
> sensor / camera / IMU / GPS / second-camera is absent or differently
> spec'd. Owner: TBD. Target start: after Wave-4 HIL gates are CLOSED.

### The problem

LifeTrac v25 firmware + base-station + web UI today assume the
canonical [`HARDWARE_BOM.md`](DESIGN-CONTROLLER/HARDWARE_BOM.md) build:
one Coral X8 camera, one IMU on the M7 I²C bus, one GPS on the X8,
the documented mushroom-button E-stop wiring, the four-track / four-arm
hydraulic axis count from
[`MASTER_TEST_PROGRAM.md`](MASTER_TEST_PROGRAM.md) §3. A builder who
cannot source — or chooses to omit — any of those parts today has
three bad options: ship anyway and let the code log errors / crash;
fork the firmware; or hand-edit constants. None of those scale to a
fleet. Specific failure modes already lurking in the codebase:

* `web_ui` rendering a "second camera" tile that always says
  `no signal` because the second X8 is absent.
* M7 logging Modbus failures forever for a non-existent Opta input
  (different E-stop topology that doesn't use the AUX bank).
* IMU-derived telemetry fields (`pitch_deg`, `roll_deg`) emitted as
  `null` and crashing chart code that assumed numbers.
* GPS-stamped audit log lines emitting `lat=0.000, lon=0.000` and
  poisoning any post-run analysis.
* W4-XX HIL gates failing because a builder omitted the sensor under
  test even though their build doesn't need that gate.

### Goals

1. **Fail-soft on absent parts.** Every optional capability must
   declare itself optional in code; UI and logging must tolerate
   `feature_disabled` without raising or rendering broken widgets.
2. **Per-build configuration as data, not code.** Each fleet unit
   has one canonical config file (`config/build.<unit_id>.toml`)
   that selects which BOM-optional features are present and pins
   their parameters (axis count, valve flow ratings, IMU/GPS port,
   camera count, E-stop topology id, …).
3. **Web UI editor.** Operator can view + edit the active config
   from the base-station web UI, with form-level validation against
   a JSON Schema, audit-logged save + reboot, and a one-click
   "export this config" for fleet onboarding.
4. **Compile-time + run-time gating.** Firmware respects the same
   config for `#define`-style feature flags via a generated
   `lifetrac_build_config.h`; bridge / UI honour it at run time.
5. **No regression in existing SIL coverage.** Existing 329 SIL
   tests must continue passing under the *default* (canonical-BOM)
   config; new SIL tests gate the variant matrix.

### Proposed work breakdown (BC-01 … BC-10)

> **Round ordering (post-Round-27):** BC-01 + BC-02 are landed. The
> recommended order from here is **Round 28 = BC-04** (consumers
> first, so the loader actually changes behaviour); **Round 29 =
> BC-10** (delivery + hot-reload contract, ahead of BC-05 because
> BC-05 is essentially a thin web UI on top of BC-10's atomic-write +
> reload machinery); **Round 30 = BC-05**; **Round 31 = BC-03**
> (firmware codegen + reduced-BOM compile gates is the largest piece
> and benefits from CI work landing on its own).

* **BC-01 — Capability inventory.** Walk the BOM, the firmware
  sketches, [`web_ui.py`](DESIGN-CONTROLLER/base_station/web_ui.py),
  the [`lora_bridge`](DESIGN-CONTROLLER/base_station/), and
  [`MASTER_TEST_PROGRAM.md`](MASTER_TEST_PROGRAM.md) §3 to enumerate
  every feature that is *physically optional* or *parameterised*.
  Output: a markdown table of capability id → default → spec range
  → consumer modules. This is the source of truth for the schema.
* **BC-02 — Config schema (JSON Schema + TOML loader).** Define
  `config/schema/build_config.schema.json` with one property per
  capability and per-capability `enum` / `min` / `max` / `default`.
  Land a Python loader
  (`base_station/build_config.py::load(unit_id) -> BuildConfig`)
  with strict-mode validation and a documented fallback chain
  (env override → `config/build.<unit_id>.toml` → `config/build.default.toml`).
* **BC-03 — Firmware codegen.** A pre-build script consumes the
  active TOML and emits
  `firmware/common/lifetrac_build_config.h` with the
  `#define LIFETRAC_HAS_IMU 1` style flags + the parameter literals
  (`LIFETRAC_AXIS_COUNT`, `LIFETRAC_CAMERA_COUNT`, …). Sketch code
  switches from hard-coded to `#if LIFETRAC_HAS_*` guards. Adds a
  CI job that builds the canonical-BOM config and three
  representative reduced-BOM configs to the existing five compile
  gates.
* **BC-04 — Base-station + web_ui plumbing.** Replace the
  hard-coded `CAMERA_COUNT = 1` / `HAS_IMU = True` constants with
  attribute reads off the loaded `BuildConfig`. UI templates render
  optional tiles only when the corresponding capability is enabled;
  MQTT topics that depend on absent hardware are not subscribed.
  Audit-log records the active `config_sha256` on every boot.
* **BC-05 — Web UI editor page.** New `/config` admin route
  (PIN-gated, same as the existing admin surface): form generated
  from the JSON Schema, client-side validation, save → write
  `config/build.<unit_id>.toml.next` → atomic rename → audit-log →
  prompt operator for restart. Read-only "diff against canonical"
  view to make fleet-onboarding obvious.
* **BC-06 — Per-build HIL harness gating.** Extend the Round-26
  [`hil/dispatch.ps1`](DESIGN-CONTROLLER/hil/dispatch.ps1) to read
  the active build config and report gates that don't apply as
  `N/A` rather than `NOT-STARTED`. Each `w4-XX_*.ps1` harness
  consults the config in `Assert-Section0-Ready` and `SKIP`s if
  the capability under test is disabled (e.g. W4-08 SKIPs when
  `camera_count == 0`).
* **BC-07 — SIL variant matrix.** New
  `base_station/tests/test_build_config_variants_sil.py` that
  loads the schema, instantiates ≥ 4 representative configs
  (canonical, no-camera, no-IMU/GPS, single-axis), and asserts
  the bridge + UI startup paths run clean (no exceptions, no
  subscription to disabled topics, no broken template renders).
  Plus a source-grep tripwire that `web_ui.py` references no
  optional capability without a `BuildConfig.has_*` guard.
* **BC-08 — Migration + onboarding doc.** New
  `DESIGN-CONTROLLER/BUILD_CONFIG.md` that explains the schema,
  the editor, the codegen, and the supported variants; existing
  builders create a `build.<unit_id>.toml` once and never edit
  firmware again.
* **BC-09 — BOM cross-reference.** Update
  [`HARDWARE_BOM.md`](DESIGN-CONTROLLER/HARDWARE_BOM.md) so each
  optional row lists its BC capability id, and a SIL test asserts
  every `optional` BOM row is referenced by exactly one capability
  in the schema (no orphans either way).
* **BC-10 — Config delivery & hot-reload contract.** Defines *how*
  a new `build.<unit_id>.toml` lands on a tractor / base station
  and *when* the running daemons reload it. Explicitly **non-OTA
  over LoRa** — LoRa airtime is reserved for control + telemetry
  + thumbnails, and the LoRa write surface is intentionally narrow
  (control / E-stop / camera-select opcodes only); rewriting the
  config over LoRa would be a category jump in trust boundary.
  Supported delivery paths, ranked by preference:
  1. **USB-cable from a laptop** (preferred for technical operators).
     The X8 enumerates as `g_ether` USB-Ethernet gadget; the laptop
     sees a `usb0` interface and runs `lifetrac-config push --target
     usb0 ./build.<unit_id>.toml`. CLI streams installer stdout
     (validation errors, applied SHA, deferral reason) to the laptop
     terminal in real time. Narrowest attack surface, best UX for
     anyone comfortable with a terminal.
  2. **USB-stick installer bundle generated by the base-station web
     UI** (preferred for non-technical operators — default field
     workflow). Operator opens the PIN-gated `/config` page on the
     base, edits fields via the schema-driven form, clicks
     **Download installer bundle** — base emits a single file named
     `lifetrac-config-<unit_id>-<sha8>.toml` with the validated TOML
     plus an embedded `# sha256:<full>` header comment. Operator
     copies the file to any USB stick, walks it to the tractor,
     plugs in. The tractor X8 installer auto-discovers the file
     (`lifetrac-config-*.toml` glob), refuses if `unit_id` in the
     filename or payload doesn't match this tractor, refuses if the
     embedded SHA doesn't match the file body (anti-corruption),
     then applies via the atomic-rename + quiescence-gate path.
     Result file `lifetrac-config-result.json` is written back to
     the stick: `{unit_id, source_sha256, applied_sha256, accepted,
     reason, applied_fields[], reload_class, timestamp}`. Operator
     pulls the stick, plugs it back into the base, the `/config`
     page surfaces the receipt and any deferral status. **All
     validation happens on the base UI before the file ever leaves
     the building**, so no syntax/range errors reach the tractor.
     Plus tractor-side OLED status line (`CFG: validated, applied
     <sha8>` / `CFG: REJECTED <reason>` / `CFG: deferred (in use)`)
     and X8 status-LED flash convention (3 long = applied, 3 short
     = rejected, alternating = deferred).
  3. **Hand-authored USB-stick TOML** (fallback for ops without the
     base online). Same on-tractor installer logic as path 2, but
     no UI-side validation — the installer is the only line of
     defence. Same OLED + LED feedback. Same result-file write-back.
     Documented as "power-user" in BUILD_CONFIG.md.
  4. **Web admin form on the base station** (BC-05, base-side
     config only — the base's *own* `BuildConfig` reload, not the
     tractor's).
  5. **SSH from any networked laptop** (advanced operators only).
     Same `lifetrac-config push` CLI, just over the LAN/WiFi link
     to the X8 instead of `usb0`. Documented in BUILD_CONFIG.md
     as the path for fleet-wide scripted updates; not the
     primary field workflow because not every operator is
     comfortable with SSH key management.

  **Quiescence gate** (precondition for live reload): tractor in
  `STATE_PARKED` for ≥ N seconds (default 30, configurable), no
  active `/ws/control` subscriber, engine off OR idle-low,
  M7 TX queue empty. If any condition fails, installer writes
  `.toml.next` and the daemons pick it up at the next quiescent
  window; operator sees a "reload pending" banner.

  **Reload taxonomy** (encoded as a `reload_class` annotation on
  every JSON Schema property):
  * `live` — reloadable without daemon restart (UI strings, camera
    presence flags, MQTT host with reconnect, `max_control_subscribers`,
    IMU/GPS presence flags, `hyd_pressure_sensor_count`).
  * `restart_required` — needs graceful daemon restart
    (`lora_region`, `estop_topology`, `m4_watchdog_ms`, axis counts).
    Detected by diffing old vs new `BuildConfig`; logged as
    "restart required" and operator confirms via OLED button or
    web UI before reboot.

  Deliverables: `tools/lifetrac-config` Python CLI (laptop-side,
  for paths 1 + 5); X8-side `lifetrac-config-installer` daemon
  (handles USB-stick discovery for paths 2 + 3, atomic rename,
  result-file write-back, OLED + LED feedback); base-side
  `/config/download` route emitting the named installer bundle
  for path 2; quiescence detector module in
  `base_station/build_config.py`; new `test_build_config_delivery_sil.py`
  (~15 tests covering USB-stick filename + unit-id-match
  enforcement, embedded-SHA verification, base-side bundle
  generation, quiescence-gate logic, reload-class annotations,
  result-file schema, atomic-rename invariant, deferred-reload
  pickup).

### Out of scope (for now)

* **OTA delivery of build configs over LoRa.** Wrong trust
  boundary (LoRa write surface is intentionally narrow), wrong
  airtime budget (would steal from control/telemetry/thumbnails),
  and would require reinventing transactional delivery + rollback
  on a half-duplex radio. Deliver via USB-cable / SSH / web admin
  per BC-10 instead.
* OTA delivery of build configs over cellular backup (would need
  signing + rollback; defer).
* Per-axis hydraulic flow auto-calibration from manifold response
  (separate initiative; the config only carries operator-supplied
  rated flows).
* Fundamentally different control topologies (e.g. CAN bus vs
  RS-485). The schema is for *presence/absence and parameters*,
  not for swapping the control fabric.

### Acceptance criteria

* `pwsh ./dispatch.ps1` shows correct PASS / N/A / NOT-STARTED for
  three distinct test fleets (canonical, no-camera, single-axis).
* All 329 + new BC SIL tests pass under the canonical config.
* Web UI `/config` editor round-trips a config edit → reboot →
  audit-logged `config_sha256` change with no firmware re-flash
  needed for run-time-only flags.
* No firmware sketch contains a hard-coded "is this hardware
  present" check that the schema doesn't also gate.

## Open initiative — Hydraulic soft-stop sequencer & build variants (BC-18 / BC-19 / BC-20)

Tracked in detail in
[DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md](DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md)
and the canonical decisions in
[DESIGN-CONTROLLER/DECISIONS.md](DESIGN-CONTROLLER/DECISIONS.md) §D-HYD1 and
§D-HYD2. **Status:** designed, not yet implemented. Three sequential rounds:

* **BC-18 — Valve-centre settling timer (firmware sequencer).** Add the
  100 ms (default) settling delay between EFC reaching zero and the
  directional-valve solenoid de-energising, gating the tandem-centre spool
  (`D1VW00*8*CNKW`) becoming usable in field hardware. Concrete work:
  add `LIFETRAC_HYDRAULIC_VALVE_SETTLING_MS` macro to
  [`firmware/common/lifetrac_build_config.h`](DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h);
  gate solenoid de-energise in
  [`firmware/tractor_opta/tractor_opta.ino`](DESIGN-CONTROLLER/firmware/tractor_opta/tractor_opta.ino)
  valve-drive loop behind a settling timer that starts when each
  `REG_FLOW_SP_*` register reaches zero; new SIL gate
  `base_station/tests/test_valve_settling_sil.py` (4 cases:
  axis-active→zero starts timer; EFC reaches zero before timer; valve
  de-energises only after timer; E-stop cancels timer immediately);
  MASTER_TEST_PROGRAM.md row.

* **BC-19 — Hydraulic build-variant schema. ✓ LANDED Round 43.**
  Make the soft-stop parameters configurable per unit so non-canonical
  hydraulic builds (OSE-legacy float + PO check, high-performance
  open + counterbalance, etc.) work without forking firmware. Three
  new leaves added under `[hydraulic]` in
  [`base_station/config/build_config.schema.json`](DESIGN-CONTROLLER/base_station/config/build_config.schema.json) —
  `spool_type` (enum: `tandem` / `float` / `closed` / `open`,
  `restart_required`), `load_holding` (enum: `spool_inherent` /
  `po_check` / `counterbalance` / `none`, `restart_required`),
  `valve_settling_ms` (uint 0..250, `live`).
  [`build.default.toml`](DESIGN-CONTROLLER/base_station/config/build.default.toml),
  `HydraulicConfig` dataclass in
  [`build_config.py`](DESIGN-CONTROLLER/base_station/build_config.py),
  schema-driven codegen in
  [`build_config_codegen.py`](DESIGN-CONTROLLER/base_station/build_config_codegen.py)
  (auto-emits 11 new `LIFETRAC_HYDRAULIC_*` macros incl. enum side-
  flags), and `CAPABILITY_INVENTORY.md` rows all updated.
  Cross-leaf compatibility validator
  `_validate_hydraulic_compatibility` lives in
  [`build_config.py`](DESIGN-CONTROLLER/base_station/build_config.py)
  and runs after JSON-Schema validation in `load()`; rejects three
  contradictory combinations (tandem/closed + load_holding=none;
  float/open + load_holding=spool_inherent; float/open + non-zero
  valve_settling_ms). New SIL gate
  [`test_hydraulic_compatibility_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_hydraulic_compatibility_sil.py)
  pins 13 cases (BC19_A canonical, BC19_B four reference builds load,
  BC19_C six invalid pairings rejected, BC19_D rejection-at-load
  tripwire). MASTER_TEST_PROGRAM.md §5 row promoted from "(planned)"
  to landed. BUILD_CONFIG.md gets a new "Common pitfalls" entry on
  the cross-leaf validator hook. Firmware consumer (BC-18) will read
  the new `LIFETRAC_HYDRAULIC_SPOOL_TYPE` and `_VALVE_SETTLING_MS`
  macros in the next round instead of hard-coding tandem.

* **BC-20 — IMU-adaptive ramp tuning (jerk-based auto-tune).** Use
  the BNO086 IMU already on the canonical BOM
  ([HARDWARE_BOM.md](DESIGN-CONTROLLER/HARDWARE_BOM.md)) to detect jerk
  spikes during start/stop transitions and slowly nudge
  `LIFETRAC_HYDRAULIC_TRACK_RAMP_SECONDS` /
  `_ARM_RAMP_SECONDS` / `_VALVE_SETTLING_MS` toward a smoother
  trajectory. **Read-only telemetry first; closed-loop tuning gated.**
  Detail in [DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md](DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md)
  §"IMU-adaptive ramp tuning". Three sub-rounds:

  * **BC-20A — passive jerk telemetry.** Compute `d(accel)/dt` from
    BNO086 linear-acceleration samples on the Portenta H7 M7 core,
    publish per-transition jerk peak (g/s, signed per axis) over the
    existing telemetry channel, surface as a chart in the web UI
    diagnostics page. **No control-loop change.** SIL: synthetic
    accel time series → expected jerk peak.

  * **BC-20B — recommendation engine (offline / advisory).** Base
    station accumulates per-build jerk-vs-ramp histograms over
    operator-confirmed clean transitions and recommends ramp-leaf
    edits (e.g. "track_ramp_seconds 2.0 → 2.4 reduces P95 jerk from
    1.8 g/s to 0.9 g/s"). Operator approves via the existing
    `lifetrac-config` flow (validate / bundle / verify / push) — no
    autonomous writes. SIL: mock telemetry stream → expected
    recommendation; reject recommendations that would push values
    outside the schema range.

  * **BC-20C — closed-loop online tuning (gated, opt-in).** New
    schema leaf `hydraulic.adaptive_ramp_tuning` (bool, default
    `false`, `restart_required`). When enabled, M7 firmware adjusts
    ramp seconds in ±5 % steps per accepted clean transition,
    bounded by the schema range, audit-logged, with operator panic
    button to revert to defaults. Disabled by default for v25
    canonical because closed-loop control on a safety-relevant
    parameter requires real bench validation (BC-12 HIL hardware
    dependency). SIL: simulated jerk feedback drives ramp
    adjustments; out-of-range writes refused; revert-on-panic
    restores baseline.

  **Why three sub-rounds:** BC-20A is pure observability and is safe
  to ship anytime. BC-20B closes the loop through the human via the
  existing config-delivery flow, which already has all the safety
  gates we need. BC-20C is the only sub-round that adds autonomous
  writes to a safety parameter, so it is opt-in, gated by HIL bench
  validation, and exposes a panic revert.

## Current sprint status — DESIGN-CONTROLLER LoRa stack (2026-04-27)

Tracked in detail in [DESIGN-CONTROLLER/TODO.md](DESIGN-CONTROLLER/TODO.md) and
[DESIGN-CONTROLLER/DECISIONS.md](DESIGN-CONTROLLER/DECISIONS.md). Snapshot:

### Done
- [x] Handheld firmware: full RX, KISS, per-source replay, `CMD_LINK_TUNE` /
  `CMD_ESTOP` / `CMD_CLEAR_ESTOP` ingest, OLED status, latching mushroom E-stop
  ([firmware/handheld_mkr/handheld_mkr.ino](DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino)).
- [x] Base-station audit log
  ([base_station/audit_log.py](DESIGN-CONTROLLER/base_station/audit_log.py))
  + bridge wiring (rx/tx/tx_error/gcm_tag_reject/bad_header/replay_reject/
  link_tune/encode_mode_change/airtime_alarm) + per-source ReplayWindow.
- [x] DECISIONS.md option tables for D-A2 / D-A3 / D-C1 / D-C2 / D-C6 / D-E1.
- [x] **D-A2 control PHY → SF7/BW250** (~46 ms encrypted ControlFrame, fits
  20 Hz cadence). Updated in C, Python, both LADDER tables, LORA_PROTOCOL.md.
- [x] **D-A3 image PHY → SF7/BW500** (~18 ms 32 B fragment, fits 25 ms cap).
- [x] **D-C1/C2 real-crypto scaffold**:
  [lp_crypto_real.cpp](DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp)
  with MbedTLS (Portenta H7) + rweather/Crypto (MKR) backends, gated on
  `-DLIFETRAC_USE_REAL_CRYPTO`; `crypto_stub.c` re-guarded so it skips when
  real crypto wins.
- [x] **D-C6 CSMA scanChannel** wired into handheld TX under
  `#ifdef LIFETRAC_FHSS_ENABLED`.
- [x] **D-E1 web UI PIN auth**: `LIFETRAC_PIN`, HttpOnly+SameSite=strict
  cookie, 30 min idle TTL, 5-fail/60 s IP lockout, WS cookie-check, new
  `/api/login` `/api/logout` `/api/session` routes
  ([base_station/web_ui.py](DESIGN-CONTROLLER/base_station/web_ui.py),
  [tests/test_web_ui_auth.py](DESIGN-CONTROLLER/base_station/tests/test_web_ui_auth.py)).
- [x] Tractor X8 service scaffolds:
  [time_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/time_service.py),
  [params_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/params_service.py),
  [logger_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/logger_service.py).
- [x] [DESIGN-CONTROLLER/KEY_ROTATION.md](DESIGN-CONTROLLER/KEY_ROTATION.md)
  operator procedure.
- [x] arduino_libraries.txt: + Adafruit SSD1306/GFX + rweather Crypto.
- [x] All 26 base_station unit tests pass (1 env-skip when fastapi/paho
  missing).
- [x] **Mirror handheld CSMA #ifdef into the tractor M7 TX path** —
  shared `csma_pick_hop_before_tx()` helper called from both
  `emit_topic()` and `send_link_tune()` under `LIFETRAC_FHSS_ENABLED`
  ([tractor_h7.ino](DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)).
- [x] **`g_fhss_hop_counter` / `g_fhss_key_id` globals** defined on M7 and
  handheld (file-scope, not static, so a future shared header can re-extern
  them). Initialised to 0; epoch-rotated by `time_service.py` once the
  X8↔H747 UART tick lands.
- [x] **D6 source-active telemetry enrichment**: `emit_source_active()`
  payload extended from 8 → 20 bytes, now carries per-source RSSI (int16)
  + SNR×10 (int16) for `[HANDHELD, BASE, AUTONOMY]` alongside the
  existing source/rung/estop/pending/tune-failures fields. SourceState
  gained `snr_db_x10`; `process_air_frame` captures `radio.getSNR()` per RX.
- [x] **Web operator HTML/JS** PIN-entry page +
  [`/login`](DESIGN-CONTROLLER/base_station/web/login.html) route on
  `web_ui.py`; `/` now redirects to `/login` when no valid session.
  The operator console (joysticks, telemetry sidebar, camera selector,
  E-stop, gamepad support) was already in place.
- [x] **Real-crypto golden vectors**:
  [vectors.json](DESIGN-CONTROLLER/firmware/bench/crypto_vectors/vectors.json)
  + Python regression
  [test_crypto_vectors.py](DESIGN-CONTROLLER/base_station/tests/test_crypto_vectors.py)
  + host-build cross-check
  [host_check.c + Makefile](DESIGN-CONTROLLER/firmware/bench/crypto_vectors/)
  (`make crypto-check`, needs `libmbedtls-dev`).
- [x] **Tractor X8 service follow-through**: real
  [time_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/time_service.py)
  (PPS_FETCH ioctl + UART tick + wall-clock fallback),
  [params_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/params_service.py)
  (atomic JSON store + FastAPI sub-app + MQTT publish-on-change), and
  [logger_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/logger_service.py)
  (paho subscriber + queued SQLite writer + JSONL audit).
- [x] **`tools/provision.py`** gained a `--write-port` USB-CDC mode
  implementing the wire protocol from KEY_ROTATION.md (prologue → KEY:
  → COMMIT). Header-generation mode unchanged.
- [x] All 39 base_station unit tests pass (2 env-skips: fastapi + paho
  optional; cryptography optional for the golden-vector test).
- [x] **Phase 5 Docker / compose stack**:
  [Dockerfile](DESIGN-CONTROLLER/Dockerfile),
  [docker-compose.yml](DESIGN-CONTROLLER/docker-compose.yml),
  [base_station/mosquitto.conf](DESIGN-CONTROLLER/base_station/mosquitto.conf),
  [.env.example](DESIGN-CONTROLLER/.env.example), and
  [systemd units](DESIGN-CONTROLLER/base_station/systemd/) for the
  base + tractor X8 services. `audit_log.py` gained a `--tail-mqtt`
  CLI mode used by the `audit_tail` compose service.
- [x] **Tractor M4 core hardened**:
  [firmware/common/shared_mem.h](DESIGN-CONTROLLER/firmware/common/shared_mem.h)
  formalises the M7↔M4 SRAM4 layout (version + alive_tick + loop_counter
  + estop_request); rewrote
  [tractor_h7_m4.ino](DESIGN-CONTROLLER/firmware/tractor_h7_m4/tractor_h7_m4.ino)
  with three independent trip conditions (stale tick, stuck loop counter,
  M7 e-stop request) + heartbeat LED. M7 stamps version/loop_counter/
  estop_request every iteration.
- [x] **Opta expansion shims replaced with real `OptaBlue` library
  calls** (digitalWrite via `DigitalExpansion`, analog out/in via
  `AnalogExpansion`); `OptaController.begin()`/`update()` wired into
  `setup()`/`loop()`.
- [x] **Web UI tractor-params proxy** —
  `web_ui` subscribes to `lifetrac/v25/params/changed`, exposes
  GET/POST `/api/params`, publishes patches on `lifetrac/v25/params/set`;
  `params_service` listens on the same topic and applies. Settings page
  gained a JSON params editor.
- [x] **Tractor X8 image pipeline encoder** —
  [camera_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py)
  with libcamera + synthetic backends, 12×8 tile diff, WebP per tile
  with quality back-off, I/P frames + CMD_REQ_KEYFRAME subscription;
  publishes ready-to-fragment payloads on `lifetrac/v25/cmd/image_frame`.
- [x] **Web UI image canvas + audit viewer** — `<canvas id="image-canvas">`
  in `index.html`; `/ws/image` binary WS in `web_ui.py` forwarding the
  `lifetrac/v25/video/canvas` topic; new
  [`web/audit.html`](DESIGN-CONTROLLER/base_station/web/audit.html) page
  + `/api/audit` route reading the rotating JSONL tail with filter and
  auto-refresh.
- [x] **Cross-cutting docs**:
  [SAFETY_CASE.md](DESIGN-CONTROLLER/SAFETY_CASE.md) (ISO 25119 hazard
  table + AgPL targets), [CYBERSECURITY_CASE.md](DESIGN-CONTROLLER/CYBERSECURITY_CASE.md)
  (IEC 62443 SL-1 sketch + STRIDE per zone),
  [OTA_STRATEGY.md](DESIGN-CONTROLLER/OTA_STRATEGY.md) (signed image
  flow + A/B + 60 s health-check rollback),
  [PAIRING_PROCEDURE.md](DESIGN-CONTROLLER/PAIRING_PROCEDURE.md)
  (initial provision, field re-pair, decommission).
- [x] **Tooling**: [tools/keygen.py](DESIGN-CONTROLLER/tools/keygen.py)
  (offline 16/32 B fleet-key generator) and
  [tools/replay_pcap.py](DESIGN-CONTROLLER/tools/replay_pcap.py)
  (offline GCM + replay-window validator over a hex capture file).

### Remaining — code (no hardware needed)

The 2026-04-27 backlog sweep against
[DESIGN-CONTROLLER/TODO.md](DESIGN-CONTROLLER/TODO.md) Phase 5A/5A.B/5B/5C +
Cross-cutting + Tractor image pipeline identified six pure-code buckets
(A-F). **All six are now code-complete; G remains intentionally skipped.**
71/71 base_station unit tests pass (`python -m unittest discover tests`).

**A — Base-station image pipeline (Python, `base_station/image_pipeline/`):** ✅ Done

- [x] `reassemble.py` — collect TileDeltaFrame (topic `0x25`) fragments,
  time out missing fragments, surface stale-tile bitmap.
- [x] `canvas.py` — persistent tile canvas; on `base_seq` mismatch publish
  `CMD_REQ_KEYFRAME` (opcode `0x62`); attach badge enum to every published
  tile.
- [x] `bg_cache.py` — rolling per-tile median; fills missed tiles with
  `Cached` badge + age.
- [x] `state_publisher.py` — authoritative WS publisher (canvas tiles +
  per-tile age + badge + detections + safety verdicts + accel status).
- [x] `fallback_render.py` — server-side 1 fps composite for HDMI console
  + headless QA.
- [x] `recolourise.py` — Y-only luma + 30 s colour reference (scheme Z),
  sets `Recolourised` badge.
- [x] `motion_replay.py` — applies `0x28` motion vectors to canvas, sets
  `Predicted` badge (Q degraded mode).
- [x] `wireframe_render.py` — renders `0x29` PiDiNet wireframe overlay,
  sets `Wireframe` badge (P extreme degraded mode).
- [x] `link_monitor.py` `LinkMonitor` orchestrator with on_air()/tick(),
  publish_command/publish_status callbacks, and 3-window hysteresis ladder.

**B — Browser image-tier modules (`base_station/web/img/`):** ✅ Done

- [x] `canvas_renderer.js` — WS subscriber; per-tile blits via OffscreenCanvas
  worker pattern; dispatches `lifetrac-state` + `lifetrac-tile-painted` events.
- [x] `fade_shader.js` — 3-frame alpha pulse on `#image-fade` overlay.
- [x] `staleness_overlay.js` — yellow tint scaling with server-supplied
  `age_ms` (1 s threshold, max at 5 s).
- [x] `badge_renderer.js` — fail-closed badge enforcement; refusals POST
  to `/api/health/refusal`.
- [x] `detection_overlay.js` — bbox rendering by class colour; toggles
  `#detector-disagree` banner.
- [x] `accel_status.js` — fixed-position pill with 4 status colours.
- [x] `raw_mode_toggle.js` — body.raw-mode CSS class toggle, persisted in
  localStorage; choice POSTs to `/api/audit/view_mode`.
- [x] `web_ui.py` wired with `/ws/state` + `/api/health/refusal` +
  `/api/audit/view_mode`; tile_delta MQTT topics dispatched into
  Canvas + StatePublisher singletons.

**C — Tractor X8 image pipeline (split out of monolithic `camera_service.py`):** ✅ Done

- [x] `firmware/tractor_x8/image_pipeline/register.py` — phase-correlation
  pre-diff (NumPy + optional OpenCV NEON path).
- [x] `firmware/tractor_x8/image_pipeline/roi.py` — ROI mask from valve
  activity + `CMD_ROI_HINT` (opcode `0x61`) honour.
- [x] `firmware/tractor_x8/image_pipeline/encode_motion.py` — block-match
  optical-flow microframe encoder for topic `0x28`.
- [x] `firmware/tractor_x8/image_pipeline/encode_wireframe.py` — packed-bitmap
  edge encoder for topic `0x29`.
- [x] `firmware/tractor_x8/image_pipeline/ipc_to_h747.py` — UART ring-buffer
  hand-off with CRC-8/SMBUS-framed envelope.

**D — AI detector CPU scaffolds (model weights out of band, scaffolding pure
code):** ✅ Done

- [x] `base_station/image_pipeline/detect_yolo.py` — base-side independent
  safety detector with NanoDet-Plus default (Apache-2.0) and YOLOv8 path
  guarded by `LIFETRAC_DETECTOR=yolov8`; `cross_check()` IoU verdict +
  `DetectorWorker` thread.
- [x] `base_station/image_pipeline/superres_cpu.py` — ncnn Real-ESRGAN-x4v3
  façade with passthrough fallback.
- [x] `firmware/tractor_x8/image_pipeline/detect_nanodet.py` — NanoDet-Plus
  scaffold + topic-`0x26` `pack_detection_frame` wire format.

**E — Tooling + cross-cutting code:** ✅ Done

- [x] `tools/pair_handheld.py` — handheld provisioning over USB-CDC with
  signed-handshake + audit-log append.
- [x] LoRa R-6 fragment scheme extended to `TelemetryFrame` —
  `pack_telemetry_fragments()` / `parse_telemetry_fragment()` /
  `TelemetryReassembler` in `lora_proto.py`; obeys 25 ms-per-fragment cap.
- [x] Persistent AES-GCM nonce counter — `base_station/nonce_store.py`
  (file-backed, fsync'd, gap-bumped on every reserve, restore on boot).
- [x] Tests: `tests/test_telemetry_fragmentation.py` covers R-6 single +
  multi-fragment round-trips, dedup, timeouts, and per-source `NonceStore`
  persistence across instances.

**F — Documentation:** ✅ Done

- [x] [DESIGN-CONTROLLER/NON_ARDUINO_BOM.md](DESIGN-CONTROLLER/NON_ARDUINO_BOM.md)
  — DigiKey/Mouser/L-com/Phoenix/Bürkert/McMaster consolidated order list.
- [x] [DESIGN-CONTROLLER/CALIBRATION.md](DESIGN-CONTROLLER/CALIBRATION.md)
  — joystick deadband, flow-valve curve, pressure zero, GPS offset, IMU bias.
- [x] [DESIGN-CONTROLLER/FIELD_SERVICE.md](DESIGN-CONTROLLER/FIELD_SERVICE.md)
  — spare-parts kit, fuse map, diagnostic flowcharts, escalation.
- [x] [DESIGN-CONTROLLER/OPERATIONS_MANUAL.md](DESIGN-CONTROLLER/OPERATIONS_MANUAL.md)
  — operator-facing power-on, pairing, take-control, E-stop, charging.
- [x] [DESIGN-CONTROLLER/FIRMWARE_UPDATES.md](DESIGN-CONTROLLER/FIRMWARE_UPDATES.md)
  — per-node update path (X8 OCI vs H747 USB-CDC vs handheld USB-CDC),
  signing, fleet sequencing.
- [x] [DESIGN-CONTROLLER/BASE_STATION.md](DESIGN-CONTROLLER/BASE_STATION.md)
  trust-boundary table per IMAGE_PIPELINE §6.1 (server-only vs browser surface).

**H — 2026-04-27 follow-up sweep (residual no-hardware items):** ✅ Done

- [x] [firmware/tractor_x8/image_pipeline/tile_diff.py](DESIGN-CONTROLLER/firmware/tractor_x8/image_pipeline/tile_diff.py)
  — 64-bit pHash per 32 px tile + Hamming-distance differ.
- [x] [firmware/tractor_x8/image_pipeline/fragment.py](DESIGN-CONTROLLER/firmware/tractor_x8/image_pipeline/fragment.py)
  — image-PHY fragmenter that reuses the R-6 magic so base-station
  reassembly stays identical for telemetry + image.
- [x] [firmware/tractor_x8/image_pipeline/capture.py](DESIGN-CONTROLLER/firmware/tractor_x8/image_pipeline/capture.py)
  — `CaptureRing` newest-wins ring buffer with `MockCaptureBackend`
  (unit-testable on Windows / CI) and a `V4l2CaptureBackend` shim
  whose embedded body is filled in by the vendor patch.
- [x] [tools/lora_rtt.py](DESIGN-CONTROLLER/tools/lora_rtt.py)
  — RTT harness with JSONL logging, percentile summary, and an
  `--echo-loopback` CI mode that needs no real radios.
- [x] [base_station/person_alert.py](DESIGN-CONTROLLER/base_station/person_alert.py)
  — `CMD_PERSON_APPEARED` (opcode `0x60`) packer + `PersonAlertEmitter`
  with confidence filter, debounce, and audit-log hook; ready to be
  passed as `DetectorWorker(on_result=emitter.feed)`.
- [x] Named `audit_log.py` event helpers (`log_sf_step`, `log_encode_mode_change`,
  `log_fhss_skip`, `log_replay_reject`, `log_gcm_reject`, `log_source_transition`,
  `log_person_appeared`); `lora_bridge.py` and `link_monitor.py` switched
  to the named helpers so the JSONL stays queryable.
- [x] [base_station/web/diagnostics.html](DESIGN-CONTROLLER/base_station/web/diagnostics.html)
  + [diagnostics.js](DESIGN-CONTROLLER/base_station/web/diagnostics.js)
  — airtime utilization graph (30 % WARN / 60 % ALARM bands), SF rung
  history with hysteresis marker, FHSS 8×60 s heatmap, link-loss
  timeline; subscribes to `/ws/state` only.
- [x] [base_station/web/map.html](DESIGN-CONTROLLER/base_station/web/map.html)
  + [map.js](DESIGN-CONTROLLER/base_station/web/map.js)
  — Leaflet with offline-first tile pyramid (`/tiles/{z}/{x}/{y}.png`),
  OSM fallback when online, live tractor marker + breadcrumb track,
  range-estimate sidebar from the FSPL model in IMAGE_PIPELINE Appendix B.

88/88 base_station unit tests pass (was 71; +17 new tests for the helpers,
emitter, and `lora_rtt._percentile`).

**G — Skip (explicitly out of scope this pass):**

- [ ] Legacy-prototype safety bugs (rest of this file) — only worth fixing
  if the prototype gets used for any further bench tests.
- Anything Coral-only (`superres_coral.py`, `interp_rife.py`, `inpaint_lama.py`).
- Anything in Phase 6+ (mast install, integration, field test, FCC verification).
- Phase 10+ stretch goals (SVT-AV1, neural-inflate, NDVI, ROS 2 bridge, etc.).

### Remaining — needs hardware

- [ ] **Phase A1 R-7 retune bench** —
  [firmware/bench/lora_retune_bench/](DESIGN-CONTROLLER/firmware/bench/lora_retune_bench/)
  is ready; needs two LoRa boards on a desk to measure actual retune cost.
- [ ] **Phase B Opta Modbus slave** — replace the `OptaController` /
  D1608S / A0602 placeholder shims in
  [firmware/tractor_opta/opta_modbus_slave.ino](DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino)
  with real library calls; needs an Opta + expansions on the bench.
- [ ] **Compile `lp_crypto_real.cpp` on-target** (Portenta H7 + MKR WAN
  1310) and verify with the golden-vector test from above.
- [ ] **Tractor radio driver/control path** (bridge currently talks
  KISS-over-serial; on-tractor needs the real radio path). 2026-05-04
  update: Max Carrier evidence points away from a bare M7↔SX1276 SPI
  path and toward the Murata LPWAN AT modem interface on X8 Linux
  `/dev/ttymxc3`; revalidate before writing more raw-SPI RadioLib code.
- [ ] **Bench-validate E-stop end-to-end** per the procedure in
  [KEY_ROTATION.md](DESIGN-CONTROLLER/KEY_ROTATION.md) §7.

---

## Hardware-in-the-loop (HIL) gate — required before any wet hydraulic test

These are the items the Round 1–4 implementation plan explicitly deferred
because they cannot be fully validated without hardware. W4-pre and W4-00
use the minimal two-carrier desk setup from the runbook; W4-01 and later
need the full bench setup (handheld + tractor H747 + Opta + expansions +
SX1276 modems on both ends). Each gate must pass before the system is
allowed to drive real hydraulics. Tracked
against the Wave-4 gates in
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md` § Wave 4](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md).

**HIL bench setup prerequisites:**

W4-pre and W4-00 use their own minimal hardware lists in the runbook. The
full setup below applies before W4-01 and later.

- [ ] Two SX1276 LoRa modems wired (handheld MKR WAN 1310 + tractor H747).
- [ ] Tractor H747 + Opta connected over RS-485 with both expansions
  (D1608S + A0602) chained.
- [ ] Solenoid-valve bank powered through the PSR safety relay with the
  hydraulic supply valved off (dry test only, no fluid pressure).
- [ ] Logic analyzer or scope on the X8↔H747 UART, the M7↔M4 GPIO, and
  the PSR-alive line. CRC-protected `.csv` capture for each test.
- [ ] Battery emulator on the 12 V rail so a brownout can be simulated
  without yanking the keyswitch.

**HIL tests (Wave 4 gates):** Step-by-step bench procedures, capture
conventions, pass criteria, and sign-off rules for every gate below are
documented in
[`DESIGN-CONTROLLER/HIL_RUNBOOK.md`](DESIGN-CONTROLLER/HIL_RUNBOOK.md).
Execute from the runbook; the bullets below are the index.

- [ ] **W4-pre Board bring-up sanity.** Partial 2026-05-04 evidence:
  both Portenta X8 M7 cores flash, boot, reach `loop()`, and update
  SRAM4 liveness with CFSR/HFSR = 0. Still open: USB enumeration,
  3.3 V/5 V rail measurements, blink/echo, stock M7<->M4 handshake,
  and formal capture/sign-off package.
- [ ] **W4-00 LoRa stack dual-Portenta bench.** Blocked until the X8
  no-USB bench image has a control plane plus a real base-role
  1000-frame encrypted ControlFrame burst sender and receiver-side
  counters/export. Also blocked on Max Carrier radio-interface
  revalidation: the onboard Murata `CMWX1ZZABZ-078` appears to be
  exposed through the X8 Linux AT-modem path (`/dev/ttymxc3`), while
  the current M7 raw-SPI RadioLib diagnostic fails before TX start.
  The current PowerShell harness records metrics but does not generate
  the burst.
- [ ] **W4-01 Handheld E-stop latch latency.** Mushroom-button press →
  PSR-alive drops → all 8 valve coils de-energize within **< 100 ms**
  measured at the relay terminals. Repeat 100× across the SF7/SF8/SF9
  PHY rungs. Capture worst-case + histogram.
- [ ] **W4-02 Link-tune walk-down.** Force RSSI degradation
  (RF attenuator) and verify the M7 walks SF7 → SF8 → SF9 and back via
  `CMD_LINK_TUNE` with **< 1% packet loss** during each transition.
  Verify `try_step_ladder()` revert deadline (500 ms) fires correctly
  on a synthetic missing-HB.
  *SIL coverage:* [`base_station/tests/test_link_tune_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_link_tune_sil.py)
  (Round 19) ports `try_step_ladder()`, `poll_link_ladder()`, the HB
  ingress hook, and `apply_phy_rung()` from
  [`firmware/tractor_h7/tractor_h7.ino`](DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
  to pure-Python; 16 tests cover TL-A through TL-I including the
  3-bad-window walk-down ladder, 6-good-window walk-back ladder, SF9
  saturation gate, SF7 saturation gate, twice-back-to-back announce
  invariant, 500 ms revert deadline (commit/revert/edge cases), and
  the stale-active-source pessimism rule. The < 1% packet-loss
  measurement is the only piece that still needs HIL bench time.
- [ ] **W4-03 M7↔M4 watchdog trip.** Halt the M7 with a debugger
  breakpoint or a deliberate `while(1)` injected via a debug build;
  verify the M4 trips the PSR within **200 ms** of the last
  `alive_tick_ms` and that `estop_request = 0xA5A5A5A5` is observed
  on the SRAM4 capture (IP-105/106 seqlock holds).
- [ ] **W4-04 Modbus failure → E-stop.** Pull the RS-485 cable mid-run;
  verify IP-205 counter ticks, `apply_control(-1)` fires after 10
  consecutive failures, valves go neutral, and the audit log shows
  the disconnect within 1 s.
  *SIL coverage:* [`base_station/tests/test_modbus_slave_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_modbus_slave_sil.py)
  (Round 10) models both the M7 `apply_control` writer and the
  Opta-side `on_holding_change`/`check_safety` surface; 13 tests cover
  TR-A through TR-J including the < 1 s W4-04 budget. Note TR-I:
  current firmware uses *cumulative* not *consecutive* fail count —
  pinned by `test_fail_count_is_cumulative_not_consecutive` so a
  future firmware fix will be a loud test failure.
- [ ] **W4-05 Proportional valve ramp-out (IP-303 Round-4 follow-on).**
  Hold full-speed track for 3 s, release joystick; verify
  `REG_FLOW_SP_*` ramps from 10000 mV to 0 over **2 s** (track ladder)
  on the A0602 output, valve coil stays energized through the ramp,
  drops at t=2 s. Repeat for arm axes (1 s ramp).
  *SIL coverage:* [`base_station/tests/test_axis_ramp_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py)
  (Round 18) ports `step_axis_ramp()` + the `apply_control()`
  arbitration to pure Python. TR-A pins the 2 s deadline at full speed,
  TR-B the linear-interpolation midpoint, TR-C/D both ladders
  (track 2/1/0.5 s, arm 1/0.5/0.25 s), TR-E the coil-stays-engaged
  invariant during ramp, TR-F flow-setpoint monotonicity. Bench job is
  now confirm-only.
- [ ] **W4-06 Mixed-mode skip.** Drive both axes; release one; verify
  released axis stops *immediately* (no ramp) because sibling is
  active. Critical for coordinated dig→drive transitions.
  *SIL coverage:* same suite as W4-05. TR-G two-axis skip; TR-H
  orientation-agnostic across all four sibling combinations; TR-I
  last-axis release falls back to ramp; TR-J pins the
  `apply_control` comment that `other_active` is computed from *raw*
  inputs so a stale ramping sibling can't block a fresh release's
  skip-vs-ramp decision.
- [ ] **W4-07 Boot-PHY first-frame decode (IP-006 verification).**
  Cold-boot both ends; first encrypted frame decodes on the receiver
  with no `bad_header` events in the audit log.
  *SIL coverage:* [`base_station/tests/test_boot_phy_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_boot_phy_sil.py)
  (Round 20) parses `firmware/handheld_mkr/handheld_mkr.ino`,
  `firmware/tractor_h7/tractor_h7.ino`, and
  `firmware/tractor_x8/params_service.py` and asserts: BP-A both
  `LADDER[3]` tables are byte-identical (rung mapping for
  `CMD_LINK_TUNE`); BP-A2 `LADDER[0]` matches DECISIONS.md D-A2
  (SF7/BW250/CR4-5/bw_code=1); BP-A3 SF strictly increases down the
  ladder; BP-A4 each rung is uniquely identifiable; BP-B/B2 every
  `radio.begin(...)` references `LADDER[0].sf` / `.bw_khz` / `.cr_den`
  symbolically + sync word `0x12`; BP-B3 nobody slipped a numeric SF /
  BW / CR literal back in; BP-C the Pi-side
  `DEFAULT_PARAMS['link']['control_phy']` agrees with the canonical
  `SF7_BW250` string; BP-D parser self-tests so the regex extractor
  catches mis-formats. The on-air no-`bad_header` confirmation is the
  only piece that still needs HIL bench time.
- [ ] **W4-08 Camera back-channel round-trip (IP-104 Round-4 follow-on).**
  Operator presses "Force keyframe" on web UI → `CMD_REQ_KEYFRAME`
  over LoRa → M7 forwards on Serial1 → `camera_service.py` emits
  I-frame within **< 200 ms** end-to-end. Verify with frame-flag
  capture from the X8↔H747 UART.
- [ ] **W4-09 Async M7 TX state machine (IP-107 follow-on).** Bench-
  validate `radio.startTransmit()` + `isTransmitDone()` IRQ timing
  on real H747 + SX1276 wiring. Once green, replace the
  `refresh_m4_alive_before_tx()` watchdog-refresh hack with the
  proper non-blocking queue. Removes the worst-case time-on-air
  margin assumption.
- [ ] **W4-10 Fleet-key provisioning sanity (IP-008 verification).**
  Flash a fresh image with `lp_keys_secret.h` deleted; confirm the
  M7 halts in `setup()` with the OLED "FLEET KEY NOT PROVISIONED"
  message and the bridge container exits non-zero.

**Status memo for ongoing HIL work:**
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md)
— Round 4 deferred items + Round 5 candidate queue.

---

## Mechanical / UTU integration

- [ ] Verify the loader-arm hydraulic (lift) cylinders do not collide with
  the upper UTU drive shaft / motor across the full arm travel range
  (`ARM_MIN_ANGLE` … `ARM_MAX_ANGLE`). The drive shaft was lowered to leave
  only 0.25" clearance above the rear cross frame tube, so cylinder swing
  geometry should be re-checked at all lift angles.

## Software / firmware safety (from
[2026-04-25 code review](AI%20NOTES/CODE%20REVIEWS/2026-04-25_Review_ClaudeOpus4_7.md))

The first four items also appear in the
[2026-04-16 review](AI%20NOTES/CODE%20REVIEWS/2026-04-16_Review_GPT5_4.md).
~~They are still unfixed. They are the highest-priority work on the software side.~~
*Superseded \u2014 see WONTFIX banner immediately below.*

> **2026-04-29 status update — entire section below is WONTFIX against archived
> prototype code.** All file paths in this section
> (`DESIGN-CONTROLLER/arduino_opta_controller/`,
> `DESIGN-CONTROLLER/esp32_remote_control/`,
> `DESIGN-CONTROLLER/raspberry_pi_web_controller/`) point at code that was
> moved to
> [`DESIGN-CONTROLLER/RESEARCH-CONTROLLER/`](DESIGN-CONTROLLER/RESEARCH-CONTROLLER/README.md)
> in the 2026-04-26 cleanup. The RESEARCH-CONTROLLER README is explicit:
> *"Treat everything in this folder as ideas/prior-art only — not
> implementation plans. The canonical v25 build is LoRa-only, defined in
> `MASTER_PLAN.md`. Code reviews and readiness analyses for the v25
> hardware test do **not** apply to files under `RESEARCH-CONTROLLER/`."*
> The bug surfaces below (Opta MQTT-over-WiFi proportional control,
> blocking reconnect, BLE-without-pairing, hardcoded broker IP, ESP32
> Qwiic-disconnect stale axes, Pi web-UI auth, mode-switch boot-only
> sampling, etc.) all live on the **superseded MQTT-over-WiFi path** that
> the canonical Portenta H7 + MKR WAN 1310 LoRa stack replaced.
>
> **The canonical v25 stack does not use WiFi or BLE for control.** The
> active Opta firmware
> ([firmware/tractor_opta/tractor_opta.ino](DESIGN-CONTROLLER/firmware/tractor_opta/tractor_opta.ino))
> explicitly states *"The Opta's WiFi and BLE radios are intentionally NOT
> initialised here. No WiFi.begin(), no BLE.begin(), no MQTT client, no
> HTTP server."* and `base_station/` contains zero references to WiFi /
> BLE / bluetooth (verified 2026-04-29 via repo-wide grep). The
> equivalent functionality of every legacy bug below has been
> re-implemented from scratch in the active LoRa-only stack with full
> SIL coverage:
>
> | Legacy bug surface | Replaced by canonical-stack equivalent |
> | --- | --- |
> | Opta MQTT joystick float-parse, clamp, NaN-reject | `lora_proto.unpack_control` + Round 18 axis-ramp SIL ([test_axis_ramp_sil.py](DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py)) |
> | Opta blocking MQTT reconnect / safety timeout starvation | Round 25 MQTT retry/backoff fake-clock SIL ([test_mqtt_retry_backoff_sil.py](DESIGN-CONTROLLER/base_station/tests/test_mqtt_retry_backoff_sil.py)) + M4 watchdog (Round 39 boot_self_test) |
> | Opta MQTT mode-switch / flow-valve jumper boot-only sampling | BC-XX BuildConfig (Rounds 27\u201341): per-unit TOML reloaded via `config_watcher.py`, classified `live`/`restart_required`/`firmware_required` |
> | Opta hardcoded MQTT broker IP / credentials | `LIFETRAC_MQTT_HOST` env + PIN-gated web auth (D-E1, Round 9 \u00a7E) |
> | Opta BLE characteristics accept unpaired writes | LoRa control path: AES-GCM + per-source `ReplayWindow` + `LIFETRAC_ALLOW_UNCONFIGURED_KEY` opt-in (Round 22 fleet-key SIL) |
> | ESP32 stale axes on Qwiic disconnect | Handheld MKR replaces ESP32 entirely; LoRa control loop has 200 ms M4 watchdog (`tractor_h7_m4.ino`) that drops PSR on stale `alive_tick_ms` |
> | Pi web controller no-auth, hardcoded broker | Round 9 \u00a7B/\u00a7C/\u00a7D + D-E1 PIN auth on `base_station/web_ui.py` |
> | `setupWiFi()` boot-time blocking | Canonical stack does not use WiFi for control; LoRa is link-up-from-cold |
> | `paho-mqtt < 2.0` pin / atexit cleanup on `app.py` | Active code uses `paho.mqtt.client.Client` via `web_ui._connect_mqtt_with_retry()` (Round 25) and lives under systemd unit lifecycle |
>
> Every item below is therefore WONTFIX in the canonical-stack sense \u2014
> kept here as historical record. The `[ ]` checkboxes are left unticked
> deliberately so a future operator who chooses to revive the prototype
> for a one-off bench test can see what they would need to fix first.
> See [`RESEARCH-CONTROLLER/README.md`](DESIGN-CONTROLLER/RESEARCH-CONTROLLER/README.md)
> for the harvest-from-this list per legacy module.

### Stale-command / fail-safe (must fix before further deployment)

- [ ] **Opta MQTT proportional control is broken.** Replace
  `doc["..."] | 0` with `doc["..."] | 0.0f` in
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L329-L332)
  so MQTT joystick values are parsed as floats, not coerced to int. Also
  clamp to `[-1.0, 1.0]` and reject NaN, mirroring the BLE path's
  `validateAndClampJoystickValue()`.
- [ ] **Opta MQTT reconnect blocks the safety timeout.** Make
  `reconnectMQTT()` non-blocking (one attempt per loop, gated by `millis()`),
  call `stopAllMovement()` immediately on detected disconnect, and run the
  `SAFETY_TIMEOUT` check on every loop iteration regardless of broker
  state. See
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L221-L300).
- [ ] **Browser keyboard control latches motion after key release.** In
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/static/js/controller.js](DESIGN-CONTROLLER/raspberry_pi_web_controller/static/js/controller.js#L177-L181)
  the `activeKeys.size === 0` branch must zero `currentControl.{left_x,
  left_y, right_x, right_y}` — currently it only refreshes the display.
  Also fix the wholesale-overwrite of `currentControl` in the active branch
  (lines 174–186) so simultaneous joystick + keyboard input does not stomp
  each other; use the existing-but-unused `keyboardControl` accumulator.
- [ ] **ESP32 remote keeps publishing stale axes when a Qwiic joystick
  drops off the I²C bus.** Zero the corresponding axes when
  `joystick.connected()` is false in
  [DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino](DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino#L177-L201).
- [ ] **Mode switch and flow-valve jumper are sampled only at boot.**
  Either poll them in `loop()` (calling `stopAllMovement()` on change) or
  update `MODE_SWITCH_WIRING.md` and the README to make clear that switch
  changes require a reboot. See
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L207-L217).

### Boot-time blocking

- [ ] Time-box `setupWiFi()` (e.g. 30 s) in both
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L268-L276)
  and
  [DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino](DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino#L141-L147).
  On the Opta, fall back to BLE if MQTT mode WiFi fails to associate so the
  unit does not hang in `setup()`.

### Authentication / authorization

- [ ] **Web controller has no auth.** Add a login (Flask-Login or HTTP
  basic over TLS), restrict `cors_allowed_origins` from `"*"` to a known
  origin list, and load `SECRET_KEY` from environment / `config.yaml`.
  See [DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py](DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py#L40-L41).
- [ ] **BLE characteristics accept writes from any unpaired device.**
  Require encryption/pairing on the joystick characteristics in
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L621-L646),
  and add a disconnect hook that calls `stopAllMovement()` explicitly.
- [ ] **Hardcoded MQTT credentials and broker IP** in
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L48-L53)
  and
  [DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino](DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino#L26-L30).
  Move to a `secrets.h` / NVS-stored config and stop documenting the literal
  password as the default.

### Configuration / docs vs code

- [ ] **`config/config.yaml` is documented but not consumed** by
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py](DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py#L40-L54).
  Add a YAML loader that overrides the module-level constants (broker, port,
  credentials, camera resolution, secret key), or remove the docs that
  reference it.
- [ ] Default `MQTT_BROKER` in
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py](DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py#L42)
  should be `127.0.0.1` (broker runs on the Pi itself per the install
  guide), not the literal `192.168.1.100`.
- [ ] Doc-vs-code audit pass: README, INSTALLATION_GUIDE, and
  MODE_SWITCH_WIRING describe runtime behaviors (live mode switching, YAML
  config, etc.) that the firmware does not implement.

### Robustness / quality

- [ ] Normalize tank-steering output before clamping in `computeTrackSpeeds()`
  ([DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L342-L355))
  so high `baseSpeed + turnRate` does not produce a "dead zone" near full
  forward where additional turn input has no effect.
- [ ] Update `previousInput` only after deceleration completes, so a new
  input arriving mid-ramp does not cancel the active→zero transition
  detection.
- [ ] Pin `paho-mqtt < 2.0` in
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/requirements.txt](DESIGN-CONTROLLER/raspberry_pi_web_controller/requirements.txt)
  *or* migrate to `mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, ...)` and
  update callbacks.
- [ ] Register `cleanup()` with `atexit` and install a `SIGTERM` handler in
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py](DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py#L385-L399)
  so the camera process is terminated under systemd shutdown.
- [ ] Restart `libcamera-vid` automatically on stream loss in
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py](DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py#L142-L196)
  and notify the browser via SocketIO.
- [ ] Rate-limit / debounce `control_command` events server-side in
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py](DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py#L243-L271).
- [ ] After an `emergency_stop`, server should publish a sticky zero
  `control_command` every ~100 ms for ~1 s to be robust to packet loss
  (mitigates browser `setInterval` throttling on hidden tabs).
- [ ] `serialEvent()` on ESP32 ([DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino](DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino#L353-L380))
  is dead code; the documented "type `stop`" e-stop does not work. Poll
  `Serial.available()` from `loop()` instead.
- [ ] Either remove the placeholder `readBatteryVoltage()` in
  [DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino](DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino#L286-L293)
  or document and calibrate the divider.
- [ ] Detect the placeholder literals `"YOUR_WIFI_SSID"` /
  `"YOUR_WIFI_PASSWORD"` at runtime and refuse to enter MQTT mode (fall
  back to BLE) instead of looping forever in `setupWiFi()`.
- [ ] Migrate from `DynamicJsonDocument` to `StaticJsonDocument` (or
  ArduinoJson 7's `JsonDocument`) for the small fixed-size payloads on both
  firmwares.
- [ ] De-duplicate `controlValve()` and `controlTrack()` in
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L498-L527).
- [ ] Factor the joystick wire format into a shared `lifetrac_protocol.h`
  consumed by both firmwares so the Opta and ESP32 cannot drift on
  deadzone / clamp / type expectations.

### Test coverage

- [ ] Extend [DESIGN-CONTROLLER/test_scripts/mqtt_test.py](DESIGN-CONTROLLER/test_scripts/mqtt_test.py) (or add a
  pytest) that publishes representative float payloads and asserts they
  round-trip through the parsing logic the Opta firmware uses. This would
  catch regressions of the int-coercion class.
- [ ] Wire the MQTT contract test into `ARDUINO_CI` so it runs on PRs.

---

## Future ideas � autonomous routines & mission library

Beyond tele-op, the v25 controller stack reserves source-id `AUTONOMY`
and the MASTER_PLAN already carves out autonomy opcodes. Once GPS/RTK,
IMU dead-reckoning, vision-based obstacle stop, and the autonomy mode
arbitration land, the tractor becomes a programmable mobile hydraulic
robot. This list is the running brainstorm of mission-level routines we
want the autonomy layer to be able to express. None of these are
in-scope for v25 itself � they are the target use cases the autonomy
APIs need to be expressive enough to support.

Each routine should compose from a small set of primitives � *go-to
pose*, *follow path*, *bucket dig*, *bucket dump*, *engage implement*,
*hold posture*, *wait-for-condition* � and run under the same hard
safety envelope as tele-op (PSR E-stop chain, person-alert vision stop,
geofence, link-loss timeout).

### Operator-requested routines

- [ ] **Material movement loop (dig ? haul ? dump).** Operator marks a
  source pile or dig area on the map and a destination (another pile,
  the hopper of a Compressed Earth Block press, a trailer, a spoil
  heap). Tractor drives to the source, executes a bucket-fill cycle
  (curl + crowd until pressure plateau or load cell threshold),
  reverses out, follows the planned path to the destination, and dumps.
  Loops until the source is exhausted, the destination is full (CEB
  hopper level sensor, trailer load cell), the operator pauses it, or
  fuel/battery falls below a return-to-base threshold. Records each
  cycle for cycle-time and yield analytics.
- [ ] **Field pass routine (plow / disc / water / fertilize / seed).**
  Operator selects a field polygon and an implement profile (plow,
  disc, sprayer, spreader, seed drill). Tractor generates a coverage
  pattern (boustrophedon / spiral / contour-following) honouring
  implement width, headland turn radius, no-go zones, and slope limits,
  then executes the pattern with implement engaged. Variable-rate
  application uses a prescription map (e.g. heavier fertilizer on
  low-N zones from a soil map) when one is supplied; otherwise uniform.
- [ ] **Land levelling to target grade.** Operator defines a target
  surface � flat at elevation Z, a planar slope (azimuth + grade %),
  or an arbitrary heightmap. Tractor surveys current grade with the
  bucket-mounted GNSS/laser, computes a cut/fill map, then iteratively
  scrapes high spots into low spots until residual elevation error is
  within tolerance everywhere in the polygon. Pairs naturally with the
  material-movement loop when net cut ? net fill.

### Additional routines worth adding to the same backlog

- [ ] **Trenching along a marked line at constant depth.** For
  irrigation drip lines, electrical conduit, water service, or french
  drains. Operator draws a polyline; tractor follows it with a
  trenching attachment held at depth via bucket-tip GNSS feedback.
- [ ] **Stockpile reshaping / push-up.** Loose material from a dump pile
  pushed up onto an existing stockpile to a target geometry (cone,
  windrow, bermed rectangle). Useful before tarping or for compost
  windrow turning when paired with a turner attachment.
- [ ] **CEB-press tending loop.** A specialisation of the
  material-movement loop with the press as the destination: monitor
  press hopper level over MQTT, refill on demand, fall idle when the
  press is full or paused. Closes the open-source-housing-system loop
  with no human in the bucket cycle.
- [ ] **Hole-grid drilling for tree planting / fence posts / pier
  footings.** Operator defines a grid (rows � columns, spacing,
  azimuth) or imports a planting plan. Tractor drives to each point
  with an auger attachment, drills to a specified depth, withdraws,
  advances. Records actual location of every hole for the as-built map.
- [ ] **Perimeter / contour swale digging.** Follows a survey contour
  line at a specified cross-section to build keyline-style water
  retention features. Reuses the trenching primitive with a wider,
  shallower bucket pose.
- [ ] **Brush / mowing pattern over a polygon.** Same coverage planner
  as the field pass, but with a flail mower or brush hog and a more
  conservative obstacle-stop policy (vision detector tuned for fence
  posts, irrigation risers, animals).
- [ ] **Snow / debris clearing along a route.** Plough or bucket follows
  a defined polyline (driveway, footpath, lane) repeatedly, pushing
  material to a designated spoil zone. Triggered by weather alert or
  manual start.
- [ ] **Rock / debris pickup over a polygon.** Vision detector picks out
  rocks above a size threshold; tractor drives to each, scoops with
  forks or bucket, deposits in a bin or spoil pile. Random-sweep
  pattern with revisit on detector confidence.
- [ ] **Trailer / truck loading to a target weight.** Repeats the
  material-movement loop with the trailer as destination and stops
  when an on-trailer load cell (or estimated bucket cycle count) hits
  the target weight, leaving the operator to drive away.
- [ ] **Geofence patrol / perimeter inspection.** Tractor follows a
  perimeter polyline at low speed with the camera streaming, flagging
  fence breaks, downed trees, washouts, or strange objects to the
  operator for review. Pairs with the existing person-alert pipeline.
- [ ] **Return-to-base on fuel / battery / weather.** Background
  routine, not a job: any active mission yields and the tractor
  drives itself to a configured shelter or charge dock when fuel
  drops below reserve, battery SOC drops below threshold, lightning is
  forecast within N km, or wind exceeds a safety cap. Mission state is
  checkpointed so the operator can resume after refuel/recharge.
- [ ] **Multi-tractor cooperative jobs.** Two LifeTracs sharing the
  same site: one parks at the dig face on the material-movement loop
  while the other shuttles between the loader and the destination.
  Requires the autonomy layer to negotiate non-overlapping path
  reservations and shared safety zones.
- [ ] **Implement auto-engage / auto-disengage at the implement rack.**
  Drive to the rack, align with the target attachment, hydraulically
  couple, drive away. Eliminates the manual swap that gates most of
  the routines above when a job needs more than one implement.
