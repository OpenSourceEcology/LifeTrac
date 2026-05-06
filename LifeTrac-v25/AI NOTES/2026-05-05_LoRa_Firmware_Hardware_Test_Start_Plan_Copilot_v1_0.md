# LoRa Firmware Hardware Test Start Plan (Copilot v1.0)

Superseded by: `2026-05-05_LoRa_Firmware_Hardware_Test_Start_Plan_Copilot_v1_1.md`

Date: 2026-05-05
Status: Ready-to-execute draft
Scope: Start bench hardware testing for the new Method G LoRa firmware on Portenta H7 + Max Carrier + Murata CMWX1ZZABZ-078.

## 1. Objective

Start hardware testing with controlled risk, proving three things in order:

1. Board boots cleanly with HSE clock and radio initialized.
2. H7-to-L072 host link is correct for the selected `LIFETRAC_MH_SERIAL`.
3. Two-board LoRa round-trip meets the acceptance threshold.

## 2. Inputs and constraints

Primary references:

- `DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md`
- `DESIGN-CONTROLLER/BUILD_CONFIG.md`
- `DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino`

Hard constraints to honor before bench power-up:

- `Serial1` is excluded (used by X8<->H747 link).
- `Serial3` is excluded (on-carrier cellular modem path).
- Bench candidates for `LIFETRAC_MH_SERIAL`: `Serial2`, `Serial4`, `Serial5`.
- CN2 SWD header is DNP by default; choose either:
  - SWD with soldered CN2 (ST-Link/J-Link), or
  - DFU path if board wiring supports it.

## 3. Entry criteria (must be true before Day 1 bench)

1. Host protocol software gates are green:
   - `check_mh_wire_sync.py` pass
   - `mh_cobs_crc_unit` pass
   - `mh_stats_vectors` pass
   - loopback harness pass (PTY on Linux or TCP on Windows)
2. Chosen serial candidate is documented (`Serial2` or `Serial4` or `Serial5`) with schematic trace notes.
3. Flash path chosen and physically feasible on the specific board (SWD CN2 present or DFU proven).
4. Evidence folder created for this run.

## 4. Day 0 (prep, 60-90 min)

### 4.1 Tooling and artifacts

Run from repository root unless noted.

```powershell
arduino-cli version
arduino-cli core update-index
arduino-cli core install arduino:mbed_portenta
```

Build L072 firmware artifact:

```powershell
Set-Location LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072
make all
```

Optional host-only protocol recheck:

```powershell
Set-Location ../../..
python LifeTrac-v25/DESIGN-CONTROLLER/tools/check_mh_wire_sync.py
```

### 4.2 Choose and lock Method G serial

Pick one candidate (`Serial2`/`Serial4`/`Serial5`) and document rationale in:

- `DESIGN-CONTROLLER/bench-evidence/T6_bringup_<date>/serial_routing.md`
- `DESIGN-CONTROLLER/bench-evidence/T6_bringup_<date>/pinmap_audit.md`

Compile H7 Method G with chosen serial macro:

```powershell
New-Item -ItemType Directory -Force -Path LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/src | Out-Null
Copy-Item -Recurse -Force LifeTrac-v25/DESIGN-CONTROLLER/firmware/common/lora_proto LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/src/
Copy-Item -Force LifeTrac-v25/DESIGN-CONTROLLER/firmware/common/shared_mem.h LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/src/
```

Compile default-off preflight first (optional but recommended), then Method G enabled:

```powershell
arduino-cli compile --fqbn arduino:mbed_portenta:envie_m7 `
   --build-property "compiler.cpp.extra_flags=-DLIFETRAC_ALLOW_UNCONFIGURED_KEY -DLIFETRAC_ALLOW_STUB_CRYPTO" `
   --build-property "compiler.c.extra_flags=-DLIFETRAC_ALLOW_UNCONFIGURED_KEY -DLIFETRAC_ALLOW_STUB_CRYPTO" `
   LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7
```

```powershell
arduino-cli compile --fqbn arduino:mbed_portenta:envie_m7 `
  --build-property "compiler.cpp.extra_flags=-DLIFETRAC_ALLOW_UNCONFIGURED_KEY -DLIFETRAC_ALLOW_STUB_CRYPTO -DLIFETRAC_USE_METHOD_G_HOST=1 -DLIFETRAC_MH_SERIAL=Serial<N>" `
  --build-property "compiler.c.extra_flags=-DLIFETRAC_ALLOW_UNCONFIGURED_KEY -DLIFETRAC_ALLOW_STUB_CRYPTO -DLIFETRAC_USE_METHOD_G_HOST=1 -DLIFETRAC_MH_SERIAL=Serial<N>" `
  LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7
```

Replace `<N>` with the chosen port number.

### 4.3 Evidence folder bootstrap

```powershell
New-Item -ItemType Directory -Force -Path LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05 | Out-Null
New-Item -ItemType File -Force -Path LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05/serial_routing.md | Out-Null
New-Item -ItemType File -Force -Path LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05/pinmap_audit.md | Out-Null
```

## 5. Day 1 (hardware bring-up sequence)

### Stage A: single-board flash + boot gate (Board A)

1. Connect board, antenna/load, and selected debug path.
2. Flash L072:
   - SWD: OpenOCD/J-Link path from runbook
     - Example OpenOCD command:

```powershell
Set-Location LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072
openocd -f openocd/stlink.cfg -f openocd/stm32l0_swd.cfg -c "program build/firmware.elf verify reset exit"
```

   - DFU: runbook DFU path
3. Capture first BOOT_URC window.
4. Gate checks:
   - `clock_source_id == 0` (required)
   - `radio_ok == 1` (required)

If either fails: stop and troubleshoot before any RF test.

### Stage B: single-board RX sanity

1. Keep board in normal boot (RX auto-armed).
2. Transmit known-good LoRa frame from peer.
3. Verify:
   - `HOST_TYPE_RX_FRAME_URC` observed
   - STATS `radio_rx_ok` increments

### Stage C: two-board round-trip (Board A <-> Board B)

1. Prepare Board B with same firmware profile.
2. Board A sends `HOST_TYPE_TX_FRAME_REQ` at fixed cadence.
3. Verify:
   - Board A receives `HOST_TYPE_TX_DONE_URC`
   - Board B receives matching `HOST_TYPE_RX_FRAME_URC`
4. Run 5 minutes over attenuated coax.

Pass thresholds:

- TX_DONE success rate >= 99%
- Matching RX_FRAME payload rate >= 99%
- No persistent growth in `radio_tx_abort_airtime`

## 6. Stop/fail criteria

Immediate stop conditions:

- `clock_source_id == 1` (HSE fallback)
- recurring `HOST_FAULT_CODE_CLOCK_HSE_FAILED (0x08)`
- no UART traffic after Method G enable despite valid flash
- no TX_DONE for valid TX requests

Primary first checks when stop occurs:

1. Serial mapping wrong (`LIFETRAC_MH_SERIAL` candidate mismatch).
2. Flash path incomplete or wrong image.
3. RF path/wiring mismatch (antenna/attenuator/switch path).

## 7. Evidence requirements

Save into `DESIGN-CONTROLLER/bench-evidence/T6_bringup_<date>/`:

1. Boot log with BOOT_URC and early FAULT_URC/STATS.
2. Two-board TX_DONE/RX_FRAME timestamp CSV.
3. `serial_routing.md` and `pinmap_audit.md`.
4. Proof of flash path used (CN2 photo for SWD or DFU enumeration log).

## 8. Go/No-Go decision after first session

Go to extended bench only if all are true:

1. Stage A, B, C pass without unresolved blocker.
2. Evidence package complete and reproducible.
3. No unresolved clock or serial-routing ambiguity remains.

If not, hold and open a focused fix tranche with exact failing gate and artifacts.

## 9. 90-minute quick-start script (operator order)

1. Verify tooling and compile artifacts.
2. Lock `LIFETRAC_MH_SERIAL` candidate and compile Method G build.
3. Flash Board A and run Stage A gate.
4. Run Stage B RX check.
5. Flash Board B and run Stage C two-board check.
6. Archive evidence and decide go/no-go.

## 10. Review & System Architecture Suggestions (Gemini 3.1 Pro)

**Review Date:** 2026-05-05
**Reviewer:** Gemini 3.1 Pro (Preview)

The hardware test start plan provides a solid procedural checklist. However, there are a few critical discrepancies and hardware safety notes to resolve before powering up the bench:

### 1. Resolve the `clock_source_id` Enum Contradiction
*   **The Issue:** Section 6 lists `clock_source_id == 1` as an immediate stop condition ("HSE fallback"), demanding `0` instead. However, the T6 plan (Section 4.2) explicitly stated: *"expect BOOT_URC with clock_source_id=1 (HSE)"*. 
*   **Suggestion:** Audit `host_types.h` immediately. Determine definitively whether `0` or `1` maps to `HOST_CLOCK_SOURCE_HSE` vs `HOST_CLOCK_SOURCE_HSI16` to prevent the operator from aborting a perfect boot sequence.

### 2. Serial Port Tracing and Baud Rate Fallback
*   **The Issue:** This document correctly forbids `Serial3` (which is wired to the cellular modem on the Max Carrier), fully overriding the Tranche 4 specs that initially targeted `Serial3`. You have whittled the candidates down to `Serial2, 4, 5`.
*   **Suggestion:** Ensure `LIFETRAC_MH_SERIAL` mapping is validated with a multimeter/schematic check first. Additionally, the 921600 baud rate might experience framing errors if the physical traces/jumpers you are using are too long or noisy. Add an instruction: *If Stage A UART yields garbage, drop both L072 and H7 to 115200 baud temporarily to rule out signal integrity boundaries.*

### 3. Hardware Safety: The "No-Load" PA Burnout Risk
*   **The Issue:** Step 5.C commands transmitting at potentially high power (up to +20dBm/+22dBm).
*   **Suggestion:** Add a hard warning in Section 5.A: **Never power the Murata SiP without a 50 Ω load or mounted antenna.** Even brief TX bursts without a matched load can permanently degrade or destroy the SX1276 power amplifier.

### 4. Ensure MCO Pin is Accessible
*   **The Issue:** The T6 plan relies heavily on an MCO oscilloscope probe (PA8 on the L072) to physically verify the 2.000 MHz clock out.
*   **Suggestion:** Confirm that the Max Carrier or the Murata SiP breakout actually exposes PA8. If PA8 is buried under the SiP without a test pad, the operator will not be able to execute the Stage A clock safety check, and you will need to rely purely on the URC telemetry.

## 11. Review & Hardware Test Readiness Suggestions (GitHub Copilot v1.0)

**Review Date:** 2026-05-05
**Reviewer:** GitHub Copilot
**Scope:** Checked this start plan against the current `BRINGUP_MAX_CARRIER.md`, `BUILD_CONFIG.md`, `tractor_h7.ino`, `murata_host/mh_runtime.c`, L072 `host_cmd.c`, `platform.h`, `config.h`, and the loopback tooling.

### 11.1 Executive verdict

The plan is close to bench-ready and has the right stage order: software gates, serial-route proof, single-board boot, RX sanity, then two-board TX/RX. Before powering hardware, I would add one short "bench freeze" pass that locks the exact build artifacts, proves the selected serial path electrically, and verifies BOOT_URC from the raw wire bytes rather than from any cached runtime state.

The `clock_source_id` question is resolved in current source: `0` is HSE OK, `1` is HSI fallback, and `2` is MSI fallback. The plan's Stage A gate of `clock_source_id == 0` is therefore correct. Tighten Section 6 to stop on any nonzero `clock_source_id`, not only `1`.

### 11.2 Must-fix before Day 1

1. **Decode BOOT_URC from the raw frame for Stage A.** The L072 BOOT_URC payload is currently `{reset_cause, radio_ok, radio_version, protocol_ver, wire_schema_ver, clock_source_id}`. The current H7 `mh_runtime.c` caches `radio_ok` from payload byte 4, which is actually `HOST_WIRE_SCHEMA_VER`, not the radio flag at byte 1. Until that runtime parser is fixed, Stage A must use a raw COBS/inner-frame decoder, loopback diagnostic, or serial log decoder that prints all six BOOT bytes explicitly.

2. **Make the Day 0 commands location-proof.** The plan says commands run from repo root, but `Set-Location LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072` persists. The later `Set-Location ../../..` lands in `LifeTrac-v25/DESIGN-CONTROLLER`, where `python LifeTrac-v25/DESIGN-CONTROLLER/tools/check_mh_wire_sync.py` is no longer a valid relative path. Use `Push-Location` / `Pop-Location`, or define a `$repo` variable and always return to it before repo-root commands.

3. **Freeze exact software gates and outputs.** The loopback harness now supports both PTY and TCP transports, so the entry criterion can be concrete: build `loopback_driver`, run `murata_host_loopback.py --transport auto --iterations <N>`, and save the pass line. Also note that `check_mh_wire_sync.py` currently checks 45 constants, including the clock-fault constant; do not carry forward older "44 constants" language.

4. **Prove the flash path before the timed bench session.** CN2 is DNP on stock Max Carrier boards, and the DFU path is explicitly board-specific. Treat "flash path chosen" as insufficient unless the operator has already produced either an OpenOCD/J-Link verify transcript or a `dfu-util -l` enumeration plus readback/verify evidence for this exact board.

### 11.3 Hardware safety controls

- Add a pre-power checklist item for a 50 ohm load or antenna on every board before any TX-capable firmware is exercised. The existing Stage A instruction says "antenna/load," but it should be a hard stop item, not a casual setup note.
- Use a current-limited bench supply or monitored USB power for first boot. Record idle current after BOOT_URC and current during the first single TX. Unexpected current is often a faster clue than protocol logs for RF-switch or PA mistakes.
- Start Stage C at minimum practical TX power. The L072 default TX power is not the same as the legacy H7 `LIFETRAC_BENCH_TX_DBM` RadioLib default. Before first RF TX, either issue `CFG_SET tx_power_dbm` to the minimum bench value or document the compiled/default L072 value being used.
- Keep the Method G bench build off a live tractor control stack. Current `tractor_h7.ino` routes `setup()` and `loop()` entirely through `mh_runtime` when `LIFETRAC_USE_METHOD_G_HOST=1`, returning before the legacy shared-memory watchdog, Modbus, arbitration, and telemetry loop. That is fine for radio bring-up, but it is not a production tractor runtime test.

### 11.4 Serial-route and baud suggestions

The `Serial2`/`Serial4`/`Serial5` choice should be proved as an electrical fact, not inferred from name similarity. For each candidate, capture at least one of:

- scope or logic-analyzer trace showing idle-high 3.3 V UART and 921600 8N1 decode,
- on-carrier `UART_SNIFF` capture tied to the schematic net name,
- continuity/path notes from ABX00043 sheet 1/sheet 2 to the H747 Arduino `SerialN` instance.

If Stage A produces garbage UART, first suspect wrong `SerialN`, swapped TX/RX, missing ground, or sniffing the wrong side of the level path. Dropping to 115200 is useful only with a deliberate matching L072 debug build or a proven CFG baud-change path; the current firmware default is `HOST_BAUD_DEFAULT = 921600`, so a one-sided baud change will only add confusion.

### 11.5 Stage refinements

- **Stage A:** Require BOOT_URC payload bytes, protocol version, wire schema version, and radio version in the log. Then send a host `PING_REQ` and require the L072 echo before proceeding to RX. This separates "the L072 booted" from "the H7 request path works."
- **Stage B:** Pin the known-good transmitter profile in the evidence folder: frequency, SF, bandwidth, coding rate, sync word, CRC-on setting, payload bytes, and cadence. RX is already armed after boot; do not introduce any `RX_START` concept.
- **Stage C:** Run one single `TX_FRAME_REQ` first, then a 1 Hz cadence, then the 5-minute run. Log `seq`, `tx_id`, `TX_DONE_URC.status`, and Board B payload bytes. Optional echo should be implemented by the Board B H7 bench runtime sending its own `TX_FRAME_REQ`, not by assuming an L072 firmware echo path.
- Under attenuated coax and conservative cadence, unexpected growth in `radio_tx_abort_lbt` or `radio_tx_abort_airtime` should be investigated, even if the headline TX/RX success rate stays above 99%.

### 11.6 Evidence package additions

Add these to Section 7 so the run can be reproduced later:

- git commit SHA and `git diff --stat` for the firmware used,
- SHA-256 of `firmware.elf`, `firmware.bin`, and the H7 compiled artifact,
- exact `arduino-cli compile` commands and `LIFETRAC_MH_SERIAL` value,
- exact L072 flash command and verify/readback output,
- decoded raw BOOT_URC, PING echo, STATS_URC, TX_DONE_URC, and RX_FRAME_URC logs,
- photos or notes for the load/attenuator setup and serial wiring.

### 11.7 Bottom line

Proceed to hardware only after the raw BOOT_URC decoder, serial-route proof, and flash-path proof are in the evidence folder. The plan's sequence is sound, but the first session should be treated as a bring-up experiment with explicit artifacts, not as a timed acceptance run. Once Stage A and the single-packet Stage C test pass twice in a row on the same setup, the 5-minute two-board threshold becomes meaningful.

## 12. Review & Source-Verified Bring-up Suggestions (GitHub Copilot v1.1)

**Review Date:** 2026-05-05
**Reviewer:** GitHub Copilot (Claude Opus 4.7)
**Scope:** Cross-checked the plan against current source: [`tractor_h7/murata_host/mh_runtime.c`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_runtime.c), [`mh_stream.c`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_stream.c), [`mh_uart_arduino.cpp`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_uart_arduino.cpp), [`murata_l072/host/host_cmd.c`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c), [`BRINGUP_MAX_CARRIER.md`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md), and the T6 plan v1.2.

### 12.1 Verdict

The plan's stage order and acceptance thresholds are sound. However, source inspection reveals **three runtime gaps that will silently break Stage B and Stage C even on perfectly good hardware** — these are H7-side firmware defects, not bench procedure issues, and they must be patched (or worked around with a sniffer) before the first session is meaningful. §11 (Copilot v1.0) flagged the BOOT_URC parser bug; this review confirms it from source and adds two more.

### 12.2 Confirmed runtime defects (must fix or work around)

1. **BOOT_URC parser is misaligned (confirms §11.2.1).** [`mh_runtime.c`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_runtime.c#L40-L41) reads `radio_ok = payload[4]` and `clock_source_id = payload[5]`. The L072 sender at [`host_cmd.c`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c#L57-L70) writes `payload = {reset_cause, radio_ok, radio_version, HOST_PROTOCOL_VER, HOST_WIRE_SCHEMA_VER, clock_source_id}`. Therefore the H7 cache will report `radio_ok = HOST_WIRE_SCHEMA_VER = 1` (always true, masking real radio failures) and `clock_source_id` is correct only by coincidence (byte 5 is the right slot). **Fix:** change line 40 to `payload[1]`. Test path: PTY harness boot vector with `radio_ok=0` from a synthetic L072 mock — this should be a regression test in `bench/h7_host_proto/`.
2. **`HOST_TYPE_RX_FRAME_URC` is not handled in `runtime_on_frame()`.** [`mh_runtime.c`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_runtime.c#L37-L65) switch covers BOOT, FAULT, STATS, PING_REQ, CFG_OK, CFG_DATA, ERR_PROTO — but neither `RX_FRAME_URC` nor `TX_DONE_URC`. The default case returns `false`, which [`mh_stream.c`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_stream.c#L118-L119) translates into `counters.reject_unknown_type++`. **Stage B will appear to fail** (no observable RX surface) and **Stage C will report no TX_DONE** even when the L072 is doing exactly the right thing. **Fix:** add `case HOST_TYPE_RX_FRAME_URC:` and `case HOST_TYPE_TX_DONE_URC:` arms that record the event and (per §12.3) pump it to the bench logger.
3. **Runtime has no operator-visible logging surface.** `mh_runtime.c` mutates internal state but never prints to the H7 USB CDC `Serial` port. The bench operator running this firmware has zero visibility into BOOT_URC bytes, PING echoes, STATS deltas, RX_FRAME arrival, or TX_DONE status without an external UART sniffer on the L072 line. The plan's Stage A "Capture first BOOT_URC window" is therefore not executable from the H7 side as built. **Fix (small):** add a compile-time `LIFETRAC_MH_BENCH_LOG=1` flag that, in `runtime_on_frame()`, emits a one-line hex/decoded summary per URC over `Serial` at 115200. This is the bench-visibility surface called out in T6 plan §6 PR#3 acceptance — it is not yet implemented.

### 12.3 Plan items that need a Stop-the-Line gate

These are entry-criterion uplifts to §3:

- **E5:** [`mh_runtime.c`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_runtime.c) parser bug at line 40 is patched and a PTY regression vector for `radio_ok=0` is committed and green.
- **E6:** `RX_FRAME_URC` and `TX_DONE_URC` are recognized cases in `runtime_on_frame()`, with PTY vectors that drive both and assert the bench-logger output.
- **E7:** Bench-visibility logger (`LIFETRAC_MH_BENCH_LOG`) is wired and produces a parseable line per URC.

Without E5–E7, the plan is executable only with an external L072-side UART sniffer (which the on-carrier `UART_SNIFF` channel can provide — see [`BRINGUP_MAX_CARRIER.md`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md) §1). If the operator is going that route, document it explicitly in §3 as the fallback path.

### 12.4 H7 UART configuration gap (medium-priority)

[`mh_uart_arduino.cpp`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/murata_host/mh_uart_arduino.cpp#L13) calls `serial->begin(baud)` only. The L072 `SERIAL` net is wired with 4-wire HW flow control (RTS/CTS) per ABX00043 sheet 6. The H7 side currently does not enable RTS/CTS, so flow control is effectively disabled end-to-end. At 921600 with 64-68 B STATS frames every 1000 ms plus 500 ms PING traffic, this is fine on a quiet bench. Under heavier traffic (e.g. RX_FRAME bursts during Stage C), the H7 RX FIFO can overrun without backpressure. **Suggestion:** non-blocking — note this as a Stage C watch-item; if `counters.reject_crc` or `reject_length` grows during Stage C without a corresponding `radio_*` counter movement on the L072 side, suspect H7 RX-FIFO overrun, not a wire-level integrity problem.

### 12.5 TX-power and CFG plumbing gap (relates to §11.3)

§11.3 (Copilot v1.0) recommends issuing `CFG_SET tx_power_dbm` to the minimum bench value before Stage C. **`mh_runtime.c` does not implement any CFG_SET emission path** — it only schedules `PING_REQ` (every 500 ms) and `STATS_DUMP_REQ` (every 1000 ms). Two viable paths:

- **(a) Compile-time L072 default:** rebuild L072 firmware with `tx_power_dbm` default lowered to the bench minimum and document the SHA in `bench-evidence/.../l072_tx_power_default.md`. Simpler, no H7-side change.
- **(b) Add bench helper:** extend `mh_stream` with `mh_stream_send_cfg_set(key, value)` and call it once from `mh_runtime_begin()` after BOOT_URC. More flexible, but requires a code change on a path the harness has not exercised.

For first session, recommend path (a). Capture the chosen `tx_power_dbm` in the evidence package.

### 12.6 Day-0 build-recipe corrections

- **Working-directory drift in §4.1/§4.2.** §11.2.2 already flagged this; reinforce by switching all snippets to a `$repo = $PWD` pattern with explicit `Set-Location $repo` between each block, OR convert to one PowerShell script in `LifeTrac-v25/DESIGN-CONTROLLER/tools/bench/day0_compile.ps1` and just reference it.
- **`Copy-Item -Recurse lora_proto` and `Copy-Item shared_mem.h` (§4.2)** — these are legacy-path build glue for the RadioLib build of `tractor_h7.ino`. The Method G build path (`LIFETRAC_USE_METHOD_G_HOST=1`) routes `setup()`/`loop()` through `mh_runtime` and skips the legacy code that consumes `lora_proto`/`shared_mem.h`, but the `.ino` file still includes those headers unconditionally for the disabled-arm code. The copies are still required for the build to compile. **Suggestion:** add a one-line comment in the script explaining *why* the copies are needed (the `#if/#else` arm still parses).
- **Default-off preflight build is the correct discipline** (proves S1 byte-identical default-off path before turning Method G on). Make this a hard step, not "recommended".

### 12.7 Two-board enumeration (relates to Stage C)

Stage C implicitly requires two complete bring-up kits. Make this an explicit BOM line in §2 inputs:

- 2× Portenta H7 + ABX00043 Max Carrier
- 2× chosen flash path (2× soldered CN2, 2× DFU-proven, or one of each — but document)
- 2× 50 Ω load + antenna
- coax + 20-40 dB attenuator chain rated for the chosen TX power
- bench supply or two USB ports with current monitoring (per §11.3)

If only one full kit is available on Day 1, defer Stage C to Day 2 explicitly rather than half-running it.

### 12.8 RX_START is a non-existent concept (cross-reference)

§5 Stage B currently says "Keep board in normal boot (RX auto-armed)" which is correct and matches L072 behavior — there is no `HOST_TYPE_RX_START` opcode in [`host_types.h`](LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h). §10.4 (Gemini) and §11.5 (Copilot v1.0) both reinforced this. Keep this clarity; do not let a future revision re-introduce `RX_START` language.

### 12.9 Suggested §6 sharpening

Replace the current §6 line "`clock_source_id == 1` (HSE fallback)" with:

> `clock_source_id != 0` (HSE did not lock — HSI/MSI fallback or unknown source). Stop and capture full BOOT_URC payload bytes before retrying.

This matches §11.1's clarification that `0=HSE`, `1=HSI`, `2=MSI`, and avoids accepting a degraded clock just because the byte is not `1`.

### 12.10 Bottom line

**The plan is procedurally good but the H7 firmware is not yet ready to execute it cleanly.** Three small, well-scoped patches to `mh_runtime.c` (BOOT byte index, RX/TX URC cases, bench logger) close the visibility gap and turn this from a "needs an external sniffer to interpret anything" exercise into the self-contained bench session the plan describes. Land those patches with PTY regression vectors in `bench/h7_host_proto/` first, then run Day 0 / Day 1 as written.

One-line summary for the next reviewer: **don't power up until §12.2 items 1-3 are patched and PTY-regressed, or until §3 explicitly adopts the on-carrier `UART_SNIFF` fallback as the official Stage A/B/C readout path.**
