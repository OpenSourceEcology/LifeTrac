# LoRa Firmware Hardware Test Start Plan (Copilot v1.1)

Date: 2026-05-05
Status: Updated after Gemini 3.1 Pro + Copilot v1.0/v1.1 review pass
Scope: Start bench hardware testing for Method G firmware on Portenta H7 + Max Carrier + Murata CMWX1ZZABZ-078.

> **Target correction (2026-05-05):** The production tractor-node target per [`BUILD-CONTROLLER/01_bill_of_materials.md`](../BUILD-CONTROLLER/01_bill_of_materials.md) is **Portenta X8 (ABX00049) + Max Carrier**, not a bare Portenta H7. The `tractor_h7/` sketch runs on the X8's onboard STM32H747 co-MCU. The `envie_m7` FQBN compile commands below are valid as a **non-production smoke test** against a stock Portenta H7 dev board if one is available, but real hardware testing must use the X8 FQBN (`arduino:mbed_portenta:portenta_x8`) against the patched X8 core. See [`2026-05-05_LoRa_Firmware_Hardware_Test_Blockers_Copilot_v1_0.md`](2026-05-05_LoRa_Firmware_Hardware_Test_Blockers_Copilot_v1_0.md) §6 for the build matrix and [`BUILD_CONFIG.md`](../DESIGN-CONTROLLER/BUILD_CONFIG.md) §10.1 for the FQBN-driven `ARDUINO_PORTENTA_X8` auto-gate that enables the X8-required override TUs (`lp_ticker_override.c`, `us_ticker_override.c`, `sysclock_x8.c`, `software_init_hook`). Add an X8 FQBN compile arm to §4.2/§5 alongside the existing `envie_m7` smoke arm.

## 0. What changed from v1.0

This revision integrates the review suggestions into executable gates:

1. Resolves clock-source ambiguity with source-verified mapping: `clock_source_id == 0` is HSE OK.
2. Adds a mandatory pre-bench runtime-fix gate (or explicit sniffer fallback) for three H7 visibility defects.
3. Hardens Day 0 commands to be location-safe and reproducible.
4. Upgrades hardware safety to hard-stop checks (50 ohm load/antenna, current monitoring).
5. Expands evidence requirements with commit/artifact fingerprints and flash-proof logs.

## 1. Source-verified facts

Checked against current source files:

- `murata_l072/host/host_cmd.c`: BOOT_URC payload order is `{reset_cause, radio_ok, radio_version, protocol_ver, wire_schema_ver, clock_source_id}`.
- `murata_l072/include/platform.h`: `PLATFORM_CLOCK_SOURCE_HSE_OK = 0`, `HSI_FALLBACK = 1`, `MSI_FALLBACK = 2`.
- `tractor_h7/murata_host/mh_runtime.c`: currently reads `radio_ok` from payload byte 4 (incorrect for BOOT_URC), and does not handle `RX_FRAME_URC` or `TX_DONE_URC`.
- `tractor_h7/murata_host/mh_uart_arduino.cpp`: nonblocking read/write are present, but no explicit RTS/CTS handling.
- `tractor_h7/murata_host/mh_stream.c`: unknown frame types increment `reject_unknown_type`.
- `murata_l072/config.h`: default host baud is `921600`.

Routing constraints remain unchanged:

- `Serial1` excluded (X8 link in `tractor_h7.ino`).
- `Serial3` excluded (cell modem path).
- `LIFETRAC_MH_SERIAL` bench candidates: `Serial2`, `Serial4`, `Serial5`.

## 2. Objective

Start hardware testing with controlled risk and clear go/no-go gates:

1. Confirm boot integrity and clock source on real hardware.
2. Confirm H7<->L072 host link on the selected serial route.
3. Confirm two-board TX/RX round-trip reliability under attenuated bench conditions.

## 3. Entry criteria (must be true before Day 1)

### 3.1 Software gates (required)

1. `check_mh_wire_sync.py` passes.
2. `mh_cobs_crc_unit` passes.
3. `mh_stats_vectors` passes.
4. Loopback harness passes (`--transport auto`), with pass line captured in evidence.

### 3.2 Runtime visibility gate (required: choose A or B)

Choose one of the following before power-up:

A) Preferred: patch and regress H7 runtime first.

1. Fix BOOT_URC parser index in `mh_runtime.c` (`radio_ok` from payload byte 1).
2. Add handling for `HOST_TYPE_RX_FRAME_URC` and `HOST_TYPE_TX_DONE_URC` in `runtime_on_frame()`.
3. Add bench-visible URC logging surface (for example a compile-time bench log flag).
4. Run regression vectors showing these paths are exercised and parsed correctly.

B) Fallback: if A is not landed, declare sniffer path as mandatory for Stage A/B/C readout.

1. Use on-carrier `UART_SNIFF` (or external logic/UART capture).
2. Decode raw COBS/inner frames for BOOT_URC, PING echo, RX_FRAME_URC, TX_DONE_URC.
3. Record decoder output into evidence artifacts.

### 3.3 Hardware/flash readiness (required)

1. `LIFETRAC_MH_SERIAL` route is proven and documented (`serial_routing.md`, `pinmap_audit.md`).
2. Flash path is proven on this exact board:
   - OpenOCD/J-Link verify transcript, or
   - `dfu-util -l` evidence plus successful image write/verify log.
3. Two-board inventory is available for Stage C (both fully bring-up capable).
4. Evidence folder exists and includes run metadata template.

## 4. Day 0 prep (90-120 min)

Use a location-safe shell pattern:

```powershell
$repo = (Get-Location).Path
Set-Location $repo
```

### 4.1 Tooling and core setup

```powershell
arduino-cli version
arduino-cli core update-index
arduino-cli core install arduino:mbed_portenta
```

### 4.2 Build and host-gate preflight

```powershell
Set-Location "$repo/LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072"
make all

Set-Location $repo
python LifeTrac-v25/DESIGN-CONTROLLER/tools/check_mh_wire_sync.py

Set-Location "$repo/LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7"
New-Item -ItemType Directory -Force -Path src | Out-Null
Copy-Item -Recurse -Force ../common/lora_proto src/
Copy-Item -Force ../common/shared_mem.h src/
```

### 4.3 Compile preflight (default-off then Method G)

Default-off compile is a hard step:

```powershell
Set-Location $repo
arduino-cli compile --fqbn arduino:mbed_portenta:envie_m7 `
  --build-property "compiler.cpp.extra_flags=-DLIFETRAC_ALLOW_UNCONFIGURED_KEY -DLIFETRAC_ALLOW_STUB_CRYPTO" `
  --build-property "compiler.c.extra_flags=-DLIFETRAC_ALLOW_UNCONFIGURED_KEY -DLIFETRAC_ALLOW_STUB_CRYPTO" `
  LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7
```

Method G compile with chosen serial:

```powershell
Set-Location $repo
arduino-cli compile --fqbn arduino:mbed_portenta:envie_m7 `
  --build-property "compiler.cpp.extra_flags=-DLIFETRAC_ALLOW_UNCONFIGURED_KEY -DLIFETRAC_ALLOW_STUB_CRYPTO -DLIFETRAC_USE_METHOD_G_HOST=1 -DLIFETRAC_MH_SERIAL=Serial<N>" `
  --build-property "compiler.c.extra_flags=-DLIFETRAC_ALLOW_UNCONFIGURED_KEY -DLIFETRAC_ALLOW_STUB_CRYPTO -DLIFETRAC_USE_METHOD_G_HOST=1 -DLIFETRAC_MH_SERIAL=Serial<N>" `
  LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7
```

Replace `<N>` with the selected candidate (`2`, `4`, or `5`).

### 4.4 Evidence folder bootstrap

```powershell
$ev = "$repo/LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-05"
New-Item -ItemType Directory -Force -Path $ev | Out-Null
New-Item -ItemType File -Force -Path "$ev/serial_routing.md" | Out-Null
New-Item -ItemType File -Force -Path "$ev/pinmap_audit.md" | Out-Null
New-Item -ItemType File -Force -Path "$ev/run_manifest.md" | Out-Null
```

## 5. Hardware safety hard-stops (before any TX-capable boot)

1. Never power a TX-capable board without a mounted antenna or 50 ohm load.
2. Use current-limited bench power or monitored USB power for first boot.
3. Confirm common ground on all UART/logic paths.
4. Start Stage C at minimum practical TX power.

Note on TX power control:

- Current H7 runtime does not send CFG_SET commands.
- For first session, use a known low L072 default or rebuild L072 with explicit bench-safe default and record it.

## 6. Day 1 staged execution

### Stage A: single-board flash and boot gate (Board A)

1. Flash L072 via proven path (OpenOCD/J-Link/DFU).
2. Capture BOOT_URC from runtime logger (if patched) or raw sniffer decode (fallback path).
3. Require all of:
   - `clock_source_id == 0`
   - `radio_ok == 1`
   - protocol/wire versions match expected values
4. Send one host `PING_REQ` and require echo success before moving to Stage B.

### Stage B: single-board RX sanity

1. Keep normal boot (RX is auto-armed).
2. Send known-good LoRa frames from a calibrated peer profile.
3. Require:
   - visible `RX_FRAME_URC`
   - rising `radio_rx_ok` stats

### Stage C: two-board round-trip

1. Bring up Board B with same profile.
2. Run one single `TX_FRAME_REQ` smoke packet first.
3. Run 1 Hz cadence for warm-up.
4. Run 5-minute attenuated test and require:
   - TX_DONE success rate >= 99%
   - matching RX_FRAME payload rate >= 99%
   - no persistent growth in `radio_tx_abort_airtime`

## 7. Stop/fail criteria

Immediate stop conditions:

1. `clock_source_id != 0`.
2. recurring `HOST_FAULT_CODE_CLOCK_HSE_FAILED (0x08)`.
3. missing TX_DONE for valid TX requests.
4. no visible RX_FRAME in Stage B despite known-good transmitter profile.

First triage order:

1. Wrong serial route or wrong `LIFETRAC_MH_SERIAL` build.
2. Flash path mismatch or stale firmware artifact.
3. RF path setup issue (load/antenuator/wiring).

## 8. Evidence package (required)

Save in `bench-evidence/T6_bringup_<date>/`:

1. BOOT_URC, PING, STATS, TX_DONE, RX_FRAME decoded logs.
2. Stage C timestamped TX/RX CSV.
3. `serial_routing.md` and `pinmap_audit.md`.
4. Flash proof logs (OpenOCD/J-Link verify or DFU transcript).
5. Build fingerprint set:
   - git SHA
   - `git diff --stat`
   - SHA-256 of L072 and H7 artifacts
   - exact compile commands and selected `LIFETRAC_MH_SERIAL`

## 9. Go/No-Go rule

Go to extended bench only if all are true:

1. Entry criteria passed (including runtime visibility gate A or fallback gate B).
2. Stage A/B/C all pass with no unresolved blocker.
3. Evidence package is complete and reproducible.

Else: hold and open a focused fix tranche with failing gate, logs, and reproduction commands.

## 10. Operator quick order

1. Run Day 0 build and host-gate preflight.
2. Lock serial route and compile both default-off and Method G variants.
3. Prove flash path on this board.
4. Execute Stage A, then B, then C.
5. Archive evidence and make go/no-go call.
