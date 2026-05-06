# LoRa Firmware Hardware Test Blockers (Copilot v1.0)

Date: 2026-05-05
Status: Blocker A reframed and patched at source; Blockers B & C remain.
Scope: Current blockers to begin bench hardware testing of Method G firmware on the production tractor-node target (Portenta X8 + Max Carrier + Murata CMWX1ZZABZ-078).

> **Update 2026-05-05 (post-fix v2):** The production tractor-node target is **Portenta X8 (ABX00049) + Max Carrier** per [`BUILD-CONTROLLER/01_bill_of_materials.md`](../../BUILD-CONTROLLER/01_bill_of_materials.md) and [`02_tractor_node_assembly.md`](../../BUILD-CONTROLLER/02_tractor_node_assembly.md) — the `tractor_h7/` sketch name refers to the X8's onboard STM32H747 co-MCU, not a standalone Portenta H7. The four override translation units (`lp_ticker_override.c`, `us_ticker_override.c`, `sysclock_x8.c`, `software_init_hook` in `tractor_h7.ino`) are **required** for the X8 build because the X8 ships with a patched `libmbed.a` (`SetSysClock` renamed to `SetSysClock_HSE_disabled`, etc.) that depends on them. They are now auto-gated on `ARDUINO_PORTENTA_X8` (set by the X8 FQBN), with `LIFETRAC_TARGET_X8` accepted as a manual override. The link collisions the operator hit therefore mean the local `arduino-cli` is using the **stock** `arduino:mbed_portenta` core, not the patched X8 variant. **Resolution path:** install the patched X8 core (per [`BUILD-CONTROLLER/05_firmware_installation.md`](../../BUILD-CONTROLLER/05_firmware_installation.md)) and re-run the X8 FQBN compile. The stock `envie_m7` FQBN is now also a valid non-production smoke-test path (link succeeds because the overrides stay inactive).

## 1. Executive Status

The Method G host/runtime tranche is functionally validated at host level (protocol vectors + Windows TCP loopback), but full hardware-test readiness is currently blocked by one primary build-system issue and two dependent readiness gaps.

## 2. Active Blockers

### Blocker A (Primary): Local Portenta full-sketch link collisions

**STATUS: REFRAMED + PATCHED at source (2026-05-05).** The production target is Portenta X8 + Max Carrier; the four override TUs are *required* by the X8 patched `libmbed.a`, not optional. The link collisions mean the local `arduino-cli` is using the **stock** `arduino:mbed_portenta` core instead of the patched X8 variant. **Two valid resolution paths:**

1. **(Production path — required for actual X8 hardware testing)** Install the patched X8 core per [`BUILD-CONTROLLER/05_firmware_installation.md`](../../BUILD-CONTROLLER/05_firmware_installation.md) and compile against the X8 FQBN. The overrides auto-activate via `ARDUINO_PORTENTA_X8`.
2. **(Smoke-test path — stock H7 envie_m7 FQBN)** Compile against `arduino:mbed_portenta:envie_m7` with the stock core; the overrides stay inactive and the link succeeds against stock `libmbed.a`. **This binary will NOT run on production X8 hardware** (no patched clock path) but is useful for Method G protocol-layer smoke tests on a bare Portenta H7 dev board if one is available.

What is blocked:
- Local `arduino-cli compile` for `firmware/tractor_h7` does not complete link stage when the wrong core is paired with the wrong FQBN.

Observed collision set:

Observed collision set:

Observed collision set:
- `software_init_hook`
- `HAL_GetREVID`
- `HAL_GetDEVID`
- `lp_ticker_*` override symbols
- `us_ticker_*` override symbols
- `SetSysClock`
- `SystemCoreClockUpdate`

Impact:
- Cannot use local full sketch compile as a go/no-go signal for bench start.
- Cannot close Method G compile preflight matrix on this workstation/toolchain state.

Unblock criteria:
1. Isolate why duplicate low-level/system symbols are being linked simultaneously in local recipe.
2. Align local build recipe with known-good CI/expected object inclusion.
3. Re-run full compile preflight (default, Method G, Method G + bench log).

**Verification (operator next step):** confirm which Arduino core is installed (`arduino-cli core list`). If only `arduino:mbed_portenta` is present and the goal is real X8 hardware testing, install the patched X8 core. If only stock-H7 hardware is on the bench right now, recompile against the `envie_m7` FQBN and document this as a non-production smoke run.

### Blocker B (Dependent): Method G compile matrix not yet closed end-to-end locally

What is blocked:
- Full local compile matrix for `tractor_h7` remains incomplete because Blocker A aborts link.

Required closure set:
1. Default compile path (legacy radio path) passes.
2. Method G compile with `LIFETRAC_USE_METHOD_G_HOST=1` and `LIFETRAC_MH_SERIAL=<chosen serial>` passes.
3. Optional Method G bench-log compile with `LIFETRAC_MH_BENCH_LOG=1` passes.

Impact:
- Hardware test plan entry criteria for deterministic firmware artifact generation are not fully satisfied locally.

### Blocker C (Operational): Bench serial-route and flash-path proof still needs board-specific evidence package

What is blocked:
- Hardware-test launch packet requires board-specific proof artifacts (serial-route proof + flash-path proof) prior to Day 1 run.

Required artifacts:
1. Selected `LIFETRAC_MH_SERIAL` route (`Serial2`/`Serial4`/`Serial5`) with schematic and/or probe evidence.
2. Confirmed flash path proof for the exact board under test (SWD or DFU evidence).

Impact:
- Even with host regressions green, bench execution should not start until path proof is captured to avoid wiring/route false negatives.

## 3. What Is Not Blocking

The following gates are currently green and should be treated as completed, not blockers:

1. Host-wire constant sync gate.
2. COBS/CRC unit tests.
3. STATS parser vectors.
4. Runtime-health vectors (BOOT/TX_DONE/RX_FRAME/STATS/rejects).
5. Windows TCP loopback integration run.

## 4. Recommended Unblock Order

1. Resolve local link-collision set (Blocker A).
2. Re-run and close all three local compile preflights (Blocker B).
3. Finalize serial-route and flash-path evidence packet for target boards (Blocker C).
4. Re-evaluate hardware test start readiness and issue explicit go/no-go.

## 5. Bottom Line

As of this note, hardware test start should remain on hold. The protocol/runtime layer is validated, but full bench readiness depends on clearing the local link-collision blocker and closing compile + board-path evidence gates.

## 6. Update 2026-05-05: Blocker A Source Fix Summary (v2 — X8-correct)

Files modified:

* [`lp_ticker_override.c`](../../DESIGN-CONTROLLER/firmware/tractor_h7/lp_ticker_override.c) — wrapped in `#if defined(ARDUINO_PORTENTA_X8) || defined(LIFETRAC_TARGET_X8) ... #endif`
* [`us_ticker_override.c`](../../DESIGN-CONTROLLER/firmware/tractor_h7/us_ticker_override.c) — same gate
* [`sysclock_x8.c`](../../DESIGN-CONTROLLER/firmware/tractor_h7/sysclock_x8.c) — same gate
* [`tractor_h7.ino`](../../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) — `software_init_hook` definition wrapped with the same gate
* [`x8_no_usb_main.cpp`](../../DESIGN-CONTROLLER/firmware/tractor_h7/x8_no_usb_main.cpp) — existing `LIFETRAC_X8_NO_USB_SERIAL` gate now also accepts `ARDUINO_PORTENTA_X8` and `LIFETRAC_TARGET_X8`
* [`BUILD_CONFIG.md`](../../DESIGN-CONTROLLER/BUILD_CONFIG.md) — §10.1 rewritten to identify X8 as the production target and document the FQBN-driven auto-gate

Build-recipe matrix:

| Target | FQBN | Core | Override TUs | Use case |
|---|---|---|---|---|
| **Production tractor node** | `arduino:mbed_portenta:portenta_x8` | **Patched X8 core** | Active (auto, via `ARDUINO_PORTENTA_X8`) | Real bench/field testing on Portenta X8 + Max Carrier |
| Smoke test on bare H7 dev board | `arduino:mbed_portenta:envie_m7` | Stock `arduino:mbed_portenta` | Inactive | Method G protocol-layer compile/link sanity only — NOT a substitute for X8 testing |
| Cross-FQBN compile check | any | any | Forced active by `-DLIFETRAC_TARGET_X8` | CI / dev workstation that wants to validate the X8 code path without the X8 FQBN |

Not validated locally: `arduino-cli` is not on this workstation's PATH, so neither FQBN compile could be exercised in this session. Operator must run the production X8 FQBN compile first; the link transcript belongs in `bench-evidence/T6_bringup_<date>/compile_preflight.log`.

## 7. The real Blocker A right now

Is not the source code — it is the **toolchain**. Confirm:

1. `arduino-cli core list` shows the patched X8 core (or document which X8-targeted core/FQBN combination this workstation will use).
2. The X8 FQBN compile produces a valid binary (no link errors).
3. The X8 `dfu-util`/SWD flash path is proven on the actual production board.

Until that is done, hardware testing on real X8 hardware cannot start regardless of how green the host-side regressions are.
