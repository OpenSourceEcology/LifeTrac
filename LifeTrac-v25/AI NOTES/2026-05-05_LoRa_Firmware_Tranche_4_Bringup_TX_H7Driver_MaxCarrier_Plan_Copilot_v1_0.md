# LoRa Firmware — Tranche 4 Bring-up Plan
### C3 TX path · BOOT_URC reset cause · H7 minimal host driver · Cross-compile gate · Max Carrier flashing & bench bring-up

**Author:** GitHub Copilot (Claude Opus 4.7)
**Version:** v1.0
**Date:** 2026-05-05
**Predecessors:**
- [2026-05-05_LoRa_Firmware_Next_Tranche_Implementation_Plan_Copilot_v1_0.md](2026-05-05_LoRa_Firmware_Next_Tranche_Implementation_Plan_Copilot_v1_0.md)
- [2026-05-05_LoRa_Firmware_Tranche_2_CFG_Modes_Stats_Plan_Copilot_v1_0.md](2026-05-05_LoRa_Firmware_Tranche_2_CFG_Modes_Stats_Plan_Copilot_v1_0.md)
- [2026-05-05_LoRa_Firmware_Tranche_3_CFG_Policies_RX_Wiring_CI_Plan_Copilot_v1_0.md](2026-05-05_LoRa_Firmware_Tranche_3_CFG_Policies_RX_Wiring_CI_Plan_Copilot_v1_0.md)

**Scope:** Plan only. Closes the gap between "host-side static checks pass" and "Max Carrier on the bench, transmitting and receiving end-to-end." This tranche is **the bring-up tranche** — every item below is a hard prerequisite for the first power-on. After Tranche 4 lands, the L072 can ping itself through a second Max Carrier, and the H7 can drive it.

**Authoritative references:**
- [02 Firmware Architecture Plan](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md)
- [04 Hardware Interface and Recovery](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) §2.2 (TX flow), §2.4 (boot/reset), §5 (recovery)
- [05 Method G Review](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md)
- SX1276 datasheet §4.1.7 (LoRa TX flow), §6.4 (RegIrqFlags), §5.4.3 (PA_BOOST)
- STM32L0 Reference Manual RM0367 §6.3.3 (RCC_CSR reset cause flags), §39.7 (USART)
- Portenta H7 + Max Carrier schematic (vendor: Arduino Pro)

---

## 0. Honest pre-bench gap audit

What landed (Tranches 1–3, recap):
- ✓ Memory map locked, `make check` enforces invariants.
- ✓ Host protocol B1 set: PING/VER/UID/RESET/STATS/REG_R/REG_W (allowlisted).
- ✓ DMA HT/TC + IDLE servicing on USART2.
- ✓ B2 CFG_SET/GET RAM-backed with policy-locked validators and host contract tests.
- ✓ C1 `sx1276_modes` is sole owner of `RegOpMode`, CI-enforced.
- ✓ C2 RX path: `sx1276_rx_arm/service`, RegIrqFlags decision tree, `RX_FRAME_URC`, `radio_rx_ok` + `radio_crc_err` wired to real call sites.
- ✓ Reserved STATS slots 11–15 emit real values (rx_ok/crc_err live; tx_ok/tx_abort_lbt still 0).
- ✓ GHA `firmware-l072-static-checks` runs `make check` + `check-opmode-owner` + `check-cfg-contract`.

**What blocks bench bring-up** (the seven items this tranche fixes):

| # | Gap | Effect on bench |
|---|---|---|
| **G1** | No L072 TX path; `HOST_TYPE_TX_FRAME_REQ (0x10)` declared, never handled | Cannot transmit. Two-radio loopback impossible. |
| **G2** | `BOOT_URC.reset_cause` hard-coded to `0` | Cannot distinguish power-on from IWDG bite from soft reset on first power-up. Brick recovery becomes guesswork. |
| **G3** | No `TX_DONE_URC (0x90)` emitter | H7 has no completion signal even after G1 lands. |
| **G4** | No `FAULT_URC (0xF1)` emitter | First-bench faults are silent. |
| **G5** | `make all` (cross-compile) has never run in CI; no proof the ELF links, fits 180 KB APP, or boots | First flash attempt could fail at link, at boot, or silently overrun a region. |
| **G6** | No H7-side host driver (COBS+CRC16+req/URC) | Even if the L072 boots, nobody can talk to it. |
| **G7** | No documented Max Carrier flash procedure (SWD wiring, ST-Link, openocd config) | First bring-up session burns hours on cabling. |

This tranche fixes all seven. Order matters: G5 first (so we have a build), G2+G4 piggyback on existing files, G1+G3 together (TX is meaningless without `TX_DONE_URC`), G6 in parallel with G1, G7 last as the human-facing runbook.

---

## 1. Sequencing (six PRs)

| # | Branch | Lands | Why this order |
|---|---|---|---|
| 1 | `chore/ci-cross-compile` | `make all` runs in CI under `arm-none-eabi-gcc`; size report posted; required check | Must catch link/region failures **before** any C3 code is written. Provides the cross-compile baseline subsequent PRs are diffed against. |
| 2 | `feat/boot-reset-cause-and-fault-urc` | G2 + G4: `RCC_CSR` decoded into `BOOT_URC.reset_cause`, `FAULT_URC` emitter + `Hard_Fault` / `Default_Handler` plumbing | Small, additive, no dependencies. Brings the existing `BOOT_URC` and reserved `FAULT_URC` to real values so PR #3's TX path and PR #6's bench session both have observability from the first boot. |
| 3 | `feat/radio-tx-c3` | G1 + G3: `radio/sx1276_tx.c`, `host_cmd::handle_tx_frame`, `TX_DONE_URC`, fills `radio_tx_ok` STATS slot, sole-writer guard | Core firmware capability. Mirrors C2 structure exactly; reuses `sx1276_modes_to_tx`. |
| 4 | `feat/h7-host-driver-min` | G6: `tractor_h7/src/murata_host/` library — COBS, CRC16-CCITT, frame builder, URC parser, req/resp pump on Serial3 @ 921600. Adds H7-side `loraHostBegin()`, `loraHostPing()`, `loraHostGetStats()`, `loraHostTxFrame()`. | Independent of #3 in code; depends on #3's wire types being final. Can be developed against the L072 by another developer in parallel and PR-merged after #3. |
| 5 | `chore/h7-host-driver-tests` | Host-CI Python or C unit tests for the H7 driver's COBS+CRC layer (no Arduino dep). Adds a Python PTY harness `tools/murata_host_loopback.py` that simulates the L072. | Locks the wire so the bench session validates *radio* behavior, not framing bugs. |
| 6 | `docs/max-carrier-bringup` | G7: `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md` — SWD pinout, openocd config, flash recipe, three-stage bench procedure (single-board → known-good TX source → two-board ping-pong). | Last because it references all prior PRs' artifacts. |

PRs #2, #3, #4 can land in parallel after #1. #5 must precede the first bench session. #6 is a docs PR and can follow within the same merge train.

---

## 2. PR #1 — Cross-compile gate in CI

### 2.1 Why this is non-negotiable

[Makefile](../DESIGN-CONTROLLER/firmware/murata_l072/Makefile) defines `all` (full link via `arm-none-eabi-gcc`) but **CI has never run it**. The first time we discover the linker can't find a symbol, or that `.text` overflowed APP, or that the preprocessed `.ld` mis-substituted a constant, must not be on the bench bench at 11 PM with a Max Carrier on the desk. Run it in CI on every PR.

### 2.2 GHA changes

Extend the existing [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml) `firmware-l072-static-checks` job (or split into a new `firmware-l072-cross-compile` job — see §2.4).

```yaml
firmware-l072-cross-compile:
  runs-on: ubuntu-latest
  name: L072 cross-compile (arm-none-eabi-gcc)
  needs: firmware-l072-static-checks
  steps:
    - uses: actions/checkout@v4
    - name: Install ARM toolchain
      run: |
        sudo apt-get update
        sudo apt-get install -y gcc-arm-none-eabi binutils-arm-none-eabi
        arm-none-eabi-gcc --version
    - name: make all
      working-directory: LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072
      run: make all
    - name: Verify region budgets
      working-directory: LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072
      run: |
        # Hard fail if .text + .data exceed APP region (180 KB) or SRAM exceeds 17.5 KB
        # (20 KB total minus 2.5 KB stack reserve per memory_map.h).
        arm-none-eabi-size -A -d build/firmware.elf | tee build/size.txt
        python3 tools/check_size_budget.py build/firmware.elf
    - name: Upload ELF + map
      uses: actions/upload-artifact@v4
      with:
        name: l072-firmware-${{ github.sha }}
        path: |
          LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/build/firmware.elf
          LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/build/firmware.bin
          LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/build/firmware.hex
          LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/build/firmware.map
          LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/build/size.txt
        retention-days: 30
```

### 2.3 New helper: `tools/check_size_budget.py`

Tiny Python (no deps) that parses `arm-none-eabi-size -A` and asserts:

```
APP region (FLASH .isr_vector + .text + .rodata + .data initializers) <= 180 KB
RAM (.data + .bss + heap) <= (RAM_SIZE - STACK_RESERVE) = 17.5 KB
```

Constants pulled from `include/memory_map.h` via the same preprocessor invocation `make check` already uses, so a memory-map change automatically updates the budget. **Single source of truth preserved.**

Failure prints: section sizes, the offending region, and the headroom we would have needed. Exit non-zero.

### 2.4 Single job vs. split

**Recommendation: split.** Keeps the existing `firmware-l072-static-checks` (host-only, ~5 s) as a fast pre-gate that reviewers see immediately. Cross-compile (~30 s incl. apt install — cache the toolchain) lives in its own job that depends on static checks. Reviewers don't pay the cross-compile latency to see a CFG-contract failure.

### 2.5 Toolchain version pinning

Use Ubuntu's `gcc-arm-none-eabi` (currently 13.2 on `ubuntu-latest`/24.04, matching the user's stated build target). **Pin the runner to `ubuntu-24.04`** (not `ubuntu-latest`) so a future runner-image bump cannot silently change toolchain version. Document the pinning + version in [README.md](../DESIGN-CONTROLLER/firmware/murata_l072/README.md) §"Toolchain."

### 2.6 Acceptance gates

1. PR opens, `firmware-l072-cross-compile` runs green, posts `size.txt` artifact showing `.text` well under 180 KB and SRAM under 17.5 KB.
2. Intentionally bumping `MM_APP_SIZE` down to 4 KB in a throwaway branch makes `check_size_budget.py` fail with a clear over-budget message.
3. The artifact contains a valid `firmware.bin` and `firmware.hex` of plausible size (between 8 KB and 180 KB).
4. Branch protection lists the new check as required.

### 2.7 Risks

- **Toolchain version skew across maintainers.** PRs built on a 14.x toolchain locally vs. CI's 13.2 may have ~2% size deltas. Acceptable; the budget has plenty of headroom. Worth noting in README.
- **Headers under `arm-none-eabi-gcc` strict mode catch latent warnings the host CC missed.** Expect a small initial round of `-Werror` cleanups. That's the point of running the gate now.

---

## 3. PR #2 — `BOOT_URC.reset_cause` (G2) + `FAULT_URC` emitter (G4)

### 3.1 Reset-cause decoding

Today [host/host_cmd.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c) emits `BOOT_URC` with `reset_cause = 0 TODO`. STM32L0 RCC_CSR (`0x40021094`) latches the cause in bits 24–31. Decode at boot **before** any other init touches RCC_CSR, then clear via `RMVF`:

```c
// hal/platform.c — new function
typedef enum {
    RESET_CAUSE_UNKNOWN  = 0x00U,
    RESET_CAUSE_LPWR     = 0x01U,   // RCC_CSR bit 31
    RESET_CAUSE_WWDG     = 0x02U,   // bit 30
    RESET_CAUSE_IWDG     = 0x03U,   // bit 29  ← brick recovery indicator
    RESET_CAUSE_SOFT     = 0x04U,   // bit 28
    RESET_CAUSE_POR_BOR  = 0x05U,   // bit 27 (POR/PDR/BOR)
    RESET_CAUSE_PIN      = 0x06U,   // bit 26 (NRST low)
    RESET_CAUSE_OBL      = 0x07U,   // bit 25 (option byte loader)
    RESET_CAUSE_FW       = 0x08U,   // bit 24 (firewall)
} reset_cause_t;

reset_cause_t platform_reset_cause_take(void);  // reads + clears, idempotent
```

The decoder examines RCC_CSR exactly once at the *very first* line of `main()` (before `safe_mode_listen()`), caches the value in a `static` in `platform.c`, then writes `RCC_CSR |= RCC_CSR_RMVF` to clear. `safe_mode_listen()` may consult `platform_reset_cause_take()` to decide whether to enter safe mode (e.g. on three consecutive `RESET_CAUSE_IWDG` boots — Phase 5 work, but the cause is needed now to feed it).

`host_cmd_init` reads `platform_reset_cause_take()` (returns the cached value, doesn't re-read RCC_CSR) and feeds it into `BOOT_URC` byte 0.

### 3.2 `FAULT_URC` emitter

Wire format (`HOST_TYPE_FAULT_URC = 0xF1`):
```
+-------+-----+-------+--------+--------+--------+--------+
| code  | sub | pc    | lr     | psr    | bfar   | up_ms  |
| u8    | u8  | u32LE | u32LE  | u32LE  | u32LE  | u32LE  |
+-------+-----+-------+--------+--------+--------+--------+
       (24 bytes)
```

`code` taxonomy (new in [host_types.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h)):

| `code` | Name | Source |
|---|---|---|
| `0x01` | HARDFAULT | Cortex-M0+ HardFault_Handler |
| `0x02` | NMI | NMI_Handler |
| `0x03` | RADIO_INIT_FAIL | `sx1276_init()` returned false |
| `0x04` | RADIO_OPMODE_DRIFT | `HOST_DEBUG_OPMODE_GUARD` mismatch (see [sx1276_modes.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_modes.c)) |
| `0x05` | RX_CRC_DISABLED | `sx1276_rx_arm` found CRC-on-payload bit clear |
| `0x06` | HOST_DMA_OVERRUN | DMA TE counter rising |
| `0x07` | STACK_GUARD | (Phase 5 — placeholder) |

`sub` is a code-specific sub-discriminator (e.g. for HARDFAULT, the lower 8 bits of `SCB_HFSR`).

**Two emission paths:**

1. **Synchronous** from main-loop code: `host_cmd_emit_fault(code, sub)` — builds the URC payload from the call site (PC/LR/PSR are zero — they're for the async path), pushes through `host_uart_send_urc`. Used for `RADIO_INIT_FAIL`, `RX_CRC_DISABLED`, `HOST_DMA_OVERRUN`.

2. **Asynchronous from a Cortex-M0+ exception handler:** `HardFault_Handler` is rewritten in `startup.c` as a naked function that captures `MSP`/`PSP`, extracts stacked PC/LR/PSR, writes them into a `static volatile fault_record_t s_pending_fault`, raises `s_fault_pending = true`, then **does NOT return** — sits in a tight loop waiting for the watchdog (M0+ has no return-from-fault contract anyway). The IWDG bites within `IWDG_RUN_WINDOW_MS`, the chip resets, and on the next boot `BOOT_URC.reset_cause = RESET_CAUSE_IWDG` plus an immediate `FAULT_URC` from a new `platform_fault_replay_on_boot()` (which checks a magic value in a backup register or in a no-init RAM section that survives reset).

   STM32L0 has 5 backup registers in RTC (`RTC_BKPxR`). Reserve `RTC_BKP0R..RTC_BKP4R` for fault replay: magic word `0xFA075DEC` in BKP0, then code/sub/PC/LR in BKP1..BKP4. RTC BKP survives reset but not power loss — perfect for "fault → IWDG → reset → replay" but not for "yanked the plug." That's acceptable; pulled-plug faults aren't observable anyway.

### 3.3 Files touched

- New: `include/platform_fault.h`, `include/reset_cause.h`
- Modified: `hal/platform.c` (RCC_CSR decode, RTC BKP fault stash/replay, `cpu_get_psr`/`cpu_get_pc_from_msp` helpers)
- Modified: `startup.c` (HardFault_Handler rewrite, NMI_Handler stub that also replays via BKP)
- Modified: `host/host_cmd.c` (synchronous emitter, `BOOT_URC.reset_cause` populated, fault-replay-on-boot call inserted right after `BOOT_URC` is sent)
- Modified: `radio/sx1276_rx.c` (replace its current "return false on CRC-off" with `host_cmd_emit_fault(FAULT_RX_CRC_DISABLED, 0)` and refuse to arm)
- Modified: `radio/sx1276.c` (`sx1276_init` failure path emits `RADIO_INIT_FAIL`)
- Modified: `host/host_uart.c` (DMA TE handler increments a flag the main loop converts to `HOST_DMA_OVERRUN` once per second max — rate-limited to avoid flood)

### 3.4 Acceptance gates

1. Cold power-up → `BOOT_URC.reset_cause == RESET_CAUSE_POR_BOR`.
2. Press NRST → `BOOT_URC.reset_cause == RESET_CAUSE_PIN`.
3. Send `RESET_REQ (0x03)` → next boot's `BOOT_URC.reset_cause == RESET_CAUSE_SOFT`.
4. Force IWDG by busy-looping past `IWDG_RUN_WINDOW_MS` → `BOOT_URC.reset_cause == RESET_CAUSE_IWDG`.
5. Trigger a deliberate hard fault (write to `0x00000000`) → IWDG bites → `BOOT_URC.reset_cause == RESET_CAUSE_IWDG`, immediately followed by `FAULT_URC{code=0x01 HARDFAULT, sub=SCB_HFSR_low8, pc=<address of bad write>}`.
6. Pulling the modem CRC-on bit low (in a debug build) and arming RX → `FAULT_URC{code=0x05 RX_CRC_DISABLED}`, `STATS_URC.radio_state == STANDBY` (refused to enter RX).
7. `_Static_assert(sizeof(fault_urc_payload_t) == 24)`.

### 3.5 Risks

- **Backup-register write requires RTC clock + DBP bit.** `platform_clock_init` already enables PWR; ensure `PWR_CR_DBP` is set and `RCC_CSR_RTCEN` is not required for BKP register *writes* on L0 — verify against RM0367 §27.4. (On L0, BKP is in RTC peripheral — the LSE/LSI clock enable is required.)
- **Naked HardFault handler stack discipline.** Writing it in C requires `__attribute__((naked))` plus inline asm to extract MSP. Test on QEMU first (`qemu-system-arm -M netduinoplus2` — close enough, M3 vs M0+ stack frames are identical for the relevant bits) before bench.
- **Fault replay loop.** If the fault re-fires immediately after replay, we get an infinite reset loop. Mitigate: BKP1 also stores a `fault_replay_count`; after 3 consecutive replays without a successful PING, enter safe mode (G7 work but stub the gate now).

---

## 4. PR #3 — C3 TX path (G1 + G3)

### 4.1 Mirror of C2 structure

| C2 (RX) | C3 (TX) |
|---|---|
| `radio/sx1276_rx.c` | `radio/sx1276_tx.c` |
| `sx1276_rx_arm/disarm/service` | `sx1276_tx_begin/poll` |
| `host_stats_radio_rx_ok` | `host_stats_radio_tx_ok` |
| `RX_FRAME_URC (0x91)` | `TX_DONE_URC (0x90)` |
| `check-rx-counter-owner` | `check-tx-counter-owner` |

### 4.2 SX1276 TX flow (datasheet §4.1.7)

```
caller has payload[N], N <= 255
1. ensure modem in STANDBY (sx1276_modes_to_standby)
2. write RegFifoTxBaseAddr (0x0E) = 0x00
3. write RegFifoAddrPtr     (0x0D) = 0x00
4. write RegPayloadLength   (0x22) = N
5. burst-write RegFifo      (0x00) <- payload, N bytes
6. clear RegIrqFlags        (0x12) = 0xFF
7. (LBT gate — Tranche 5)
8. sx1276_modes_to_tx        (also flips RF switch to TX path)
9. wait for DIO0 rising edge (TxDone)
10. read RegIrqFlags, verify TxDone bit
11. clear TxDone in RegIrqFlags
12. sx1276_modes_to_standby  (release PA)
13. host_stats_radio_tx_ok()
14. emit TX_DONE_URC{ tx_id, time_on_air_us, tx_power_dbm, status }
```

### 4.3 API

```c
typedef struct sx1276_tx_request_s {
    uint8_t  tx_id;          // echoed in TX_DONE_URC for H7 correlation
    uint8_t  payload[256];
    uint8_t  length;
} sx1276_tx_request_t;

typedef struct sx1276_tx_result_s {
    uint8_t  tx_id;
    uint8_t  status;         // 0=OK, 1=TIMEOUT, 2=LBT_ABORT (future), 3=BUSY
    uint32_t time_on_air_us;
} sx1276_tx_result_t;

bool sx1276_tx_begin(const sx1276_tx_request_t *req);   // false if busy
bool sx1276_tx_poll(uint32_t events,
                    sx1276_tx_result_t *out_result);     // true if completed this tick
bool sx1276_tx_busy(void);
```

`sx1276_tx_poll` is called from the main loop with `radio_events` (same `events` bitmask as `sx1276_rx_service`). One-call-at-a-time design — no queue at this layer; the host owns queueing. Internal state machine: `IDLE → ARMED → COMPLETED → IDLE`.

### 4.4 Coexistence with RX

C2 leaves the radio in `RXCONTINUOUS` indefinitely. C3 must:

1. On `sx1276_tx_begin`: snapshot whether RX was armed (`s_rx_was_armed` flag), call `sx1276_rx_disarm()` (drops to STANDBY).
2. Run TX flow.
3. On TX completion: if `s_rx_was_armed`, re-arm RX via `sx1276_rx_arm()`.

This is a temporary half-duplex policy. Phase 4 will introduce a proper half-duplex scheduler when LBT lands. **Do not** attempt to run RX and TX simultaneously — the RF switch can only be in one position. The single-owner-of-RegOpMode rule (`sx1276_modes`) gives us a free invariant: only one mode at a time.

### 4.5 `time_on_air_us` calculation

Compute from current SF/BW/CR + `length` per Semtech AN1200.13 formula. **Do not** read it from a register (no such register). Use the formula already implemented in [base_station/lora_proto.py](../DESIGN-CONTROLLER/base_station/lora_proto.py) `lora_time_on_air_ms()` — port to C in `radio/sx1276_toa.c` (host-testable), unit-test against the Python reference for SF7..SF12 × BW125/250/500.

### 4.6 `host_cmd::handle_tx_frame`

Wire format of `TX_FRAME_REQ (0x10)`:
```
+--------+--------+----------+
| tx_id  | length | payload  |
| u8     | u8     | N B      |
+--------+--------+----------+
```

Handler:
1. Validate `length <= 255`. If not, emit `ERR_PROTO{TOO_LARGE}`.
2. Validate `sx1276_tx_busy() == false`. If busy, emit `ERR_PROTO{QUEUE_FULL}` (re-use existing sub-code).
3. Copy into `s_pending_tx_request`, call `sx1276_tx_begin`.
4. **Do not** block waiting for `TxDone`. Return immediately. The main-loop `sx1276_tx_poll` produces `TX_DONE_URC` asynchronously.

### 4.7 STATS slot wiring

Replace the placeholder `s_radio_tx_ok` increment site (currently has no caller) with a single call from `sx1276_tx_poll` on the `status == 0 OK` path. CI grep guard `check-tx-counter-owner` enforces sole ownership analogous to C2.

`radio_tx_abort_lbt` stays at zero — explicitly documented in `host_stats.c` until C5 lands.

### 4.8 Bench dryrun

Mirror C2's `bench/radio/rx_fsm_dryrun.c`. New `bench/radio/tx_fsm_dryrun.c`:

1. Scripted "TxDone arrives within timeout" → `tx_ok=1`, status=OK.
2. Scripted "TxDone never arrives" → after timeout, status=TIMEOUT, `tx_ok` not incremented.
3. `sx1276_tx_begin` while already armed → returns false, no FIFO writes observed.
4. RX-was-armed → TX → RX-rearmed sequence verified by mock `io_ops` recording the order of mode transitions.

### 4.9 Acceptance gates

1. `make check-tx-counter-owner` passes; intentional violation fails.
2. All 4 dryrun scenarios pass.
3. With H7 driver from PR #4 issuing `TX_FRAME_REQ` followed by `STATS_DUMP`: `radio_tx_ok` increments by 1, `TX_DONE_URC` arrives within `time_on_air_us + 50 ms`.
4. Bench: a scope on the RF switch GPIOs shows TXRX line goes high *before* PA enable (settle delay enforced inside `sx1276_modes_to_tx`).
5. `_Static_assert(sizeof(tx_done_urc_t) == 7)` (1 tx_id + 1 status + 4 time_on_air + 1 padding).

### 4.10 Risks

- **Forgetting to drop to STANDBY after TX = stuck PA.** The PA stays hot, drains battery, can damage if held. The `sx1276_tx_poll` cleanup path *must* call `sx1276_modes_to_standby` even on timeout. Add an explicit unit test for the timeout cleanup path.
- **Antenna policy.** First TX with no antenna can damage the SX1276 PA. Bench procedure (PR #6) requires "antenna or 50 Ω load attached **before** any `TX_FRAME_REQ` is issued." Add a fault: if a `TX_FRAME_REQ` arrives within 1 s of boot, return `ERR_PROTO{FORBIDDEN}` — gives the operator time to verify the antenna. Configurable via a CFG key `tx_arming_grace_ms` (new key 0x0F).
- **STM32 BKP-DR clobbering by fault replay.** The BKP registers used by PR #2's fault replay must not collide with anything C3 needs. Reserve BKP0..BKP4 for fault, leave BKP rest unused.

---

## 5. PR #4 — H7 minimal host driver (G6)

### 5.1 What "minimal" means

Bench bring-up needs exactly five primitives from the H7:

| Primitive | Purpose | Wire |
|---|---|---|
| `loraHostBegin(baud)` | Open Serial3 @ 921600, prime DMA RX | n/a |
| `loraHostPoll()` | Drain incoming bytes, decode COBS+CRC frames, dispatch URCs | n/a |
| `loraHostPing(timeout_ms)` | Round-trip PING, return measured RTT µs | `PING_REQ` → `PING_URC` |
| `loraHostGetStats(out_struct)` | Read all 64 STATS bytes into a typed struct | `STATS_DUMP_REQ` → `STATS_URC` |
| `loraHostTxFrame(payload, len, tx_id, on_done_cb)` | Issue a TX, callback on `TX_DONE_URC` | `TX_FRAME_REQ` → `TX_DONE_URC` |

Plus passive URC handlers for `BOOT_URC`, `RX_FRAME_URC`, `FAULT_URC` that route into existing H7 logging.

**Out of scope for this PR:** `CFG_SET/GET`, `REG_READ/WRITE`. Add in Tranche 5 once bench bring-up is stable. Bench operators can use the Python tool from PR #5 for those.

### 5.2 Library structure

New directory: `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/src/murata_host/`. Files:

| File | Responsibility |
|---|---|
| `murata_host.h` | Public API + struct typedefs (matches `host_types.h` byte-for-byte; share via vendored copy) |
| `murata_host.cpp` | State machine, RX byte-by-byte COBS decoder, frame dispatcher |
| `mh_cobs.c` | COBS encode/decode (no allocation) |
| `mh_cobs.h` | declarations |
| `mh_crc16.c` | CRC16-CCITT (poly 0x1021, init 0xFFFF) |
| `mh_crc16.h` | declarations |
| `mh_wire.h` | **vendored copy** of L072's `host_types.h` so type IDs stay in sync via PR review |

The vendored `mh_wire.h` is updated by a CI check that diffs it against `host_types.h` and fails the build if they drift (similar to existing `lora_proto/` vendoring per [arduino-ci.yml](../../.github/workflows/arduino-ci.yml)).

### 5.3 RX state machine

The H7 sees a stream of bytes on `Serial3.read()`. State machine:

```
START
  └── byte = 0x00 (COBS delimiter) → emit pending frame, return to START
  └── byte != 0x00 → push into rx_buf, continue
On frame emit:
  - COBS-decode rx_buf into decoded_buf
  - if length < 4 (type + payload_len + crc16) → drop, increment rx_errors
  - extract type (1B) + payload_len (1B) + payload (N B) + crc16 (2B LE)
  - verify crc16 over [type | payload_len | payload]
  - if mismatch → drop, increment crc_errors, emit nothing
  - dispatch by type: pending request match (PING/STATS/TX), or URC handler
  - clear rx_buf
```

**No interrupt handling.** Pure poll-driven from `loraHostPoll()` called every loop tick. The H7 main loop is already at ~1 kHz; that's fast enough for a 921600 baud stream (max ~115 KB/s). Use `Serial3.available()` + `Serial3.read()`.

### 5.4 TX side

`loraHostTxFrame` builds the frame in a stack buffer:

```
[type=0x10][payload_len][tx_id][length][payload...][crc16_lo][crc16_hi]
```

COBS-encodes into a sibling buffer, appends `0x00` delimiter, calls `Serial3.write(buf, len)`. Stores `tx_id` + `on_done_cb` in a small pending-tx ring (size 4) so the URC dispatcher can match the response.

`loraHostPing` is sync-with-timeout: builds and sends, then loops `loraHostPoll() + millis() < deadline` until either the matching URC dispatches into a static `g_last_ping_rtt` or the deadline expires.

### 5.5 Integration with existing tractor_h7.ino

Today [tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) speaks an older "raw SPI to Murata + KISS framing" protocol that does not match Method G's host_cmd. **Do not** delete the old code in this PR. Add the new `murata_host` library as a *parallel* path, gated by:

```cpp
#ifdef LIFETRAC_USE_METHOD_G_HOST
  // new path — Serial3 @ 921600 to L072 running custom firmware
  loraHostPoll();
#else
  // legacy KISS-over-SPI path (Tranche 1–3 unchanged)
  loraPoll();
#endif
```

A `build_flag` in the Arduino CI job for tractor_h7 toggles between the two so both compile-test cleanly. Bench uses `LIFETRAC_USE_METHOD_G_HOST=1`. Migration to Method G as default is Tranche 6 work.

### 5.6 Acceptance gates

1. arduino-cli compile of `tractor_h7` passes both with and without `LIFETRAC_USE_METHOD_G_HOST` defined.
2. Vendor-drift check fails when `host_types.h` is edited without updating `mh_wire.h`.
3. With PR #5's PTY harness on the dev machine: `loraHostPing()` round-trip <5 ms, 1000-iteration soak with zero CRC errors.
4. `loraHostGetStats` returns a populated struct whose `radio_state` field reads `2` (STANDBY).

### 5.7 Risks

- **Serial3 wiring conflict.** The Max Carrier may already use Serial3 for a debug header. Verify against the carrier's schematic before assigning. If conflict, fall back to Serial2 — both are exposed; document the chosen UART in PR #6.

---

## 6. Companion Analysis (Gemini 3.1 Pro Preview, 2026-05-05)

**Reviewer:** GitHub Copilot (Gemini 3.1 Pro)
**Purpose:** Technical validation of Tranche 4 cross-compilation infrastructure, bare-metal UART configurations, and H7 initialization sync.

### 6.1 Cross Compile Size Guarantees (Work Item G5)
- **CI Dependency Integrity:** Relying on `arm-none-eabi-size` inside an automated CI pipeline prevents catastrophic field deployments where `.text` stealthily overwrites the bootloader segment or RAM statically overflows. Your `tools/check_size_budget.py` tool perfectly models embed defensive CI.
- **Compiler Drift:** By strictly pinning to Ubuntu 24.04 and GCC 13.2, you actively prevent sudden ABI alterations or optimization flag deviations (like LTO unrolling loops differently) that might break tight loop timing for LBT/CAD routines down the road. 

### 6.2 BKP Fault Replay Design (Work Item G4 & G2)
- **Architecture Soundness:** The STM32L0 RTC Backup Registers are specifically built to survive soft-resets and IWDG expirations. Your approach of dumping the HardFault `PS/LR` state directly into `RTC_BKPxR` and halting for the watchdog is the industry standard for bare-metal crash forensics.
- **Risk Mitigation - DBP Enable:** You rightfully flagged the `PWR_CR_DBP` hazard. On the L072, Backup registers are in the RTC domain. The firmware *must* execute `HAL_PWR_EnableBkUpAccess()` (or bare-metal equivalent `SET_BIT(PWR->CR, PWR_CR_DBP)`) during very early boot, otherwise the HardFault handler will silently fail to write to the backup registers, destroying your forensics.

### 6.3 RX/TX Half-Duplex Mutex (Work Item C3)
- **State Flow Safety:** Automatically snapshotting `s_rx_was_armed`, disarming to STANDBY, conducting the TX transit, and instantly re-arming RX is a bulletproof method of enforcing half-duplex. 
- **PA Antenna Protection:** Implementing a `tx_arming_grace_ms` buffer upon boot to prevent accidental un-terminated TX fires (which will instantly scorch the SX1276 PA) is an elite-level hardware preservation feature. 

### 6.4 Minimal H7 Host Driver Poll Rates (Work Item G6)
- **H7 MCU Processing Overhead:** Calling `loraHostPoll()` continuously in the Arduino `loop()` at ~1kHz is ample velocity to drain the host UART buffer, as 921600 baud theoretically moves ~100 bytes per millisecond. However, the H7 Arduino core's `Serial3.available()` relies on an underlying RingBuffer. Ensure the `SERIAL_BUFFER_SIZE` inside the Arduino core configuration for Portenta is large enough (e.g., > 256 bytes) to hold at least one full COBS frame, otherwise high-volume bursts during loop blocking logic or RTOS context-switches will still overrun the host-side buffers.
- **921600 jitter on the H7's USART.** The H7's USART block is highly accurate; not a concern. The L072's HSI16 is ±1% — acceptable for 921600 with 1 stop bit. **Verify** by computing actual baud divisors in [hal/platform.c](../DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c) and asserting <2% error against the target.
- **DMA on the H7 side.** Not used in v1. Plain `Serial3.read()` is sufficient; revisit only if STATS rate exceeds 10 Hz sustained.

---

## 6. PR #5 — H7 driver host-CI tests + PTY loopback harness

### 6.1 Why this exists

PR #4's library has no firmware-runtime tests. Bench bring-up needs *confidence the wire layer works* before we blame radios. Solution: a Python harness that emulates the L072 over a PTY and exercises the H7 driver's COBS+CRC layer compiled as a host binary.

### 6.2 Two artifacts

1. **`tools/murata_host_loopback.py`** — Opens a pseudo-tty pair, plays the L072 side: responds to `PING_REQ`, returns synthetic `STATS_URC`, ACKs `TX_FRAME_REQ` and emits a `TX_DONE_URC` 50 ms later. Implements COBS + CRC16-CCITT in pure Python.

2. **`bench/h7_host_proto/mh_unit.c`** — Compiles `mh_cobs.c` + `mh_crc16.c` + a minimal stub of the dispatcher with a host CC; runs unit tests:
   - Round-trip 1000 random byte sequences through COBS encode/decode.
   - CRC16-CCITT against a known vector (`"123456789"` → `0x29B1`).
   - Frame builder produces a deterministic byte sequence for a known `(type, payload, crc)` tuple.

### 6.3 GHA

New job `h7-host-driver-tests` running on `ubuntu-latest`, depending on `firmware-l072-cross-compile` (so vendoring drift catches first):

```yaml
h7-host-driver-tests:
  runs-on: ubuntu-latest
  needs: firmware-l072-cross-compile
  steps:
    - uses: actions/checkout@v4
    - name: COBS + CRC unit tests
      working-directory: LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7
      run: |
        cc -std=gnu11 -Wall -Wextra -Werror \
           -Isrc/murata_host \
           bench/h7_host_proto/mh_unit.c \
           src/murata_host/mh_cobs.c src/murata_host/mh_crc16.c \
           -o build/mh_unit && build/mh_unit
    - name: Vendor drift check
      run: |
        diff -q LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h \
                LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/src/murata_host/mh_wire.h
    - name: Python loopback smoke
      run: python3 LifeTrac-v25/DESIGN-CONTROLLER/tools/murata_host_loopback.py --self-test
```

### 6.4 Acceptance gates

1. All three CI steps green.
2. Manual `--self-test` mode of `murata_host_loopback.py` runs to completion in <2 s with zero failures.
3. Editing `mh_cobs.c` to introduce an off-by-one breaks a unit test with a useful error.

### 6.5 Risks

- **PTYs are POSIX-only.** Windows developers can't run the loopback locally. Acceptable — CI runs on Linux. Document the limitation in [README.md](../DESIGN-CONTROLLER/firmware/tractor_h7/README.md).

---

## 7. PR #6 — `BRINGUP_MAX_CARRIER.md` runbook (G7)

### 7.1 Document outline

New file: `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md`.

```
1. Hardware checklist
   - 1 × Portenta H7 + Max Carrier
   - 1 × ST-Link V2 (or J-Link Edu Mini)
   - 1 × USB-C to Portenta
   - 1 × 915 MHz antenna OR 50 Ω SMA dummy load (DO NOT POWER PA WITHOUT EITHER)
   - 1 × known-good LoRa transmitter for Stage B (HackRF, second Max Carrier, or
         commercial LoRaWAN gateway in raw 915 SF7 BW250 mode)
   - For Stage C: a second Max Carrier + Portenta H7

2. SWD wiring (STM32L072 inside Murata SiP)
   - Max Carrier exposes the SiP's PA13 (SWDIO) and PA14 (SWCLK) on the high-density
     connector pins X (TBD — verify against carrier schematic rev D or later).
   - Connect ST-Link:
        ST-Link 1 (VCC sense)  → Max Carrier 3V3
        ST-Link 2 (SWCLK)      → PA14
        ST-Link 4 (SWDIO)      → PA13
        ST-Link 6 (GND)        → GND
        ST-Link 8 (NRST)       → optional; Max Carrier typically does not expose

3. Toolchain on bench laptop
   - arm-none-eabi-gcc 13.2 (apt install gcc-arm-none-eabi)
   - openocd 0.12+ (apt install openocd)
   - Pre-built artifact from PR #1's CI run (download firmware.bin from Actions)
     OR build locally: `make all`

4. Flash recipe
   - openocd config: stub here, full file at `tools/openocd/stm32l0.cfg`
        source [find interface/stlink.cfg]
        transport select hla_swd
        source [find target/stm32l0.cfg]
   - Flash:
        openocd -f tools/openocd/stm32l0.cfg \
                -c "program build/firmware.elf verify reset exit"
   - Expected output: "Verified OK" + "** Resetting Target **"

5. Stage A — Single-board sanity (no RF)
   - Antenna or 50 Ω load attached. (Even at STANDBY we may briefly enter RX which
     emits no power, but discipline matters.)
   - Power H7 over USB-C. H7 sketch built with LIFETRAC_USE_METHOD_G_HOST=1.
   - Open Arduino Serial Monitor on H7's USB-CDC at 115200.
   - Expected log within 200 ms of boot:
        [MH] BOOT_URC reset_cause=5 (POR_BOR) protocol_ver=1 schema_ver=1 radio_ok=1
        [MH] PING ok rtt=312us
        [MH] STATS radio_state=2 (STANDBY) host_irq_idle>0 host_dropped=0
   - Failure modes:
        - No BOOT_URC: check Serial3 wiring, baud, COBS framing
        - radio_ok=0: check SiP power (Max Carrier solder bridge for 1.8/3.3 V SiP),
          re-flash with FAULT_URC capture enabled
        - host_dropped > 0: H7 not polling fast enough, increase Serial3 buffer

6. Stage B — RX validation
   - Configure known-good transmitter on 915.000 MHz, SF7, BW250, CR4/5,
     CRC on, payload "LIFETRAC RX TEST 0001" (incrementing counter).
   - On H7 Serial Monitor expect:
        [MH] RX_FRAME len=20 snr=8 rssi=-72 ts=...
   - After 60 s, issue STATS_DUMP and verify:
        radio_rx_ok == TX rate × 60 (within ±2)
        radio_crc_err == 0
        radio_dio0 >= radio_rx_ok (DIO0 fires on every RxDone)
   - CRC torture: same transmitter with intentional bit-flip in payload:
        radio_crc_err counts; radio_rx_ok stays flat; no RX_FRAME_URC for bad frames

7. Stage C — Two-board ping-pong (after PR #3 lands)
   - Both boards flashed identically. One H7 sketch acts as "talker," other as "listener."
   - Talker calls loraHostTxFrame() at 1 Hz with monotonic counter.
   - Listener observes RX_FRAME_URC at 1 Hz; talker observes TX_DONE_URC.
   - 10-minute soak: zero crc_err, zero tx timeouts, radio_state oscillates
     between TX (3) and RX_CONT (4).

8. Recovery procedures
   - Brick: NRST + boot0 → STM32 ROM bootloader on USART (verify Max Carrier
     exposes boot0 — typically a solder-bridge selectable). Use stm32flash:
        stm32flash -w build/firmware.bin -v -g 0x08000000 /dev/ttyACM0
   - IWDG storm: BOOT_URC.reset_cause repeatedly == 3 (IWDG). Hold safe-mode
     pin (TBD) at boot to skip app, listen for RECOVERY_REQ on Serial3 from H7.

9. Logging conventions
   - All bench output should be captured with `script -t bench-$(date +%Y%m%d-%H%M).log`.
   - Save the resulting log + the firmware.bin SHA into AI NOTES with date-stamped filename.
```

### 7.2 What this PR contains

- The Markdown above (skeleton; fill in carrier-schematic-specific pin numbers as the operator verifies them).
- `tools/openocd/stm32l0.cfg` (10-line file).
- A small Arduino sketch `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7_bench/` that uses `murata_host` to do exactly what's described in Stages A/B/C with `#define BENCH_STAGE A|B|C` at the top. Three `.ino`s or one with conditional compilation; recommend the latter to keep one source of truth.

### 7.3 Acceptance gates

1. A second engineer, given only this document, can flash and run Stage A in <30 minutes from scratch.
2. The openocd config flashes cleanly to a Max Carrier on the bench (validate during PR #6 itself — operator paste of openocd output in PR body).
3. All three bench stages have explicit pass/fail criteria.

### 7.4 Risks

- **Carrier-schematic uncertainty.** Pin numbers for SWD breakout depend on Max Carrier revision. Mark pin numbers `<TBD: verify on carrier rev D>` in the doc until first hands-on session confirms; update in a follow-up PR.
- **First TX antenna mistake.** Mitigate by software (1-second TX grace from PR #3) **and** by red-bold-text in the runbook.

---

## 8. Combined timeline & dependency graph

```
PR #1 ci-cross-compile ─────────┐
                                ├──→ PR #4 h7-host-driver-min ──→ PR #5 h7-tests ──┐
PR #2 boot-fault ───────────────┤                                                  ├──→ PR #6 bringup-doc + bench
                                │                                                  │
PR #3 radio-tx-c3 ──────────────┘────────────────────────────────────────────────→─┘
```

PRs #2, #3, #4 can be developed in parallel after #1 lands. Bench session begins **only after #6 is merged**. Estimated wall-clock per PR (excluding review): #1 small, #2 medium, #3 medium-large, #4 large, #5 small, #6 small — six PRs total, two of them small enough to land same-day after #1.

---

## 9. Out of scope for Tranche 4 (do NOT land here)

- LBT (`tx_abort_lbt`) — Tranche 5.
- Quality-aware FHSS (N-06) — Phase 4+.
- CFG persistence to Flash — Phase 6.
- Crypto Profile B switch — Profile A locked at launch.
- A/B firmware slots — Phase 6 (N-26).
- Migrating tractor_h7.ino *off* the legacy KISS path as the default — Tranche 6.
- HCI-style debug tools beyond `murata_host_loopback.py` — wait until bench needs them.
- Bumping `HOST_PROTOCOL_VER` or `HOST_WIRE_SCHEMA_VER` — additions in this tranche are payload-additive on already-declared types. Schema stays at v1.

---

## 10. Tranche 4 success criterion (the only one that matters)

> A bench operator can power a Max Carrier, see `BOOT_URC` on the H7's USB serial within 200 ms, issue a `loraHostPing()` and see <5 ms RTT, run `loraHostGetStats()` and see `radio_state == STANDBY`, then with a second Max Carrier on the bench: ping-pong `TX_FRAME_REQ` ↔ `RX_FRAME_URC` for 10 minutes with zero CRC errors and zero TX timeouts. Any `FAULT_URC` during that 10 minutes is a bug to be filed against the firmware, not a bench-procedure error.

If we hit that, Tranche 4 is done and Phase 2 is bench-validated. Then Tranche 5 (LBT, CAD, link adaptation) is real-radio work, not desk work.

---

## 11. One-line summary

> **Land an `arm-none-eabi-gcc` cross-compile gate in CI first so we know the firmware actually links and fits 180 KB; populate `BOOT_URC.reset_cause` from RCC_CSR and add a `FAULT_URC` emitter (synchronous + RTC-BKP-replay-after-IWDG) so first-boot observability exists; ship a C3 TX path mirroring C2 (`radio/sx1276_tx.c`, `TX_DONE_URC`, `radio_tx_ok` wired, sole-writer guard, half-duplex coexistence with RX); build a minimal H7 host driver (`tractor_h7/src/murata_host/`) covering exactly PING + STATS + TX + URC dispatch, gated behind `LIFETRAC_USE_METHOD_G_HOST` so the legacy path stays compilable; gate the H7 driver with host-CC unit tests for COBS+CRC + a Python PTY loopback harness; finally write `BRINGUP_MAX_CARRIER.md` documenting SWD pinout, openocd flash recipe, and a three-stage bench procedure (single-board boot → known-good RX → two-board ping-pong) — after all six PRs land, the first physical bring-up session has zero unresolved unknowns.**

---

## 12. Roadmap — Tranche 5 through Tranche 9

These are sketches, not commitments. Each will be expanded into its own versioned plan document at the start of its tranche, after the prior tranche's bench results are in. Listed here so reviewers can see the whole arc and so design decisions inside Tranche 4 can avoid foreclosing options needed downstream.

### Tranche 5 — Channel hygiene & link adaptation (post-bring-up, pre-field)

**Trigger to start:** Tranche 4 §10 success criterion met (10-minute zero-error ping-pong on the bench).

**Theme:** Make the link survive a real RF environment. Today the radio yells on a fixed channel at fixed power and would happily step on any neighbor.

**Work items:**

| ID | Item | Files (anticipated) |
|---|---|---|
| C5 | LBT (Listen-Before-Talk) gate ahead of `sx1276_modes_to_tx` | `radio/sx1276_lbt.c`, hooks into `sx1276_tx_begin` |
| C6 | CAD (Channel Activity Detection) state machine using `sx1276_modes_to_cad` | `radio/sx1276_cad.c` |
| C7 | TX-power adaptation driven by recent `RX_FRAME_URC.snr/rssi` returned in ACKs | logic in `host_cmd.c`, no new file |
| C8 | Channel mask enforcement against `cfg.fhss_channel_mask` (per-frame frequency hop, deterministic schedule per [02 Phase 3 W3-1](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md)) | `radio/sx1276_fhss.c` |
| B5 | New CFG keys: `0x10 lbt_max_backoff_ms`, `0x11 cad_symbols`, `0x12 tx_power_step_db`, `0x13 fhss_dwell_ms` | extends `host_cfg_keys.h` + apply-hooks |
| C9 | Real `radio_tx_abort_lbt` counter wiring (last reserved STATS slot reaches a real call site) | one-line addition to `sx1276_lbt.c` |
| C10 | Air-time accounting per region (US 915 MHz: 400 ms per second per channel cap) | `radio/sx1276_airtime.c` |

**Out of scope:** Quality-aware FHSS (N-06 — Tranche 7). True random channel hopping with PRNG seeded from session nonce (Tranche 7).

**Bench validation:** spectrum analyzer or SDR confirms LBT honors a deliberately-injected interferer; air-time counter rejects overshoot when payload+rate intentionally exceeds duty cycle.

---

### Tranche 6 — Migrate `tractor_h7` to Method G as default; full host command surface

**Trigger:** Tranche 5 stable on the bench, two consecutive 24-hour soaks with zero `FAULT_URC`.

**Theme:** Retire the legacy KISS-over-SPI path; expand the H7 driver from "minimal bench-bring-up surface" to "production driver."

**Work items:**

| ID | Item |
|---|---|
| H1 | Default `LIFETRAC_USE_METHOD_G_HOST=1`; demote legacy KISS path behind opposite flag for one release cycle, then delete |
| H2 | H7 driver gains: `loraHostCfgSet/Get`, `loraHostRegRead/Write` (debug-only), `loraHostStatsReset`, `loraHostReset`, `loraHostFaultLog` (queues last 16 `FAULT_URC` entries) |
| H3 | Migrate H7's existing application-layer logic (handheld arbitration, telemetry shipping) from KISS frames to Method G `TX_FRAME_REQ` payloads — encryption stays in the H7 (Crypto Profile A) |
| H4 | New URC `0xC1 STATS_URC_DELTA` for high-rate stats streaming (1 Hz) without re-sending the full 64 B every time — schema-additive |
| H5 | Add `0x42 STATS_STREAM_ENABLE` request so H7 can opt in/out of the high-rate stream |
| H6 | Bump `HOST_PROTOCOL_VER` to 2 to advertise the expanded command set; **wire schema stays at v1** (additive only) |
| B6 | New CFG keys: `0x14 stats_stream_period_ms`, `0x15 fault_log_depth` |
| H7 | Document the fully-migrated wire in `LifeTrac-v25/DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/06_Host_Wire_v2_Spec.md` |

**Out of scope:** A/B firmware slots, persistence — those are Phase 6.

**Validation:** H7 + L072 endurance run with arbitration loop driving a real Opta over Ethernet at 50 Hz for 8 hours; `FAULT_URC` count = 0; `radio_state` time-in-RX_CONT ≥ 95 %.

---

### Tranche 7 — Quality-aware FHSS (N-06) + adversarial resistance

**Trigger:** Tranche 6 in production, field reports of channel-localized interference.

**Theme:** Move from "fixed 8-channel mask" to "learn which channels are useful right now." This is the first tranche that introduces *learning state* on the L072.

**Work items:**

| ID | Item |
|---|---|
| N06-1 | Per-channel rolling SNR/RSSI/loss estimator (16-channel × 8-sample EWMA, ~256 B RAM) |
| N06-2 | Reweight `cfg.fhss_channel_mask` toward channels with recent good metrics; weighted-random next-hop selector |
| N06-3 | New URC `0xC2 FHSS_QUALITY_URC` reporting per-channel scores (debug + tuning) |
| N06-4 | Unblock the `cfg.fhss_quality_aware` apply-hook (currently returns `APPLY_FAILED`); make it a real toggle |
| N06-5 | Adversarial-resistance fuzzer: replay-attack vectors at the Crypto Profile A boundary (H7 owns AES-GCM nonce window, but L072 must still sanely handle out-of-order arrival timestamps from the FHSS schedule) |
| C11 | Channel scan ("listen on all 16 for N seconds") — debug command for field tuning |
| H8 | H7-side telemetry of FHSS_QUALITY_URC into the existing arbitration UI |

**Bumps:** `HOST_PROTOCOL_VER` to 3, `HOST_WIRE_SCHEMA_VER` stays at 1 (still additive).

---

### Tranche 8 — Power management & deep sleep (Phase 5 in [02](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md))

**Trigger:** Battery-powered field deployment requirement (handheld remote that doesn't tether to the tractor's 12 V).

**Theme:** Get average current under 2 mA in the listen-mostly profile. Today the L072 + SX1276 idle around 12–15 mA.

**Work items:**

| ID | Item |
|---|---|
| P1 | STM32L0 Stop mode entry/exit; LSE-driven RTC wake; preserve UART RX detect via WKUP pin |
| P2 | SX1276 `OCP` + `RegOpMode SLEEP` between scheduled RX windows |
| P3 | RX windowing scheduler (e.g. 100 ms RX every 1 s — duty 10 %) per [04 §5.4](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) |
| P4 | Wake-on-UART-byte from H7 (interrupt on PA3 RX low edge) |
| P5 | Unblock `cfg.deep_sleep_enable` apply-hook (currently `APPLY_FAILED`) |
| P6 | New STATS counters `host_irq_wake_uart`, `host_irq_wake_rtc`, `radio_sleep_us_total`. Adding 12 B requires bumping `HOST_WIRE_SCHEMA_VER` to **2** — first wire bump since launch. |
| P7 | New CFG keys: `0x16 sleep_mode` (off/light/deep), `0x17 rx_window_period_ms`, `0x18 rx_window_dwell_ms` |
| H9 | H7 driver respects DEEP_SLEEP — buffers requests until next wake window |

**Risk callout:** First wire-schema bump. Migration plan must include a `STATS_URC_V1_COMPAT` fallback so older H7 firmware still parses 64 B before they upgrade.

---

### Tranche 9 — A/B firmware slots + secure boot + persistence (Phase 6 in [02](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md))

**Trigger:** Field deployment requires OTA. Until then, the single-slot launch profile is sufficient.

**Theme:** Make in-field upgrades survivable. The single biggest reliability lever for an autonomous fleet.

**Work items:**

| ID | Item |
|---|---|
| N26-1 | Re-partition APP region into two 88 KB slots + 4 KB slot-header per [02 Phase 6](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md). **`memory_map.h` change** — update preprocessed `.ld`, `check_size_budget.py`, the works. |
| N26-2 | Bootloader (4 KB, `MM_BOOT_*` already reserved) gains slot picker: validates SHA-256 of slot header, picks higher-version-with-valid-rollback-counter, fall-through to safe mode |
| N26-3 | New protocol surface: `OTA_BEGIN_REQ`, `OTA_CHUNK_REQ`, `OTA_COMMIT_REQ`, `OTA_PROGRESS_URC` |
| N26-4 | Anti-rollback counter in CFG region (the 8 KB CFG block finally gets used for something beyond runtime config) |
| N26-5 | CFG persistence — also gets implemented now, since we have a live writer of CFG region. Unblocks Phase 6 promise: `host_baud` writes survive reset. |
| N26-6 | Optional: ed25519 image signature verification in bootloader (additional 8 KB code in BOOT — verify the 4 KB BOOT budget can grow to 8 KB by trimming APP to 176 KB; this is a memory-map decision that must be reviewed) |
| H10 | H7 driver gains OTA orchestrator: parse a delivered `firmware.bin` from the cloud, chunk-feed to L072, watch progress URCs, command commit, reboot |

**Risk callout:** Memory-map change is the single most invasive thing in this roadmap. Schedule for a quiet release window. Pin all current downstream consumers (`memory_map.h` includers) and verify they recompile cleanly. The `_Static_assert` discipline in [include/static_asserts.c](../DESIGN-CONTROLLER/firmware/murata_l072/include/static_asserts.c) is what makes this survivable.

---

### Beyond Tranche 9 — known unknowns (sketched, no commitment)

These are open design questions to be resolved by data, not by planning today:

| Question | What we'd need to decide |
|---|---|
| **Crypto Profile B switch** (move AES-GCM from H7 into L072) | Decided locked-A at launch per [05 §5.4](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md). Revisit only if H7 → L072 link is the documented bottleneck for crypto throughput. Adds ~6 KB code + a hard real-time constraint on the L072 main loop. |
| **Migrate to CMWX1ZZABZ-091 (SX1262) variant** | If supply or regulatory concerns push the SiP. SX1262 has a different register map (different state machine, sub-GHz band-switching, etc.) — not a drop-in. Estimate: 2 tranches of work. Defer until forced. |
| **LoRaWAN class-A compliance** | Only if we ever interop with public infrastructure. Method G is point-to-point today; LoRaWAN would be a separate firmware target sharing the radio HAL. |
| **Multi-master arbitration on the H7 link** | If a second M0+ ever lives on the same UART (unlikely on Max Carrier; trivially possible on a custom carrier). Method G's COBS framing is half-duplex single-master; multi-master needs explicit per-frame address bytes. |
| **Time-synchronized FHSS (Bluetooth-LE-style)** | If quality-aware FHSS (Tranche 7) hits a wall. Requires a shared clock between transmitter and receiver — likely a GPS PPS input on both ends. Big architectural shift. |

---

### Tranche dependency graph (full arc)

```
Tranche 1  Tranche 2  Tranche 3  Tranche 4  Tranche 5    Tranche 6   Tranche 7    Tranche 8   Tranche 9
 ─────     ─────      ─────      ─────      ─────        ─────       ─────        ─────       ─────
 boot      CFG +      CFG        TX +       LBT/CAD      H7 full     Quality      Deep        OTA +
 host B1   modes +    policy     bring-up   FHSS         migration   FHSS         sleep       A/B +
 DMA HT/TC stats      tests + CI runbook    air-time     PROTOCOL_   adversarial  WIRE_       persistence
                      RX wiring             stats        VER=2       PROTOCOL_    SCHEMA_     PROTOCOL_
                                                                     VER=3        VER=2       VER=4
                                                                                              (memory map
                                                                                              repartition)
 ───────────────────────────►──────►───────►──────────►──────────►──────────►──────────►──────────►
 deck work             bench             field-ready                   field-hardened             OTA-ready
```

The horizontal arrows indicate where each tranche unblocks the next. Tranches 1–3 are deck work (no hardware needed). Tranche 4 is the bench transition. Tranches 5–7 are field-readiness. Tranche 8 is battery-life work. Tranche 9 is fleet-deployment-readiness.

---

### Schema-version bump schedule (decision log)

This is a load-bearing table. Reviewers should challenge any tranche that wants to bump a schema version outside this schedule.

| When | `HOST_PROTOCOL_VER` | `HOST_WIRE_SCHEMA_VER` | Why |
|---|---|---|---|
| Tranche 1 launch | 1 | 1 | Initial. |
| Tranches 2–5 | 1 | 1 | All additive — payload-additive on declared types only. **No bumps.** |
| Tranche 6 | **2** | 1 | Full command surface; PROTOCOL_VER advertises capability bitmap expansion. Wire bytes still parse under v1 readers. |
| Tranche 7 | **3** | 1 | New `FHSS_QUALITY_URC` type ID. PROTOCOL_VER signals presence; wire stays additive. |
| Tranche 8 | 3 | **2** | New STATS counters force a 64 B → 76 B `STATS_URC` payload. First and only wire-schema bump in this roadmap. Compat shim required. |
| Tranche 9 | **4** | 2 | New OTA command surface. |

Anything outside this table is an unplanned bump and needs its own RFC.
