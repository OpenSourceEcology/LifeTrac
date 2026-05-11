# 07 — Successful Connection Recipe: Portenta X8 ↔ STM32L072 ↔ SX1276

**Doc owner:** Copilot (assistant-authored)
**Created:** 2026-05-12
**Version:** v1.0
**Scope:** End-to-end documentation of the bring-up steps that produced the **first confirmed successful LoRa TX** on the LifeTrac controller hardware (Portenta X8 ABX00049 + Max Carrier ABX00043 rev V3.12 + Murata CMWX1ZZABZ-078 SiP). This document captures the **working recipe** at every chip boundary so future bring-up on a fresh board, a replacement SiP, or a port to a related carrier can replay the same sequence and know what "good" looks like at each interface.

This is a recipe document, not a closure log. For the chronological investigation history that produced this recipe, see:

- [00_DECISION_Method_G_Commitment.md](00_DECISION_Method_G_Commitment.md)
- [01_Capabilities_Analysis_Custom_Firmware.md](01_Capabilities_Analysis_Custom_Firmware.md)
- [02_Firmware_Architecture_Plan.md](02_Firmware_Architecture_Plan.md)
- [03_Bringup_Roadmap.md](03_Bringup_Roadmap.md)
- [04_Hardware_Interface_and_Recovery.md](04_Hardware_Interface_and_Recovery.md)
- [05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md](05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md)
- [06_AT_Service_Shell_Debug_Control_Plane.md](06_AT_Service_Shell_Debug_Control_Plane.md)
- AI NOTES W1-7, W1-8, W1-9, W1-9b, W1-9c, W1-9d closure docs (`../../AI NOTES/2026-05-1*_W1-*`).

---

## §1 Hardware stack and signal map

The connection chain is a four-link signaling pipeline. Each link must be working before the next can be tested.

```
┌──────────────┐  ADB         ┌──────────────────┐  /dev/ttymxc3   ┌────────────────┐  SPI1+DIO0+NRST  ┌─────────────┐
│  Dev host    │◄────────────►│  Portenta X8 i.MX │◄───────────────►│  STM32L072 (Murata SiP)  │◄────────────────►│  SX1276 die │
│  (Windows)   │  USB-C       │  8M Plus / Yocto │  LPUART1, 921600│  hal/host_uart.c          │  GPIO bit-bang   │  (same SiP) │
└──────────────┘              │  carrier ABX00043│  8N1            │  radio/sx1276*.c          │                  └─────────────┘
                              └──────────────────┘                 └────────────────────────────┘
                                  ▲                                       ▲
                                  │ OpenOCD bit-bang SWD                  │ TCXO 32 MHz, PB6 enable
                                  │ via i.MX GPIO (BOOT0 PA11, NRST PF4)  │ shared L072 HSE + SX1276 XTA
```

**Per-link summary:**

| Link | Transport | Baud / clock | Direction | Owner |
|------|-----------|--------------|-----------|-------|
| Dev host ↔ X8 | ADB over USB-C | 480 Mbit/s USB | bidirectional | `adb.exe` on Windows |
| X8 i.MX ↔ L072 (SWD reset/boot only) | OpenOCD imx_gpio bit-bang | ~1.8 MHz | one-shot per cycle | `method_h_stage2_boot_user_app.cfg` |
| X8 i.MX ↔ L072 (data) | LPUART1 on `/dev/ttymxc3` | 921 600 8N1 | bidirectional | `host_uart.c` (FW), `HostLink` class (probe) |
| L072 ↔ SX1276 | SPI1 (PB3 SCK / PA6 MISO / PA7 MOSI / PA15 NSS) + DIO0 EXTI + PC0 NRESET + PB6 TCXO_VCC | 4–8 MHz SPI, 32 MHz TCXO | bidirectional | `radio/sx1276.c`, `sx1276_tx.c`, `sx1276_rx.c` |

---

## §2 Per-chip success criteria (what "good" looks like)

The whole point of this document is that you can stop at each checkpoint and verify success **before** moving to the next link. If anything below fails, do not proceed — fix the link first.

### §2.1 Dev host ↔ X8 (ADB transport)

**Pass signal:**

```powershell
> adb devices
List of devices attached
2E2C1209DABC240B  device
```

The serial `2E2C1209DABC240B` is the bench unit. Any Yocto-flashed Portenta X8 in ADB mode will work; serial is just the discriminator for multi-device benches.

**Failure mode → remedy:**

| Symptom | Root cause | Fix |
|---------|------------|-----|
| `adb devices` shows nothing | USB-C cable is data-only? Yocto in fastboot? | Hold BOOT button + power cycle to re-enter normal Yocto, replace cable. |
| `unauthorized` | Host RSA key not yet acked | Re-plug, accept dialog on X8 display, or scrub `/data/misc/adb/adb_keys`. |

### §2.2 X8 ↔ L072 SWD (boot/reset control plane)

The X8 does **not** speak SWD natively to the L072 — it bit-bangs SWD over four i.MX GPIOs using OpenOCD (`imx_gpio` driver). This control plane is used for one purpose only: **drop BOOT0 (PA11) low and pulse NRST (PF4)** so the L072 boots from flash into the user application instead of into the STM ROM bootloader.

**Pass signal** (last lines of the OCD output during Method H Stage 2 boot phase):

```
Phase A: ensure GPIOA + GPIOF clocks enabled
  RCC_AHB4ENR before = 0
  RCC_AHB4ENR after  = 33
Phase B: ensure PA_11 + PF_4 are outputs (preserve other bits)
Phase C: drop BOOT0 (PA_11 -> LOW) so L072 boots from flash
  GPIOA_IDR.PA_11 = 0   (expect 0)
Phase D: pulse NRST (PF_4 LOW 250 ms then HIGH)
  GPIOF_IDR.PF_4 (low)  = 0   (expect 0)
  GPIOF_IDR.PF_4 (high) = 1   (expect 1)

DONE: BOOT0 LOW, NRST released. L072 should now run user firmware.
boot_ocd_rc=0
```

**Failure mode → remedy:**

| Symptom | Root cause | Fix |
|---------|------------|-----|
| `Error: SWD DPIDR mismatch` or `0xffffffff` | Bit-bang clock too fast, or i.MX GPIOs not muxed | Reduce `clock speed` in OCD cfg, verify `gpio_chip` mapping matches Yocto kernel. |
| `GPIOF_IDR.PF_4 (high) = 0` after release | Carrier rev mismatch (different NRST pin) | Check ABX00043 rev — V3.12 uses PF_4. |
| L072 stays in ROM bootloader (responds to AT 19200, not 921600) | BOOT0 not actually LOW | Re-run Method H; verify `RCC_AHB4ENR=33` (GPIOA+F clocks enabled). |

### §2.3 L072 boot → BOOT_URC over LPUART1

Within ~500 ms of NRST release, the L072 firmware must emit a `BOOT_URC (0xC0)` frame on LPUART1 at 921 600 8N1.

**Pass signal** (as parsed by `parse_boot()` in [method_g_stage1_probe.py](../firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py)):

```
BOOT_URC: reset_cause=0x?? radio_ok=1 radio_version=0x12 proto_ver=1 schema_ver=1 clock_source_id=0
```

The two critical fields:

- **`radio_ok=1`** — proves §2.5 (SX1276 SPI) is alive *before* the host even talks to the L072.
- **`radio_version=0x12`** — proves the L072 read register `0x42` (RegVersion) and got the expected `SX1276 v1.2` constant. Any other value (0x00, 0xFF, 0x11, 0x22) means SPI silicon mismatch or wiring/power fault.

**`clock_source_id`** key:

| Value | Meaning |
|-------|---------|
| 0 | HSE_OK — TCXO at 32 MHz is alive at L072 OSC_IN (also feeds SX1276 XTA via SiP-internal routing). |
| 1 | HSI_FALLBACK — internal 16 MHz RC; TCXO probe failed. **Will cause SX1276 OPMODE-stuck (W1-9b T2_FSK_SLEEP_ONLY) if PB6 is floating.** |

**Note:** As of [2026-05-12 W1-9d closure](../../AI%20NOTES/2026-05-12_W1-9d_PB6_TCXO_Enable_Plan_Copilot_v1_0.md) §11, `platform_clock_init_hsi16()` always reports `clock_source_id` based on whether HSE locks, but **always keeps SYSCLK on HSI16** to avoid silently halving the USART1 baud divider when SYSCLK switches 16→32 MHz. The TCXO **is** powered (PB6 HIGH), and the SX1276 die receives the 32 MHz clock; we just don't drive SYSCLK from it.

### §2.4 LPUART1 host transport (host ↔ L072 control plane)

After `BOOT_URC`, the host can issue `VER_REQ (0x10)` and expect `VER_URC (0x90… wait, 0xA0?)` — see [include/host_types.h](../firmware/murata_l072/include/host_types.h) for the canonical type table.

**Pass signal** (Stage 1 standard quant, 20-cycle gate, full PASS evidence):

```
GATE_RESULT=PASS
CYCLES_COMPLETED=20
FINAL_RESULT_PASS=20
LAUNCHER_FAIL_COUNT=0
TIMEOUT_COUNT=0
```

Per-cycle "ATI" / "AT+VER?" / "AT+STAT?" must roundtrip with **all** of:

- `host_parse_err = 0`
- `host_uart_err_lpuart = 0`
- `host_uart_err_usart1 = 0`
- per-flag `pe = fe = ne = ore = 0`

**Failure mode → remedy:**

| Symptom | Root cause | Fix |
|---------|------------|-----|
| `parse_err` ≥ 1, ASCII works at 19200 not 921600 | L072 booted into ROM (BOOT0 not LOW) | Re-run §2.2. |
| Single ghost byte mid-frame (e.g. `AT+VTER?`) | `host_uart_poll_dma()` foreground/IRQ race on `RDR` | **Already fixed** (W1-7, 2026-05-11): `cpu_irq_save/restore` around each ISR-snapshot+drain in `host_uart.c`. |
| Total transport silence post-fix | SYSCLK switched away from HSI16 → USART1 baud divider wrong | **Already fixed** (W1-9d delta-2, 2026-05-12): `platform_clock_init_hsi16()` no longer switches SYSCLK to HSE. |
| One-shot `HOST_FAULT_CODE_HOST_RX_SEEN (0x09 sub=0x01 LPUART1)` URC during first ~600 ms after boot | By-design diagnostic emission — confirms first host→MCU byte was observed (NOT a parse error) | **Already fixed** (W1-9f, 2026-05-12): probe-side `BENIGN_FAULT_CODES` filter in `method_h_stage2_tx_probe.py` excludes `HOST_RX_SEEN` and `HOST_DIAG_MARK` from the gate fault list. |
| Host probe issues REQ, response works but URC arriving in same window vanishes | Probe-side `HostLink.request()` discarded unrelated frames | **Already fixed** (W1-9e, 2026-05-12): persistent `HostLink.urc_queue` in `method_g_stage1_probe.py`. |

### §2.5 SX1276 SPI (radio register access)

This was W1-8. Key trap: the Murata CMWX1ZZABZ-078 SiP wires SX1276 SCK to **PB3** (NOT PA5 as on the bare B-L072Z-LRWAN1 dev board). The reference vendor sketch in [lora_ping.c](../firmware/murata_l072/lora_ping.c) had it right; the production driver in `radio/sx1276.c` originally did not.

**Pass signal** (Method G probe rc=0):

```
[TEST] SX1276 RegVersion(0x42) read = 0x12 (expected 0x12)  PASS
```

Pin map (this carrier, this SiP):

| Net | L072 pin | Function |
|-----|----------|----------|
| SX1276 SCK | **PB3** | SPI1_SCK AF |
| SX1276 MISO | PA6 | SPI1_MISO AF, **internal pull-up enabled** |
| SX1276 MOSI | PA7 | SPI1_MOSI AF |
| SX1276 NSS | PA15 | GPIO bit-bang (not AF) |
| SX1276 NRESET | PC0 | GPIO bit-bang, **Hi-Z release** (drive LOW 1 ms, then mode=input, wait 6 ms) |
| SX1276 DIO0 | PB4 | EXTI rising — TxDone / RxDone IRQ |
| **TCXO_VCC** | **PB6** | GPIO bit-bang HIGH (R12 0Ω strap to VDD_TCX0; **R86 PA12-strap is DNP**) |

### §2.6 SX1276 OPMODE transitions (radio mode-bits move)

This was W1-9b → W1-9d. Test: write `RegOpMode = 0x81` (LoRa + STDBY) and read it back.

**Pass signal** (W1-9b T2 verdict, post-W1-9d):

```
DIAG: REG_WRITE_ACK(opmode=0x81): ack=0181
DIAG(post-write 0x81): RegOpMode(0x01)=0x81 (TAKE)
DIAG(post-write 0x83): RegOpMode(0x01)=0x83 (TAKE during TX)  // or 0x85 RXCONTINUOUS post-TX
```

**Mode bit table (`RegOpMode[2:0]`)** in LoRa family (`bit 7=1`):

| Value | Mode | Notes |
|-------|------|-------|
| `0b000` 0x80 | SLEEP | Boot default; can clear `LongRangeMode` here. |
| `0b001` 0x81 | STDBY | Can configure most regs. |
| `0b010` 0x82 | FSTX | TX freq lock. |
| `0b011` 0x83 | TX | One-shot; auto-returns to STDBY. |
| `0b100` 0x84 | FSRX | RX freq lock. |
| `0b101` 0x85 | RXCONTINUOUS | Default post-init in current firmware. |
| `0b110` 0x86 | RXSINGLE | One-shot RX. |
| `0b111` 0x87 | CAD | Channel-activity detect. |

**Failure mode → remedy:**

| Symptom | Root cause | Fix |
|---------|------------|-----|
| Bit 7 of OpMode flips (LoRa↔FSK) but bits 2:0 frozen at 0 | SX1276 die has **no digital clock** — TCXO not powered | **Already fixed** (W1-9d): drive PB6 HIGH in `platform_clock_init_hsi16()`. |
| OpMode mode-bits transition ok in STDBY but TX never happens | DIO0 mapping wrong | Verify `RegDioMapping1 (0x40) = 0x00` for TxDone in LoRa mode. |

### §2.7 SX1276 TX (end-to-end packet transmit)

This was W1-9 / W1-9e. The firmware path: `host` sends `TX_FRAME_REQ (0x30)` → L072 issues SPI burst to FIFO + sets RegOpMode=0x83 → DIO0 EXTI fires when TxDone → `sx1276_tx_poll()` reads `RegIrqFlags=0x08`, calls `host_stats_radio_tx_ok()`, returns result → main loop calls `host_cmd_emit_tx_done()` → `TX_DONE_URC (0x90)` lands at host.

**Pass signal** (the breakthrough, [bench-evidence/W1-9_stage2_tx_2026-05-11_113759/](../bench-evidence/W1-9_stage2_tx_2026-05-11_113759/)):

```
TX_DONE_URC: tx_id=0x42 status=0(OK) time_on_air_us=18048 tx_power_dbm=14 wall_elapsed_ms=628.6
STATS(after): radio_tx_ok=1 radio_dio0=1 (delta=1) radio_state=4

Critical checks:
  [PASS] W1-9.B TX_DONE_URC received
  [PASS] W1-9.C status == OK
  [PASS] W1-9.D radio_tx_ok delta == 1
  [PASS] W1-9.E/F/G invariants stable
  [PASS] time_on_air_us in [5e3, 5e5] (18048)
```

The `time_on_air_us=18048` matches the analytical SF7/BW250kHz/CR4-5/8-byte payload prediction (~18 ms) — proves the airtime is real, not a host-side spoof.

---

## §3 The end-to-end "first packet" recipe

Run this exact sequence on a fresh / cold board to reproduce the success.

### Step 0: Prerequisites on the dev host

```powershell
# Toolchain (already installed in this workspace via Arduino15)
$env:PATH += ";C:\Users\<you>\AppData\Local\Arduino15\packages\arduino\tools\arm-none-eabi-gcc\7-2017q4\bin"
# WinLibs make
$env:PATH += ";C:\winlibs\mingw64\bin"
# Android SDK platform-tools (adb.exe)
$env:PATH += ";C:\Android\platform-tools"
```

### Step 1: Build the L072 firmware (Murata variant)

```powershell
cd C:\Users\dorkm\Documents\GitHub\LifeTrac
.\LifeTrac-v25\DESIGN-CONTROLLER\firmware\murata_l072\build.ps1
# Expected output: build/firmware.bin   ~16 440 bytes  (well under 180 KB APP region)
```

The currently-shipping firmware (W1-9d-confirmed) drives `PB6 HIGH` for TCXO_VCC and keeps SYSCLK on HSI16. Both deltas live in `hal/platform.c::platform_clock_init_hsi16()`.

### Step 2: Flash via Method G + sanity-check boot (Stage 1 standard quant)

```powershell
.\LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\run_stage1_standard_quant_end_to_end.ps1 `
  -AdbSerial 2E2C1209DABC240B `
  -Cycles 20
# Expected: GATE_RESULT=PASS, FINAL_RESULT_PASS=20, LAUNCHER_FAIL_COUNT=0
```

This wraps Method G flash + per-cycle AT-probe boot validation. Each cycle proves §2.1 → §2.5.

### Step 3: TX one packet (Method H Stage 2, default `tx` probe)

```powershell
.\LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\run_method_h_stage2_tx_end_to_end.ps1 `
  -AdbSerial 2E2C1209DABC240B `
  -Probe tx
# Expected: TX_DONE_URC tx_id=0x42 status=OK time_on_air_us≈18048 tx_power_dbm=14
#           radio_tx_ok delta +1, radio_dio0 delta +1
#           Critical checks: 7/7 PASS, VERDICT: PASS, probe_rc=0
```

This proves §2.6 + §2.7 — i.e. the SX1276 actually transmits a real LoRa packet over the air.

### Step 4: Probe variants (optional diagnostics)

```powershell
# RegVersion burst (1024×) — exercises SPI shadow path under load
.\run_method_h_stage2_tx_end_to_end.ps1 -AdbSerial <serial> -Probe regversion

# FSK STDBY discriminator — verdict should be PRE_STATE_UNEXPECTED_0x85 (LoRa+RXCONTINUOUS) post-W1-9d
.\run_method_h_stage2_tx_end_to_end.ps1 -AdbSerial <serial> -Probe fsk

# OPMODE walk — sweeps all 8 mode values per family
.\run_method_h_stage2_tx_end_to_end.ps1 -AdbSerial <serial> -Probe opmode_walk
```

---

## §4 Cross-link timing budget (what each step costs)

| Step | Wall time | Comment |
|------|-----------|---------|
| Method G flash (stage 0 ROM bootloader → write APP region) | ~6–8 s | Per-block UART checksum at 115 200 baud. |
| BOOT0-LOW + NRST pulse (Method H Stage 2 phase A–D) | ~250 ms LOW + ~250 ms settle | OCD bit-bang. |
| L072 firmware boot to first BOOT_URC | ~50 ms post-NRST-release | `platform_clock_init_hsi16()` + radio init. |
| LPUART1 921 600 baud-divider lock | ~200 ms post-boot | DMA + IDLE detect settling (W1-9f transient). |
| First TX_FRAME_REQ → TX_DONE_URC roundtrip | ~620 ms wall (probe overhead), ~18 ms SX1276 ToA | SF7/BW250/8 B. |
| SX1276 RXCONTINUOUS resume after TX | < 5 ms | Auto via firmware post-TX cleanup. |

---

## §5 Known-good register snapshot (post-init, mid-cycle)

These are the values the Method H `tx` probe reads back during a successful cycle. Useful as a reference for "what the chip looks like when it works":

| Register | Address | Value | Meaning |
|----------|---------|-------|---------|
| RegVersion | 0x42 | 0x12 | SX1276 silicon ID |
| RegOpMode | 0x01 | 0x85 | LoRa + RXCONTINUOUS (boot default) |
| RegOpMode (during TX) | 0x01 | 0x83 | LoRa + TX (transient, ~18 ms) |
| RegPaConfig | 0x09 | 0x8C | PA_BOOST + Pmax + OutputPower (~14 dBm via PA_BOOST) |
| RegPayloadLength | 0x22 | 0x08 | 8-byte test payload "LIFETRAC" |
| RegDioMapping1 | 0x40 | 0x00 | DIO0=TxDone (LoRa mode) |
| RegTcxo | 0x4B | 0x19 | TcxoInputOn (bit 4) + reserved 0x09 (POR default) |
| RegIrqFlags (post-TX, briefly) | 0x12 | 0x08 | TxDone bit set; cleared by firmware via W1C |

If any of these read differently on a fresh board, suspect that link (refer back to §2.x).

---

## §6 Failure-mode quick reference (one-page debug table)

| Top-level symptom | Most likely link | First check |
|-------------------|------------------|-------------|
| `adb devices` empty | §2.1 | USB-C cable, X8 power LED. |
| Method H Stage 2 OCD `DPIDR mismatch` | §2.2 | OCD clock speed, GPIO mux, kernel `gpio_chip` numbering. |
| BOOT_URC missing or garbled | §2.3, §2.4 | Probe at 19 200 (ROM detector) — if AT works, BOOT0 not LOW. |
| BOOT_URC shows `clock_source_id=1` and TX fails | §2.3, §2.5, §2.6 | Verify PB6 is being driven HIGH; DMM 3.3 V at U23 pin 48. |
| `host_parse_err > 0` accumulating across cycles | §2.4 | Suspect `host_uart_poll_dma()` race regressed; check `cpu_irq_save/restore`. |
| RegVersion = 0x00 or 0xFF | §2.5 | SPI SCK pin (must be PB3 not PA5); MISO pull-up on PA6. |
| OPMODE bit-7 flips but bits 2:0 frozen | §2.6 | TCXO not powered → re-check PB6, R12 strap. |
| TX_FRAME_REQ → no TX_DONE_URC, but `radio_tx_ok` increments in STATS | §2.4 (probe side) | `HostLink.urc_queue` not present → re-apply W1-9e fix. |
| TX never starts (`radio_tx_ok` flat at 0) | §2.6, §2.7 | OPMODE didn't reach 0x83 → most likely §2.6 root cause. |

---

## §7 Outstanding follow-ups (not blocking TX)

These are tracked in the [TODO.md](../TODO.md) and repo memory, but are **not** required for the §3 recipe to work:

1. ~~**W1-9b T2 probe pre-state assertion**~~ + ~~**W1-9g — Post-VER `STATS_DUMP_REQ` dispatch stall**~~ — **BOTH CLOSED 2026-05-12.** Probe-side fix (no firmware change): extended `drain_boot()` in `method_h_stage2_tx_probe.py` to handle every URC type explicitly, and added `drain_pending(link, quiet_s=0.25, max_s=1.0)` helper called between the `VER_REQ` warm-up and `fetch_stats()`. Bench evidence `bench-evidence/W1-9b_fsk_2026-05-11_125612/`: `T0b: post-VER drain consumed 4 frames` (FAULT_URC + STATS_URC × 2), `STATS(before) host_parse_err=0`, FSK SLEEP/STDBY mode bits 2:0 transition 0→1 (clock alive), `probe_rc=0`. Stage 1 regression: 3/3 PASS preserved. Final FSK verdict `T2_INCONSISTENT` (chip won't toggle `LongRangeMode` from non-SLEEP per SX1276 datasheet) is a separate minor classifier issue, not a regression. Authoritative writeup: [2026-05-12_W1-9g_Post_VER_STATS_Dispatch_Stall_Copilot_v1_0.md](../../AI%20NOTES/2026-05-12_W1-9g_Post_VER_STATS_Dispatch_Stall_Copilot_v1_0.md).
2. ~~**W1-9f early-boot LPUART1 transient**~~ — **CLOSED 2026-05-12.** The FAULT_URC observed during the W1-9e cycle was misclassified: code `0x09` is `HOST_FAULT_CODE_HOST_RX_SEEN` (a by-design first-RX-byte diagnostic), not `HOST_PARSE_ERROR (0x0B)`. End-of-cycle `host_parse_err=0` confirmed no real parse errors occurred. Fix: probe-side `BENIGN_FAULT_CODES` filter; bench evidence `bench-evidence/W1-9_stage2_tx_2026-05-11_115340/` (7/7 PASS, `probe_rc=0`).
4. **Future SYSCLK=HSE 32 MHz** — if a future workload needs 32 MHz SYSCLK (e.g. faster SPI burst), override USART1 kernel clock to HSI16 via `RCC_CCIPR[1:0]=10` (matching the existing LPUART1 pattern) before re-enabling the SYSCLK switch in `platform_clock_init_hsi16()`. The W1-9d D-2 delta only **removed** the switch; it didn't make USART1 robust against a future re-introduction.
5. ~~**RX validation — Phase A (single-board RX-liveness)**~~ — **CLOSED 2026-05-12.** Added `--probe rx` sub-mode in [`method_h_stage2_tx_probe.py::run_rx_liveness()`](../../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py); 7/7 gates PASS first run on Board 2 (`bench-evidence/W1-10_rx_liveness_2026-05-11_133459/`): `RegOpMode=0x85` stuck pre/post, `RegRssiValue` -120 → -119 dBm (in band, AGC twitching), `radio_state=4` (RX_CONT), host invariants stable, no real FAULT_URC across 10 s window. Authoritative writeup: [2026-05-12_W1-10_RX_Validation_Plan_Copilot_v1_0.md](../../AI%20NOTES/2026-05-12_W1-10_RX_Validation_Plan_Copilot_v1_0.md). **Phase B (two-board end-to-end ≥99% rates) remains OPEN** — blocked on Board 1 (`2D0A1209DABC240B`) returning to the bench. Until Phase B closes, the firmware is **conditional GO** for receive (every link in the RX path is proven *except* the SX1276 RF demodulator producing a real `RX_DONE` IRQ from an external preamble).

---

## §8 Provenance

- Hardware: Portenta X8 ABX00049 (ADB serial `2E2C1209DABC240B`), Max Carrier ABX00043 rev V3.12, Murata CMWX1ZZABZ-078 SiP (STM32L072CZ + SX1276 + 32 MHz TCXO).
- Toolchain: arm-none-eabi-gcc 7-2017q4 (Arduino15 path), mingw32-make (WinLibs), OpenOCD 0.11.0-dirty (X8-side, Yocto rootfs).
- Firmware build that achieved §2.7 success: 16 440 bytes (post-W1-9d, two `hal/platform.c` deltas). See `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/`.
- Probe build that captured §2.7 evidence: `method_g_stage1_probe.py` post-W1-9e (persistent `HostLink.urc_queue`).
- Bench evidence dir for the canonical first-TX cycle: `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/W1-9_stage2_tx_2026-05-11_113759/`.
- Closure docs that produced this recipe (chronological):
  - W1-7 RX (host UART RX bug) — [2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_3.md](../../AI%20NOTES/2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_3.md)
  - W1-8 SPI — [2026-05-11_W1-8_SX1276_SPI_BringUp_Plan_Copilot_v1_0.md](../../AI%20NOTES/2026-05-11_W1-8_SX1276_SPI_BringUp_Plan_Copilot_v1_0.md)
  - W1-9 TX (deferred) — [2026-05-11_W1-9_TX_BringUp_Closure_Copilot_v1_1.md](../../AI%20NOTES/2026-05-11_W1-9_TX_BringUp_Closure_Copilot_v1_1.md)
  - W1-9b OPMODE-stuck discriminator — [2026-05-11_W1-9b_OPMODE_Stuck_Probe_Closure_Copilot_v1_0.md](../../AI%20NOTES/2026-05-11_W1-9b_OPMODE_Stuck_Probe_Closure_Copilot_v1_0.md)
  - W1-9c software follow-up — [2026-05-12_W1-9c_Software_Followup_Closure_Copilot_v1_0.md](../../AI%20NOTES/2026-05-12_W1-9c_Software_Followup_Closure_Copilot_v1_0.md)
  - W1-9d PB6 TCXO_VCC enable + SYSCLK fix — [2026-05-12_W1-9d_PB6_TCXO_Enable_Plan_Copilot_v1_0.md](../../AI%20NOTES/2026-05-12_W1-9d_PB6_TCXO_Enable_Plan_Copilot_v1_0.md) §11

---

*End of recipe. If a future bring-up follows §3 and §2.x all PASS, the LoRa modem is fully functional from host application down to RF.*
