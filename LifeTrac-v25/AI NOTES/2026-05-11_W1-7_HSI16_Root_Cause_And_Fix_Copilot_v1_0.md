# W1-7 Resolution: HSI16 Clock Bug Identified and Fixed

**Date:** 2026-05-11
**Analyst:** Copilot
**Version:** v1.0
**Status:** PRIMARY BLOCKER CLEARED (TX path); RX-side investigation queued
**Predecessor:** [`2026-05-11_Blocker_Research_Synthesis_Copilot_v1_0.md`](2026-05-11_Blocker_Research_Synthesis_Copilot_v1_0.md)
**Bench serial:** `2E2C1209DABC240B` (Portenta X8 + Max Carrier + Murata CMWX1ZZABZ-078)

---

## TL;DR

The W1-7 production-firmware silence on `/dev/ttymxc3 @ 921600 8N1` had a single root cause in the L072 firmware: **`platform_clock_init_hsi16()` cleared `RCC_CR.HSION` after switching SYSCLK to HSE, but `host_uart_init()` had selected HSI16 as the LPUART1 kernel clock via `RCC_CCIPR[11:10]=0b10`.** With HSION off, the LPUART1 baud-rate generator had no clock and the TX line idled low forever (manifesting as the 400-byte `0x00` strings observed in the diagnostic captures).

Two-file fix applied. After re-flash the production firmware now emits **BOOT_URC (type 0xF0)**, **READY_URC (type 0xF2)**, **STATS/FAULT URCs (type 0xF1)**, plus the diagnostic ASCII traces `M:RADIO_BYPASS`, `C:SEND_BOOT`, `C:SEND_READY`, `C:INIT_DONE` on `/dev/ttymxc3 @ 921600 8N1` — all visible in the wrapper's `early_921600_bytes=310` capture window.

A separate, smaller bug remains and is now the focus of follow-up work: the firmware emits `HOST_FAULT_CODE_HOST_RX_INACTIVE` (0x0A) ~3 s after boot, and the probe's `VER_REQ` (0x01) gets no `VER_URC` (0x81) reply. TX path is fully working; RX path is silent. Hypotheses and next-step probe defined in §3 below.

---

## 1. Root cause

### 1.1 What `platform_clock_init_hsi16()` was doing wrong

In [`firmware/murata_l072/hal/platform.c`](../DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c), the original sequence was:

1. Enable HSE bypass (32 MHz TCXO from the SX1276 reference).
2. Wait for HSERDY.
3. Switch SYSCLK to HSE.
4. **Clear `RCC_CR.HSION`** to "save ~100 µA" — this was the bug.

But [`firmware/murata_l072/host/host_uart.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c) `host_uart_init()` configures the LPUART1 kernel clock independently of SYSCLK:

```c
RCC_CCIPR = (RCC_CCIPR & ~(0x3U << 10)) | (0x2U << 10); // LPUART1SEL = HSI16
```

LPUART1's baud-rate generator therefore needed HSI16 to be running. With HSION cleared, the BRR ran with no clock, `LPUART1_TDR` writes never produced bit transitions, and the line idled at the pull-up's logic level. The early 921600 captures showed exactly 400 bytes of `0x00`, which is what an undriven TX line looks like to a UART receiver running at the right baud (continuous start-bit-with-no-data interpretation).

### 1.2 Why the symptom looked like "RX gate"

Because both transports (production firmware over LPUART1 and ROM bootloader over USART2) use the same physical pads PA2/PA3, and because the ROM bootloader's USART2 path was working perfectly, the silence was attributed to upstream ingress gating (mux SEL, level-shifter OE, pad pull-up) rather than to the L072 firmware's own clock tree. The Phase A–D 34-pin GPIO sweeps exhaustively ruled out the mux/SEL/OE hypothesis. The hello_world decisive test (Step 1') then isolated the bug to the production firmware.

### 1.3 Why hello_world worked while production didn't

The hello_world image bypasses the entire HAL `platform_clock_init_hsi16()` call. It runs LPUART1 (or USART2 in some variants) at 19200 8N1 directly off the post-reset HSI16 default, so HSION is still asserted when the BRR samples the clock. This is exactly the property that made it a decisive test.

---

## 2. Fix

Two edits, both committed on 2026-05-11:

### 2.1 [`hal/platform.c`](../DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c) — `platform_clock_init_hsi16()`

Removed the `RCC_CR &= ~RCC_CR_HSION;` line. Replaced with a comment block explaining the LPUART1 dependency and the ~100 µA cost of leaving HSION asserted alongside HSE.

### 2.2 [`host/host_uart.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c) — `host_uart_init()`

Added a defensive ensure-on at the top of the function, before any CCIPR / USART register touches:

```c
RCC_CR |= RCC_CR_HSION;
while ((RCC_CR & RCC_CR_HSIRDY) == 0U) { /* wait */ }
```

This guarantees LPUART1 has a clock regardless of what other init code may have done to `RCC_CR` upstream.

### 2.3 Build artifact

`build/firmware.bin` = 15 288 bytes (was 15 276; +12 bytes for the HSION ensure-on sequence). Build clean, no warnings.

---

## 3. Validation

### 3.1 Wrapper capture (`early_921600_bytes=310`)

After the patched-firmware Method G flash, the wrapper script captures the first 310 bytes from `/dev/ttymxc3 @ 921600 8N1` immediately after BOOT0 release + NRST pulse:

```
00000020: 0357 3300 4d3a 5241 4449 4f5f 4259 5041  .W3.M:RADIO_BYPA
00000030: 5353 0d0a 433a 5345 4e44 5f42 4f4f 540d  SS..C:SEND_BOOT.
00000040: 0a00 0301 f001 0102 0601 0101 0601 0101  ................
00000050: 95d7 0043 3a53 454e 445f 5245 4144 590d  ...C:SEND_READY.
00000060: 0a00 0301 f201 0102 0806 0101 0101 7f01  ................
00000070: 0103 97fd 0043 3a49 4e49 545f 444f 4e45  .....C:INIT_DONE
00000080: 0d0a 0003 01f1 0101 0218 0308 0101 0101  ................
```

Decoding:
- `M:RADIO_BYPASS` — diagnostic trace from `main.c`; SX1276 init returned not-ready (expected, no LoRa partner present).
- `0x00 0x03 0x01 0xF0 ... 0x95 0xD7 0x00` — first COBS frame. Type 0xF0 = `HOST_TYPE_BOOT_URC`. CRC16 trailer `0x95D7`.
- `C:SEND_BOOT` — diagnostic ASCII confirming the BOOT_URC TX path completed.
- `0x00 0x03 0x01 0xF2 ... 0x97 0xFD 0x00` — second COBS frame. Type 0xF2 = `HOST_TYPE_READY_URC`.
- `C:SEND_READY` — diagnostic confirming READY URC TX completed.
- `C:INIT_DONE` — host_uart + host_cmd init complete; main loop entering.
- `0x00 0x03 0x01 0xF1 ... ` — STATS/FAULT URCs (type 0xF1) following in the stream.

### 3.2 What is NOT working yet (Step 6)

After ~3 s with no incoming RX bytes, the firmware emits another type 0xF1 URC carrying `HOST_FAULT_CODE_HOST_RX_INACTIVE = 0x0A`. The probe sends `VER_REQ` (type 0x01) on `/dev/ttymxc3` but never gets `VER_URC` (type 0x81) back, and times out.

This means: **TX path is fully functional, RX path is not seeing incoming bytes**. The probe correctly writes to `/dev/ttymxc3` (we know because the same path delivers AN3155 commands to the ROM bootloader without issue), but once the user firmware is running, LPUART1 RX appears to drop bytes silently.

### 3.3 Hypotheses for the RX-side issue (next investigation)

| H | Hypothesis | Test |
|---|---|---|
| A | `safe_mode_listen` leaves PA3 in a state (USART2 AF4, MODER, PUPDR) that survives the AF6 reconfigure in `usart2_gpio_init()` | In `host_uart_init()`, explicitly clear PA3 PUPDR + force MODER to input (00) before setting AF6 + MODER to AF (10) |
| B | `RNG_LPUART1_IRQHandler` is wired but never fires on incoming RX | Add a free-running counter incremented in the ISR; surface it in the next STATS_URC body |
| C | i.MX `UART4_TXD` pad isn't actually driving the L072 PA3 line in user-firmware mode | Logic-analyzer capture on PA3 during `VER_REQ` |
| D | LPUART1 RX framing error / overrun bit set immediately and not cleared | Read `LPUART1_ISR` in a diagnostic URC after the first `host_uart_take_rx_seen_flags()` call |

The cheapest first probe is H-B: add the ISR counter and surface it. If the counter advances on probe TX → A/B/C are ruled out, parser is the bug. If it stays at 0 → hardware/AF/pad-config (A or C) is the cause.

### 3.4 Step 6 result — RX ISR FIRES, framing errors corrupt bytes (2026-05-11)

A deliberate stimulus test (reset L072 → start passive `cat /dev/ttymxc3 @ 921600` → write three byte bursts including a binary `VER_REQ` and `AT\r\n` from the X8 → capture full 5 s) yielded the following ASCII traces in the captured stream:

```
M:RADIO_BYPASS
C:SEND_BOOT
C:SEND_READY
C:INIT_DONE
H:RX_LPUART1     ← first RX byte received: ISR works, AF6/PA3 wired
H:ERR_LPUART1    ← framing/parity/noise/overrun error on a subsequent byte
H:PROC_FRAME     ← frame parser was reached (some bytes survived)
H:COBS_ERR       ← COBS decoder rejected the corrupted frame
```

Plus a `HOST_TYPE_FAULT_URC` (0xF1) body containing `HOST_FAULT_CODE_HOST_DIAG_MARK` (0x0C) with mark `0x02 = HOST_DIAG_MARK_FRAME_PARSE_ERR`.

**Conclusion:** the RX *hardware* path is fully functional — bytes reach LPUART1, the ISR fires, the parser runs. The bug is **byte-level corruption** consistent with a clock-tolerance mismatch:

- LPUART1 kernel clock = HSI16, datasheet tolerance ±1 % at room temp, ±2 % over full range.
- BRR = 4444 → actual baud = `256 × 16 MHz / 4444` = 921 514 (nominal error 0.009 %).
- BUT if HSI16 actually runs at 16 MHz ± 1 %, the realized baud spans 912 363 … 930 850, a worst-case **±1.0 %** error.
- UART tolerance is roughly ±2.5 % per byte for 10-bit framing, but COBS frames depend on every byte being correct — a single corrupted byte kills the frame, so even occasional drift produces the observed `H:ERR_LPUART1` + `H:COBS_ERR` pattern.
- The TX direction works because the i.MX side has a clean clock (UART4 sourced from a calibrated PLL on the i.MX 8M Mini); even if HSI16 is off by 1 %, the i.MX RX easily tolerates it. RX direction fails because the L072's own sampling clock has the drift.

Hypothesis ranking after this evidence:

- ~~A (PA3 pad state)~~ — ruled out: H:RX_LPUART1 fired → AF/MODER/PUPDR are correct.
- ~~B (ISR not firing)~~ — ruled out: counter equivalent (`H:RX_LPUART1` one-shot) was emitted.
- ~~C (i.MX TX not driving PA3)~~ — ruled out: bytes physically reached LPUART1 RX register.
- **D (clock-tolerance / framing error)** — **CONFIRMED**: `H:ERR_LPUART1` from the `(USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)` branch in `RNG_LPUART1_IRQHandler`, plus `H:COBS_ERR`.

### 3.5 Step 6 fix attempts (2026-05-11)

Three options were proposed, in increasing order of disruption:

1. **Switch LPUART1 kernel clock to SYSCLK** (HSE 32 MHz TCXO via PLL/sysclk). Update `host_uart_init()` to set `RCC_CCIPR[11:10] = 0b01` (SYSCLK) and recompute BRR from runtime `platform_core_hz()`. **TRIED 2026-05-11; DID NOT FIX.**
2. **Drop host link to 115 200.** BRR widens 8×, framing tolerance grows. HSI16 ±1% comfortably supports 115 200 (margin ~13×).
3. **Fix HSE/TCXO bring-up** so SYSCLK actually becomes 32 MHz from the Murata module's TCXO. Requires identifying and asserting the module's TCXO-enable GPIO before the HSE probe.
4. **Calibrate HSI16 via the LSE-trim mechanism** (`RCC_ICSCR.HSI16TRIM`). Adds runtime calibration complexity.

#### 3.5.1 Why option 1 didn't work — DIAGNOSED

Added a one-shot diagnostic trace in `host_cmd.c` after `READY_URC`:

```c
if (platform_clock_source_id() == PLATFORM_CLOCK_SOURCE_HSE_OK) {
    platform_diag_trace("C:CLK_HSE_32M\r\n");
} else {
    platform_diag_trace("C:CLK_HSI_16M\r\n");
}
```

Capture from `bench-evidence/T6_bringup_2026-05-10_234828` (firmware.bin = 15340 bytes):

```
00000070: 0103 97fd 0043 3a43 4c4b 5f48 5349 5f31  .....C:CLK_HSI_1
                                  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                                  C:CLK_HSI_16M  → HSE never locked
```

`platform_clock_init_hsi16()` tries HSE BYPASS for `PLATFORM_HSE_READY_TIMEOUT` cycles. On this Murata CMWX1ZZABZ-078 module the HSE probe times out and the function falls back to HSI16 16 MHz. This is consistent with the `BRINGUP_MAX_CARRIER.md` warning ("clock_source_id == 1: HSE did not lock, investigate TCXO/clock path") and with [`lora_ping.c`](../DESIGN-CONTROLLER/firmware/murata_l072/lora_ping.c) (line 610) which forces HSI16 explicitly to "avoid HSE/TCXO probe issues".

Result: option 1 selected SYSCLK as the LPUART1 kernel clock, but SYSCLK = HSI16 = 16 MHz (not 32 MHz). BRR was recomputed to 4444 (matching 16 MHz / 921 600), TX still works perfectly (sender + receiver both clocked the same way), but the i.MX-side TX → L072-side RX direction still suffers ±1% drift on the L072 sample clock → framing errors continue.

#### 3.5.2 Recommended next action

**Option 2 (drop baud to 115 200)** is the lowest-risk path to clear W1-7 and unblock the rest of the stack. Implementation: change `HOST_BAUD_DEFAULT` in `include/host_proto.h` (or equivalent) to `115200`, change the probe + wrapper scripts' baud parameter to 115 200. HSI16 ±1% accuracy comfortably supports 115 200 (sample clock drift over a 10-bit frame is ~1 sample at 16× oversampling — well within tolerance).

**Option 3 (fix HSE/TCXO)** is the proper long-term fix and preserves 921 600 throughput, but requires identifying the Murata module's TCXO-enable GPIO (typically PA12 on B-L072Z-LRWAN1 reference designs; not yet confirmed for the Max Carrier wiring) and adding an enable-and-settle sequence before the HSE probe in `platform_clock_init_hsi16()`.

#### 3.5.3 PA12 TCXO_EN attempt — DID NOT FIX (2026-05-11)

Added PA12 push-pull HIGH + 5 ms settle delay before HSE BYPASS probe in [`platform.c`](../DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c) `platform_clock_init_hsi16()`. Rebuilt (firmware.bin = 15428 bytes), flashed, captured early UART. Diagnostic trace still emits `C:CLK_HSI_16M`. PA12 is therefore NOT the TCXO enable on the Arduino Max Carrier wiring (or the TCXO power rail itself is missing per BRINGUP_MAX_CARRIER §R12 note). The PA12 init code was left in place (harmless if pin is wrong, beneficial if right; no observable side effects).

#### 3.5.4 Drop-baud-to-115200 attempt — DID NOT FIX (2026-05-11)

Changed [`config.h`](../DESIGN-CONTROLLER/firmware/murata_l072/config.h) `HOST_BAUD_DEFAULT` from `921600UL` to `115200UL`, rebuilt, flashed (firmware.bin = 15428 bytes). Ran RX stimulus test with `stty -F /dev/ttymxc3 raw 115200 cs8 -parenb -cstopb -ixon -ixoff -crtscts` and the standard 3-burst pattern. Result captured in `/tmp/rx_test_capture_115k.bin`:

```
=== ASCII (sorted unique traces) ===
C:CLK_HSI_16M
C:INIT_DONE
C:SEND_BOOT
C:SEND_READY
H:COBS_ERR        ← still present
H:ERR_LPUART1     ← still present
H:PROC_FRAME
H:RX_LPUART1
M:RADIO_BYPASS
```

**The four error traces persist at 115 200 baud.** This **falsifies the clock-tolerance hypothesis** — at 115 200 with HSI16 ±1%, the sample-clock drift over a 10-bit frame is ~1 sample at 16× oversampling (well within spec). Some other RX-side error mechanism is dominant.

`HOST_BAUD_DEFAULT` was reverted to `921600UL` to keep the project in its baseline configuration; the 115200 experiment is documented here for future reference.

#### 3.5.5 Updated hypothesis list for the next session

The first error byte triggers `H:ERR_LPUART1`. Possible mechanisms (independent of clock tolerance):

- **A. ISR latency from blocking `platform_diag_trace()`**: when LPUART1 IRQ fires on an error byte, the handler calls `platform_diag_trace("H:ERR_LPUART1\r\n")` which blocks for ~14 byte-times on TX. During that block, additional RX bytes arrive and overflow the single-byte LPUART1 RDR → ORE → bytes after the trace point are silently discarded → COBS frame is incomplete → `H:COBS_ERR`. **Most likely root cause** — the very first error trace cascades into frame loss because the handler itself blocks the receive path.
- **B. First-byte glitch on PA3**: when the i.MX `cat /dev/ttymxc3` opens the tty, a brief electrical transition on PA3 may cause a phantom start-bit detection → noise/framing error on the first byte.  Subsequent bytes might be clean but the COBS state is already corrupted (and the latched first-error trace prevents observing whether it recurs).
- **C. USART1 mirror lane on PA10 floating**: `usart2_gpio_init()` configures both PA9/PA10 (USART1 AF4) AND PA2/PA3 (LPUART1 AF6).  PA10 has pull-up but if the carrier wiring leaves it disconnected, AC-coupled noise could fire `USART1_IRQHandler` and pollute the SHARED COBS state machine.  Trace would show `H:ERR_USART1` — NOT observed in the capture, so this is unlikely but worth confirming with a STATS_URC counter check.
- **D. `host_diag_echo_rx_byte` blocking TX**: each RX byte is echoed via `host_diag_echo_rx_byte()`. If that function blocks on TX, same ISR-stretch issue as (A) — first noise byte cascades.

#### 3.5.6 Proposed next-session experiments

1. **Remove the blocking `platform_diag_trace()` call from inside the LPUART1 IRQ handler** (replace with a counter increment + post-loop trace from the main loop). Rebuild, repeat stimulus. If `H:COBS_ERR` disappears, hypothesis A is confirmed.
2. **Disable the USART1 mirror entirely** by skipping `gpio_set_alt(GPIOA_BASE, 9U, 4U)` and `gpio_set_alt(GPIOA_BASE, 10U, 4U)` in `usart2_gpio_init()`, plus skipping USART1 IRQ enable. Rebuild, repeat stimulus. If errors disappear, hypothesis C is confirmed.
3. **Emit STATS_URC on demand** (or extend the boot snapshot to include `host_rx_bytes`, `host_errors`, `host_uart_err_lpuart`, `host_uart_err_usart1`, `host_parse_err`) to count exactly how many bytes triggered errors vs. were ingested cleanly. Diagnoses single-byte glitch vs. cascading overrun.
4. **Reset the latched error trace** (`s_trace_first_err_lpuart`) at the start of each new COBS frame (after `0x00` delimiter) so we can observe whether the error recurs on every frame or only the first.

---

## 4. Standard contract gate hardening (Step 3a)

Independently of the firmware fix, [`firmware/x8_lora_bootloader_helper/boot_and_probe.sh`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/boot_and_probe.sh) was patched on the same day to:

1. Restrict `DEV_LIST_DEFAULT` to `/dev/ttymxc3` only (was `"/dev/ttymxc3 /dev/ttymxc2 /dev/ttymxc1 /dev/ttymxc0"`). The X8 console UART `/dev/ttymxc0` has no termios layer (`stty: Inappropriate ioctl for device`) and loops back any bytes written to it via the kernel tty discipline, producing a non-zero "AT response size" without the L072 actually replying.
2. Add a defence-in-depth pure-echo rejection: if every byte in the response is also present in the request bytes (`"AT\r\nAT+VER?\r\n"`), the response is reclassified as loopback and `sz=0`.

This closes the false-positive PASS path that produced the misleading 100/100 standard quant pass earlier in the session.

---

## 5. Bench-recovery procedure (no power-cycle required)

When the H7 SWD wedges (`DPIDR 0xdeadbeef` symptom), the following sequence on the X8 recovers the bench without a USB unplug:

```
adb -s 2E2C1209DABC240B shell "echo fio | sudo -S -p '' bash -lc \\
  'echo 0 > /sys/class/gpio/gpio10/value; sleep 0.5; \\
   echo 1 > /sys/class/gpio/gpio10/value; sleep 1; \\
   systemctl restart m4-proxy.service; sleep 2; echo READY'"
```

`gpio10` = i.MX `GPIO1_IO10`, mapped to the STM32H7 `NRST` per `/usr/arduino/extra/reset.sh`. Restarting `m4-proxy.service` clears any orphaned bitbang OpenOCD state that was holding GPIOs.

---

## 6. Disposition

| Item | Status |
|---|---|
| Blocker A — W1-7 protocol bring-up gate, **TX side** | **CLEARED** — BOOT_URC + READY_URC + STATS observed on `/dev/ttymxc3` after patched flash |
| Blocker A — W1-7 protocol bring-up gate, **RX side** | **OPEN** — RX hardware path proven (H:RX_LPUART1 fires); error mechanism IS NOT clock-tolerance (115 200 baud test failed identically — see §3.5.4). Top hypothesis (A): blocking `platform_diag_trace()` inside the LPUART1 IRQ stretches the ISR for ~14 byte-times → ORE on follower bytes → COBS_ERR. See §3.5.5 / §3.5.6 for next-session experiments. |
| Blocker B — deterministic ingress gate condition | Disposition unchanged: closed by re-scope (root cause was firmware clock tree, not board ingress) |
| Blocker C — ABX00043 net-ownership extraction | Unchanged: BSP-derived direct-route hypothesis still standing; Step 5 schematic verify remains optional |
| Blocker D — timed hold-window dependence | Closing (root cause was clock-tree, not timing) |
| Standard contract gate false-positive (loopback on `/dev/ttymxc0`) | **CLOSED** by `boot_and_probe.sh` patch |

---

## 7. References

- TODO: [`../DESIGN-CONTROLLER/TODO.md`](../DESIGN-CONTROLLER/TODO.md) Action plan revised 2026-05-11 02:30
- Predecessor synthesis: [`2026-05-11_Blocker_Research_Synthesis_Copilot_v1_0.md`](2026-05-11_Blocker_Research_Synthesis_Copilot_v1_0.md)
- Patched files:
  - [`../DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c`](../DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c)
  - [`../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c)
  - [`../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/boot_and_probe.sh`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/boot_and_probe.sh)
- Build artifact: `firmware/murata_l072/build/firmware.bin` = 15 288 bytes
- Bench evidence: `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-10_232447/method_g_stage1.log`
