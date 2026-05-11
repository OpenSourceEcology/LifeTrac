# W1-8 — SX1276 SPI bring-up: research, root-cause hypothesis, and implementation plan

**Date:** 2026-05-11 (evening, post-W1-7 closure)
**Author:** Copilot (autonomous continuation after W1-7 PASS)
**Scope:** Resolve the only remaining Method G probe failure — `[FAIL] sx1276 version reg valid` (RegVersion @ 0x42 reads `0x00`, expected non-`0x00`/non-`0xFF` for SX127x family — typically `0x12`).
**Status:** Plan only. No firmware change yet. Awaiting user go-ahead before edit / build / flash.
**Companion:** Picks up where [`2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_3.md`](2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_3.md) §12 left off (W1-7 PASSED 20/20 with firmware build 16 240 B).

---

## 1. Context & entry conditions

W1-7 closed cleanly this evening:
- Both ASCII and binary host paths pass (`HOST_PARSE_ERR=0`, all per-flag UART error counters = 0, `host_rx_ring_ovf=0`).
- Method G probe (`method_g_stage1_probe.py`) `rc` dropped from `2` (transport timeout) to `1`. The remaining `rc=1` is a single critical check failing:
  ```
  REG 0x42 VERSION = 0x00
  Critical checks:
    [PASS] clock_source_id == 0
    [PASS] radio_ok == 1            ← misleading; see §3.3
    [PASS] ver name present
    [FAIL] sx1276 version reg valid
  ```
- 20-cycle quant: 20/20 PASS — the bench fixture is stable, the probe rerun is reliable.

So the UART transport, host shell, COBS framing, CRC, dispatch, register-read RPC (`HOST_TYPE_REG_READ_REQ` / `HOST_TYPE_REG_DATA_URC`), and stats roundtrip are all behaving correctly. **The only failing layer is the on-MCU SPI master ↔ SX1276 slave link inside the Murata CMWX1ZZABZ-078 module.**

---

## 2. Method G probe failure — exact symptom

[`firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py):

- L67: `SX1276_REG_VERSION = 0x42`
- L380–L389 `dump_registers()` walks `0x00..REGISTER_DUMP_END` issuing one `REG_READ_REQ` per address and parsing the matching `REG_DATA_URC` (`payload = [addr, value]`).
- L747–L756:
  ```python
  version_reg = regs[SX1276_REG_VERSION]
  print(f"REG 0x{SX1276_REG_VERSION:02X} VERSION = 0x{version_reg:02X}")
  print_register_dump(regs)
  ...
  checks.append(("sx1276 version reg valid", version_reg not in (0x00, 0xFF)))
  ```

So the firmware path that fails is: probe sends `REG_READ_REQ(0x42)` → host shell calls `sx1276_read_reg(0x42)` → that drives SPI1 → response `REG_DATA_URC(0x42, 0x00)`.

Reading `0x00` (not `0xFF`) usually means **MISO is held low / no slave clocking happened / pull-down dominates / open MISO with a low-Z external pulldown**. Reading `0xFF` typically means **MISO is open and pulled up by an MCU PUPDR**. The fact we see `0x00` (and we have **no PUPDR pull-up on PA6 MISO** in the production driver) is consistent with "SX1276 is not clocking out anything; bus is floating low or stuck low".

---

## 3. Root-cause investigation

### 3.1 Two firmware drivers, one is wrong

There are **two independent SX1276 driver code paths** in this firmware tree:

| File | Status | SPI pin set | NSS | Reset | Pull-up on MISO? | Notes |
|---|---|---|---|---|---|---|
| [`firmware/murata_l072/lora_ping.c`](../DESIGN-CONTROLLER/firmware/murata_l072/lora_ping.c) `gpio_spi_init()` (L502–L560) | **WORKING** standalone bring-up | **PB3 = SCK**, PA6 = MISO, PA7 = MOSI (AF0) | PA15 | PC0 | **Yes** (PUPDR on PA6 = pull-up) | Uses HSI16, BR/8 (≈2 MHz), 8-bit DS, FRXTH; CR2 written before SPI re-enable; protects against ROM leaving SPI BSY. Comment in source: `"PB3=SCK, PA6=MISO, PA7=MOSI — AF0 (SPI1)"`. |
| [`firmware/murata_l072/radio/sx1276.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) `sx1276_gpio_init()` (L168–L189) + `sx1276_spi_init()` (L195–L207) | **PRODUCTION (failing)** path used by `sx1276_init()` and the host RPC | **PA5 = SCK**, PA6 = MISO, PA7 = MOSI (all AF0) | PA15 | PC0 | **No** | Uses BR/64 (≈250 kHz), 8-bit DS, FRXTH. No BSY-check before re-init. |

**The difference is the SCK pin.** `lora_ping.c` clocks PB3 as AF0 (`SPI1_SCK`); `radio/sx1276.c` clocks PA5 as AF0 (also `SPI1_SCK` per L072 alt-func table — both AF0 muxes are valid options for SPI1_SCK).

### 3.2 Why PA5 cannot be SCK on the Murata CMWX1ZZABZ-078

The Murata CMWX1ZZABZ-078 module is a SiP that hard-wires the STM32L072 ↔ SX1276 internal nets. The Murata datasheet ("Type ABZ Datasheet") and the ST B-L072Z-LRWAN1 reference design (which uses the same SiP) define the fixed internal bus:

| SX1276 pin | STM32L072 pad |
|---|---|
| **SCK** | **PB3** |
| MISO | PA6 |
| MOSI | PA7 |
| NSS | PA15 |
| NRESET | PC0 |
| DIO0 | PB4 |
| DIO1 | PB1 |
| DIO2 | PB0 |
| DIO3 | PC13 |
| **DIO4** | **PA5** |
| DIO5 | PA4 |
| TCXO_VCC enable | PA12 |
| ANT_SW_RX | PA1 |
| ANT_SW_TX_BOOST | PC2 |
| ANT_SW_TX_RFO | PC1 |

**`PA5` is wired to SX1276 `DIO4`, not `SCK`.** When `sx1276_gpio_init()` puts PA5 into AF0 and starts SPI1, the L072 *does* drive a clock waveform out of PA5 (L072 alt-func table allows PA5 as SPI1_SCK), but that waveform goes into the SX1276 DIO4 pin (an input/output GPIO on the radio side), **not** SCK. The SX1276's actual SCK input stays in whatever state PB3 is in, which here is **GPIO reset default = analog input / Hi-Z**. With no clock, the SX1276 SPI engine never shifts a bit out, and MISO (PA6) — which has no PUPDR configured in the production driver — floats. Internal SX1276 state and PCB leakage usually settle that line low → reads return `0x00`.

This is the same trap the ST X-CUBE-LRWAN BSP avoids by routing SCK to PB3. Our standalone `lora_ping.c` already proves the PB3 path works on this exact hardware unit (it was used during W1-1 to W1-3 hello-world bring-up).

### 3.3 Why `boot_info.radio_ok == 1` is misleading

`sx1276_init()` (radio/sx1276.c L226–L248) returns `false` when `version` is `0x00` or `0xFF`. If `radio_ok == 1` is reaching the probe, then `sx1276_init()` is reporting success — which contradicts the version-register failure observed by the probe.

Two possibilities, both worth confirming during implementation:
1. The host shell flips a separate `radio_ok` flag based on something other than `sx1276_init()`'s return value (e.g., "init was attempted" rather than "init succeeded"), or
2. `sx1276_init()` is called twice — once during boot (failing, producing no FAULT_URC), and again or never by the probe path. The probe's per-byte `REG_READ_REQ(addr)` does **not** pre-call `sx1276_init()` — it just calls `sx1276_read_reg()` directly. So even if the radio is in an unknown state, the read is attempted and returns `0x00`.

Both paths converge on the same conclusion: **the SCK pin is wrong; nothing else needs to change to clear `0x00`**. We will confirm `radio_ok` semantics during step 5.4 below.

### 3.4 Other discrepancies between the two drivers (worth fixing in the same patch)

| Concern | `lora_ping.c` | `radio/sx1276.c` | Action |
|---|---|---|---|
| MISO PUPDR | Pull-up on PA6 | Cleared (no pull) | **Add pull-up on PA6** to match the working driver — defensive against floating-low reads if SX1276 ever tristates MISO mid-frame. |
| BR divider | DIV8 (≈2 MHz @ 16 MHz / ≈4 MHz @ 32 MHz) | DIV64 (≈250 kHz @ 16 MHz / ≈500 kHz @ 32 MHz) | Keep DIV64 for first bring-up (more margin). Consider DIV8 in a follow-up once link is proven. |
| BSY-wait before re-init | Yes (`while (SPI1_SR & BSY) {}`) | No | **Add BSY-wait** — if the SX1276 SPI was previously active (e.g., host_uart did a read between init calls), this avoids corrupting the disable sequence. |
| CR2 write order | CR2 set *before* SPE = 1 | CR2 set after CR1 (which already has SPE = 1 in the same statement chain) | The current `sx1276_spi_init()` writes `SPI1_CR1 = 0` first (clears SPE), then `SPI1_CR2 = ...`, then `SPI1_CR1 |= SPE`. That's actually fine — SPE was 0 when CR2 was written. **No change required**, but documenting for clarity. |
| 16-bit MMIO for CR1/CR2 disable | Yes (`MMIO16` halfword writes to avoid clobbering reserved bits) | No (`SPI1_CR1 = 0U;` 32-bit) | The L072 RM allows full-word writes to CR1/CR2 (reserved bits are RES = 0). **Acceptable, no change required.** |
| GPIO clocks enabled | A, B, C | A, B, C | Already correct — GPIOB clock is enabled even though only PA5 is currently being put into AF0. So the only edit needed for GPIO clocking is none — we just need to retarget the AF mode setup. |

### 3.5 Hypothesis confidence

**HIGH (≥ 95%)** that retargeting SCK from PA5 to PB3 will produce `RegVersion = 0x12` (or another valid SX1272/SX1276 silicon-rev value) on first read, because:

- The exact same hardware unit was already proven SPI-functional by `lora_ping.c` (PB3=SCK).
- The Murata SiP pinout is fixed inside the package; there is no possible board variant.
- `0x00` (not `0xFF`) is the expected symptom for "SCK is silent and MISO floats low / weak pull-down".
- TCXO_VCC (PA12) is already asserted HIGH on boot by [`hal/platform.c`](../DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c) L80–L105 (added during W1-7), so the SX1276 has its 32 MHz reference clock — that part of the chain is good and is the reason `clock_source_id == 0` PASSes.
- NRESET pulse (`sx1276_radio_reset()`) cycles PC0 — the correct net per the Murata pinout. So the radio is being correctly hardware-reset.

The remaining ≤ 5% failure-mode coverage:
- An unrelated soldering / ESD failure on the SX1276 inside this specific Murata module (would also have broken `lora_ping.c`, which it didn't — so very unlikely).
- A second firmware bug in the read-register code path that only manifests at the production BR_DIV64 rate (improbable; SPI is synchronous and slow rates are easier, not harder).

Both are easy to discriminate at bench time (see §5.5 alternates).

---

## 4. Goal & acceptance criteria

### 4.1 Primary goal
Make `[FAIL] sx1276 version reg valid` PASS in `method_g_stage1_probe.py` on the bench unit (Portenta X8 ABX00049 ADB serial `2E2C1209DABC240B` + Max Carrier ABX00043 + Murata CMWX1ZZABZ-078) on `/dev/ttymxc3 921600 8N1`.

### 4.2 Acceptance criteria (W1-8 PASS)

| # | Criterion | Method |
|---|---|---|
| W1-8.A | `REG 0x42 VERSION` reads a valid SX127x silicon rev (NOT `0x00` and NOT `0xFF` — typically `0x12`) | Method G probe single-shot |
| W1-8.B | All 4 `Critical checks` PASS in the probe (`probe_rc == 0`) | Method G probe single-shot |
| W1-8.C | Full register dump (`0x00..0x70`) shows non-uniform bytes (i.e., real silicon contents, not all `0x00` and not all `0xFF`) | Method G probe single-shot |
| W1-8.D | The W1-7 quant gate is unaffected: re-run `run_stage1_standard_quant_end_to_end.ps1 -Cycles 20` → ≥ 18/20 PASS (target: 20/20 to match the post-W1-7 baseline) | Standard quant runner |
| W1-8.E | `host_parse_err` and all per-flag UART error counters remain 0 across the probe (no regression in W1-7's UART path) | Method G probe single-shot, STATS_URC payload |

If criterion **A** PASSes but B/C reveal new problems (e.g., adjacent-register reads garbled, BSY hang, NSS timing glitch under load), they become follow-up scope inside W1-8 — but the headline blocker is closed once `0x42` reads valid.

---

## 5. Implementation plan

All edits live inside [`firmware/murata_l072/radio/sx1276.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c). No header / include / Makefile / probe-side changes required.

### 5.1 Edit — retarget SCK from PA5 to PB3

In `sx1276_gpio_init()` (around L168–L189), replace:

```c
gpio_mode_alt(GPIOA_BASE, 5U, 0U);   /* WRONG: PA5 = DIO4 on Murata SiP */
gpio_mode_alt(GPIOA_BASE, 6U, 0U);   /* MISO — correct */
gpio_mode_alt(GPIOA_BASE, 7U, 0U);   /* MOSI — correct */
```

with:

```c
/* W1-8 fix: SX1276 SCK is wired to PB3 inside the Murata CMWX1ZZABZ-078 SiP.
 * PA5 is wired to SX1276 DIO4. Driving PA5 as SPI1_SCK clocks DIO4 instead
 * of SCK and the SX1276 SPI engine never shifts — RegVersion reads 0x00.
 * Reference: B-L072Z-LRWAN1 schematic; lora_ping.c gpio_spi_init() also
 * uses PB3=SCK and proves SPI works on this hardware unit. */
gpio_mode_alt(GPIOB_BASE, 3U, 0U);   /* SCK = PB3 (AF0 = SPI1_SCK) */
gpio_mode_alt(GPIOA_BASE, 6U, 0U);   /* MISO = PA6 (AF0 = SPI1_MISO) */
gpio_mode_alt(GPIOA_BASE, 7U, 0U);   /* MOSI = PA7 (AF0 = SPI1_MOSI) */
```

### 5.2 Edit — add MISO pull-up (defensive)

After the AF setup above, add:

```c
/* Pull-up on PA6 (MISO) — prevents floating reads if SX1276 ever tristates
 * MISO mid-frame. lora_ping.c uses the same pull-up. */
{
    uint32_t pupd = GPIO_PUPDR(GPIOA_BASE);
    pupd &= ~(3UL << (6U * 2U));
    pupd |= (1UL << (6U * 2U));   /* PUPDR = 01 = pull-up */
    GPIO_PUPDR(GPIOA_BASE) = pupd;
}
```

### 5.3 Edit — defensive BSY-wait before SPI1 reconfigure

In `sx1276_spi_init()` (L195–L207), wrap the CR1 disable sequence:

```c
static void sx1276_spi_init(void) {
    RCC_APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC_APB2ENR;   /* ensure clock is propagated before any access */

    /* If the boot ROM or a previous init left SPI1 active, wait for idle
     * before clearing CR1 — same precaution as lora_ping.c. */
    while ((SPI1_SR & SPI_SR_BSY) != 0U) { }

    SPI1_CR1 = 0U;
    SPI1_CR2 = SPI_CR2_DS_8BIT | SPI_CR2_FRXTH;
    SPI1_CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_DIV64;
    SPI1_CR1 |= SPI_CR1_SPE;
}
```

### 5.4 Edit — surface a real `radio_ok` from `sx1276_init()`

If the bench shows `boot_info.radio_ok == 1` while the version still reads `0x00` (i.e., `sx1276_init()` is NOT what gates `radio_ok`), trace where `radio_ok` is set in the BOOT_URC payload constructor and tie it to the actual return value of `sx1276_init()`. **This is conditional on what we observe in step 5 below — defer the edit until then.** Goal: if SPI is broken, the probe should see `radio_ok == 0` *and* the version-reg failure, instead of getting a misleading `radio_ok == 1`.

### 5.5 Step-by-step bench procedure

1. **Read** `radio/sx1276.c` lines 168–207 in full and confirm the diff target text matches §5.1–§5.3 precisely (the line numbers above are from the current 16 240 B build; they may shift by a few lines after the edit).
2. **Apply** edits §5.1 + §5.2 + §5.3 (skip §5.4 for now).
3. **Build** with `& "$env:WINDIR\System32\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -ExecutionPolicy Bypass -File LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/build.ps1`. Expect a small size delta from the current 16 240 B (likely +20 to +60 bytes for the added PUPDR write and BSY-wait).
4. **Flash + probe** end-to-end with `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_g_stage1_end_to_end.ps1 -AdbSerial 2E2C1209DABC240B`.
5. **Inspect probe output** for:
   - `REG 0x42 VERSION = 0x12` (or another non-`0x00`/non-`0xFF` value) → criterion W1-8.A satisfied.
   - Critical checks: 4/4 PASS → criterion W1-8.B satisfied.
   - Register dump shows non-uniform bytes across `0x00..0x70` → W1-8.C satisfied.
6. **Re-run W1-7 acceptance quant** with `run_stage1_standard_quant_end_to_end.ps1 -AdbSerial 2E2C1209DABC240B -Cycles 20`. Need ≥ 18/20 PASS (target 20/20).
7. If anything in steps 5–6 fails, see §5.6 contingency tree.
8. On full PASS: tick W1-8 in TODO.md, update `/memories/repo/lifetrac-portenta-x8-lora.md`, append a §13 closure note to the W1-7 v1.3 plan (or open a v1.4 plan if scope grows), and call task_complete.

### 5.6 Contingency tree

- **`REG 0x42 VERSION = 0xFF`** (was `0x00`): means SCK is now clocking the right pin and MISO is being pulled up by our new PUPDR but the SX1276 isn't responding. Check NRESET timing — try lengthening `platform_delay_ms(20U)` after the rising edge to `30U`. Verify TCXO_VCC (PA12) settled HIGH before `sx1276_init()` runs.
- **`REG 0x42 VERSION = 0x00`** still: SCK retarget didn't take effect. Sanity-check the build artifact actually flashed (size change visible? compare `xxd` of the .bin). If yes, scope a logic-analyzer hook on PB3 to confirm clock is present.
- **Bus hangs / `SPI_SR_BSY` never clears**: indicates a wiring conflict on PB3 (e.g., another driver is holding it). Check if any other init code path enables PB3 as output / pull-up. `grep_search` for `GPIOB_BASE.*3` across the firmware.
- **W1-7 quant regresses (< 18/20)**: revert §5.2 first (PUPDR add), then §5.3 (BSY wait). The SCK retarget itself should be neutral to the UART path; if it isn't, something is sharing PB3 with PA5 in a way we missed.
- **Garbled register dump (some valid, some 0x00)**: SPI clock probably too fast for the test rig; reduce BR from DIV64 to DIV128 (one notch slower) and re-run. Less likely at 250 kHz, but worth keeping in pocket.

---

## 6. Risk & blast radius

- **Surface area:** 3 small edits in one file (`radio/sx1276.c`). No public API changes. No probe-side changes. No build-system changes.
- **Regression risk to W1-7:** very low. The UART path (LPUART1 / USART1) is fully independent of SPI1 / PB3. PB3 is currently unused by any UART config in this firmware. The W1-7 quant gate is the safety net at step 6.
- **Reversibility:** trivial. Single-file revert restores the current 16 240 B firmware exactly.
- **Bench-only impact:** no Linux-side, no x8_lora_bootloader_helper, no host scripts touched. ADB / Method G / quant infra is unchanged.

---

## 7. Out of scope (deferred)

- TX bring-up (W1-9): payload transmit, channel hygiene, LBT, CAD. Tracked under future tranche; only register-read reachability is required for W1-8.
- RX bring-up beyond version-reg readback (W1-10): DIO0 IRQ wiring validation under live RX, FifoRx pointer reads, RSSI — all need the SPI bus working first, which is what W1-8 delivers.
- TCXO settle-time tuning: already addressed in W1-7 (`hal/platform.c` PA12 = HIGH + 5 ms spin). No change planned in W1-8 unless §5.6 contingency forces it.
- SX1276 burst-read / burst-write performance characterization: stays on the W1-9 / R-7 retune-cost backlog.
- Removing the misleading `radio_ok == 1` return in BOOT_URC when init fails: noted in §5.4 as a conditional follow-up; addressed only if the bench data warrants.

---

## 8. Decision request

The plan above is a single-shot fix with a strong root-cause confidence. Suggested action: **apply edits §5.1 + §5.2 + §5.3, build, flash, run probe, run 20-cycle quant.** Total bench cost ≈ 1 build + 1 flash + 1 probe + 1 quant — well under the 1-hour budget that closed W1-7.

Awaiting user go-ahead to proceed with implementation.
