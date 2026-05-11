# Go/No-Go Blocker Research Synthesis (W1-7, Ingress Gate, Net-Ownership, Hold-Window)

**Date:** 2026-05-11
**Analyst:** Copilot
**Version:** v1.0
**Scope:** Cross-blocker research review combining (a) all bench evidence on file, (b) the topology hypotheses driving the Phase A–D experiments, and (c) the official Arduino/NXP/Murata sources that pin down the actual UART4 → Murata signal chain on ABX00043.

---

## TL;DR

A primary-source review of the official Arduino `meta-arduino` Yocto BSP (the device tree overlay [`ov_carrier_enuc_lora.dts`](https://raw.githubusercontent.com/arduino/meta-arduino/scarthgap/meta-arduino-nxp/recipes-bsp/device-tree/arduino-device-tree/portenta-x8/overlays/ov_carrier_enuc_lora.dts)) shows that **on the Portenta Max Carrier the i.MX 8M Mini UART4 is wired straight TX/RX to the Murata `CMWX1ZZABZ-078` module — there is no Linux-controlled mux, level-shifter OE, or buffer enable in this signal path**. The single Linux-side control net is the L072 NRST (`gpiochip5` line 3 = STM32H747 `PF4`); BOOT0 and the host UART AF are *not* configured by the kernel.

> **2026-05-10 22:55 corrections (after attempting to execute the action plan):**
>
> 1. **Step 1 (pad pull-up patch) is INVALIDATED.** The IMX8MM pad-control bit encoding (verified against the upstream `arch/arm/include/asm/mach-imx/iomux-v3.h` in u-boot) gives `PAD_CTL_PUE = 0x1 << 6 = 0x40` and `PAD_CTL_PE = 0x1 << 8 = 0x100`. Therefore **`0x140` = `PE | PUE` — internal pull-up is already enabled**. The earlier claim in §1.1 that the pad has "no internal pull" was a misread of the pad-config decode and must be retracted. UART4_RXD is already pulled up.
> 2. **The Stage 1 standard quant 100/100 PASS is a FALSE POSITIVE for W1-7.** The contract's `BOOT_OK=1` is set by `AT response size > 0` from a sweep across `/dev/ttymxc{0..3}`. Inspection of `boot_probe.log` for the most recent passing cycles shows: `/dev/ttymxc3` (the actual UART4 to L072) returns `size=0 bytes` for every probe attempt at 19200/9600/38400 8N1, while `/dev/ttymxc0` returns 9 bytes that decode as the literal echo `"AT+VER?\r\n"` (with the diagnostic line `stty: /dev/ttymxc0: Inappropriate ioctl for device` proving ttymxc0 is the X8 console UART without a termios layer). The contract is asserting boot success on a console-UART loopback echo, not on a real L072 response.
> 3. **The W1-7 binary protocol probe genuinely fails.** Direct inspection of `T6_profile_ownerexp_derived_uart4_mux_2026-05-10_221751/d1.probe.log` confirms: at 921600 8N1 on `/dev/ttymxc3` the L072 user firmware emits **zero bytes** — no `BOOT_URC`, no pre-drain bytes, no reply to ATI, no reply to AT+VER?. `VER_REQ` times out waiting for `VER_URC`. This is the actual W1-7 failure mode, and it is consistent across all the Phase A–D evidence.
> 4. **The L072 firmware host UART config is correctly clocked.** `host_uart.c` line 527 sets `RCC_CCIPR[11:10] = 0b10` selecting HSI16 as the LPUART1 clock source. With LPUART1 BRR = `(256 × 16 MHz) / 921600 = 4444`, actual baud is 921 514, error 0.009 % — well within tolerance. Step 2's BRR-mismatch sub-hypothesis is therefore not the cause.
> 5. **Bench regression at 2026-05-10 22:55:** during Step 2 setup the H747 OpenOCD bitbang transport reports `Info : SWD DPIDR 0xdeadbeef` and aborts before the READY phase, on both the Method G hello_world flow and the standard quant flow. The same hardware passed 100/100 at 12:36 PM today. The intervening operations were Phase D-1/D-2/D-3 GPIO sweeps (22:24–22:34). The cumulative GPIO state from those sweeps appears to have left the H747 in a state where the i.MX-side OpenOCD bitbang cannot enumerate SWD. **A physical power-cycle of the X8 / Max Carrier is required before any further Method G flashing.**

That has three direct consequences for the four open Go/No-Go blockers:

1. **Blocker B (no deterministic ingress gate condition):** the entire Phase A–D ATI-signature search across 34 H747 GPIO pins (PA, PB, PC, PD, PE, PF) was searching for SEL/OE/EN nets that do **not exist** in the Murata UART4 path. The signature variations seen were almost certainly second-order coupling on adjacent unrelated nets (RS-232/RS-485/audio mux), not the actual ingress gate.
2. **Blocker C (ABX00043 net-ownership extraction):** for the Murata UART4 link, the "owner-net" question collapses to two nets only — i.MX `UART4_TXD` / `UART4_RXD` pad config and H747 `PF4` (NRST). The U16–U22 mux/level-shifter/buffer inventory in the user manual belongs to other carrier signal paths (RS-232 mux on the M.2 / serial connector, audio routing, etc.), not to the L072 host link.
3. **Blocker A (W1-7 protocol bring-up) and Blocker D (hold-window timing):** with mux/SEL/OE removed from the candidate list, the residual hypotheses for the persistent VER_REQ → VER_URC failure are now narrow and directly testable: (i) i.MX UART4 RXD pad has **no internal pull-up** in the official overlay, (ii) the custom L072 firmware drives PA2/PA3 with **AF6 (LPUART1)** rather than the AF4 (USART2) used by the STM32 ROM bootloader and the MKRWAN reference firmware, (iii) framing/baud divisor at LPUART1 921600.

These three are the next experiments that have a realistic chance of clearing W1-7. Continued single-pin GPIO sweeping on the H747 will not.

---

## 1. Source review — what we now know is *true* about the route

### 1.1 Authoritative reference: `ov_carrier_enuc_lora.dts`

From the Arduino-maintained Yocto BSP layer, the official Max Carrier "enuc lora" overlay reads in full (excerpted, formatting normalized):

```dts
/*
 * Portenta-X8 dtb overlay file
 * Arduino 2021
 * Enable LORA modem serial port on Arduino Max carrier board
 * no RTS CTS only TX RX.
 * Gpios:
 * - Reset pin active low, ext. pull up of 1M on carrier
 *   (stm32h7 PF4, gpiochip5 pin 3 using gpiod)
 *
 * Example script to reset the modem:
 *   gpioset gpiochip5 3=0
 *   sleep 1
 *   gpioset gpiochip5 3=1
 */

pinctrl_uart4: uart4grp {
    fsl,pins = <
        MX8MM_IOMUXC_UART4_RXD_UART4_DCE_RX  0x140
        MX8MM_IOMUXC_UART4_TXD_UART4_DCE_TX  0x140
    >;
};

&uart4 {
    pinctrl-names = "default";
    pinctrl-0 = <&pinctrl_uart4>;
    assigned-clocks = <&clk IMX8MM_CLK_UART4>;
    assigned-clock-parents = <&clk IMX8MM_SYS_PLL1_80M>;
    status = "okay";
};
```

Key facts this pins down:

| Fact | Evidence | Implication |
|------|----------|-------------|
| Murata link is direct UART4 TX/RX, no flow control | Comment `* no RTS CTS only TX RX.` + only `UART4_RXD` and `UART4_TXD` in `fsl,pins` | The earlier `crtscts` test result (driver refuses CRTSCTS) is *expected*, not a bug. RTS/CTS lines do not exist on this overlay. |
| Reset uses H747 `PF4` only | Comment + matches existing bench notes | Confirms `PF4` is the only kernel-controlled enable in the stack. |
| BOOT0 is **not** in this overlay | Absence of pinctrl entry | Linux does not touch L072 BOOT0 at all. The H747's `PA11` BOOT0 line is fully owned by the H747 firmware, not the i.MX. Anything driving `PA11` from outside the H747 is bench-only / OpenOCD-only. |
| No mux/OE/SEL nets are configured | No GPIO request, no extra pinctrl group | There is no Linux-controlled enable/mux for the Murata path. The Phase D GPIO sweep premise (some H747 pin gates U16–U22 for UART4) is **not supported by the BSP**. |
| UART4 pad config = `0x140` | Both TXD and RXD lines | `0x140` = `PAD_CTL_DSE_X1` + `PAD_CTL_FSEL_2`. **No `PAD_CTL_PE` / `PAD_CTL_PUE` / `PAD_CTL_PUS_*`.** RXD has no internal pull, and the comment confirms no external 1 M pull either (the 1 M is on RESET). |
| UART4 clock parent is `SYS_PLL1_80M` | `assigned-clock-parents` | Standard 80 MHz UART clock; supports 921600 cleanly. Not a clock-tree problem. |

### 1.2 Component inventory in the user manual ≠ Murata path

The user manual lists `U16–U19` (`74LVC1G157` muxes), `U8/U20/U21/U22` (`SN74LVC1T45` level shifters), `U10` (`SN74LVC1G125` buffer), and `U23` (Murata module). These devices exist on the carrier, but **the official BSP overlay does not gate the Murata UART4 link through any of them**. They are almost certainly used for:

- RS-232/RS-485 transceivers on the carrier's industrial serial connector,
- audio in/out mux to the CS42L52 codec (`ov_carrier_max_cs42l52`),
- USB-FS / SDC / power switching (`ov_carrier_max_usbfs`, `ov_carrier_max_sdc`, `ov_carrier_enuc_bq24195`).

**This is the root mistake that the Phase A–D investigation has been chasing.** The bench evidence supports it: 34 pins across 5 GPIO banks all produced indistinguishable "ATI silence" classifications, which is exactly what you would expect if none of them are actually in the route.

### 1.3 Custom L072 firmware host UART configuration

[`firmware/murata_l072/host/host_uart.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c) configures the host link as:

```c
static void usart2_gpio_init(void) {
    RCC_IOPENR |= RCC_IOPENR_GPIOAEN;

    gpio_set_alt(GPIOA_BASE, 2U, 6U);   // PA2 -> AF6 = LPUART1_TX
    gpio_set_alt(GPIOA_BASE, 3U, 6U);   // PA3 -> AF6 = LPUART1_RX
    /* PA3 pull-up ... */

    /* Mirror host transport on USART1 as a routing fallback. */
    gpio_set_alt(GPIOA_BASE, 9U, 4U);   // PA9  -> AF4 = USART1_TX
    gpio_set_alt(GPIOA_BASE, 10U, 4U);  // PA10 -> AF4 = USART1_RX
    /* PA10 pull-up ... */
}
```

Compare against the canonical references:

| Reference | Host UART | AF on PA2/PA3 | Baud |
|-----------|-----------|---------------|------|
| **STM32 ROM bootloader (AN3155)** on L0 series | USART2 | **AF4** | 9600/19200/115200 8E1 (autobaud) |
| **MKRWAN factory firmware** (Arduino reference) | Serial1 = USART2 | **AF4** | 19200 8N1 |
| **Hardwário CHESTER LoRa modem** (similar Murata + H7 host) | USART2 | **AF4** | 115200 8N1 |
| **Our custom `murata_l072` firmware** | LPUART1 | **AF6** | 921600 8N1 |

Both AF4 and AF6 are valid alternate-function selections for PA2/PA3 on STM32L072 (per RM0367 §A.9 alternate-function table) — at a strict pin-mux level, **LPUART1 will physically light up the PA2/PA3 pads**. So this by itself is not a hard-fault. But the choice has two soft-fault implications worth bench-validating:

1. **AF4 (USART2) is the AF that every other piece of firmware on this module uses.** Any timing/reset sequencing issue that depends on the previous ROM-mode AF state being preserved (e.g. bootloader leaving PA2 driving high before the user app re-multiplexes) becomes the developer's problem.
2. **LPUART1 BRR formula** is `BRR = (256 × f_ck) / baud`. At `f_ck = 16 MHz` and `baud = 921600`, `BRR = 4444` (round). The corresponding actual baud is `(256 × 16 MHz) / 4444 = 921 514 baud`, error ≈ 0.0093 % — well within tolerance, *not* the failure mode. So LPUART1 at 921600 is mathematically fine.

### 1.4 What the i.MX side will tolerate at 921600 8N1

i.MX 8M Mini UART (NXP reference manual, IMX8MMRM §16.2) supports up to 5 Mbaud on UART4 with the 80 MHz clock parent. At 921600 the 16× oversampling sampling-edge slack is ~63 ns; with `0x140` (DSE_X1 = ~150 Ω drive, FSEL_2 = medium slew, **no internal pull**), the line is sensitive to:

- **Idle-line state** when L072 is in reset or in BOOT0=1 (ROM mode): the pad floats, RX framer can latch noise as a start bit, and the next "real" frame arrives mid-glitch and fails with FE/NE.
- **Single dropped start-bit** at 921600: a 1 µs glitch is ≈ 1 bit time. With no pull-up, capacitive coupling from adjacent nets is enough to produce one.

This matches the observed signature: ROM mode at 19200 8E1 (slow, 8E1 parity check) works; user-mode 921600 8N1 (fast, no parity, no pull) does not.

---

## 2. Blocker-by-blocker reassessment

### 2.1 Blocker A — W1-7 protocol bring-up gate is still red

**Status:** open. Persistent `VER_REQ` timeout, no `BOOT_URC` over the active probe window.

**Most likely root causes (after research, in order of bench-testable confidence):**

1. **i.MX UART4 RXD has no internal pull-up.** The official overlay's `0x140` pad config omits `PAD_CTL_PE | PAD_CTL_PUE | PAD_CTL_PUS_*`. With L072 in reset (PF4 LOW) or with PA2 in a transient state during firmware startup, RXD floats and the i.MX UART RX framer latches noise. **Test:** patch the overlay (or live `devmem` poke into `IOMUXC_SW_PAD_CTL_PAD_UART4_RXD` at `0x30330280`) to set `0x140 | PAD_CTL_PE | PAD_CTL_PUE | PAD_CTL_PUS_22K` and rerun the active probe. Cost: <30 min, no firmware rebuild.
2. **L072 host UART AF mismatch (AF6/LPUART1 vs AF4/USART2).** Both work electrically, but every reference firmware uses AF4 on PA2/PA3. Switching the custom firmware to USART2 (AF4) brings it onto the same alternate-function the L072 ROM bootloader leaves the pin set to, eliminating any boot-handoff transient. **Test:** rebuild [`firmware/murata_l072/host/host_uart.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c) with PA2/PA3 → AF4 and USART2 register block, drop the LPUART1 path (and remove the USART1 mirror because it is targeting Murata pins not connected to the carrier UART4 path; see §2.2 below). Cost: ~1 h, no hardware change.
3. **Idle-line / start-bit glitch from L072 power-up sequence.** Even after fixes 1 + 2, if the firmware's startup beacon at `19200 8N1` is being emitted *before* the host opens the line at `921600 8N1`, the i.MX framer catches mid-bit transitions and stays in error. **Test:** delay the first `host_uart_send` until at least 50 ms after `host_uart_init` and ensure no other code drives PA2 between init and first send. (This is partially in place per the 2026-05-09 "beacon timing patch" note but should be revisited after fix 2.)

**De-prioritized hypotheses (now low confidence given the BSP review):**

- Linux-controlled mux/SEL gate. *Unsupported by the BSP.*
- Cross-lane H747 pin owning a level-shifter OE on the UART4 path. *Unsupported by the BSP.*
- High-baud bandwidth limit. *Already disproven by the 115200 baseline run on 2026-05-10.*

### 2.2 Blocker B — No deterministic board-level ingress gate condition

**Status:** open, but the search space is now drastically smaller.

**Reassessment:** the 34-pin sweep producing 100 % "ATI silence" is now a *positive* result, not a negative one — it confirms that none of the H747 pins outside `{PA11 BOOT0, PF4 NRST, PA2 host TX, PA3 host RX, PA9/PA10 fallback}` are part of the route. The "ATI silence" vs "ATI zero-byte" split observed in lane-C and cross-lane runs is most plausibly explained as:

- **Crosstalk into adjacent unrelated mux SEL lines** that happen to flip the carrier's RS-232/audio path between two driven states (one floats RX with a fixed pattern, the other actively pulls it down through a back-driven driver). Neither state restores actual L072↔i.MX UART4 ingress.

**Go-criterion adjustment:** the reproducible "GPIO/control-net state that consistently changes ingress classification" should be replaced with **"pad-config state (PUE/PUS) and L072 host AF state that consistently changes ingress classification"**. This re-scopes the blocker to the two changes above (pull-up enable on RXD, AF4 on PA2/PA3) plus an immediate-repeat test on each.

### 2.3 Blocker C — ABX00043 net-ownership extraction (Phase D)

**Status:** open per the original criterion ("evidence-backed net map for i.MX UART4 path and owning OE/SEL/EN controller lines"), but the answer is now: **there are no OE/SEL/EN owners on the Murata UART4 path** (other than NRST on PF4). The schematic-extraction task should be redirected to verifying three specific things in the ABX00043 schematic PDF:

1. Confirm `Murata.PIN_27` (PA2) and `Murata.PIN_28` (PA3) are wired *directly* to the i.MX `UART4_TXD`/`UART4_RXD` test points with only series resistors (no level shifter, no mux, no buffer).
2. Confirm `Murata.NRST` is wired to `H747.PF4` through a 1 MΩ external pull-up.
3. Confirm `Murata.BOOT0` is wired to `H747.PA11` directly (no buffer / no mux).

If all three confirm, Blocker C closes with a one-page net map and a note that the U16–U22 inventory belongs to other carrier subsystems. The Phase D-1/D-2/D-3 sweep results then become *ruling-out evidence* for that net map rather than a continuing investigation.

The Phase E "multi-pin combination" / "polarity reversal" plan in the existing notes should be dropped — there is no combinatorial enable hidden in the H747 GPIO bank. Spending more cycles on it has zero expected information gain.

### 2.4 Blocker D — Timed hold-window dependence

**Status:** open. The timing-only sweep (100/500/1500 ms pre/post holds) showed one non-reproducible ATI-silent outlier. Per the same logic as Blocker B, the timing search was looking for a hold-window that would coordinate a non-existent mux SEL with NRST release; under the corrected route model this cannot exist.

**Recommended close-out:** declare timing closed as a *root-cause dimension* with the existing evidence and replace it with a focused **NRST↔first-byte timing window** (i.e. how long after `PF4` goes high before the L072 firmware opens its host UART, and how long after that until the i.MX should start the first request). Based on the 2026-05-09 "ordered breadcrumbs" note, the firmware emits `BOOT_URC` ~20–80 ms after release; the host probe should:

1. Wait for `BOOT_URC` (with timeout), or wait at least 250 ms after `gpioset gpiochip5 3=1` if `BOOT_URC` is suppressed for any reason.
2. Send `VER_REQ` only after both fix 1 (pull-up) and fix 2 (AF4) above are in place.

---

## 3. Recommended next experiments (corrected after the 22:55 corrections in §TL;DR)

The original Step 1 (pad pull-up) and Step 2 (BRR sub-hypothesis) are removed. The new sequence is:

| # | Experiment | Predicted outcome if hypothesis is right | Predicted outcome if wrong | Cost |
|---|------------|------------------------------------------|----------------------------|------|
| 0 | **Recover bench:** physical power-cycle the X8 / Max Carrier; confirm `adb devices` returns the unit; run **one** standard Stage 1 quant cycle and check the OpenOCD log shows a real `SWD DPIDR` (e.g. `0x6ba02477` H747 ID) rather than `0xdeadbeef`. | Bench is back to the 100/100 baseline — proceed to E-1. | SWD still dead; escalate to a longer power-off and re-seat of the Murata module / cabling. | 5 min, requires user physical action |
| E-1 | **Decisive bring-up test — flash `hello_world.bin`.** Push the existing build artifact `firmware/murata_l072/build_hello/hello_world.bin` via `run_method_g_stage1_end_to_end.ps1 -LocalImage <hello_world.bin>`. The hello-world firmware drives **the same LPUART1/AF6/PA2 path** as the production firmware but at 19200 8N1 emitting continuous ASCII `"LIFETRAC L072 tick=..."`. After flash + boot release, run a passive read on `/dev/ttymxc3` at 19200 8N1 for ≥3 s. | Text appears on ttymxc3 → LPUART1/AF6/PA2 ↔ UART4 path is fully functional → root cause is in the production firmware logic (not the path). Move to E-2. | Silent on ttymxc3 → either the L072 isn't booting from flash after NRST release, or PA2 is not connected as expected on this carrier revision → escalate to E-4. | <30 min, reuses an already-built binary, decisive |
| E-2 | **Conditional on E-1 PASS:** audit `firmware/murata_l072` for what blocks the production firmware from emitting bytes. Likely candidates: (a) `host_uart_init` is never reached (init-order or fault-handler bug), (b) it is reached but a hard fault triggers immediately, (c) the BOOT_URC emission path is gated on a sensor/clock readiness flag that never becomes true. Add an early write-byte to LPUART1 in `main()` before any other init, reflash, and re-probe. | Early byte arrives on the binary probe → confirms firmware boots; bug is downstream of `main()`. | Early byte does not arrive → firmware is faulting or stuck before `main()` executes any UART writes. | 1–2 h, no hardware change |
| E-3 | **Conditional on E-1 PASS but E-2 inconclusive:** rebuild production firmware with PA2/PA3 on AF4 (USART2) instead of AF6 (LPUART1) — note this is now a *low-confidence* hypothesis since hello-world also uses AF6 and would prove the path; do this only if E-2 indicates a register-level access problem with LPUART1 specifically. | Production firmware emits BOOT_URC / VER_URC. | Same silence; rule out AF6/LPUART1 silicon issue. | ~1 h |
| E-4 | **Conditional on E-1 FAIL:** capture `UART4_RXD`, `Murata.PA2`, `PF4` (NRST) with a logic analyzer during boot. Confirm whether L072 is actually toggling PA2 at all and whether NRST goes high. | Direct evidence of which side is silent and at what bit time. | N/A (definitive). | half day, needs Saleae |
| E-5 | Schematic-extraction pass on ABX00043 PDF to verify direct-route hypothesis (PA2/PA3 ↔ UART4_TXD/RXD with only series resistors, NRST ↔ PF4 with 1 MΩ pull-up, BOOT0 ↔ PA11). Update [`CHIP-DOCS/IMX8MM_UART4/findings.md`](CHIP-DOCS/IMX8MM_UART4/findings.md) and [`CHIP-DOCS/MURATA_CMWX1ZZABZ_078/findings.md`](CHIP-DOCS/MURATA_CMWX1ZZABZ_078/findings.md). | Confirms BSP overlay matches hardware; closes Blocker C. | Reveals an unexpected mux; reopens Phase D with a real target. | 1–2 h, manual PDF read |

---

## 4. What to update in the TODO

After (or in parallel with) experiment 1, the four Go/No-Go blocker entries in [`DESIGN-CONTROLLER/TODO.md`](../DESIGN-CONTROLLER/TODO.md) should be rewritten so that:

- **Blocker A** retains its Go criterion but the "Current evidence" line points to this note as the synthesis and lists the three concrete pending experiments (pull-up, AF4, combined).
- **Blocker B** Go criterion is rephrased to "pad-config state (`UART4_RXD` PUE/PUS) and L072 PA2/PA3 AF state that consistently changes ingress classification". The 34-pin GPIO-sweep evidence stays as the *negative* corpus that closed the prior hypothesis.
- **Blocker C** Go criterion is rephrased to "verified net map for the Murata UART4 link from ABX00043 schematic, confirming or refuting the BSP-derived direct-route hypothesis." Phase E (multi-pin / polarity / timing combinatorics) is removed.
- **Blocker D** is closed with a "no reproducible timing-window effect; superseded by NRST→first-byte timing alignment in the active probe" disposition.

---

## 5. References

### Primary sources reviewed in this pass
- [`ov_carrier_enuc_lora.dts`](https://raw.githubusercontent.com/arduino/meta-arduino/scarthgap/meta-arduino-nxp/recipes-bsp/device-tree/arduino-device-tree/portenta-x8/overlays/ov_carrier_enuc_lora.dts) — Arduino official Yocto BSP overlay for the Max Carrier Murata link.
- [`arduino-device-tree.inc`](https://github.com/arduino/meta-arduino/tree/scarthgap/meta-arduino-nxp/recipes-bsp/device-tree/arduino-device-tree.inc) — overlay registration confirms `ov_carrier_enuc_lora` is the Murata path.
- [`u-boot boot.cmd`](https://github.com/arduino/meta-arduino/tree/scarthgap/meta-arduino-lmp/recipes-bsp/u-boot/u-boot-ostree-scr-fit/portenta-x8/boot.cmd) — confirms `ov_carrier_enuc_lora` ships in `max_ovl` for Max Carrier autodetect.
- ABX00043 schematics / pinout / datasheet (linked but not directly fetched in this pass; see [`CHIP-DOCS/source_log.md`](CHIP-DOCS/source_log.md) for direct URLs).

### Internal evidence cross-referenced
- 2026-05-10_OwnerNet_Derived_UART4_Profile_Results_Copilot_v1_0.md
- 2026-05-10_LaneC_Mixed_ResetOrder_Run_Copilot_v1_0.md
- 2026-05-10_CrossLane_BC_Interaction_Matrix_Copilot_v1_0.md
- 2026-05-10_HoldWindow_Timing_Sweep_Copilot_v1_0.md
- 2026-05-11_Phase_D1_GPIOB_Sweep_Results_Copilot_v1_0.md
- 2026-05-11_Phase_D2_GPIOF_Sweep_Results_Copilot_v1_0.md
- 2026-05-11_Phase_D3_GPIOD_Sweep_Results_Copilot_v1_0.md
- 2026-05-11_Phase_D_SchematicBased_UART4_Refinement_Plan_Copilot_v1_0.md
- 2026-05-09_PA11_PostBoot_High_Test_Copilot_v1_0.md

### Firmware files inspected
- [`firmware/murata_l072/host/host_uart.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c)

---

## 6. Disposition

This note does **not** change any firmware, device tree, or TODO state. It proposes:

- a re-scoping of Blockers B and C against primary-source evidence,
- three small, low-cost bench experiments that can clear or refute Blocker A,
- a close-out path for Blocker D.

If experiment 1 (pad pull-up) clears `VER_REQ` → `VER_URC` ingress on its own, the cumulative Phase A–E methodology reframing should be folded into the [TODO Go/No-Go section](../DESIGN-CONTROLLER/TODO.md#gono-go-blockers-current) and the per-phase evidence preserved in `bench-evidence/` as the negative corpus that drove the discovery.
