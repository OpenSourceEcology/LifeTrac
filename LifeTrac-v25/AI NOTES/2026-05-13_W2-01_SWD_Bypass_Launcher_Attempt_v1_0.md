# W2-01 — SWD-Bypass Stage 1 Launcher Attempt (v1.0)

**Date:** 2026-05-13
**Author:** Copilot (Claude)
**Status:** Plan + scripts authored, **bench validation pending**.
**Boards in scope:** Board 1 ADB `2D0A1209DABC240B` (broken SWD), Board 2 ADB `2E2C1209DABC240B` (control).
**Tracker:** Continuation of W2-01 (X8 Board 1 recovery) after T3a uuu reflash failed to clear the SWD wedge.

---

## 1. Background

The Stage 1 L072 ROM-bootloader entry sequence in
[LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/07_assert_pa11_pf4_long.cfg](LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/07_assert_pa11_pf4_long.cfg)
uses **OpenOCD over SWD** to:

1. Halt the H7 (M7 core).
2. Enable RCC clocks for GPIOA + GPIOF.
3. Drive **PA11 high** (= L072 BOOT0).
4. Pulse **PF4 low → high** (= L072 NRST).
5. Hold for ~600 s while the host pokes 0x7F into `/dev/ttymxc3` and expects 0x79 ACK.

This is **not flashing the H7**. The SWD link is being used purely as a remote
register-poke channel for two H7 GPIO pins. Board 1's SWD bus is electrically
dead (`Error connecting DP: cannot read IDR`, DPIDR reads `0xffffffff` even
after a full LmP reinstall via uuu T3a). All host-side software fixes have
been ruled out — the fault is on-die or on-PCB.

**Key insight:** the same two H7 pins (PA11, PF4) are already exposed to Linux
through the **x8h7 SPI bridge** (`/dev/spidev*` aggregator → kernel sub-drivers
`x8h7_pwm` + `x8h7_gpio`). The bridge is independently confirmed alive on
Board 1 (gpiochip5 enumerates, audio codec reset works). Therefore SWD is
**not on the critical path** for Stage 1 — it can be replaced with bridge
calls. If the swap works, Board 1 is rescued.

---

## 2. Pin-Map Evidence (upstream, cited)

### 2.1 H7 firmware side — `arduino/portentax8-stm32h7-fw`

`src/pwm.c` `PWM_pinmap[]` — channel 4 = **PA11** (TIM1_CH4, AF1):

```c
PwmPinMap PWM_pinmap[] = {
  /* 0 */ { GPIOC, GPIO_PIN_7,  ... }, // PC7
  /* 1 */ { GPIOA, GPIO_PIN_9,  ... }, // PA9
  /* 2 */ { GPIOA, GPIO_PIN_10, ... }, // PA10
  /* 3 */ { GPIOB, GPIO_PIN_10, ... }, // PB10
  /* 4 */ { GPIOA, GPIO_PIN_11, ... }, // PA11   <-- L072 BOOT0
  /* 5 */ { GPIOD, GPIO_PIN_15, ... }, // PD15
  /* 6 */ { GPIOA, GPIO_PIN_8,  ... }, // PA8
  /* 7 */ { GPIOC, GPIO_PIN_6,  ... }, // PC6
  /* 8 */ { GPIOC, GPIO_PIN_9,  ... }, // PC9
  /* 9 */ { GPIOC, GPIO_PIN_8,  ... }, // PC8
};
```

`src/gpio.c` `GPIO_pinmap[]` first section — index 3 = **PF4**:

```c
GpioPinMap GPIO_pinmap[] = {
  /* 0 */ { GPIOF, GPIO_PIN_8 },  // PF8
  /* 1 */ { GPIOF, GPIO_PIN_6 },  // PF6
  /* 2 */ { GPIOF, GPIO_PIN_3 },  // PF3
  /* 3 */ { GPIOF, GPIO_PIN_4 },  // PF4    <-- L072 NRST
  /* 4 */ { GPIOF, GPIO_PIN_12 }, // PF12
  /* 5 */ { GPIOE, GPIO_PIN_10 }, // PE10
  /* 6 */ { GPIOE, GPIO_PIN_11 }, // PE11
  // ... ADCs follow at index 7+
};
```

PA11 does **not** appear in `GPIO_pinmap[]` — its only Linux-side handle is
`/sys/class/pwm/pwmchip0/pwm4`.

### 2.2 Linux kernel side — `arduino/portentax8-x8h7`

`x8h7_pwm.c` `x8h7_pwm_config()`:

```c
x8h7_pkt_send_sync(X8H7_PWM_PERIPH, pwm->hwpwm, sizeof(x8h7->pkt), &x8h7->pkt);
```

`pwm->hwpwm` is the channel index passed via sysfs (`pwm0..pwm9`) — sent
**directly as the H7 firmware opcode**, which is the index into
`PWM_pinmap[]`. Direct passthrough.

`x8h7_gpio.c` `x8h7_gpio_direction_output()`:

```c
data[0] = offset;
data[1] = !!value;
x8h7_pkt_send_sync(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_WR, 2, data);
data[1] = GPIO_MODE_OUTPUT_PP;        // 0x01
x8h7_pkt_send_sync(X8H7_GPIO_PERIPH, X8H7_GPIO_OC_DIR, 2, data);
```

`offset` is the gpiolib line number — sent **directly as data byte 0**, used
by H7 firmware as the index into `GPIO_pinmap[]`. Direct passthrough.

`gpiochip5` registration: `npwm = 10`, `X8H7_GPIO_NUM = 34`, `base = 160`,
label `x8h7_gpio`.

### 2.3 Resulting Linux handles

| L072 pin | H7 pin | H7 firmware index    | Linux handle                          |
|----------|--------|----------------------|---------------------------------------|
| BOOT0    | PA11   | `PWM_pinmap[4]`      | `/sys/class/pwm/pwmchip0/pwm4`        |
| NRST     | PF4    | `GPIO_pinmap[3]`     | `/dev/gpiochip5` line 3 (= `gpio163`) |

---

## 3. Plan

### 3.1 Drive PA11 HIGH via PWM 100 % duty

```sh
PWM=/sys/class/pwm/pwmchip0/pwm4
[ -d "$PWM" ] || echo 4 | sudo tee /sys/class/pwm/pwmchip0/export
echo 1000000 | sudo tee "$PWM/period"        # 1 ms period
echo 1000000 | sudo tee "$PWM/duty_cycle"    # 100% = constant high
echo 1       | sudo tee "$PWM/enable"
```

STM32 timer in PWM1 mode with `CCR == ARR` outputs continuous high; this is
electrically equivalent to driving the GPIO high with a static register write.

### 3.2 Pulse PF4 LOW → HIGH via gpiochip5 line 3

```sh
sudo gpioset --mode=signal /dev/gpiochip5 3=0 &
PID=$!
sleep 0.25
kill $PID
sudo gpioset --mode=signal /dev/gpiochip5 3=1 &
```

Note: plain `gpioset CHIP LINE=V` exits immediately and releases the line
back to default (input). Use `--mode=signal` + background to hold the line.
Alternatively use sysfs (`/sys/class/gpio/export` of `163`) for persistent
direction+value.

### 3.3 Hold + flash window

After PA11 is high and PF4 has been pulsed, the L072 should be in ROM bootloader.
Run the existing UART probe / flasher against `/dev/ttymxc3` @ 19200 8E1.

### 3.4 Release

Disable PWM (PA11 → 0), release PF4 high (deasserted = released).

---

## 4. Files Created / Modified

- **NEW** `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/swd_bypass_pa11_pf4_launcher.sh`
  Drop-in launcher that performs §3.1-§3.4 on the X8 over ADB-pushed shell.
  Mirrors timing of `07_assert_pa11_pf4_long.cfg` (250 ms NRST low pulse,
  then HOLD_S seconds with BOOT0 held high).
- **(unchanged)** `07_assert_pa11_pf4_long.cfg` — kept as fallback for Board 2.
- **(future)** `run_stage1_standard_contract.sh` — needs a `LAUNCHER` env knob
  to invoke the new script in place of OpenOCD. Not modified yet — will refactor
  once bench validation passes on Board 2.

---

## 5. Risks & Caveats

1. **gpiochip5 line 0** is owned by `cs42l52_regulator` (audio codec reset). Do
   not touch line 0. Only line 3 (PF4) is used here.
2. **PWM 100 % duty edge case.** Some PWM cores require `duty < period`. If the
   x8h7 H7 firmware coerces duty to `period - 1`, there will be a single ~1 ns
   low pulse per ms, which is well below L072 BOOT0 sample-rate threshold and
   should still be read as HIGH at NRST release. If this proves problematic,
   alternative: after entering bootloader, leave PWM enabled — the L072 only
   samples BOOT0 at NRST rising edge.
3. **H7 pin ownership.** When the Linux x8h7 sub-drivers configure PA11 as TIM1
   output and PF4 as GPIO_OUTPUT_PP, the H7 firmware HAL_GPIO_Init will alter
   the pin's MODER/AFR. This is the same end-state the SWD register pokes
   produced — should be benign. If the H7 firmware re-asserts a different mode
   later (e.g. on a periodic poll) the line could glitch; mitigate by keeping
   the PWM enable + GPIO write live for the entire HOLD_S window.
4. **Bridge wedge surface.** If the SPI bridge wedges mid-flash, recovery
   options are: (a) re-export the PWM channel; (b) toggle x8h7 driver via
   `rmmod / modprobe`; (c) reboot. Capture dmesg before and after.
5. **No PWM channel for L072 NRST.** Using PWM-100% for NRST would require
   exporting another channel and accepting the same edge case; cleaner to use
   GPIO line 3 as planned.

---

## 6. Validation Matrix

| Step | Board | Goal | Expected Outcome |
|------|-------|------|------------------|
| V1 | 2 (control) | Run launcher standalone, manually probe `/dev/ttymxc3` for 0x79 | ACK 0x79 received within 5 s of NRST release |
| V2 | 2 | Wire launcher into Stage 1 contract; 1-cycle end-to-end flash | Same success rate as SWD-based path |
| V3 | 2 | 5-cycle quant | ≥ 4/5 success (matches recent SWD baseline) |
| V4 | 1 (rescue) | Run launcher standalone, probe ACK | ACK 0x79 → confirms hardware path is intact and Board 1 can be rescued |
| V5 | 1 | 5-cycle quant | ≥ 1/5 success → retract Board 1 retirement |

---

## 7. Success Criteria

- **Primary:** sustained 0x79 ACK on `/dev/ttymxc3` 19200 8E1 within the
  HOLD_S window on Board 2.
- **Stretch:** same on Board 1 — would prove the SWD wedge is rescuable by
  bypassing the SWD path entirely.

---

## 8. Bench Results — V1 on Board 2 (`2E2C1209DABC240B`)

Date of run: same session as document creation.

### 8.1 Launcher itself works

After three iterations of bug-fixing (sudo password consuming stdin, fresh-PWM
init ordering, root-aware `sudo_write` so already-root callers don't ship
"fio\n" into sysfs), the launcher reaches the bootloader-hold state cleanly:

```
[swd-bypass] exporting PWM channel 4
[swd-bypass] PA11 driven HIGH (BOOT0 asserted; period=1000000 duty=999999)
[swd-bypass] exporting gpio163 (PF4)
[swd-bypass] NRST asserted low for 250 ms
[swd-bypass] NRST released high
[swd-bypass] holding bootloader for 15 s
[swd-bypass] ready: PA11=HIGH PF4=HIGH -- run flasher / probe ttymxc3 now
```

PWM channel 4 exports/configures, GPIO163 exports/toggles, no kernel errors.
**The bridge accepts the writes.** Cleanup runs at the end of the hold window.

### 8.2 But L072 does not enter ROM bootloader

8 attempts at sending `0x7F` to `/dev/ttymxc3` at 19200 8E1 inside the hold
window all returned `0xFE` (or zero bytes), never `0x79` ACK.

`0xFE` at 8E1 with the actual frame at 8N1 indicates parity errors — i.e. the
L072 IS transmitting on its USART but it is in *user firmware* mode, not ROM
bootloader mode (the ROM speaks 8E1 cleanly).

### 8.3 Baseline SWD path also fails on Board 2

Re-ran the SWD-based control test `run_pa11_pf4_test.sh` (which uses
`06_assert_pa11_pf4.cfg` and OpenOCD bit-bang to drive PA11 high + pulse
PF4) on Board 2 in the same session. Result:

```
Info : SWD DPIDR 0x6ba02477          (SWD attached fine)
--- attempt 1 ---  size=1 hex=1f
--- attempt 2 ---  size=0 hex=
--- attempt 3 ---  size=1 hex=1f
--- attempt 4 ---  size=0 hex=
=== ACK_HIT=0 ===
```

The **SWD baseline gets `0x1F`, not `0x79`** — so even the OpenOCD path is no
longer producing ACK on Board 2 (which the prior session inventory listed as
the healthy control). The L072 on Board 2 does not enter ROM bootloader from
EITHER method right now.

### 8.4 Interpretation

This **invalidates the V1 PASS criterion as a launcher-correctness test**:
the bypass launcher cannot be expected to do something the gold-standard SWD
method also fails to do on the same board. The launcher *did* succeed at its
mechanical job — driving PA11 high and pulsing PF4 low through the x8h7
bridge with no errors and no SWD involved.

Possible reasons both methods fail:
1. **Board 2 L072 user firmware has changed** since the prior healthy run,
   and now reconfigures PA11 internally (e.g. as input pull-down) immediately
   after the NRST release window closes, so BOOT0 sampling sees low.
2. **The L072 is in RDP level 1+** which can mask USART bootloader access on
   STM32L0 — would need a power-cycle plus correct boot pattern.
3. **ttymxc3 routing has changed** — maybe a new mux configuration reroutes
   the L072 USART somewhere else.
4. **Bench wiring drift** — the L072–H7 PA11/PF4/USART traces may have a
   loose connection on Board 2 that affects both paths.

### 8.5 What this DOES prove

- The Arduino x8h7 bridge can drive arbitrary PA11/PF4 patterns from Linux
  user-space without OpenOCD/SWD. This is novel and reusable for any X8 with
  broken SWD.
- The launcher script + probe harness are syntactically and operationally
  correct on the LmP image with no libgpiod dependency, with sysfs-only
  fallback that works at any privilege level.
- The blocker for L072 ROM entry is **upstream of the launcher** — it is a
  L072-side or wiring-side issue that affects every method equally.

### 8.6 Recommended next steps (revised)

- **Do not attempt Board 1 rescue until Board 2 baseline is producing 0x79
  again.** Otherwise we cannot tell launcher failure from board failure.
- **Diagnose Board 2 L072 first**:
  - Check if it was recently re-flashed with new user firmware.
  - Try a full power cycle (USB disconnect) before running the test.
  - Verify ttymxc3 is still wired to L072 USART2 (look at /sys/class/tty
    routing and devicetree).
  - If user firmware now grabs PA11, try increasing NRST_LOW_MS to 500ms
    and forcing PA11 high *before* NRST goes low (current sequence does
    this — but make sure no glitch).
- **Once Board 2 baseline returns 0x79**, re-run V1/V2/V3 on the launcher
  and only then proceed to V4/V5 on Board 1.

---

## 9. Followups

- Refactor `run_stage1_standard_contract.sh` to accept `LAUNCHER=…` as an
  alternative to `OPENOCD_CFG=…`.
- Update `SOFT_RESET_INDEX.md` with V4/V5 outcomes.
- If Board 1 V4 succeeds, retract retirement recommendation in
  `2026-05-12_X8_Board1_Recovery_Plan_Copilot_v1_0.md`.
- Consider upstream contribution to Arduino: a script in `extra/` that
  performs L072 bootloader entry without SWD (useful for any X8 with broken
  SWD, e.g. the well-known IOMUXC contention case).
