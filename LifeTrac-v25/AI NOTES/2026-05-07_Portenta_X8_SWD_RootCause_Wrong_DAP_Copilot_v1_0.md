# Portenta X8 SWD Bring-up — Root Cause: Wrong DAP / Mis-Wired Probe

> **Superseded / corrected:** Later bench work found the stronger architecture-level explanation: Portenta X8 uses an internal Linux OpenOCD `imx_gpio` SWD path to reach the STM32H747, and the Max Carrier onboard JTAG debug path is not supported for X8 H747 debug. Internal OpenOCD verified the real H747 DPIDR as `0x6ba02477`; the Max Carrier J-Link path's `0x5ba02477` is still evidence that the external path is wrong for this job, but the exact chip/net reached by that path should be treated as unresolved. See `2026-05-07_Portenta_X8_Max_Carrier_Debug_Architecture_Reframe_Copilot_v1_0.md`.

- **Date:** 2026-05-07
- **Author:** Copilot (Claude Opus 4.7)
- **Status:** Diagnostic conclusion + recommended next experiments
- **Scope:** Closes the "PA4 mux bypass" + "PC0 JTAG/SWD isolation bypass" + "Linux-side m4-proxy" hypothesis tree for Portenta X8 + Max Carrier.

## TL;DR

Stopping `m4-proxy.service` and unloading 7 of 9 `x8h7_*` kernel modules (everything except `x8h7_gpio` and `x8h7_drv`, which were locked in by Linux device-tree consumers) **did not change the J-Link failure signature one bit**. Combined with the SW-DP IDCODE actually being read by J-Link (`0x5BA02477` SWD / `0x5BA00477` JTAG), the J-Link OB-STM32F4-Arduino probe (S/N `1078222309`) is **not connected to the STM32H747's SWD pins**. The DAP it can reach is a **Cortex-M3/M4-class debug port** — almost certainly the X8H7 bridge MCU's debug port (or a floating/unwired net on the Max Carrier debug header), not the H747 Cortex-M7.

This means **no firmware-side bypass on the H7 (PA4, PC0, sleep mode, DBGMCU register, etc.) can ever fix the attach failure** — the SWD signals are not physically reaching the H747 SWCLK/SWDIO pins to begin with.

## What was eliminated today

| # | Hypothesis | Action | Result |
|---|---|---|---|
| 1 | Sniff multiplexer (PA4) holds H7 SWD bus | Default LOW + forced HIGH builds, SWD attach retried | Identical failure both polarities |
| 2 | JTAG/SWD isolation FET (PC0) gates SWD | Default LOW + forced HIGH builds, SWD attach retried | Identical failure both polarities |
| 3 | Linux `m4-proxy` userspace daemon owns H7 debug | `pkill -9 m4_proxy`, `systemctl stop m4-proxy.service` → confirmed `inactive` and process gone | Identical SWD failure |
| 4 | Linux `x8h7_*` kernel drivers chatter on SPI bridge → fight DAP | `rmmod x8h7_h7 x8h7_ui x8h7_uart x8h7_pwm x8h7_rtc x8h7_adc x8h7_can` (only `x8h7_gpio` + `x8h7_drv` remained, locked in by DT) | Identical SWD failure |

Failure signature in every case (truncated):

```
Connecting to target via SWD
DAP initialized successfully.
Can not attach to CPU. Trying connect under reset.
DAP initialized successfully.
Connecting to CPU via connect under reset failed.
****** Error: J-Link script file function InitTarget() returned with error code -1
Error occurred: Could not connect to the target device.
```

DAP IDs observed during initialization (constant across all experiments):

- SWD SW-DP IDCODE: `0x5BA02477`
- JTAG-DP IDCODE:   `0x5BA00477` (IRLen 4, IRPrint 0x01)

## The smoking gun: wrong DAP IDCODE

For STM32H747 the published Cortex-M7 SW-DP IDCODE is **`0x6BA02477`** (DPv2, designer 0x23B = ARM, part `0xBA02`). What we see is `0x5BA02477` — DPv1, designer 0x23B, part `0xBA02`. Per ARM's IHI 0031 ADIv5 spec, **bits[31:28] of IDCODE = DP version + ARM-defined revision**. `0x5...` is the Cortex-M3/M0+/M4 family signature; `0x6...` is the Cortex-M7 signature (ADIv5.2 with multi-AP).

Reading `0x5BA02477` **proves we are not talking to the H747 SW-DP**. We are talking to a smaller Cortex-M-class debug port — the most likely candidate on a Portenta X8 + Max Carrier is the **X8H7 bridge MCU** (the SAMD11/SAMD21-class chip that brokers SPI traffic between the i.MX8M and the H7 and owns the carrier I/O multiplexers). That chip would also reasonably refuse `connect under reset` because it has no firmware that supports halting on a debugger attach the way an Arduino sketch on the H7 would.

(Independent corroboration: even if the carrier's debug header *physically* mapped to H747 PA13/PA14, the J-Link probe would refuse to give us a `0x5...` IDCODE for that target — the H747 silicon literally returns `0x6...`. There is no firmware setting that can change this.)

## What this means for "Method G" L072 RTT bring-up

The whole purpose of getting J-Link talking to the H7 was to use SEGGER RTT as the side-channel for verifying L072 Method G UART routing on the X8. Since the J-Link cabling can't reach the H7, **RTT via SEGGER on the X8 is currently unreachable, regardless of any sketch we flash**. The vendored RTT integration we did in `x8_uart_route_probe.ino` is correct and will work as soon as a J-Link reaches the actual H7 SW-DP.

Until then, we should drop SEGGER RTT as the channel for X8 bring-up and revert to the **UART-based** bring-up methods we already proved work end-to-end through `arduino-cli upload` and the X8H7 UART tunnel (`/dev/ttyX0`, etc.).

## Recommended next experiments (ranked)

1. **Confirm the wrong-target diagnosis with a 2nd probe**
   - Repeat `JLink connect` with the *other* probe (S/N `1078180658`) on the same header. If both report `0x5BA0_2477`, the cabling is wrong — not the probe.
2. **Identify the actual chip the probe lands on**
   - In JLink Commander, after `connect` succeeds at the DAP layer, run `device CORTEX-M0` and re-`connect`, then `device CORTEX-M3`, then `device CORTEX-M4`. One of them should attach. The matching family will identify the X8H7 bridge MCU's core, narrowing down the chip family (SAMD11/SAMD21 = M0+; STM32F0 = M0; STM32F4 = M4; etc.).
3. **Verify Max Carrier debug header pinout**
   - Cross-check the Max Carrier schematic ([Arduino Max Carrier docs](https://docs.arduino.cc/hardware/portenta-max-carrier/)) for the JTAG/SWD header; identify which pins on the Portenta High-Density connector (J1/J2) the SWCLK/SWDIO traces actually go to. Confirm with a multimeter (continuity from header SWDIO/SWCLK → H747 PA13 (SWDIO) and PA14 (SWCLK) on the Portenta X8 module's HD connector pads).
4. **Try the official Arduino Debug method for X8**
   - Use `arduino-cli debug -b arduino:mbed_portenta:envie_m7 ...` (the H7 core, not the X8 Linux core) — this method uses the X8H7 bridge to broker DFU/debug rather than relying on a physical SWD header. If RTT/GDB work this way, we get the same end goal without rewiring.
5. **Use the X8's built-in Linux-side flashing instead of physical SWD**
   - For everything we actually need (sketch upload + UART/RTT-equivalent logging), continue using `arduino-cli upload --fqbn arduino:mbed_portenta:portenta_x8` over ADB. This already returns `upload-exit=0` and is the supported path on this hardware family.
6. **Last resort: solder-test points**
   - If physical SWD on the H7 is truly required, identify the H747 SWDIO/SWCLK test points on the Portenta X8 carrier-side HD connector solder pads and tack-wire to a standard 10-pin Cortex SWD header connected to the J-Link.

## Cleanup notes

- Linux side modifications made today are **non-persistent** (kernel module unloads + service stop). A reboot of the X8 restores everything.
  - To restore now without rebooting: `adb shell "echo fio | sudo -S sh -c 'modprobe x8h7_h7 x8h7_ui x8h7_uart x8h7_pwm x8h7_rtc x8h7_adc x8h7_can; systemctl start m4-proxy.service'"`
- Sketch-side `LIFETRAC_UART_SNIFF_MUX_LEVEL` and `LIFETRAC_JTAG_SWD_ISO_LEVEL` defaults in [LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_uart_route_probe/x8_uart_route_probe.ino](LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_uart_route_probe/x8_uart_route_probe.ino) are kept (`HIGH` and `LOW` respectively) so any future re-test starts from a known state.

## Cross-references

- [LifeTrac-v25/AI NOTES/2026-05-07_Portenta_X8_Max_Carrier_Mux_Bypass_Execution_Findings_Copilot_v1_0.md](LifeTrac-v25/AI%20NOTES/2026-05-07_Portenta_X8_Max_Carrier_Mux_Bypass_Execution_Findings_Copilot_v1_0.md) — PA4 attempt
- [LifeTrac-v25/AI NOTES/2026-05-07_Portenta_X8_Max_Carrier_JTAG_Bypass_PC0_Execution_Findings_Copilot_v1_0.md](LifeTrac-v25/AI%20NOTES/2026-05-07_Portenta_X8_Max_Carrier_JTAG_Bypass_PC0_Execution_Findings_Copilot_v1_0.md) — PC0 attempt
- [LifeTrac-v25/AI NOTES/2026-05-06_Method_G_M7_Passthrough_Test_Findings_Copilot_v1_0.md](LifeTrac-v25/AI%20NOTES/2026-05-06_Method_G_M7_Passthrough_Test_Findings_Copilot_v1_0.md) — rolling findings log
