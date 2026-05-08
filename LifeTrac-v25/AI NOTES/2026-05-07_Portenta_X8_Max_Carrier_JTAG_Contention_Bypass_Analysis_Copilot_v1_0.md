# Portenta X8 Max Carrier JTAG Contention Bypass Plan - Copilot Analysis v1.0

- **Date:** 2026-05-07
- **Author:** Copilot
- **Scope:** Review of `2026-05-07_Portenta_X8_Max_Carrier_JTAG_Contention_Bypass_v1.0.md`
- **Verdict:** Do not execute as written. The plan's central contention theory is not supported by current bench evidence.

## Summary

The plan correctly notices one important fact: Portenta X8 Linux can reach the STM32H747 over an internal OpenOCD `imx_gpio` SWD path. However, it draws the wrong operational conclusion.

The evidence now points to an architecture mismatch, not a two-part "PC0 gate plus Linux contention" jailbreak. The supported X8 path is:

```text
Windows / Arduino CLI -> ADB -> X8 Linux -> OpenOCD imx_gpio -> STM32H747 SWD
```

The Max Carrier onboard J-Link/debug path is documented as unsupported for Portenta X8 H747 debug, and previous external J-Link attempts did not start returning the H747 DPIDR after PA4/PC0 or Linux-service changes.

## Claim-by-claim review

### Claim 1: PC0 opens the Max Carrier H747 JTAG/SWD gate

Status: **Not proven; already tested as insufficient.**

The route-probe firmware already drove `PC_0` in both polarities while keeping `PA_4` deterministic:

- `PC0=LOW`: external J-Link still failed with `InitTarget -1`.
- `PC0=HIGH`: external J-Link still failed with the same signature.

This does not prove no carrier-side gate exists anywhere, but it does disprove the plan's practical Step 1 as a complete unlock.

### Claim 2: `monitor-m4-elf-file.service` is continuously clamping SWD

Status: **False as written.**

Live check after the latest upload:

```text
pgrep -af openocd -> no openocd process, except the shell command itself
monitor-m4-elf-file.service -> inactive
m4-proxy.service -> active
```

`monitor-m4-elf-file.service` is a one-shot programmer triggered by `monitor-m4-elf-file.path` when `/tmp/arduino/m4-user-sketch.elf` changes. It runs OpenOCD, programs/verifies/resets, then exits. It is not a continuously running SWD master.

Stopping only the service also does not disable the path trigger. If a future upload changes the ELF again, the path unit can start the service again.

### Claim 3: external J-Link reads `0x5ba02477` because Linux corrupts the real H747 ID

Status: **Unlikely.**

The X8 internal OpenOCD path reads the expected H747 value:

```text
SWD DPIDR 0x6ba02477
```

The external Max Carrier J-Link path repeatedly reads:

```text
0x5ba02477 / 0x5ba00477
```

That is not random noise. It is a stable, valid-looking ARM DP ID from a different debug-port class/revision. A real bus fight would more likely produce unstable ACK/parity/read failures, not a repeatable coherent alternate DPIDR across tests.

The better interpretation remains: the external Max Carrier debug path is not electrically reaching the same H747 SWD path that X8 Linux OpenOCD uses.

### Claim 4: stopping `m4-proxy.service` releases SWD

Status: **Not supported.**

`m4-proxy.service` is relevant to X8/M4 RPC and Arduino infrastructure. Earlier tests stopped/killed it and unloaded most `x8h7_*` modules; the external J-Link failure did not change.

Stopping `m4-proxy` may disrupt runtime services, but it is not a demonstrated SWD release mechanism.

## Important nuance

The plan's contention idea is not impossible at the electrical-theory level. If the i.MX GPIO SWD pins and external J-Link were tied to the same SWD nets, residual GPIO direction/state after OpenOCD exits could matter. But the proposed procedure does not actually prove or fix that, because:

- It does not verify whether an OpenOCD process is running.
- It does not stop `monitor-m4-elf-file.path`, only the service.
- It does not actively set i.MX SWCLK/SWDIO/SRST GPIOs to high-impedance/input.
- It does not explain why the external probe sees a stable `0x5ba...` while internal OpenOCD sees `0x6ba...`.
- It conflicts with Arduino's documented "on board JTAG debugging: No" for X8 on Max Carrier.

## If we want to test the contention hypothesis anyway

A safer discriminating test would be:

1. Upload the PC0-controlled sketch through the normal X8 ADB route.
2. Stop the path trigger and one-shot service:

```powershell
adb shell "echo fio | sudo -S systemctl stop monitor-m4-elf-file.path monitor-m4-elf-file.service"
```

3. Confirm no internal OpenOCD is running:

```powershell
adb shell "pgrep -af openocd || true"
```

4. Attempt external J-Link attach and record the DPIDR.
5. If it still returns `0x5ba02477`, the contention hypothesis is effectively falsified for the external Max Carrier J-Link path.

Even this test is lower value than improving the known-good internal OpenOCD path, because current evidence already shows internal OpenOCD can reach the real H747 while external J-Link cannot.

## Corrected path forward

Use the X8 as its own debug probe:

```powershell
adb forward tcp:3333 tcp:3333
adb shell "echo fio | sudo -S openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg"
```

For M4 sketch debug, continue work on a custom dual-core attach script:

```powershell
adb forward tcp:3334 tcp:3334
adb shell "echo fio | sudo -S openocd -c 'set DUAL_CORE 1' -f /usr/arduino/extra/openocd_script-imx_gpio.cfg"
```

For Method G route discovery, prioritize physical UART telemetry instead of external J-Link RTT:

- Max Carrier UART sniff channels if their firmware/mux path is proven.
- External USB-UART or logic analyzer on the candidate H747-to-Murata UART lines.
- Direct Murata L072 bootloader/SWD paths for radio firmware work.

## Bottom line

The plan is a useful thought experiment, but it is not a reliable fix. The failure is not primarily "Linux is holding the bus while the J-Link waits." The stronger finding is: **external Max Carrier J-Link is the wrong/unsupported debug path for Portenta X8 H747; internal X8 Linux OpenOCD is the path that actually reaches the chip.**
