# Controller Stage 1 Early UART Capture Analysis

Date: 2026-05-09
Author: Copilot
Scope: Method G Phase 1 / W1-7 bring-up on the custom `murata_l072` image.
Primary evidence: `DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_133825/`

## Summary

The latest Stage 1 rerun does not support the earlier "complete silence" model anymore.

Flash and boot are still healthy, but immediate post-reset UART capture now shows pathological output instead of valid host protocol traffic:

- Earliest `19200 8N1` capture shows non-text garbage bytes, not readable startup text.
- Early `921600 8N1` capture shows a burst of raw `0x00` bytes only.
- By the time the active probe opens, the line is silent again.
- No valid `BOOT_URC`, `VER_URC`, or ASCII fallback response is observed.

This means the Murata is doing something on the UART immediately after reset, but that activity is not a usable host protocol frame.

## What Changed In This Pass

### Firmware instrumentation

Added short ordered breadcrumbs around the critical bring-up path:

- `RST:START`
- `RST:MAIN`
- `M:ENTER`
- `M:SAFE0` / `M:SAFE1`
- `M:CLK0` / `M:CLK1`
- `M:TICK1`
- `M:UART0` / `M:UART1`
- `M:RAD0` / `M:RAD1` / `M:RADF`
- `M:CMD0` / `M:CMD1`

These were intentionally short to avoid the long startup beacon perturbing timing.

### Harness instrumentation

`run_method_g_stage1.sh` was extended to capture UART immediately after reset rather than waiting 4 seconds before first opening `/dev/ttymxc3`.

Two harness refinements were tested:

1. Immediate early capture at `19200`, then delayed active probe.
2. Split early capture at `19200` and `921600` to narrow the transition window.

## Observed Result

From `T6_bringup_2026-05-09_133825/method_g_stage1.log`:

- Flash succeeds using AN3155 page-batch erase fallback.
- BOOT0/NRST sequence succeeds.
- Earliest `19200` capture shows garbage.
- Early `921600` capture shows only `0x00` bytes.
- Active probe still fails with timeout waiting for `VER_URC` (`0x81`).

## Interpretation

The strongest current interpretation is:

1. Execution likely reaches at least part of the startup-to-host-UART transition.
2. The UART line is active very early after reset.
3. The first high-speed output is malformed or degenerate rather than a valid COBS host frame.
4. The fault is now more likely in the first transmit path after `host_uart_init` than in ROM boot, flash layout, or board routing.

The raw `0x00` burst is especially important because a valid host frame should not degenerate into a sustained zero stream.

## What This Rules Out

This pass materially weakens these hypotheses:

- "The firmware never executes after reset."
- "The UART lane is electrically dead."
- "The only problem is that the probe opens too late and misses a clean boot URC."

The line is active, but the observed bytes are not a valid boot response.

## Highest-Value Next Step

Inspect the first transmit path after `host_uart_init` and before/inside `host_cmd_init` for a zero-byte flood or TX-low condition.

Priority targets:

1. `host_uart_send_urc` and its framing path.
2. Any transmit call that runs before the first request/response cycle.
3. UART register setup that could leave TX driving a low stream or emit delimiters/zeros repeatedly.
4. DMA / IRQ interactions that could be feeding zeroed buffers into the transport.

## Files Touched In This Pass

- `DESIGN-CONTROLLER/firmware/murata_l072/include/platform.h`
- `DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c`
- `DESIGN-CONTROLLER/firmware/murata_l072/startup.c`
- `DESIGN-CONTROLLER/firmware/murata_l072/main.c`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_g_stage1.sh`

## Evidence Folder

- `DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_133825/`
