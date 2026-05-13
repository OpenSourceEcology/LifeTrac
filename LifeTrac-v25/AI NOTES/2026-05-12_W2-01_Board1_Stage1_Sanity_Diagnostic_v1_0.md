# Board 1 (`2D0A1209DABC240B`) Stage 1 Sanity Failure — Diagnostic Summary v1.0

Date: 2026-05-12 evening (post W2-01 BLS restore)
Target: confirm Board 1 health via known-success Stage 1 quant after USB-wedge research session.

## Headline

**Board 1: Stage 1 FAIL_SYNC × 10 cycles** (5+5 across two runs), `Error connecting DP: cannot read IDR` at openocd attach on every cycle.
**Board 2 (control): Stage 1 PASS × 2/2 cycles**, gate PASS.

Toolchain is healthy. Failure is Board-1-specific.

## Evidence Files

| File | Result |
| --- | --- |
| `2026-05-12_W2-01_Board1_Sanity_Stage1_v1_0.log` | Board 1, 5 cycles, all FAIL_SYNC |
| `2026-05-12_W2-01_Board1_Sanity_Stage1_v2_0.log` | Board 1, 5 cycles after manual gpio preflight, all FAIL_SYNC |
| `2026-05-12_W2-01_Board2_Sanity_Stage1_v1_0.log` | Board 2, 2 cycles, both PASS, gate PASS |
| `2026-05-12_W2-01_Board1_Gpio_Probe_v1_0.log` | Board 1 initial state (no gpios exported) |
| `2026-05-12_W2-01_Board1_Manual_Gpio_Preflight_v1_0.log` | After manual preflight: gpio10=out val=1 |
| `2026-05-12_W2-01_Board1_Bridge_State_v1_0.log` | Board 1 bridge fully healthy |
| `bench-evidence/T6_stage1_standard_quant_2026-05-12_194601_654-36996/` | Run id (Board 1 cycles 1–5 v2) |
| `bench-evidence/T6_stage1_standard_quant_2026-05-12_194918_432-26052/` | Run id (Board 2 cycles 1–2) |

## Root Cause Triage Status

| Hypothesis | Status | Evidence |
| --- | --- | --- |
| gpio10 NRST not HIGH (per repo memory line 132) | **RULED OUT** | Verified `gpio10 EXPORTED dir=out val=1` immediately before each launcher invocation; manual preflight script confirmed working state |
| Toolchain / launcher / openocd cfg regression | **RULED OUT** | Identical launcher passes 2/2 on Board 2 |
| Bridge wedge / x8h7 stuck | **RULED OUT** | x8h7_drv + 8 sub-modules loaded, `/sys/kernel/x8h7_firmware/version` returns string in <1s, `m4_proxy` running, `cs42l52` codec bound, `[irq/44-x8h7]` + `[x8h7_gpio_irq_w]` + `[x8h7_uart_work]` kthreads alive, Board 1 uptime 8 min |
| H7 SWD itself unreachable from i.MX gpio8/15 bit-bang | **CURRENT WORKING HYPOTHESIS** | OpenOCD `imx_gpio` driver opens at 1805 kHz then immediately fails `cannot read IDR`; H7 SWD is the only thing this signature can mean (per memory line 35, imx_gpio bit-bang on gpio8/15 talks to H7 SWD, not L072 SWD) |
| L072 firmware actively driving PA13/PA14 | **POSSIBLE** | Doesn't apply: imx_gpio talks to H7, not L072 |
| Subtle gpio8/gpio15 state difference vs Board 2 | **POSSIBLE CONTRIBUTOR** | Board 1: `gpio8 dir=in val=1`, `gpio15 dir=in val=1`. Board 2: `gpio8 dir=in val=0`, `gpio15 dir=out val=0`. Both pins on Board 1 are weakly held HIGH from somewhere (likely H7-side pull-ups on SWD pins), whereas on Board 2 they are LOW because openocd most recently drove them. Should not prevent imx_gpio from re-driving them, but is a notable delta. |
| H7 in non-attachable state (stale bootloader, RDP, wedged after recent uuu/BLS work) | **PLAUSIBLE — REQUIRES PHYSICAL POWER CYCLE TO TEST** | Cannot be checked from software; H7 is downstream of everything the bridge exposes |

## Recommended Next Step (User Action Required)

1. **Full physical power cycle of Board 1**: unplug X8 USB-C AND any 12V barrel attached to Max Carrier, wait ≥10 s, replug. Wait ~60 s for boot.
2. Re-run `run_stage1_standard_quant_end_to_end.ps1 -AdbSerial 2D0A1209DABC240B -Cycles 1` (single-cycle smoke).
3. If it PASSES → Board 1 is fine; the H7 was simply in a wedged post-uuu/post-BLS state that survived `adb reboot` but not a real power cycle.
4. If it FAILS again → escalate: this would be the first hard-Board-1-only Stage 1 regression and warrants checking with a meter that gpio8/gpio15 (i.MX8 SoM B2B side) physically toggle, plus checking H7 BOOT0 / NRST pin states.

## Already-Confirmed Good (Do Not Re-Investigate)

- BLS file `/boot/loader/entries/ostree-1-lmp.conf` restored from `.bak.w2_01` backup — `options` line clean.
- `/proc/cmdline` does NOT contain `usbcore.autosuspend=-1` — Option A applier was a no-op (LmP rebuilds cmdline from kernel/initrd/ostree/root subset only).
- §6f appendix added to `2026-05-12_W2-01_USB_Wedge_Avoidance_Research_Copilot_v1_0.md` documenting both deprecated helpers and three remaining Option A paths.
- Board 2 (`2E2C1209DABC240B`) is healthy and usable for any non-Board-1-specific bench work.
