# X8 + Max Carrier — Health Checks & Recovery

Operational knowledge base for diagnosing and recovering Portenta X8 + Portenta Max
Carrier boards on the LifeTrac bench. Tracks **what works, what doesn't, and why**,
so we don't re-discover the same dead ends every session.

## Folder layout

| Folder | Purpose |
|---|---|
| [routines/](routines/) | Health-check procedures (one `.md` per routine) with the exact commands, expected output, pass/fail criteria, and a pointer to the on-disk helper script. |
| [recovery/](recovery/) | Recovery methods, ordered by escalation tier. Each file documents the procedure, when to use it, and a verdict matrix of past attempts. |
| [log/](log/) | Per-session bench logs (`YYYY-MM-DD_<board>_<routine>_<vN>.md`) — short structured notes pointing at the raw evidence under `LifeTrac-v25/AI NOTES/`. |

Helper scripts themselves continue to live next to the firmware that uses them
(`LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/`); routine
docs in this folder reference them by path rather than duplicating.

## Boards under test

| Tag | ADB serial | Role | Notes |
|---|---|---|---|
| Board 1 | `2D0A1209DABC240B` | Primary / TX in pair tests | Currently soft-hung post-W2-01 camera session (2026-05-12). |
| Board 2 | `2E2C1209DABC240B` | Control / RX in pair tests | Healthy, 3+ day uptime as of 2026-05-13. |

## Routine index

| ID | Name | File | Last verified |
|---|---|---|---|
| HC-01 | adb + Windows USB enumeration | [routines/HC-01_adb_and_usb_enumeration.md](routines/HC-01_adb_and_usb_enumeration.md) | 2026-05-13 |
| HC-02 | Linux bridge + GPIO + UART health (full) | [routines/HC-02_linux_bridge_full.md](routines/HC-02_linux_bridge_full.md) | 2026-05-13 |
| HC-03 | H7 SWD attach diff (`imx_gpio` openocd) | [routines/HC-03_h7_swd_attach_diff.md](routines/HC-03_h7_swd_attach_diff.md) | 2026-05-12 |
| HC-04 | Stage 1 standard quant (canonical pass) | [routines/HC-04_stage1_standard_quant.md](routines/HC-04_stage1_standard_quant.md) | 2026-05-12 |

## Recovery method index (by escalation tier)

| Tier | Method | File | Verdict |
|---|---|---|---|
| 0 | Host adb daemon kick (`adb kill-server`) | [recovery/T0_adb_daemon_kick.md](recovery/T0_adb_daemon_kick.md) | Useful for stale host state only. |
| 1 | Linux soft-recovery (rmmod cascade + svc restart) | [recovery/T1_bridge_rmmod_cascade.md](recovery/T1_bridge_rmmod_cascade.md) | Triggers watchdog auto-reboot; restores bridge but does **not** clear post-camera IOMUXC wedge. |
| 2 | Cold power cycle (USB-C + 12V unplug ≥10 s) | [recovery/T2_cold_power_cycle.md](recovery/T2_cold_power_cycle.md) | Required to reset i.MX IOMUXC; sometimes insufficient if the wedge is in eMMC boot. |
| 2.5 | Carrier-power sanity check | [recovery/T2_5_carrier_power_sanity.md](recovery/T2_5_carrier_power_sanity.md) | Pre-flight before Tier 3 (J-Link BMP `1366:0105` must enumerate). |
| 3a | SDP / `uuu` full image reflash | [recovery/T3a_sdp_uuu_reflash.md](recovery/T3a_sdp_uuu_reflash.md) | Definitive fix; mask-ROM independent of eMMC. |
| 3b | SDP RAM-only initramfs rescue | [recovery/T3b_sdp_ram_rescue.md](recovery/T3b_sdp_ram_rescue.md) | Diagnostic: capture journals from broken eMMC. |

## Soft-reset menu (no physical contact)

For a complete enumeration of every reset / restart that can be issued from
the keyboard — across host, X8 Linux, STM32H747, Murata L072, and i.MX SoC
layers — including which Tier already uses each one and which gaps still need
documenting, see:

- [recovery/SOFT_RESET_INDEX.md](recovery/SOFT_RESET_INDEX.md)

## Conventions

- **Pass/fail criteria** must be machine-checkable (string match, exit code, range).
- **Verdict matrix** entries are dated and tag the bench session, not the calendar
  day, so retroactive corrections are visible.
- Don't delete failed-attempt rows — mark them `FAILED (reason)` so future sessions
  know not to retry the same dead end.
