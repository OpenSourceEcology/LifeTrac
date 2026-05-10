# Controller Stage 1 RTS/CTS A/B Checklist (Copilot v1.0)

Date: 2026-05-09
Target board: 2E2C1209DABC240B
Goal: Determine whether host UART hardware flow control is the missing ingress gate in user mode.

## Scope

- Keep the existing Stage 1 firmware and harness unchanged.
- Run one ROM baseline, then two user-mode runs:
  - user mode with hardware flow control off
  - user mode with hardware flow control on
- Compare whether any host->Murata ingress appears only in one flow-control mode.

## Tooling Anchors

- End-to-end launcher (Windows): `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_g_stage1_end_to_end.ps1`
- X8 Stage 1 runner: `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_g_stage1.sh`
- Flash orchestrator: `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_flash_l072.sh`
- ROM hold config: `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/07_assert_pa11_pf4_long.cfg`
- User boot config: `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/08_boot_user_app.cfg`
- ROM verifier: `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/verify_l072_rom.sh`
- UART diagnostic (now flow-toggle capable): `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/diag_uart_rxtx.py`

## Step 1: ROM Baseline (must pass)

1. Push helper toolkit and open a shell on the X8.
2. Enter ROM hold mode via the existing flash path (`run_flash_l072.sh`), or run the OpenOCD ROM hold config directly.
3. In a second X8 shell, run:

```bash
echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/verify_l072_rom.sh
```

Expected:
- `ROM_RESP_SIZE > 0`
- first byte after `0x7F` is usually `0x79` (ACK)

If ROM baseline fails, stop. The rest of the A/B result is not trustworthy.

## Step 2: Boot User Firmware

Use existing Stage 1 flow to flash and boot the custom image to user mode:

```bash
echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/run_method_g_stage1.sh /tmp/lifetrac_p0c/firmware.bin
```

This will capture standard Stage 1 logs under `/tmp/lifetrac_p0c/`.

## Step 3: User-Mode A/B (Flow Off vs On)

Run both commands back-to-back on the X8 while the board is in user mode.

A: hardware flow control OFF

```bash
python3 /tmp/lifetrac_p0c/diag_uart_rxtx.py --dev /dev/ttymxc3 --baud 921600 --hwflow off
```

B: hardware flow control ON

```bash
python3 /tmp/lifetrac_p0c/diag_uart_rxtx.py --dev /dev/ttymxc3 --baud 921600 --hwflow on
```

Capture both stdout logs into the same bench-evidence folder.

## Decision Matrix

- Case A: both OFF and ON are silent
  - Flow control is not the only blocker.
  - Next step: route-pin ownership tracing on Max Carrier/H747 side.

- Case B: OFF silent, ON gets replies
  - Missing gate is strongly tied to RTS/CTS behavior.
  - Next step: lock host serial settings and document required RTS/CTS state for probe path.

- Case C: ON silent, OFF gets replies
  - Flow-control assertion is blocking ingress.
  - Next step: force no-hwflow in all probe/launcher paths and verify repeatability.

- Case D: both get replies
  - Gate is timing-sensitive or run-order dependent.
  - Next step: preserve this run's exact sequencing and compare to failing sequence.

## Minimal Evidence To Save

- `method_g_stage1.log`
- `method_g_stage1_ocd.log`
- `flash_run.log`
- `flash_ocd.log`
- A/B outputs from:
  - `diag_uart_rxtx.py --hwflow off`
  - `diag_uart_rxtx.py --hwflow on`

## Current Acceptance Criterion

This checklist is complete when one of the four matrix cases is observed with reproducible evidence on board `2E2C1209DABC240B` and linked from the corresponding bench-evidence folder.