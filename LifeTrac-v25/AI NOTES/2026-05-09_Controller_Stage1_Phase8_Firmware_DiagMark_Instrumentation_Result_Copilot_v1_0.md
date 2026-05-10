# Controller Stage 1 Phase 8 Firmware Diag-Mark Instrumentation Result (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Scope

Escalate from GPIO/AF perturbations to firmware-level runtime observability by instrumenting host ingress branch progression and surfacing it over existing `FAULT_URC` telemetry.

## Firmware Changes

### New diagnostic fault code

- `HOST_FAULT_CODE_HOST_DIAG_MARK = 0x0C` added in `include/host_types.h`.

### New diagnostic marks

Added mark bits in `include/host_uart.h`:

- `HOST_DIAG_MARK_VER_REQ_PARSED (0x01)`
- `HOST_DIAG_MARK_FRAME_PARSE_ERR (0x02)`
- `HOST_DIAG_MARK_VER_REQ_DISPATCHED (0x04)`
- `HOST_DIAG_MARK_VER_URC_SENT (0x08)`
- `HOST_DIAG_MARK_AT_VER_DISPATCHED (0x10)`

### Runtime mark plumbing

- `host/host_uart.c`
  - tracks mark bits (`s_diag_marks`)
  - sets marks on parse errors and `VER_REQ` parse
  - exposes `host_uart_note_diag_mark()` and `host_uart_take_diag_marks()`
- `host/host_cmd.c`
  - sets marks on `VER_REQ` dispatch, `VER_URC` send path, and `AT+VER?` dispatch
- `main.c`
  - emits `FAULT_URC` code `0x0C` when new marks are observed

### Probe decoder update

- `x8_lora_bootloader_helper/method_g_stage1_probe.py`
  - decodes `FAULT_URC code=0x0C` and prints symbolic marks

## Validation Run

Evidence folder:

- `DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_190924/`

Run status:

- build succeeded (`firmware.bin` 15000 bytes)
- flash succeeded (AN3155 verify pass)
- Stage 1 probe still failed (`__METHOD_G_RC__=2`)

Observed probe signature:

- `STATS_URC(init): host_rx_bytes=0 host_rx_lpuart=0 host_rx_usart1=0 ...`
- `BOOT_URC: not observed during initial window`
- `FAULT_URC code=0x0A` (`HOST_RX_INACTIVE`)
- timeout waiting for `VER_URC` (`0x81` to request `0x01`)

Not observed:

- no `FAULT_URC code=0x0C` (`HOST_DIAG_MARK`) in active probe window

## Interpretation

The new instrumentation did not fire because ingress still appears absent before parser/dispatch branch points.

This is a stronger confirmation of the existing classifier:

- host bytes are not reaching the firmware ingest path during active probe traffic,
- therefore parser-level or command-dispatch fixes cannot restore `VER_URC` until transport ingress is restored.

## Recommended Next Step

Focus next discriminators on pre-parser transport ingress itself (physical route owner/gating domain), with explicit A/B evidence tied to first non-zero `host_rx_bytes` transition.
