# Controller Stage 1 RTS/CTS A/B Run Result (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B
Evidence folder: `DESIGN-CONTROLLER/bench-evidence/T6_rtscts_ab_2026-05-09_183033/`

## Commands Run

```bash
python3 /tmp/lifetrac_p0c/diag_uart_rxtx.py --dev /dev/ttymxc3 --baud 921600 --hwflow off
python3 /tmp/lifetrac_p0c/diag_uart_rxtx.py --dev /dev/ttymxc3 --baud 921600 --hwflow on
```

## Observed Output Summary

- `hwflow=off`
  - passive listen: no bytes
  - `ATI`: no bytes
  - binary `VER_REQ`: no bytes
  - binary `PING_REQ`: no bytes

- `hwflow=on`
  - `stty` reported: `unable to perform all requested operations`
  - passive listen: no bytes
  - `ATI`: no bytes
  - binary `VER_REQ`: no bytes
  - binary `PING_REQ`: no bytes

## Classification

- Case A from the checklist: both OFF and ON runs are silent.
- Result does not support RTS/CTS mode mismatch as the primary root cause on this path.

## Immediate Implication

- Keep the ingress blocker classified as carrier-side route/control-gate.
- Prioritize Max Carrier/H747 route net ownership tracing (mux/enable/strap path), not protocol/framing changes.
