# i.MX8M Mini UART4

## Scope
- UART4_TXD and UART4_RXD pad configuration and driver behavior
- DTS/pinctrl ownership and runtime state
- TX path diagnostics in user mode

## Add Here
- `docs/` i.MX8MM UART chapter extracts and pinctrl references
- `notes/` runtime checks (stty, dmesg, pinctrl dumps)
- `findings.md` confirmed UART4 behavior
- `open_questions.md` unresolved TX path anomalies

## Immediate Checklist
- Validate no runtime reconfiguration of UART4_TXD pad
- Map UART4_TXD net through carrier-level circuitry
