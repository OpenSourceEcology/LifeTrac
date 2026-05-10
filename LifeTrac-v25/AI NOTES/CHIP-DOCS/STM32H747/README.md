# STM32H747

## Scope
- H7 ownership of LoRa control pins (BOOT0/NRST and possible gating lines)
- OpenOCD attach/reset side effects
- Control-state persistence that may impact UART path direction

## Add Here
- `docs/` reference manual, datasheet, board-specific pin maps
- `notes/` control-pin experiments and OpenOCD scripts
- `findings.md` confirmed pin ownership and behavior
- `open_questions.md` unresolved H7 control-state effects

## Immediate Checklist
- Identify all H7-controlled nets in the LoRa UART path
- Verify any OE/SEL nets that can disable host->L072 direction only
