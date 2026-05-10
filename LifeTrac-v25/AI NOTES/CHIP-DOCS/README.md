# Chip Documents and Notes

Purpose: centralize vendor documentation, board-level references, and debug notes by chip.

## Folder Layout
- `STM32L072/` - STM32L072 docs and notes (Murata internal MCU)
- `STM32H747/` - H7 docs and notes (Portenta X8 bridge/control MCU)
- `IMX8MM_UART4/` - i.MX8M Mini UART4 and pinctrl docs/notes
- `SX1276/` - SX1276 radio docs and notes
- `MURATA_CMWX1ZZABZ_078/` - Murata module package/integration docs

## Conventions
- Save original vendor files in a `docs/` subfolder when added.
- Keep investigation logs in `notes/`.
- Use `findings.md` per chip for confirmed conclusions only.
- Use `open_questions.md` per chip for unresolved items and next checks.

## Current Priority
1. Max Carrier schematic and exact net path from i.MX UART4_TXD to Murata RX
2. Any buffer/mux/OE control net and its owning GPIO/controller
3. ROM mode vs user mode path differences on the same electrical route

## Seed Notes
- [2026-05-09_Ingress_Parity_Contradiction_Resolution_Copilot_v1_0.md](../2026-05-09_Ingress_Parity_Contradiction_Resolution_Copilot_v1_0.md)
- [2026-05-09_ROM_vs_User_AB_19200_8E1_and_RX_Echo_Diagnostic_Copilot_v1_0.md](../2026-05-09_ROM_vs_User_AB_19200_8E1_and_RX_Echo_Diagnostic_Copilot_v1_0.md)
- [2026-05-09_RX_Echo_Diagnostic_Run_and_Chip_Document_Checklist_Copilot_v1_0.md](../2026-05-09_RX_Echo_Diagnostic_Run_and_Chip_Document_Checklist_Copilot_v1_0.md)
