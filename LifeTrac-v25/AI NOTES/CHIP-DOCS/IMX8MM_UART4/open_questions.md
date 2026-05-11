# Open Questions

- What are the exact ABX00043 schematic net names for i.MX8 UART4_TXD and UART4_RXD?
- Which active components exist on UART4_TXD path to Murata L072 RX (level shifter, mux, buffer, analog switch)?
- Are there OE/EN/SEL control pins on that path, and which controller owns them?
- Does boot-state (ROM vs user firmware) alter any carrier-level route selection logic that affects only host-to-L072 direction?

## Owner-Net Extraction Checklist (Next Pass)

- For `U16-U19 (74LVC1G157)`, record all signal nets and select-pin owners that touch UART4/L072 route.
- For `U8/U20/U21/U22 (SN74LVC1T45)`, record DIR/OE ownership and default strap state.
- For `U10 (SN74LVC1G125)`, record enable pin owner and polarity.
- Map each discovered control net to one of: i.MX-owned, H747-owned, strap/fixed.
- Produce an OpenOCD matrix containing only owner nets that directly gate host->L072 ingress.
