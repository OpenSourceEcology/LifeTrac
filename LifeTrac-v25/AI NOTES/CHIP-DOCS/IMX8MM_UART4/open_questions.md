# Open Questions

- What are the exact ABX00043 schematic net names for i.MX8 UART4_TXD and UART4_RXD?
- Which active components exist on UART4_TXD path to Murata L072 RX (level shifter, mux, buffer, analog switch)?
- Are there OE/EN/SEL control pins on that path, and which controller owns them?
- Does boot-state (ROM vs user firmware) alter any carrier-level route selection logic that affects only host-to-L072 direction?
