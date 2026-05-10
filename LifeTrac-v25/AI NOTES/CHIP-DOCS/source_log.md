# Source Log

Date: 2026-05-09

## Arduino Official Pages
- Portenta Max Carrier hardware page:
  - https://docs.arduino.cc/hardware/portenta-max-carrier/
- Portenta Max Carrier user manual:
  - https://docs.arduino.cc/tutorials/portenta-max-carrier/user-manual
- Portenta Max Carrier store page:
  - https://store.arduino.cc/products/portenta-max-carrier

## Primary Documents to Pull into CHIP-DOCS
- Max Carrier schematics PDF (ABX00043):
  - https://docs.arduino.cc/resources/schematics/ABX00043-schematics.pdf
- Max Carrier datasheet PDF (ABX00043):
  - https://docs.arduino.cc/resources/datasheets/ABX00043-datasheet.pdf
- Max Carrier full pinout PDF (ABX00043):
  - https://docs.arduino.cc/resources/pinouts/ABX00043-full-pinout.pdf

## Confirmed from Official Metadata
- Product SKU for Max Carrier: ABX00043
- Wireless module listed: Murata CMWX1ZZABZ-078

## Immediate Extraction Tasks
1. Parse ABX00043 schematics for i.MX8 UART4_TXD and UART4_RXD net names.
2. Trace all intermediate components from UART4_TXD to Murata L072 RX candidate pins.
3. Identify any direction-control elements (OE, EN, SEL, mux gates) on that path.
4. Map control nets to owning controller pins (H7, i.MX, strap resistors, fixed logic).
5. Add extracted evidence to:
   - CHIP-DOCS/IMX8MM_UART4/findings.md
   - CHIP-DOCS/MURATA_CMWX1ZZABZ_078/findings.md
   - CHIP-DOCS/STM32H747/findings.md

## Manual Value
- The user manual is useful for carrier-level topology, connector inventory, and DIP-switch behavior.
- For exact UART/control routing, the schematics and full pinout remain the authoritative references.
