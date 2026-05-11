# Findings

- Official Max Carrier documentation is tied to SKU ABX00043.
- Official downloadable artifacts for ABX00043 include:
	- Schematics: https://docs.arduino.cc/resources/schematics/ABX00043-schematics.pdf
	- Datasheet: https://docs.arduino.cc/resources/datasheets/ABX00043-datasheet.pdf
	- Full pinout: https://docs.arduino.cc/resources/pinouts/ABX00043-full-pinout.pdf
- In current bench runs on board 2E2C1209DABC240B, Linux host traffic uses /dev/ttymxc3 as the active serial endpoint for Murata/L072 probing and flashing workflow.
- The current working notes treat /dev/ttymxc3 as a direct i.MX UART endpoint, not an H7-bridged path, so passive Linux probing does not observe the H7-side UART traffic.
- The carrier-side host link is described in the existing notes as a 4-wire UART with RTS/CTS present, so TX/RX-only assumptions are unsafe for later route tests.
- Controlled A/B behavior is confirmed:
	- STM32 ROM mode is reachable and responds at 19200 8E1.
	- User firmware mode emits outbound URCs but does not receive host ingress on the same path.
- From the official Portenta Max Carrier user-manual topology table, the following active devices are present and should be treated as first-class route-control candidates on UART-adjacent paths:
	- `U16-U19`: `74LVC1G157` (single 2-input multiplexers)
	- `U8/U20/U21/U22`: `SN74LVC1T45` (bi-directional level converters)
	- `U10`: `SN74LVC1G125` (single bus buffer gate)
	- `U23`: `CMWX1ZZABZ-078` Murata LPWAN module
- This component inventory supports the current blocker classification that ingress may be gated by control ownership (OE/SEL/EN/DIR) rather than by baud/framing only.
