# 1. Bill of Materials

> **Heads up!** Lead times on Arduino Pro hardware (Portenta, Opta, expansions) run 2–4 weeks at most distributors. Order this list **before** you start machining the enclosure — you don't want a finished box and no boards.

This BOM matches the canonical design in [`../DESIGN-CONTROLLER/HARDWARE_BOM.md`](../DESIGN-CONTROLLER/HARDWARE_BOM.md). Prices are USD as of April 2026 and are approximate — check the linked product page for current numbers.

For each line item we link to **DigiKey or Mouser first** (most reliable stock + datasheets), then the manufacturer or Arduino Store as a fallback. Any UVC webcam, any 12 V buzzer, any 18650 cell etc. will work — the linked parts are just ones we've verified.

## Tier 1 — Tractor Node

| Qty | Part | Source | Approx. price |
|---:|---|---|---:|
| 1 | **Arduino Portenta Max Carrier** (ABX00043) | [Mouser](https://www.mouser.com/c/?q=ABX00043) · [DigiKey](https://www.digikey.com/en/products/result?keywords=ABX00043) · [Arduino Store](https://store-usa.arduino.cc/products/portenta-max-carrier) | $335 |
| 1 | **Arduino Portenta X8** (ABX00049) | [Mouser](https://www.mouser.com/c/?q=ABX00049) · [DigiKey](https://www.digikey.com/en/products/result?keywords=ABX00049) · [Arduino Store](https://store-usa.arduino.cc/products/portenta-x8) | $200 |
| 1 | **Arduino Opta WiFi** (AFX00002) | [Mouser](https://www.mouser.com/c/?q=AFX00002) · [DigiKey](https://www.digikey.com/en/products/result?keywords=AFX00002) · [Arduino Store](https://store-usa.arduino.cc/products/opta-wifi) | $190 |
| 1 | **Opta Ext D1608S** (AFX00006, **SSR variant** — *not* the D1608E EMR) | [Mouser](https://www.mouser.com/c/?q=AFX00006) · [DigiKey](https://www.digikey.com/en/products/result?keywords=AFX00006) · [Arduino Store](https://store-usa.arduino.cc/products/opta-ext-d1608s) | $130 |
| 1 | **Opta Ext A0602** | [Mouser](https://www.mouser.com/c/?q=AFX00007) · [DigiKey](https://www.digikey.com/en/products/result?keywords=AFX00007) · [Arduino Store](https://store-usa.arduino.cc/products/opta-ext-a0602) | $130 |
| 1 | LoRa antenna, 915 MHz SMA, 3 dBi whip | [DigiKey: ANT-916-CW-RCT-SS](https://www.digikey.com/en/products/result?keywords=ANT-916-CW-RCT-SS) | $8 |
| 1 | microSD, 32 GB industrial | [DigiKey: industrial microSD 32GB](https://www.digikey.com/en/products/filter/memory-cards/498) | $15 |
| 1 set | EMI ferrite bead kit, snap-on, mixed sizes | [DigiKey: ferrite bead kit](https://www.digikey.com/en/products/result?keywords=ferrite+bead+kit) | $8 |
| 1 | Master battery cutoff switch, 175 A | [DigiKey: Blue Sea 6005](https://www.digikey.com/en/products/result?keywords=blue+sea+6005) | $25 |
| 1 set | ATC blade fuse holder + assorted fuses (5/10/20 A) | [DigiKey: ATC fuse kit](https://www.digikey.com/en/products/result?keywords=ATC+fuse+kit) | $15 |
| 1 | 18650 LiPo, 3500 mAh, **protected** | [DigiKey: 18650 protected](https://www.digikey.com/en/products/result?keywords=18650+protected+3500) | $12 |
| 1 | TP4056 18650 charger module, USB-C | [DigiKey: TP4056 USB-C](https://www.digikey.com/en/products/result?keywords=TP4056) | $4 |
| 1 | **Phoenix Contact PSR-MC38** dual-channel safety relay | [Mouser: 2986960](https://www.mouser.com/c/?q=2986960) · [DigiKey](https://www.digikey.com/en/products/result?keywords=PSR-MC38) | $135 |
| 1 | Engine-kill automotive relay, 30 A SPDT (Bosch 0332019150) | [DigiKey: Bosch 0332019150](https://www.digikey.com/en/products/result?keywords=0332019150) | $8 |
| 3 | Status LED, 12 V panel-mount (red, green, amber) | [DigiKey: 12V panel LED](https://www.digikey.com/en/products/filter/panel-indicators-pilot-lights/465) | $15 total |
| 1 | 12 V piezo buzzer, panel-mount | [DigiKey: 12V piezo buzzer panel](https://www.digikey.com/en/products/result?keywords=12V+panel+buzzer) | $8 |
| 1 | DIN rail, 35 mm × 300 mm + end stops | [DigiKey: DIN rail 300mm](https://www.digikey.com/en/products/result?keywords=DIN+rail+300mm) | $12 |
| 2 | M12-A 5-pin cordset, preassembled, 2 m (TURCK / Phoenix) — for RS-485 | [Mouser: M12-A 5-pin 2m](https://www.mouser.com/c/?q=M12+A-coded+5-pin+2m) | $40 total |
| 1 | RS-485 termination resistor pair, 120 Ω | [DigiKey: 120Ω 1/4W](https://www.digikey.com/en/products/result?keywords=120+ohm+1%2F4W) | $1 |
| 2 | Hydraulic pressure sensor, 0–250 bar, 4–20 mA, M12-A | [DigiKey: pressure transducer 4-20mA M12](https://www.digikey.com/en/products/filter/pressure-sensors-transducers/498) | $80 total |
| 2 | M12-A 4-pin cordset, 2 m, for the pressure sensors | [Mouser: M12-A 4-pin 2m](https://www.mouser.com/c/?q=M12+A-coded+4-pin+2m) | $30 total |
| 1 | **Hammond 1554 / 1555 polycarbonate IP66 enclosure**, ~250×200×100 mm | [DigiKey: Hammond 1554](https://www.digikey.com/en/products/filter/boxes/164?s=N4IgTCBcDaIIIBkCMBOAtAVgGwCYwBoQBdAXyA) · [Mouser: Hammond 1554](https://www.mouser.com/c/?q=Hammond+1554) | $60 |
| 1 set | Cable glands, M16/M20 IP68 (×6) | [DigiKey: cable gland kit IP68](https://www.digikey.com/en/products/result?keywords=cable+gland+IP68+M20) | $20 |
| 1 | Conformal coating spray (MG Chemicals 419 acrylic) | [DigiKey: MG Chemicals 419](https://www.digikey.com/en/products/result?keywords=MG+419) | $18 |
| 1 set | Deutsch DT connector kit + crimper (rent or share) | [DigiKey: Deutsch DT kit](https://www.digikey.com/en/products/result?keywords=Deutsch+DT+kit) | $90 |
| 1 | Inline 5 A blade fuse holder + SMBJ33A TVS | [DigiKey: SMBJ33A](https://www.digikey.com/en/products/result?keywords=SMBJ33A) | $5 |
| 1 | **Adafruit MCP2221A** breakout (USB ↔ I²C, Stemma QT) | [DigiKey: Adafruit 4471](https://www.digikey.com/en/products/result?keywords=adafruit+4471) · [Mouser](https://www.mouser.com/c/?q=adafruit+4471) | $8 |
| 1 | **SparkFun BNO086 Qwiic IMU** (DEV-22857) | [DigiKey: SparkFun DEV-22857](https://www.digikey.com/en/products/result?keywords=DEV-22857) · [Mouser](https://www.mouser.com/c/?q=DEV-22857) | $30 |
| 1 | **SparkFun NEO-M9N Qwiic GPS** SMA variant (GPS-15733) | [DigiKey: GPS-15733](https://www.digikey.com/en/products/result?keywords=GPS-15733) · [Mouser](https://www.mouser.com/c/?q=GPS-15733) | $75 |
| 2 | Qwiic / Stemma QT 50 mm cable | [DigiKey: Qwiic cable 50mm](https://www.digikey.com/en/products/result?keywords=qwiic+cable+50mm) | $4 total |
| 1 | Active SMA GPS patch antenna, magnetic mount, 3 m | [DigiKey: GPS patch SMA 3m](https://www.digikey.com/en/products/result?keywords=GPS+antenna+SMA+magnetic) | $20 |
| 1 | SMA bulkhead pass-through, panel-mount | [DigiKey: SMA bulkhead](https://www.digikey.com/en/products/result?keywords=SMA+bulkhead+panel) | $5 |
| 1 | **Kurokesu C2-290C** USB camera (or Logitech C920 substitute) | [Kurokesu shop](https://www.kurokesu.com/shop/cameras/CAMUSB1) · [DigiKey: Logitech C920](https://www.digikey.com/en/products/result?keywords=C920) | $70–$240 |
| 1 | Panel-mount USB-A bulkhead + IP-rated cable gland | [DigiKey: USB-A bulkhead](https://www.digikey.com/en/products/result?keywords=USB-A+bulkhead+panel) | $12 |

**Tier 1 subtotal: ~$1,750**

## Tier 2 — Base Station

| Qty | Part | Source | Approx. price |
|---:|---|---|---:|
| 1 | Arduino Portenta Max Carrier (ABX00043) | (same links as Tier 1) | $335 |
| 1 | Arduino Portenta X8 (ABX00049) | (same links as Tier 1) | $200 |
| 1 | LoRa antenna, **8 dBi 915 MHz omni mast** | [L-com HG908U-PRO](https://www.l-com.com/wireless-antenna-915-mhz-8-dbi-omni) · [DigiKey: 915 MHz 8dBi omni](https://www.digikey.com/en/products/filter/rf-antennas/871) | $80 |
| 1 | LMR-400 coax, 3 m, SMA-M to N-M | [DigiKey: LMR-400 SMA-N 3m](https://www.digikey.com/en/products/result?keywords=LMR-400+SMA+N+3m) | $45 |
| 1 | N-female bulkhead lightning arrestor | [DigiKey: lightning arrestor N-female](https://www.digikey.com/en/products/filter/lightning-arrestors/1019) | $35 |
| 1 | Galvanized mast, ~3 m + ground rod | local hardware store | $50 |
| 1 | 12 V / 5 A indoor PSU | [DigiKey: 12V 5A bench PSU](https://www.digikey.com/en/products/filter/ac-dc-converters/924) | $25 |
| 1 | Mini UPS, 12 V, 30 min runtime | [DigiKey: 12V mini UPS](https://www.digikey.com/en/products/result?keywords=12V+mini+UPS) | $80 |
| 1 | Indoor ventilated enclosure (rack box or wall-mount) | local | $50 |
| 1 | Cat6 Ethernet, 5 m | local | $10 |

**Tier 2 subtotal: ~$910**

## Tier 3 — Handheld (Optional)

| Qty | Part | Source | Approx. price |
|---:|---|---|---:|
| 1 | **Arduino MKR WAN 1310** (ABX00029) | [Mouser](https://www.mouser.com/c/?q=ABX00029) · [DigiKey](https://www.digikey.com/en/products/result?keywords=ABX00029) · [Arduino Store](https://store-usa.arduino.cc/products/arduino-mkr-wan-1310) | $45 |
| 1 | 1/4-wave 915 MHz whip with u.FL pigtail | [DigiKey: 915 MHz whip uFL](https://www.digikey.com/en/products/result?keywords=915+MHz+whip+uFL) | $8 |
| 2 | Dual-axis analog joystick (Adafruit 444 or similar) | [DigiKey: Adafruit 444](https://www.digikey.com/en/products/result?keywords=adafruit+444) | $12 total |
| 1 | 22 mm latching mushroom **E-stop** | [DigiKey: 22mm mushroom estop](https://www.digikey.com/en/products/filter/emergency-stop-switches/237) | $20 |
| 4 | Momentary push button, IP67 tactile | [DigiKey: IP67 momentary 12mm](https://www.digikey.com/en/products/result?keywords=IP67+pushbutton+12mm) | $25 total |
| 1 | SSD1306 OLED, 128×64, I²C | [DigiKey: SSD1306 128x64 I2C](https://www.digikey.com/en/products/result?keywords=SSD1306+128x64+I2C) | $10 |
| 1 | LiPo, 1S 2000 mAh | [DigiKey: 1S LiPo 2000mAh](https://www.digikey.com/en/products/result?keywords=1S+LiPo+2000) | $10 |
| 1 | USB-C panel-mount breakout | [DigiKey: USB-C panel breakout](https://www.digikey.com/en/products/result?keywords=USB-C+panel+breakout) | $8 |
| 1 | IP54 handheld enclosure (~120×80×40 mm) | [DigiKey: Hammond 1554SBKBT](https://www.digikey.com/en/products/result?keywords=Hammond+handheld+1554) | $25 |
| 1 | Custom joystick PCB (KiCad files in `/handheld/pcb/`) | [OSH Park](https://oshpark.com/) · [JLCPCB](https://jlcpcb.com/) | $20 / 5-pack |

**Tier 3 subtotal: ~$185**

## Development Gear (One-Time)

| Qty | Part | Why | Approx. price |
|---:|---|---|---:|
| 1 | NooElec NESDR Smart RTL-SDR | LoRa packet sniffing during bring-up | $40 |
| 1 | Saleae Logic 8 (or quality clone) | RS-485 + Modbus debugging | $100–400 |
| 1 | NanoVNA-H4 | Antenna SWR check before mounting | $60 |
| 2 | 50 Ω SMA dummy load, 5 W | Bench-test radios safely without antennas | $20 total |
| 1 | Bench PSU, 0–30 V / 0–5 A | Power-up / brown-out testing | $80 |

## Spare Parts (Recommended)

Order **two of each** of these so a single dead board doesn't stall the build:

- Portenta Max Carrier · Portenta X8 · MKR WAN 1310 · Opta WiFi
- LoRa whip antenna · GPS patch antenna · joystick · OLED

Spare-parts subtotal: **~$1,400**.

## Grand Totals

| Scope | Hardware cost | Recurring |
|---|---:|---:|
| **One deployed system** (1 tractor + 1 base + 1 handheld) | **~$2,850** | $0/mo (no SIM cards in v25) |
| Plus development gear | $300–600 | |
| Plus recommended spares | $1,400 | |

> **Pro tip:** If you're sourcing for a single first build and on a budget, you can **skip the handheld** and the second cellular SIM (cellular is out of scope for v25 anyway — see [`../DESIGN-CONTROLLER/MASTER_PLAN.md`](../DESIGN-CONTROLLER/MASTER_PLAN.md) §1). That brings the minimum viable build down to about **$2,650**.

## Substitutions and Regional Notes

- **EU (868 MHz)** instead of US (915 MHz): swap to **MKR WAN 1300** for the handheld, confirm the Max Carrier's Murata module is the 868 MHz variant at order time, and pick an 868 MHz omni mast antenna in Tier 2.
- **Any UVC USB camera** works for first-light video. Logitech C920, ELP IP67, Kurokesu C1 PRO are all fine — they all enumerate as `/dev/video0` on the X8 Yocto image.
- **Camera enclosure** — see [`../DESIGN-CONTROLLER/HARDWARE_BOM.md` § "Camera enclosure path"](../DESIGN-CONTROLLER/HARDWARE_BOM.md#notes-on-substitutions) for cab-mounted vs NEMA 4X external trade-off.
- **Do not substitute the D1608S with the D1608E.** The "S" is solid-state relays (sub-millisecond, what we want for valves); the "E" is electromechanical (8–20 ms pickup latency, too slow for the control hot path). See [`../DESIGN-CONTROLLER/MASTER_PLAN.md`](../DESIGN-CONTROLLER/MASTER_PLAN.md) §8.18.

## Next Step

Once parts are in, head to [**2. Tractor Node Assembly →**](02_tractor_node_assembly.md).
