# Hardware Bill of Materials — Controller System

All prices in USD, approximate, from primary distributor (Arduino Store, SparkFun, DigiKey, Mouser). Verify availability and regional frequency compliance (915 MHz parts shown; substitute 868 MHz for EU).

## Tier 1 — Tractor Node (Portenta Max Carrier + Portenta H7)

| Part | Qty | Unit | Subtotal | Source | Notes |
|---|---:|---:|---:|---|---|
| Arduino Portenta Max Carrier (ABX00043) | 1 | $335 | $335 | [Arduino Store](https://store-usa.arduino.cc/products/portenta-max-carrier) | Carrier with onboard LoRa + Cat-M1, both SMA |
| Arduino Portenta H7 (ABX00042) | 1 | $115 | $115 | [Arduino Store](https://store-usa.arduino.cc/products/portenta-h7) | M7 + M4 dual-core; can substitute Portenta X8 ($200) for Linux |
| LoRa antenna 915 MHz, SMA, 3 dBi whip | 1 | $8 | $8 | DigiKey / Mouser | Arduino-recommended ANT-8/9-IPW1-SMA |
| Cellular antenna 700–2700 MHz, SMA | 1 | $12 | $12 | DigiKey / Mouser | For SARA-R412M |
| Activated Cat-M1 SIM card (e.g. Hologram, Soracom) | 1 | $5 | $5/mo | [Hologram](https://www.hologram.io/) | IoT data plan, ~10 MB/mo sufficient for telemetry |
| microSD card, 32 GB, industrial | 1 | $15 | $15 | DigiKey | For onboard data logging |
| EMI filter / ferrite beads on 12 V input | 1 | $8 | $8 | DigiKey | Tractor electrical is electrically noisy |
| Master battery cutoff switch, 175 A | 1 | $18 | $18 | Blue Sea Systems | Shop-maintenance disconnect |
| Inline fuse holder + ATC blade fuses (assorted 5/10/20 A) | 1 set | $15 | $15 | DigiKey | Per-channel + main valve rail protection |
| 18650 LiPo cell, 3500 mAh, protected | 1 | $12 | $12 | Battery Junction | Backup power for graceful shutdown |
| TP4056 / MCP73831 LiPo charger module | 1 | $5 | $5 | DigiKey | Charges 18650 from main 12 V rail |
| **Arduino Opta WiFi (AFX00002)** | 1 | $150 | $150 | [Arduino Store](https://store-usa.arduino.cc/products/opta-wifi) | Industrial PLC: 4× 10 A relays + 8 digital/analog inputs + RS-485 + Ethernet. Acts as Modbus-RTU **slave**; carries valve coils + watchdog |
| **Opta Ext D1608S** expansion (8× relay outputs + 8× digital inputs) | 1 | $100 | $100 | [Arduino Store](https://store-usa.arduino.cc/products/opta-ext-d1608s) | Adds the remaining 4 directional-valve relays + 4 spare; 8 digital inputs for ignition / mode switch / etc. |
| **Opta Ext A0602** expansion (6× analog inputs + **2× 0–10 V analog outputs**) | 1 | $100 | $100 | [Arduino Store](https://store-usa.arduino.cc/products/opta-ext-a0602) | **Drives Burkert 8605 flow valve(s)** — supports dual-flow-valve config natively. 6 analog inputs for battery V / oil P / coolant T / etc. with built-in conditioning |
| **Engine-kill relay**, automotive 30 A SPDT | 1 | $8 | $8 | DigiKey | Energizes diesel fuel-cut solenoid; switched by an Opta relay channel through the safety chain |
| **Phoenix Contact PSR safety relay** (or equivalent dual-channel) | 1 | $90 | $90 | DigiKey (PSR-MC38 series) | Dual-channel monitored E-stop loop; gates valve power + engine kill, independent of both Opta and Max Carrier |
| Modbus RS-485 bus cable + 120 Ω termination resistors (×2) | 1 set | $8 | $8 | DigiKey | RJ-style or 3-wire shielded twisted-pair, Max Carrier J6 ↔ Opta RS-485 |
| DIN rail (35 mm × 300 mm) + end stops | 1 | $15 | $15 | McMaster | Mounts Opta + expansions inside enclosure |
| GPS active patch antenna, SMA, 28 dB LNA | 1 | $20 | $20 | DigiKey (Taoglas / U-blox) | For onboard u-blox GPS |
| Cab-roof antenna mounting bracket + N-bulkhead | 1 | $25 | $25 | L-com / DPD | Ground plane + weather-sealed pass-through for LoRa whip |
| Status LED panel (3 ×, panel-mount, 12 V) | 1 set | $10 | $10 | DigiKey | POWER / LINK / FAULT exterior indicators |
| Piezo buzzer, 12 V panel-mount | 1 | $6 | $6 | DigiKey | Audible alarm + take-control transition cue |
| Deutsch DT connector kit (4/6/8/12 way) + crimp tool rental | 1 | $80 | $80 | Ladd / DigiKey | Weatherproof harness terminations |
| IP65 enclosure, ~250×200×100 mm | 1 | $45 | $45 | Polycase / McMaster | Bigger than original; needs to fit the relay/driver/safety boards |
| Vibration-isolating enclosure mounts (rubber bobbins, ×4) | 1 set | $12 | $12 | McMaster | Protects PCB solder joints |
| Conformal coating spray (acrylic) | 1 | $15 | $15 | MG Chemicals | Humidity protection for all PCBs |
| Cable glands, M16/M20, IP68 | 6 | $3 | $18 | McMaster | Antenna + power + valve harness + telemetry pass-throughs |
| Misc connectors, ferrules, heat shrink, wire | — | — | $40 | — | |
| **Tier 1 Subtotal** | | | **~$1,265** + $5/mo | | Adopts Arduino Opta + expansions in place of discrete relay/DAC/opto/conditioning boards. See [TRACTOR_NODE.md § Why Opta as the valve controller](TRACTOR_NODE.md#why-opta-as-the-valve-controller) |

## Tier 2 — Base Station (Portenta Max Carrier + Portenta X8)

| Part | Qty | Unit | Subtotal | Source | Notes |
|---|---:|---:|---:|---|---|
| Arduino Portenta Max Carrier (ABX00043) | 1 | $335 | $335 | [Arduino Store](https://store-usa.arduino.cc/products/portenta-max-carrier) | |
| Arduino Portenta X8 (ABX00049) | 1 | $200 | $200 | [Arduino Store](https://store-usa.arduino.cc/products/portenta-x8) | Linux on i.MX 8M Mini quad-core |
| LoRa mast antenna, 915 MHz, 8 dBi omni | 1 | $45 | $45 | [L-com](https://www.l-com.com/wireless-antenna-900-mhz-omnidirectional-antennas) | For long-range tractor link |
| Cellular antenna 700–2700 MHz, SMA | 1 | $12 | $12 | DigiKey | Backup uplink |
| LMR-400 coax cable, SMA-M to N-M, 3 m | 1 | $35 | $35 | L-com | Mast-to-carrier; low-loss for 915 MHz |
| SMA-F to N-F bulkhead, lightning arrestor | 1 | $30 | $30 | L-com | Lightning protection at mast base |
| Mast, ~3 m galvanized + ground rod | 1 | $80 | $80 | Local hardware | Base station mast install |
| Activated Cat-M1 SIM card | 1 | $5 | $5/mo | Hologram | Backup uplink data |
| microSD card, 32 GB | 1 | $15 | $15 | DigiKey | X8 root filesystem expansion |
| 12 V / 5 A power supply, AC adapter | 1 | $25 | $25 | DigiKey | Indoor base station; UPS-backed |
| Mini UPS for base station (12 V, 30 min runtime) | 1 | $60 | $60 | Amazon | Survives brief power drops |
| Indoor enclosure, ventilated | 1 | $25 | $25 | Polycase | Wall-mount or shelf |
| Ethernet cable, Cat6, 5 m | 1 | $10 | $10 | DigiKey | To office switch/router |
| **Tier 2 Subtotal** | | | **$872** + $5/mo | | |

## Tier 3 — Handheld Remote (MKR WAN 1310)

| Part | Qty | Unit | Subtotal | Source | Notes |
|---|---:|---:|---:|---|---|
| Arduino MKR WAN 1310 (ABX00029) | 1 | $45 | $45 | [Arduino Store](https://store-usa.arduino.cc/products/arduino-mkr-wan-1310) | Same Murata LoRa SiP as Max Carrier |
| LoRa antenna, 915 MHz, 1/4-wave whip | 1 | $5 | $5 | Adafruit / DigiKey | u.FL connector or via dipole |
| Analog joystick, dual-axis with click | 2 | $8 | $16 | SparkFun / Adafruit | Hydraulic drive + implement control |
| Latching mushroom E-stop button, 22 mm | 1 | $15 | $15 | DigiKey (IDEC) | Hard-wired to engine kill relay logic |
| Momentary push buttons (TAKE CONTROL, mode, etc.) | 4 | $2 | $8 | DigiKey | Tactile, IP67-rated |
| OLED display, 128×64, I²C, SSD1306 | 1 | $10 | $10 | Adafruit | Status: source priority, RSSI, battery |
| LiPo battery, 1S 2000 mAh | 1 | $10 | $10 | Adafruit | ~8 h continuous operation |
| LiPo charger / protection IC (or use MKR's onboard) | — | — | — | — | MKR WAN 1310 has onboard charger |
| USB-C connector breakout (charge + program) | 1 | $4 | $4 | SparkFun | Panel-mount |
| IP54 handheld enclosure, ~120×80×40 mm | 1 | $25 | $25 | Hammond / Polycase | With strap/lanyard mount |
| Custom PCB (joysticks → MKR pin headers) | 1 | $30 | $30 | OSHPark (5-pack ÷ 5) | Optional; can use perfboard for prototype |
| Misc wiring, heat shrink, mounting hardware | — | — | $15 | — | |
| **Tier 3 Subtotal** | | | **$183** | | |

## Development & test gear (one-time)

| Part | Qty | Unit | Subtotal | Source | Notes |
|---|---:|---:|---:|---|---|
| RTL-SDR USB receiver | 1 | $35 | $35 | NooElec | LoRa packet sniffing during bring-up |
| Logic analyzer (Saleae Logic 8 or clone) | 1 | $80 | $80 | Saleae / Amazon | Latency measurement |
| Spectrum analyzer access (rental or borrowed) | — | — | — | — | For FCC EIRP verification |
| Bench power supply, 0–30 V, 0–5 A | 1 | $80 | $80 | Amazon | Brown-out testing |
| 50 Ω dummy load, SMA, 5 W | 2 | $15 | $30 | DigiKey | Bench TX without radiating |
| Spare parts (×2 of each tier hardware) | — | — | ~$1,700 | — | Back-up units for development |
| **Dev Subtotal** | | | **$225** + spares | | |

## Grand totals

| Scope | Hardware cost | Recurring |
|---|---:|---:|
| **Single deployed system** (1 tractor + 1 base + 1 handheld) | **~$2,320** | **$10/mo** (2× SIM cards) |
| **Plus development gear** (one-time) | $1,914 | |
| **Plus spare parts** (recommended) | $3,400 | |

> Tier 1 was revised upward from the initial $634 estimate after auditing against
> [DESIGN-HYDRAULIC/HYDRAULIC_DIAGRAM.md](../DESIGN-HYDRAULIC/HYDRAULIC_DIAGRAM.md):
> the directional valves are 12 V (not 24 V), but a realistic deployment also
> needs the safety relay, opto-isolation, fuses, EMI filtering, harness connectors,
> 0–10 V DAC for the Burkert 8605 flow valve, and external status indicators.

## Notes on substitutions

- **Portenta H7 vs X8 on the tractor:** H7 is the default (cheaper, real-time M4 core for valve control). Swap to X8 ($85 more) only if you need Linux on the tractor for camera processing or onboard ML. The base station X8 is *not* optional — you need Linux there for the web UI.
- **MKR WAN 1310 vs MKR WAN 1300:** the 1310 adds a battery charger and faster processor; spec is otherwise identical. Always buy the 1310.
- **Mast height:** for sites where 8 dBi omni isn't enough range, swap to a 12 dBi Yagi pointed at the typical work area. Yagi gives ~15 dB more gain in one direction; total link budget improves by ~7 dB after antenna match losses.
- **Cellular SIM:** any Cat-M1 IoT SIM works. Hologram, Soracom, Twilio Super SIM all have global coverage.
- **915 MHz vs 868 MHz:** for EU deployments, swap to MKR WAN 1300 (868 MHz variant) and confirm the Max Carrier's Murata module is the 868 MHz variant at order time.

## Lead times (typical)

- Arduino Pro hardware: 2–4 weeks (often in stock at distributors)
- Mast antenna + LMR-400 cable: 1 week (L-com US stock)
- Custom PCB (handheld joystick board): 2 weeks via OSHPark / JLCPCB
- IP65 enclosures: 1 week
