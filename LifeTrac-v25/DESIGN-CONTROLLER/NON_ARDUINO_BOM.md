# Non-Arduino Bill of Materials

> Counterpart to the Arduino-store list in [HARDWARE_BOM.md](HARDWARE_BOM.md). This file consolidates every line item that ships from somewhere other than store.arduino.cc — DigiKey / Mouser / L-com / Phoenix Contact / Bürkert / McMaster-Carr / Coral.ai. Use it to copy-paste into a single procurement spreadsheet.

## Conventions

- Quantities are **per LifeTrac v25 unit** (1 tractor + 1 base station + 1 handheld).
- Prices are excluded — they fluctuate weekly. Use the linked vendor for live pricing.
- Distributor part numbers (DPN) are given alongside manufacturer part numbers (MPN) so a buyer can paste the DPN straight into a cart.
- "Spare?" column flags items that should ship in the spare-parts kit per [FIELD_SERVICE.md](FIELD_SERVICE.md).

## Tier 1 — Tractor node (Portenta H747 + Max Carrier)

| # | Item | MPN | Distributor | DPN | Qty | Spare? | Notes |
|---|------|-----|-------------|-----|-----|--------|-------|
| 1 | SX1276 LoRa breakout, 915 MHz | RFM95W-915S2 | DigiKey | 1568-1656-ND | 1 | ✓ | Connects to Max Carrier MKR header per [LORA_PROTOCOL.md](LORA_PROTOCOL.md). |
| 2 | u.FL → SMA pigtail, 100 mm | 415-0058-100 | Mouser | 415-0058-100 | 1 | | LoRa antenna whip. |
| 3 | 915 MHz quarter-wave SMA whip | ANT-916-CW-RAH-SMA | DigiKey | LP0916-ND | 1 | ✓ | |
| 4 | M12 8-pole field-wireable connector (male, A-coded) | 1424227 | Phoenix Contact | 277-12085-ND | 4 | ✓ | Valve harness. |
| 5 | Bürkert 6011A solenoid valve, 24 V DC | 134607 | Bürkert direct | 134607 | 6 | 1 | Hydraulic block per [TRACTOR_NODE.md](TRACTOR_NODE.md). |
| 6 | Pressure transducer, 0-3000 PSI, 0.5-4.5 V | M5151-000005-03KPG | DigiKey | 480-3963-ND | 2 | 1 | Loader + boom circuits. |
| 7 | IP67 cable gland, M16, 4-7 mm cable | 19000005082 | DigiKey | 281-1832-ND | 8 | 2 | Enclosure feedthroughs. |
| 8 | DIN-rail terminal block, 4 mm² grey | 3044102 | Phoenix Contact | 277-1186-ND | 30 | 5 | |
| 9 | DIN-rail end stop | 1201442 | Phoenix Contact | 277-1146-ND | 6 | | |
| 10 | 24 V → 5 V DC-DC, 3 A, isolated | TMR 6-2411WI | DigiKey | 811-2773-5-ND | 1 | ✓ | Powers Portenta + camera. |
| 11 | Camera module, IMX477 12 MP, M12 lens | RPI-CAM-IMX477 | DigiKey | 1690-RPI-CAM-IMX477-ND | 1 | ✓ | Drives `firmware/tractor_x8/camera_service.py`. |
| 12 | M12 mount lens, 6 mm f/2.0 | LM-M12-6 | DigiKey | varies | 1 | | |

## Tier 2 — Base station (Portenta Max Carrier + Portenta X8)

| # | Item | MPN | Distributor | DPN | Qty | Spare? | Notes |
|---|------|-----|-------------|-----|-----|--------|-------|
| 13 | LoRa SX1276 breakout (same as #1) | RFM95W-915S2 | DigiKey | 1568-1656-ND | 1 | ✓ | |
| 14 | 915 MHz outdoor LPDA antenna, 8 dBi | A09-Y8NF | L-com | A09-Y8NF | 1 | | Mast-mount per [BASE_STATION.md](BASE_STATION.md). |
| 15 | 8 m LMR-400 jumper, N-male/SMA-male | CA400N-SF-X | L-com | CA400N-SF-8 | 1 | | |
| 16 | Lightning surge protector, N-F/F | AL-NFNFB | L-com | AL-NFNFB | 1 | | |
| 17 | Coral Mini PCIe Edge TPU | G650-04686-01 | Coral.ai | direct | 1 | ✓ | Optional accel; gate per [CORAL.md](CORAL.md). |
| 18 | mSATA → PCIe adapter (if Mini PCIe slot lacks Coral mode pin) | — | Mouser | varies | 0/1 | | Only if the Max Carrier slot fights with Coral. |
| 19 | NVMe → USB-C dock (for X8 Yocto image flashing) | StarTech M2E1BMU31C | DigiKey | varies | 1 | | One-time bring-up tool. |

## Tier 3 — Handheld controller

| # | Item | MPN | Distributor | DPN | Qty | Spare? | Notes |
|---|------|-----|-------------|-----|-----|--------|-------|
| 20 | LoRa breakout (same as #1) | RFM95W-915S2 | DigiKey | 1568-1656-ND | 1 | ✓ | |
| 21 | OLED 1.3" SH1106 SPI | OLED-013O-WS | DigiKey | varies | 1 | | Status display. |
| 22 | 18650 Li-ion cell, 3500 mAh, button-top | NCR18650B-BT | DigiKey | varies | 2 | 2 | One in service, one in charger. |
| 23 | TP4056 charge IC module | LP-XXXXXX | DigiKey | varies | 1 | ✓ | |
| 24 | Joystick module, dual-axis, 10 kΩ | COM-09032 | SparkFun | COM-09032 | 2 | ✓ | LH + RH. |
| 25 | E-stop pushbutton, 22 mm, NC contact | A22NE-M-01R | DigiKey | Z3413-ND | 1 | ✓ | |
| 26 | Polycarbonate enclosure, IP65, 200×120×75 | 1554Q2GY | DigiKey | HM911-ND | 1 | | |

## Hydraulic bill (excerpt — see also [BUILD-HYDRAULIC](BUILD-HYDRAULIC/))

| # | Item | MPN | Distributor | DPN | Qty | Spare? | Notes |
|---|------|-----|-------------|-----|-----|--------|-------|
| 27 | -8 JIC swivel × -8 ORB straight | 6400-08-08 | McMaster | 5346K321 | 12 | 4 | |
| 28 | -8 JIC tee | 2603-08-08-08 | McMaster | varies | 4 | 1 | |
| 29 | 1/2" hydraulic hose, 3/8 R2T 4000 PSI | 5836K23 | McMaster | 5836K23 | 25 ft | | |

## Reorder thresholds

When the spare-parts kit drops below the **Spare?** quantities listed above, reorder the affected line. The `FIELD_SERVICE.md` flowchart references this list — keep them in sync.

## See also

- [HARDWARE_BOM.md](HARDWARE_BOM.md) — Arduino-store-only items
- [FIELD_SERVICE.md](FIELD_SERVICE.md) — spare-parts policy
- [TRACTOR_NODE.md](TRACTOR_NODE.md), [BASE_STATION.md](BASE_STATION.md), [HANDHELD_REMOTE.md](HANDHELD_REMOTE.md)
