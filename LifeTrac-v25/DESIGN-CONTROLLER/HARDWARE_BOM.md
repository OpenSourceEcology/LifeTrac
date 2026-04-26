# Hardware Bill of Materials — Controller System

All prices in USD, approximate, from primary distributor (Arduino Store, SparkFun, DigiKey, Mouser). Verify availability and regional frequency compliance (915 MHz parts shown; substitute 868 MHz for EU).

## Tier 1 — Tractor Node (Portenta Max Carrier + Portenta X8 + Arduino Opta)

| Part | Qty | Unit | Subtotal | Source | Notes |
|---|---:|---:|---:|---|---|
| Arduino Portenta Max Carrier (ABX00043) | 1 | $335 | $335 | [Arduino Store](https://store-usa.arduino.cc/products/portenta-max-carrier) | Carrier with onboard LoRa + Cat-M1, both SMA |
| **Arduino Portenta X8 (ABX00049)** | 1 | $200 | $200 | [Arduino Store](https://store-usa.arduino.cc/products/portenta-x8) | Linux on i.MX 8M Mini quad Cortex-A53 + **STM32H747 co-MCU**. The H747 co-MCU runs the same M7+M4 firmware as a standalone H7 — we get H7 capability *and* Linux on one board. Also enables MIPI camera and SSH field-debug. **Fallback: substitute Portenta H7 (ABX00042) at $115 — saves $85, loses Linux** (see notes below) |
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
| GPS active patch antenna, SMA, 28 dB LNA | 1 | $20 | $20 | DigiKey (Taoglas / U-blox) | Cab-roof antenna for the GPS module below; connects via U.FL→SMA pigtail |
| Cab-roof antenna mounting bracket + N-bulkhead | 1 | $25 | $25 | L-com / DPD | Ground plane + weather-sealed pass-through for LoRa whip |
| Status LED panel (3 ×, panel-mount, 12 V) | 1 set | $10 | $10 | DigiKey | POWER / LINK / FAULT exterior indicators |
| Piezo buzzer, 12 V panel-mount | 1 | $6 | $6 | DigiKey | Audible alarm + take-control transition cue |
| Deutsch DT connector kit (4/6/8/12 way) + crimp tool rental | 1 | $80 | $80 | Ladd / DigiKey | Weatherproof harness terminations |
| IP65 enclosure, ~250×200×100 mm | 1 | $45 | $45 | Polycase / McMaster | Bigger than original; needs to fit the relay/driver/safety boards |
| Vibration-isolating enclosure mounts (rubber bobbins, ×4) | 1 set | $12 | $12 | McMaster | Protects PCB solder joints |
| Conformal coating spray (acrylic) | 1 | $15 | $15 | MG Chemicals | Humidity protection for all PCBs |
| Cable glands, M16/M20, IP68 | 6 | $3 | $18 | McMaster | Antenna + power + valve harness + telemetry pass-throughs |
| **Adafruit MCP2221A breakout (4471)** — USB ↔ I²C bridge with Stemma QT/Qwiic | 1 | $8 | $8 | [DigiKey 4471](https://www.digikey.com/en/products/detail/adafruit-industries-llc/4471/11497510) | Hosts the IMU on the **X8 Linux side** (mainline `hid-mcp2221` driver, no install). USB-C to X8 host port; Qwiic JST-SH 4-pin to BNO086 board. See note below. |
| **SparkFun Qwiic 9DoF IMU — BNO086 (DEV-22857)** | 1 | $30 | $30 | [SparkFun DEV-22857](https://www.sparkfun.com/products/22857) | On-chip sensor fusion → quaternion straight out, no Madgwick/Mahony work. Used for tip-over warning, heading hold, vibration logging |
| Qwiic / Stemma QT 4-pin cable, 50 mm | 2 | $1 | $2 | SparkFun / Adafruit | MCP2221A → BNO086, BNO086 → GPS (Qwiic daisy-chains, no hub needed) |
| **SparkFun GPS Breakout — NEO-M9N, SMA Qwiic (GPS-15733)** | 1 | $75 | $75 | [SparkFun GPS-15733](https://www.sparkfun.com/sparkfun-gps-breakout-neo-m9n-sma-qwiic.html) | u-blox NEO-M9N: 4-constellation GNSS (GPS/GLONASS/Galileo/BeiDou), 25 Hz max, I²C @ 0x42 on the same Qwiic chain as the IMU. **Onboard SMA connector** — antenna feedline runs straight to the cab-roof patch antenna through a panel-mount SMA bulkhead, no fragile U.FL pigtail inside the enclosure. ~1.5 m typical accuracy without RTK. Adafruit-supported `adafruit-circuitpython-gps` *or* SparkFun `sparkfun-ublox-gps` libs |
| SMA female-female bulkhead, panel-mount, IP67 | 1 | $5 | $5 | DigiKey | Brings the GPS antenna feedline through the IP65 enclosure wall; mates with the cab-roof patch antenna's SMA male connector |
| **Hydraulic pressure sensor**, 0–250 bar, 4–20 mA, M12-A 4-pin | 2 | $40 | $80 | DigiKey (Wika / IFM) | Wires into Opta A0602 inputs (`hyd_supply_psi` / `hyd_return_psi`) |
| M12-A 4-pin cordset, 2 m, female straight | 2 | $12 | $24 | TURCK / Phoenix Contact | For the pressure sensors; preassembled (don't field-terminate M12) |
| M12-A 5-pin cordset, 2 m, for RS-485 | 2 | $14 | $28 | TURCK / Phoenix Contact | Master ↔ Opta RS-485 (one each end if using M12 panel jacks) |
| **USB UVC webcam** — see *Camera path* note below for OSHW-aligned options. Recommended: **Kurokesu C1 PRO** (USB UVC, schematics + V4L2 drivers public on [github.com/Kurokesu](https://github.com/Kurokesu), M12 swappable lens). Acceptable substitutes: Logitech C920 (cheaper, fully closed) or ELP-USBFHD06H (M12/IP67) | 1 | $120 | $120 | [Kurokesu](https://www.kurokesu.com/) / Amazon / DigiKey | First-light video. UVC = mainline `uvcvideo` driver, zero install on the X8 Yocto image. Industrial M12 lens = swappable wide/tele/IR-cut |
| Panel-mount USB-A bulkhead + cable gland | 1 | $12 | $12 | DigiKey | Brings the webcam cable through the IP65 enclosure wall (consumer USB cables are not field-sealed) |
| Misc connectors, ferrules, heat shrink, wire | — | — | $40 | — | |
| **Tier 1 Subtotal** | | | **~$1,731** + $5/mo | | Standardized on Portenta X8 (matches base station). Adopts Arduino Opta + expansions in place of discrete relay/DAC/opto/conditioning boards. See [TRACTOR_NODE.md § Why Opta as the valve controller](TRACTOR_NODE.md#why-opta-as-the-valve-controller) |

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
| **Single deployed system** (1 tractor + 1 base + 1 handheld) | **~$2,786** | **$10/mo** (2× SIM cards) |
| **Plus development gear** (one-time) | $1,914 | |
| **Plus spare parts** (recommended) | $3,400 | |

> Tier 1 was revised upward from the initial $634 estimate after auditing against
> [DESIGN-HYDRAULIC/HYDRAULIC_DIAGRAM.md](../DESIGN-HYDRAULIC/HYDRAULIC_DIAGRAM.md):
> the directional valves are 12 V (not 24 V), but a realistic deployment also
> needs the safety relay, opto-isolation, fuses, EMI filtering, harness connectors,
> 0–10 V DAC for the Burkert 8605 flow valve, and external status indicators.

## Notes on substitutions

- **Portenta X8 vs H7 on the tractor (primary vs fallback):** X8 is now the **primary** choice on both the tractor and the base station for SKU consistency. The X8 contains a full STM32H747 as a co-MCU, so the M7+M4 valve/radio firmware is binary-compatible with a standalone H7 — no firmware changes needed. The X8 *also* gives you Linux + Docker on the tractor for free, which enables: SSH-in field debug, journaling/log rotation that survives reboots, future MIPI camera integration, container-based telemetry recorder, and one Yocto image to maintain across both Max Carrier nodes. **Fallback to H7 (ABX00042, $115)** is acceptable if (a) X8 is out of stock, (b) cost is critical and you don't need Linux on the tractor, or (c) you want to minimize attack surface (bare-metal only). The fallback path is a pure module swap — same connectors, same firmware. Update [TRACTOR_NODE.md](TRACTOR_NODE.md) hardware table accordingly when running fallback.
- **GPS path — Qwiic vs UART, and RTK upgrade:** the Max Carrier has **no onboard GNSS** (cellular Mini PCIe slot + CAN + Ethernet only), so the v25 BOM adds a discrete receiver. Primary choice is the **SparkFun NEO-M9N Qwiic (GPS-15712, ~$75)** because it daisy-chains on the *same* Qwiic bus as the BNO086 IMU — one MCP2221A bridge handles both, owned entirely by the X8 Linux side, no M7 work. Library: `adafruit-circuitpython-gps` (NMEA over I²C) or SparkFun's u-blox UBX library if we want raw binary at higher rates.
  - **Alternative — U.FL variant** (SparkFun GPS-15712, also ~$75): same NEO-M9N silicon but with a U.FL connector that requires a U.FL→SMA pigtail (~$3) inside the enclosure. Pick this only if board height is tight or you already have U.FL pigtails on hand.
  - **RTK upgrade path** (Phase 3 / autonomy): swap to **SparkFun ZED-F9P RTK Qwiic (GPS-RTK-SMA, ~$275)** + a base station with the same module (or NTRIP-fed RTCM3 corrections). Centimeter accuracy, drop-in firmware change — same I²C address space, just enable the RTK message set. Track in [TODO.md Autonomy section](TODO.md#future--autonomy-phase-on-hold).
  - **Antenna mounting**: the existing $20 active SMA patch antenna stays — with the **SMA-variant GPS board** the antenna feedline runs straight from the board's onboard SMA jack to a panel-mount SMA bulkhead in the enclosure wall (then to the cab-roof patch antenna). No U.FL pigtail inside the enclosure means one less fragile micro-coax connector to worry about during install/service. Mount the patch antenna on the cab roof with at least 5 cm clear sky in all directions and >30 cm from the LoRa whip to avoid intermod.
- **IMU path — USB→Qwiic vs native I²C:** the BOM lists the **MCP2221A USB-to-I²C bridge (Adafruit 4471) + BNO086 Qwiic** as the primary IMU path. Reasoning: (a) the IMU is owned by the **X8 Linux side**, not the M7, since tip-over warning / heading hold / vibration logging are not sub-millisecond critical and the M7 is already loaded with the 50 Hz Modbus + LoRa work; (b) MCP2221A's `hid-mcp2221` driver has been mainline since Linux 5.10, so a stock Yocto build enumerates it as `/dev/i2c-N` with no driver install; (c) Qwiic JST-SH cabling is solderless and the **NEO-M9N GPS daisy-chains on the same bus** — one bridge, two sensors, zero extra USB ports. **Alternative — native I²C:** wire the BNO086 (or BNO055/ICM-20948) directly to the Max Carrier's I²C breakout pins and poll from the M7. Saves $9 and one USB port, costs you M7 loop-time and a soldering job. Choose native I²C only if you find the IMU pipeline needs sub-millisecond determinism — none of the v25 use cases do. **Industrial-environment caveat:** for the production build inside a noisy enclosure, add a USB galvanic isolator (ADuM3160, ~$25) inline or move to native I²C.
- **Camera path — USB UVC vs MIPI vs PoE/IP, and the OSHW question:** BOM lists USB UVC as the primary camera. UVC is a mainline kernel driver (`uvcvideo`) — no install, just `/dev/video0`.
  - **OSHW-aligned recommendation — Kurokesu C1 PRO (~$120):** schematics, V4L2 kernel drivers, 3D models and SDKs are all published on [github.com/Kurokesu](https://github.com/Kurokesu) under GPL-2/3 / BSD-2-Clause. M12 lens mount means we can pick wide-angle, telephoto, or IR-cut without changing the camera body. Standard UVC, so no driver work on the X8.
  - **Boxed-body upgrade — Kurokesu C2-290C (~$240):** Sony IMX290 Starvis low-light sensor + **hardware H.264 encoder** + boxed 33×33 mm aluminum body + lock-screw USB-C cable. Drops X8 ffmpeg load to near-zero on the LTE-preview path. Pick this when running at dusk/dawn or when the X8 needs CPU headroom for crop-health onboard analysis (see below). Pair with the Kurokesu **L169-FZA-2.8Z12-CS varifocal CS lens** (€90, 2.8–12 mm, 130°→39° FOV) for the cab-forward camera.
  - **OSHW DIY alternative (~$50):** **Raspberry Pi Zero 2 W + Camera Module 3 + `uvc-gadget`** — the Pi Zero turns into a USB UVC webcam in software (the kernel `g_webcam` gadget driver wraps libcamera). The Pi 3D-printed mount, board files (in part), and *all* application software is open. Tradeoff: only the Broadcom SoC silicon stays closed, and you maintain a second Linux box on the tractor. Track as a Phase-2 swap.
  - **OpenMV Cam H7 Plus (~$80):** the *most* OSHW-licensed option (CERN-OHL-S hardware, MIT firmware). Has a UVC firmware mode but the resolution and lens quality are below what we want for a tractor cab camera. Keep in mind for sub-cameras (e.g. implement-monitor cam).
  - **Acceptable closed-hardware fallbacks if the above are unavailable:** Logitech C920 (~$70, US-stocked, well-tested under Linux UVC), ELP-USBFHD06H (~$60, M12 + IP67 housed). These are *substitutes for budget/availability only* — they keep us off the OSHW path on the camera line item.
  - **MIPI CSI** (e.g. Arducam IMX219, ~$30) gives lower latency but needs a Yocto device-tree overlay and is X8-only — track in [RESEARCH-CONTROLLER/VIDEO_COMPRESSION/](RESEARCH-CONTROLLER/VIDEO_COMPRESSION/README.md) as a Phase 2 swap.
  - **PoE IP camera** (Reolink RLC-510A, ~$70) is a third option if night vision / IR / single-cable PoE is wanted; pulled by FFmpeg/GStreamer over RTSP, also no driver install.

- **Multi-camera deployment (front + rear + optional implement):** v25 supports up to 4 USB cameras on the X8 (one root hub, ~480 Mbit/s shared — fine at MJPEG 720p × 4 or H.264 1080p × 4 on the C2). Camera switching is driven by `CMD_CAMERA_SELECT` over LoRa (see [LORA_PROTOCOL.md § Command frame opcodes](LORA_PROTOCOL.md#command-frame-opcodes)). The tractor X8 multiplexes — only the *selected* camera's thumbnails go to LoRa; the other feeds stay local on the tractor for log replay or LTE preview. Recommended layout:
  - **Front (primary):** Kurokesu C2-290C boxed + 2.8–12 mm varifocal CS, mounted in cab through the windshield, or in a NEMA 4X enclosure on the cab roof.
  - **Rear (reverse):** second Kurokesu C2-290C boxed (or C1 PRO board $120) + fixed wide CS lens (~$20, 3.6 mm), mounted at the rear ROPS bar pointed back. Auto-selects when the joystick is held in reverse for >1 s (default `CMD_CAMERA_SELECT 0x00 auto-by-mode`).
  - **Implement (optional):** a small board cam (C1 PRO or OpenMV) at the boom or three-point-hitch, watching the bucket / ripper. Useful for hitching and bucket-fill verification. Manually selected via the UI.
  - **Crop-health (optional Phase-3):** dedicated **dual-band NoIR camera** with 850 nm filter swap or a **MAPIR Survey3W NDVI / OCN** for proper red-edge / NIR. Onboard NDVI / GNDVI / canopy-cover computation runs on the X8 (see [VIDEO_OPTIONS.md § Crop-health analysis](VIDEO_OPTIONS.md#crop-health-analysis)) and a 30 B summary goes over LoRa on `topic 0x24`.

- **Camera enclosure path — cab-mounted vs NEMA 4X external box:** two viable paths, pick per deployment.
  - **Path A (default for cab tractors): mount inside the cab, point through the windshield.** Free waterproofing — cab is already weather-sealed. Cheapest. Limited mounting flexibility, glare/reflection at night (mitigate with polarizer or anti-reflective sticker). C1 PRO board or boxed C2-290C both fine.
  - **Path B (open-cab / external / multi-cam): Wiegmann WA-series NEMA 4X fiberglass enclosure with clear polycarbonate window** (e.g. Grainger 52XA84, ~$60–95). Mount the camera + lens on the included aluminum backplate; single M20 cable gland for USB-C; **mandatory M12 Gore-Tex vent gland** ($8) to prevent solar greenhouse effect; black-flock paper inside ($15) to kill internal reflections. Total adder ~$100 per camera. Lets you mount cameras anywhere (cab roof, fender, rear ROPS, mast). Switches the camera selection back toward **board-level Kurokesu C2 M12 (~$160)** since the boxed metal body becomes redundant inside the fiberglass shell. With Path B, the IP-rated ELP-USBFHD06H ($60) also becomes unnecessary — the enclosure is the IP rating.
  - For mixed fleets, **standardize on Path B** so a single mounting bracket + enclosure + cable gland kit works for any camera position. Path A is the single-tractor / single-camera quick-start.
- **Base station X8 is *not* optional** — you need Linux there for the web UI containers.
- **MKR WAN 1310 vs MKR WAN 1300:** the 1310 adds a battery charger and faster processor; spec is otherwise identical. Always buy the 1310.
- **Mast height:** for sites where 8 dBi omni isn't enough range, swap to a 12 dBi Yagi pointed at the typical work area. Yagi gives ~15 dB more gain in one direction; total link budget improves by ~7 dB after antenna match losses.
- **Cellular SIM:** any Cat-M1 IoT SIM works. Hologram, Soracom, Twilio Super SIM all have global coverage.
- **915 MHz vs 868 MHz:** for EU deployments, swap to MKR WAN 1300 (868 MHz variant) and confirm the Max Carrier's Murata module is the 868 MHz variant at order time.

## Lead times (typical)

- Arduino Pro hardware: 2–4 weeks (often in stock at distributors)
- Mast antenna + LMR-400 cable: 1 week (L-com US stock)
- Custom PCB (handheld joystick board): 2 weeks via OSHPark / JLCPCB
- IP65 enclosures: 1 week
