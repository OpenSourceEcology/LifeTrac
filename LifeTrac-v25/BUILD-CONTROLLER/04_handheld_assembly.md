# 4. Handheld Assembly *(Optional)*

> **Heads up!** The handheld is **optional** for v25 — the base-station web UI can drive the tractor by itself. Build the handheld only after the base + tractor path is bench-proven, so you have a known-good radio link to compare against.

The handheld is an MKR WAN 1310 in an IP54 plastic enclosure with two analog joysticks, a 22 mm latching E-stop, four buttons, and an OLED status screen. It runs on a 1S LiPo and charges over USB-C.

## What You're Building

```
   ┌─────────── IP54 handheld enclosure (~120×80×40) ──────────┐
   │                                                            │
   │    ┌───────────┐                          ┌──────────┐    │
   │    │ Joystick  │                          │ Joystick │    │
   │    │   left    │      ┌──────────┐        │   right  │    │
   │    │  (drive)  │      │  OLED    │        │ (boom +  │    │
   │    │           │      │ 128×64   │        │  bucket) │    │
   │    └───────────┘      └──────────┘        └──────────┘    │
   │                                                            │
   │   [BTN1] [BTN2]   [22 mm RED MUSHROOM E-STOP]   [BTN3] [BTN4]
   │                                                            │
   │              ┌──────────────────────────────┐              │
   │              │  MKR WAN 1310 + custom PCB   │              │
   │              └──────────────────────────────┘              │
   │                                                            │
   │   USB-C ───── 1S LiPo + TP4056 charger                     │
   └────────────────────────────────────────────────────────────┘
```

## Required Materials

From Tier 3 BOM ([01_bill_of_materials.md](01_bill_of_materials.md)).

You also need a **custom PCB** that breaks out the joystick connectors, button matrix, OLED I²C, E-stop signal, and charging circuit to one MKR WAN 1310 socket. The KiCad files live in `/handheld/pcb/` (TBD — first revision is stripboard for prototyping). Order from [OSH Park](https://oshpark.com/) or [JLCPCB](https://jlcpcb.com/) once the schematic is settled.

## Step 1 — Drill the Enclosure

Lay out and drill the front panel for:

- 2× joystick clearance holes (typically Ø 21 mm with 4× M3 mount holes).
- 1× 22 mm hole for the E-stop.
- 4× Ø 12 mm holes for the buttons.
- 1× rectangular cutout (~25×16 mm) for the OLED window (or use a transparent overlay sticker).
- 1× USB-C panel-mount cutout on the bottom edge.

> **Pro tip:** Print the panel layout on paper at 1:1 first and tape it to the enclosure. Measure twice, drill once. The E-stop hole is the hardest to fix if you put it in the wrong spot.

## Step 2 — Solder the PCB

If you're using the custom PCB:

1. Solder the **MKR WAN 1310 sockets** (two 14-pin female headers, 2.54 mm pitch).
2. Solder the **joystick connectors** (4-pin JST-XH or pin headers).
3. Solder the **button connectors** + the **E-stop terminals** (screw terminals are easiest for the E-stop).
4. Solder the **OLED I²C connector** (4-pin: VCC / GND / SDA / SCL).
5. Solder the **TP4056 module** (or use a separate breakout).
6. Solder the **USB-C breakout** for the charging input.

Plug the MKR WAN 1310 in last, after smoke-testing the bare PCB on the bench supply.

## Step 3 — Wire the E-Stop

The E-stop is the only safety-critical signal on the handheld. It must work even if firmware crashes.

1. Wire the **NC contacts** of the latching mushroom switch in series between the LiPo and the MKR WAN 1310's `Vin` pin.
2. When the mushroom is **pressed** the contacts open → the MKR loses power → no LoRa packets → the tractor times out within 500 ms and goes neutral.
3. **Twist the mushroom** to release latch → power restored → MKR boots → LoRa link re-established.

> **Pro tip:** The "E-stop kills MCU power" pattern is more robust than "E-stop sets a software flag." A locked-up firmware can't ignore a missing power rail.

Some builders prefer to *also* route the E-stop to a digital input so the MCU can announce the press to the base UI before going dark. That's optional — wire it to D2 with a pull-up if you want.

## Step 4 — Mount the Front-Panel Components

1. Joysticks → 4× M3 screws each, from the inside.
2. E-stop → press fit, threaded ring on the inside.
3. Buttons → press fit, threaded ring on the inside.
4. OLED → hot-glue dabs in the corners, or 3D-print a snap-in bezel.
5. USB-C panel-mount → from the outside with the captive nut on the inside.

## Step 5 — Final Wiring and Battery

1. Run flying leads from each panel component to the PCB connectors. Use ~10 cm of slack so the lid can hinge open without tugging on solder joints.
2. Plug in the **1S LiPo** to the TP4056 input.
3. Verify on a bench:
   - With E-stop released, MCU powers up; OLED initializes.
   - With E-stop pressed, MCU powers off within ~50 ms (LiPo capacitor discharge).
   - Plug USB-C → TP4056 charge LED comes on red, switches to blue when full.

## Step 6 — Antenna

Mount the 1/4-wave 915 MHz whip externally on the top of the enclosure with a u.FL → SMA pigtail to the MKR's u.FL connector. **Don't operate the radio without an antenna** — the PA can be damaged by a permanent VSWR mismatch.

## Acceptance Test

- [ ] Handheld powers on with E-stop released.
- [ ] Handheld powers off within 200 ms of pressing E-stop.
- [ ] All four buttons read distinctly on the MKR's GPIO.
- [ ] Both joysticks read full-range on their analog channels with deadband centered.
- [ ] OLED displays a "boot" message.
- [ ] LiPo charges from USB-C; charge LED behavior matches TP4056 datasheet.

## Next Step

[**5. Firmware & Software Installation →**](05_firmware_installation.md).
