# Field Service

> Per [TODO.md § Operations](TODO.md). Diagnostic flowcharts, fuse map, common failure modes, and the spare-parts kit contents. Aimed at a field technician with basic 12/24 V troubleshooting skills, **not** the original engineer.

## Spare-parts kit (one per machine)

Stocked in a Pelican 1500 case. The list mirrors items flagged "Spare?" in [NON_ARDUINO_BOM.md](NON_ARDUINO_BOM.md):

| Slot | Item | Qty |
|------|------|-----|
| A1   | RFM95W-915S2 LoRa module | 2 |
| A2   | 915 MHz quarter-wave whip | 1 |
| A3   | u.FL → SMA pigtail | 1 |
| B1   | M12 8-pole connector kit (male + female) | 4 ea |
| B2   | DC-DC TMR 6-2411WI | 1 |
| B3   | Pressure transducer 0-3000 PSI | 1 |
| C1   | DIN terminal blocks 4 mm² | 5 |
| C2   | Cable glands M16 | 2 |
| D1   | Bürkert 6011A solenoid valve | 1 |
| D2   | E-stop pushbutton, 22 mm | 1 |
| D3   | Joystick module COM-09032 | 1 |
| D4   | TP4056 charge module | 1 |
| E1   | 18650 cell NCR18650B-BT (charged) | 2 |
| E2   | Camera IMX477 module | 1 |
| F1   | -8 JIC fittings (assorted) | 4 |
| F2   | -8 hose tee | 1 |

**Reorder rule:** if a slot drops below the listed quantity, file a procurement request the same shift. Don't let the kit deplete before reordering — lead times on Coral / Bürkert / Phoenix Contact items can run weeks.

## Fuse map

| Fuse # | Rating | Protects | Location |
|--------|--------|----------|----------|
| F1 | 30 A blade | Main 12 V → 24 V boost converter | Battery box, rear |
| F2 | 10 A blade | Tractor controller (Portenta H747 + Max Carrier) | Tractor enclosure DIN, top |
| F3 | 7.5 A blade | Camera + IR illuminator | Tractor enclosure DIN, top |
| F4 | 15 A blade | Solenoid valve bank (24 V) | Hydraulic block enclosure |
| F5 | 5 A blade | Pressure-transducer 5 V rail | Tractor enclosure DIN, bottom |
| F6 | 2 A glass | Base station Portenta + X8 (5 V via DC-DC) | Base enclosure |
| F7 | 1 A glass | Base station LoRa SX1276 (3.3 V) | Base enclosure |
| F8 | (none — TP4056 has internal protection) | Handheld 18650 cell |  — |

When F4 blows, it's almost always a shorted solenoid coil. Disconnect each valve in turn, swap with the spare from kit slot D1, and re-fuse.

## Diagnostic flowcharts

### Symptom: handheld won't pair with base

```
        [Handheld OLED says "PAIR FAIL"]
                    │
        ┌───────────┴───────────┐
        │ USB-CDC enumerates?   │
        └───────────┬───────────┘
                    │ no
                    ├──► swap USB cable; if still no, swap handheld (kit D3 won't help — escalate)
                    │ yes
                    ▼
        ┌───────────────────────┐
        │ tools/pair_handheld.py│
        │ exits 0?              │
        └───────────┬───────────┘
                    │ no, exit 4 "rejected"
                    ├──► firmware version mismatch — flash matching FW per FIRMWARE_UPDATES.md
                    │ no, exit 3 "cannot open"
                    ├──► check operator ACL on /dev/ttyACM0 (dialout group)
                    │ yes
                    ▼
        ┌───────────────────────┐
        │ Audit log shows       │
        │ "handheld_paired"?    │
        └───────────┬───────────┘
                    │ no
                    └──► /var/lib/lifetrac mount missing — remount and re-run
                    │ yes
                    ▼
                  [DONE]
```

### Symptom: tractor video drops to "WIREFRAME" badge and stays there

1. Open the base UI status pill (bottom-right). Is the LoRa link RSSI < -110 dBm?
   - Yes → mast antenna alignment problem; rotate or raise mast.
   - No → continue.
2. Is `link_monitor` reporting 100 % airtime utilisation in the last 10 s?
   - Yes → another transmitter on 915 MHz is jamming. Check for nearby unlicensed devices; consider FHSS hop with a different `key_id`.
   - No → continue.
3. Is the encode-mode ladder stuck at WIREFRAME despite the link recovering?
   - Yes → restart `link_monitor` process; check `EncodeModeController.cooldown_ms` config.

### Symptom: solenoid clicks but no boom motion

1. Engine running? Reservoir at temp? If no, fix that first.
2. Pressure reading on UI ≈ 2500 PSI when commanded? If 0, F4 fuse blown or pump failure.
3. Solenoid coil ohming out at 6-8 Ω? If open, swap from kit slot D1.
4. Spool sticking — flush or replace valve.

## Escalation

Anything not on the flowcharts: page the on-call engineer with:

- machine serial, hour-meter reading
- last working timestamp from `audit_log.jsonl`
- last 200 lines of `journalctl -u lifetrac-bridge`
- last 5 minutes of MQTT topic `lifetrac/v25/telemetry/sensor_faults`

## See also

- [CALIBRATION.md](CALIBRATION.md) — when a sensor reads weird before it goes hard-fault
- [NON_ARDUINO_BOM.md](NON_ARDUINO_BOM.md) — vendor / part numbers for any of the kit items
- [FIRMWARE_UPDATES.md](FIRMWARE_UPDATES.md)
