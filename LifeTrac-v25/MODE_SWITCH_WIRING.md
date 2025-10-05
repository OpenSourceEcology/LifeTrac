# LifeTrac v25 Mode Switch Wiring Guide

## Overview

The LifeTrac v25 controller supports three operating modes via a 3-position double pole double throw (DPDT) switch:
- **OFF**: No power to controller
- **MQTT**: WiFi/MQTT network control
- **BLE**: Bluetooth Low Energy direct control (default)

## Required Components

- 1x 3-position DPDT switch (ON-OFF-ON type)
- Wire for connections (22-24 AWG recommended)
- Arduino Opta controller with D1608S extension

## Switch Positions and Logic

```
Switch Position    Power    D9 Pin    D10 Pin    Mode
─────────────────────────────────────────────────────
Position 1 (OFF)    OFF     n/a       n/a        No Operation
Position 2 (MQTT)   ON      HIGH      LOW        WiFi/MQTT
Position 3 (BLE)    ON      LOW       LOW        Bluetooth
No Switch           ON      LOW*      LOW*       Bluetooth (Default)

* Internal pulldown resistors ensure LOW when not connected
```

## Wiring Diagram

### Simple Wiring (Recommended)

```
                        3-Position DPDT Switch
                    ┌─────────────────────────┐
                    │      [1]  [2]  [3]      │
                    │       │    │    │        │
                    │   OFF │ MQTT│  BLE      │
                    │       │    │    │        │
12V Supply ─────────┤   A   X────●────●        │
                    │       │    │    │        │
GND ────────────────┤   B   ●────●────●        │
                    │       │    │    │        │
                    └───────┼────┼────┼────────┘
                            │    │    │
                            │    │    └──────────────┐
                            │    │                   │
                            │    └───────────────┐   │
                            │                    │   │
                    ┌───────▼────────────────────▼───▼──────┐
                    │ Arduino Opta D1608S                   │
                    │                                        │
                    │  VIN ◄── (via switch position 2 & 3)  │
                    │  GND ◄── (common)                      │
                    │  D9  ◄── (switch position indicator A) │
                    │  D10 ◄── (switch position indicator B) │
                    └────────────────────────────────────────┘
```

### Detailed Connection Diagram

```
12V+ ────┬──────────────────────────────┐
         │                              │
         │   3-Position Switch          │
         │   Pole 1 (Power)             │
         └─► Common                     │
                │                       │
          ┌─────┼─────┬─────┐          │
          │     │     │     │          │
        [OFF] [MQTT] [BLE]  │          │
          X     │     │     │          │
               ├─────┴─────┤           │
               │           │           │
               └─── TO ────┴───────────┘
                    Opta VIN


GND ─────┬──────────────────────────────┬─────────────────┐
         │   3-Position Switch          │                 │
         │   Pole 2 (Signal)            │                 │
         └─► Common                     │                 │
                │                       │                 │
          ┌─────┼─────┬─────┐          │                 │
          │     │     │     │          │                 │
        [OFF] [MQTT] [BLE]  │          │                 │
          X     │     │     │          │                 │
               12V    │     │          │                 │
                │     │     │          │                 │
                │     └─────┴──────────┼─────────────────┘
                │                      │
                └─────── TO ───────────┘
                         Opta D9


        Opta D10 ◄───────── GND (always LOW, unused as position indicator)
```

## Switch Configuration Details

### Position 1: OFF
- **Power Switch Pole**: Open circuit (no connection)
- **Signal Switch Pole**: Open circuit (no connection)
- **Result**: No power to Arduino Opta - complete shutdown

### Position 2: MQTT Mode
- **Power Switch Pole**: Connects 12V+ to Opta VIN
- **Signal Switch Pole**: Connects 12V+ to D9
- **D9**: HIGH (12V)
- **D10**: LOW (internal pulldown)
- **Result**: Controller boots in MQTT mode

### Position 3: BLE Mode
- **Power Switch Pole**: Connects 12V+ to Opta VIN
- **Signal Switch Pole**: Not connected (D9 remains LOW)
- **D9**: LOW (internal pulldown)
- **D10**: LOW (internal pulldown)
- **Result**: Controller boots in BLE mode

### No Switch Installed (Default)
- **VIN**: Connected directly to 12V+
- **D9**: LOW (internal pulldown resistor)
- **D10**: LOW (internal pulldown resistor)
- **Result**: Controller boots in BLE mode (default)

## Bill of Materials

| Component | Description | Quantity | Notes |
|-----------|-------------|----------|-------|
| DPDT Switch | 3-position, ON-OFF-ON, 12V 10A rated | 1 | Center position is OFF |
| Wire | 22-24 AWG stranded, multiple colors | ~3 feet | Red for 12V, Black for GND, Yellow for D9 |
| Terminal Block | Optional, for secure connections | 2 | Makes installation cleaner |
| Switch Mounting | Panel mount hardware | 1 set | Depends on enclosure |

## Recommended Switch Models

- **Heavy Duty**: Toggle switch, DPDT 3-position (ON-OFF-ON)
  - Part example: Carling Technologies LT Series
  - Rating: 20A @ 12VDC
  
- **Compact**: Miniature DPDT switch 3-position
  - Part example: C&K Components 7101 Series
  - Rating: 6A @ 12VDC
  
- **Industrial**: Selector switch with positions labeled
  - Part example: Eaton M22 Series
  - Rating: 10A @ 12VDC
  - Features: Labeled positions, panel mount

## Installation Steps

### Step 1: Prepare Switch
1. Mount switch in accessible location on enclosure
2. Label positions: OFF, MQTT, BLE
3. Ensure switch is in OFF position

### Step 2: Wire Power Path
1. Connect 12V+ supply to switch common terminal (Pole 1)
2. Connect switch Pole 1 Position 2 (MQTT) to Opta VIN
3. Connect switch Pole 1 Position 3 (BLE) to Opta VIN
4. Leave Position 1 (OFF) unconnected
5. Connect GND directly to Opta GND

### Step 3: Wire Signal Path
1. Connect switch common terminal (Pole 2) to GND
2. Connect switch Pole 2 Position 2 (MQTT) to 12V+ (via resistor if desired)
3. Wire from switch Pole 2 Position 2 to Opta D9
4. Leave Position 3 (BLE) unconnected on Pole 2
5. Leave Opta D10 unconnected (uses internal pulldown)

### Step 4: Test Connections
1. Use multimeter to verify:
   - Position 1 (OFF): No voltage at Opta VIN
   - Position 2 (MQTT): 12V at Opta VIN, 12V at D9
   - Position 3 (BLE): 12V at Opta VIN, 0V at D9
2. Verify GND continuity to Opta

### Step 5: Upload Firmware
1. Connect Arduino Opta via USB
2. Upload lifetrac_v25_controller.ino firmware
3. Verify firmware includes mode switch support (v25+)

### Step 6: Verify Operation
1. Set switch to BLE position
2. Open Serial Monitor (115200 baud)
3. You should see: "Mode: BLE (Default)"
4. Set switch to MQTT position
5. Power cycle, should see: "Mode: MQTT"

## Simplified 2-Wire Method (Alternative)

For a simpler installation, you can use just the power switching:

```
12V+ ──► Switch Position 2 & 3 ──► Opta VIN
         (Position 1 = OFF)

D9 ◄──── Direct 12V connection (permanent HIGH for MQTT only)
D10 ◄─── Leave disconnected (LOW via pulldown)
```

In this method:
- Position 1: OFF (no power)
- Position 2 & 3: ON (always MQTT mode since D9 is HIGH)
- No BLE mode available
- Simpler wiring, less flexibility

## Troubleshooting

### Controller Always in BLE Mode
- Check D9 connection to switch
- Verify 12V present at D9 in MQTT position
- Check switch is actually changing positions

### Controller Always in MQTT Mode
- Check D9 isn't shorted to 12V
- Verify switch is properly wired
- Test D9 voltage in BLE position (should be ~0V)

### No Power in Any Position
- Check 12V supply
- Verify switch pole 1 connections
- Test switch continuity with multimeter

### Power Works But Mode Doesn't Change
- Verify D9 connection
- Check firmware version (needs v25+)
- Open Serial Monitor to see mode detection

## Safety Notes

1. **Power Off First**: Always disconnect 12V before wiring
2. **Double Check Polarity**: Wrong polarity can damage Arduino Opta
3. **Secure Connections**: Use terminal blocks or solder connections
4. **Strain Relief**: Use cable ties to prevent wire stress
5. **Test Before Installing**: Verify all connections with multimeter
6. **Fuse Protection**: Add inline fuse (5A) to 12V supply
7. **Weather Protection**: Use weatherproof switch for outdoor use

## Alternative: Software Mode Selection

If you prefer not to install a hardware switch, the controller defaults to BLE mode. You can:

1. **Flash different firmware** for MQTT-only mode
2. **Use serial commands** to change modes (requires firmware modification)
3. **Leave D9 disconnected** for permanent BLE mode (default)
4. **Connect D9 to 12V** for permanent MQTT mode

## Appendix: Mode Detection Code

The Arduino Opta firmware detects mode using this logic:

```cpp
// Read mode switch pins with internal pulldown
pinMode(MODE_SWITCH_PIN_A, INPUT_PULLDOWN);  // D9
pinMode(MODE_SWITCH_PIN_B, INPUT_PULLDOWN);  // D10

bool pinA = digitalRead(MODE_SWITCH_PIN_A);   // D9
bool pinB = digitalRead(MODE_SWITCH_PIN_B);   // D10

if (pinA == HIGH && pinB == LOW) {
  currentMode = MODE_MQTT;  // MQTT mode
} else {
  currentMode = MODE_BLE;   // BLE mode (default)
}
```

This ensures:
- **No switch**: Both LOW (pulldown) → BLE mode
- **BLE position**: Both LOW → BLE mode  
- **MQTT position**: D9 HIGH, D10 LOW → MQTT mode

---

For more information, see:
- [INSTALLATION_GUIDE.md](INSTALLATION_GUIDE.md) - Complete setup guide
- [WIRING_DIAGRAM.md](WIRING_DIAGRAM.md) - Full system wiring
- [DROIDPAD_BLE_SETUP.md](DROIDPAD_BLE_SETUP.md) - BLE control setup
