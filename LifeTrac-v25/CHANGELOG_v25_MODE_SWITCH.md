# LifeTrac v25 - Mode Switch Feature Changelog

## Version 25 - Mode Switch Implementation

**Date:** December 2024  
**Feature:** MQTT/OFF/BLE Mode Selection Switch

### Summary

Added support for a 3-position hardware switch that allows users to select between:
1. **OFF mode** - Complete power cutoff (hardware switch disconnects 12V)
2. **MQTT mode** - Traditional WiFi/MQTT control via broker
3. **BLE mode** - Direct Bluetooth Low Energy control from DroidPad app (default)

### Key Features

- **Default BLE Mode**: If no switch is installed, controller defaults to BLE mode
- **Hardware Power Cutoff**: OFF position physically disconnects 12V power
- **Automatic Mode Detection**: Controller reads switch position on startup
- **DroidPad Direct Control**: BLE mode enables direct connection without WiFi/broker
- **Backward Compatible**: Existing MQTT functionality preserved

### Hardware Changes

#### Pin Assignments
- **D9 (MODE_SWITCH_PIN_A)**: First switch position pin
- **D10 (MODE_SWITCH_PIN_B)**: Second switch position pin (unused but available for future expansion)

#### Switch Logic
```
D9=LOW,  D10=LOW  -> BLE mode (default when switch not installed)
D9=HIGH, D10=LOW  -> MQTT mode
OFF position      -> Hardware power cutoff (no power to controller)
```

#### Recommended Switch
- 3-position DPDT switch (ON-OFF-ON type)
- Pole 1: Controls 12V power to Arduino Opta VIN
- Pole 2: Controls D9 signal pin for mode detection
- Rating: 10A @ 12VDC minimum

### Software Changes

#### Modified Files

1. **lifetrac_v25_controller.ino**
   - Added `#include <ArduinoBLE.h>` for BLE support
   - Added mode switch pin definitions (D9, D10)
   - Added `ControlMode` enum (MODE_BLE, MODE_MQTT)
   - Added BLE service and characteristic definitions
   - Modified `setup()` to read mode switch and initialize appropriate mode
   - Modified `loop()` to handle both MQTT and BLE control paths
   - Added new functions:
     - `readModeSwitch()` - Detects switch position
     - `setupBLE()` - Initializes BLE service
     - `readBLEJoystickData()` - Reads joystick data from BLE characteristics

2. **arduino_libraries.txt**
   - Added ArduinoBLE library requirement

3. **INSTALLATION_GUIDE.md**
   - Added mode switch hardware connections
   - Added mode selection instructions
   - Updated pin reference table
   - Added BLE troubleshooting section

4. **WIRING_DIAGRAM.md**
   - Added mode switch wiring diagram
   - Updated digital I/O pin assignments
   - Added mode switch logic explanation

5. **DROIDPAD_INTEGRATION.md**
   - Added control mode selection section
   - Added BLE direct control setup instructions
   - Added BLE data format and characteristic UUIDs
   - Added comparison between BLE and MQTT modes

6. **README.md**
   - Updated features list to highlight mode selection
   - Added BLE as primary/simplest control option
   - Updated quick start guide with BLE option first

#### New Documentation Files

1. **DROIDPAD_BLE_SETUP.md** (8.4 KB)
   - Complete step-by-step BLE setup guide
   - Hardware configuration instructions
   - DroidPad configuration with UUIDs
   - Control mapping reference
   - Troubleshooting guide
   - Advantages and limitations of BLE mode

2. **MODE_SWITCH_WIRING.md** (9.8 KB)
   - Detailed switch wiring diagrams
   - Bill of materials for switch
   - Recommended switch models
   - Installation steps
   - Testing procedures
   - Troubleshooting
   - Alternative simplified wiring methods

### BLE Implementation Details

#### Service UUID
```
19B10000-E8F2-537E-4F6C-D104768A1214
```

#### Characteristics

**Left Joystick Characteristic:**
- UUID: `19B10001-E8F2-537E-4F6C-D104768A1214`
- Properties: Read + Write
- Format: 8 bytes (2 x float32)
  - Bytes 0-3: left_x (-1.0 to 1.0)
  - Bytes 4-7: left_y (-1.0 to 1.0)

**Right Joystick Characteristic:**
- UUID: `19B10002-E8F2-537E-4F6C-D104768A1214`
- Properties: Read + Write
- Format: 8 bytes (2 x float32)
  - Bytes 0-3: right_x (-1.0 to 1.0)
  - Bytes 4-7: right_y (-1.0 to 1.0)

#### BLE Device Name
```
LifeTrac-v25
```

### Code Statistics

- **Total Lines Added**: ~160 lines in controller
- **New Functions**: 3 (readModeSwitch, setupBLE, readBLEJoystickData)
- **Documentation Added**: ~18 KB in new/updated files

### Safety Features Maintained

All existing safety features work in both modes:
- **Deadzone**: 10% (0.1) deadzone on all inputs
- **Safety Timeout**: 1 second timeout stops all movement
- **Emergency Stop**: Send all zeros or disconnect to stop
- **Input Validation**: Values outside ±1.0 are clamped

### Benefits of This Implementation

1. **Simplicity**: BLE mode requires no network setup
2. **Lower Latency**: BLE typically 20-50ms vs 100-200ms for MQTT
3. **Mobile-First**: Perfect for smartphone/tablet control
4. **Backward Compatible**: Existing MQTT setups continue to work
5. **Flexible**: Easy to switch between modes with hardware switch
6. **Safe Default**: BLE mode is default if switch not installed
7. **Power Saving**: OFF mode provides complete shutdown

### Testing Recommendations

1. **Mode Detection Test**
   - Test with switch in each position
   - Verify Serial Monitor shows correct mode
   - Test with no switch installed (should default to BLE)

2. **BLE Functionality Test**
   - Scan for "LifeTrac-v25" from mobile device
   - Connect and verify characteristic UUIDs
   - Send test data and verify movement
   - Test safety timeout
   - Test emergency stop

3. **MQTT Functionality Test**
   - Verify WiFi connection in MQTT mode
   - Test MQTT message reception
   - Verify existing ESP32 remote still works
   - Test web interface compatibility

4. **Switch Reliability Test**
   - Test switching between modes
   - Verify power cutoff in OFF position
   - Test mode persistence across power cycles

### Known Limitations

1. **BLE Range**: Typically 30-100 feet (10-30m) vs potentially unlimited with WiFi
2. **Single Connection**: BLE mode supports only one connected device at a time
3. **No Status Publishing**: BLE mode doesn't publish status to MQTT (not connected)
4. **Compilation Requirement**: Requires ArduinoBLE library (built-in for Arduino Opta)

### Future Enhancements

Possible future improvements:
1. Add BLE status characteristic for reading system state
2. Support multiple simultaneous BLE connections
3. Add BLE battery level reporting
4. Implement BLE security (pairing/encryption)
5. Add mode switching via serial command (without hardware switch)
6. Add visual LED indicators for mode selection

### Migration Guide

For existing installations:
1. Update Arduino Opta firmware to include BLE support
2. Optionally install mode switch hardware
3. If no switch installed, controller defaults to BLE mode
4. To continue using MQTT only, connect D9 to 12V permanently

### Required Libraries

- ArduinoBLE (v1.3.0 or newer) - Arduino official library
- OptaController - Arduino Opta official library
- PubSubClient (v2.8 or newer) - For MQTT
- ArduinoJson (v6.21 or newer) - For JSON parsing
- WiFi - Built-in ESP32 library

### Compatibility

- **Arduino Board**: Arduino Opta WiFi
- **Arduino IDE**: 2.0 or newer
- **DroidPad**: Android/iOS versions with BLE support
- **Mobile OS**: Android 5.0+ or iOS 10.0+ (BLE 4.0+ required)

### Related Documentation

- [DROIDPAD_BLE_SETUP.md](DROIDPAD_BLE_SETUP.md) - Complete BLE setup guide
- [MODE_SWITCH_WIRING.md](MODE_SWITCH_WIRING.md) - Hardware wiring instructions
- [DROIDPAD_INTEGRATION.md](DROIDPAD_INTEGRATION.md) - General DroidPad guide
- [INSTALLATION_GUIDE.md](INSTALLATION_GUIDE.md) - Complete installation guide
- [WIRING_DIAGRAM.md](WIRING_DIAGRAM.md) - System wiring diagrams

### Support

For issues or questions:
- GitHub Issues: https://github.com/OpenSourceEcology/LifeTrac/issues
- Documentation: See README.md and linked guides above

### Contributors

This feature was developed to simplify mobile control while maintaining flexibility for network-based operations.

---

**Note**: This implementation provides a foundation for direct mobile control. Users can choose the mode that best fits their use case: simple BLE for mobile operation, or MQTT for network integration and multiple control interfaces.
