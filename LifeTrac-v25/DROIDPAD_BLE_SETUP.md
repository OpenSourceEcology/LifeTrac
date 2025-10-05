# DroidPad BLE Setup Guide for LifeTrac v25

## Overview

This guide provides step-by-step instructions for connecting DroidPad to the LifeTrac v25 controller via Bluetooth Low Energy (BLE) for direct control without requiring WiFi or MQTT broker.

## Prerequisites

- LifeTrac v25 with Arduino Opta controller (firmware v25 or newer)
- DroidPad app installed on Android or iOS device
- Mode switch set to BLE position (or no switch installed for default BLE mode)
- Bluetooth enabled on your mobile device

## Hardware Setup

### 1. Mode Switch Configuration

The LifeTrac v25 uses a 3-position switch to select control mode:

```
Position 1: OFF  - No power (hardware cutoff)
Position 2: MQTT - WiFi/MQTT control mode
Position 3: BLE  - Bluetooth direct control (default)
```

**If you have a mode switch installed:**
- Set switch to BLE position (Position 3)
- This connects 12V power and sets D9=LOW, D10=LOW

**If you don't have a mode switch:**
- Controller defaults to BLE mode automatically
- Internal pulldown resistors ensure D9=LOW, D10=LOW

### 2. Power On Controller

1. Ensure 12V power supply is connected
2. If using mode switch, set to BLE position
3. Power on the system
4. The Arduino Opta will start in BLE mode

**Verification via Serial Monitor (optional):**
- Connect to Arduino Opta via USB
- Open Serial Monitor at 115200 baud
- You should see: "Mode: BLE (Default)" and "BLE LifeTrac Control Service started"

## DroidPad Configuration

### Step 1: Install and Open DroidPad

1. Download DroidPad from Google Play Store (Android) or App Store (iOS)
2. Open the DroidPad app
3. Grant Bluetooth permissions when prompted

### Step 2: Scan for BLE Device

1. In DroidPad, go to **Settings** or **Connection** menu
2. Select **Bluetooth** or **BLE** as connection type
3. Tap **Scan for Devices**
4. Look for "LifeTrac-v25" in the device list
5. Tap to connect

**Note:** If you don't see "LifeTrac-v25", ensure:
- Arduino Opta is powered on
- Mode switch is in BLE position (or not installed)
- You're within 30-100 feet (10-30m) of the controller
- Bluetooth is enabled on your device

### Step 3: Configure BLE Service

Once connected, configure DroidPad to use the LifeTrac BLE service:

**Service Configuration:**
- **Service UUID**: `19B10000-E8F2-537E-4F6C-D104768A1214`

**Characteristic Configuration:**

1. **Left Joystick Control:**
   - **Characteristic UUID**: `19B10001-E8F2-537E-4F6C-D104768A1214`
   - **Properties**: Read + Write
   - **Data Format**: 8 bytes (2 floats)
   - **Byte 0-3**: left_x (float, -1.0 to 1.0)
   - **Byte 4-7**: left_y (float, -1.0 to 1.0)

2. **Right Joystick Control:**
   - **Characteristic UUID**: `19B10002-E8F2-537E-4F6C-D104768A1214`
   - **Properties**: Read + Write
   - **Data Format**: 8 bytes (2 floats)
   - **Byte 0-3**: right_x (float, -1.0 to 1.0)
   - **Byte 4-7**: right_y (float, -1.0 to 1.0)

### Step 4: Map Joysticks to Controls

Configure your DroidPad joysticks to send data to the correct characteristics:

**Left Joystick (Tracks):**
- **Function**: Forward/Backward + Left/Right turning
- **Characteristic**: Left Joystick (UUID ending in ...1214)
- **X-axis**: Left/Right turn (-1.0 left, +1.0 right)
- **Y-axis**: Forward/Backward (-1.0 backward, +1.0 forward)

**Right Joystick (Hydraulics):**
- **Function**: Bucket and Arms control
- **Characteristic**: Right Joystick (UUID ending in ...1214)
- **X-axis**: Bucket control (-1.0 dump, +1.0 curl)
- **Y-axis**: Arms control (-1.0 down, +1.0 up)

### Step 5: Test Connection

1. Gently move left joystick forward - left track should move forward
2. Gently move left joystick left - machine should turn left
3. Move right joystick up - arms should lift
4. Move right joystick right - bucket should curl

**Safety Note:** Start with small, slow movements to verify correct operation!

## Control Mapping Reference

| DroidPad Input | Value Range | LifeTrac Function |
|----------------|-------------|-------------------|
| Left Stick Y=1.0 | Full forward | Forward movement |
| Left Stick Y=-1.0 | Full backward | Backward movement |
| Left Stick X=1.0 | Full right | Right turn |
| Left Stick X=-1.0 | Full left | Left turn |
| Right Stick Y=1.0 | Full up | Arms up |
| Right Stick Y=-1.0 | Full down | Arms down |
| Right Stick X=1.0 | Full right | Bucket curl/up |
| Right Stick X=-1.0 | Full left | Bucket dump/down |

## Data Format Details

### Float Encoding

DroidPad should send float values as **IEEE 754 32-bit little-endian** format:

**Example: Sending left_x = 0.5, left_y = 0.75**

```
Byte array: [0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x40, 0x3F]

Breakdown:
- Bytes 0-3: 0x3F000000 = 0.5 (left_x)
- Bytes 4-7: 0x3F400000 = 0.75 (left_y)
```

Most programming environments handle this automatically when you write float values to BLE characteristics.

## Troubleshooting

### Cannot Find "LifeTrac-v25" Device

**Check:**
1. Is Arduino Opta powered on?
2. Is mode switch in BLE position (or not installed)?
3. Open Serial Monitor - do you see "BLE LifeTrac Control Service started"?
4. Is Bluetooth enabled on your mobile device?
5. Are you within range (30-100 feet / 10-30m)?
6. Try restarting the Arduino Opta

### Connected But No Control

**Check:**
1. Are you sending float values in range -1.0 to 1.0?
2. Are characteristic UUIDs correct?
3. Are you writing to characteristics (not just reading)?
4. Check Serial Monitor for "BLE Left: X=..." or "BLE Right: X=..." messages
5. Verify 12V power is reaching hydraulic valves

### Connection Drops Frequently

**Check:**
1. Reduce distance between device and controller
2. Minimize obstacles (walls, metal) between device and controller
3. Check for other BLE devices causing interference
4. Ensure mobile device battery is charged
5. Check Arduino Opta power supply is stable

### Wrong Control Mapping

**If movements are reversed:**
- Invert joystick values in DroidPad (multiply by -1)
- Or swap which characteristic receives which joystick data

**If left/right are swapped:**
- Swap X and Y axis mapping in DroidPad configuration

## Safety Features

### Deadzone
- Values between -0.1 and +0.1 are ignored (10% deadzone)
- This prevents drift from joystick centering errors

### Safety Timeout
- Controller stops all movement if no commands received for 1 second
- Always keep the DroidPad app active during operation
- If app is minimized, controller will timeout and stop

### Emergency Stop
- Send all zeros (0.0, 0.0, 0.0, 0.0) to stop all movement
- Or simply close DroidPad connection
- Or power off via mode switch

## Advantages of BLE Mode

1. **No WiFi Required**: Direct connection, no network setup needed
2. **Lower Latency**: Typically 20-50ms vs 100-200ms for MQTT
3. **Simpler Setup**: No broker configuration required
4. **Mobile Friendly**: Perfect for smartphone/tablet control
5. **Power Efficient**: BLE uses less power than WiFi

## Limitations of BLE Mode

1. **Range**: ~30-100 feet (10-30m) vs potentially unlimited with WiFi
2. **Single Connection**: Only one device can connect at a time
3. **Line of Sight**: Works best without obstacles
4. **No Logging**: No automatic data logging like MQTT mode

## Switching Between Modes

To switch from BLE to MQTT mode:
1. Set mode switch to MQTT position
2. Power cycle the controller
3. Controller will connect to WiFi/MQTT
4. Disconnect DroidPad BLE connection

To switch from MQTT to BLE mode:
1. Set mode switch to BLE position
2. Power cycle the controller
3. Scan for "LifeTrac-v25" in DroidPad

## Advanced Configuration

### Custom BLE Names

Edit `lifetrac_v25_controller.ino` to change device name:

```cpp
BLE.setLocalName("MyCustomName");
BLE.setDeviceName("MyCustomName");
```

### Custom UUIDs

For multiple LifeTrac units, change UUIDs to avoid conflicts:

```cpp
#define BLE_SERVICE_UUID "19B10000-XXXX-537E-4F6C-D104768A1214"
```

Replace XXXX with unique values for each unit.

## Support and Resources

- **Main Documentation**: See README.md in LifeTrac-v25 folder
- **Wiring Diagram**: See WIRING_DIAGRAM.md for hardware connections
- **Installation Guide**: See INSTALLATION_GUIDE.md for complete setup
- **DroidPad Integration**: See DROIDPAD_INTEGRATION.md for MQTT mode setup
- **GitHub Issues**: Report problems at https://github.com/OpenSourceEcology/LifeTrac/issues

## Version History

- **v25**: Initial BLE support with mode switch
  - Added 3-position switch (OFF/MQTT/BLE)
  - BLE default when switch not installed
  - Direct DroidPad control support
