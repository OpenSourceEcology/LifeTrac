# DroidPad Integration Guide for LifeTrac v25

## Overview

LifeTrac v25 now supports DroidPad and other joystick interfaces that output values in the range of **-1.0 to 1.0** (floating point). DroidPad can connect to the LifeTrac v25 controller in two ways:

1. **BLE Direct Control (New)**: Direct Bluetooth Low Energy connection to Arduino Opta
2. **MQTT Control (Traditional)**: Via WiFi network and MQTT broker

This document explains both methods and the changes made to enable this compatibility.

## What Changed

### Previous System
- **Value Range**: -100 to 100 (integer)
- **Data Type**: `int`
- **Deadzone**: 10 (integer)

### New System (DroidPad Compatible)
- **Value Range**: -1.0 to 1.0 (float)
- **Data Type**: `float`
- **Deadzone**: 0.1 (10% of range)

## Component Changes

### 1. ESP32 Remote Control
- **File**: `esp32_remote_control/lifetrac_v25_remote.ino`
- **Changes**:
  - ControlData struct now uses `float` values
  - Joystick conversion: `(value - 511.5) / 511.5` to get -1.0 to 1.0 range
  - Deadzone check uses 0.1 instead of 10

### 2. Arduino Opta Controller
- **File**: `arduino_opta_controller/lifetrac_v25_controller.ino`
- **Changes**:
  - JoystickData struct now uses `float` values
  - DEADZONE constant changed from `10` to `0.1`
  - Flow control mapping updated: `4 + (int)(maxInput * 16.0)` for 4-20mA range
  - All control functions accept `float` parameters

### 3. ROS2 Integration
- **Message Definition**: `ros2_bridge/lifetrac_msgs/msg/ControlCommand.msg`
  - Changed from `int32` to `float32` for all joystick fields
- **Bridge Node**: `ros2_bridge/lifetrac_mqtt_bridge/lifetrac_mqtt_bridge/mqtt_bridge_node.py`
  - Validation updated to check range -1.0 to 1.0
- **Test Publisher**: Updated all demo values to use floats

### 4. Test Scripts
- **File**: `test_scripts/mqtt_test.py`
- **Changes**:
  - All test sequences use float values (e.g., 0.5 instead of 50)
  - Interactive mode commands updated

## Control Mode Selection

The Arduino Opta controller supports two control modes via a 3-position hardware switch:

### BLE Mode (Default)
- Direct Bluetooth connection to DroidPad app
- No WiFi network or MQTT broker required
- Lower latency, simpler setup
- Automatically selected if mode switch is not installed
- Range: Typical BLE range of 30-100 feet (10-30m)

### MQTT Mode (Traditional)
- WiFi network with MQTT broker required
- Supports multiple control interfaces (ESP32 remote, web, ROS2)
- Longer range via WiFi network
- Selected by setting mode switch to MQTT position

### OFF Mode
- Hardware power cutoff
- Completely powers down the Opta controller

## Using DroidPad

DroidPad outputs joystick values from **1.0 to -1.0**. The LifeTrac v25 system now natively accepts these values through both BLE and MQTT.

### Method 1: BLE Direct Control (Recommended for DroidPad)

**Setup Steps:**
1. Ensure mode switch is in BLE position (or not installed for default BLE mode)
2. Power on the LifeTrac v25 controller
3. Open DroidPad app on your Android/iOS device
4. Enable Bluetooth on your mobile device
5. In DroidPad, scan for BLE devices and connect to "LifeTrac-v25"
6. Configure DroidPad to use BLE GATT characteristics:
   - Service UUID: `19B10000-E8F2-537E-4F6C-D104768A1214`
   - Left Joystick Characteristic: `19B10001-E8F2-537E-4F6C-D104768A1214`
   - Right Joystick Characteristic: `19B10002-E8F2-537E-4F6C-D104768A1214`

**Data Format for BLE:**
Each characteristic accepts 8 bytes (2 floats):
- Left Joystick: [left_x (float), left_y (float)]
- Right Joystick: [right_x (float), right_y (float)]
- Each float value should be in range -1.0 to 1.0
- Send as little-endian IEEE 754 32-bit floats

**BLE Control Mapping:**
| DroidPad Control | BLE Characteristic | Bytes | Function |
|-----------------|-------------------|-------|----------|
| Left Stick | Left Joystick Char | 0-3: left_x, 4-7: left_y | Forward/Backward + Turning |
| Right Stick | Right Joystick Char | 0-3: right_x, 4-7: right_y | Bucket + Arms |

### Method 2: MQTT Control (Traditional)

**Setup Steps:**
1. Set mode switch to MQTT position
2. Ensure WiFi network and MQTT broker are running
3. Configure DroidPad to send MQTT messages

### MQTT Message Format

```json
{
  "left_x": 0.0,    // -1.0 to 1.0 (left/right turn)
  "left_y": 0.5,    // -1.0 to 1.0 (forward/backward)
  "right_x": 0.0,   // -1.0 to 1.0 (bucket control)
  "right_y": 0.7,   // -1.0 to 1.0 (arms control)
  "timestamp": 1234567890
}
```

### Example DroidPad Setup

1. **Configure DroidPad** to send MQTT messages to your broker
2. **MQTT Topic**: `lifetrac/v25/control`
3. **Message Format**: JSON with fields as shown above
4. **Broker Settings**:
   - Host: Your MQTT broker IP (default: 192.168.1.100)
   - Port: 1883
   - Username: lifetrac
   - Password: lifetrac_pass

### Control Mapping

| DroidPad Output | LifeTrac Function |
|----------------|-------------------|
| Left Stick Y (1.0 to -1.0) | Forward (1.0) / Backward (-1.0) |
| Left Stick X (1.0 to -1.0) | Right Turn (1.0) / Left Turn (-1.0) |
| Right Stick Y (1.0 to -1.0) | Arms Up (1.0) / Arms Down (-1.0) |
| Right Stick X (1.0 to -1.0) | Bucket Up (1.0) / Bucket Down (-1.0) |

## Value Interpretation

The system interprets the float values as follows:

- **0.0**: No movement (stopped)
- **0.1 to 1.0**: Progressive movement in positive direction
- **-0.1 to -1.0**: Progressive movement in negative direction
- **±0.0 to ±0.1**: Deadzone (no movement to prevent drift)

## Flow Control

The proportional flow control valve responds to the maximum absolute value from all inputs:

- **0.0**: 4mA (no flow)
- **0.1 to 1.0**: 6-20mA (progressive flow from ~12.5% to 100%)

Formula: `currentValue = 4 + (maxInput * 16)`

## Backwards Compatibility

While the system now uses float values internally, you can still send commands from any source as long as they:
1. Use float values in the range -1.0 to 1.0
2. Follow the JSON format shown above
3. Are published to the correct MQTT topic

## Testing

### Quick Test with MQTT Test Script

```bash
cd LifeTrac-v25/test_scripts
python3 mqtt_test.py [broker_ip]
```

The test script has been updated to use the new float value range.

### ROS2 Test Publisher

```bash
ros2 run lifetrac_mqtt_bridge test_publisher
```

Or in interactive mode:
```bash
ros2 run lifetrac_mqtt_bridge test_publisher --interactive
```

## Migration Notes

If you have existing code that sends integer values (-100 to 100), you'll need to update it to send float values (-1.0 to 1.0). Simply divide your integer values by 100:

```python
# Old code
msg["left_y"] = 50  # 50% forward

# New code
msg["left_y"] = 0.5  # 50% forward
```

## Safety Features

All existing safety features remain in place:
- **Deadzone**: 10% of range (0.1) to prevent drift
- **Timeout**: 1 second without commands triggers stop
- **Emergency Stop**: Send all zeros (0.0, 0.0, 0.0, 0.0)
- **Input Validation**: Values outside ±1.0 are rejected

## Support

For issues or questions:
1. Check the main README.md for general setup
2. Review INSTALLATION_GUIDE.md for hardware setup
3. See ros2_bridge/README.md for ROS2 integration details
4. Open an issue on GitHub for specific problems
