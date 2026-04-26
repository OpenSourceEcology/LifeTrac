# LifeTrac v25 - Mode Switch Quick Reference Card

## 🔌 Switch Positions (HONEYWELL 2NT1-1)

```
╔═══════════════════════════════════════════╗
║  Position 1: [MQTT]  → WiFi/MQTT Mode    ║
║  Position 2: [OFF]   → Power OFF (center)║
║  Position 3: [BLE]   → Bluetooth Mode    ║
╚═══════════════════════════════════════════╝
```

## 🎮 Control Modes

### BLE Mode (Default) 📱
- **No WiFi needed** ✓
- **Direct connection** to phone/tablet
- **Range:** 30-100 feet
- **Latency:** 20-50ms
- **Perfect for:** Mobile control, simple setup

### MQTT Mode 🌐
- **WiFi required** ✓
- **MQTT broker required** ✓
- **Range:** Unlimited (WiFi range)
- **Latency:** 100-200ms
- **Perfect for:** Fixed installation, multiple devices

### OFF Mode 🔴
- **Complete power shutdown**
- Hardware switch disconnects 12V

## 🔧 Pin Configuration

| Mode | D9 Pin | D10 Pin | Power |
|------|--------|---------|-------|
| Position 1 (MQTT) | HIGH | LOW | ✅ |
| Position 2 (OFF) | n/a | n/a | ❌ |
| Position 3 (BLE) | LOW | LOW | ✅ |
| No Switch | LOW* | LOW* | ✅ |

*Internal pulldown → BLE default

## 🚀 Quick Start

### For BLE Mode (Easiest)
1. Power on (switch to Position 3 or no switch)
2. Open DroidPad app
3. Scan for "LifeTrac-v25"
4. Connect and control!

### For MQTT Mode
1. Set switch to Position 1 (MQTT)
2. Ensure WiFi & broker running
3. Use ESP32 remote or web interface
4. Start controlling!

## 📡 BLE Connection Info

**Device Name:** `LifeTrac-v25`

**Service UUID:**  
`19B10000-E8F2-537E-4F6C-D104768A1214`

**Left Joystick Characteristic:**  
`19B10001-E8F2-537E-4F6C-D104768A1214`  
(8 bytes: left_x, left_y as floats)

**Right Joystick Characteristic:**  
`19B10002-E8F2-537E-4F6C-D104768A1214`  
(8 bytes: right_x, right_y as floats)

## 🎯 Control Mapping

| Input | Range | Function |
|-------|-------|----------|
| Left Y | +1.0 | Forward |
| Left Y | -1.0 | Backward |
| Left X | +1.0 | Right turn |
| Left X | -1.0 | Left turn |
| Right Y | +1.0 | Arms up |
| Right Y | -1.0 | Arms down |
| Right X | +1.0 | Bucket curl |
| Right X | -1.0 | Bucket dump |

## 🛡️ Safety Features

- **Deadzone:** ±0.1 (10% of range)
- **Timeout:** 1 second without commands → STOP
- **E-Stop:** Send all zeros (0.0, 0.0, 0.0, 0.0)

## 🔍 Troubleshooting

### Can't find "LifeTrac-v25"
- ✓ Check power is on
- ✓ Switch in BLE position (or no switch)
- ✓ Bluetooth enabled on phone
- ✓ Within 30-100 feet range

### Connected but no control
- ✓ Verify characteristic UUIDs
- ✓ Sending float values -1.0 to 1.0
- ✓ Check 12V power to valves

### Mode doesn't change
- ✓ Verify D9 connection
- ✓ Check switch wiring
- ✓ Power cycle after mode change

## 📚 Documentation

- **BLE Setup:** [DROIDPAD_BLE_SETUP.md](DESIGN-CONTROLLER/DROIDPAD_BLE_SETUP.md)
- **Switch Wiring:** [MODE_SWITCH_WIRING.md](MODE_SWITCH_WIRING.md)
- **Installation:** [INSTALLATION_GUIDE.md](INSTALLATION_GUIDE.md)
- **Full Guide:** [README.md](README.md)

## 🔧 Required Libraries

Install via Arduino Library Manager:
- **ArduinoBLE** (v1.3.0+)
- **OptaController** (Arduino official)
- **PubSubClient** (v2.8+)
- **ArduinoJson** (v6.21+)

## ⚙️ System Status

Check Serial Monitor (115200 baud) for:
```
Mode: BLE (Default)         ← BLE mode active
Mode: MQTT                  ← MQTT mode active
BLE LifeTrac Control...     ← BLE service started
```

---

**Quick Help:** Open Serial Monitor to see current mode and status messages!

**Version:** LifeTrac v25 | **Status:** Production Ready
