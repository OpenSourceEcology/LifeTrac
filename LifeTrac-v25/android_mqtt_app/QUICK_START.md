# LifeTrac v25 Android App - Quick Start Guide

## What's Included

This Android MQTT app provides smartphone/tablet control for LifeTrac v25 using Kodular.io (MIT App Inventor) and the UrsPahoMqttClient component.

## 🚀 Quick Setup (5 Minutes)

### 1. Prerequisites Check
- ✅ LifeTrac v25 system with Arduino Opta controller running
- ✅ Raspberry Pi with Mosquitto MQTT broker configured
- ✅ Android device (7.0+) on same WiFi network
- ✅ Kodular.io account (free)

### 2. Build the App
```bash
# Option A: Follow the detailed guide
# Read: KODULAR_BUILD_INSTRUCTIONS.md

# Option B: Use the template  
# Import: LifeTracV25_Remote_Template.txt into Kodular
```

### 3. Test Without App First
```bash
# Test MQTT connectivity
python3 mqtt_test_listener.py 192.168.1.100 lifetrac lifetrac_pass

# Simulate app messages
python3 simulate_android_app.py 192.168.1.100 lifetrac lifetrac_pass
```

### 4. Install & Configure
1. Build APK in Kodular and install on Android
2. Open app → Settings → Configure MQTT broker details
3. Connect and test basic movements
4. Use Emergency Stop if needed

## 🎮 Control Layout

```
┌─────────────────────────────────────────┐
│          LifeTrac v25 Remote            │
├─────────────────────────────────────────┤
│ Status: Connected | WiFi:● | MQTT:●     │
├─────────────────────────────────────────┤
│                                         │
│  ┌─────────┐              ┌─────────┐   │
│  │  MOVE   │              │ HYDRAUL │   │
│  │    ●    │              │    ●    │   │
│  │ (Tank   │              │(Arms &  │   │
│  │Steering)│              │Bucket)  │   │
│  └─────────┘              └─────────┘   │
│                                         │
├─────────────────────────────────────────┤
│ [Connect] [E-STOP] [Settings] [Help]    │
└─────────────────────────────────────────┘
```

## 📁 File Overview

| File | Purpose |
|------|---------|
| `README.md` | Complete user manual and troubleshooting |
| `KODULAR_BUILD_INSTRUCTIONS.md` | Step-by-step app development |
| `LifeTracV25_Remote_Template.txt` | Project structure template |
| `mqtt_test_listener.py` | Debug MQTT messages from app |
| `simulate_android_app.py` | Test system without app |
| `ANDROID_PERMISSIONS.md` | Device requirements |

## 🔧 MQTT Message Format

```json
{
  "left_x": -50,    // Turn left/right (-100 to +100)
  "left_y": 75,     // Forward/back (-100 to +100)
  "right_x": 0,     // Bucket control (-100 to +100)
  "right_y": -25,   // Arms control (-100 to +100)
  "timestamp": 12345
}
```

**Topic:** `lifetrac/v25/control`  
**Frequency:** 20Hz (every 50ms)  
**Authentication:** Username/password required

## ⚠️ Safety Features

- **1-Second Timeout**: LifeTrac stops if no commands received
- **Emergency Stop**: Red button sends all zeros immediately  
- **Connection Monitor**: Visual WiFi and MQTT status
- **Fail-Safe**: Controller stops on communication loss

## 🐛 Common Issues

**App won't connect:**
```bash
# Test MQTT broker
mosquitto_sub -h 192.168.1.100 -t "lifetrac/v25/#" -u lifetrac -P your_password
```

**Controls not responding:**
- Check LifeTrac controller is powered on
- Verify same WiFi network
- Test with simulate_android_app.py

**Performance issues:**
- Close other apps
- Move closer to WiFi router
- Check signal strength in app

## 📞 Getting Help

1. **Read the docs**: Start with README.md
2. **Test connectivity**: Use the Python testing tools  
3. **Check hardware**: Verify LifeTrac v25 system status
4. **Kodular help**: Check community forums for app issues

## 📦 Distribution Options

- **Direct APK**: Build in Kodular and sideload
- **Google Play**: Upload APK to Play Store  
- **F-Droid**: See F-DROID_SUBMISSION_GUIDE.md for open-source distribution

## 🎯 Next Steps

After basic operation:
- Customize joystick sensitivity in Kodular blocks
- Add data logging features
- Implement voice control
- Create multiple operator profiles
- Add diagnostics dashboard

---

**⏱️ Total setup time: ~30 minutes (including app build)**  
**🔒 Network security: All communication stays on local WiFi**  
**📱 Compatible with: Android 7.0+ smartphones and tablets**