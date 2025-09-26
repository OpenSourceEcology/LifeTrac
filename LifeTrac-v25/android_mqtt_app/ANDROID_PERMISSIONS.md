# Android App Permissions and Requirements

## Required Permissions

The LifeTrac v25 Android MQTT Remote Control app requires the following permissions:

### Network Permissions
```xml
<uses-permission android:name="android.permission.INTERNET" />
<uses-permission android:name="android.permission.ACCESS_NETWORK_STATE" />
<uses-permission android:name="android.permission.ACCESS_WIFI_STATE" />
```

**Purpose:**
- `INTERNET`: Required for MQTT communication over WiFi
- `ACCESS_NETWORK_STATE`: Monitor network connectivity status
- `ACCESS_WIFI_STATE`: Check WiFi connection strength and status

### Power Management
```xml
<uses-permission android:name="android.permission.WAKE_LOCK" />
```

**Purpose:**
- `WAKE_LOCK`: Prevent device from sleeping during active control sessions

## Android Version Requirements

### Minimum Requirements
- **Minimum SDK**: Android 7.0 (API Level 24)
- **Target SDK**: Android 10 (API Level 29) 
- **Compile SDK**: Android 11 (API Level 30)

### Recommended
- **Android 8.0+** for optimal MQTT performance
- **4GB RAM** for smooth operation
- **Dual-band WiFi** for best connectivity

## Device Compatibility

### Supported Devices
- **Smartphones**: Android phones with 5"+ screens
- **Tablets**: 7"+ tablets recommended for better joystick control
- **Rugged Devices**: Industrial Android devices for field use

### Screen Requirements
- **Minimum Resolution**: 720x1280 (HD)
- **Recommended**: 1080x1920 (Full HD) or higher
- **Orientation**: Portrait and landscape supported

## Hardware Requirements

### Essential
- **WiFi**: 802.11n (2.4GHz) minimum
- **Touch Screen**: Multi-touch capable
- **RAM**: 2GB minimum, 4GB recommended
- **Storage**: 50MB free space for app

### Recommended
- **WiFi**: 802.11ac (5GHz) for lower latency
- **GPS**: For location logging (future feature)
- **Accelerometer**: For gesture controls (future feature)
- **Large Battery**: For extended operation sessions

## Network Requirements

### WiFi Network
- **2.4GHz or 5GHz**: Device and LifeTrac system on same network
- **Bandwidth**: Minimum 1Mbps, 10Mbps+ recommended
- **Latency**: <100ms to MQTT broker preferred
- **Range**: Stay within 100m of access point for best performance

### MQTT Broker
- **Mosquitto**: v2.0+ running on Raspberry Pi
- **Port**: 1883 (standard MQTT)
- **Authentication**: Username/password required
- **Persistence**: Enabled for message reliability

## Security Considerations

### Network Security
- **WPA2/WPA3**: Secure WiFi network required
- **VPN**: Optional for remote access scenarios
- **Firewall**: MQTT port (1883) must be accessible

### App Security
- **Local Storage**: Settings encrypted in device storage
- **No Cloud**: All communication stays on local network
- **Authentication**: MQTT credentials required for connection

## Power Management

### Battery Optimization
- **Screen Dimming**: Automatic after 30 seconds of inactivity
- **Background Limits**: App pauses MQTT when backgrounded
- **Low Power Mode**: Reduces update frequency on low battery

### Recommended Settings
- **Battery Saver**: Disable for this app during operation
- **WiFi Sleep**: Keep WiFi alive during operation
- **Background Apps**: Close unnecessary apps for better performance

## Troubleshooting Common Issues

### Permission Denied
**Problem**: App can't access network
**Solution**: Check app permissions in Android settings

### Poor Performance
**Problem**: Laggy controls or dropped connections
**Solution**: 
- Close background apps
- Move closer to WiFi router
- Switch to 5GHz WiFi if available

### Connection Timeouts
**Problem**: Frequent disconnections
**Solution**:
- Check WiFi signal strength
- Verify MQTT broker is running
- Restart router/access point

## Development Notes

### Kodular Specific
- **Extensions**: UrsPahoMqttClient must be properly imported
- **Permissions**: Automatically handled by Kodular build process
- **Testing**: Use Kodular Companion for development testing

### APK Signing
- **Debug**: Kodular signs automatically for testing
- **Release**: Use proper signing certificate for distribution
- **Verification**: Test APK on multiple devices before release

## Installation Guide

### From APK File
1. Enable "Unknown Sources" in Android security settings
2. Download APK to device
3. Tap APK file to install
4. Grant required permissions when prompted
5. Launch app and configure MQTT settings

### From Google Play Store (Future)
1. Search for "LifeTrac v25 Remote"
2. Install app from store
3. Launch and configure settings
4. No need to enable unknown sources

## Compliance Notes

### Privacy
- **No Data Collection**: App doesn't collect user data
- **Local Only**: All data stays on local network
- **No Analytics**: No tracking or analytics implemented

### Accessibility
- **Large Buttons**: Suitable for work gloves
- **High Contrast**: Visible in outdoor conditions
- **Voice Feedback**: Audio status updates (future feature)

This permissions document ensures proper Android app configuration and helps users understand system requirements.