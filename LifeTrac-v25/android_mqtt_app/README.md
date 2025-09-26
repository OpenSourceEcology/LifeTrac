# LifeTrac v25 Android MQTT Remote Control App

This Android app provides wireless control of the LifeTrac v25 using MQTT over WiFi. The app is built using Kodular.io (MIT App Inventor) and uses the UrsPahoMqttClient component for MQTT communication.

## Features

- **Dual Joystick Control**: Virtual joysticks for tank steering and hydraulic functions
- **Real-time MQTT Communication**: Sends control commands at 20Hz frequency
- **Connection Status**: Visual indicators for WiFi and MQTT connection status
- **Emergency Stop**: One-tap emergency stop button
- **Settings**: Configurable MQTT broker settings

## Control Scheme

### Left Joystick (Tank Steering)
- **Y-axis**: Forward/backward movement for both tracks (-100 to +100)
- **X-axis**: Differential steering for turning (-100 to +100)

### Right Joystick (Hydraulic Functions)
- **Y-axis**: Arms up/down control (-100 to +100)
- **X-axis**: Bucket up/down control (-100 to +100)

## Prerequisites

1. **LifeTrac v25 System**: Fully configured with Arduino Opta controller and MQTT broker
2. **WiFi Network**: Android device and LifeTrac system on same network
3. **MQTT Broker**: Mosquitto running on Raspberry Pi with credentials
4. **Kodular Account**: Free account at [kodular.io](https://kodular.io)

## Installation Guide

### Step 1: Import Project to Kodular

1. Go to [kodular.io](https://kodular.io) and create/login to your account
2. Click "Start Creating" to open the project creator
3. Click "Projects" → "Import project (.aia) from my computer"
4. Upload the `LifeTracV25_Remote.aia` file from this directory
5. Wait for the project to import and open

### Step 2: Configure MQTT Settings

1. In the Kodular Designer, click on "SettingsScreen"
2. Locate the default MQTT settings in the blocks:
   - **MQTT Broker IP**: Set to your Raspberry Pi's IP address (default: 192.168.1.100)
   - **MQTT Port**: 1883
   - **Username**: lifetrac
   - **Password**: lifetrac_pass (or your configured password)

### Step 3: Build and Install App

1. In Kodular, click "Build" → "Android App (.apk)"
2. Wait for the build to complete (usually 1-2 minutes)
3. Download the APK file to your computer
4. Transfer the APK to your Android device
5. Enable "Install from unknown sources" in Android settings
6. Install the APK on your Android device

### Step 4: App Configuration

1. Open the "LifeTrac v25 Remote" app on your Android device
2. Tap the "Settings" button (gear icon)
3. Configure the following settings:
   - **MQTT Broker IP**: Your Raspberry Pi's IP address
   - **MQTT Port**: 1883 (default)
   - **Username**: lifetrac
   - **Password**: Your MQTT password
   - **Control Topic**: lifetrac/v25/control (default)
4. Tap "Save Settings"
5. Return to main control screen

### Step 5: Testing Connection

1. Ensure your LifeTrac v25 system is powered on and connected to WiFi
2. Verify the MQTT broker is running on the Raspberry Pi
3. On the Android app, tap "Connect"
4. Check the connection status indicators:
   - **WiFi Icon**: Should be green when connected to network
   - **MQTT Icon**: Should be green when connected to broker
   - **Status Text**: Should show "Connected" or "Ready to control"

## Usage Instructions

### Basic Operation

1. **Connect**: Tap the "Connect" button to establish MQTT connection
2. **Control**: Use the virtual joysticks to control the LifeTrac:
   - Drag the left joystick for movement and steering
   - Drag the right joystick for hydraulic functions
3. **Emergency Stop**: Tap the red "E-STOP" button to immediately stop all movement
4. **Disconnect**: Tap "Disconnect" when finished to close MQTT connection

### Safety Features

- **Automatic Timeout**: LifeTrac stops if no commands received for 1 second
- **Emergency Stop**: Instantly sends zero values to all controls
- **Connection Monitoring**: Real-time status of WiFi and MQTT connections
- **Visual Feedback**: Joystick positions and connection status clearly displayed

## MQTT Message Format

The app sends JSON messages to the `lifetrac/v25/control` topic:

```json
{
  "left_x": -50,      // Left joystick X-axis (-100 to 100)
  "left_y": 75,       // Left joystick Y-axis (-100 to 100) 
  "right_x": 0,       // Right joystick X-axis (-100 to 100)
  "right_y": -25,     // Right joystick Y-axis (-100 to 100)
  "timestamp": 12345  // Milliseconds since app start
}
```

## Troubleshooting

### Connection Issues

**Problem**: App shows "Disconnected" status
- **Solution**: Check WiFi connection, verify MQTT broker IP address and credentials

**Problem**: Controls not responding
- **Solution**: Ensure LifeTrac controller is powered on and check MQTT broker status

### Performance Issues

**Problem**: Delayed response or jerky movement
- **Solution**: Check WiFi signal strength, reduce distance to router if needed

**Problem**: App crashes or freezes
- **Solution**: Restart app, check Android device memory and close other apps

### MQTT Broker Issues

**Problem**: Authentication failed
- **Solution**: Verify MQTT username/password match broker configuration

```bash
# Test MQTT broker connection
mosquitto_sub -h YOUR_PI_IP -t "lifetrac/v25/control" -u lifetrac -P your_password
```

## Customization

### Modifying Control Sensitivity

1. Open project in Kodular
2. Go to "MainScreen" blocks
3. Locate the "JoystickMoved" procedures
4. Adjust the scaling factors to change sensitivity

### Adding New Features

The app can be extended with additional features:
- Multiple control profiles
- Data logging and recording
- Status monitoring and diagnostics
- Custom button controls
- Voice commands

## Technical Details

### Components Used

- **UrsPahoMqttClient**: MQTT communication
- **Canvas**: Virtual joystick implementation
- **Clock**: Timer for periodic message sending
- **TinyDB**: Settings storage
- **Notifier**: User alerts and confirmations

### Message Frequency

- **Control Messages**: 20Hz (every 50ms) when joysticks active
- **Status Updates**: 2Hz (every 500ms) for connection monitoring
- **Timeout**: 1 second safety timeout on LifeTrac controller

## Safety Warnings

⚠️ **Important Safety Information**

- Always maintain visual contact with LifeTrac during remote operation
- Test in a safe, open area away from people and obstacles
- Have emergency stop procedures ready
- Check hydraulic fluid levels before operation
- Ensure all personnel are clear of the operating area
- Use appropriate personal protective equipment

## Support

For technical support:
1. Check this README and troubleshooting section
2. Review the main LifeTrac v25 documentation
3. Test MQTT connection using command line tools
4. Check Kodular community forums for app-specific issues

## Version History

- **v1.0**: Initial release with basic joystick control and MQTT integration