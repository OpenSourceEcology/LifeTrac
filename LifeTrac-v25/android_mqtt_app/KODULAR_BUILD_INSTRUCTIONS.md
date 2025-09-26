# Kodular App Development Instructions

This document provides detailed instructions for creating the LifeTrac v25 Android MQTT Remote Control app using Kodular.io.

## Project Structure

The app consists of two main screens:
1. **MainScreen**: Primary control interface with virtual joysticks
2. **SettingsScreen**: Configuration for MQTT connection parameters

## Components Required

### Extensions/Components to Add:
1. **UrsPahoMqttClient** - For MQTT communication
2. **Canvas** (2x) - For virtual joysticks implementation  
3. **Clock** - For periodic message timing
4. **TinyDB** - For settings persistence
5. **Notifier** - For user alerts
6. **Button** (Multiple) - For Connect, Disconnect, E-Stop, Settings
7. **Label** (Multiple) - For status displays and titles
8. **TextBox** (Multiple) - For settings input (in SettingsScreen)
9. **HorizontalArrangement/VerticalArrangement** - For layout organization

## Screen Layouts

### MainScreen Layout

```
┌─────────────────────────────────────────┐
│  LifeTrac v25 Remote Control            │
├─────────────────────────────────────────┤
│ Status: [Connected] WiFi:[●] MQTT:[●]   │
├─────────────────────────────────────────┤
│                                         │
│  ┌─────────┐              ┌─────────┐   │
│  │ Left    │              │ Right   │   │
│  │Joystick │              │Joystick │   │
│  │(Move)   │              │(Hydr.)  │   │
│  │    ●    │              │    ●    │   │
│  └─────────┘              └─────────┘   │
│                                         │
├─────────────────────────────────────────┤
│ [Connect] [E-STOP] [Settings] [Info]    │
└─────────────────────────────────────────┘
```

### SettingsScreen Layout

```
┌─────────────────────────────────────────┐
│              Settings                   │
├─────────────────────────────────────────┤
│ MQTT Broker IP: [192.168.1.100]        │
│ MQTT Port:      [1883]                  │  
│ Username:       [lifetrac]              │
│ Password:       [lifetrac_pass]         │
│ Control Topic:  [lifetrac/v25/control]  │
│                                         │
│ Send Rate (Hz): [20]                    │
│ Deadzone (%):   [10]                    │
├─────────────────────────────────────────┤
│        [Save] [Cancel] [Test]           │
└─────────────────────────────────────────┘
```

## Block Programming Logic

### Global Variables

```
Global Variables:
- mqttConnected: false
- leftJoyX: 0
- leftJoyY: 0  
- rightJoyX: 0
- rightJoyY: 0
- lastSendTime: 0
- sendInterval: 50 (20Hz = 50ms)
- deadzone: 10
- mqttBrokerIP: "192.168.1.100"
- mqttPort: 1883
- mqttUsername: "lifetrac"
- mqttPassword: "lifetrac_pass"
- controlTopic: "lifetrac/v25/control"
```

### Screen Initialize

```blocks
When MainScreen.Initialize:
  Do:
    - LoadSettings from TinyDB
    - SetupUI (button colors, initial status)
    - StartStatusUpdateClock (500ms interval)
    - Set joystick canvas backgrounds
    - Draw initial joystick positions
```

### MQTT Connection Logic

```blocks
When ConnectButton.Click:
  Do:
    If not mqttConnected:
      - Set UrsPahoMqttClient.ServerURI to "tcp://" + mqttBrokerIP + ":" + mqttPort
      - Set UrsPahoMqttClient.Username to mqttUsername
      - Set UrsPahoMqttClient.Password to mqttPassword
      - Call UrsPahoMqttClient.Connect
    Else:
      - Call UrsPahoMqttClient.Disconnect

When UrsPahoMqttClient.ConnectionEstablished:
  Do:
    - Set mqttConnected to true
    - Update UI status indicators
    - Change ConnectButton text to "Disconnect"
    - Start control message timer (Clock1)

When UrsPahoMqttClient.ConnectionLost:
  Do:
    - Set mqttConnected to false
    - Update UI status indicators  
    - Change ConnectButton text to "Connect"
    - Stop control message timer
    - Send emergency stop
```

### Virtual Joystick Implementation

```blocks
When LeftJoystickCanvas.Touched:
  Do:
    - Calculate relative position from canvas center
    - Convert to -100 to +100 range
    - Apply deadzone threshold
    - Set leftJoyX and leftJoyY
    - Redraw joystick visual indicator
    - Trigger control message send

When LeftJoystickCanvas.Dragged:
  Do:
    - Same as Touched logic
    - Ensure joystick stays within circular bounds

When RightJoystickCanvas.Touched:
  Do:
    - Calculate relative position from canvas center
    - Convert to -100 to +100 range
    - Apply deadzone threshold
    - Set rightJoyX and rightJoyY
    - Redraw joystick visual indicator
    - Trigger control message send
```

### Control Message Sending

```blocks
When Clock1.Timer (every 50ms):
  Do:
    If mqttConnected:
      - Create JSON message:
        {
          "left_x": leftJoyX,
          "left_y": leftJoyY,
          "right_x": rightJoyX, 
          "right_y": rightJoyY,
          "timestamp": current time in milliseconds
        }
      - Publish to controlTopic
      - Update last send time

Procedure SendControlMessage:
  Do:
    If (current time - lastSendTime) > sendInterval:
      - Build JSON control message
      - Call UrsPahoMqttClient.Publish with topic and message
      - Set lastSendTime to current time
```

### Emergency Stop

```blocks
When EStopButton.Click:
  Do:
    - Set all joystick values to 0
    - Send immediate control message with all zeros
    - Flash E-Stop button red
    - Show notification "EMERGENCY STOP ACTIVATED"
    - Log emergency stop event
```

### Settings Management

```blocks
When SettingsButton.Click:
  Do:
    - Open SettingsScreen
    - Load current settings into text boxes

When SettingsScreen.SaveButton.Click:
  Do:
    - Validate IP address format
    - Validate port number (1-65535)
    - Store all settings to TinyDB
    - Update global variables
    - Show "Settings Saved" notification
    - Close SettingsScreen

Procedure LoadSettings:
  Do:
    - Get mqttBrokerIP from TinyDB (default: "192.168.1.100")
    - Get mqttPort from TinyDB (default: 1883)
    - Get mqttUsername from TinyDB (default: "lifetrac")
    - Get mqttPassword from TinyDB (default: "lifetrac_pass")
    - Get controlTopic from TinyDB (default: "lifetrac/v25/control")
    - Get sendInterval from TinyDB (default: 50)
    - Get deadzone from TinyDB (default: 10)
```

### Status Updates

```blocks
When StatusClock.Timer (every 500ms):
  Do:
    - Update WiFi status indicator
    - Update MQTT connection status
    - Update signal strength display
    - Check for connection timeouts
    - Update joystick position displays
```

## Joystick Drawing Functions

```blocks
Procedure DrawJoystick(canvas, centerX, centerY, joyX, joyY):
  Do:
    - Clear canvas
    - Draw outer circle (joystick boundary)
    - Calculate joystick knob position based on joyX, joyY
    - Draw inner circle (joystick knob) at calculated position
    - Draw crosshairs for center reference
    - Add axis value text displays

Procedure CalculateJoystickValues(canvas, touchX, touchY):
  Returns: [normalizedX, normalizedY]
  Do:
    - Get canvas center coordinates
    - Calculate relative position (touchX - centerX, touchY - centerY)
    - Limit to circular boundary
    - Normalize to -100 to +100 range
    - Apply deadzone threshold
    - Return normalized values
```

## Error Handling

```blocks
When UrsPahoMqttClient.Error:
  Do:
    - Log error message
    - Set mqttConnected to false
    - Update UI status
    - Show error notification
    - Attempt reconnection after delay

When UrsPahoMqttClient.MessageDelivered:
  Do:
    - Update transmission indicator
    - Log successful message delivery

When UrsPahoMqttClient.MessageNotDelivered:
  Do:
    - Log failed delivery
    - Increment retry counter
    - Show connection warning if needed
```

## UI Color Coding

- **Green**: Connected, active, normal operation
- **Red**: Disconnected, error, emergency stop
- **Yellow/Orange**: Warning, transitional states
- **Blue**: Information, settings, configuration
- **Gray**: Inactive, disabled elements

## Development Steps in Kodular

1. **Create New Project**: "LifeTracV25_Remote"
2. **Add Components**: All required components from palette
3. **Design Layout**: Arrange components per layout specifications
4. **Add Extension**: Import UrsPahoMqttClient extension
5. **Program Blocks**: Implement all logic blocks as specified
6. **Test Simulator**: Use Kodular's built-in testing tools
7. **Build APK**: Generate Android package for installation
8. **Test on Device**: Install and test with actual MQTT broker

## Testing Checklist

- [ ] App starts without crashes
- [ ] Settings screen saves/loads correctly  
- [ ] MQTT connection establishes successfully
- [ ] Joysticks respond to touch/drag
- [ ] Control messages sent at correct frequency
- [ ] Emergency stop functions immediately
- [ ] Connection status indicators work
- [ ] App handles network disconnection gracefully
- [ ] Settings validation prevents invalid inputs
- [ ] App recovers from MQTT broker restart

## Performance Considerations

- **Message Rate**: 20Hz provides good responsiveness without overwhelming network
- **JSON Size**: Keep messages compact for faster transmission
- **Battery Usage**: Optimize timer intervals and reduce unnecessary updates
- **Memory**: Clear unused variables and optimize image resources
- **Network**: Handle poor WiFi conditions gracefully

This documentation provides the complete blueprint for building the Kodular app. The actual .aia file would be generated by following these instructions in the Kodular IDE.