# LifeTrac v25 Remote Control Installation Guide

This guide provides step-by-step instructions for setting up the LifeTrac v25 remote control system using WiFi and MQTT communication.

## System Overview

The LifeTrac v25 remote control system consists of three main components:

1. **Arduino Opta Controller** - Main hydraulic control unit mounted on the LifeTrac
2. **ESP32 Remote Control** - Handheld remote with dual joysticks
3. **MQTT Broker** - Raspberry Pi running Mosquitto for communication

For a detailed view of the hydraulic system layout and component connections, see [HYDRAULIC_DIAGRAM.md](HYDRAULIC_DIAGRAM.md).

## Hardware Requirements

### Onboard Controller (LifeTrac)
- Arduino Opta WiFi ($199.00)
- Arduino Pro Opta Ext D1608S - Digital I/O Extension ($151.00)
- Arduino Pro Opta Ext A0602 - Analog Extension ($229.00)
- 4x Hydraulic Directional Valves (20 gpm, D03 NFPA, 12V DC)
- 1x Electronically Adjustable Proportional Flow Control Valve
- 1x Burkert 8605 Type 316532 Flow Valve Controller

### Remote Control Unit
- SparkFun Thing Plus - ESP32 WROOM (USB-C)
- 2x SparkFun Qwiic Joystick
- Qwiic cables for joystick connections
- Enclosure for handheld remote
- Battery pack (Li-Po recommended)

### MQTT Broker
- Raspberry Pi (3B+ or newer recommended)
- MicroSD card (16GB minimum)
- WiFi router or access point

## Software Requirements

### Arduino IDE Setup
1. Install Arduino IDE 2.0 or newer
2. Install ESP32 board package:
   - Go to File → Preferences
   - Add `https://dl.espressif.com/dl/package_esp32_index.json` to Additional Board Manager URLs
   - Go to Tools → Board → Boards Manager
   - Search for "ESP32" and install the ESP32 package

3. Install required libraries:
   - `WiFi` (ESP32 built-in)
   - `PubSubClient` by Nick O'Leary
   - `ArduinoJson` by Benoit Blanchon
   - `SparkFun Qwiic Joystick Arduino Library`

## Step 1: Set Up MQTT Broker (Raspberry Pi)

### 1.1 Install Raspberry Pi OS
1. Flash Raspberry Pi OS Lite to SD card
2. Enable SSH by creating empty `ssh` file on boot partition
3. Configure WiFi by editing `wpa_supplicant.conf`
4. Boot the Pi and connect via SSH

### 1.2 Install Mosquitto MQTT Broker
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Mosquitto
sudo apt install mosquitto mosquitto-clients -y

# Enable Mosquitto service
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```

### 1.3 Configure Mosquitto
Create configuration file:
```bash
sudo nano /etc/mosquitto/conf.d/lifetrac.conf
```

Add the following content:
```
# Basic configuration
port 1883
allow_anonymous false
password_file /etc/mosquitto/passwd

# Logging
log_dest file /var/log/mosquitto/mosquitto.log
log_type all

# Persistence
persistence true
persistence_location /var/lib/mosquitto/
```

Create user credentials:
```bash
# Create password file
sudo mosquitto_passwd -c /etc/mosquitto/passwd lifetrac

# Restart Mosquitto
sudo systemctl restart mosquitto

# Check status
sudo systemctl status mosquitto
```

## Step 2: Configure Arduino Opta Controller

### 2.1 Hardware Connections

**Digital I/O Extension (D1608S) Connections:**
- D1: Left Track Forward Valve
- D2: Left Track Backward Valve
- D3: Right Track Forward Valve
- D4: Right Track Backward Valve
- D5: Arms Up Valve
- D6: Arms Down Valve
- D7: Bucket Up Valve
- D8: Bucket Down Valve

**Analog Extension (A0602) Connections:**
- A0: Proportional Flow Control Valve (PWM output)
- A0: Burkert 8605 Controller Interface (analog input, may require voltage scaling)

**Power Connections:**
- 12V DC supply for hydraulic valves
- 24V DC supply for Arduino Opta (if required)
- 24V DC supply for Burkert 8605 Controller (separate supply recommended)

### 2.2 Software Configuration

1. Open `arduino_opta_controller/lifetrac_v25_controller.ino` in Arduino IDE
2. Update WiFi credentials:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
3. Update MQTT broker IP address:
   ```cpp
   const char* mqtt_server = "192.168.1.100";  // Your Pi's IP
   ```
4. Select Board: "Arduino Opta WiFi"
5. Upload the code to Arduino Opta

## Step 3: Build ESP32 Remote Control

### 3.1 Hardware Assembly

**Joystick Connections:**
- Left Joystick: Connect to Qwiic bus with I2C address 0x20
- Right Joystick: Connect to Qwiic bus with I2C address 0x21

**Power:**
- Connect Li-Po battery or USB-C power supply

### 3.2 Set Joystick I2C Addresses

Each joystick needs a unique I2C address. Use the SparkFun Qwiic Joystick library example to set addresses:

1. Connect one joystick at a time
2. Upload the address-changing example
3. Set left joystick to address 0x20
4. Set right joystick to address 0x21

### 3.3 Software Configuration

1. Open `esp32_remote_control/lifetrac_v25_remote.ino` in Arduino IDE
2. Update WiFi credentials (same as controller)
3. Update MQTT broker IP address (same as controller)
4. Select Board: "ESP32 Dev Module" or "SparkFun ESP32 Thing Plus C"
5. Upload the code to ESP32

## Step 4: System Testing

### 4.1 Initial Power-Up Tests

1. **MQTT Broker Test:**
   ```bash
   # Test MQTT broker
   mosquitto_sub -h localhost -t "lifetrac/v25/#" -u lifetrac -P your_password
   ```

2. **Controller Test:**
   - Power up Arduino Opta
   - Check serial monitor for WiFi and MQTT connection
   - Verify status messages are published

3. **Remote Test:**
   - Power up ESP32 remote
   - Check serial monitor for joystick detection
   - Verify control messages are published

### 4.2 Control System Verification

**Tank Steering Test:**
- Left joystick Y-axis: Forward/backward movement
- Left joystick X-axis: Left/right turning
- Verify proper valve activation for differential steering

**Hydraulic Functions Test:**
- Right joystick Y-axis: Arms up/down
- Right joystick X-axis: Bucket up/down

**Flow Control Test:**
- Move joysticks with varying intensity
- Verify proportional flow control responds to maximum input
- Check minimum flow threshold activation
- Ensure Burkert 8605 controller provides smooth flow modulation
- Test response time and accuracy of flow control

### 4.3 Safety System Tests

**Emergency Stop:**
- Disconnect remote control
- Verify all movement stops within 1 second (safety timeout)
- Test serial emergency stop command: `stop`

**Communication Loss:**
- Move remote out of WiFi range
- Verify automatic stop when connection lost
- Test reconnection behavior

## Step 5: Operational Guidelines

### 5.1 Startup Procedure

1. Power up Raspberry Pi MQTT broker
2. Power up Arduino Opta controller
3. Wait for WiFi and MQTT connection (check status LED)
4. Power up ESP32 remote control
5. Verify joystick connectivity
6. Test movement in safe area before full operation

### 5.2 Control Operation

**Tank Steering:**
- Push left joystick forward: Both tracks forward
- Push left joystick backward: Both tracks backward
- Push left joystick left: Left track slower, right track faster (left turn)
- Push left joystick right: Right track slower, left track faster (right turn)

**Hydraulic Controls:**
- Right joystick up: Arms up
- Right joystick down: Arms down
- Right joystick left: Bucket down
- Right joystick right: Bucket up

**Speed Control:**
- System speed is automatically controlled by proportional flow valve
- Speed is determined by the maximum joystick input
- Light joystick movements = slow operation
- Full joystick deflection = maximum speed

### 5.3 Troubleshooting

**Connection Issues:**
- Check WiFi signal strength (LED blink patterns)
- Verify MQTT broker is running: `sudo systemctl status mosquitto`
- Check IP addresses in code match network configuration

**Control Issues:**
- Verify joystick I2C addresses are correct (0x20, 0x21)
- Monitor serial output for debugging information

**Safety Issues:**
- If system becomes unresponsive, power cycle all components
- Use emergency stop serial command
- Check safety timeout is functioning (1-second limit)

## Step 6: Maintenance and Updates

### 6.1 Regular Maintenance
- Check battery voltage on remote control
- Verify WiFi signal strength in operating area
- Test emergency stop function monthly
- Update software as needed

### 6.2 Configuration Changes
- WiFi credentials: Update in both controller and remote code
- MQTT settings: Update broker IP and credentials
- Control sensitivity: Adjust deadzone values in code
- Safety timeouts: Modify timeout values as needed

## Appendix A: Pin Reference

### Arduino Opta Digital I/O (D1608S)
| Pin | Function |
|-----|----------|
| D1  | Left Track Forward |
| D2  | Left Track Backward |
| D3  | Right Track Forward |
| D4  | Right Track Backward |
| D5  | Arms Up |
| D6  | Arms Down |
| D7  | Bucket Up |
| D8  | Bucket Down |

### Arduino Opta Analog (A0602)
| Pin | Function |
|-----|----------|
| A0  | Proportional Flow Control (PWM) / Burkert 8605 Interface |

### ESP32 Remote Control
| Pin | Function |
|-----|----------|
| GPIO 2  | Status LED |
| SDA/SCL | Qwiic Joysticks |

## Appendix B: MQTT Topics

| Topic | Direction | Description |
|-------|-----------|-------------|
| `lifetrac/v25/control` | Remote → Controller | Joystick control data |
| `lifetrac/v25/status` | Controller → Broker | System status and valve states |
| `lifetrac/v25/remote_status` | Remote → Broker | Remote battery and connection status |

## Appendix C: Safety Considerations

- Always test in a safe, open area before full operation
- Maintain visual contact with LifeTrac during remote operation
- Have emergency stop procedures ready
- Check hydraulic fluid levels and system pressure regularly
- Ensure all personnel are clear of the operating area
- Use appropriate personal protective equipment

## Support and Development

For issues, improvements, or contributions to this project:
- GitHub Repository: [OpenSourceEcology/LifeTrac](https://github.com/OpenSourceEcology/LifeTrac)
- Wiki Documentation: [OSE Wiki](https://wiki.opensourceecology.org/)
- Community Forum: [OSE Community](https://community.opensourceecology.org/)

---

**Version:** 1.0  
**Last Updated:** 2024  
**License:** CC-BY-SA-4.0 International