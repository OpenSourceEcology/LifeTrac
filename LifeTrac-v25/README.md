# LifeTrac v25 - Remote Control System

LifeTrac v25 features a comprehensive remote control system with multiple control modes including WiFi/MQTT and direct Bluetooth (BLE) communication for hydraulic operation control.

## Features

* **Multiple Control Modes:** 3-position switch for OFF/MQTT/BLE mode selection (BLE default)
* **BLE Direct Control:** Direct Bluetooth connection to DroidPad app (no WiFi/broker needed)
* **Remote Joystick Control:** Dual joystick setup for tank steering and hydraulic functions
* **DroidPad Compatible:** Native support for DroidPad via BLE or MQTT using -1.0 to 1.0 value range
* **WiFi Communication:** Reliable wireless control with MQTT protocol
* **ROS2 Integration:** Control LifeTrac from BeagleBone or any ROS2-enabled device
* **Proportional Flow Control:** Speed regulation based on joystick input intensity
* **Safety Features:** Emergency stop, communication timeout, and fail-safe operation
* **Status Monitoring:** Real-time system status and diagnostics

## Control Scheme

### Tank Steering
- **Left Joystick Y-axis:** Forward/backward movement for both tracks
- **Left Joystick X-axis:** Differential steering (left/right turns)

### Hydraulic Functions  
- **Right Joystick Y-axis:** Arms up/down control
- **Right Joystick X-axis:** Bucket up/down control

### Speed Control
- **Proportional Flow Valve:** Automatically adjusts system speed based on maximum joystick input
- **Deadzone:** 10% deadzone prevents unwanted movements from joystick drift

# Hardware Bill of Materials

## Onboard Controller (LifeTrac)

* (4) Hydraulic Directional Valve: 20 gpm Max Flow Rate, D03 NFPA Size, Three Positions, 12V DC 
* (1) Electronically Adjustable Pressure Compensated Proportional Flow Control https://www.brand-hyd.com/Products/Valves/Flow_Controls/EFC/
* (1) Burkert 8605 Type 316532 Flow Valve Controller https://www.burkert-usa.com/en/products/solenoid-control-valves/controllers/316532

* Arduino Opta WiFi - $199.00
* Arduino Pro Opta Ext D1608S - $151.00  
* Arduino Pro Opta Ext A0602 - $229.00

## Remote Control Unit

* SparkFun Qwiic Joystick (2x units) - https://www.sparkfun.com/products/15168
* SparkFun Thing Plus - ESP32 WROOM (USB-C) - https://www.sparkfun.com/products/20168
* Enclosure for handheld remote control
* Li-Po battery pack with charging circuit

## MQTT Broker & Web Controller Infrastructure

* Raspberry Pi (3B+ or newer, 4GB+ recommended) for [Mosquitto](https://github.com/eclipse-mosquitto/mosquitto) and web interface
* MicroSD card (16GB minimum, 32GB recommended)
* WiFi router or access point
* Arducam IMX335 Camera Module (5MP) - https://www.arducam.com/imx335-camera-module-for-rpi-arducam-opensource-camera.html
* Camera cable (included with Arducam)

# Software Components

## Arduino Code
- **arduino_opta_controller/**: Main controller code for Arduino Opta
- **esp32_remote_control/**: Remote control code for ESP32

## Raspberry Pi Web Controller
- **raspberry_pi_web_controller/**: Browser-based control with live video feed
  - Live camera streaming from Arducam IMX335 using libcamera
  - Touch-enabled on-screen joysticks for phone/tablet control
  - Keyboard shortcuts for desktop control
  - WebSocket for real-time communication
  - See [raspberry_pi_web_controller/README.md](raspberry_pi_web_controller/README.md) for setup

## ROS2 Integration
- **ros2_bridge/**: ROS2 packages for BeagleBone control via MQTT
  - **lifetrac_msgs/**: Custom ROS2 message definitions
  - **lifetrac_mqtt_bridge/**: Bridge node using mqtt_client package
  - See [ros2_bridge/README.md](ros2_bridge/README.md) for complete ROS2 setup

## Configuration Files
- **config/**: MQTT broker and WiFi configuration files
- **arduino_libraries.txt**: Required Arduino libraries

## Documentation
- **INSTALLATION_GUIDE.md**: Complete setup and installation instructions
- **DROIDPAD_INTEGRATION.md**: Guide for DroidPad integration via BLE or MQTT
- **DROIDPAD_BLE_SETUP.md**: Step-by-step BLE direct control setup for DroidPad
- **MODE_SWITCH_WIRING.md**: Hardware switch wiring for OFF/MQTT/BLE selection
- **WIRING_DIAGRAM.md**: Detailed wiring and connection diagrams
- **HYDRAULIC_DIAGRAM.md**: ASCII hydraulic system diagram showing component layout
- **ros2_bridge/README.md**: ROS2 integration and BeagleBone setup guide
- **raspberry_pi_web_controller/README.md**: Web interface setup and usage guide

## Testing Tools
- **test_scripts/**: MQTT testing and debugging utilities

# Quick Start

## Option 1: DroidPad via BLE (Simplest - Recommended for Mobile)
1. **Program Arduino Opta:** Upload controller code with BLE support
2. **Set Mode:** Set mode switch to BLE position (or leave switch uninstalled for default BLE)
3. **Connect DroidPad:** Open DroidPad app, scan for "LifeTrac-v25" via Bluetooth
4. **Configure:** Set up BLE characteristics using provided UUIDs
5. **Control:** Start controlling immediately - no WiFi/network needed!

For detailed instructions, see [DROIDPAD_BLE_SETUP.md](DROIDPAD_BLE_SETUP.md).

## Option 2: Raspberry Pi Web Controller (Best for Fixed Installation)
1. **Set up MQTT Broker:** Install Mosquitto on Raspberry Pi using config/mosquitto.conf
2. **Program Arduino Opta:** Upload controller code
3. **Set Mode:** Set mode switch to MQTT position
4. **Install Web Controller:** Run `sudo ./install.sh` in raspberry_pi_web_controller/
5. **Connect Camera:** Attach Arducam IMX335 to Raspberry Pi
6. **Access Interface:** Open `http://<raspberry-pi-ip>:5000` in any browser

For detailed instructions, see [raspberry_pi_web_controller/README.md](raspberry_pi_web_controller/README.md).

## Option 3: ESP32 Handheld Remote Control
1. **Set up MQTT Broker:** Install Mosquitto on Raspberry Pi using config/mosquitto.conf
2. **Program Controllers:** Upload Arduino code to Opta and ESP32 boards
3. **Set Mode:** Set mode switch to MQTT position
4. **Configure Network:** Update WiFi credentials and MQTT settings in code
5. **Wire System:** Follow WIRING_DIAGRAM.md for all connections
6. **Test System:** Use test_scripts/mqtt_test.py for validation

For detailed instructions, see [INSTALLATION_GUIDE.md](INSTALLATION_GUIDE.md).

## Option 4: ROS2 Control from BeagleBone
1. **Set up MQTT Broker:** Same as above
2. **Program Arduino Opta:** Upload controller code
3. **Set Mode:** Set mode switch to MQTT position
4. **Install ROS2 on BeagleBone:** Follow ROS2 installation guide
5. **Build ROS2 packages:** Build lifetrac_msgs and lifetrac_mqtt_bridge
6. **Launch bridge:** `ros2 launch lifetrac_mqtt_bridge lifetrac_bridge.launch.py`

For detailed ROS2 instructions, see [ros2_bridge/README.md](ros2_bridge/README.md).

# Safety Features

- **Communication Timeout:** All movement stops if no commands received within 1 second
- **Emergency Stop:** Serial command or communication loss triggers immediate stop
- **Fail-Safe Operation:** Valves return to neutral position on power loss
- **Status Monitoring:** Real-time diagnostics and connection status

# Development Status

✅ **Completed:**
- Arduino Opta controller code with MQTT integration
- ESP32 remote control with dual joystick support  
- Proportional flow control implementation
- Safety systems and emergency stop functionality
- Complete installation and wiring documentation
- MQTT test utilities for debugging
- ROS2 integration packages for BeagleBone control
- Custom ROS2 message definitions for control commands
- Bridge using mqtt_client for ROS2-MQTT communication
- **Raspberry Pi web controller with live video feed and browser-based joysticks**

🔄 **In Progress:**
- Field testing and calibration

# External Links

- [Arduino Opta Documentation](https://docs.arduino.cc/hardware/opta)
- [SparkFun Qwiic Joystick Guide](https://learn.sparkfun.com/tutorials/qwiic-joystick-hookup-guide)
- [Eclipse Mosquitto MQTT Broker](https://mosquitto.org/)
- [Arducam IMX335 Documentation](https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/5MP-IMX335)
- [Raspberry Pi libcamera Guide](https://www.raspberrypi.com/documentation/computers/camera_software.html)
