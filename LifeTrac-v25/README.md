# LifeTrac v25 - Remote Control System

LifeTrac v25 features a comprehensive remote control system using WiFi and MQTT communication for hydraulic operation control.

## Features

* **Remote Joystick Control:** Dual joystick setup for tank steering and hydraulic functions
* **WiFi Communication:** Reliable wireless control with MQTT protocol
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

## MQTT Broker Infrastructure

* Raspberry Pi (3B+ or newer) for [Mosquitto](https://github.com/eclipse-mosquitto/mosquitto)
* MicroSD card (16GB minimum)
* WiFi router or access point

# Software Components

## Arduino Code
- **arduino_opta_controller/**: Main controller code for Arduino Opta
- **esp32_remote_control/**: Remote control code for ESP32

## Configuration Files
- **config/**: MQTT broker and WiFi configuration files
- **arduino_libraries.txt**: Required Arduino libraries

## Documentation
- **INSTALLATION_GUIDE.md**: Complete setup and installation instructions
- **WIRING_DIAGRAM.md**: Detailed wiring and connection diagrams
- **HYDRAULIC_DIAGRAM.md**: ASCII hydraulic system diagram showing component layout

## Testing Tools
- **test_scripts/**: MQTT testing and debugging utilities

# Quick Start

1. **Set up MQTT Broker:** Install Mosquitto on Raspberry Pi using config/mosquitto.conf
2. **Program Controllers:** Upload Arduino code to Opta and ESP32 boards
3. **Configure Network:** Update WiFi credentials and MQTT settings in code
4. **Wire System:** Follow WIRING_DIAGRAM.md for all connections
5. **Test System:** Use test_scripts/mqtt_test.py for validation

For detailed instructions, see [INSTALLATION_GUIDE.md](INSTALLATION_GUIDE.md).

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

🔄 **In Progress:**
- Field testing and calibration
- Mobile app interface (future enhancement)

# External Links

- [Arduino Opta Documentation](https://docs.arduino.cc/hardware/opta)
- [SparkFun Qwiic Joystick Guide](https://learn.sparkfun.com/tutorials/qwiic-joystick-hookup-guide)
- [Eclipse Mosquitto MQTT Broker](https://mosquitto.org/)
