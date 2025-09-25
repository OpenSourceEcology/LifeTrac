# LifeTrac v25 Wiring Diagram

This document describes the electrical connections for the LifeTrac v25 remote control system.

## System Architecture

```
[ESP32 Remote] --WiFi--> [WiFi Router] <--WiFi-- [Arduino Opta] --> [Hydraulic Valves]
                              |
                         [Raspberry Pi]
                         (MQTT Broker)
```

## Arduino Opta Controller Wiring

### Power Supply
```
12V DC Supply ────┬─── Arduino Opta VIN
                  ├─── Hydraulic Valve Commons
                  └─── Flow Control Valve (+)

GND ──────────────┬─── Arduino Opta GND
                  ├─── Hydraulic Valve Commons
                  └─── Flow Control Valve (-)
```

### Digital I/O Extension (D1608S) Connections
```
Arduino Opta D1608S          Hydraulic Valves
┌─────────────────────┐      ┌──────────────────────────┐
│ D1 ─────────────────────── │ Left Track Forward       │
│ D2 ─────────────────────── │ Left Track Backward      │
│ D3 ─────────────────────── │ Right Track Forward      │
│ D4 ─────────────────────── │ Right Track Backward     │
│ D5 ─────────────────────── │ Arms Up                  │
│ D6 ─────────────────────── │ Arms Down                │
│ D7 ─────────────────────── │ Bucket Up                │
│ D8 ─────────────────────── │ Bucket Down              │
└─────────────────────┘      └──────────────────────────┘
```

### Analog Extension (A0602) Connections
```
Arduino Opta A0602           Flow Control Valve
┌─────────────────────┐      ┌──────────────────────────┐
│ O2+ (4-20mA) ───────────── │ Current Loop Input (+)   │
│ O2- (4-20mA) ───────────── │ Current Loop Input (-)   │
│ +12V ───────────────────── │ Power Supply (+)         │
│ GND ────────────────────── │ Power Supply (-)         │
└─────────────────────┘      └──────────────────────────┘
```

## ESP32 Remote Control Wiring

### Power and Basic Connections
```
Battery Pack (7.4V Li-Po)
├─── ESP32 VIN
├─── Power Switch
└─── Charging Circuit

ESP32 Ground ─── Battery Ground
```

### Joystick Connections (Qwiic/I2C)
```
ESP32 Thing Plus            Left Joystick (0x20)
┌─────────────────────┐     ┌──────────────────────┐
│ SDA ────────────────────── │ SDA                  │
│ SCL ────────────────────── │ SCL                  │
│ 3.3V ───────────────────── │ 3.3V                 │
│ GND ────────────────────── │ GND                  │
└─────────────────────┘     └──────────────────────┘
                            
                            Right Joystick (0x21)
                            ┌──────────────────────┐
                            │ SDA (chained)        │
                            │ SCL (chained)        │
                            │ 3.3V (chained)       │
                            │ GND (chained)        │
                            └──────────────────────┘
```

### Status LED
```
ESP32 GPIO 2 ─── Built-in LED (status indication)
```

## Hydraulic Valve Specifications

### Directional Control Valves (4x)
- **Type:** 3-position, 4-way hydraulic directional valve
- **Flow Rate:** 20 GPM maximum
- **Size:** D03 NFPA mounting pattern
- **Voltage:** 12V DC solenoid actuation
- **Current:** ~1.5A per solenoid (typical)

### Valve Wiring Pattern
```
Valve Solenoid A ── Arduino Digital Output ── 12V Supply
Valve Solenoid B ── Arduino Digital Output ── 12V Supply
Valve Common ────── Ground
```

### Proportional Flow Control Valve
- **Input:** 4-20mA current loop (industrial standard)
- **Control:** 4-20mA from Arduino Opta A0602 O2 output
- **Response:** Flow rate proportional to current signal

### Burkert 8605 Type 316532 Flow Valve Controller
- **Power Supply:** 12V DC (compatible with existing system supply)
- **Control Interface:** 4-20mA current loop (industrial standard)
- **Flow Range:** Depends on connected valve size (up to 30 GPM typical)
- **Response Time:** <100ms typical
- **Connection:** Arduino Opta A0602 O2 output (4-20mA current loop)

### Burkert Controller 4-20mA Interface
```
Arduino Opta A0602           Burkert 8605 Controller
┌─────────────────────┐      ┌──────────────────────────┐
│ O2+ (4-20mA) ───────────── │ Current Input (+)        │
│ O2- (4-20mA) ───────────── │ Current Input (-)        │
│                     │      │ Power: 12V DC Supply     │
└─────────────────────┘      └──────────────────────────┘

4-20mA current loop provides superior noise immunity and precision
over PWM/voltage control for industrial applications.
```

## Network Infrastructure

### Raspberry Pi Connections
```
Raspberry Pi 4B
├─── Power: 5V 3A USB-C
├─── Network: WiFi or Ethernet
├─── Storage: 16GB+ MicroSD Card
└─── Optional: HDMI display for setup
```

### WiFi Router/Access Point
- **Standard:** 802.11n (2.4GHz) minimum, 802.11ac preferred
- **Range:** Ensure coverage of entire operating area
- **Security:** WPA2/WPA3 with strong password
- **DHCP:** Enabled for automatic IP assignment

## Cable Specifications

### Power Cables
- **12V Supply:** 12 AWG minimum for valve power
- **Control Signals:** 22-24 AWG shielded cable for digital signals
- **I2C/Qwiic:** Standard Qwiic cables (24 AWG, 4-conductor)

### Recommended Cable Lengths
- **Valve Control:** Maximum 50 feet (15m) for reliable 12V switching
- **Joystick I2C:** Maximum 10 feet (3m) for reliable I2C communication
- **Power:** As short as practical to minimize voltage drop

## Enclosure Requirements

### Arduino Opta Controller Enclosure
- **Rating:** IP65 minimum for outdoor use
- **Size:** Accommodate Opta + extensions + terminal blocks
- **Ventilation:** Consider heat dissipation
- **Access:** Cable glands for all connections

### ESP32 Remote Control Enclosure
- **Type:** Handheld controller case
- **Material:** Impact-resistant plastic
- **Features:** 
  - Joystick mounting holes
  - Battery compartment
  - Charging port access
  - Status LED visibility

## Safety Considerations

### Electrical Safety
- Use appropriate fuses on 12V supply lines
- Include emergency stop button that cuts all power
- Proper grounding of all metal enclosures
- GFCI protection if AC power is used

### Communication Safety
- Implement watchdog timers in software
- Physical emergency stop independent of communication
- Fail-safe valve positions (return to center on power loss)

### Installation Safety
- Secure all connections with appropriate strain relief
- Use weatherproof connectors for outdoor installations
- Label all cables and connections clearly
- Include circuit breakers for easy troubleshooting

## Testing Points

### Signal Testing
- Use multimeter to verify 12V at valve connections
- Oscilloscope to verify PWM signal on flow control
- I2C scanner to verify joystick addresses
- WiFi signal strength testing throughout operating area

### Functional Testing
- Test each valve individually before system integration
- Verify emergency stop function
- Test communication loss recovery
- Validate proportional flow control response

---

**Note:** This wiring diagram is for reference only. Always consult local electrical codes and hydraulic system requirements. Professional installation recommended for production systems.