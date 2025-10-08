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

### Power Supply with HONEYWELL 2NT1-1 Mode Switch (On/Off/On)
```
                                    ┌─────────────────┐
                                    │  HONEYWELL      │
                                    │  2NT1-1 Switch  │
12V DC Supply ──────────────────────│  (On/Off/On)    │
                                    │  DPDT           │
                                    └─────────────────┘
                                           │
                          ┌────────────────┼────────────────┐
                          │                │                │
                       [MQTT]            [OFF]            [BLE]
                          │                │                │
                    ┌─────┴─────┐          X          ┌─────┴─────┐
                    │           │     (Open/No        │           │
                    │  12V to   │      Power)         │  12V to   │
                    │  Opta VIN │   (Center Pos)      │  Opta VIN │
                    │           │                     │           │
                    │  D9=HIGH  │                     │  D9=LOW   │
                    │  D10=LOW  │                     │  D10=LOW  │
                    └───────────┘                     └───────────┘

Note: If switch is not installed, D9 and D10 are LOW (internal pulldown) -> BLE mode (default)

12V DC Supply (after switch) ─┬─── Arduino Opta VIN
                              ├─── Hydraulic Valve Commons
                              └─── Flow Control Valve (+)

GND ──────────────────────────┬─── Arduino Opta GND
                              ├─── Hydraulic Valve Commons
                              ├─── Flow Control Valve (-)
                              └─── Mode Switch Common
```

### Digital I/O Extension (D1608S) Connections
```
Arduino Opta D1608S          Hydraulic Valves / Mode Switch / Flow Config
┌─────────────────────┐      ┌──────────────────────────┐
│ D1 ─────────────────────── │ Left Track Forward       │
│ D2 ─────────────────────── │ Left Track Backward      │
│ D3 ─────────────────────── │ Right Track Forward      │
│ D4 ─────────────────────── │ Right Track Backward     │
│ D5 ─────────────────────── │ Arms Up                  │
│ D6 ─────────────────────── │ Arms Down                │
│ D7 ─────────────────────── │ Bucket Up                │
│ D8 ─────────────────────── │ Bucket Down              │
│ D9 ─────────────────────── │ Mode Switch Pin A        │
│ D10 ────────────────────── │ Mode Switch Pin B        │
│ D11 ────────────────────── │ Flow Valve Config Jumper │
└─────────────────────┘      └──────────────────────────┘

Mode Switch Logic (HONEYWELL 2NT1-1):
- Position 1 (MQTT): D9=HIGH, D10=LOW
- Position 2 (OFF): No 12V power (hardware cutoff at center)
- Position 3 (BLE): D9=LOW, D10=LOW (default when switch not installed)

Flow Valve Configuration Jumper (D11):
- No jumper (D11=HIGH): ONE_VALVE mode - Single valve controls all (default)
- Jumper to GND (D11=LOW): TWO_VALVES mode - Independent valve control
  See FLOW_VALVE_CONFIGURATION.md for details



### Analog Extension (A0602) Connections

#### Single Valve Configuration (Default - No Jumper on D11)
```
Arduino Opta A0602           Flow Control System
┌─────────────────────┐      ┌──────────────────────────┐
│ O2+ (4-20mA) ───────────── │ Burkert 8605 Controller  │
│ O2- (4-20mA) ───────────── │ Current Input            │
│ +12V ───────────────────── │ Power Supply (+)         │
│ GND ────────────────────── │ Power Supply (-)         │
│                     │      └──────────────────────────┘
│ O3+ (4-20mA) [Unused]                  │
│ O3- (4-20mA) [Unused]                  │ Control Output
└─────────────────────┘                  ▼
                             ┌──────────────────────────┐
                             │ Brand Hydraulics EFC     │
                             │ Flow Control Valve       │
                             │ Controls ALL hydraulics  │
                             └──────────────────────────┘
```

#### Dual Valve Configuration (Jumper on D11)
```
Arduino Opta A0602           Flow Control System
┌─────────────────────┐      
│ O2+ (4-20mA) ───────────── Burkert 8605 Controller #1
│ O2- (4-20mA) ───────────── Current Input            
│ +12V ───────────────────── Power Supply (+)         
│ GND ────────────────────── Power Supply (-)         
│                     │      └──► Brand Hydraulics EFC Valve #1
│                     │           (Left Track + Arms)
│                     │      
│ O3+ (4-20mA) ───────────── Burkert 8605 Controller #2
│ O3- (4-20mA) ───────────── Current Input
│ +12V ───────────────────── Power Supply (+)
│ GND ────────────────────── Power Supply (-)
└─────────────────────┘      └──► Brand Hydraulics EFC Valve #2
                                  (Right Track + Bucket)
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

### Brand Hydraulics EFC Proportional Flow Control Valve
- **Model:** Electronically Adjustable Pressure Compensated Proportional Flow Control
- **Input:** Controlled by Burkert 8605 Controller
- **Control Signal:** 4-20mA from Arduino Opta A0602 O2 output via Burkert controller
- **Response:** Flow rate proportional to current signal
- **Flow Range:** Up to 20 GPM (system specification)

### Burkert 8605 Type 316532 Flow Valve Controller
- **Role:** Electronic controller for Brand Hydraulics EFC valve
- **Power Supply:** 12V DC (compatible with existing system supply)
- **Control Interface:** 4-20mA current loop input (industrial standard)
- **Output:** Proportional control signal to Brand Hydraulics valve coil
- **Flow Range:** Up to 30 GPM (controller capability)
- **Response Time:** <100ms typical
- **Connection:** Arduino Opta A0602 O2 output (4-20mA current loop)

### Burkert Controller to Brand Hydraulics Valve Interface
```
Arduino Opta A0602           Burkert 8605 Controller      Brand Hydraulics EFC Valve
┌─────────────────────┐      ┌──────────────────────┐      ┌────────────────────────┐
│ O2+ (4-20mA) ───────────── │ Current Input (+)    │      │                        │
│ O2- (4-20mA) ───────────── │ Current Input (-)    │      │                        │
│                     │      │                      │      │                        │
│                     │      │ Valve Control Output ─────► │ Proportional Coil     │
│                     │      │                      │      │ (Flow Control)        │
│                     │      │ Power: 12V DC Supply │      │                        │
└─────────────────────┘      └──────────────────────┘      └────────────────────────┘

Signal Flow:
1. Arduino O2 sends 4-20mA current signal to Burkert controller
2. Burkert controller converts current signal to proportional valve control
3. Burkert controller drives Brand Hydraulics EFC valve coil for flow control
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