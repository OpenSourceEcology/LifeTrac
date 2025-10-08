# LifeTrac v25 - Proportional Flow Valve Configuration Guide

This document describes the two proportional flow valve configuration options for the LifeTrac v25 hydraulic system and how to select between them using hardware jumpers.

## Overview

The LifeTrac v25 supports two different proportional flow valve configurations:

### Option 1: Single Valve (Default)
- **Hardware**: One proportional flow control valve controls flow to all hydraulic functions
- **Control**: All movements (left track, right track, arms, bucket) share the same flow rate
- **Speed**: Limited to the maximum joystick input across all functions
- **Turning**: Limited capability - cannot have different speeds for left and right tracks
- **Use Case**: Simpler installation, lower cost, adequate for basic operations

### Option 2: Dual Valve (Advanced)
- **Hardware**: Two proportional flow control valves for independent control
  - Valve 1 controls: Left track + Arms
  - Valve 2 controls: Right track + Bucket
- **Control**: Independent speed control for each valve group
- **Speed**: Each side can operate at different speeds simultaneously
- **Turning**: Fully adjustable turning with variable radius control
- **Use Case**: Advanced maneuvering, precise control, zero-radius turning

## Hardware Configuration

### Jumper Setup

The flow valve configuration is selected using a jumper on the Arduino Opta's D11 pin:

```
┌─────────────────────────────────────────────┐
│          Arduino Opta D1608S                │
│                                             │
│  D11 ──┐                                    │
│        │  [Jumper Position]                 │
│  GND ──┘                                    │
│                                             │
└─────────────────────────────────────────────┘

Configuration:
├── No Jumper (D11 open):    ONE_VALVE mode (default)
└── Jumper installed:        TWO_VALVES mode
    (D11 connected to GND)
```

### Pin Connections

#### Single Valve Configuration (Default)
```
Arduino Opta A0602               Flow Control System
┌─────────────────────┐         
│ O2+ (4-20mA) ─────────────┐   
│ O2- (4-20mA) ─────────────┼───► Burkert 8605 Controller #1
│                           │     └─► Brand Hydraulics EFC Valve
│ O3+ (4-20mA)    [Unused]  │         Controls ALL hydraulics
│ O3- (4-20mA)    [Unused]  │
└─────────────────────┘     
```

#### Dual Valve Configuration (Jumper on D11)
```
Arduino Opta A0602               Flow Control System
┌─────────────────────┐         
│ O2+ (4-20mA) ─────────────┐   
│ O2- (4-20mA) ─────────────┼───► Burkert 8605 Controller #1
│                           │     └─► Brand Hydraulics EFC Valve #1
│                           │         Controls: Left Track + Arms
│                           │
│ O3+ (4-20mA) ─────────────┐   
│ O3- (4-20mA) ─────────────┼───► Burkert 8605 Controller #2
│                           │     └─► Brand Hydraulics EFC Valve #2
│                           │         Controls: Right Track + Bucket
└─────────────────────┘     
```

## Control Behavior Differences

### Single Valve Mode (Default)

**Characteristics:**
- All hydraulic functions share a single flow rate
- System speed determined by the maximum joystick input across all axes
- Simple turning: full speed one side, no power to other side OR equal forward/backward on both sides

**Example Scenarios:**
```
Scenario 1: Forward driving with bucket movement
- Left joystick: Y=0.8 (forward)
- Right joystick: X=0.5 (bucket)
- Result: Flow rate = max(0.8, 0.5) = 0.8 (16.8mA)
- Both tracks move at same speed, bucket operates at same flow rate

Scenario 2: Turning
- Left joystick: Y=1.0 (forward), X=0.5 (right turn)
- Result: 
  - Left track: 1.0 + 0.5 = 1.0 (clamped to max)
  - Right track: 1.0 - 0.5 = 0.5
  - Flow rate: max(1.0, 0.5) = 1.0 (20mA)
  - Speed limited by single valve flow rate
```

### Dual Valve Mode (Jumper Installed)

**Characteristics:**
- Independent flow control for each valve group
- Left side (track + arms) can operate at different speed than right side (track + bucket)
- Advanced turning: variable speed differential between left and right
- Zero-radius turning capability

**Example Scenarios:**
```
Scenario 1: Forward driving with bucket movement
- Left joystick: Y=0.8 (forward), X=0
- Right joystick: X=0.5 (bucket), Y=0
- Result: 
  - Left track speed: 0.8 + 0 = 0.8
  - Right track speed: 0.8 - 0 = 0.8
  - Valve 1 (left track + arms): max(0.8, 0) = 0.8 (16.8mA)
  - Valve 2 (right track + bucket): max(0.8, 0.5) = 0.8 (16.8mA)
  - Both sides move forward, bucket operates independently

Scenario 2: Advanced turning with different speeds
- Left joystick: Y=1.0 (forward), X=0.5 (right turn)
- Result:
  - Left track speed: 1.0 + 0.5 = 1.0 (clamped to max)
  - Right track speed: 1.0 - 0.5 = 0.5
  - Valve 1 (left track): max(1.0) = 1.0 (20mA)
  - Valve 2 (right track): max(0.5) = 0.5 (12mA)
  - Left track runs at full speed, right at half speed
  - Creates smooth, variable-radius turn

Scenario 3: Zero-radius turning (spin in place)
- Left joystick: Y=0, X=1.0 (full right turn)
- Result:
  - Left track speed: 0 + 1.0 = 1.0 (forward)
  - Right track speed: 0 - 1.0 = -1.0 (backward)
  - Valve 1 (left track): max(1.0) = 1.0 (20mA forward)
  - Valve 2 (right track): max(1.0) = 1.0 (20mA backward)
  - Tank rotates in place with full power to both sides
  - This is only possible with independent valve control!
```

## Installation Instructions

### For Single Valve Configuration (Default)

1. **Do not install jumper on D11**
2. Connect Burkert 8605 Controller to Opta O2 output:
   - O2+ → Burkert Controller Input (+)
   - O2- → Burkert Controller Input (-)
3. Connect Burkert Controller to Brand Hydraulics EFC valve
4. Upload firmware to Arduino Opta
5. Power on system
6. Verify serial output shows: `ONE_VALVE (Single valve for all)`

### For Dual Valve Configuration (Advanced)

1. **Install jumper connecting D11 to GND**
   - Use a standard 2.54mm jumper or wire connection
   - Connect Arduino Opta D11 pin to any GND pin
2. Connect first Burkert 8605 Controller to Opta O2 output:
   - O2+ → Burkert Controller #1 Input (+)
   - O2- → Burkert Controller #1 Input (-)
3. Connect first Burkert Controller to Brand Hydraulics EFC Valve #1
4. Connect second Burkert 8605 Controller to Opta O3 output:
   - O3+ → Burkert Controller #2 Input (+)
   - O3- → Burkert Controller #2 Input (-)
5. Connect second Burkert Controller to Brand Hydraulics EFC Valve #2
6. **Configure hydraulic circuit:**
   - Route Valve #1 output to: Left track + Arms hydraulics
   - Route Valve #2 output to: Right track + Bucket hydraulics
7. Upload firmware to Arduino Opta
8. Power on system
9. Verify serial output shows: `TWO_VALVES (Valve 1: left+arms, Valve 2: right+bucket)`

## Hydraulic Circuit Modifications

### Single Valve Circuit (Default)
```
[Pump] → [Proportional Flow Valve #1] → [Flow Distribution]
                                            ├─► Left Track Forward/Backward
                                            ├─► Right Track Forward/Backward
                                            ├─► Arms Up/Down
                                            └─► Bucket Up/Down
```

### Dual Valve Circuit (Advanced)
```
[Pump] → [Flow Splitter] ─┬─► [Proportional Flow Valve #1] → [Group 1 Distribution]
                          │                                      ├─► Left Track Forward/Backward
                          │                                      └─► Arms Up/Down
                          │
                          └─► [Proportional Flow Valve #2] → [Group 2 Distribution]
                                                                 ├─► Right Track Forward/Backward
                                                                 └─► Bucket Up/Down
```

**Note:** Consult a qualified hydraulic technician for proper circuit design and installation.

## Additional Hardware Requirements

### Single Valve Configuration
- 1x Brand Hydraulics EFC Proportional Flow Control Valve
- 1x Burkert 8605 Type 316532 Flow Valve Controller
- Standard hydraulic distribution manifold

### Dual Valve Configuration (Additional)
- 1x Additional Brand Hydraulics EFC Proportional Flow Control Valve
- 1x Additional Burkert 8605 Type 316532 Flow Valve Controller
- Hydraulic flow splitter (to divide pump output)
- Two separate distribution manifolds (one per valve group)
- Additional hydraulic hoses and fittings

## Advantages and Disadvantages

### Single Valve (Default)

**Advantages:**
✅ Lower cost (one valve system)
✅ Simpler hydraulic circuit
✅ Easier installation and maintenance
✅ Adequate for most operations
✅ Less power consumption

**Disadvantages:**
❌ Limited turning capability
❌ Cannot have different speeds for left/right sides
❌ Speed limited to slowest required movement
❌ Basic maneuverability

### Dual Valve (Advanced)

**Advantages:**
✅ Independent left/right speed control
✅ Fully adjustable turning radius
✅ Zero-radius turning (spin in place)
✅ More precise maneuvering
✅ Better control in tight spaces
✅ Can perform complex movements simultaneously

**Disadvantages:**
❌ Higher cost (two valve systems)
❌ More complex hydraulic circuit
❌ More installation and maintenance work
❌ Requires hydraulic flow splitter
❌ Higher power consumption

## Troubleshooting

### Configuration Not Detected Correctly

**Symptoms:**
- Serial output shows wrong configuration
- System behaves incorrectly

**Solutions:**
1. Check jumper connection on D11
   - Ensure good electrical contact
   - Verify jumper connects D11 to GND
2. Power cycle the Arduino Opta
3. Check serial output during startup
4. Measure voltage at D11 pin (should be 0V with jumper, 3.3V without)

### One Valve Not Working in Dual Mode

**Symptoms:**
- Only left or right side responds
- Partial system functionality

**Solutions:**
1. Verify jumper is installed on D11
2. Check O3 connections to second Burkert controller
3. Test 4-20mA output at O3 with multimeter
4. Verify second Burkert controller power supply
5. Check hydraulic connections to second valve

### Inconsistent Flow Control

**Symptoms:**
- Erratic speeds
- Unexpected behavior

**Solutions:**
1. Check all 4-20mA current loop connections
2. Verify Burkert controller wiring
3. Ensure proper grounding
4. Check for electrical interference
5. Verify hydraulic circuit is properly configured

## Testing Procedure

### Single Valve Configuration Test

1. Power on system
2. Check serial output: `ONE_VALVE (Single valve for all)`
3. Test forward movement (left joystick Y=1.0)
4. Test turning (left joystick X=±1.0)
5. Test arms and bucket movements
6. Verify all movements work but share same flow rate

### Dual Valve Configuration Test

1. Install jumper on D11
2. Power on system
3. Check serial output: `TWO_VALVES (Valve 1: left+arms, Valve 2: right+bucket)`
4. Test left track only (should see flow from Valve 1)
5. Test right track only (should see flow from Valve 2)
6. Test zero-radius turn (left track forward, right track backward)
7. Test independent arm and bucket movements
8. Verify smooth turning with variable radius

## Safety Considerations

⚠️ **Warning:** Changing flow valve configuration affects hydraulic system behavior

- Test new configuration in a safe, open area
- Start with slow movements to verify correct operation
- Ensure all personnel are clear of the machine during testing
- Have emergency stop readily accessible
- Consult qualified hydraulic technician for installation
- Follow all manufacturer safety guidelines

## Serial Debugging

During startup, the system will output configuration information:

**Single Valve Mode:**
```
LifeTrac v25 Controller Starting...
Flow valve configuration jumper: D11=LOW -> Config: ONE_VALVE (Single valve for all)
Mode switch: A=LOW B=LOW -> Mode: BLE
Controller initialized successfully!
```

**Dual Valve Mode:**
```
LifeTrac v25 Controller Starting...
Flow valve configuration jumper: D11=HIGH -> Config: TWO_VALVES (Valve 1: left+arms, Valve 2: right+bucket)
Mode switch: A=LOW B=LOW -> Mode: BLE
Controller initialized successfully!
```

## Technical Specifications

### 4-20mA Current Loop Outputs

| Output | Function | Configuration |
|--------|----------|---------------|
| O2 | Primary Flow Valve | Used in both modes |
| O3 | Secondary Flow Valve | Used only in TWO_VALVES mode |

### Current Signal Mapping

| Joystick Input | Current Output | Flow Rate |
|----------------|----------------|-----------|
| 0.0 (no input) | 4 mA | No flow |
| 0.1 (minimum) | 6 mA | ~12.5% flow |
| 0.5 (half) | 12 mA | 50% flow |
| 1.0 (maximum) | 20 mA | 100% flow |

### Response Time
- Configuration detection: At startup (<100ms)
- Flow valve response: <100ms typical (Burkert controller)
- System stabilization: 1 second after power-on

---

**Document Version:** 1.0
**Last Updated:** 2024
**Applies to:** LifeTrac v25 Firmware v1.0+
