# LifeTrac v25 - Mechanical and Electronics Integration

This document describes how the mechanical design integrates with the existing LifeTrac v25 electronic control system.

## Overview

The LifeTrac v25 combines:
- **Mechanical System** (this directory): Physical structure, hydraulics, and wheels
- **Electronic System** (parent directory): Arduino controllers, remote control, web interface

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    LifeTrac v25 System                       │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌────────────────────┐         ┌─────────────────────┐    │
│  │   Remote Control   │         │   Web Controller    │    │
│  │   (ESP32/DroidPad) │         │   (Raspberry Pi)    │    │
│  └─────────┬──────────┘         └──────────┬──────────┘    │
│            │                               │                 │
│            │    WiFi/BLE/MQTT             │                 │
│            └───────────┬──────────────────┘                 │
│                        │                                     │
│              ┌─────────▼──────────┐                         │
│              │  Arduino Opta WiFi │                         │
│              │   Main Controller  │                         │
│              └─────────┬──────────┘                         │
│                        │                                     │
│         ┌──────────────┼──────────────┐                    │
│         │              │              │                     │
│    ┌────▼────┐    ┌───▼────┐    ┌───▼────┐               │
│    │Hydraulic│    │Hydraulic│    │Wheel   │               │
│    │Valves   │    │Flow     │    │Motors  │               │
│    │(4x)     │    │Control  │    │(4x)    │               │
│    └────┬────┘    └───┬────┘    └───┬────┘               │
│         │             │              │                      │
│    ┌────▼─────────────▼──────────────▼────┐               │
│    │      Mechanical System                │               │
│    │  • Base Frame                         │               │
│    │  • Loader Arms + Cylinders           │               │
│    │  • Bucket + Cylinders                │               │
│    │  • Wheels + Hydraulic Motors         │               │
│    └───────────────────────────────────────┘               │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Physical Integration Points

### 1. Control Housing (Part G1)

**Location:** Center of base frame (Part A1)  
**Purpose:** Houses all electronics and control systems

**Dimensions:**
- Width: 300mm
- Height: 400mm  
- Depth: 200mm
- Material: Sheet metal with 1/4" mounting plate

**Contains:**
- Arduino Opta WiFi controller
- Arduino Opta extensions (D1608S and A0602)
- Hydraulic valve controllers (Burkert 8605)
- WiFi antenna
- Wiring connections
- Optional battery backup

**Mounting:**
- Bolts to base frame cross member
- Vibration-isolated mounting recommended
- Weatherproof gaskets required
- Cable glands for wire entry

**Design Reference:** `openscad/lifetrac_v25.scad` module `control_housing()`

---

### 2. Hydraulic Valve Manifold

**Location:** Adjacent to control housing  
**Purpose:** Central hydraulic control point

**Components from Electronics BOM:**
- 4× Parker Hydraulic Directional Solenoid Valves (D1VW001CNKW)
- 8× Hirschmann DIN Connectors
- Manifold subplate (2-station or 4-station)
- Optional: 1-2× Proportional flow control valves (Brand PEFC12-20)
- Optional: 1-2× Burkert valve controllers

**Mechanical Interface:**
- Mounts to frame with 1/2" bolts
- Hydraulic hoses connect to cylinders and motors
- Electrical connectors to Arduino Opta relays/outputs

**Connection Mapping:**

| Valve | Purpose | Mechanical Connection | Arduino Output |
|-------|---------|----------------------|----------------|
| Valve 1 | Left wheel forward/reverse | B1 (Front Left Motor) | Relay 1-2 |
| Valve 2 | Right wheel forward/reverse | B2 (Front Right Motor) | Relay 3-4 |
| Valve 3 | Loader arm lift | D1-D2 (Lift Cylinders) | Relay 5-6 |
| Valve 4 | Bucket tilt | D3-D4 (Bucket Cylinders) | Relay 7-8 |

**Design Considerations:**
- Keep hose runs short to minimize pressure drop
- Use hose clamps to prevent rubbing
- Install pressure gauges for monitoring
- Add emergency pressure relief

---

### 3. Wheel Motors (Parts B1-B4)

**Type:** Hydraulic motors (100cc displacement)  
**Quantity:** 2 or 4 (configurable)

**Power Delivery:**
- Supply from hydraulic pump through valve manifold
- Proportional flow control adjusts speed
- Return to hydraulic reservoir

**Mechanical Mounting:**
- Bolted to wheel hub assemblies
- Shaft connects via key and set screw
- Motor case mounted to wheel mounting plate (A2)

**Electronic Control:**
- Direction: Controlled by solenoid valves
- Speed: Controlled by proportional flow valve PWM
- Tank steering: Differential speed between left/right

**Wiring:**
- No direct electrical connection to motors
- All control via hydraulic valves
- Sensors (optional): Hall effect for speed sensing

---

### 4. Hydraulic Cylinders (Parts D1-D4)

**Lift Cylinders (D1, D2):**
- Bore: 2.5" (63.5mm)
- Stroke: 16" (400mm)
- Function: Raise/lower loader arms

**Bucket Cylinders (D3, D4):**
- Bore: 2" (50mm)
- Stroke: 12" (300mm)
- Function: Tilt bucket

**Electronic Control:**
- Extend/retract via directional solenoid valves
- Speed controlled by proportional flow valves
- Position feedback (optional): Linear potentiometers

**Mechanical Interface:**
- Clevis mounts at both ends
- 1" pins for pivot points
- Hoses route to valve manifold
- Allow slack for full range of motion

---

### 5. Sensors and Feedback (Optional Additions)

**Recommended Sensors for Enhanced Control:**

| Sensor | Location | Purpose | Arduino Connection |
|--------|----------|---------|-------------------|
| Pressure transducers | Hydraulic lines | Monitor system pressure | Analog inputs |
| Linear potentiometers | Lift cylinders | Arm position feedback | Analog inputs |
| Rotary encoder | Wheel axle | Speed/distance tracking | Digital inputs |
| Tilt sensor | Base frame | Machine angle | Analog input |
| Emergency stop button | Standing deck | Safety shutdown | Digital input |

**Integration:**
- Connect to Arduino Opta Ext A0602 (analog inputs)
- Connect to Arduino Opta Ext D1608S (digital inputs)
- Configure in Arduino code for monitoring and safety

---

## Communication Systems

### WiFi/MQTT Mode

**Hardware:**
- Arduino Opta WiFi (built-in)
- Raspberry Pi running Mosquitto MQTT broker
- WiFi router (eero 6 recommended)

**Network Setup:**
```
Router (eero 6)
    │
    ├─── Arduino Opta (lifetrac/control/...)
    ├─── Raspberry Pi (MQTT broker + web controller)
    └─── Remote devices (phone/tablet/laptop)
```

**Physical Placement:**
- WiFi router: Near machine or on standing platform
- Raspberry Pi: Can be on machine or at base station
- Keep WiFi antenna away from metal structures

### BLE Mode

**Hardware:**
- Arduino Opta WiFi (built-in BLE)
- Smartphone/tablet with DroidPad app

**Physical Considerations:**
- BLE range: ~10-30 meters
- Metal frame may reduce signal
- External antenna may improve range
- No additional infrastructure needed

---

## Power System

### Hydraulic Power

**Source:** Engine-driven hydraulic pump  
**Specifications:**
- Flow: 15-20 GPM
- Pressure: 2500 PSI
- Power: 15-20 HP engine

**Distribution:**
- From pump → to valve manifold → to motors/cylinders
- Return lines → filter → reservoir → pump

**Mechanical Considerations:**
- Pump location: Front of machine for weight distribution
- Reservoir: 25-30 gallon, mounted low for stability
- Engine mounting: Vibration isolation required
- Cooling: Ensure adequate airflow

### Electrical Power

**Source:** Engine alternator or separate battery  
**Requirements:**
- 12V DC system
- Minimum 20A for all electronics
- Recommended: 50Ah deep cycle battery

**Distribution:**
```
Battery → Main fuse/breaker → Distribution:
    ├─ Arduino Opta (12V)
    ├─ Hydraulic valve solenoids (12V, high current)
    ├─ Valve controllers (12V)
    ├─ WiFi router (via DC converter)
    └─ Raspberry Pi (via DC-DC to 5V)
```

**Mechanical Integration:**
- Battery location: Low and central for weight
- Wiring: Protected conduit along frame
- Fusing: Individual circuits for safety
- Weatherproofing: All connections sealed

---

## Cable Routing

### High-Level Routes

**From Control Housing to:**

1. **Valve Manifold** (2m, shielded cable)
   - 4× valve control signals (12V PWM)
   - 2× flow controller signals (PWM)
   - Power and ground

2. **Wheel Motors** (varies, no direct connection)
   - Controlled via hydraulic valves only
   - Optional: Speed sensors (2 wires per motor)

3. **Hydraulic Cylinders** (varies, optional sensors)
   - Optional: Position sensors (3 wires per cylinder)

4. **Standing Deck** (2m)
   - Optional: Emergency stop button
   - Optional: Manual control switches

5. **External Connections** (varies)
   - WiFi antenna extension
   - External sensors
   - Charging port

### Cable Management

**Best Practices:**
- Use cable ties every 300mm
- Avoid sharp bends
- Keep away from moving parts
- Protect from hydraulic fluid
- Use split loom conduit
- Label all cables

**Design Support:**
- Cable clips welded to frame
- Routing channels in square tubing
- Grommets at entry points
- Weather-sealed connectors

---

## Assembly Integration Sequence

When assembling the complete LifeTrac v25:

### Phase 1: Mechanical Frame
1. Build base frame (A1)
2. Install wheel mounting plates (A2)
3. Mount wheels (B1-B4)

### Phase 2: Hydraulics - Mechanical
4. Install hydraulic pump and reservoir
5. Mount valve manifold to frame
6. Install cylinders (D1-D4)
7. Route all hydraulic hoses

### Phase 3: Mechanical - Loader System
8. Install loader arms (C1)
9. Install bucket attachment (C2)
10. Assemble and mount bucket (E1)
11. Connect cylinder hoses

### Phase 4: Electronics Integration
12. Install control housing (G1)
13. Mount Arduino Opta and extensions
14. Wire valve controllers to manifold
15. Connect power distribution
16. Install WiFi/communication hardware

### Phase 5: Optional Components
17. Install standing deck (F1)
18. Add sensors if using
19. Install emergency stop

### Phase 6: Testing
20. Test hydraulic system (no load)
21. Test electronics (no hydraulics)
22. Test integrated system
23. Calibrate and tune

---

## Maintenance Considerations

### Access Points

**For Mechanical Maintenance:**
- All hydraulic fittings accessible with wrenches
- Wheel bearings accessible from outside
- Cylinder mounting pins have clear access
- Bucket detaches for maintenance

**For Electronics Maintenance:**
- Control housing opens from top
- All connectors labeled and accessible
- Test points on Arduino visible
- USB programming port accessible

### Regular Maintenance Integration

**Mechanical + Electronics:**
- Inspect all electrical connections during mechanical checks
- Test sensors during hydraulic tests
- Verify controller operation before full system test
- Clean sensors when checking cylinders

---

## Troubleshooting Matrix

| Symptom | Possible Mechanical Cause | Possible Electronic Cause |
|---------|---------------------------|---------------------------|
| No movement | Hydraulic leak, broken pump | Controller not powered, valve failure |
| One wheel not moving | Motor failure, hose kinked | Valve control signal lost |
| Loader arms slow | Low hydraulic pressure | Flow valve not opening |
| Erratic control | Mechanical binding | Wireless interference |
| Total failure | Hydraulic system failure | Power loss, controller crash |

---

## Design Files Cross-Reference

### Mechanical Design
- Main assembly: `openscad/lifetrac_v25.scad`
- Parts list: `documentation/PARTS_LIST.md`
- Assembly: `documentation/ASSEMBLY.md`
- BOM: `documentation/BILL_OF_MATERIALS.md`

### Electronics Design
- Main README: `../README.md`
- Installation: `../INSTALLATION_GUIDE.md`
- Wiring: `../WIRING_DIAGRAM.md`
- Hydraulics: `../HYDRAULIC_DIAGRAM.md`
- Arduino code: `../arduino_opta_controller/`

---

## Future Enhancements

### Potential Mechanical/Electronic Integrations

1. **Automated Position Control**
   - Add position sensors to all cylinders
   - Implement closed-loop control
   - Enable programmable operations

2. **Telemetry System**
   - Log all operations
   - Track machine hours
   - Monitor health metrics

3. **Advanced Safety**
   - Proximity sensors
   - Load monitoring
   - Automatic stability control

4. **Vision System**
   - Camera on Raspberry Pi (already available)
   - Computer vision for guidance
   - Object detection for safety

---

**Document Version:** 1.0  
**Last Updated:** 2025-12-06  
**Status:** Integration guidance for complete system
