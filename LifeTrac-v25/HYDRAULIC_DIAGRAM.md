# LifeTrac v25 - ASCII Hydraulic Diagram

This document provides an ASCII text art representation of the LifeTrac v25 hydraulic system, showing the flow paths, components, and connections.

## System Overview

The LifeTrac v25 hydraulic system consists of a pressurized reservoir, pump, proportional flow control valve, manifold block for valve mounting, directional control valves, hydraulic motors for track drive, hydraulic cylinders for arms and bucket, and a cooler for temperature management.

## Complete Hydraulic System Diagram

```
                           LifeTrac v25 Hydraulic System
                                                                    
    ┌──────────────────┐                                    ┌─────────────────┐
    │     RESERVOIR    │                                    │     COOLER      │
    │  ╔════════════╗  │                                    │  ╔═══════════╗  │
    │  ║            ║  │                                    │  ║    ░░░    ║  │
    │  ║  ████████  ║  │◄───────────────────────────────────┤  ║   ░░░░░   ║  │
    │  ║  ████████  ║  │                                    │  ║    ░░░    ║  │
    │  ║  ████████  ║  │                                    │  ╚═══════════╝  │
    │  ╚════════════╝  │                                    └─────────────────┘
    └─────────┬────────┘                                             ▲
              │                                                      │ Return
              ▼                                                      │
    ┌─────────────────┐                                              │
    │      PUMP       │                                              │
    │   ╔═════════╗   │                                              │
    │   ║    ▲    ║   │                                              │
    │   ║   ▲▲▲   ║   │                                              │
    │   ║  ▲▲▲▲▲  ║   │                                              │
    │   ╚═════════╝   │                                              │
    └─────────┬───────┘                                              │
              │ Pressure                                             │
              ▼                                                      │
    ┌─────────────────┐                                              │
    │ FLOW CONTROL    │                                              │
    │ VALVE (A0)      │                                              │
    │  ╔═══════════╗  │                                              │
    │  ║     ►     ║  │                                              │
    │  ║   ◄═══►   ║──┼──────────────────────────────────────────────│
    │  ║     ◄     ║  │                                              │
    │  ╚═══════════╝  │                                              │
    └────────┬────────┘                                              │  
             │                                                       │
             ▼                                                       │ 
    ┌───────────────────────────────────────────┐                    │
    │                                           │                    │
    │                  MANIFOLD                 │────────────────────│
    │                                           │                    │
    └──┬───▼───┬─┬───▼───┬─┬───▼───┬─┬───▼───┬──┘                    │
       │ LFT   │ │ RIGHT │ │ ARMS  │ │BUCKET │                       │
       │ TRK   │ │ TRACK │ │ VALVE │ │ VALVE │                       │
       │ VLV   │ │ VALVE │ │(D5/D6)│ │(D7/D8)│                       │
       │ D1/D2 │ │(D3/D4)│ │       │ │       │                       │
       │       │ │       │ │       │ │       │                       │
       │ ╔═══╗ │ │ ╔═══╗ │ │ ╔═══╗ │ │ ╔═══╗ │                       │
       │ ║ ▲ ║ │ │ ║ ▲ ║ │ │ ║ ▲ ║ │ │ ║ ▲ ║ │                       │
       │ ║▲│▲║ │ │ ║▲│▲║ │ │ ║▲│▲║ │ │ ║▲│▲║ │                       │
       │ ║ ▼ ║ │ │ ║ ▼ ║ │ │ ║ ▼ ║ │ │ ║ ▼ ║ │                       │
       │ ╚═══╝ │ │ ╚═══╝ │ │ ╚═══╝ │ │ ╚═══╝ │                       │
       │ A   B │ │ A   B │ │ A   B │ │ A   B │                       │
       └─┬───┬─┘ └─┬───┬─┘ └─┬───┬─┘ └─┬───┬─┘                       │
         │   │     │   │     │   │     │   │                         │
       ┌─▼───▼─┐ ┌─▼───▼─┐ ┌─▼───▼─┐ ┌─▼───▼─┐                       │
       │  LEFT │ │ RIGHT │ │ ARMS  │ │ BUCKET│                       │
       │ TRCK  │ │ TRACK │ │ CYL.  │ │ CYL.  │                       │
       │ MOTR  │ │ MOTOR │ │  A B  │ │  A B  │                       │
       │╔═════╗│ │╔═════╗│ │ ╔▼═▲╗ │ │ ╔▼═▲╗ │                       │
       │║  M  ║│ │║  M  ║│ │ ║▲ ▲║ │ │ ║▲ ▲║ │                       │
       │║  ▲  ║│ │║  ▲  ║│ │ ║▲ ▲║ │ │ ║▲ ▲║ │                       │
       │║  ▲  ║│ │║  ▲  ║│ │ ║▼ ▼║ │ │ ║▼ ▼║ │                       │
       │║ ▲▲▲ ║│ │║ ▲▲▲ ║│ │ ║▼ ▼║ │ │ ║▼ ▼║ │                       │
       │║▲▲▲▲▲║│ │║▲▲▲▲▲║│ │ ╚▲═▼╝ │ │ ╚▲═▼╝ │                       │
       │║▲▲▲▲▲║│ │║▲▲▲▲▲║│ └───────┘ └───────┘                       │
       │║ ▼▼▼ ║│ │║ ▼▼▼ ║│                                           │
       │╚══▲══╝│ │╚══▲══╝│                                           │
       └───▼───┘ └───▼───┘                                           │
           │         │                                               │
           │         │                                               │
           └─────────────────────────────────────────────────────────┘
                                    Return Line
```

## Component Descriptions

### Primary Components

- **RESERVOIR**: Stores hydraulic fluid and allows for thermal expansion
- **PUMP**: Pressurizes hydraulic fluid from reservoir
- **FLOW CONTROL VALVE (A0)**: Proportional valve controlling system flow rate with return line for excess flow
- **MANIFOLD BLOCK**: Distribution block where all directional control valves are mounted, includes return manifold
- **COOLER**: Heat exchanger to maintain optimal fluid temperature

### Control Valves

- **LEFT TRACK VALVE (D1/D2)**: 3-position, 4-way valve controlling left track motor via A and B ports
- **RIGHT TRACK VALVE (D3/D4)**: 3-position, 4-way valve controlling right track motor via A and B ports
- **ARMS VALVE (D5/D6)**: 3-position, 4-way valve controlling arms cylinder via A and B ports
- **BUCKET VALVE (D7/D8)**: 3-position, 4-way valve controlling bucket cylinder via A and B ports

### Actuators

- **LEFT TRACK MOTOR**: Hydraulic motor driving left track (A and B ports for bi-directional rotation)
- **RIGHT TRACK MOTOR**: Hydraulic motor driving right track (A and B ports for bi-directional rotation)
- **ARMS CYLINDER**: Hydraulic cylinder for loader arms up/down (A port extends, B port retracts)
- **BUCKET CYLINDER**: Hydraulic cylinder for bucket curl/dump (A port extends, B port retracts)

## Flow Path Description

1. **Supply**: Pump draws fluid from reservoir and pressurizes the system
2. **Distribution**: Pressurized fluid flows through the proportional flow control valve
3. **Manifold**: Flow is distributed through the manifold block to all directional control valves
4. **Control**: Each valve directs flow to its respective motor or cylinder via A and B ports
5. **Actuation**: Motors and cylinders perform their designated functions
6. **Return**: Used fluid returns from actuators through a single consolidated return line
7. **Flow Control Return**: Excess flow from the proportional flow control valve returns to manifold
8. **Manifold Return**: Combined return flow exits manifold and passes through cooler back to reservoir

## Control Logic

- **Tank Steering**: Left/right track valves operate differentially for steering
- **Loader Operations**: Arms and bucket operate independently 
- **Speed Control**: Proportional flow valve regulates overall system speed
- **Safety**: All valves return to neutral position when de-energized
- **Port Control**: Each 4-way valve has A and B ports that control actuator direction:
  - **Motors**: A port = forward rotation, B port = reverse rotation
  - **Cylinders**: A port = extend, B port = retract
- **Return System**: Single consolidated return line collects fluid from all actuators and returns to reservoir via cooler

## Arduino Pin Assignments

```
Digital Outputs (12V DC Solenoids):
D1 - Left Track Forward    D5 - Arms Up
D2 - Left Track Backward   D6 - Arms Down  
D3 - Right Track Forward   D7 - Bucket Up
D4 - Right Track Backward  D8 - Bucket Down

Analog Output (PWM):
D9 (PWM) - Flow Control Valve (requires 0-10V output: use PWM + RC filter and op-amp or external driver)
```

## Specifications

- **System Pressure**: Variable based on load requirements
- **Flow Rate**: Up to 20 GPM per valve  
- **Valve Type**: 3-position, 4-way, D03 NFPA mounting
- **Motor Size**: Suitable for track drive applications
- **Cylinder Type**: Double-acting hydraulic cylinders
- **Control Voltage**: 12V DC for valve solenoids
- **Flow Control**: 0-10V proportional control signal

---

**Note**: This diagram represents the hydraulic circuit topology. Actual component sizing, pressure ratings, and flow rates should be determined based on specific application requirements and safety factors.
