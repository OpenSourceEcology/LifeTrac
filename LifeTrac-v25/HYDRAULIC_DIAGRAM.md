# LifeTrac v25 - ASCII Hydraulic Diagram

This document provides an ASCII text art representation of the LifeTrac v25 hydraulic system, showing the flow paths, components, and connections.

## System Overview

The LifeTrac v25 hydraulic system consists of a pressurized reservoir, pump, directional control valves, hydraulic motors for track drive, hydraulic cylinders for arms and bucket, proportional flow control, and a cooler for temperature management.

## Complete Hydraulic System Diagram

```
                           LifeTrac v25 Hydraulic System
                                                                    
    ┌─────────────────┐                                    ┌─────────────────┐
    │   RESERVOIR     │                                    │     COOLER      │
    │  ╔═══════════╗  │                                    │  ╔═══════════╗  │
    │  ║           ║  │                                    │  ║    ░░░    ║  │
    │  ║  ████████  ║  │◄───────────────────────────────────┤  ║   ░░░░░   ║  │
    │  ║  ████████  ║  │                                    │  ║    ░░░    ║  │
    │  ║  ████████  ║  │                                    │  ╚═══════════╝  │
    │  ╚═══════════╝  │                                    └─────────────────┘
    └─────────┬───────┘                                              ▲
              │ Suction                                              │ Return
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
    │  ║   ◄═══►   ║──┼──────────┬─────────────┬─────────────┬──────┤
    │  ║     ◄     ║  │          │             │             │      │
    │  ╚═══════════╝  │          │             │             │      │
    └─────────────────┘          │             │             │      │
                                 │             │             │      │
         ┌───────────────────────┴──┐      ┌───┴──┐      ┌───┴──┐   │
         │   LEFT TRACK VALVE       │      │ ARMS │      │BUCKET│   │
         │        (D1/D2)           │      │VALVE │      │VALVE │   │
         │  ╔═══════════════════╗   │      │(D5/D6│      │(D7/D8│   │
         │  ║ ▲               ▲ ║   │      │  )   │      │  )   │   │
         │  ║ │               │ ║   │      │╔═══╗ │      │╔═══╗ │   │
         │  ║ │       ▲       │ ║   │      │║ ▲ ║ │      │║ ▲ ║ │   │
         │  ║ │      ▲▲▲      │ ║   │      │║▲│▲║ │      │║▲│▲║ │   │
         │  ║ │     ▲▲▲▲▲     │ ║   │      │║ ▼ ║ │      │║ ▼ ║ │   │
         │  ║ │               │ ║   │      │╚═══╝ │      │╚═══╝ │   │
         │  ║ ▼               ▼ ║   │      └──┬───┘      └──┬───┘   │
         │  ╚═══════════════════╝   │         │             │      │
         └──────┬─────────┬─────────┘         │             │      │
                │         │                   │             │      │
                │         │                   │             │      │
         ┌──────▼──┐   ┌──▼──────┐         ┌──▼───┐      ┌──▼───┐  │
         │ LEFT    │   │ RIGHT   │         │ ARMS │      │BUCKET│  │
         │ TRACK   │   │ TRACK   │         │ CYL. │      │ CYL. │  │
         │ MOTOR   │   │ VALVE   │         │      │      │      │  │
         │ ╔═════╗ │   │ (D3/D4) │         │ ╔══╗ │      │ ╔══╗ │  │
         │ ║  M  ║ │   │╔══════╗ │         │ ║▲▲║ │      │ ║▲▲║ │  │
         │ ║     ║ │   │║  ▲▲▲ ║ │         │ ║▲▲║ │      │ ║▲▲║ │  │
         │ ║     ║ │   │║ ▲▲▲▲▲║ │         │ ║▼▼║ │      │ ║▼▼║ │  │
         │ ║ ▲▲▲ ║ │   │║  ▼▼▼ ║ │         │ ║▼▼║ │      │ ║▼▼║ │  │
         │ ║▲▲▲▲▲║ │   │╚══════╝ │         │ ╚══╝ │      │ ╚══╝ │  │
         │ ║▲▲▲▲▲║ │   └──┬─────┬┘         └──────┘      └──────┘  │
         │ ║ ▼▼▼ ║ │      │     │                                  │
         │ ╚═════╝ │      │     │                                  │
         └─────────┘      │     │                                  │
                          │     │                                  │
                   ┌──────▼──┐  │                                  │
                   │ RIGHT   │  │                                  │
                   │ TRACK   │  │                                  │
                   │ MOTOR   │  │                                  │
                   │ ╔═════╗ │  │                                  │
                   │ ║  M  ║ │  │                                  │
                   │ ║     ║ │  │                                  │
                   │ ║     ║ │  │                                  │
                   │ ║ ▲▲▲ ║ │  │                                  │
                   │ ║▲▲▲▲▲║ │  │                                  │
                   │ ║▲▲▲▲▲║ │  │                                  │
                   │ ║ ▼▼▼ ║ │  │                                  │
                   │ ╚═════╝ │  │                                  │
                   └─────────┘  │                                  │
                                │                                  │
                                └──────────────────────────────────┘
                                              Return Line
```

## Component Descriptions

### Primary Components

- **RESERVOIR**: Stores hydraulic fluid and allows for thermal expansion
- **PUMP**: Pressurizes hydraulic fluid from reservoir
- **FLOW CONTROL VALVE (A0)**: Proportional valve controlling system flow rate
- **COOLER**: Heat exchanger to maintain optimal fluid temperature

### Control Valves

- **LEFT TRACK VALVE (D1/D2)**: 3-position, 4-way valve controlling left track motor
- **RIGHT TRACK VALVE (D3/D4)**: 3-position, 4-way valve controlling right track motor  
- **ARMS VALVE (D5/D6)**: 3-position, 4-way valve controlling arms cylinder
- **BUCKET VALVE (D7/D8)**: 3-position, 4-way valve controlling bucket cylinder

### Actuators

- **LEFT TRACK MOTOR**: Hydraulic motor driving left track
- **RIGHT TRACK MOTOR**: Hydraulic motor driving right track
- **ARMS CYLINDER**: Hydraulic cylinder for loader arms up/down
- **BUCKET CYLINDER**: Hydraulic cylinder for bucket curl/dump

## Flow Path Description

1. **Supply**: Pump draws fluid from reservoir and pressurizes the system
2. **Distribution**: Pressurized fluid flows through the proportional flow control valve
3. **Control**: Flow is distributed to four directional control valves
4. **Actuation**: Each valve directs flow to its respective motor or cylinder
5. **Return**: Used fluid returns through the cooler back to the reservoir

## Control Logic

- **Tank Steering**: Left/right track valves operate differentially for steering
- **Loader Operations**: Arms and bucket operate independently 
- **Speed Control**: Proportional flow valve regulates overall system speed
- **Safety**: All valves return to neutral position when de-energized

## Arduino Pin Assignments

```
Digital Outputs (12V DC Solenoids):
D1 - Left Track Forward    D5 - Arms Up
D2 - Left Track Backward   D6 - Arms Down  
D3 - Right Track Forward   D7 - Bucket Up
D4 - Right Track Backward  D8 - Bucket Down

Analog Output (PWM):
A0 - Flow Control Valve (0-10V via PWM + RC filter)
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