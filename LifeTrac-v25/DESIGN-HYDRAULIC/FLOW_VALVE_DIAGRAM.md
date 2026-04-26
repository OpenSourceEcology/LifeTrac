# Flow Valve Configuration - Visual Diagrams

## System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    LifeTrac v25 System                      │
│                                                             │
│  ┌──────────┐      ┌──────────────┐      ┌──────────────┐ │
│  │ Joystick │ ───► │ Arduino Opta │ ───► │  Hydraulic   │ │
│  │  Input   │      │ Controller   │      │   Valves     │ │
│  └──────────┘      └──────────────┘      └──────────────┘ │
│                           │                                 │
│                           │ Configuration                   │
│                           ▼                                 │
│                    ┌──────────────┐                        │
│                    │ Flow Valves  │                        │
│                    │ (1 or 2)     │                        │
│                    └──────────────┘                        │
└─────────────────────────────────────────────────────────────┘
```

## Configuration Selection

```
Arduino Opta D1608S Extension
┌────────────────────────────────┐
│                                │
│  D9  [•]  Mode Switch A        │
│  D10 [•]  Mode Switch B        │
│                                │
│  D11 [•]────────[•] GND        │ ◄── Jumper Location
│       └──── Configuration       │
│                                │
└────────────────────────────────┘

NO JUMPER:          ONE_VALVE Mode  (Single valve controls all)
JUMPER INSTALLED:   TWO_VALVES Mode (Dual independent valves)
```

## Option 1: Single Valve Configuration (Default)

### Hardware Diagram
```
┌──────────────┐
│ Arduino Opta │
│              │
│  D11 [•]     │  ◄── No jumper (open)
│              │
│  O2  ───────────────┐
│  O3  [not used]     │
└──────────────┘      │
                      │
                      ▼
              ┌──────────────┐
              │   Burkert    │
              │ Controller   │
              └──────────────┘
                      │
                      ▼
              ┌──────────────┐
              │    Brand     │
              │ Hydraulics   │
              │ EFC Valve    │
              └──────────────┘
                      │
        ┌─────────────┼─────────────┐
        │             │             │
        ▼             ▼             ▼             ▼
   Left Track    Right Track     Arms        Bucket
   (Forward/     (Forward/     (Up/Down)   (Up/Down)
    Backward)     Backward)

ALL movements share the SAME flow rate
Speed = minimum non-zero joystick input across active functions
```

### Control Flow
```
Joystick Input:
  left_x:  -1.0 to 1.0 (turning)
  left_y:  -1.0 to 1.0 (forward/backward)
  right_x: -1.0 to 1.0 (bucket)
  right_y: -1.0 to 1.0 (arms)

Flow Calculation:
  minInput = min(non-zero |left_x|, |left_y|, |right_x|, |right_y|)
  
  O2 current = 4 + (minInput × 16) mA
  O3 current = 4 mA (off)

Result:
  All hydraulics limited to slowest commanded movement
```

### Turning Example
```
Input: left_y = 1.0, left_x = 0.5 (forward + right turn)

Track Speeds:
  Left:  1.0 + 0.5 = 1.0 (clamped)  ███████████ 100%
  Right: 1.0 - 0.5 = 0.5            █████       50%

Flow:
  O2: min(1.0, 0.5) = 0.5 → 12mA    █████       50%
  
Both tracks limited to slowest commanded speed (right track at 0.5)
All movements proportionally slower
```

## Option 2: Dual Valve Configuration (Advanced)

### Hardware Diagram
```
┌──────────────┐
│ Arduino Opta │
│              │
│  D11 [●]═════╪═══ GND  ◄── Jumper installed
│              │
│  O2  ───────────────┐
│              │      │
│  O3  ───────────────┼───┐
└──────────────┘      │   │
                      │   │
                      ▼   ▼
              ┌─────────────┐  ┌─────────────┐
              │  Burkert #1 │  │  Burkert #2 │
              │ Controller  │  │ Controller  │
              └─────────────┘  └─────────────┘
                      │                │
                      ▼                ▼
              ┌─────────────┐  ┌─────────────┐
              │   Brand     │  │   Brand     │
              │ Hydraulics  │  │ Hydraulics  │
              │ EFC Valve 1 │  │ EFC Valve 2 │
              └─────────────┘  └─────────────┘
                      │                │
            ┌─────────┴────┐    ┌─────┴──────┐
            │              │    │            │
            ▼              ▼    ▼            ▼
       Left Track       Arms  Right Track  Bucket
       (Forward/     (Up/Down) (Forward/  (Up/Down)
        Backward)              Backward)

INDEPENDENT flow control for each valve group
```

### Control Flow
```
Joystick Input:
  left_x:  -1.0 to 1.0 (turning)
  left_y:  -1.0 to 1.0 (forward/backward)
  right_x: -1.0 to 1.0 (bucket)
  right_y: -1.0 to 1.0 (arms)

Track Speed Calculation:
  leftTrackSpeed  = left_y + left_x
  rightTrackSpeed = left_y - left_x
  (both clamped to -1.0 to 1.0)

Flow Calculation:
  Valve 1: maxInput1 = max(|leftTrackSpeed|, |right_y|)
  Valve 2: maxInput2 = max(|rightTrackSpeed|, |right_x|)
  
  O2 current = 4 + (maxInput1 × 16) mA
  O3 current = 4 + (maxInput2 × 16) mA

Result:
  Independent flow control for each side
```

### Turning Example (Same Input)
```
Input: left_y = 1.0, left_x = 0.5 (forward + right turn)

Track Speeds:
  Left:  1.0 + 0.5 = 1.0 (clamped)  ███████████ 100%
  Right: 1.0 - 0.5 = 0.5            █████       50%

Flow:
  Valve 1: max(1.0) = 1.0 → 20mA    ███████████ 100%
  Valve 2: max(0.5) = 0.5 → 12mA    █████       50%
  
Each track gets appropriate flow rate
Smooth, controlled turning
```

### Zero-Radius Turn (Spin in Place)
```
Input: left_y = 0, left_x = 1.0 (full right turn, no forward)

Track Speeds:
  Left:  0 + 1.0 =  1.0 (forward)   ███████████► 100%
  Right: 0 - 1.0 = -1.0 (backward)  ◄███████████ 100%

Flow:
  Valve 1: max(1.0) = 1.0 → 20mA    ███████████ Full power
  Valve 2: max(1.0) = 1.0 → 20mA    ███████████ Full power
  
BOTH tracks get FULL INDEPENDENT POWER
Perfect zero-radius turn!

⚠️ Note: Zero-radius turning works in BOTH configurations, but:
- Single valve: Both tracks share the same 20mA flow (power split between them)
- Dual valve: Each track gets its own independent 20mA flow (full power to each)
```

## Comparison Chart

```
┌──────────────────┬────────────────────┬────────────────────┐
│    Feature       │   Single Valve     │    Dual Valve      │
├──────────────────┼────────────────────┼────────────────────┤
│ Hardware         │ 1 valve, simple    │ 2 valves, complex  │
│ Cost             │ Lower (baseline)   │ Higher (+$1000)    │
│ Flow Control     │ Shared             │ Independent        │
│ Turning Radius   │ Limited            │ Variable           │
│ Zero-Radius Turn │ Limited power      │ Full power         │
│ Installation     │ Easy               │ Moderate           │
│ Maneuvering      │ Basic              │ Advanced           │
└──────────────────┴────────────────────┴────────────────────┘
```

## Code Flow Diagram

```
┌─────────────────────────────────────────────────────────┐
│                    STARTUP                              │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │ Read D11 Jumper Pin   │
              └───────────────────────┘
                          │
            ┌─────────────┴─────────────┐
            │                           │
       D11 = HIGH                  D11 = LOW
    (No jumper)              (Jumper to GND)
            │                           │
            ▼                           ▼
    ┌───────────────┐          ┌────────────────┐
    │  ONE_VALVE    │          │  TWO_VALVES    │
    │     Mode      │          │      Mode      │
    └───────────────┘          └────────────────┘
            │                           │
            └─────────────┬─────────────┘
                          │
┌─────────────────────────────────────────────────────────┐
│                    MAIN LOOP                            │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │ Read Joystick Input   │
              │  (left_x, left_y,     │
              │   right_x, right_y)   │
              └───────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │ Calculate Track Speeds│
              │  left = base + turn   │
              │  right = base - turn  │
              └───────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │   Control Valves      │
              │ (Forward/Backward)    │
              └───────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │   setFlowControl()    │
              └───────────────────────┘
                          │
            ┌─────────────┴─────────────┐
            │                           │
        ONE_VALVE                   TWO_VALVES
            │                           │
            ▼                           ▼
    ┌───────────────┐          ┌────────────────┐
    │ max = max of  │          │ val1 = max of  │
    │ all inputs    │          │ left + arms    │
    │               │          │                │
    │ O2 = 4+max×16 │          │ val2 = max of  │
    │ O3 = 4 (off)  │          │ right + bucket │
    └───────────────┘          │                │
                               │ O2 = 4+val1×16 │
                               │ O3 = 4+val2×16 │
                               └────────────────┘
```

## Signal Flow Example

### Single Valve: Forward with Right Turn
```
Joystick:
  left_y = 1.0 ──┐
  left_x = 0.5 ──┤
                 ├──► Track Calculation ──► Left:  1.0 ──┐
  right_y = 0  ──┤                          Right: 0.5 ──┤
  right_x = 0  ──┘                                       │
                                                         │
                     Flow Calculation:                   │
                     max(1.0, 0.5, 0, 0) = 1.0          │
                              │                          │
                              ▼                          │
                        ┌──────────┐                    │
                        │ O2: 20mA │◄───────────────────┘
                        │ O3: 4mA  │
                        └──────────┘
                              │
                    ┌─────────┴─────────┐
                    │                   │
        Same 20mA flow to both tracks   │
        Left at 100%, Right at 50%      │
        (Right track underpowered)      │
```

### Dual Valve: Forward with Right Turn
```
Joystick:
  left_y = 1.0 ──┐
  left_x = 0.5 ──┤
                 ├──► Track Calculation ──► Left:  1.0 ──┐
  right_y = 0  ──┤                          Right: 0.5 ──┤
  right_x = 0  ──┘                                       │
                                                         │
                     Flow Calculation:                   │
                     Valve 1: max(1.0, 0) = 1.0         │
                     Valve 2: max(0.5, 0) = 0.5         │
                              │                          │
                    ┌─────────┴──────────┐              │
                    │                    │              │
                    ▼                    ▼              │
              ┌──────────┐         ┌──────────┐        │
              │ O2: 20mA │         │ O3: 12mA │◄───────┘
              └──────────┘         └──────────┘
                    │                    │
        Independent flow control         │
        Left: 100% (20mA)                │
        Right: 50% (12mA)                │
        (Perfect power distribution)     │
```

## Installation Checklist

### Single Valve (Default)
- [ ] Upload new firmware to Arduino Opta
- [ ] Leave D11 jumper disconnected
- [ ] Verify O2 connected to Burkert controller
- [ ] Power on system
- [ ] Check serial: "ONE_VALVE (Single valve for all)"
- [ ] Test all movements

### Dual Valve (Advanced)
- [ ] Install second Brand Hydraulics EFC valve
- [ ] Install second Burkert 8605 controller
- [ ] Connect Burkert #1 to O2 output
- [ ] Connect Burkert #2 to O3 output
- [ ] Configure hydraulics: Valve 1 → Left+Arms, Valve 2 → Right+Bucket
- [ ] Install jumper: D11 to GND
- [ ] Upload firmware to Arduino Opta
- [ ] Power on system
- [ ] Check serial: "TWO_VALVES (Valve 1: left+arms, Valve 2: right+bucket)"
- [ ] Test basic movements
- [ ] Test zero-radius turning
- [ ] Verify independent speed control

---

**For complete documentation, see FLOW_VALVE_CONFIGURATION.md**
