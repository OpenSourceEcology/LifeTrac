# Quick Reference: Flow Valve Configuration

## Quick Decision Guide

**Choose Single Valve if:**
- ✅ Budget is limited
- ✅ Simple operations (forward, backward, basic turning)
- ✅ First-time installation
- ✅ Easier maintenance is priority

**Choose Dual Valve if:**
- ✅ Need precise maneuvering
- ✅ Require enhanced zero-radius turning with full power
- ✅ Work in tight spaces frequently
- ✅ Budget allows for advanced system

## Quick Setup

### Single Valve (Default)
```
1. Leave D11 jumper DISCONNECTED
2. Connect ONE Burkert controller to O2
3. Power on and verify: "ONE_VALVE (Single valve for all)"
```

### Dual Valve (Advanced)
```
1. Install jumper: D11 to GND
2. Connect FIRST Burkert controller to O2
3. Connect SECOND Burkert controller to O3
4. Configure hydraulics:
   - Valve 1 → Left track + Arms
   - Valve 2 → Right track + Bucket
5. Power on and verify: "TWO_VALVES (Valve 1: left+arms, Valve 2: right+bucket)"
```

## Hardware Jumper Location

```
Arduino Opta D1608S Digital Extension
┌────────────────────────────┐
│  ...                       │
│  D9  [•]  Mode Switch A    │
│  D10 [•]  Mode Switch B    │
│  D11 [•]──────[•] GND      │  ← Install jumper here for dual valve
│  ...                       │
└────────────────────────────┘
```

## Quick Troubleshooting

**Problem: Configuration not detected**
- Solution: Check jumper connection, power cycle

**Problem: Only one side works in dual mode**
- Solution: Verify O3 connections and second Burkert controller

**Problem: Erratic behavior**
- Solution: Check 4-20mA connections and grounding

## Cost Comparison

| Component | Single Valve | Dual Valve |
|-----------|--------------|------------|
| Flow Control Valves | 1x | 2x |
| Burkert Controllers | 1x | 2x |
| Installation Complexity | Simple | Moderate |
| Approximate Additional Cost | — | +$1000-1500 |

## See Full Documentation

For complete details, see: **FLOW_VALVE_CONFIGURATION.md**
