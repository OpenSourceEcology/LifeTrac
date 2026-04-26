# Deceleration Feature Test Scenarios

## Test Setup
Before testing, ensure:
- Arduino Opta controller is properly connected
- Hydraulic system is functional
- All safety systems are operational
- Test area is clear and safe

## Test Scenarios

### Test 1: Basic Wheel Deceleration - Full Speed
**Objective**: Verify 2-second deceleration at full speed

**Steps**:
1. Push left joystick forward to maximum (Y = 1.0)
2. Wait for wheels to reach full speed
3. Release joystick to center (Y = 0.0)
4. Observe deceleration behavior

**Expected Result**:
- Hydraulic solenoid valves remain engaged
- Proportional flow valve gradually reduces over 2 seconds
- Wheels come to smooth stop after 2 seconds
- No jerky motion

### Test 2: Basic Wheel Deceleration - Half Speed
**Objective**: Verify 1-second deceleration at half speed

**Steps**:
1. Push left joystick forward to ~50% (Y = 0.5)
2. Wait for wheels to stabilize
3. Release joystick to center (Y = 0.0)
4. Observe deceleration behavior

**Expected Result**:
- Smooth deceleration over 1 second
- No jerky motion

### Test 3: Basic Wheel Deceleration - 25% Speed
**Objective**: Verify 0.5-second deceleration at 25% speed

**Steps**:
1. Push left joystick forward to ~25% (Y = 0.25)
2. Wait for wheels to stabilize
3. Release joystick to center (Y = 0.0)
4. Observe deceleration behavior

**Expected Result**:
- Smooth deceleration over 0.5 seconds

### Test 4: Arms Deceleration - Full Speed
**Objective**: Verify 1-second deceleration for arms at full speed

**Steps**:
1. Push right joystick up to maximum (Y = 1.0)
2. Wait for arms to move
3. Release joystick to center (Y = 0.0)
4. Observe deceleration behavior

**Expected Result**:
- Smooth deceleration over 1 second
- Half the time of wheels at same speed

### Test 5: Arms Deceleration - Half Speed
**Objective**: Verify 0.5-second deceleration for arms at half speed

**Steps**:
1. Push right joystick up to ~50% (Y = 0.5)
2. Wait for arms to stabilize
3. Release joystick to center (Y = 0.0)
4. Observe deceleration behavior

**Expected Result**:
- Smooth deceleration over 0.5 seconds

### Test 6: Arms Deceleration - 25% Speed
**Objective**: Verify 0.25-second deceleration for arms at 25% speed

**Steps**:
1. Push right joystick up to ~25% (Y = 0.25)
2. Wait for arms to stabilize
3. Release joystick to center (Y = 0.0)
4. Observe deceleration behavior

**Expected Result**:
- Smooth deceleration over 0.25 seconds

### Test 7: Mixed Mode - No Deceleration
**Objective**: Verify deceleration is skipped in mixed mode

**Steps**:
1. Push both joysticks (left Y and right Y) simultaneously
2. Release only left joystick to center
3. Observe behavior

**Expected Result**:
- Wheels stop immediately (no deceleration)
- Arms continue moving based on right joystick

**Rationale**: When multiple inputs are active, deceleration is disabled to allow quick transitions

### Test 8: Emergency Stop Override
**Objective**: Verify emergency stop bypasses deceleration

**Steps**:
1. Push left joystick forward to full speed
2. Wait for wheels to reach full speed
3. Trigger emergency stop (disconnect controller or wait for timeout)
4. Observe behavior

**Expected Result**:
- All movement stops immediately
- No deceleration applied
- All solenoid valves close immediately

### Test 9: Deceleration Cancellation
**Objective**: Verify new input cancels ongoing deceleration

**Steps**:
1. Push left joystick forward to full speed
2. Release joystick (start deceleration)
3. After 0.5 seconds, push joystick forward again
4. Observe behavior

**Expected Result**:
- Deceleration cancels when new input is detected
- Wheels respond immediately to new input
- No lag or delay

### Test 10: Turning Deceleration
**Objective**: Verify deceleration works for turning (left joystick X)

**Steps**:
1. Push left joystick right to maximum (X = 1.0)
2. Wait for turning to stabilize
3. Release joystick to center
4. Observe deceleration behavior

**Expected Result**:
- Smooth deceleration over 2 seconds (full speed)
- Differential steering gradually reduces

### Test 11: Bucket Deceleration
**Objective**: Verify deceleration works for bucket (right joystick X)

**Steps**:
1. Push right joystick right to maximum (X = 1.0)
2. Wait for bucket to move
3. Release joystick to center
4. Observe deceleration behavior

**Expected Result**:
- Smooth deceleration over 1 second (full speed for arms/bucket)

### Test 12: Enable/Disable Feature
**Objective**: Verify ENABLE_DECELERATION flag works correctly

**Steps**:
1. Set `ENABLE_DECELERATION = false` in code
2. Upload to controller
3. Test any movement (e.g., wheels at full speed)
4. Release joystick
5. Observe behavior

**Expected Result**:
- Immediate stop (no deceleration)
- System behaves like pre-deceleration implementation

**Steps to Re-enable**:
1. Set `ENABLE_DECELERATION = true`
2. Upload to controller
3. Retest - deceleration should work again

### Test 13: Rapid Start-Stop Cycles
**Objective**: Verify system handles rapid input changes

**Steps**:
1. Rapidly toggle left joystick between forward and center
2. Perform quick start-stop-start-stop cycles
3. Observe system stability

**Expected Result**:
- System remains stable
- Each deceleration properly starts and cancels
- No stuck states or unresponsive behavior

### Test 14: All Axes Simultaneous Deceleration
**Objective**: Verify independent deceleration per axis

**Steps**:
1. Move all four axes (left X, left Y, right X, right Y)
2. Hold for 2 seconds
3. Release all joysticks simultaneously
4. Observe behavior

**Expected Result**:
- Mixed mode detected - immediate stop (no deceleration)
- All movement stops simultaneously

### Test 15: Sequential Single-Axis Deceleration
**Objective**: Verify each axis decelerates independently when used alone

**Steps**:
1. Move left Y axis only, release, observe
2. Wait for complete stop
3. Move left X axis only, release, observe
4. Wait for complete stop
5. Move right Y axis only, release, observe
6. Wait for complete stop
7. Move right X axis only, release, observe

**Expected Result**:
- Each axis decelerates independently
- Left Y: 2s (wheel, full speed)
- Left X: 2s (wheel, full speed)
- Right Y: 1s (arm, full speed)
- Right X: 1s (bucket, full speed)

## Monitoring and Debugging

### Serial Output
Monitor serial output for:
- "BLE Left: X=... Y=..." messages showing input values
- Deceleration state changes (if debug output added)
- Warning messages about out-of-range values

### Flow Valve Current Output
Monitor the 4-20mA current to verify:
- Current gradually decreases during deceleration
- Current reaches 4mA (BASE_CURRENT) after deceleration completes
- No abrupt changes during deceleration

### Visual/Physical Inspection
Observe:
- Smooth motion during deceleration
- No jerky movements
- Hydraulic oil flow gradually reducing
- Solenoid valves staying engaged during deceleration

## Troubleshooting

### Issue: Deceleration too long
**Solution**: Adjust duration constants in `calculateDecelerationDuration()`

### Issue: Deceleration too short
**Solution**: Increase duration constants in `calculateDecelerationDuration()`

### Issue: Jerky motion during deceleration
**Solution**: 
- Check hydraulic system for air in lines
- Verify proportional valve calibration
- Review linear interpolation calculation

### Issue: Mixed mode detection too sensitive
**Solution**: Adjust DEADZONE constant or modify `isInMixedMode()` logic

### Issue: Deceleration not working
**Checklist**:
- Verify `ENABLE_DECELERATION = true`
- Check that input actually goes to zero (within DEADZONE)
- Verify not in mixed mode
- Verify no emergency stop condition
- Check serial output for errors

## Safety Notes
- Always have emergency stop accessible during testing
- Test in a safe, controlled environment
- Start with low speeds before testing full speed
- Ensure area is clear of personnel during testing
- Monitor hydraulic system for leaks or unusual behavior
