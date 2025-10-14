# Deceleration Feature Documentation

## Overview
The deceleration feature provides smooth stopping of hydraulic movements by gradually reducing flow before closing the hydraulic solenoid valve. This prevents jerky stops and reduces mechanical stress on the system.

## Configuration
The feature can be enabled or disabled by changing the `ENABLE_DECELERATION` constant:
```cpp
bool ENABLE_DECELERATION = true; // Set to false to disable deceleration
```

## Deceleration Timing

### Wheels (left_x, left_y)
- **Full speed (≥75%)**: 2 seconds deceleration
- **Half speed (≥37.5%)**: 1 second deceleration
- **25% speed**: 0.5 seconds deceleration

### Arms and Bucket (right_x, right_y)
- **Full speed (≥75%)**: 1 second deceleration
- **Half speed (≥37.5%)**: 0.5 seconds deceleration
- **25% speed**: 0.25 seconds deceleration

## Behavior

### Normal Operation
1. When joystick input transitions from active to zero (within deadzone)
2. The system starts a deceleration sequence
3. Hydraulic solenoid valve stays engaged
4. Proportional flow valve gradually reduces commanded flow
5. After deceleration completes, the solenoid valve closes

### Conditions That Skip Deceleration

#### Mixed Mode
If multiple inputs are active simultaneously (e.g., moving wheels and arms at the same time), deceleration is disabled. The system stops immediately when inputs go to zero to allow quick transitions between combined movements.

#### Emergency Stop
If the safety timeout is exceeded (no commands received for >1 second), deceleration is bypassed and all movement stops immediately.

#### Feature Disabled
If `ENABLE_DECELERATION` is set to `false`, the system reverts to immediate stopping behavior.

## Implementation Details

### Deceleration State Tracking
Each control axis (left_x, left_y, right_x, right_y) has its own independent deceleration state:
- `leftXDecel` - Left joystick X (turning)
- `leftYDecel` - Left joystick Y (forward/backward)
- `rightXDecel` - Right joystick X (bucket)
- `rightYDecel` - Right joystick Y (arms)

### Deceleration Algorithm
The system uses linear interpolation to smoothly transition from the initial speed to zero:
```
current_value = start_speed + (target_speed - start_speed) * progress
where progress = elapsed_time / deceleration_duration
```

### Flow Control
During deceleration, the proportional flow valve receives the interpolated value instead of the raw joystick input. The hydraulic solenoid valve remains engaged throughout the deceleration period.

## Testing
To test the deceleration feature:
1. Move the wheels at full speed
2. Release the joystick to center (zero position)
3. Observe that movement gradually slows over 2 seconds before stopping
4. Repeat at half speed - should decelerate in 1 second
5. Test arms/bucket - should use half the deceleration time of wheels

## Troubleshooting
- If deceleration feels too long or too short, adjust the timing constants in `calculateDecelerationDuration()`
- If mixed mode detection is too sensitive, adjust the logic in `isInMixedMode()`
- Monitor serial output to see when deceleration is active
