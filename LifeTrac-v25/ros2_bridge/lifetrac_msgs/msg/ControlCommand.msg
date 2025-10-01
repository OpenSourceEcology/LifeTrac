# LifeTrac v25 Control Command Message
# Used to send joystick control commands to the LifeTrac via MQTT

# Left joystick control (tank steering)
int32 left_x     # Left joystick X-axis (-100 to 100) - turning
int32 left_y     # Left joystick Y-axis (-100 to 100) - forward/backward

# Right joystick control (hydraulic functions)
int32 right_x    # Right joystick X-axis (-100 to 100) - bucket control
int32 right_y    # Right joystick Y-axis (-100 to 100) - arms control

# Timestamp
uint64 timestamp # Timestamp in milliseconds
