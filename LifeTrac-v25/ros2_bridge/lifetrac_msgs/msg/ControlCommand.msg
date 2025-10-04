# LifeTrac v25 Control Command Message
# Used to send joystick control commands to the LifeTrac via MQTT

# Left joystick control (tank steering)
float32 left_x   # Left joystick X-axis (-1.0 to 1.0) - turning
float32 left_y   # Left joystick Y-axis (-1.0 to 1.0) - forward/backward

# Right joystick control (hydraulic functions)
float32 right_x  # Right joystick X-axis (-1.0 to 1.0) - bucket control
float32 right_y  # Right joystick Y-axis (-1.0 to 1.0) - arms control

# Timestamp
uint64 timestamp # Timestamp in milliseconds
