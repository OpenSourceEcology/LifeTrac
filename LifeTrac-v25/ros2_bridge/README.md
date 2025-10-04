# LifeTrac v25 ROS2-MQTT Bridge

This directory contains ROS2 packages for integrating the LifeTrac v25 control system with ROS2, enabling control from BeagleBone or any ROS2-enabled device via MQTT.

## Overview

The ROS2-MQTT bridge allows you to:
- Send control commands from ROS2 topics to the LifeTrac v25 via MQTT
- Receive status updates from the LifeTrac system on ROS2 topics
- Integrate LifeTrac control into ROS2-based autonomous systems
- Use BeagleBone or other ROS2 devices as control interfaces

## Architecture

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────────┐
│  BeagleBone/    │ ROS2    │  mqtt_client     │  MQTT   │  MQTT Broker    │
│  ROS2 Device    ├────────>│  Bridge          ├────────>│  (Raspberry Pi) │
└─────────────────┘ Topics  └──────────────────┘         └────────┬────────┘
                                                                    │ MQTT
                                                           ┌────────┴────────┐
                                                           │ Arduino Opta    │
                                                           │  Controller     │
                                                           └─────────────────┘
```

## Packages

### 1. `lifetrac_msgs`
Custom ROS2 message definitions for LifeTrac control.

**Messages:**
- `ControlCommand.msg` - Control command message with joystick values

### 2. `lifetrac_mqtt_bridge`
ROS2 node that bridges control commands to MQTT using the [mqtt_client](https://github.com/ika-rwth-aachen/mqtt_client) package.

**Nodes:**
- `mqtt_bridge_node` - Main bridge node with logging and validation
- `test_publisher` - Test publisher for development and debugging

## Prerequisites

### On BeagleBone or Control Device

1. **ROS2 Installation** (Humble or newer recommended)
   ```bash
   # Follow ROS2 installation instructions for your platform
   # https://docs.ros.org/en/humble/Installation.html
   ```

2. **mqtt_client Package**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/ika-rwth-aachen/mqtt_client.git
   cd ~/ros2_ws
   colcon build --packages-select mqtt_client
   ```

3. **Network Configuration**
   - Ensure BeagleBone can reach the MQTT broker (Raspberry Pi)
   - Default broker IP: 192.168.1.100
   - Default broker port: 1883

### On MQTT Broker (Raspberry Pi)

Ensure Mosquitto MQTT broker is running with the LifeTrac configuration:
```bash
sudo systemctl status mosquitto
```

## Installation

1. **Copy ROS2 packages to your workspace:**
   ```bash
   cd ~/ros2_ws/src
   cp -r /path/to/LifeTrac/LifeTrac-v25/ros2_bridge/* .
   ```

2. **Build the packages:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select lifetrac_msgs lifetrac_mqtt_bridge
   source install/setup.bash
   ```

3. **Verify installation:**
   ```bash
   ros2 pkg list | grep lifetrac
   ```

## Configuration

Edit the configuration file to match your setup:
```bash
nano ~/ros2_ws/src/lifetrac_mqtt_bridge/config/mqtt_bridge_params.yaml
```

Key parameters to configure:
- `broker.host`: MQTT broker IP address (default: 192.168.1.100)
- `broker.port`: MQTT broker port (default: 1883)
- `broker.user`: MQTT username (default: lifetrac)
- `broker.pass`: MQTT password (default: lifetrac_pass)

## Usage

### Starting the Bridge

**Basic usage:**
```bash
ros2 launch lifetrac_mqtt_bridge lifetrac_bridge.launch.py
```

**With custom broker settings:**
```bash
ros2 launch lifetrac_mqtt_bridge lifetrac_bridge.launch.py \
    broker_host:=192.168.1.50 \
    broker_port:=1883 \
    mqtt_user:=myuser \
    mqtt_pass:=mypassword
```

### Testing the Bridge

**Run the test publisher (demo mode):**
```bash
ros2 run lifetrac_mqtt_bridge test_publisher
```

**Run the test publisher (interactive mode):**
```bash
ros2 run lifetrac_mqtt_bridge test_publisher --interactive
```

Interactive mode commands:
- `w/s` - Forward/backward movement
- `a/d` - Left/right turn
- `i/k` - Arms up/down
- `j/l` - Bucket down/up
- `0` - Emergency stop (all zeros)
- `quit` - Exit

### Publishing Control Commands

From your own ROS2 node:

```python
import rclpy
from rclpy.node import Node
from lifetrac_msgs.msg import ControlCommand
import time

class LifeTracController(Node):
    def __init__(self):
        super().__init__('lifetrac_controller')
        self.publisher = self.create_publisher(
            ControlCommand,
            '/lifetrac/control_cmd',
            10
        )
    
    def send_command(self, left_x=0, left_y=0, right_x=0, right_y=0):
        msg = ControlCommand()
        msg.left_x = left_x
        msg.left_y = left_y
        msg.right_x = right_x
        msg.right_y = right_y
        msg.timestamp = int(time.time() * 1000)
        self.publisher.publish(msg)

def main():
    rclpy.init()
    controller = LifeTracController()
    
    # Example: Move forward
    controller.send_command(left_x=0, left_y=50)
    
    rclpy.spin(controller)
```

### Monitoring Status

Subscribe to status topics:

```bash
# Monitor controller status
ros2 topic echo /lifetrac/status

# Monitor remote control status
ros2 topic echo /lifetrac/remote_status
```

## ROS2 Topics

### Published Topics (from your code)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/lifetrac/control_cmd` | `lifetrac_msgs/ControlCommand` | Control commands to send to LifeTrac |

### Subscribed Topics (received from MQTT)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/lifetrac/status` | `std_msgs/String` | System status from Arduino Opta controller |
| `/lifetrac/remote_status` | `std_msgs/String` | Status from ESP32 remote control |

## Control Values

All control values use floating point ranges compatible with DroidPad and similar joystick interfaces:

- **Left joystick X (left_x)**: -1.0 to 1.0 (negative = left turn, positive = right turn)
- **Left joystick Y (left_y)**: -1.0 to 1.0 (negative = backward, positive = forward)
- **Right joystick X (right_x)**: -1.0 to 1.0 (negative = bucket down, positive = bucket up)
- **Right joystick Y (right_y)**: -1.0 to 1.0 (negative = arms down, positive = arms up)

## Safety Features

- **Input validation**: Commands outside valid ranges are rejected
- **Timeout protection**: Arduino Opta stops all movement if no commands received for 1 second
- **Emergency stop**: Send all zeros to immediately stop all movement

## Troubleshooting

### Bridge not connecting to MQTT broker

1. Verify MQTT broker is running:
   ```bash
   mosquitto_sub -h 192.168.1.100 -t "lifetrac/v25/#" -u lifetrac -P lifetrac_pass
   ```

2. Check network connectivity:
   ```bash
   ping 192.168.1.100
   ```

3. Verify firewall settings allow MQTT traffic (port 1883)

### No control response from LifeTrac

1. Check Arduino Opta is powered and connected to MQTT:
   - Monitor Arduino serial output
   - Verify WiFi connection
   - Check MQTT subscription

2. Verify MQTT messages are being published:
   ```bash
   mosquitto_sub -h 192.168.1.100 -t "lifetrac/v25/control" -u lifetrac -P lifetrac_pass
   ```

3. Check ROS2 topics are publishing:
   ```bash
   ros2 topic echo /lifetrac/control_cmd
   ```

### ROS2 package build errors

1. Ensure all dependencies are installed:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. Clean and rebuild:
   ```bash
   cd ~/ros2_ws
   rm -rf build install log
   colcon build
   ```

## Integration with Autonomous Systems

The ROS2 bridge enables integration with:
- **Nav2**: Autonomous navigation with LifeTrac mobility
- **MoveIt**: Coordinated arm and bucket control
- **Vision systems**: Camera-based control using ROS2 image processing
- **Sensor fusion**: Combine multiple sensor inputs for autonomous operation

Example integration with Nav2:
```python
# Translate Nav2 velocity commands to LifeTrac control
def cmd_vel_callback(self, msg):
    # Convert linear.x and angular.z to tank steering
    left_y = int(msg.linear.x * 100)
    left_x = int(msg.angular.z * 100)
    
    self.send_lifetrac_command(left_x=left_x, left_y=left_y)
```

## Development

### Running Tests

```bash
cd ~/ros2_ws
colcon test --packages-select lifetrac_msgs lifetrac_mqtt_bridge
colcon test-result --all
```

### Code Style

Follow ROS2 Python style guidelines:
```bash
ament_flake8 src/lifetrac_mqtt_bridge
ament_pep257 src/lifetrac_mqtt_bridge
```

## References

- [mqtt_client ROS2 package](https://github.com/ika-rwth-aachen/mqtt_client)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Eclipse Mosquitto](https://mosquitto.org/)
- [LifeTrac v25 Documentation](../README.md)

## License

GPL-3.0 - See LICENSE file for details

## Support

For issues or questions:
- Open an issue on the LifeTrac GitHub repository
- Contact Open Source Ecology: info@opensourceecology.org
