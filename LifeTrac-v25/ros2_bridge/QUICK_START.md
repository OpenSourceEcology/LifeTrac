# LifeTrac v25 ROS2 Bridge - Quick Start

This quick start guide gets you up and running with ROS2 control of LifeTrac v25 in under 15 minutes.

## Prerequisites
- BeagleBone with ROS2 Humble installed
- LifeTrac v25 with Arduino Opta controller running
- MQTT broker (Raspberry Pi) running and accessible
- Network connectivity between all devices

## 5-Minute Setup

### 1. Build ROS2 Packages (3 minutes)
```bash
cd ~/ros2_ws/src
git clone https://github.com/ika-rwth-aachen/mqtt_client.git
git clone https://github.com/OpenSourceEcology/LifeTrac.git
cp -r LifeTrac/LifeTrac-v25/ros2_bridge/* .

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 2. Configure Bridge (1 minute)
```bash
nano ~/ros2_ws/src/lifetrac_mqtt_bridge/config/mqtt_bridge_params.yaml
```

Update broker IP if different from 192.168.1.100:
```yaml
broker:
  host: "YOUR_BROKER_IP"  # Change this line
```

### 3. Test Connection (1 minute)
```bash
# Terminal 1: Start bridge
source ~/ros2_ws/install/setup.bash
ros2 launch lifetrac_mqtt_bridge lifetrac_bridge.launch.py

# Terminal 2: Run test
source ~/ros2_ws/install/setup.bash
ros2 run lifetrac_mqtt_bridge test_publisher --interactive
```

Type commands:
- `w` = forward
- `s` = backward  
- `0` = stop
- `quit` = exit

## Common Commands

### Start Bridge
```bash
ros2 launch lifetrac_mqtt_bridge lifetrac_bridge.launch.py
```

### With Custom Broker
```bash
ros2 launch lifetrac_mqtt_bridge lifetrac_bridge.launch.py broker_host:=192.168.1.50
```

### Monitor Topics
```bash
ros2 topic list
ros2 topic echo /lifetrac/control_cmd
ros2 topic echo /lifetrac/status
```

### Send Commands Manually
```bash
ros2 topic pub /lifetrac/control_cmd lifetrac_msgs/msg/ControlCommand \
  "{left_x: 0, left_y: 50, right_x: 0, right_y: 0, timestamp: 0}"
```

## Simple Python Control

Create `test_control.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lifetrac_msgs.msg import ControlCommand
import time

def main():
    rclpy.init()
    node = Node('test')
    pub = node.create_publisher(ControlCommand, '/lifetrac/control_cmd', 10)
    
    # Move forward for 2 seconds
    msg = ControlCommand()
    msg.left_y = 50
    msg.timestamp = int(time.time() * 1000)
    
    for _ in range(20):  # 2 seconds at 10Hz
        pub.publish(msg)
        time.sleep(0.1)
    
    # Stop
    msg.left_y = 0
    pub.publish(msg)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it:
```bash
chmod +x test_control.py
./test_control.py
```

## Troubleshooting

### Bridge won't start
```bash
# Check ROS2 is sourced
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Rebuild if needed
cd ~/ros2_ws
colcon build --packages-select lifetrac_msgs lifetrac_mqtt_bridge
```

### Can't reach MQTT broker
```bash
# Test connectivity
ping 192.168.1.100

# Test MQTT directly
mosquitto_sub -h 192.168.1.100 -t "lifetrac/v25/#" -u lifetrac -P lifetrac_pass
```

### No response from LifeTrac
1. Verify Arduino Opta is powered and connected to MQTT
2. Check Arduino serial monitor for connection status
3. Ensure commands are sent continuously (at least once per second)

## Next Steps

- Read [README.md](README.md) for comprehensive documentation
- See [BEAGLEBONE_SETUP.md](BEAGLEBONE_SETUP.md) for full BeagleBone setup
- Check [../INSTALLATION_GUIDE.md](../INSTALLATION_GUIDE.md) for hardware setup

## Control Values Reference

All values range from -100 to 100:

| Control | Negative | Zero | Positive |
|---------|----------|------|----------|
| left_x  | Turn left | Straight | Turn right |
| left_y  | Backward | Stop | Forward |
| right_x | Bucket down | Stop | Bucket up |
| right_y | Arms down | Stop | Arms up |

## Safety Note

⚠️ **Always maintain visual contact with LifeTrac during operation**
⚠️ **Have emergency stop ready (publish all zeros or disconnect)**
⚠️ **Test in open area before full operation**
