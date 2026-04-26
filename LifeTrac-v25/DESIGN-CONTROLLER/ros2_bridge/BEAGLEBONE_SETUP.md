# BeagleBone Setup Guide for LifeTrac v25 ROS2 Control

This guide provides step-by-step instructions for setting up a BeagleBone to control the LifeTrac v25 using ROS2 and MQTT.

## Hardware Requirements

- BeagleBone Black, BeagleBone AI, or BeagleBone AI-64
- MicroSD card (16GB or larger, recommended for ROS2)
- Power supply (5V, 2A minimum)
- Network connection (WiFi adapter or Ethernet)

## Software Requirements

- Ubuntu 22.04 or later (for ROS2 Humble)
- ROS2 Humble Hawksbill or newer
- Python 3.10+

## Step 1: Install Ubuntu on BeagleBone

### Option A: BeagleBone AI-64 (Recommended)
1. Download Ubuntu 22.04 image for BeagleBone AI-64
2. Flash to microSD card using Balena Etcher
3. Boot BeagleBone from microSD card
4. Complete initial Ubuntu setup

### Option B: BeagleBone Black/AI
1. Download Ubuntu 20.04 or 22.04 image
2. Flash to microSD card
3. Boot and configure

```bash
# After first boot, update system
sudo apt update && sudo apt upgrade -y
```

## Step 2: Install ROS2 Humble

```bash
# Set locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 packages
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-ros-base -y

# Install development tools
sudo apt install python3-pip python3-rosdep python3-colcon-common-extensions -y

# Initialize rosdep
sudo rosdep init
rosdep update
```

## Step 3: Setup ROS2 Workspace

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone mqtt_client package
git clone https://github.com/ika-rwth-aachen/mqtt_client.git

# Copy LifeTrac ROS2 packages
# Option 1: Clone from repository
git clone https://github.com/OpenSourceEcology/LifeTrac.git
cp -r LifeTrac/LifeTrac-v25/ros2_bridge/* ~/ros2_ws/src/

# Option 2: Copy directly if files are available locally
# cp -r /path/to/LifeTrac/LifeTrac-v25/ros2_bridge/* ~/ros2_ws/src/

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build packages
colcon build --symlink-install

# Source workspace
source ~/ros2_ws/install/setup.bash
```

## Step 4: Configure Network

### Connect to WiFi
```bash
# Using nmcli
sudo nmcli device wifi connect "YOUR_SSID" password "YOUR_PASSWORD"

# Verify connection
ip addr show
ping 192.168.1.100  # Test connection to MQTT broker
```

### Configure Static IP (Optional)
For reliable operation, configure a static IP:

```bash
sudo nano /etc/netplan/01-netcfg.yaml
```

Example configuration:
```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: no
      addresses: [192.168.1.50/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      access-points:
        "YOUR_SSID":
          password: "YOUR_PASSWORD"
```

Apply configuration:
```bash
sudo netplan apply
```

## Step 5: Configure LifeTrac Bridge

Edit the configuration file:
```bash
cd ~/ros2_ws/src/lifetrac_mqtt_bridge/config
nano mqtt_bridge_params.yaml
```

Update these parameters:
- `broker.host`: IP address of your MQTT broker (Raspberry Pi)
- `broker.user`: MQTT username (default: lifetrac)
- `broker.pass`: MQTT password (default: lifetrac_pass)

## Step 6: Test the Setup

### Terminal 1: Start the bridge
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch lifetrac_mqtt_bridge lifetrac_bridge.launch.py
```

### Terminal 2: Run test publisher
```bash
source ~/ros2_ws/install/setup.bash
ros2 run lifetrac_mqtt_bridge test_publisher --interactive
```

Test commands:
- Press `w` for forward
- Press `s` for backward
- Press `0` for emergency stop
- Type `quit` to exit

### Terminal 3: Monitor topics
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /lifetrac/control_cmd
```

## Step 7: Auto-start on Boot (Optional)

Create a systemd service to start the bridge automatically:

```bash
sudo nano /etc/systemd/system/lifetrac-bridge.service
```

Add this content:
```ini
[Unit]
Description=LifeTrac v25 ROS2 MQTT Bridge
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=debian
WorkingDirectory=/home/debian/ros2_ws
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/debian/ros2_ws/install/setup.bash && ros2 launch lifetrac_mqtt_bridge lifetrac_bridge.launch.py'
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable lifetrac-bridge.service
sudo systemctl start lifetrac-bridge.service
sudo systemctl status lifetrac-bridge.service
```

## Step 8: Create Custom Control Application

Example Python script to control LifeTrac:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lifetrac_msgs.msg import ControlCommand
import time

class LifeTracAutonomous(Node):
    def __init__(self):
        super().__init__('lifetrac_autonomous')
        self.publisher = self.create_publisher(
            ControlCommand,
            '/lifetrac/control_cmd',
            10
        )
        self.get_logger().info('LifeTrac Autonomous Controller Started')
    
    def move_forward(self, speed=50, duration=2.0):
        """Move forward at specified speed for duration seconds."""
        msg = ControlCommand()
        msg.left_x = 0
        msg.left_y = speed
        msg.right_x = 0
        msg.right_y = 0
        msg.timestamp = int(time.time() * 1000)
        
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher.publish(msg)
            time.sleep(0.1)  # Publish at 10Hz
        
        self.stop()
    
    def stop(self):
        """Send stop command."""
        msg = ControlCommand()
        msg.left_x = 0
        msg.left_y = 0
        msg.right_x = 0
        msg.right_y = 0
        msg.timestamp = int(time.time() * 1000)
        self.publisher.publish(msg)

def main():
    rclpy.init()
    controller = LifeTracAutonomous()
    
    try:
        # Example: Move forward for 3 seconds
        controller.get_logger().info('Moving forward...')
        controller.move_forward(speed=50, duration=3.0)
        controller.get_logger().info('Stopping...')
        
        # Keep node alive
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.stop()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Save as `lifetrac_autonomous.py`, make executable:
```bash
chmod +x lifetrac_autonomous.py
./lifetrac_autonomous.py
```

## Troubleshooting

### Bridge won't start
1. Check ROS2 installation: `ros2 --version`
2. Verify workspace build: `cd ~/ros2_ws && colcon build`
3. Source workspace: `source ~/ros2_ws/install/setup.bash`

### Cannot connect to MQTT broker
1. Verify network: `ping 192.168.1.100`
2. Test MQTT manually:
   ```bash
   sudo apt install mosquitto-clients
   mosquitto_sub -h 192.168.1.100 -t "lifetrac/v25/#" -u lifetrac -P lifetrac_pass
   ```

### High CPU usage
BeagleBone has limited resources. Optimize:
1. Reduce publishing rate in your control code
2. Use ROS2 parameter `use_sim_time:=false`
3. Consider BeagleBone AI-64 for better performance

### No response from LifeTrac
1. Verify Arduino Opta is connected to MQTT
2. Monitor MQTT traffic: `mosquitto_sub -v -h 192.168.1.100 -t "#" -u lifetrac -P lifetrac_pass`
3. Check LifeTrac safety timeout (commands must be sent within 1 second intervals)

## Performance Considerations

### BeagleBone Black/AI
- CPU: 1 GHz ARM Cortex-A8 or dual-core Cortex-A15
- RAM: 512MB - 1GB
- Recommended: Lightweight control applications only
- Publishing rate: Max 10Hz for control commands

### BeagleBone AI-64
- CPU: Dual-core Cortex-A72 @ 2.0 GHz
- RAM: 4GB
- Recommended: Full ROS2 applications including vision
- Publishing rate: Up to 50Hz for control commands

## Next Steps

1. Integrate with sensor packages (cameras, LiDAR, GPS)
2. Implement autonomous navigation using Nav2
3. Add obstacle avoidance using depth cameras
4. Create task-specific control applications
5. Implement logging and telemetry

## Additional Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [BeagleBone Getting Started](https://beagleboard.org/getting-started)
- [mqtt_client Package](https://github.com/ika-rwth-aachen/mqtt_client)
- [LifeTrac v25 Main Documentation](../README.md)
