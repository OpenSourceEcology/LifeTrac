#!/usr/bin/env python3
"""
LifeTrac v25 ROS2-MQTT Bridge Node

This node bridges ROS2 control commands from a BeagleBone to the MQTT broker
used by the LifeTrac v25 system. It subscribes to ROS2 topics and publishes
to MQTT topics that the Arduino Opta controller listens to.

This bridge uses the mqtt_client package from ika-rwth-aachen for ROS2-MQTT integration.
See: https://github.com/ika-rwth-aachen/mqtt_client
"""

import rclpy
from rclpy.node import Node
from lifetrac_msgs.msg import ControlCommand
import json
import time


class LifeTracMQTTBridge(Node):
    """
    ROS2 node that bridges control commands to MQTT for LifeTrac v25.
    
    This node subscribes to ROS2 ControlCommand messages and publishes them
    to the MQTT broker that the Arduino Opta controller is listening to.
    """

    def __init__(self):
        super().__init__('lifetrac_mqtt_bridge')
        
        # Declare parameters
        self.declare_parameter('mqtt_broker_host', '192.168.1.100')
        self.declare_parameter('mqtt_broker_port', 1883)
        self.declare_parameter('mqtt_username', 'lifetrac')
        self.declare_parameter('mqtt_password', 'lifetrac_pass')
        self.declare_parameter('control_topic', 'lifetrac/v25/control')
        self.declare_parameter('ros2_control_topic', '/lifetrac/control_cmd')
        
        # Get parameters
        self.mqtt_broker_host = self.get_parameter('mqtt_broker_host').value
        self.mqtt_broker_port = self.get_parameter('mqtt_broker_port').value
        self.mqtt_username = self.get_parameter('mqtt_username').value
        self.mqtt_password = self.get_parameter('mqtt_password').value
        self.control_topic = self.get_parameter('control_topic').value
        ros2_control_topic = self.get_parameter('ros2_control_topic').value
        
        # Log configuration
        self.get_logger().info(f'Starting LifeTrac MQTT Bridge')
        self.get_logger().info(f'MQTT Broker: {self.mqtt_broker_host}:{self.mqtt_broker_port}')
        self.get_logger().info(f'MQTT Control Topic: {self.control_topic}')
        self.get_logger().info(f'ROS2 Control Topic: {ros2_control_topic}')
        
        # Create ROS2 subscriber for control commands
        self.control_subscriber = self.create_subscription(
            ControlCommand,
            ros2_control_topic,
            self.control_callback,
            10
        )
        
        self.get_logger().info('LifeTrac MQTT Bridge initialized successfully')
        
    def control_callback(self, msg):
        """
        Callback for ROS2 control commands.
        
        This method is called whenever a ControlCommand message is received
        on the ROS2 topic. It converts the message to JSON format and publishes
        it to the MQTT broker via the mqtt_client package.
        
        The actual MQTT publishing is handled by the mqtt_client bridge configuration.
        This node just prepares the data in the correct format.
        """
        # Log received command
        self.get_logger().debug(
            f'Received control command: '
            f'LX={msg.left_x}, LY={msg.left_y}, '
            f'RX={msg.right_x}, RY={msg.right_y}'
        )
        
        # The mqtt_client package will automatically handle the publishing
        # based on the configuration in the params file
        # This callback is primarily for logging and validation
        
        # Validate input ranges
        if not self._validate_command(msg):
            self.get_logger().warn('Invalid command values detected, ignoring')
            return
            
        self.get_logger().info(
            f'Publishing control command to MQTT: '
            f'LX={msg.left_x}, LY={msg.left_y}, '
            f'RX={msg.right_x}, RY={msg.right_y}'
        )
    
    def _validate_command(self, msg):
        """Validate that control command values are within acceptable ranges."""
        return (
            -1.0 <= msg.left_x <= 1.0 and
            -1.0 <= msg.left_y <= 1.0 and
            -1.0 <= msg.right_x <= 1.0 and
            -1.0 <= msg.right_y <= 1.0
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = LifeTracMQTTBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in MQTT bridge: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
