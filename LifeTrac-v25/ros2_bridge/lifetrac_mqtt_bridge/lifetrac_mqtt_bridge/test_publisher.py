#!/usr/bin/env python3
"""
LifeTrac v25 Test Publisher

This script publishes test control commands to the ROS2 topic that will be
bridged to MQTT. Use this to test the ROS2-MQTT bridge from a BeagleBone or
any ROS2-enabled device.

Usage:
    ros2 run lifetrac_mqtt_bridge test_publisher
    
Or with interactive mode:
    ros2 run lifetrac_mqtt_bridge test_publisher --interactive
"""

import rclpy
from rclpy.node import Node
from lifetrac_msgs.msg import ControlCommand
import time
import sys


class LifeTracTestPublisher(Node):
    """Test publisher for LifeTrac control commands."""

    def __init__(self, interactive=False):
        super().__init__('lifetrac_test_publisher')
        
        self.publisher = self.create_publisher(
            ControlCommand,
            '/lifetrac/control_cmd',
            10
        )
        
        self.interactive = interactive
        
        if interactive:
            self.get_logger().info('Starting in interactive mode')
        else:
            self.get_logger().info('Starting in demo mode')
            # Create timer for demo sequence
            self.timer = self.create_timer(3.0, self.demo_callback)
            self.demo_step = 0
    
    def publish_command(self, left_x=0, left_y=0, right_x=0, right_y=0):
        """Publish a control command."""
        msg = ControlCommand()
        msg.left_x = left_x
        msg.left_y = left_y
        msg.right_x = right_x
        msg.right_y = right_y
        msg.timestamp = int(time.time() * 1000)
        
        self.publisher.publish(msg)
        
        self.get_logger().info(
            f'Published: LX={left_x}, LY={left_y}, RX={right_x}, RY={right_y}'
        )
        
        return msg
    
    def demo_callback(self):
        """Demo sequence timer callback."""
        demo_sequence = [
            ("Stop", 0.0, 0.0, 0.0, 0.0),
            ("Forward", 0.0, 0.5, 0.0, 0.0),
            ("Stop", 0.0, 0.0, 0.0, 0.0),
            ("Backward", 0.0, -0.5, 0.0, 0.0),
            ("Stop", 0.0, 0.0, 0.0, 0.0),
            ("Left turn", -0.3, 0.3, 0.0, 0.0),
            ("Stop", 0.0, 0.0, 0.0, 0.0),
            ("Right turn", 0.3, 0.3, 0.0, 0.0),
            ("Stop", 0.0, 0.0, 0.0, 0.0),
            ("Arms up", 0.0, 0.0, 0.0, 0.7),
            ("Arms down", 0.0, 0.0, 0.0, -0.7),
            ("Stop", 0.0, 0.0, 0.0, 0.0),
            ("Bucket up", 0.0, 0.0, 0.7, 0.0),
            ("Bucket down", 0.0, 0.0, -0.7, 0.0),
            ("Stop", 0.0, 0.0, 0.0, 0.0),
        ]
        
        if self.demo_step < len(demo_sequence):
            desc, lx, ly, rx, ry = demo_sequence[self.demo_step]
            self.get_logger().info(f'Demo step: {desc}')
            self.publish_command(lx, ly, rx, ry)
            self.demo_step += 1
        else:
            # Reset to beginning
            self.demo_step = 0
    
    def interactive_mode(self):
        """Run in interactive mode with keyboard input."""
        print("\n=== LifeTrac v25 ROS2 Test Publisher ===")
        print("Commands:")
        print("  w/s - forward/backward (left_y)")
        print("  a/d - left/right turn (left_x)")
        print("  i/k - arms up/down (right_y)")
        print("  j/l - bucket down/up (right_x)")
        print("  0 - emergency stop (all zeros)")
        print("  quit - exit")
        print()
        
        left_x, left_y = 0.0, 0.0
        right_x, right_y = 0.0, 0.0
        
        while rclpy.ok():
            try:
                cmd = input("Command: ").strip().lower()
                
                if cmd == 'quit':
                    break
                elif cmd == 'w':
                    left_y = 0.5
                elif cmd == 's':
                    left_y = -0.5
                elif cmd == 'a':
                    left_x = -0.5
                elif cmd == 'd':
                    left_x = 0.5
                elif cmd == 'i':
                    right_y = 0.7
                elif cmd == 'k':
                    right_y = -0.7
                elif cmd == 'j':
                    right_x = -0.7
                elif cmd == 'l':
                    right_x = 0.7
                elif cmd == '0':
                    left_x = left_y = right_x = right_y = 0.0
                    print("Emergency stop - all controls zeroed")
                else:
                    print("Unknown command")
                    continue
                
                self.publish_command(left_x, left_y, right_x, right_y)
                
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except EOFError:
                break


def main(args=None):
    rclpy.init(args=args)
    
    # Check for interactive mode flag
    interactive = '--interactive' in sys.argv or '-i' in sys.argv
    
    try:
        publisher = LifeTracTestPublisher(interactive=interactive)
        
        if interactive:
            publisher.interactive_mode()
        else:
            rclpy.spin(publisher)
            
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
