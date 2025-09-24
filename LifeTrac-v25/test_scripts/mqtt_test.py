#!/usr/bin/env python3
"""
LifeTrac v25 MQTT Test Script

This script can be used to test MQTT communication and simulate
joystick control commands for debugging purposes.

Requirements:
    pip install paho-mqtt

Usage:
    python3 mqtt_test.py [broker_ip]
"""

import json
import time
import sys
import paho.mqtt.client as mqtt
from datetime import datetime

class LifeTracMQTTTest:
    def __init__(self, broker_host="192.168.1.100", broker_port=1883):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.client = mqtt.Client()
        
        # MQTT settings
        self.client.username_pw_set("lifetrac", "lifetrac_pass")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # Topics
        self.control_topic = "lifetrac/v25/control"
        self.status_topic = "lifetrac/v25/status"
        self.remote_status_topic = "lifetrac/v25/remote_status"
        
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"Connected to MQTT broker at {self.broker_host}:{self.broker_port}")
            # Subscribe to all LifeTrac topics
            client.subscribe("lifetrac/v25/#")
            print("Subscribed to lifetrac/v25/# topics")
        else:
            print(f"Failed to connect to MQTT broker. Return code: {rc}")
    
    def on_message(self, client, userdata, msg):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        topic = msg.topic
        try:
            payload = json.loads(msg.payload.decode())
            print(f"[{timestamp}] {topic}: {json.dumps(payload, indent=2)}")
        except json.JSONDecodeError:
            payload = msg.payload.decode()
            print(f"[{timestamp}] {topic}: {payload}")
    
    def connect(self):
        """Connect to MQTT broker"""
        try:
            self.client.connect(self.broker_host, self.broker_port, 60)
            return True
        except Exception as e:
            print(f"Error connecting to MQTT broker: {e}")
            return False
    
    def send_control_command(self, left_x=0, left_y=0, right_x=0, right_y=0):
        """Send a control command to the LifeTrac"""
        command = {
            "left_x": left_x,
            "left_y": left_y,
            "right_x": right_x,
            "right_y": right_y,
            "timestamp": int(time.time() * 1000)
        }
        
        message = json.dumps(command)
        result = self.client.publish(self.control_topic, message)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"Sent control command: {message}")
        else:
            print(f"Failed to send control command. Error: {result.rc}")
    
    def run_demo_sequence(self):
        """Run a demonstration sequence of movements"""
        print("\n=== Starting Demo Sequence ===")
        
        sequences = [
            # (description, left_x, left_y, right_x, right_y, duration)
            ("Stop (all zeros)", 0, 0, 0, 0, 2),
            ("Forward movement", 0, 50, 0, 0, 3),
            ("Stop", 0, 0, 0, 0, 1),
            ("Backward movement", 0, -50, 0, 0, 3),
            ("Stop", 0, 0, 0, 0, 1),
            ("Left turn", -30, 30, 0, 0, 3),
            ("Stop", 0, 0, 0, 0, 1),
            ("Right turn", 30, 30, 0, 0, 3),
            ("Stop", 0, 0, 0, 0, 1),
            ("Arms up", 0, 0, 0, 70, 3),
            ("Arms down", 0, 0, 0, -70, 3),
            ("Stop", 0, 0, 0, 0, 1),
            ("Bucket up", 0, 0, 70, 0, 3),
            ("Bucket down", 0, 0, -70, 0, 3),
            ("Stop", 0, 0, 0, 0, 1),
            ("Final stop", 0, 0, 0, 0, 2),
        ]
        
        for description, lx, ly, rx, ry, duration in sequences:
            print(f"\n{description}...")
            self.send_control_command(lx, ly, rx, ry)
            time.sleep(duration)
        
        print("\n=== Demo Sequence Complete ===")
    
    def interactive_mode(self):
        """Interactive mode for manual testing"""
        print("\n=== Interactive Mode ===")
        print("Commands:")
        print("  w/s - forward/backward")
        print("  a/d - left/right turn")
        print("  i/k - arms up/down")
        print("  j/l - bucket left(down)/right(up)")
        print("  0 - emergency stop (all zeros)")
        print("  demo - run demo sequence")
        print("  quit - exit")
        
        left_x, left_y = 0, 0
        right_x, right_y = 0, 0
        
        while True:
            try:
                cmd = input("\nCommand: ").strip().lower()
                
                if cmd == 'quit':
                    break
                elif cmd == 'w':
                    left_y = 50
                elif cmd == 's':
                    left_y = -50
                elif cmd == 'a':
                    left_x = -50
                elif cmd == 'd':
                    left_x = 50
                elif cmd == 'i':
                    right_y = 70
                elif cmd == 'k':
                    right_y = -70
                elif cmd == 'j':
                    right_x = -70
                elif cmd == 'l':
                    right_x = 70
                elif cmd == '0':
                    left_x = left_y = right_x = right_y = 0
                    print("Emergency stop - all controls zeroed")
                elif cmd == 'demo':
                    self.run_demo_sequence()
                    left_x = left_y = right_x = right_y = 0
                    continue
                else:
                    print("Unknown command")
                    continue
                
                self.send_control_command(left_x, left_y, right_x, right_y)
                
            except KeyboardInterrupt:
                print("\nExiting...")
                break
    
    def monitor_mode(self):
        """Monitor mode - just listen to messages"""
        print("\n=== Monitor Mode ===")
        print("Listening for MQTT messages... (Ctrl+C to exit)")
        
        try:
            self.client.loop_forever()
        except KeyboardInterrupt:
            print("\nExiting monitor mode...")

def main():
    broker_ip = "192.168.1.100"
    
    if len(sys.argv) > 1:
        broker_ip = sys.argv[1]
    
    print(f"LifeTrac v25 MQTT Test Script")
    print(f"Connecting to MQTT broker at {broker_ip}")
    
    tester = LifeTracMQTTTest(broker_ip)
    
    if not tester.connect():
        print("Failed to connect to MQTT broker. Exiting.")
        return
    
    # Start the MQTT client loop in a separate thread
    tester.client.loop_start()
    
    # Wait a moment for connection
    time.sleep(2)
    
    while True:
        print("\n=== LifeTrac v25 MQTT Test Menu ===")
        print("1. Monitor mode (listen only)")
        print("2. Interactive control mode")
        print("3. Run demo sequence")
        print("4. Emergency stop")
        print("5. Exit")
        
        try:
            choice = input("Select option (1-5): ").strip()
            
            if choice == '1':
                tester.monitor_mode()
            elif choice == '2':
                tester.interactive_mode()
            elif choice == '3':
                tester.run_demo_sequence()
            elif choice == '4':
                tester.send_control_command(0, 0, 0, 0)
                print("Emergency stop command sent!")
            elif choice == '5':
                break
            else:
                print("Invalid option")
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break
    
    tester.client.loop_stop()
    tester.client.disconnect()
    print("Disconnected from MQTT broker")

if __name__ == "__main__":
    main()