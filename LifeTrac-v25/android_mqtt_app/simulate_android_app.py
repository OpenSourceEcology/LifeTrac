#!/usr/bin/env python3
"""
Android App MQTT Simulator for LifeTrac v25

This script simulates the Android app by sending MQTT control messages
to test the LifeTrac v25 system without needing the actual Android app.

Usage:
    python3 simulate_android_app.py [broker_ip] [username] [password]

Controls:
    W/S - Forward/Backward (left joystick Y)
    A/D - Turn Left/Right (left joystick X) 
    I/K - Arms Up/Down (right joystick Y)
    J/L - Bucket Down/Up (right joystick X)
    SPACE - Emergency Stop (all zeros)
    Q - Quit

Requirements:
    pip install paho-mqtt keyboard
"""

import json
import sys
import time
import threading
from datetime import datetime
import paho.mqtt.client as mqtt

try:
    import keyboard
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False
    print("⚠️  keyboard module not available. Install with: pip install keyboard")
    print("   Falling back to sequential test mode")

# Default configuration
DEFAULT_BROKER = "192.168.1.100"
DEFAULT_PORT = 1883
DEFAULT_USERNAME = "lifetrac"
DEFAULT_PASSWORD = "lifetrac_pass"
DEFAULT_TOPIC = "lifetrac/v25/control"

class AndroidAppSimulator:
    def __init__(self, broker_ip, port, username, password, topic):
        self.broker_ip = broker_ip
        self.port = port
        self.username = username
        self.password = password
        self.topic = topic
        
        # Control state
        self.left_x = 0   # Tank steering turning
        self.left_y = 0   # Tank steering forward/back
        self.right_x = 0  # Bucket control
        self.right_y = 0  # Arms control
        
        # MQTT client
        self.client = mqtt.Client()
        self.connected = False
        self.running = True
        
        # Setup callbacks
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_publish = self.on_publish
        
        # Setup authentication
        if username and password:
            self.client.username_pw_set(username, password)
    
    def on_connect(self, client, userdata, flags, rc):
        """Callback for MQTT connection."""
        if rc == 0:
            self.connected = True
            print(f"✅ Connected to MQTT broker at {self.broker_ip}:{self.port}")
            print("🚜 LifeTrac v25 Android App Simulator Ready!")
            print("=" * 50)
            if KEYBOARD_AVAILABLE:
                self.print_controls()
            else:
                print("🔄 Running automated test sequence...")
        else:
            print(f"❌ Failed to connect to MQTT broker (code {rc})")
            self.connected = False
    
    def on_disconnect(self, client, userdata, rc):
        """Callback for MQTT disconnection."""
        self.connected = False
        print(f"🔌 Disconnected from MQTT broker")
    
    def on_publish(self, client, userdata, mid):
        """Callback for successful message publish."""
        pass  # Silent for normal operation
    
    def print_controls(self):
        """Print keyboard control instructions."""
        print("⌨️  Keyboard Controls:")
        print("   W/S  - Forward/Backward movement")
        print("   A/D  - Turn Left/Right")
        print("   I/K  - Arms Up/Down")
        print("   J/L  - Bucket Down/Up")
        print("   SPACE - Emergency Stop")
        print("   Q    - Quit")
        print("=" * 50)
    
    def send_control_message(self):
        """Send current control state as MQTT message."""
        if not self.connected:
            return False
        
        # Create message matching Android app format
        message = {
            "left_x": self.left_x,
            "left_y": self.left_y,
            "right_x": self.right_x,
            "right_y": self.right_y,
            "timestamp": int(time.time() * 1000)  # Milliseconds
        }
        
        # Convert to JSON
        json_message = json.dumps(message)
        
        # Publish message
        result = self.client.publish(self.topic, json_message)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            # Print status if any controls are active
            if any([self.left_x, self.left_y, self.right_x, self.right_y]):
                timestamp = datetime.now().strftime("%H:%M:%S")
                print(f"[{timestamp}] LX:{self.left_x:+3d} LY:{self.left_y:+3d} RX:{self.right_x:+3d} RY:{self.right_y:+3d}")
            return True
        else:
            print(f"❌ Failed to send message (rc: {result.rc})")
            return False
    
    def emergency_stop(self):
        """Send emergency stop command."""
        self.left_x = 0
        self.left_y = 0
        self.right_x = 0
        self.right_y = 0
        
        print("🛑 EMERGENCY STOP!")
        self.send_control_message()
    
    def connect_mqtt(self):
        """Connect to MQTT broker."""
        try:
            print(f"🔌 Connecting to MQTT broker at {self.broker_ip}:{self.port}")
            self.client.connect(self.broker_ip, self.port, 60)
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def keyboard_control_loop(self):
        """Main loop for keyboard control."""
        print("🎮 Starting keyboard control mode...")
        print("   Press keys to control LifeTrac (ESC to stop)")
        
        # Message sending timer
        last_send_time = 0
        send_interval = 0.05  # 20Hz like Android app
        
        while self.running and self.connected:
            try:
                # Handle keyboard input
                if keyboard.is_pressed('w'):
                    self.left_y = min(100, self.left_y + 5)
                elif keyboard.is_pressed('s'):
                    self.left_y = max(-100, self.left_y - 5)
                else:
                    self.left_y = int(self.left_y * 0.9)  # Gradual decay
                
                if keyboard.is_pressed('a'):
                    self.left_x = max(-100, self.left_x - 5)
                elif keyboard.is_pressed('d'):
                    self.left_x = min(100, self.left_x + 5)
                else:
                    self.left_x = int(self.left_x * 0.9)  # Gradual decay
                
                if keyboard.is_pressed('i'):
                    self.right_y = min(100, self.right_y + 5)
                elif keyboard.is_pressed('k'):
                    self.right_y = max(-100, self.right_y - 5)
                else:
                    self.right_y = int(self.right_y * 0.9)  # Gradual decay
                
                if keyboard.is_pressed('j'):
                    self.right_x = max(-100, self.right_x - 5)
                elif keyboard.is_pressed('l'):
                    self.right_x = min(100, self.right_x + 5)
                else:
                    self.right_x = int(self.right_x * 0.9)  # Gradual decay
                
                if keyboard.is_pressed('space'):
                    self.emergency_stop()
                    time.sleep(0.5)  # Prevent rapid triggering
                
                if keyboard.is_pressed('q') or keyboard.is_pressed('esc'):
                    print("👋 Quitting...")
                    self.running = False
                    break
                
                # Send control message at regular intervals
                current_time = time.time()
                if current_time - last_send_time >= send_interval:
                    self.send_control_message()
                    last_send_time = current_time
                
                # Process MQTT messages
                self.client.loop(timeout=0.01)
                time.sleep(0.01)
                
            except KeyboardInterrupt:
                print("\n⏹️  Interrupted by user")
                self.running = False
                break
    
    def automated_test_sequence(self):
        """Run automated test sequence when keyboard is not available."""
        test_sequences = [
            ("Forward movement", [(0, 50, 0, 0)], 2.0),
            ("Backward movement", [(0, -50, 0, 0)], 2.0),
            ("Turn right", [(50, 0, 0, 0)], 2.0),
            ("Turn left", [(-50, 0, 0, 0)], 2.0),
            ("Arms up", [(0, 0, 0, 50)], 2.0),
            ("Arms down", [(0, 0, 0, -50)], 2.0),
            ("Bucket up", [(0, 0, 50, 0)], 2.0),
            ("Bucket down", [(0, 0, -50, 0)], 2.0),
            ("Combined movement", [(30, 40, 20, 30)], 3.0),
            ("Emergency stop", [(0, 0, 0, 0)], 1.0),
        ]
        
        for test_name, controls, duration in test_sequences:
            if not self.running or not self.connected:
                break
                
            print(f"🧪 Testing: {test_name}")
            
            # Set control values
            if controls:
                self.left_x, self.left_y, self.right_x, self.right_y = controls[0]
            
            # Send messages for specified duration
            start_time = time.time()
            last_send = 0
            
            while time.time() - start_time < duration and self.running:
                current_time = time.time()
                if current_time - last_send >= 0.05:  # 20Hz
                    self.send_control_message()
                    last_send = current_time
                
                self.client.loop(timeout=0.01)
                time.sleep(0.01)
            
            # Stop all movement between tests
            self.left_x = self.left_y = self.right_x = self.right_y = 0
            self.send_control_message()
            time.sleep(0.5)
        
        print("✅ Automated test sequence completed")
    
    def start(self):
        """Start the simulator."""
        if not self.connect_mqtt():
            return False
        
        # Start MQTT loop in background
        self.client.loop_start()
        
        # Wait for connection
        timeout = 10
        start_time = time.time()
        while not self.connected and time.time() - start_time < timeout:
            time.sleep(0.1)
        
        if not self.connected:
            print("❌ Failed to establish MQTT connection")
            return False
        
        try:
            if KEYBOARD_AVAILABLE:
                self.keyboard_control_loop()
            else:
                self.automated_test_sequence()
        except KeyboardInterrupt:
            print("\n⏹️  Interrupted by user")
        finally:
            # Clean shutdown
            self.emergency_stop()
            time.sleep(0.5)
            self.client.loop_stop()
            self.client.disconnect()
        
        return True

def main():
    """Main function."""
    print("🤖 LifeTrac v25 Android App MQTT Simulator")
    print("=" * 50)
    
    # Parse command line arguments
    broker_ip = sys.argv[1] if len(sys.argv) >= 2 else DEFAULT_BROKER
    username = sys.argv[2] if len(sys.argv) >= 3 else DEFAULT_USERNAME
    password = sys.argv[3] if len(sys.argv) >= 4 else DEFAULT_PASSWORD
    
    print(f"📡 Configuration:")
    print(f"   Broker: {broker_ip}:{DEFAULT_PORT}")
    print(f"   Topic: {DEFAULT_TOPIC}")
    print(f"   Auth: {username}/**********")
    print()
    
    # Create and start simulator
    simulator = AndroidAppSimulator(
        broker_ip=broker_ip,
        port=DEFAULT_PORT,
        username=username,
        password=password,
        topic=DEFAULT_TOPIC
    )
    
    success = simulator.start()
    
    if success:
        print("👋 Simulator finished successfully")
    else:
        print("❌ Simulator encountered errors")
        sys.exit(1)

if __name__ == "__main__":
    main()