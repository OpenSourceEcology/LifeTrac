#!/usr/bin/env python3
"""
MQTT Test Listener for LifeTrac v25 Android App

This script listens for MQTT messages from the Android app and displays
the control data for testing and debugging purposes.

Usage:
    python3 mqtt_test_listener.py [broker_ip] [username] [password]

Requirements:
    pip install paho-mqtt
"""

import json
import sys
import time
from datetime import datetime
import paho.mqtt.client as mqtt

# Default configuration
DEFAULT_BROKER = "192.168.1.100"
DEFAULT_PORT = 1883
DEFAULT_USERNAME = "lifetrac"
DEFAULT_PASSWORD = "lifetrac_pass"
DEFAULT_TOPIC = "lifetrac/v25/control"

class LifeTracMQTTListener:
    def __init__(self, broker_ip, port, username, password, topic):
        self.broker_ip = broker_ip
        self.port = port
        self.username = username
        self.password = password
        self.topic = topic
        self.client = mqtt.Client()
        self.last_message_time = 0
        self.message_count = 0
        
        # Setup callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        
        # Setup authentication
        if username and password:
            self.client.username_pw_set(username, password)
    
    def on_connect(self, client, userdata, flags, rc):
        """Callback for when the client receives a CONNACK response from the server."""
        if rc == 0:
            print(f"✅ Connected to MQTT broker at {self.broker_ip}:{self.port}")
            print(f"🔊 Subscribing to topic: {self.topic}")
            client.subscribe(self.topic)
            print("📱 Waiting for Android app messages...")
            print("-" * 60)
        else:
            print(f"❌ Failed to connect to MQTT broker (code {rc})")
            self.print_connection_error(rc)
    
    def on_message(self, client, userdata, msg):
        """Callback for when a PUBLISH message is received from the server."""
        try:
            self.message_count += 1
            current_time = time.time()
            
            # Parse JSON message
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            
            # Calculate message frequency
            if self.last_message_time > 0:
                time_diff = current_time - self.last_message_time
                frequency = 1.0 / time_diff if time_diff > 0 else 0
            else:
                frequency = 0
            
            self.last_message_time = current_time
            
            # Display message data
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] Message #{self.message_count} | Freq: {frequency:.1f}Hz")
            
            # Extract joystick values
            left_x = data.get('left_x', 0)
            left_y = data.get('left_y', 0)
            right_x = data.get('right_x', 0)
            right_y = data.get('right_y', 0)
            msg_timestamp = data.get('timestamp', 0)
            
            # Display joystick values with visual representation
            print(f"  Left Joystick:  X:{left_x:+4d} Y:{right_y:+4d} | {self.format_joystick_visual(left_x, left_y)}")
            print(f"  Right Joystick: X:{right_x:+4d} Y:{right_y:+4d} | {self.format_joystick_visual(right_x, right_y)}")
            
            # Show movement interpretation
            movement = self.interpret_movement(left_x, left_y, right_x, right_y)
            if movement:
                print(f"  🚜 Movement: {movement}")
            
            print(f"  📨 App Timestamp: {msg_timestamp}")
            print("-" * 60)
            
        except json.JSONDecodeError as e:
            print(f"❌ Invalid JSON received: {e}")
            print(f"   Raw payload: {msg.payload}")
        except Exception as e:
            print(f"❌ Error processing message: {e}")
    
    def on_disconnect(self, client, userdata, rc):
        """Callback for when the client disconnects from the server."""
        print(f"🔌 Disconnected from MQTT broker (code {rc})")
    
    def format_joystick_visual(self, x, y):
        """Create a simple visual representation of joystick position."""
        # Normalize to -1 to 1 range for display
        norm_x = max(-1, min(1, x / 100.0))
        norm_y = max(-1, min(1, y / 100.0))
        
        # Create a 5x5 grid representation
        grid = [['·' for _ in range(5)] for _ in range(5)]
        
        # Calculate grid position (center is 2,2)
        grid_x = int((norm_x + 1) * 2)  # 0-4 range
        grid_y = int((1 - norm_y) * 2)  # 0-4 range (inverted Y)
        
        grid_x = max(0, min(4, grid_x))
        grid_y = max(0, min(4, grid_y))
        
        grid[grid_y][grid_x] = '●'
        
        return ' '.join([''.join(row) for row in grid])
    
    def interpret_movement(self, left_x, left_y, right_x, right_y):
        """Interpret joystick values into human-readable movement description."""
        movements = []
        
        # Tank steering interpretation (left joystick)
        if abs(left_y) > 10:  # Forward/backward threshold
            if left_y > 0:
                movements.append("Forward")
            else:
                movements.append("Backward")
        
        if abs(left_x) > 10:  # Turning threshold
            if left_x > 0:
                movements.append("Turn Right")
            else:
                movements.append("Turn Left")
        
        # Hydraulic functions (right joystick)
        if abs(right_y) > 10:  # Arms threshold
            if right_y > 0:
                movements.append("Arms Up")
            else:
                movements.append("Arms Down")
        
        if abs(right_x) > 10:  # Bucket threshold
            if right_x > 0:
                movements.append("Bucket Up")
            else:
                movements.append("Bucket Down")
        
        return ", ".join(movements) if movements else "Idle"
    
    def print_connection_error(self, rc):
        """Print detailed connection error information."""
        error_messages = {
            1: "Incorrect protocol version",
            2: "Invalid client identifier", 
            3: "Server unavailable",
            4: "Bad username or password",
            5: "Not authorized"
        }
        
        error_msg = error_messages.get(rc, f"Unknown error code {rc}")
        print(f"   Error: {error_msg}")
        
        if rc == 3:
            print("   💡 Check if MQTT broker (Mosquitto) is running")
            print("   💡 Verify the broker IP address is correct")
        elif rc == 4:
            print("   💡 Check MQTT username and password")
            print("   💡 Verify broker authentication configuration")
        elif rc == 5:
            print("   💡 Check MQTT user permissions")
    
    def start_listening(self):
        """Start the MQTT listener."""
        try:
            print(f"🔌 Connecting to MQTT broker at {self.broker_ip}:{self.port}")
            print(f"👤 Username: {self.username}")
            self.client.connect(self.broker_ip, self.port, 60)
            self.client.loop_forever()
        except KeyboardInterrupt:
            print("\n⏹️  Stopping listener...")
            self.client.disconnect()
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            print("💡 Make sure the MQTT broker is running and accessible")

def main():
    """Main function to handle command line arguments and start listener."""
    print("🚜 LifeTrac v25 Android App MQTT Test Listener")
    print("=" * 60)
    
    # Parse command line arguments
    if len(sys.argv) >= 2:
        broker_ip = sys.argv[1]
    else:
        broker_ip = DEFAULT_BROKER
    
    if len(sys.argv) >= 3:
        username = sys.argv[2]
    else:
        username = DEFAULT_USERNAME
    
    if len(sys.argv) >= 4:
        password = sys.argv[3]
    else:
        password = DEFAULT_PASSWORD
    
    # Create and start listener
    listener = LifeTracMQTTListener(
        broker_ip=broker_ip,
        port=DEFAULT_PORT,
        username=username,
        password=password,
        topic=DEFAULT_TOPIC
    )
    
    print(f"📡 Configuration:")
    print(f"   Broker: {broker_ip}:{DEFAULT_PORT}")
    print(f"   Topic: {DEFAULT_TOPIC}")
    print(f"   Auth: {username}/**********")
    print()
    
    listener.start_listening()

if __name__ == "__main__":
    main()