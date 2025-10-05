#!/usr/bin/env python3
"""
LifeTrac v25 Raspberry Pi Web Controller

This application provides a web interface for controlling the LifeTrac v25
with live video feed from Arducam IMX335 camera and on-screen joystick controls.

Features:
- Live video streaming using libcamera
- Touch-enabled on-screen joysticks
- Keyboard shortcuts for control
- MQTT integration for sending commands to Arduino Opta
- WebSocket for real-time communication

Requirements:
    pip install flask flask-socketio paho-mqtt eventlet
    
Hardware:
    - Raspberry Pi (3B+ or newer)
    - Arducam IMX335 camera module
    
Usage:
    python3 app.py
    
Access the interface at:
    http://<raspberry-pi-ip>:5000
"""

import io
import time
import json
import subprocess
import threading
from datetime import datetime
from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO, emit
import paho.mqtt.client as mqtt

app = Flask(__name__)
app.config['SECRET_KEY'] = 'lifetrac_v25_secret_key_change_in_production'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# MQTT Configuration
MQTT_BROKER = "192.168.1.100"  # Raspberry Pi's own IP or external broker
MQTT_PORT = 1883
MQTT_USER = "lifetrac"
MQTT_PASSWORD = "lifetrac_pass"
MQTT_CONTROL_TOPIC = "lifetrac/v25/control"
MQTT_STATUS_TOPIC = "lifetrac/v25/status"

# Camera Configuration
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30

# Control state
current_control = {
    'left_x': 0.0,
    'left_y': 0.0,
    'right_x': 0.0,
    'right_y': 0.0,
    'timestamp': 0
}

# MQTT Client
mqtt_client = mqtt.Client()
mqtt_connected = False

# Camera process
camera_process = None
camera_stream_active = False


def on_mqtt_connect(client, userdata, flags, rc):
    """MQTT connection callback"""
    global mqtt_connected
    if rc == 0:
        print(f"Connected to MQTT broker at {MQTT_BROKER}")
        mqtt_connected = True
        client.subscribe(MQTT_STATUS_TOPIC)
    else:
        print(f"Failed to connect to MQTT broker. Return code: {rc}")
        mqtt_connected = False


def on_mqtt_message(client, userdata, msg):
    """MQTT message callback - receive status updates from LifeTrac"""
    try:
        payload = json.loads(msg.payload.decode())
        # Broadcast status to all connected web clients
        socketio.emit('lifetrac_status', payload, namespace='/')
    except json.JSONDecodeError:
        pass


def init_mqtt():
    """Initialize MQTT connection"""
    mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    mqtt_client.on_connect = on_mqtt_connect
    mqtt_client.on_message = on_mqtt_message
    
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
        print("MQTT client started")
    except Exception as e:
        print(f"Error connecting to MQTT broker: {e}")


def send_control_command(left_x, left_y, right_x, right_y):
    """Send control command via MQTT"""
    global current_control
    
    command = {
        'left_x': left_x,
        'left_y': left_y,
        'right_x': right_x,
        'right_y': right_y,
        'timestamp': int(time.time() * 1000)
    }
    
    current_control = command
    
    if mqtt_connected:
        message = json.dumps(command)
        result = mqtt_client.publish(MQTT_CONTROL_TOPIC, message)
        return result.rc == mqtt.MQTT_ERR_SUCCESS
    return False


def generate_camera_frames():
    """Generate camera frames using libcamera"""
    global camera_process, camera_stream_active
    
    # Use libcamera-vid to capture and stream
    # Output MJPEG stream to stdout
    cmd = [
        'libcamera-vid',
        '--inline',
        '--nopreview',
        '-t', '0',  # Run indefinitely
        '--width', str(CAMERA_WIDTH),
        '--height', str(CAMERA_HEIGHT),
        '--framerate', str(CAMERA_FPS),
        '--codec', 'mjpeg',
        '-o', '-'  # Output to stdout
    ]
    
    try:
        camera_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=10**8
        )
        
        camera_stream_active = True
        print("Camera stream started")
        
        # Read MJPEG stream
        while camera_stream_active:
            # Find JPEG start marker (0xFFD8)
            start_marker = b'\xff\xd8'
            end_marker = b'\xff\xd9'
            
            # Read until we find start marker
            frame_data = b''
            while True:
                chunk = camera_process.stdout.read(1)
                if not chunk:
                    break
                frame_data += chunk
                if frame_data.endswith(start_marker):
                    frame_data = start_marker
                    break
            
            # Read until we find end marker
            while True:
                chunk = camera_process.stdout.read(1)
                if not chunk:
                    break
                frame_data += chunk
                if frame_data.endswith(end_marker):
                    break
            
            if len(frame_data) > 0:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')
    
    except Exception as e:
        print(f"Camera error: {e}")
        camera_stream_active = False
    finally:
        if camera_process:
            camera_process.terminate()
            camera_process = None


@app.route('/')
def index():
    """Main control interface"""
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(
        generate_camera_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/status')
def status():
    """Get current system status"""
    return jsonify({
        'mqtt_connected': mqtt_connected,
        'camera_active': camera_stream_active,
        'current_control': current_control,
        'timestamp': int(time.time() * 1000)
    })


@socketio.on('connect')
def handle_connect():
    """Handle WebSocket connection"""
    print('Client connected')
    emit('connection_response', {'status': 'connected', 'mqtt_connected': mqtt_connected})


@socketio.on('disconnect')
def handle_disconnect():
    """Handle WebSocket disconnection"""
    print('Client disconnected')


@socketio.on('control_command')
def handle_control_command(data):
    """Handle control command from web interface"""
    try:
        left_x = float(data.get('left_x', 0.0))
        left_y = float(data.get('left_y', 0.0))
        right_x = float(data.get('right_x', 0.0))
        right_y = float(data.get('right_y', 0.0))
        
        # Clamp values to -1.0 to 1.0 range
        left_x = max(-1.0, min(1.0, left_x))
        left_y = max(-1.0, min(1.0, left_y))
        right_x = max(-1.0, min(1.0, right_x))
        right_y = max(-1.0, min(1.0, right_y))
        
        success = send_control_command(left_x, left_y, right_x, right_y)
        
        emit('command_sent', {
            'success': success,
            'left_x': left_x,
            'left_y': left_y,
            'right_x': right_x,
            'right_y': right_y
        })
    except Exception as e:
        print(f"Error handling control command: {e}")
        emit('error', {'message': str(e)})


@socketio.on('emergency_stop')
def handle_emergency_stop():
    """Handle emergency stop command"""
    print("Emergency stop received!")
    send_control_command(0.0, 0.0, 0.0, 0.0)
    emit('emergency_stop_confirmed', {'timestamp': int(time.time() * 1000)})


def cleanup():
    """Cleanup resources on shutdown"""
    global camera_stream_active
    camera_stream_active = False
    if camera_process:
        camera_process.terminate()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()


if __name__ == '__main__':
    try:
        print("Starting LifeTrac v25 Web Controller...")
        print(f"MQTT Broker: {MQTT_BROKER}:{MQTT_PORT}")
        print(f"Camera Resolution: {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps")
        
        # Initialize MQTT
        init_mqtt()
        
        # Give MQTT time to connect
        time.sleep(2)
        
        print("\nWeb interface starting on http://0.0.0.0:5000")
        print("Access from browser: http://<raspberry-pi-ip>:5000")
        print("\nPress Ctrl+C to stop")
        
        # Run Flask app with SocketIO
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        cleanup()
