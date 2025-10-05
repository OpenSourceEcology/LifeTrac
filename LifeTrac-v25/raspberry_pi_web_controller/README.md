# LifeTrac v25 Raspberry Pi Web Controller

A web-based control interface for LifeTrac v25 featuring live video feed from Arducam IMX335 camera and touch-enabled joystick controls. Access the tractor controls from any device with a web browser on your local network.

## Features

- **Live Video Streaming**: Real-time camera feed using libcamera for Arducam IMX335
- **Touch-Enabled Joysticks**: Two on-screen joysticks for tank steering and hydraulics
- **Keyboard Controls**: Full keyboard shortcuts for desktop control
- **WebSocket Communication**: Low-latency real-time control updates
- **MQTT Integration**: Seamless communication with Arduino Opta controller
- **Emergency Stop**: Quick-access emergency stop button and spacebar shortcut
- **Responsive Design**: Works on phones, tablets, and desktop browsers
- **Status Monitoring**: Real-time connection and system status display
- **Debug Console**: Built-in debugging interface for troubleshooting

## Hardware Requirements

### Raspberry Pi Setup
- Raspberry Pi 3B+, 4, or 5 (4GB+ RAM recommended)
- MicroSD card (16GB minimum, 32GB recommended)
- Arducam IMX335 camera module
- Power supply (5V 3A for Pi 4/5)
- WiFi connection or Ethernet cable

### Network
- WiFi router or access point
- Same network as Arduino Opta controller
- MQTT broker (can run on the same Raspberry Pi)

## Software Installation

### 1. Prepare Raspberry Pi

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install required system packages
sudo apt install -y python3-pip python3-dev git libcamera-apps

# Enable camera interface
sudo raspi-config
# Navigate to: Interface Options -> Camera -> Enable
```

### 2. Install Python Dependencies

```bash
# Navigate to the web controller directory
cd ~/LifeTrac/LifeTrac-v25/raspberry_pi_web_controller

# Install Python packages
pip3 install -r requirements.txt
```

### 3. Install and Configure Mosquitto MQTT Broker

If not already installed:

```bash
# Install Mosquitto
sudo apt install -y mosquitto mosquitto-clients

# Copy configuration
sudo cp ../config/mosquitto.conf /etc/mosquitto/conf.d/lifetrac.conf

# Create password file
sudo mosquitto_passwd -c /etc/mosquitto/passwd lifetrac
# Enter password: lifetrac_pass

# Restart Mosquitto
sudo systemctl restart mosquitto
sudo systemctl enable mosquitto
```

### 4. Configure the Web Controller

Edit configuration if needed:

```bash
nano config/config.yaml
```

Update the following settings:
- `mqtt.broker`: Your Raspberry Pi's IP address (default: 192.168.1.100)
- `camera.width/height`: Video resolution (default: 1280x720)
- `web.port`: Web server port (default: 5000)

### 5. Test the Camera

Before running the web controller, verify the camera works:

```bash
# Test camera with libcamera
libcamera-hello --timeout 5000

# Test MJPEG streaming
libcamera-vid --inline --nopreview -t 0 --width 1280 --height 720 --framerate 30 --codec mjpeg -o - > test.mjpg
# Press Ctrl+C to stop after a few seconds, then check test.mjpg exists
```

### 6. Run the Web Controller

```bash
python3 app.py
```

The web interface will be available at: `http://<raspberry-pi-ip>:5000`

For example: `http://192.168.1.100:5000`

### 7. Install as System Service (Optional)

To run the web controller automatically on boot:

```bash
# Copy service file
sudo cp lifetrac-web-controller.service /etc/systemd/system/

# Edit the service file to match your installation path
sudo nano /etc/systemd/system/lifetrac-web-controller.service

# Enable and start service
sudo systemctl daemon-reload
sudo systemctl enable lifetrac-web-controller.service
sudo systemctl start lifetrac-web-controller.service

# Check status
sudo systemctl status lifetrac-web-controller.service

# View logs
sudo journalctl -u lifetrac-web-controller.service -f
```

## Usage

### Web Interface

1. **Access**: Open a web browser and navigate to `http://<raspberry-pi-ip>:5000`
2. **Video Feed**: Live camera feed displays at the top
3. **Left Joystick**: Controls tank steering
   - Up/Down: Forward/Backward movement
   - Left/Right: Differential steering (turning)
4. **Right Joystick**: Controls hydraulics
   - Up/Down: Arms up/down
   - Left/Right: Bucket down/up
5. **Emergency Stop**: Red button in center or press Spacebar

### Keyboard Controls

| Key | Function |
|-----|----------|
| W | Forward |
| S | Backward |
| A | Left Turn |
| D | Right Turn |
| I | Arms Up |
| K | Arms Down |
| J | Bucket Down |
| L | Bucket Up |
| Space | Emergency Stop |

### Touch Controls

- **Joysticks**: Touch and drag on the circular joystick areas
- **Emergency Stop**: Tap the red emergency stop button
- **Multi-touch**: Both joysticks can be operated simultaneously

## Network Configuration

### Static IP for Raspberry Pi

Edit `/etc/dhcpcd.conf`:

```bash
sudo nano /etc/dhcpcd.conf
```

Add at the end:

```
interface wlan0
static ip_address=192.168.1.100/24
static routers=192.168.1.1
static domain_name_servers=192.168.1.1 8.8.8.8
```

Reboot to apply:

```bash
sudo reboot
```

### WiFi Configuration

Edit WiFi settings in `/etc/wpa_supplicant/wpa_supplicant.conf`:

```bash
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```

Add network:

```
network={
    ssid="YOUR_WIFI_SSID"
    psk="YOUR_WIFI_PASSWORD"
    key_mgmt=WPA-PSK
}
```

## Troubleshooting

### Camera Not Working

1. Check camera is properly connected:
   ```bash
   libcamera-hello --list-cameras
   ```

2. Ensure camera interface is enabled:
   ```bash
   sudo raspi-config
   # Interface Options -> Camera -> Enable
   ```

3. Update libcamera:
   ```bash
   sudo apt update && sudo apt upgrade libcamera-apps
   ```

### MQTT Connection Failed

1. Check Mosquitto is running:
   ```bash
   sudo systemctl status mosquitto
   ```

2. Test MQTT connection:
   ```bash
   mosquitto_sub -h 192.168.1.100 -u lifetrac -P lifetrac_pass -t "lifetrac/v25/#" -v
   ```

3. Check firewall:
   ```bash
   sudo ufw allow 1883/tcp
   ```

### Web Interface Not Accessible

1. Check the app is running:
   ```bash
   sudo systemctl status lifetrac-web-controller.service
   ```

2. Check port is listening:
   ```bash
   sudo netstat -tulpn | grep 5000
   ```

3. Test from Raspberry Pi itself:
   ```bash
   curl http://localhost:5000
   ```

4. Check firewall:
   ```bash
   sudo ufw allow 5000/tcp
   ```

### Video Feed Not Displaying

1. Check browser console for errors (F12 -> Console)
2. Verify camera is working (see Camera Not Working above)
3. Try reducing resolution in `config/config.yaml`
4. Check CPU temperature (camera streaming is CPU-intensive):
   ```bash
   vcgencmd measure_temp
   # Should be < 80°C
   ```

### High Latency

1. Reduce video resolution in config
2. Lower frame rate (try 15-20 FPS)
3. Use Ethernet instead of WiFi
4. Ensure Raspberry Pi is not overheating
5. Close other applications on the Pi

## Performance Optimization

### For Raspberry Pi 3B+
```yaml
camera:
  width: 800
  height: 600
  fps: 20
```

### For Raspberry Pi 4/5
```yaml
camera:
  width: 1280
  height: 720
  fps: 30
```

### For Remote Access (Lower Bandwidth)
```yaml
camera:
  width: 640
  height: 480
  fps: 15
```

## Security Considerations

⚠️ **IMPORTANT**: This system is designed for LOCAL NETWORK use only!

### Recommended Security Measures

1. **Change Default Passwords**:
   - Update MQTT password in `config/config.yaml`
   - Change Raspberry Pi password: `passwd`

2. **Firewall Configuration**:
   ```bash
   sudo apt install ufw
   sudo ufw default deny incoming
   sudo ufw default allow outgoing
   sudo ufw allow 22/tcp   # SSH (from trusted IPs only)
   sudo ufw allow 5000/tcp # Web interface
   sudo ufw allow 1883/tcp # MQTT
   sudo ufw enable
   ```

3. **Disable SSH Password Authentication**:
   Use SSH keys instead of passwords for remote access.

4. **Network Isolation**:
   - Use a separate VLAN for LifeTrac equipment
   - Don't expose to the internet without VPN

5. **Regular Updates**:
   ```bash
   sudo apt update && sudo apt upgrade
   ```

## Integration with Existing System

This web controller integrates seamlessly with the existing LifeTrac v25 system:

- **Arduino Opta**: Receives MQTT commands on `lifetrac/v25/control` topic
- **ESP32 Remote**: Can coexist on the same network
- **ROS2 Bridge**: Compatible with ROS2 control systems
- **MQTT Broker**: Shared by all components

Multiple control methods can be used simultaneously, but be aware of conflicting commands.

## Development and Customization

### Adding Custom Features

The modular design allows easy customization:

- `app.py`: Main Flask application and MQTT logic
- `templates/index.html`: Web interface structure
- `static/css/style.css`: Visual styling
- `static/js/controller.js`: Client-side control logic

### Testing Without Hardware

You can test the web interface without camera:

1. Comment out camera-related code in `app.py`
2. Replace video feed with a placeholder image
3. Use MQTT test scripts from `../test_scripts/` to simulate responses

### API Endpoints

- `GET /`: Main control interface
- `GET /video_feed`: MJPEG video stream
- `GET /status`: System status JSON
- WebSocket `/`: Real-time control and status updates

## Support and Troubleshooting

For issues and support:

1. Check the debug console in the web interface
2. View application logs: `sudo journalctl -u lifetrac-web-controller.service -f`
3. Test MQTT separately using `../test_scripts/mqtt_test.py`
4. Refer to [INSTALLATION_GUIDE.md](../INSTALLATION_GUIDE.md) for overall system setup

## License

Part of the Open Source Ecology LifeTrac project.

## Contributing

Contributions welcome! Please test thoroughly before submitting pull requests.

## Changelog

### Version 1.0 (Initial Release)
- Live video streaming with libcamera
- Touch-enabled joystick controls
- Keyboard shortcuts
- MQTT integration
- Emergency stop functionality
- Debug console
- Systemd service support
