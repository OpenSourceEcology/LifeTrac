# Quick Start Guide - Raspberry Pi Web Controller

This guide will get you up and running with the LifeTrac v25 web controller in under 30 minutes.

## Prerequisites

- Raspberry Pi 3B+ or newer (4GB+ recommended)
- Arducam IMX335 camera module connected
- MicroSD card with Raspberry Pi OS installed
- Network connection (WiFi or Ethernet)
- Arduino Opta controller already set up and running

## Installation Steps

### 1. Clone Repository

```bash
cd ~
git clone https://github.com/OpenSourceEcology/LifeTrac.git
cd LifeTrac/LifeTrac-v25/raspberry_pi_web_controller
```

### 2. Run Installation Script

```bash
sudo ./install.sh
```

The script will:
- Update system packages
- Install Python dependencies
- Configure MQTT broker
- Enable camera interface
- Install systemd service
- Configure firewall

### 3. Reboot

```bash
sudo reboot
```

### 4. Access Web Interface

After reboot, open a web browser on any device on the same network:

```
http://<raspberry-pi-ip>:5000
```

For example: `http://192.168.1.100:5000`

**How to find your Raspberry Pi's IP:**
```bash
hostname -I
```

## Using the Interface

### Touch Controls (Phone/Tablet)

1. **Left Joystick** (left side): 
   - Drag to control tank steering
   - Up/Down: Forward/Backward
   - Left/Right: Turning

2. **Right Joystick** (right side):
   - Drag to control hydraulics
   - Up/Down: Arms up/down
   - Left/Right: Bucket down/up

3. **Emergency Stop** (center):
   - Tap red button to stop all movement

### Keyboard Controls (Desktop)

| Key | Action |
|-----|--------|
| W | Forward |
| S | Backward |
| A | Left Turn |
| D | Right Turn |
| I | Arms Up |
| K | Arms Down |
| J | Bucket Down |
| L | Bucket Up |
| Space | Emergency Stop |

## Troubleshooting

### Camera Not Showing

```bash
# Check camera connection
libcamera-hello --list-cameras

# If no camera found, check cable connection and try:
sudo raspi-config
# Navigate to: Interface Options -> Camera -> Enable
sudo reboot
```

### Can't Access Web Interface

```bash
# Check if service is running
sudo systemctl status lifetrac-web-controller.service

# View logs
sudo journalctl -u lifetrac-web-controller.service -f

# Restart service
sudo systemctl restart lifetrac-web-controller.service
```

### MQTT Not Connected

```bash
# Check MQTT broker is running
sudo systemctl status mosquitto

# Test MQTT connection
mosquitto_sub -h localhost -u lifetrac -P lifetrac_pass -t "lifetrac/v25/#" -v

# Restart MQTT broker
sudo systemctl restart mosquitto
```

### High Latency or Lag

1. Reduce video resolution in `config/config.yaml`:
   ```yaml
   camera:
     width: 800
     height: 600
     fps: 20
   ```

2. Use Ethernet instead of WiFi if possible

3. Check Raspberry Pi temperature:
   ```bash
   vcgencmd measure_temp
   # Should be below 80°C
   ```

## Manual Control

If you need to control the service manually:

```bash
# Start service
sudo systemctl start lifetrac-web-controller.service

# Stop service
sudo systemctl stop lifetrac-web-controller.service

# Restart service
sudo systemctl restart lifetrac-web-controller.service

# Disable auto-start
sudo systemctl disable lifetrac-web-controller.service

# Enable auto-start
sudo systemctl enable lifetrac-web-controller.service

# View real-time logs
sudo journalctl -u lifetrac-web-controller.service -f
```

## Testing Without Arduino Opta

You can test the web interface without the Arduino Opta:

```bash
# Monitor MQTT messages
mosquitto_sub -h localhost -u lifetrac -P lifetrac_pass -t "lifetrac/v25/control" -v
```

Move the joysticks in the web interface and you'll see MQTT messages being published.

## Next Steps

- **Configure Network**: Set static IP for Raspberry Pi (see README.md)
- **Adjust Camera Settings**: Modify resolution/framerate in `config/config.yaml`
- **Security**: Change default MQTT password
- **Integration**: Connect Arduino Opta controller

## Getting Help

- Check full documentation: [README.md](README.md)
- View system status in web interface debug console
- Check logs: `sudo journalctl -u lifetrac-web-controller.service -f`

## Safety Reminder

⚠️ **Always test in a safe environment first!**

- Keep emergency stop easily accessible
- Test all controls before actual operation
- Ensure good network connection
- Have a backup control method available
