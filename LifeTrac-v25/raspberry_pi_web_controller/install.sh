#!/bin/bash
# LifeTrac v25 Raspberry Pi Web Controller Installation Script
#
# This script automates the installation of the web controller on Raspberry Pi
#
# Usage: sudo ./install.sh

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo -e "${RED}Please run as root (use sudo)${NC}"
    exit 1
fi

# Get the actual user (not root when using sudo)
ACTUAL_USER=${SUDO_USER:-$USER}
USER_HOME=$(eval echo ~$ACTUAL_USER)

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}LifeTrac v25 Web Controller Installation${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

# Update system
echo -e "${YELLOW}[1/8] Updating system packages...${NC}"
apt update && apt upgrade -y

# Install system dependencies
echo -e "${YELLOW}[2/8] Installing system dependencies...${NC}"
apt install -y python3-pip python3-dev python3-venv git libcamera-apps mosquitto mosquitto-clients

# Install Python packages
echo -e "${YELLOW}[3/8] Installing Python dependencies...${NC}"
pip3 install -r requirements.txt

# Configure Mosquitto
echo -e "${YELLOW}[4/8] Configuring MQTT broker...${NC}"
if [ -f ../config/mosquitto.conf ]; then
    cp ../config/mosquitto.conf /etc/mosquitto/conf.d/lifetrac.conf
    echo -e "${GREEN}Mosquitto configuration installed${NC}"
fi

# Create MQTT password
echo -e "${YELLOW}Setting up MQTT password...${NC}"
echo -e "Creating MQTT user 'lifetrac' with password 'lifetrac_pass'"
mosquitto_passwd -b -c /etc/mosquitto/passwd lifetrac lifetrac_pass

# Restart Mosquitto
systemctl restart mosquitto
systemctl enable mosquitto

# Enable camera interface
echo -e "${YELLOW}[5/8] Enabling camera interface...${NC}"
raspi-config nonint do_camera 0
echo -e "${GREEN}Camera interface enabled${NC}"

# Test camera
echo -e "${YELLOW}[6/8] Testing camera...${NC}"
if libcamera-hello --list-cameras 2>&1 | grep -q "Available cameras"; then
    echo -e "${GREEN}Camera detected successfully${NC}"
else
    echo -e "${RED}Warning: No camera detected. Please check camera connection.${NC}"
    echo -e "${YELLOW}You can continue but camera feed will not work.${NC}"
fi

# Install as systemd service
echo -e "${YELLOW}[7/8] Installing systemd service...${NC}"

# Update service file paths
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TMP_SERVICE_FILE="/tmp/lifetrac-web-controller.service"
cp lifetrac-web-controller.service "$TMP_SERVICE_FILE"
sed -i "s|/home/pi/LifeTrac|${USER_HOME}/LifeTrac|g" "$TMP_SERVICE_FILE"
sed -i "s|User=pi|User=${ACTUAL_USER}|g" "$TMP_SERVICE_FILE"

# Install service
cp "$TMP_SERVICE_FILE" /etc/systemd/system/
rm -f "$TMP_SERVICE_FILE"
systemctl daemon-reload
systemctl enable lifetrac-web-controller.service

echo -e "${GREEN}Service installed and enabled${NC}"

# Configure firewall
echo -e "${YELLOW}[8/8] Configuring firewall...${NC}"
if command -v ufw &> /dev/null; then
    ufw allow 5000/tcp comment "LifeTrac Web Interface"
    ufw allow 1883/tcp comment "MQTT Broker"
    echo -e "${GREEN}Firewall rules added${NC}"
else
    echo -e "${YELLOW}UFW not installed, skipping firewall configuration${NC}"
fi

# Display network information
echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${GREEN}Installation Complete!${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "${YELLOW}Network Information:${NC}"
echo -e "IP Address: $(hostname -I | awk '{print $1}')"
echo -e "Web Interface: http://$(hostname -I | awk '{print $1}'):5000"
echo ""
echo -e "${YELLOW}Next Steps:${NC}"
echo "1. Reboot the Raspberry Pi: sudo reboot"
echo "2. After reboot, the web controller will start automatically"
echo "3. Access the web interface from any browser on your network"
echo ""
echo -e "${YELLOW}Manual Control:${NC}"
echo "Start service:  sudo systemctl start lifetrac-web-controller.service"
echo "Stop service:   sudo systemctl stop lifetrac-web-controller.service"
echo "View logs:      sudo journalctl -u lifetrac-web-controller.service -f"
echo "Check status:   sudo systemctl status lifetrac-web-controller.service"
echo ""
echo -e "${YELLOW}Testing:${NC}"
echo "Test camera:    libcamera-hello --timeout 5000"
echo "Test MQTT:      mosquitto_sub -h localhost -u lifetrac -P lifetrac_pass -t 'lifetrac/v25/#' -v"
echo "Test web app:   cd ${SCRIPT_DIR} && python3 app.py"
echo ""
echo -e "${GREEN}Installation script completed successfully!${NC}"
