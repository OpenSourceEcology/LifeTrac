#!/bin/bash
# LifeTrac v25 Web Controller System Test Script
#
# This script tests all components of the web controller system

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}LifeTrac v25 Web Controller System Test${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

# Test 1: Check Python dependencies
echo -e "${YELLOW}[1/7] Checking Python dependencies...${NC}"
MISSING_DEPS=0
for pkg in flask flask_socketio paho.mqtt eventlet; do
    if python3 -c "import $pkg" 2>/dev/null; then
        echo -e "${GREEN}✓${NC} $pkg installed"
    else
        echo -e "${RED}✗${NC} $pkg NOT installed"
        MISSING_DEPS=1
    fi
done

if [ $MISSING_DEPS -eq 1 ]; then
    echo -e "${YELLOW}Install missing dependencies with: pip3 install -r requirements.txt${NC}"
fi
echo ""

# Test 2: Check libcamera
echo -e "${YELLOW}[2/7] Checking libcamera installation...${NC}"
if command -v libcamera-hello &> /dev/null; then
    echo -e "${GREEN}✓${NC} libcamera-hello found"
    if command -v libcamera-vid &> /dev/null; then
        echo -e "${GREEN}✓${NC} libcamera-vid found"
    else
        echo -e "${RED}✗${NC} libcamera-vid NOT found"
    fi
else
    echo -e "${RED}✗${NC} libcamera NOT installed"
    echo -e "${YELLOW}Install with: sudo apt install libcamera-apps${NC}"
fi
echo ""

# Test 3: Check camera
echo -e "${YELLOW}[3/7] Checking camera connection...${NC}"
if libcamera-hello --list-cameras 2>&1 | grep -q "Available cameras"; then
    echo -e "${GREEN}✓${NC} Camera detected"
    libcamera-hello --list-cameras 2>&1 | grep -A 2 "Available cameras"
else
    echo -e "${RED}✗${NC} No camera detected"
    echo -e "${YELLOW}Check camera cable connection and enable camera interface${NC}"
fi
echo ""

# Test 4: Check Mosquitto
echo -e "${YELLOW}[4/7] Checking MQTT broker...${NC}"
if systemctl is-active --quiet mosquitto; then
    echo -e "${GREEN}✓${NC} Mosquitto is running"
    
    # Test connection
    if mosquitto_pub -h localhost -u lifetrac -P lifetrac_pass -t "test" -m "test" 2>/dev/null; then
        echo -e "${GREEN}✓${NC} MQTT authentication working"
    else
        echo -e "${RED}✗${NC} MQTT authentication failed"
        echo -e "${YELLOW}Check MQTT credentials in /etc/mosquitto/passwd${NC}"
    fi
else
    echo -e "${RED}✗${NC} Mosquitto is NOT running"
    echo -e "${YELLOW}Start with: sudo systemctl start mosquitto${NC}"
fi
echo ""

# Test 5: Check network
echo -e "${YELLOW}[5/7] Checking network configuration...${NC}"
IP_ADDR=$(hostname -I | awk '{print $1}')
if [ -n "$IP_ADDR" ]; then
    echo -e "${GREEN}✓${NC} Network connected"
    echo -e "   IP Address: $IP_ADDR"
    echo -e "   Web Interface: http://$IP_ADDR:5000"
else
    echo -e "${RED}✗${NC} No network connection"
fi
echo ""

# Test 6: Check service status
echo -e "${YELLOW}[6/7] Checking web controller service...${NC}"
if systemctl list-unit-files | grep -q lifetrac-web-controller.service; then
    echo -e "${GREEN}✓${NC} Service installed"
    
    if systemctl is-active --quiet lifetrac-web-controller.service; then
        echo -e "${GREEN}✓${NC} Service is running"
    else
        echo -e "${YELLOW}⚠${NC} Service is not running"
        echo -e "${YELLOW}Start with: sudo systemctl start lifetrac-web-controller.service${NC}"
    fi
    
    if systemctl is-enabled --quiet lifetrac-web-controller.service; then
        echo -e "${GREEN}✓${NC} Service is enabled (auto-start on boot)"
    else
        echo -e "${YELLOW}⚠${NC} Service is not enabled"
        echo -e "${YELLOW}Enable with: sudo systemctl enable lifetrac-web-controller.service${NC}"
    fi
else
    echo -e "${RED}✗${NC} Service NOT installed"
    echo -e "${YELLOW}Install with: sudo cp lifetrac-web-controller.service /etc/systemd/system/${NC}"
fi
echo ""

# Test 7: Check firewall
echo -e "${YELLOW}[7/7] Checking firewall configuration...${NC}"
if command -v ufw &> /dev/null; then
    if sudo ufw status | grep -q "Status: active"; then
        echo -e "${GREEN}✓${NC} UFW firewall is active"
        
        if sudo ufw status | grep -q "5000.*ALLOW"; then
            echo -e "${GREEN}✓${NC} Port 5000 (Web Interface) is open"
        else
            echo -e "${YELLOW}⚠${NC} Port 5000 not explicitly allowed"
            echo -e "${YELLOW}Add with: sudo ufw allow 5000/tcp${NC}"
        fi
        
        if sudo ufw status | grep -q "1883.*ALLOW"; then
            echo -e "${GREEN}✓${NC} Port 1883 (MQTT) is open"
        else
            echo -e "${YELLOW}⚠${NC} Port 1883 not explicitly allowed"
            echo -e "${YELLOW}Add with: sudo ufw allow 1883/tcp${NC}"
        fi
    else
        echo -e "${YELLOW}⚠${NC} UFW firewall is not active"
    fi
else
    echo -e "${YELLOW}⚠${NC} UFW not installed (firewall not configured)"
fi
echo ""

# Summary
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}Test Summary${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "${YELLOW}Next Steps:${NC}"
echo "1. Fix any issues marked with ${RED}✗${NC} or ${YELLOW}⚠${NC}"
echo "2. Start the web controller: python3 app.py"
echo "3. Or start as service: sudo systemctl start lifetrac-web-controller.service"
echo "4. Access web interface at: http://$IP_ADDR:5000"
echo ""
echo -e "${YELLOW}Useful Commands:${NC}"
echo "View logs:       sudo journalctl -u lifetrac-web-controller.service -f"
echo "Test camera:     libcamera-hello --timeout 5000"
echo "Test MQTT:       mosquitto_sub -h localhost -u lifetrac -P lifetrac_pass -t 'lifetrac/v25/#' -v"
echo "Check service:   sudo systemctl status lifetrac-web-controller.service"
echo ""
