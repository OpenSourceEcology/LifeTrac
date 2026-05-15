#!/bin/bash
# Unified Recovery Script for all X8 + Max Carrier Chips
# Usage: ./x8_max_carrier_recovery.sh

echo "========================================================"
echo "==== RECOVERING ALL RELEVANT CHIPS                  ===="
echo "========================================================"

echo ""
echo "==== CHIP 1: STM32H747 (Portenta X8 Companion MCU)  ===="
# The standard Arduino way to unbrick / restart the M4 and M7 cores
echo "Restarting Arduino m4-proxy and stm32h7-program services..."
systemctl restart m4-proxy.service || echo "Failed to restart m4-proxy"
systemctl restart stm32h7-program.service || echo "Failed to restart stm32h7-program"
echo "STM32H7 services restarted."

echo ""
echo "==== CHIP 2: STM32L072 (Max Carrier LoRa Module)    ===="
echo "Invoking Option Bytes recovery (runprot / mass-erase)..."
# We assume the existing recovery script is in the same directory or deployed to /tmp
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$DIR/recover_l072_opt.sh" ]; then
   bash "$DIR/recover_l072_opt.sh" --runprot
elif [ -f /tmp/recover_l072_opt.sh ]; then
   bash /tmp/recover_l072_opt.sh --runprot
else
   echo "recover_l072_opt.sh not found! Ensure it is deployed."
fi

echo ""
echo "==== CHIP 3: CS42L52 & I2C Peripherals (Max Carrier)===="
# Resetting the I2C bus if necessary by unbinding and rebinding the I2C driver
echo "To deep-reset the audio codec and crypto element, a hard power cycle is required."

echo ""
echo "==== CHIP 4: i.MX 8M Mini (Linux Host MPU)          ===="
echo "No destructive recovery action taken against host processor here."
echo "If system remains unstable, run 'reboot' or power cycle."
echo "========================================================"
echo "Unified recovery pass complete."
