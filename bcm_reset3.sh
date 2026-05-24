#!/bin/sh
set -x
echo "=== unbind brcmfmac if alive ==="
rmmod brcmfmac 2>&1 || true
sleep 1
echo "=== unbind sdhci 30b40000.mmc ==="
echo 30b40000.mmc > /sys/bus/platform/drivers/sdhci-esdhc-imx/unbind
sleep 5
cat /sys/class/regulator/regulator.3/state
echo "=== rebind sdhci ==="
echo 30b40000.mmc > /sys/bus/platform/drivers/sdhci-esdhc-imx/bind
sleep 2
echo "=== force runtime PM resume on mmc0 host ==="
echo on > /sys/class/mmc_host/mmc0/device/power/control
sleep 1
cat /sys/class/mmc_host/mmc0/device/power/runtime_status
cat /sys/class/regulator/regulator.3/state
sleep 5
echo "=== post-resume regulator + sdio ==="
cat /sys/class/regulator/regulator.3/state
ls /sys/bus/sdio/devices/
echo "=== load brcmfmac ==="
modprobe brcmfmac
sleep 10
echo "=== final ==="
ls /sys/bus/sdio/devices/
ip link show wlan0 2>&1
dmesg | tail -n 30
