#!/bin/sh
set -x
echo "=== step 1: remove brcmfmac module ==="
rmmod brcmfmac 2>&1 || modprobe -r brcmfmac 2>&1
sleep 1
echo "=== step 2: unbind sdhci 30b40000.mmc ==="
echo 30b40000.mmc > /sys/bus/platform/drivers/sdhci-esdhc-imx/unbind
sleep 5
echo "=== step 2b: confirm mmc0 gone ==="
ls /sys/class/mmc_host/
echo "=== step 2c: regulator state (should be disabled, chip unpowered) ==="
cat /sys/class/regulator/regulator.3/state
echo "=== step 3: rebind sdhci ==="
echo 30b40000.mmc > /sys/bus/platform/drivers/sdhci-esdhc-imx/bind
sleep 6
echo "=== step 4: post-bind status ==="
ls /sys/class/mmc_host/
ls /sys/bus/sdio/devices/
cat /sys/class/regulator/regulator.3/state
echo "=== step 5: load brcmfmac ==="
modprobe brcmfmac
sleep 8
echo "=== step 6: final status ==="
ls /sys/bus/sdio/devices/
ip link show wlan0 2>&1
echo "=== dmesg tail ==="
dmesg | tail -n 40
