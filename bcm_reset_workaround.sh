#!/bin/sh
set -x

echo "=== step 1: unbind sdhci to un-wedge any blocked driver threads ==="
echo 30b40000.mmc > /sys/bus/platform/drivers/sdhci-esdhc-imx/unbind || true
sleep 2

echo "=== step 2: unload brcmfmac module ==="
rmmod brcmfmac || modprobe -r brcmfmac || true
sleep 1

echo "=== step 3: reload brcmfmac with txglomsz=0 (disables SDIO Tx packet aggregation) ==="
modprobe brcmfmac txglomsz=0
sleep 1

echo "=== step 4: rebind sdhci controller ==="
echo 30b40000.mmc > /sys/bus/platform/drivers/sdhci-esdhc-imx/bind
sleep 3

echo "=== step 5: force runtime PM active ==="
echo on > /sys/class/mmc_host/mmc0/device/power/control || true
sleep 5

echo "=== step 6: check if sdio card and interface appeared ==="
ls /sys/bus/sdio/devices/ || true
ip link show wlan0 || true

echo "=== step 7: run scans / checks ==="
nmcli dev wifi rescan ifname wlan0 2>/dev/null || true
sleep 4
nmcli -t -f SSID,SIGNAL dev wifi list ifname wlan0 | head -n 12

echo "=== dmesg ==="
dmesg | tail -n 25
