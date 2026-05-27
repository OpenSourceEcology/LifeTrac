#!/bin/bash
echo "--- fuser /dev/ttymxc3 ---"
fuser -v /dev/ttymxc3 2>&1
echo "--- lsof /dev/ttymxc3 ---"
lsof /dev/ttymxc3 2>&1 | head -40
echo "--- pgrep -af ttymxc3 / lora / camera / serialrpc / host_cmd ---"
ps -ef | grep -iE 'ttymxc3|lora|camera|murata|host_cmd|serialrpc' | grep -v grep
echo "--- systemctl listing of -lora, -camera, -murata, -host related units ---"
systemctl list-units --no-pager --no-legend 2>&1 | grep -iE 'lora|camera|murata|host|serial|x8h7'
echo "--- dmesg ttymxc3 recent ---"
dmesg | grep -i ttymxc3 | tail -10
