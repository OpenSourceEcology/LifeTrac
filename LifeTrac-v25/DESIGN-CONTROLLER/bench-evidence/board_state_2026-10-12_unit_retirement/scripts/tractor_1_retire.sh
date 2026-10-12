set -u
export PATH=$PATH:/usr/sbin:/sbin
TS=$1
U=lifetrac-tractor-compose.service
camchk() {
  a=$(systemctl is-active lifetrac-camera.service 2>/dev/null); r=$(sudo -n docker ps --format '{{.Names}}' | grep -x -E 'tractor-camera|tractor-camera-v2|tractor-image-tx-v2|camera_svc|tx_smoke' | tr '\n' ' ')
  echo "camera check ($1): lifetrac-camera.service=$a running-radio-containers=[${r}]"
  if [ "$a" = active ] || [ "$a" = activating ] || [ -n "$r" ]; then
    echo "!! something started: applying the standard bench stop"
    sudo -n systemctl stop lifetrac-camera.service; [ -n "$r" ] && sudo -n docker stop $r
  fi
}
echo "## guard"
a=$(systemctl is-active $U 2>/dev/null); echo "$U is-active=$a"; case $a in failed|inactive) ;; *) echo "ABORT: $U is $a"; exit 1;; esac
camchk before
echo "## step 1: disable (no --now) + reset-failed"
sudo -n systemctl disable $U 2>&1
camchk after-disable
sudo -n systemctl reset-failed $U 2>&1
echo "$U is-enabled=$(systemctl is-enabled $U 2>&1) is-active=$(systemctl is-active $U 2>&1)"
echo "## step 2: move the unit file aside"
D=/etc/lifetrac-retired/$TS
if [ -e "$D" ]; then echo "ABORT: $D exists"; exit 1; fi
sudo -n mkdir -p "$D" && sudo -n mv /etc/systemd/system/$U "$D"/ || { echo "ABORT: mv failed"; exit 1; }
printf '%s\n' "Retired $TS (PC UTC; this board's clock is not wall time) by the bench operator's decision: this unit was enabled and" "failed at every boot; if it ever succeeded it would start image_tx on /dev/ttymxc3 and transmit without a GO." "Do not reinstall on a bench board. See bench_tools/BENCH_BOARDS.md in the LifeTrac repo." "Undo: mv it back to /etc/systemd/system/, systemctl daemon-reload, systemctl enable <unit>." | sudo -n tee "$D/README.txt" >/dev/null
sudo -n systemctl daemon-reload
camchk after-daemon-reload
sleep 5
camchk 5s-later
echo "## after"
systemctl list-unit-files "$U" --no-pager 2>&1
ls -laR /etc/lifetrac-retired
ls -la /etc/systemd/system/multi-user.target.wants/ | grep -i lifetrac || echo "no lifetrac wants links"
echo "## failed units"; systemctl --failed --no-legend --no-pager
echo "## uart holders"; sudo -n sh -c 'if command -v fuser >/dev/null; then fuser /dev/ttymxc3; echo; echo __UART_CHK_DONE__; else echo __UART_NO_FUSER__; fi' 2>&1
echo "## running containers"; sudo -n docker ps --format '{{.Names}}\t{{.Status}}'
