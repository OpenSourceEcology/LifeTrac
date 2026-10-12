set -u
export PATH=$PATH:/usr/sbin:/sbin
TS=$1
UNITS="lifetrac-base.service lifetrac-base-compose.service"
echo "## guard: units must be failed or inactive (never stop an active one: ExecStop = compose down)"
for u in $UNITS; do a=$(systemctl is-active $u 2>/dev/null); echo "$u is-active=$a"; case $a in failed|inactive) ;; *) echo "ABORT: $u is $a"; exit 1;; esac; done
echo "## step 1: disable (no --now) + reset-failed"
sudo -n systemctl disable $UNITS 2>&1
sudo -n systemctl reset-failed $UNITS 2>&1
for u in $UNITS; do echo "$u is-enabled=$(systemctl is-enabled $u 2>&1) is-active=$(systemctl is-active $u 2>&1)"; done
echo "## step 2: move the unit files aside"
D=/etc/lifetrac-retired/$TS
if [ -e "$D" ]; then echo "ABORT: $D exists"; exit 1; fi
sudo -n mkdir -p "$D" && sudo -n mv /etc/systemd/system/lifetrac-base.service /etc/systemd/system/lifetrac-base.service.d /etc/systemd/system/lifetrac-base-compose.service "$D"/ || { echo "ABORT: mv failed"; exit 1; }
printf '%s\n' "Retired $TS (PC UTC) by the bench operator's decision: these units were enabled and failed at every boot;" "if they ever succeeded they would start a radio daemon on /dev/ttymxc3 without a GO." "Do not reinstall on a bench board. See bench_tools/BENCH_BOARDS.md in the LifeTrac repo." "Undo: mv these back to /etc/systemd/system/, systemctl daemon-reload, systemctl enable <unit>." | sudo -n tee "$D/README.txt" >/dev/null
sudo -n systemctl daemon-reload
echo "## after"
systemctl list-unit-files 'lifetrac-base*' --no-pager 2>&1
ls -laR /etc/lifetrac-retired
ls -la /etc/systemd/system/multi-user.target.wants/ | grep -i lifetrac || echo "no lifetrac wants links"
echo "## failed units"; systemctl --failed --no-legend --no-pager
echo "## broker"; sudo -n docker ps --format '{{.Names}}\t{{.Status}}'
echo "## uart holders"; sudo -n sh -c 'if command -v fuser >/dev/null; then fuser /dev/ttymxc3; echo; echo __UART_CHK_DONE__; else echo __UART_NO_FUSER__; fi' 2>&1
