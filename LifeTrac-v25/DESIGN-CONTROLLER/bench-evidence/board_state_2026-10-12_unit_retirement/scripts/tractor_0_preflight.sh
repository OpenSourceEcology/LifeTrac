export PATH=$PATH:/usr/sbin:/sbin
echo "## date/uptime (board clock is not wall time)"; date -u +%Y-%m-%dT%H:%M:%SZ; uptime; cat /proc/sys/kernel/random/boot_id
echo "## unit state"; for u in lifetrac-tractor-compose.service lifetrac-camera.service; do printf '%s active=%s enabled=%s\n' $u "$(systemctl is-active $u 2>&1)" "$(systemctl is-enabled $u 2>&1)"; done
echo "## systemd version"; systemctl --version | head -1
echo "## uart holders"; sudo -n sh -c 'if command -v fuser >/dev/null; then fuser /dev/ttymxc3; echo; echo __UART_CHK_DONE__; else echo __UART_NO_FUSER__; fi' 2>&1
echo "## docker ps -a"; sudo -n docker ps -a --format '{{.Names}}\t{{.Image}}\t{{.Status}}'
echo "## compose ls"; sudo -n docker compose ls -a
echo "## networks"; sudo -n docker network ls --format '{{.Name}}'
echo "## volumes"; sudo -n docker volume ls --format '{{.Name}}'
echo "## unit files"; ls -la /etc/systemd/system/ | grep -i -E "lifetrac|wifi"; ls -la /etc/systemd/system/multi-user.target.wants/ | grep -i -E "lifetrac|wifi"
echo "## retired dir"; ls -la /etc/lifetrac-retired 2>&1
echo "## video-test tree"; ls -laR /opt/lifetrac/video-test 2>&1 | head -40
echo "## who wants it"; systemctl show lifetrac-tractor-compose.service -p WantedBy -p RequiredBy --no-pager
echo "## failed"; systemctl --failed --no-legend --no-pager
