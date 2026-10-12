export PATH=$PATH:/usr/sbin:/sbin
echo "## date/uptime"; date -u +%Y-%m-%dT%H:%M:%SZ; uptime; cat /proc/sys/kernel/random/boot_id
echo "## unit state"; for u in lifetrac-base.service lifetrac-base-compose.service; do printf '%s active=%s enabled=%s\n' $u "$(systemctl is-active $u 2>&1)" "$(systemctl is-enabled $u 2>&1)"; done
echo "## uart holders"; sudo -n sh -c 'if command -v fuser >/dev/null; then fuser /dev/ttymxc3; echo; echo __UART_CHK_DONE__; else echo __UART_NO_FUSER__; fi' 2>&1
echo "## docker ps -a"; sudo -n docker ps -a --format '{{.Names}}\t{{.Image}}\t{{.Status}}'
echo "## compose ls"; sudo -n docker compose ls -a
echo "## networks"; sudo -n docker network ls --format '{{.Name}}'
echo "## volumes"; sudo -n docker volume ls --format '{{.Name}}'
echo "## unit files"; ls -la /etc/systemd/system/ | grep -i lifetrac; ls -la /etc/systemd/system/lifetrac-base.service.d/ /etc/systemd/system/multi-user.target.wants/ | grep -i -E "lifetrac|timeout|^/"
echo "## retired dir"; ls -la /etc/lifetrac-retired 2>&1
echo "## who wants them"; systemctl show lifetrac-base.service lifetrac-base-compose.service -p WantedBy -p RequiredBy -p Id --no-pager
echo "## failed"; systemctl --failed --no-legend --no-pager
