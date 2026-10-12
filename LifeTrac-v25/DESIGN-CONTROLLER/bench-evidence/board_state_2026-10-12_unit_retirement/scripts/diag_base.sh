export PATH=$PATH:/usr/sbin:/sbin
echo "## uptime"; uptime; cat /proc/sys/kernel/random/boot_id
echo "## journald storage"; grep -v '^#' /etc/systemd/journald.conf | grep -v '^$'; sudo -n journalctl --list-boots --no-pager 2>&1 | tail -5
for u in lifetrac-base.service lifetrac-base-compose.service; do
  echo "## status $u"; sudo -n systemctl status $u --no-pager -l 2>&1 | head -30
  echo "## journal -b $u"; sudo -n journalctl -b -u $u --no-pager -o short-iso 2>&1 | tail -60
  echo "## show $u"; sudo -n systemctl show $u -p Result -p ExecMainStatus -p ActiveEnterTimestamp -p InactiveEnterTimestamp -p ExecMainStartTimestamp -p ExecMainExitTimestamp --no-pager
done
echo "## workdir"; ls -la /opt/lifetrac/DESIGN-CONTROLLER/ | head -60
echo "## env keys"; sudo -n sed 's/=.*/=<set>/' /opt/lifetrac/DESIGN-CONTROLLER/.env 2>&1
echo "## compose services (main)"; cd /opt/lifetrac/DESIGN-CONTROLLER && sudo -n docker compose config --services 2>&1 | head -20
echo "## compose services (video-test)"; sudo -n docker compose -p lifetrac-vtest -f docker-compose.video-test.yml config --services 2>&1 | head -20
echo "## compose images"; sudo -n docker compose config 2>/dev/null | grep -E '^\s+(image|build|container_name|devices|restart):|/dev/' | head -40
echo "## vtest images"; sudo -n docker compose -p lifetrac-vtest -f docker-compose.video-test.yml config 2>/dev/null | grep -E '^\s+(image|build|container_name|restart):|/dev/|ports|published' | head -40
echo "## docker ps -a"; sudo -n docker ps -a --format '{{.Names}}\t{{.Image}}\t{{.Status}}' 2>&1
echo "## docker compose ls"; sudo -n docker compose ls -a 2>&1
echo "## images"; sudo -n docker images --format '{{.Repository}}:{{.Tag}} {{.ID}} {{.CreatedSince}}' 2>&1
echo "## ttymxc3 holders"; sudo -n fuser -v /dev/ttymxc3 2>&1 | head
echo "## failed"; systemctl --failed --no-pager
