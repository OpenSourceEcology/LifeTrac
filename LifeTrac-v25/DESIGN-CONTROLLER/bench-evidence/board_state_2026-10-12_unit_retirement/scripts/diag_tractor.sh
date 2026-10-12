export PATH=$PATH:/usr/sbin:/sbin
echo "## uptime"; uptime; cat /proc/sys/kernel/random/boot_id
sudo -n journalctl --list-boots --no-pager 2>&1 | tail -3
for u in lifetrac-tractor-compose.service lifetrac-camera.service; do
  echo "## status $u"; sudo -n systemctl status $u --no-pager -l 2>&1 | head -25
  echo "## journal -b $u"; sudo -n journalctl -b -u $u --no-pager -o short-iso 2>&1 | tail -50
  echo "## show $u"; sudo -n systemctl show $u -p Result -p ExecMainStatus -p ExecMainStartTimestamp -p ExecMainExitTimestamp --no-pager
done
echo "## workdir"; ls -la /opt/lifetrac/ /opt/lifetrac/video-test/ 2>&1 | head -40
echo "## compose services (video-test)"; cd /opt/lifetrac/video-test 2>/dev/null && sudo -n docker compose -p tractor-vtest config --services 2>&1 | head
echo "## vtest config"; sudo -n docker compose -p tractor-vtest config 2>&1 | grep -E '^\s+(image|build|container_name|restart):|/dev/|published|LIFETRAC_' | head -50
echo "## camera compose app"; sudo -n docker compose -f /opt/lifetrac/compose-apps/lifetrac-camera/docker-compose.yml config 2>&1 | grep -E '^\s+(image|build|container_name|restart):|/dev/|published' | head -20
echo "## docker ps -a"; sudo -n docker ps -a --format '{{.Names}}\t{{.Image}}\t{{.Status}}' 2>&1
echo "## docker compose ls"; sudo -n docker compose ls -a 2>&1
echo "## images"; sudo -n docker images --format '{{.Repository}}:{{.Tag}} {{.ID}} {{.CreatedSince}}' 2>&1
echo "## ttymxc3 holders"; sudo -n fuser -v /dev/ttymxc3 2>&1 | head
echo "## failed"; systemctl --failed --no-pager | head -8
