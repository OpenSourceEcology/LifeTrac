set -u
export PATH=$PATH:/usr/sbin:/sbin
C="tractor-image-tx-v2 tractor-camera-v2 tractor-mosquitto-v2"
VT=/var/rootdirs/opt/lifetrac/video-test
echo "## guard: unit gone or not enabled"
e=$(systemctl is-enabled lifetrac-tractor-compose.service 2>&1); echo "lifetrac-tractor-compose.service is-enabled: $e"
case $e in enabled*) echo "ABORT: unit still enabled"; exit 1;; esac
echo "## guard: containers exited, project video-test"
for c in $C; do s=$(sudo -n docker inspect -f '{{.State.Status}} project={{index .Config.Labels "com.docker.compose.project"}}' $c 2>&1); echo "$c: $s"; case $s in "exited project=video-test") ;; *) echo "ABORT: unexpected state"; exit 1;; esac; done
echo "## step 4: disarm the compose file (rename, nothing deleted)"
[ -f $VT/docker-compose.yml ] || { echo "ABORT: no $VT/docker-compose.yml"; exit 1; }
sudo -n mv $VT/docker-compose.yml $VT/docker-compose.yml.RETIRED-2026-10-12 || { echo "ABORT: rename failed"; exit 1; }
ls -la $VT
echo "-- compose now finds no file here (config only parses, starts nothing):"
(cd $VT && sudo -n docker compose -p tractor-vtest config --services 2>&1 | head -3)
echo "## step 5: remove the three exited containers (no -f, no -v) and their two networks"
sudo -n docker rm $C && sudo -n docker network rm video-test_default tractor-vtest_default
echo "## after"
sudo -n docker ps -a --format '{{.Names}}\t{{.Image}}\t{{.Status}}'
sudo -n docker compose ls -a
sudo -n docker network ls --format '{{.Name}}' | tr '\n' ' '; echo
sudo -n docker volume ls --format '{{.Name}}' | grep -v -E '^[0-9a-f]{64}$' | tr '\n' ' '; echo
sudo -n docker images --format '{{.Repository}}:{{.Tag}} {{.ID}}' | grep -E 'lifetrac|mosquitto'
echo "## camera service"; echo "lifetrac-camera.service=$(systemctl is-active lifetrac-camera.service 2>&1)"
echo "## uart holders"; sudo -n sh -c 'if command -v fuser >/dev/null; then fuser /dev/ttymxc3; echo; echo __UART_CHK_DONE__; else echo __UART_NO_FUSER__; fi' 2>&1
echo "## failed units"; systemctl --failed --no-legend --no-pager
