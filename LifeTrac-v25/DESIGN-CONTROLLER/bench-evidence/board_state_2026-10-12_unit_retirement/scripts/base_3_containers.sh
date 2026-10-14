set -u
export PATH=$PATH:/usr/sbin:/sbin
C="lifetrac-vtest-mosquitto-1 lifetrac-vtest-image_rx-1 lifetrac-vtest-web_ui-1 lifetrac-vtest-audit_tail-1"
echo "## guard: unit gone or not enabled"
e=$(systemctl is-enabled lifetrac-base-compose.service 2>&1); echo "lifetrac-base-compose.service is-enabled: $e"
case $e in enabled*) echo "ABORT: unit still enabled"; exit 1;; esac
echo "## guard: each container never started"
for c in $C; do s=$(sudo -n docker inspect -f '{{.State.Status}} started={{.State.StartedAt}} project={{index .Config.Labels "com.docker.compose.project"}}' $c 2>&1); echo "$c: $s"; case $s in "created started=0001-01-01T00:00:00Z project=lifetrac-vtest") ;; *) echo "ABORT: unexpected state"; exit 1;; esac; done
echo "## remove (no -f, no -v: volumes kept)"
sudo -n docker rm $C && sudo -n docker network rm lifetrac-vtest_default
echo "## after"
sudo -n docker ps -a --format '{{.Names}}\t{{.Image}}\t{{.Status}}'
sudo -n docker compose ls -a
sudo -n docker network ls --format '{{.Name}}' | tr '\n' ' '; echo
sudo -n docker volume ls --format '{{.Name}}' | grep -E 'vtest|design' | tr '\n' ' '; echo
echo "## listeners"; ss -ltn 2>/dev/null | grep -E ':(1883|8080) ' || echo "none on 1883/8080"
echo "## uart holders"; sudo -n sh -c 'if command -v fuser >/dev/null; then fuser /dev/ttymxc3; echo; echo __UART_CHK_DONE__; else echo __UART_NO_FUSER__; fi' 2>&1
echo "## failed units"; systemctl --failed --no-legend --no-pager
