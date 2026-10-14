#!/bin/sh
# DO NOT RUN on a bench board, not even with a radio GO: removes and re-creates
# the lifetrac-vtest image_rx container, which opens /dev/ttymxc3 with a
# RESET_REQ that un-parks the L072 (REG_PROFILE 0, fixed 915 MHz).
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units", rule 7.
echo fio | sudo -S docker inspect lifetrac-vtest-image_rx-1 --format '{{.State.StartedAt}}'
echo ---env---
echo fio | sudo -S docker inspect lifetrac-vtest-image_rx-1 --format '{{range .Config.Env}}{{println .}}{{end}}' | grep LIFETRAC
echo ---force restart---
cd /opt/lifetrac/DESIGN-CONTROLLER
echo fio | sudo -S docker compose -f docker-compose.video-test.yml -p lifetrac-vtest rm -sf image_rx
echo fio | sudo -S docker compose -f docker-compose.video-test.yml -p lifetrac-vtest up -d image_rx
sleep 6
echo ---new logs---
echo fio | sudo -S docker logs --tail 25 lifetrac-vtest-image_rx-1 2>&1
echo ---env after restart---
echo fio | sudo -S docker inspect lifetrac-vtest-image_rx-1 --format '{{range .Config.Env}}{{println .}}{{end}}' | grep LIFETRAC
