#!/bin/sh
set -eu
docker cp /tmp/web_ui.py lifetrac-vtest-web_ui-1:/app/base_station/web_ui.py
docker restart lifetrac-vtest-web_ui-1
sleep 6
docker logs --tail 10 lifetrac-vtest-web_ui-1 2>&1 | tail -10
