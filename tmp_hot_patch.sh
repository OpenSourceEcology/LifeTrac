#!/bin/sh
# DO NOT RUN on the bench without an explicit radio GO: copies /tmp/web_ui.py
# into lifetrac-vtest-web_ui-1 and restarts it. web_ui publishes a retained
# control/radio_profile at startup, which a running image_rx sends on air.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units".
set -eu
docker cp /tmp/web_ui.py lifetrac-vtest-web_ui-1:/app/base_station/web_ui.py
docker restart lifetrac-vtest-web_ui-1
sleep 6
docker logs --tail 10 lifetrac-vtest-web_ui-1 2>&1 | tail -10
