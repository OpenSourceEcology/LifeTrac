#!/bin/sh
ls /opt/lifetrac/DESIGN-CONTROLLER 2>/dev/null
echo ---
docker inspect lifetrac-vtest-web_ui-1 --format '{{json .Mounts}}' | python3 -m json.tool
echo ---web_ui pids
docker exec lifetrac-vtest-web_ui-1 ps -ef | head -30
echo ---listening
docker exec lifetrac-vtest-web_ui-1 sh -c 'ss -tlnp 2>/dev/null || netstat -tlnp 2>/dev/null'
echo ---fw_dir
docker exec lifetrac-vtest-web_ui-1 sh -c 'find /app -maxdepth 4 -type d 2>/dev/null'
echo ---grep_video
docker exec lifetrac-vtest-web_ui-1 sh -c 'grep -rln "tile stream\|tile_delta\|tileCanvas\|video/tile" / 2>/dev/null | head -20'
