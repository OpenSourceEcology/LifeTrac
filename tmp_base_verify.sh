#!/bin/sh
set -eu
sleep 15
echo '=== image_rx tail ==='
docker logs --tail 25 lifetrac-vtest-image_rx-1 2>&1 | tail -25
echo
echo '=== web_ui tail ==='
docker logs --tail 15 lifetrac-vtest-web_ui-1 2>&1 | tail -15
echo
echo '=== web_ui port ==='
ss -tlnp 2>/dev/null | grep -E ':8080|:8443' || netstat -tln 2>/dev/null | grep -E ':8080|:8443' || true
echo
echo '=== curl login page ==='
curl -sk -o /dev/null -w 'HTTP=%{http_code}\n' https://127.0.0.1:8443/ || true
curl -s  -o /dev/null -w 'HTTP=%{http_code}\n' http://127.0.0.1:8080/  || true
