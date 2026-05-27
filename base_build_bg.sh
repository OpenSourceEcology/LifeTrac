#!/bin/sh
cd /opt/lifetrac/DESIGN-CONTROLLER || exit 1
# Run build manually with no time limit, log to file
nohup docker compose build > /tmp/base_build.log 2>&1 &
echo "build pid=$!"
