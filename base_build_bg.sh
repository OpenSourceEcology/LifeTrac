#!/bin/sh
# DO NOT RUN on a bench base: builds on the board in the background
# (`docker compose build`). A build that succeeds moves the tag
# lifetrac-v25:latest off the flown image, and no :previous tag is kept.
# Starts no container. Build with bench_tools/deploy_base.sh instead.
# May-2026 one-off; see BENCH_BOARDS.md "Retired boot units", rule 8.
cd /opt/lifetrac/DESIGN-CONTROLLER || exit 1
# Run build manually with no time limit, log to file
nohup docker compose build > /tmp/base_build.log 2>&1 &
echo "build pid=$!"
