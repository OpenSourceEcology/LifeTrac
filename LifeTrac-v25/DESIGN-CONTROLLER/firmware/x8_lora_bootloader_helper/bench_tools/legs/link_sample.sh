#!/usr/bin/env bash
# link_sample.sh -- DURING a leg (P7): wait for the first base link_stats message
# that has seen frames, keep it, then show the first frame line of the base
# capture. rx_codec_name must read "vector" on a VECTOR leg.
#
# Usage:   bash link_sample.sh <leg>        (start it once the harness is running)
# Inputs:  BASE_SERIAL / BASE_TRANSPORT, BASE_BROKER_CONTAINER.
# Writes:  $EVIDENCE_DIR/leg<leg>_link_stats.txt
# Boards:  base only, read-only: `mosquitto_sub -C 1` inside the base broker
#          container (up to 40 tries, 40 s each) and `docker logs rs13_cap_base`.
# Radio:   none; never transmits.
# Origin:  bench-evidence/RS_13_vector_scene_2026-09-26/scripts/link_sample.sh
#          (historical copy, unchanged).
set -u
. "$(dirname "${BASH_SOURCE[0]}")/../lib/bench_env.sh" || exit 1

LEG=${1:-}
case $LEG in -h|--help|'') bench_usage; exit 0 ;; esac
E=$(bench_evidence_dir) || die "cannot create $EVIDENCE_DIR"

{ echo "# leg $LEG link_stats (base broker): first message with frames; sampler started $(date -u +%Y-%m-%dT%H:%M:%SZ) PC clock"
  for i in $(seq 1 40); do
    m=$(board_sh base "$SUDO docker exec $BASE_BROKER_CONTAINER mosquitto_sub -h 127.0.0.1 -C 1 -W 40 -t lifetrac/v25/video/link_stats")
    if printf '%s\n' "$m" | grep -q '"rx_frames_seen": [1-9]'; then echo "$(date -u +%H:%M:%SZ) $m"; break; fi
    sleep 4
  done; } | tee "$E/leg${LEG}_link_stats.txt" | tail -1 | cut -c1-420
echo "rx_codec_name: $(grep -oE '"rx_codec_name": "[^"]*"' "$E/leg${LEG}_link_stats.txt" | tail -1)"
echo "=== first frame at the base"
board_sh base "$SUDO docker logs rs13_cap_base 2>&1 | grep '^#' | head -1" | cut -c1-140
