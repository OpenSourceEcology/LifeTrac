#!/usr/bin/env bash
# stage_boards.sh -- (re)stage the bench tooling on the boards: everything a leg,
# a step-1 pass or a radio-state check needs in /tmp/lifetrac_strict, the
# harness's openocd reset cfg in /tmp/lifetrac_p0c, then this checkout's code
# tree (push_fix_to_board.sh). Run it at the start of every session, after any
# reboot or flash, and whenever the last push is 5 days old: /tmp is tmpfs and
# systemd-tmpfiles ages it at 5 days (BENCH_RUNBOOK, Board facts).
#
# Usage:   bash stage_boards.sh [--probes-only] [base|tractor ...]     (default: both)
#   --probes-only   only the radio probe tools + paho (what power_up_guard.sh
#                   stages); no code tree, no injectors
# Inputs:  lib/bench_env.sh settings (serials, BASE_TRANSPORT, ...); the files of
#          THIS checkout (repo-root _paho_pull/paho, the helper probes, bench_tools,
#          base_station, tools/, firmware/tractor_x8).
# Writes:  board: /tmp/lifetrac_strict/{probes,bench tools,paho,code tree,legs/},
#          /tmp/lifetrac_p0c/08_boot_user_app.cfg (tractor; LF-clean);
#          PC: md5 record appended to $EVIDENCE_DIR/staging_<role>.txt.
# Boards:  copies files and creates the two staging folders (mode 0777); runs an
#          import smoke in a container WITHOUT any device (push_fix_to_board.sh).
# Radio:   nothing here opens /dev/ttymxc3 or talks to the L072. Never transmits.
# Origin:  bench-evidence/RS_13_vector_scene_2026-09-26/scripts/stage_boards.sh
#          (historical copy, unchanged). New here: env-driven paths/serials, the
#          LF-clean 08_boot_user_app.cfg, md5 verification of every pushed file,
#          ssh support for the base.
set -u
. "$(dirname "${BASH_SOURCE[0]}")/../lib/bench_env.sh" || exit 1

PROBES_ONLY=0; WHO=()
for a in "$@"; do
  case $a in
    --probes-only) PROBES_ONLY=1 ;;
    base|tractor) WHO+=("$a") ;;
    -h|--help) bench_usage; exit 0 ;;
    *) die "unknown argument '$a' (base|tractor|--probes-only)" ;;
  esac
done
[ ${#WHO[@]} -gt 0 ] || WHO=(tractor base)

H="$HELPER_DIR"
PROBES=("$H/method_g_stage1_probe.py" "$H/method_h_stage2_tx_probe_v2.py" "$H/rs115_stats_probe.py"
        "$H/rs116_health_probe.py" "$BT_DIR/radio_park.py" "$BT_DIR/radio_state.py")
TOOLS=("$BT_DIR/clear_retained.py" "$BT_DIR/clear_retained_host.py" "$BT_DIR/frag_gap_report.py"
       "$BT_DIR/kf_inject.py" "$BT_DIR/dual_inject.py" "$DC/base_station/image_rx_daemon.py"
       "$DC/base_station/cmd_timing.py")
PAHO="$REPO_ROOT/_paho_pull/paho"
[ -d "$PAHO" ] || die "missing $PAHO (repo-root _paho_pull/paho, tracked in git)"

E=$(bench_evidence_dir) || die "cannot create $EVIDENCE_DIR"
rc_all=0
for who in "${WHO[@]}"; do
  ser=$(board_serial "$who"); via=$(board_via "$who")
  stamp "=== $who ($ser via $via)"
  board_present "$who" || { stamp "SKIP $who: not reachable via $via"; rc_all=1; continue; }
  REC="$E/staging_$who.txt"
  { echo "# staging $who ($ser), $(date -u +%Y-%m-%dT%H:%M:%SZ) PC clock, from $(bench_git_desc)$([ $PROBES_ONLY = 1 ] && echo ', probes only')"; } >> "$REC"

  board_sh "$who" "$SUDO mkdir -p $BOARD_STAGE/legs $BOARD_FLASH_STAGE; $SUDO chmod 0777 $BOARD_STAGE $BOARD_STAGE/legs $BOARD_FLASH_STAGE" > /dev/null

  pairs=()                               # "local|remote" for the md5 check
  list=("${PROBES[@]}"); [ $PROBES_ONLY = 1 ] || list+=("${TOOLS[@]}")
  for f in "${list[@]}"; do
    board_push "$who" "$f" "$BOARD_STAGE/" || { stamp "PUSH FAILED $f"; exit 1; }
    pairs+=("$f|$BOARD_STAGE/$(basename "$f")")
  done
  if [ $PROBES_ONLY = 0 ]; then
    board_push_lf "$who" "$H/bench_mqtt.conf" "$BOARD_STAGE/bench_mqtt.conf" || { stamp "PUSH FAILED bench_mqtt.conf"; exit 1; }
    pairs+=("$BENCH_SCRATCH/lf/bench_mqtt.conf|$BOARD_STAGE/bench_mqtt.conf")
  fi
  if [ "$(board_role "$who")" = tractor ]; then
    # run_live_radio_monitor.ps1 resets the tractor L072 with
    # openocd -f /tmp/lifetrac_p0c/08_boot_user_app.cfg (its step 4/5)
    board_push_lf "$who" "$H/08_boot_user_app.cfg" "$BOARD_FLASH_STAGE/08_boot_user_app.cfg" || { stamp "PUSH FAILED 08_boot_user_app.cfg"; exit 1; }
    pairs+=("$BENCH_SCRATCH/lf/08_boot_user_app.cfg|$BOARD_FLASH_STAGE/08_boot_user_app.cfg")
  fi
  # root-owned __pycache__ (python in the containers runs as root) blocks an overwrite
  board_sh "$who" "$SUDO rm -rf $BOARD_STAGE/paho" > /dev/null
  board_push "$who" "$PAHO" "$BOARD_STAGE/" || { stamp "PUSH FAILED paho"; exit 1; }

  # md5: every single file pushed, against what was sent
  remote=""; for p in "${pairs[@]}"; do remote="$remote ${p#*|}"; done
  sums=$(board_sh "$who" "md5sum $remote")
  bad=0
  for p in "${pairs[@]}"; do
    l=${p%%|*}; r=${p#*|}
    lm=$(md5sum < "$l" | cut -c1-32)
    rm_=$(printf '%s\n' "$sums" | awk -v f="$r" '$2==f{print $1}')
    if [ "$lm" = "$rm_" ]; then st=ok; else st=MISMATCH; bad=1; fi
    printf '%s  %-8s %s\n' "$lm" "$st" "$r" >> "$REC"
  done
  [ $bad = 0 ] && stamp "$who: ${#pairs[@]} files md5-verified (record: $REC)" || { stamp "$who: md5 MISMATCH -- see $REC"; rc_all=1; }

  if [ $PROBES_ONLY = 0 ]; then
    stamp "$who: code tree"
    bash "$LEGS_DIR/push_fix_to_board.sh" "$who" 2>&1 | grep -E "== push|MISMATCH|verified|encoder from|has _tick|SKIP|FAILED" || true
  fi
  n=$(board_sh "$who" "ls $BOARD_STAGE | wc -l" | tr -d ' ')
  stamp "$who: $BOARD_STAGE holds $n entries: $(board_sh "$who" "ls $BOARD_STAGE" | tr '\n' ' ')"
  if [ $PROBES_ONLY = 0 ] && [ "${n:-0}" -lt 19 ]; then stamp "$who: WARNING fewer than 19 entries (BENCH_RUNBOOK expects >= 19)"; rc_all=1; fi
done
exit $rc_all
