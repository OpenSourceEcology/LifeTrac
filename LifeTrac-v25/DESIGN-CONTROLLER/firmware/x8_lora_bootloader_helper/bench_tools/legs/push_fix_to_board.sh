#!/usr/bin/env bash
# push_fix_to_board.sh -- push THIS checkout's image-path code tree to
# /tmp/lifetrac_strict on one board, so a step-1 pass (step1_pass.sh --from-work)
# and the harness legs run the checkout's camera_service.py / encoder / store
# without an image rebuild. stage_boards.sh calls it for both boards.
#
# Usage:   bash push_fix_to_board.sh [tractor|base|<adb serial>] [--no-smoke]   (default: tractor)
# Inputs:  this checkout: firmware/tractor_x8/{camera_service.py,image_tx_daemon.py,
#          x8_image_pipeline/}, base_station/{lora_proto.py,image_pipeline/},
#          tools/vector_dry_run.py. The name keeps the RS-13.1 one ("the fix
#          branch"); it pushes whatever branch is checked out.
# Writes:  board: those files under /tmp/lifetrac_strict (old copies of the two
#          package dirs removed first, __pycache__ cleared);
#          PC: md5 of every pushed file appended to $EVIDENCE_DIR/staging_<role>.txt.
# Boards:  file copies; then (unless --no-smoke) one `docker run --rm` of
#          $TRACTOR_APP_IMAGE with /tmp/lifetrac_strict mounted and NO device, to
#          prove the encoder imports from /work. Skipped if the image is absent.
# Radio:   never opens /dev/ttymxc3; never transmits.
# Origin:  bench-evidence/RS_13_vector_scene_2026-09-26/scripts/push_fix_to_board.sh
#          (historical copy, unchanged; it took BOARD=<serial> and pushed from a
#          fixed checkout path).
set -u
. "$(dirname "${BASH_SOURCE[0]}")/../lib/bench_env.sh" || exit 1

WHO=${BOARD:-tractor}; SMOKE=1
for a in "$@"; do
  case $a in
    --no-smoke) SMOKE=0 ;;
    -h|--help) bench_usage; exit 0 ;;
    -*) die "unknown option '$a'" ;;
    *) WHO=$a ;;
  esac
done
ROLE=$(board_role "$WHO")
board_present "$WHO" || die "$WHO ($(board_serial "$WHO")) is not reachable via $(board_via "$WHO")"
E=$(bench_evidence_dir) || die "cannot create $EVIDENCE_DIR"
REC="$E/staging_$ROLE.txt"

stamp "== push code tree to $WHO $BOARD_STAGE ($(bench_git_desc))"
board_sh "$WHO" "$SUDO mkdir -p $BOARD_STAGE/legs; $SUDO chmod 0777 $BOARD_STAGE $BOARD_STAGE/legs; $SUDO rm -rf $BOARD_STAGE/x8_image_pipeline $BOARD_STAGE/image_pipeline" > /dev/null

FILES=("$DC/firmware/tractor_x8/camera_service.py" "$DC/firmware/tractor_x8/image_tx_daemon.py"
       "$DC/base_station/lora_proto.py" "$DC/tools/vector_dry_run.py")
DIRS=("$DC/base_station/image_pipeline" "$DC/firmware/tractor_x8/x8_image_pipeline")
for f in "${FILES[@]}" "${DIRS[@]}"; do
  board_push "$WHO" "$f" "$BOARD_STAGE/" || { stamp "PUSH FAILED $f"; exit 1; }
done
board_sh "$WHO" "$SUDO find $BOARD_STAGE -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null; true" > /dev/null

# md5 of every pushed file (relative to /tmp/lifetrac_strict), local vs board
declare -A LOCAL=()
for f in "${FILES[@]}"; do LOCAL[$(basename "$f")]=$(md5sum < "$f" | cut -c1-32); done
for d in "${DIRS[@]}"; do
  parent=$(dirname "$d")
  while IFS= read -r f; do
    LOCAL[${f#"$parent"/}]=$(md5sum < "$f" | cut -c1-32)
  done < <(find "$d" -type f ! -path '*__pycache__*' ! -name '*.pyc')
done
REMOTE=$(board_sh "$WHO" "cd $BOARD_STAGE && md5sum ${!LOCAL[*]} 2>&1")
{ echo "# code tree -> $ROLE $BOARD_STAGE, $(date -u +%Y-%m-%dT%H:%M:%SZ) PC clock, from $(bench_git_desc)"; } >> "$REC"
bad=0
for k in $(printf '%s\n' "${!LOCAL[@]}" | sort); do
  r=$(printf '%s\n' "$REMOTE" | awk -v f="$k" '$2==f{print $1}')
  if [ "${LOCAL[$k]}" = "$r" ]; then st=ok; else st=MISMATCH; bad=1; echo "MISMATCH $k (local ${LOCAL[$k]}, board ${r:-missing})"; fi
  printf '%s  %-8s %s\n' "${LOCAL[$k]}" "$st" "$k" >> "$REC"
done
[ $bad = 0 ] && stamp "== ${#LOCAL[@]} files md5-verified on $ROLE (record: $REC)" || stamp "== md5 MISMATCH on $ROLE -- see $REC"

if [ $SMOKE = 1 ]; then
  if board_has_image "$WHO" "$TRACTOR_APP_IMAGE"; then
    stamp "== import smoke inside $TRACTOR_APP_IMAGE, from /work (the checkout's encoder must load with its codec mirror)"
    board_sh "$WHO" "$SUDO $WORK_RUN $TRACTOR_APP_IMAGE -c \"import x8_image_pipeline.encode_vector as ev; print('encoder from', ev.__file__); print('codec', ev.vs.__file__, 'TTL_FRAMES', ev.vs.TTL_FRAMES); print('has _tick_ttl', hasattr(ev.VectorEncoder, '_tick_ttl'))\"" 2>&1 \
      | tee -a "$REC"
  else
    stamp "== SKIP import smoke: $TRACTOR_APP_IMAGE is not on $ROLE (build it: DEPLOY.md / build_tractor_image.sh)"
  fi
fi
exit $bad
