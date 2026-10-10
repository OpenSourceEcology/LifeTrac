#!/usr/bin/env bash
# leg_prep.sh -- prep for ONE radio leg, per BENCH_RUNBOOK "Prep" and
# RS13_VECTOR_LEG.md step 2: no radio-UART holders, scene check, health probes on
# both boards, clear_retained on the base broker, pre-brackets on both boards,
# then the base-side vector_dry_run capture started DETACHED on the base (it
# outlives this script and covers the harness's 300 s leg with --duration 480).
# Ends by printing (and saving) the harness command for this leg.
#
# Usage:   bash leg_prep.sh <leg> <image_bw500|image_bw250> [options]
#   <leg>          tag used in every file name, e.g. 2a, 2b_yt, 2c_r5 (letters,
#                  digits, '_', '.', '-' only)
#   image_bw500    DTS profile 2 (refused unless DTS_CARRIER_HZ is set, see below)
#   image_bw250    FHSS profile 1
# Options:
#   --boot-mode vector|mono_g4   camera mode at boot (default: mono_g4 for legs
#                  named 2c* / 2d*, vector otherwise)
#   --fps N        camera rate for the harness -SynthFps (default 2)
#   --strict | --no-strict   base capture --strict (default: strict, except 2c*/2d*)
#   --min-frames N base capture --min-frames (default 500 at 2 fps; 250 at 1 fps,
#                  200 for an FHSS leg at 1 fps -- RS13_VECTOR_LEG.md 2b)
#   --no-scene-check   skip the scene check (only for a leg without the camera)
#   --print-harness    only write and print the harness command (after the DTS
#                  gate); touches no board, starts no capture -- a preview
# Inputs:  lib/bench_env.sh settings; DTS_CARRIER_HZ / DTS_CARRIER_DATE for profile 2;
#          PC_HOST (the harness -HostIp; when empty, this PC's only IPv4 address on
#          BASE_HOST's /24, or its only IPv4 address at all, is used and printed --
#          refused when ambiguous); the boards staged by stage_boards.sh.
# Writes:  $EVIDENCE_DIR/leg<leg>_{prep,scene,health_base,health_tractor,clear_retained,
#          pre_base,pre_tractor}.txt (+ _scene.jpg); $BENCH_SCRATCH/leg<leg>_harness.ps1;
#          base: /tmp/lifetrac_strict/legs/leg<leg>_base.{jsonl,json} (capture, running).
# Boards:  read-only checks; rs116/rs115 probes and clear_retained in --rm
#          containers; starts the detached capture container rs13_cap_base on the base.
# Radio:   the probes open /dev/ttymxc3 and the HostLink connect wakes the L072
#          into RXCONT (receive-only). NOTHING here transmits -- the harness you
#          launch next does. Run it only on the operator's explicit GO for this leg.
# Origin:  bench-evidence/RS_13_vector_scene_2026-09-26/scripts/leg_prep.sh
#          (historical copy, unchanged). New here: DTS carrier gate, UART-holder
#          abort, defaults by leg name, the printed harness command.
set -u
. "$(dirname "${BASH_SOURCE[0]}")/../lib/bench_env.sh" || exit 1

LEG=""; PROFILE=""; MODE=""; FPS=2; STRICT=""; MINF=""; SCENE=1; PRINT_ONLY=0
while [ $# -gt 0 ]; do
  case $1 in
    --print-harness) PRINT_ONLY=1 ;;
    --boot-mode) MODE=${2:?--boot-mode vector|mono_g4}; shift ;;
    --fps) FPS=${2:?--fps N}; shift ;;
    --strict) STRICT="--strict" ;;
    --no-strict) STRICT="none" ;;
    --min-frames) MINF=${2:?--min-frames N}; shift ;;
    --no-scene-check) SCENE=0 ;;
    -h|--help) bench_usage; exit 0 ;;
    -*) die "unknown option '$1'" ;;
    *) if [ -z "$LEG" ]; then LEG=$1; elif [ -z "$PROFILE" ]; then PROFILE=$1; else die "extra argument '$1'"; fi ;;
  esac
  shift
done
[ -n "$LEG" ] && [ -n "$PROFILE" ] || { bench_usage; exit 2; }
bench_check_name "leg tag" "$LEG"
case $PROFILE in
  image_bw500) REGP=2 ;;
  image_bw250) REGP=1 ;;
  *) die "profile must be image_bw500 (DTS, profile 2) or image_bw250 (FHSS, profile 1), not '$PROFILE'" ;;
esac
case $LEG in 2c*|2d*) def_mode=mono_g4; def_strict=none ;; *) def_mode=vector; def_strict=--strict ;; esac
MODE=${MODE:-$def_mode}; STRICT=${STRICT:-$def_strict}; [ "$STRICT" = none ] && STRICT=""
case $MODE in
  vector) CAM_ENV="-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80" ;;
  mono_g4) CAM_ENV="-e LIFETRAC_ENCODE_MODE=6" ;;
  *) die "--boot-mode must be vector or mono_g4" ;;
esac
case $FPS in ''|*[!0-9.]*) die "--fps must be a number" ;; esac
if [ -z "$MINF" ]; then
  if [ "$FPS" = 1 ]; then [ $REGP = 1 ] && MINF=200 || MINF=250; else MINF=500; fi
fi

# DTS gate: never fly profile 2 on an unchecked carrier (RS-13.1 A19)
if [ $REGP = 2 ]; then bench_require_dts_carrier || exit 5; fi
[ "$BASE_TRANSPORT" = ssh ] && echo "NOTE: BASE_TRANSPORT=ssh -- run_live_radio_monitor.ps1 drives the base over adb ($BASE_SERIAL); it must be on USB too."

# --- the PC's address for the harness (-HostIp) -----------------------------------
# Without -HostIp the harness falls back to the original bench PC's lease, and the
# base rx daemon's control plane is dead on any other PC. Never omit it.
pc_ipv4s() {   # this PC's IPv4 addresses, minus loopback and link-local
  if command -v powershell.exe > /dev/null 2>&1; then
    powershell.exe -NoProfile -NonInteractive -Command \
      "Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | ForEach-Object { \$_.IPAddress }" 2>/dev/null
  elif command -v ip > /dev/null 2>&1; then
    ip -4 -o addr show scope global 2>/dev/null | awk '{ sub(/\/.*/, "", $4); print $4 }'
  fi | tr -d '\r' | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' | grep -vE '^(127\.|169\.254\.)' | sort -u
}
if [ -z "$PC_HOST" ]; then
  cands=$(pc_ipv4s)
  pick=""
  same=""
  case $BASE_HOST in
    *[!0-9.]*|'') ;;
    *.*.*.*) same=$(printf '%s\n' "$cands" | awk -v p="${BASE_HOST%.*}." 'NF && index($0, p) == 1') ;;
  esac
  n_same=$(printf '%s\n' "$same" | grep -c .)
  n_all=$(printf '%s\n' "$cands" | grep -c .)
  if [ "$n_same" = 1 ]; then pick=$same; why="the only one on BASE_HOST $BASE_HOST's /24"
  elif [ "$n_same" = 0 ] && [ "$n_all" = 1 ]; then pick=$cands; why="this PC's only IPv4 address"
  fi
  if [ -z "$pick" ]; then
    echo "REFUSED: PC_HOST is empty and this PC's LAN address is ambiguous (IPv4 candidates: $(printf '%s ' $cands | sed 's/ $//'))." >&2
    echo "         Set PC_HOST in $BT_DIR/bench.env to the address the boards reach this PC on." >&2
    exit 6
  fi
  PC_HOST=$pick
  echo "NOTE: PC_HOST is empty -- using $PC_HOST ($why) as the harness -HostIp; set PC_HOST in bench.env to pin it."
fi
case $PC_HOST in
  *[!A-Za-z0-9.:-]*) die "PC_HOST '$PC_HOST' is not a plain address or host name" ;;
esac

# --- the harness command for this leg ---------------------------------------------
FRF=""; [ $REGP = 2 ] && FRF=" -ForceFrfHz $DTS_CARRIER_HZ"
HIP=" -HostIp $PC_HOST"
LOGW=$(win_path "$BENCH_SCRATCH/leg${LEG}_harness.txt")
ARCW=$(win_path "$BENCH_SCRATCH/leg${LEG}_archive.txt")
PS1="$BENCH_SCRATCH/leg${LEG}_harness.ps1"
HARNESS=".\\run_live_radio_monitor.ps1 -TxAdbSerial $TRACTOR_SERIAL -RxAdbSerial $BASE_SERIAL$HIP -TxFeed camera -RegProfile $REGP$FRF -DurationS 300 -SynthFps $FPS -KfRequestDisable 1 -ProbeEcho 0 -LogFragArrivals 1 -TxBatch 0 -CamExtraEnv \"$CAM_ENV\" -Archive"
write_harness() {
  # Tee-Object (Windows PowerShell 5.1) writes UTF-16; leg_post.sh converts the
  # transcript. The archive the harness creates is written to leg<leg>_archive.txt,
  # where leg_post.sh looks first.
  rm -f "$BENCH_SCRATCH/leg${LEG}_archive.txt" "$BENCH_SCRATCH/leg${LEG}_harness.txt"   # no stale pointer for leg_post
  cat > "$PS1" <<EOF
# leg $LEG harness, written by leg_prep.sh $(date -u +%Y-%m-%dT%H:%M:%SZ). Run it in PowerShell, never from bash.
Set-Location "$(win_path "$HELPER_DIR")"
\$t0 = Get-Date
& { "# leg $LEG harness launched \$((Get-Date).ToUniversalTime().ToString('yyyy-MM-dd HH:mm:ss'))Z"
    $HARNESS } *>&1 | Tee-Object -FilePath "$LOGW"
\$a = Get-ChildItem -Directory "$(win_path "$DC/bench-evidence")" -Filter "radio_monitor_*" | Where-Object { \$_.CreationTime -ge \$t0 } | Sort-Object CreationTime | Select-Object -Last 1
if (\$a) { \$a.FullName | Out-File -Encoding ascii "$ARCW"; "leg $LEG archive: \$(\$a.FullName)" } else { "leg $LEG - no new radio_monitor_* archive (harness aborted?)" }
EOF
}
print_next() {
  echo "   & \"$(win_path "$PS1")\""
  echo "   which runs, from $(win_path "$HELPER_DIR"):"
  echo "   $HARNESS"
  case $LEG in 2d*) echo "   then, right after the launch, in Git Bash: bash \"$(win_path "$LEGS_DIR/leg2d_switch.sh")\" $LEG";; esac
  echo "   while frames flow:            bash \"$(win_path "$LEGS_DIR/link_sample.sh")\" $LEG"
  echo "   after '[EVIDENCE] archived to':  bash \"$(win_path "$LEGS_DIR/leg_post.sh")\" $LEG"
  if [ "${1:-}" = probed ]; then
    echo "   NOT flying after all (no GO, harness refused, round aborted)? The probes left both L072s"
    echo "   listening (RXCONT) and rs13_cap_base is running. Stop the capture and park both radios:"
    echo "                                 bash \"$(win_path "$LEGS_DIR/power_up_guard.sh")\" --stop-leg-daemons"
  fi
}
if [ $PRINT_ONLY = 1 ]; then
  write_harness
  echo "harness wrapper for leg $LEG written (preview only; no board was touched, no capture is running):"
  print_next
  exit 0
fi

E=$(bench_evidence_dir) || die "cannot create $EVIDENCE_DIR"
exec > >(tee -a "$E/leg${LEG}_prep.txt") 2>&1
hdr() { echo "# leg $LEG $1, $(date -u +%Y-%m-%dT%H:%M:%SZ) PC clock"; }

stamp "== leg $LEG prep (profile $PROFILE = harness -RegProfile $REGP, boot $MODE, $FPS fps, capture min-frames $MINF ${STRICT:-no-strict}) from $(bench_git_desc)"
for who in tractor base; do board_present "$who" || { stamp "ABORT: $who ($(board_serial $who)) not reachable via $(board_via $who)"; exit 3; }; done

stamp "-- production camera unit / radio-UART holders"
cam=$(board_sh tractor "systemctl is-active lifetrac-camera.service")
tc=$(board_containers tractor); bc=$(board_containers base)
th=$(board_uart_holders tractor); bh=$(board_uart_holders base)
echo "tractor: lifetrac-camera=$cam containers=[$tc] ttymxc3 holders=[$th]"
echo "base:    containers=[$bc] ttymxc3 holders=[$bh]"
if [ "$cam" = active ] || echo " $tc " | grep -q ' tractor-camera ' || [ -n "$th" ] || [ -n "$bh" ]; then
  stamp "ABORT: the radio UART is not free. Tractor: systemctl stop lifetrac-camera.service; docker stop tractor-camera"
  stamp "       (or bash power_up_guard.sh); stop any leftover tx_smoke / rx_smoke / camera_svc container; then re-run."
  exit 3
fi
board_sh base "$SUDO docker rm -f rs13_cap_base >/dev/null 2>&1" > /dev/null 2>&1

if [ $SCENE = 1 ]; then
  stamp "-- scene: bench video window to the front + camera check (camera only)"
  bash "$LEGS_DIR/scene_check.sh" "leg${LEG}" || { stamp "ABORT: scene check failed (see SCENE-CHECK line above) - fix the screen/aim before this leg"; exit 4; }
fi

stamp "-- health probes (rs116) both boards"
{ hdr "health base"; board_sh base "$SUDO $PROBE_RUN $BASE_IMAGE -u /work/rs116_health_probe.py 2>&1"; } > "$E/leg${LEG}_health_base.txt"
grep -E "STATS-OK|STATS-FAILED|radio_state|RS12-URC" "$E/leg${LEG}_health_base.txt" | head -4
{ hdr "health tractor"; board_sh tractor "$SUDO $PROBE_RUN $TRACTOR_PROBE_IMAGE -u /work/rs116_health_probe.py 2>&1"; } > "$E/leg${LEG}_health_tractor.txt"
grep -E "STATS-OK|STATS-FAILED|radio_state|RS12-URC" "$E/leg${LEG}_health_tractor.txt" | head -4

stamp "-- clear retained control topics on the base broker"
{ hdr "clear_retained base broker"; board_sh base "$SUDO $WORK_RUN $BASE_IMAGE -u /work/clear_retained.py 2>&1"; } | tee "$E/leg${LEG}_clear_retained.txt" | tail -1

stamp "-- pre-brackets (rs115) both boards"
{ hdr "pre bracket base"; board_sh base "$SUDO $PROBE_RUN $BASE_IMAGE -u /work/rs115_stats_probe.py 2>&1"; } > "$E/leg${LEG}_pre_base.txt"
{ hdr "pre bracket tractor"; board_sh tractor "$SUDO $PROBE_RUN $TRACTOR_PROBE_IMAGE -u /work/rs115_stats_probe.py 2>&1"; } > "$E/leg${LEG}_pre_tractor.txt"
grep -h -E "^radio_(rx_ok|tx_ok|crc_err)=" "$E/leg${LEG}_pre_base.txt" | tr '\n' ' '; echo " (base)"
grep -h -E "^radio_(rx_ok|tx_ok|crc_err)=" "$E/leg${LEG}_pre_tractor.txt" | tr '\n' ' '; echo " (tractor)"

stamp "-- base-side capture, detached (docker rs13_cap_base): profile $PROFILE, 480 s, min-frames $MINF ${STRICT:-no-strict}"
board_sh base "$SUDO rm -f $BOARD_STAGE/legs/leg${LEG}_base.jsonl $BOARD_STAGE/legs/leg${LEG}_base.json; \
  $SUDO docker run -d --name rs13_cap_base --network=host -v $BOARD_STAGE:/work -w /work \
  -e PYTHONPATH=/work:/work/paho -e PYTHONUNBUFFERED=1 --entrypoint python3 $BASE_IMAGE \
  /work/vector_dry_run.py capture --topic lifetrac/v25/video/tile_delta --profile $PROFILE \
  --duration 480 --min-frames $MINF --out /work/legs/leg${LEG}_base.jsonl --json /work/legs/leg${LEG}_base.json $STRICT" \
  | cut -c1-12 | sed 's/^/rs13_cap_base id /'
sleep 3
board_sh base "$SUDO docker logs rs13_cap_base 2>&1 | head -3"

write_harness
stamp "== leg $LEG prep done -> launch the harness NOW (the capture is already listening), in PowerShell:"
print_next probed
