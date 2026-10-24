#!/usr/bin/env bash
# leg_prep.sh -- prep for ONE radio leg, per BENCH_RUNBOOK "Prep" and
# RS13_VECTOR_LEG.md step 2: no radio-UART holders, scene check, health probes on
# both boards, clear_retained on the base broker, pre-brackets on both boards,
# then the base-side vector_dry_run capture started DETACHED on the base (it
# outlives this script and runs for the leg's length plus 180 s: --duration 480
# for a 300 s leg). Ends by printing (and saving) the harness command for this leg.
#
# Usage:   bash leg_prep.sh <leg> [image_bw500|image_bw250] [options]
#   <leg>          tag used in every file name, e.g. 2a, 2b_yt, 2c_r5 (letters,
#                  digits, '_', '.', '-' only)
#   image_bw500    DTS profile 2 (refused unless DTS_CARRIER_HZ is set, see below)
#   image_bw250    FHSS profile 1 (may be left out for a range-edge tag)
# Range-edge tags (RS13_RANGE_EDGE_LEG.md "Leg names"; a _suffix such as _r2 may
# follow): the tag sets the profile, the codec at boot, the probes and, where the
# procedure fixes it, the length. Every line has -TxBatch 0 -ProbeEcho 0, and every
# DTS line -ForceFrfHz (the DTS gate below). Since 2026-10-19 the bench has no
# attenuators: only the A2* legs are planned, on the antennas; the others need the
# conducted path and get a NOTE saying so.
#   leak_dts | leak_fhss  leakage check: VECTOR, 120 s, -ReactiveFire 0, capture
#                         without --strict (it must stay empty)
#   A2c A2c2 | A2a A2a2   Round A on the antennas (DTS): mono_g4 | VECTOR, 300 s,
#                         -ReactiveFire 1 (the conducted 0 dB step until 2026-10-19)
#   B<step>m | B<step>v   Round B, the DTS walk: mono_g4 | VECTOR, -ReactiveFire 1
#   C<step>m | C<step>v   Round C, the FHSS walk: mono_g4 | VECTOR, -ReactiveFire 0
#                         A walk leg needs --duration 120 (coarse) or 180 (fine), the
#                         step plan's choice; C0m / C0v (FHSS's 0 dB pair) are 180 s.
#   B0close | C0close     closing check: VECTOR, 120 s, DTS with probes | FHSS without
#   <step> is the step chain in dB: 0 to 66 in 3 dB steps. A tag that starts like one
#   of these (leak, A2, B or C and a digit, in any letter case) but is none of them
#   is refused, and so is a profile or --boot-mode that contradicts the tag. Other
#   tags keep the RS-13.1 defaults below.
# Options:
#   --boot-mode vector|mono_g4   camera mode at boot (default: from a range-edge tag;
#                  else mono_g4 for legs named 2c* / 2d*, vector otherwise)
#   --duration S   leg length, the harness -DurationS (default: from a range-edge tag,
#                  else 300)
#   --reactive-fire 0|1   the harness -ReactiveFire: 1 = the base fires a no-op PROBE
#                  per received frame, scored tractor-side (default: from a
#                  range-edge tag, else 0, the harness default)
#   --fps N        camera rate for the harness -SynthFps (default 2)
#   --stop-drain S the harness -StopDrainS: seconds between the feed stop and the
#                  tx_smoke stop at leg end, 0 < S <= 30 (default 4 on image_bw250,
#                  FHSS profile 1; 2, the harness default, on image_bw500, DTS profile 2).
#                  Both codecs of a round (its mono_g4 and VECTOR legs) must fly the SAME
#                  value: a train still on the air at the stop counts as received but not
#                  sent, so a short drain costs the codec with the longer queue more.
#   --strict | --no-strict   base capture --strict (default: strict on a VECTOR leg;
#                  not on 2c*/2d*, a mono_g4 range-edge leg or the leakage check)
#   --min-frames N base capture --min-frames (default 500 / 300 / 200 for a 300 / 180 /
#                  120 s leg at 2 fps; for a 300 s leg at 1 fps 250, or 200 on FHSS --
#                  RS13_VECTOR_LEG.md 2b)
#   --no-scene-check   skip the scene check (only for a leg without the camera)
#   --print-harness    only write and print the harness command (after the DTS
#                  gate); touches no board, starts no capture -- a preview
#   On a range-edge tag, --duration, --reactive-fire, --fps and --strict/--no-strict
#   override the procedure's value; the prep then prints a NOTE naming each one.
#   A range-edge tag expects EVIDENCE_DIR under bench-evidence/RS_13_range_edge_<date>/
#   (an A2* tag also under RS_13_antenna_<date>/); elsewhere the prep prints a NOTE.
#   An EVIDENCE_DIR that already holds files tracked by git (a committed evidence
#   record) is REFUSED (exit 7) before any board action, unless BENCH_EVIDENCE_APPEND=1
#   is in the environment (lib/bench_env.sh, "Committed evidence"); --print-harness
#   shows that verdict and still writes the preview. Such a deliberate append needs the
#   override on EVERY script of the leg (leg2d_switch.sh and link_sample.sh refuse the
#   folder without it, mid-leg): the follow-up lines the prep prints then carry it.
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
#          abort, defaults by leg name, the printed harness command, the range-edge
#          tags, --duration and --reactive-fire; 2026-10-22: --stop-drain (with
#          the FHSS 4 s default) and the committed-evidence refusal.
set -u
. "$(dirname "${BASH_SOURCE[0]}")/../lib/bench_env.sh" || exit 1

LEG=""; PROFILE=""; MODE=""; FPS=""; STRICT=""; MINF=""; SCENE=1; PRINT_ONLY=0; DUR=""; FIRE=""; DRAIN=""
while [ $# -gt 0 ]; do
  case $1 in
    --print-harness) PRINT_ONLY=1 ;;
    --boot-mode) MODE=${2:?--boot-mode vector|mono_g4}; shift ;;
    --duration) DUR=${2:?--duration S}; shift ;;
    --reactive-fire) FIRE=${2:?--reactive-fire 0|1}; shift ;;
    --fps) FPS=${2:?--fps N}; shift ;;
    --stop-drain) DRAIN=${2:?--stop-drain S}; shift ;;
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
[ -n "$LEG" ] || { bench_usage; exit 2; }
bench_check_name "leg tag" "$LEG"
case $PROFILE in
  ''|image_bw500|image_bw250) ;;
  *) die "profile must be image_bw500 (DTS, profile 2) or image_bw250 (FHSS, profile 1), not '$PROFILE'" ;;
esac

# --- range-edge legs (RS13_RANGE_EDGE_LEG.md "Leg names" and "Leg lines") --------------
# The tag fixes the profile, the codec at boot and the probes; the length too, except
# on a walk step (coarse 120 s or fine 180 s is the step plan's call).
RE=""; RE_NOTE=""
if [[ $LEG =~ ^(leak_dts|leak_fhss|A2[ac]2?|[BC]0close|[BC](0|[1-9][0-9]?)[vm])(_[A-Za-z0-9_.-]+)?$ ]]; then
  RE=${BASH_REMATCH[1]}
elif [[ ${LEG,,} =~ ^(leak|a2|[bc][0-9]) ]]; then
  die "leg tag '$LEG' looks like a range-edge leg but is none of RS13_RANGE_EDGE_LEG.md's names: leak_dts, leak_fhss, A2c, A2a, A2c2, A2a2, B<step>v|m, C<step>v|m, B0close, C0close (step 0..66 dB in 3 dB steps, no leading zero), each optionally with a _suffix"
fi
if [ -n "$RE" ]; then
  re_dur=""; re_strict=""
  case $RE in
    leak_dts|leak_fhss)
      [ $RE = leak_dts ] && re_prof=image_bw500 || re_prof=image_bw250
      re_mode=vector; re_fire=0; re_dur=120; re_strict=none
      re_what="leakage check (${RE#leak_}): jack pads terminated, the base must decode NO frame (the capture stays empty; its min_frames fails by design)" ;;
    A2c|A2c2) re_prof=image_bw500; re_mode=mono_g4; re_fire=1; re_dur=300; re_what="Round A on the antennas (no attenuators since 2026-10-19), mono_g4 (fly A2c, A2a, A2c2, A2a2 in that order)" ;;
    A2a|A2a2) re_prof=image_bw500; re_mode=vector; re_fire=1; re_dur=300; re_what="Round A on the antennas (no attenuators since 2026-10-19), VECTOR (fly A2c, A2a, A2c2, A2a2 in that order)" ;;
    B0close) re_prof=image_bw500; re_mode=vector; re_fire=1; re_dur=120; re_what="Round B closing check: step chain back at 0 dB, VECTOR" ;;
    C0close) re_prof=image_bw250; re_mode=vector; re_fire=0; re_dur=120; re_what="Round C closing check: step chain back at 0 dB, VECTOR" ;;
    *) step=${RE:1:${#RE}-2}
       { [ $((step % 3)) = 0 ] && [ "$step" -le 66 ]; } || die "leg $LEG: $step dB is not a step of the step set (0 to 66 dB in 3 dB steps)"
       case $RE in *v) re_mode=vector ;; *m) re_mode=mono_g4 ;; esac
       case $RE in
         B*) re_prof=image_bw500; re_fire=1; re_what="Round B, DTS walk, step $step dB, $re_mode" ;;
         C*) re_prof=image_bw250; re_fire=0; re_what="Round C, FHSS walk, step $step dB, $re_mode"
             [ "$step" = 0 ] && re_dur=180 ;;
       esac ;;
  esac
  if [ -z "$re_strict" ]; then [ $re_mode = vector ] && re_strict=--strict || re_strict=none; fi
  PROFILE=${PROFILE:-$re_prof}
  [ "$PROFILE" = "$re_prof" ] || die "leg $LEG flies $re_prof ($([ $re_prof = image_bw500 ] && echo DTS, profile 2 || echo FHSS, profile 1); RS13_RANGE_EDGE_LEG.md 'Leg names'), not $PROFILE"
  [ -z "$MODE" ] || [ "$MODE" = "$re_mode" ] || die "leg $LEG boots $re_mode (its tag names the codec); --boot-mode $MODE contradicts it -- use the tag that names $MODE"
  # an explicit option that differs from the procedure is kept, and named in a NOTE
  if [ -n "$DUR" ] && [ -n "$re_dur" ] && [ "$DUR" != "$re_dur" ]; then RE_NOTE="$RE_NOTE --duration $DUR (procedure: $re_dur);"; fi
  if [ -n "$FIRE" ] && [ "$FIRE" != "$re_fire" ]; then RE_NOTE="$RE_NOTE --reactive-fire $FIRE (procedure: $re_fire);"; fi
  if [ -n "$FPS" ] && [ "$FPS" != 2 ]; then RE_NOTE="$RE_NOTE --fps $FPS (procedure: 2);"; fi
  if [ -n "$STRICT" ] && [ "$STRICT" != "$re_strict" ]; then RE_NOTE="$RE_NOTE capture ${STRICT/none/--no-strict} (procedure: ${re_strict/none/--no-strict});"; fi
  MODE=$re_mode; DUR=${DUR:-$re_dur}; FIRE=${FIRE:-$re_fire}; def_strict=$re_strict
  [ -n "$DUR" ] || die "leg $LEG: a walk leg's length comes from the step plan (RS13_RANGE_EDGE_LEG.md 'Step plan': coarse steps 120 s, fine steps 180 s) -- pass --duration 120 or --duration 180"
else
  [ -n "$PROFILE" ] || { bench_usage; exit 2; }
  case $LEG in 2c*|2d*) def_mode=mono_g4; def_strict=none ;; *) def_mode=vector; def_strict=--strict ;; esac
  MODE=${MODE:-$def_mode}; DUR=${DUR:-300}; FIRE=${FIRE:-0}
fi
case $PROFILE in image_bw500) REGP=2 ;; image_bw250) REGP=1 ;; esac
FPS=${FPS:-2}; STRICT=${STRICT:-$def_strict}; [ "$STRICT" = none ] && STRICT=""
case $MODE in
  vector) CAM_ENV="-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80" ;;
  mono_g4) CAM_ENV="-e LIFETRAC_ENCODE_MODE=6" ;;
  *) die "--boot-mode must be vector or mono_g4" ;;
esac
case $FPS in ''|*[!0-9.]*) die "--fps must be a number" ;; esac
case $DUR in ''|*[!0-9]*|0*) die "--duration must be a whole number of seconds, not '$DUR'" ;; esac
case $FIRE in 0|1) ;; *) die "--reactive-fire must be 0 or 1, not '$FIRE'" ;; esac
# Leg-end drain (2026-10-22). The harness default of 2 s is short for a saturated
# multi-fragment FHSS leg: a train still on the air at the stop is decoded but missing
# from 'sent', the last reactive PROBE lands on a stopped daemon, and bracket_deltas
# marks the FHSS criteria UNVERIFIED -- all of which favour the codec with the shorter
# queue. FHSS legs get 4 s; DTS legs keep 2 s.
case $REGP in 1) def_drain=4; DRAIN_WHY="FHSS default" ;; 2) def_drain=2; DRAIN_WHY="DTS default" ;; esac
if [ -n "$DRAIN" ]; then DRAIN_WHY="--stop-drain; default $def_drain"; else DRAIN=$def_drain; fi
{ [[ $DRAIN =~ ^[0-9]+(\.[0-9]+)?$ ]] && awk -v s="$DRAIN" 'BEGIN { exit !(s > 0 && s <= 30) }'; } \
  || die "--stop-drain must be a number of seconds with 0 < S <= 30, not '$DRAIN'"
if [ -z "$MINF" ]; then
  case $DUR:$FPS in
    300:1) [ $REGP = 1 ] && MINF=200 || MINF=250 ;;
    300:*) MINF=500 ;;
    *) MINF=$(awk -v d="$DUR" -v f="$FPS" 'BEGIN { printf "%d", d * f * 5 / 6 }') ;;   # 300 / 200 for 180 / 120 s at 2 fps
  esac
fi
CAPDUR=$((DUR + 180))                     # the base capture outlives the leg by 180 s
PLAN="profile $PROFILE = harness -RegProfile $REGP, boot $MODE, $FPS fps, $DUR s, -ReactiveFire $FIRE, -StopDrainS $DRAIN ($DRAIN_WHY), capture $CAPDUR s min-frames $MINF ${STRICT:-no-strict}"
plan_notes() {
  local want
  [ -z "$RE" ] || echo "range edge: $re_what"
  [ -z "$RE_NOTE" ] || echo "NOTE: overrides RS13_RANGE_EDGE_LEG.md for $RE:${RE_NOTE%;}"
  [ "$DRAIN" = "$def_drain" ] || echo "NOTE: --stop-drain $DRAIN (default $def_drain): fly the other codec of this round with the same --stop-drain $DRAIN"
  case $RE in
    A2*|'') ;;
    *) echo "NOTE: $RE needs the conducted attenuator path, which the bench does not have since 2026-10-19 (RS13_RANGE_EDGE_LEG.md, Status): only the A2* legs are planned, on the antennas" ;;
  esac
  if [ -n "$RE" ]; then
    case $RE in A2*) want="RS_13_antenna_<date>/legs or RS_13_range_edge_<date>/legs" ;; *) want="RS_13_range_edge_<date>/legs" ;; esac
    case $EVIDENCE_DIR in
      */RS_13_range_edge_*) ;;
      */RS_13_antenna_*) case $RE in A2*) ;; *) echo "NOTE: EVIDENCE_DIR is $EVIDENCE_DIR; a conducted-path leg's evidence goes to bench-evidence/$want (set EVIDENCE_DIR in bench.env)" ;; esac ;;
      *) echo "NOTE: EVIDENCE_DIR is $EVIDENCE_DIR; this leg's evidence goes to bench-evidence/$want (set EVIDENCE_DIR in bench.env)" ;;
    esac
  fi
}

# Committed-evidence guard (2026-10-22), BEFORE the first board action: the probes below
# wake both L072s (RXCONT) and the capture starts on the base, so a refusal must come
# before any of that. Called directly (not in $(...)) so BENCH_EVIDENCE_MSG survives for
# the prep record; a preview (--print-harness) shows the verdict and still writes itself.
EV_OK=1
bench_evidence_guard 2>/dev/null || EV_OK=0
if [ $EV_OK = 0 ] && [ $PRINT_ONLY = 0 ]; then printf '%s\n' "$BENCH_EVIDENCE_MSG" >&2; exit 7; fi
# A deliberate append (BENCH_EVIDENCE_APPEND=1 on a committed folder, 2026-10-22): the
# follow-up scripts run in new shells and would refuse that folder while the harness is
# already transmitting (a 2d leg without its switch, no link stats), so print_next puts
# the same override in front of each of them.
APX=""
if [ $EV_OK = 1 ] && [ "${BENCH_EVIDENCE_APPEND:-}" = 1 ] && [ "$(bench_evidence_tracked "$EVIDENCE_DIR")" != 0 ]; then
  APX="BENCH_EVIDENCE_APPEND=1 "
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
HARNESS=".\\run_live_radio_monitor.ps1 -TxAdbSerial $TRACTOR_SERIAL -RxAdbSerial $BASE_SERIAL$HIP -TxFeed camera -RegProfile $REGP$FRF -DurationS $DUR -StopDrainS $DRAIN -SynthFps $FPS -KfRequestDisable 1 -ReactiveFire $FIRE -ProbeEcho 0 -LogFragArrivals 1 -TxBatch 0 -CamExtraEnv \"$CAM_ENV\" -Archive"
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
  case $LEG in 2d*) echo "   then, right after the launch, in Git Bash: ${APX}bash \"$(win_path "$LEGS_DIR/leg2d_switch.sh")\" $LEG";; esac
  echo "   while frames flow:            ${APX}bash \"$(win_path "$LEGS_DIR/link_sample.sh")\" $LEG"
  echo "   after '[EVIDENCE] archived to':  ${APX}bash \"$(win_path "$LEGS_DIR/leg_post.sh")\" $LEG"
  [ -z "$APX" ] || echo "   (each line keeps BENCH_EVIDENCE_APPEND=1: this leg appends to a committed EVIDENCE_DIR on purpose)"
  if [ "${1:-}" = probed ]; then
    echo "   NOT flying after all (no GO, harness refused, round aborted)? The probes left both L072s"
    echo "   listening (RXCONT) and rs13_cap_base is running. Stop the capture and park both radios:"
    echo "                                 bash \"$(win_path "$LEGS_DIR/power_up_guard.sh")\" --stop-leg-daemons"
  fi
}
if [ $PRINT_ONLY = 1 ]; then
  write_harness
  echo "leg $LEG: $PLAN"
  plan_notes
  [ -z "$BENCH_EVIDENCE_MSG" ] || printf '%s\n' "$BENCH_EVIDENCE_MSG"
  [ $EV_OK = 1 ] || echo "NOTE: the prep itself (without --print-harness) stops at that REFUSED line, exit 7, before any board action"
  echo "harness wrapper for leg $LEG written (preview only; no board was touched, no capture is running):"
  print_next
  exit 0
fi

E=$(bench_evidence_dir) || die "no evidence folder: $EVIDENCE_DIR (see above)"
exec > >(tee -a "$E/leg${LEG}_prep.txt") 2>&1
hdr() { echo "# leg $LEG $1, $(date -u +%Y-%m-%dT%H:%M:%SZ) PC clock"; }

stamp "== leg $LEG prep ($PLAN) from $(bench_git_desc)"
[ -z "$BENCH_EVIDENCE_MSG" ] || printf '%s\n' "$BENCH_EVIDENCE_MSG"      # the guard's NOTE / WARNING, into the record
plan_notes
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

stamp "-- base-side capture, detached (docker rs13_cap_base): profile $PROFILE, $CAPDUR s, min-frames $MINF ${STRICT:-no-strict}"
board_sh base "$SUDO rm -f $BOARD_STAGE/legs/leg${LEG}_base.jsonl $BOARD_STAGE/legs/leg${LEG}_base.json; \
  $SUDO docker run -d --name rs13_cap_base --network=host -v $BOARD_STAGE:/work -w /work \
  -e PYTHONPATH=/work:/work/paho -e PYTHONUNBUFFERED=1 --entrypoint python3 $BASE_IMAGE \
  /work/vector_dry_run.py capture --topic lifetrac/v25/video/tile_delta --profile $PROFILE \
  --duration $CAPDUR --min-frames $MINF --out /work/legs/leg${LEG}_base.jsonl --json /work/legs/leg${LEG}_base.json $STRICT" \
  | cut -c1-12 | sed 's/^/rs13_cap_base id /'
sleep 3
board_sh base "$SUDO docker logs rs13_cap_base 2>&1 | head -3"

write_harness
stamp "== leg $LEG prep done -> launch the harness NOW (the capture is already listening), in PowerShell:"
print_next probed
