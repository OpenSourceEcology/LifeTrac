#!/usr/bin/env bash
# Export one rigid group of the LifeTrac v25 assembly as STL at one animation time.
#
# Uses only the existing show_* toggles in lifetrac_v25.scad, so the model file is not
# modified. A sanitized copy of the assembly is generated on the fly that (a) makes the
# include/use paths absolute and (b) drops the top-level calculate_cog() call, because
# that module draws a 100 mm marker sphere which would otherwise end up in every export.
#
# Usage: export_groups.sh <t> <group> [openscad-binary]
#   t      animation time 0..1 (0 = arms down, 0.5 = arms fully raised)
#   group  frame | wheels | arms | bucket | hydraulics | platform
set -u
T="$1"; GROUP="$2"; BIN="${3:-${OPENSCAD:-openscad}}"

HERE="$(cd "$(dirname "$0")" && pwd)"
SCAD_DIR="$(cd "$HERE/../../DESIGN-STRUCTURAL/openscad" && pwd)"
OUT="${COLLISION_OUT:-$HERE/out}"
mkdir -p "$OUT"

PROBE="$OUT/_probe_lifetrac_v25.scad"
if [ ! -f "$PROBE" ] || [ "$SCAD_DIR/lifetrac_v25.scad" -nt "$PROBE" ]; then
  sed -e "s|^include <lifetrac_v25_params.scad>|include <$SCAD_DIR/lifetrac_v25_params.scad>|" \
      -e "s|^use <\(.*\)>|use <$SCAD_DIR/\1>|" \
      -e "s|^calculate_cog();|// calculate_cog(); // removed: draws a 100mm COG marker sphere|" \
      "$SCAD_DIR/lifetrac_v25.scad" > "$PROBE"
fi

FLAGS="-D show_frame=false -D show_wheels=false -D show_hydraulics=false -D show_loader_arms=false -D show_bucket=false -D show_folding_platform=false"
case "$GROUP" in
  frame)      ON="-D show_frame=true" ;;
  wheels)     ON="-D show_wheels=true" ;;
  arms)       ON="-D show_loader_arms=true" ;;
  bucket)     ON="-D show_bucket=true" ;;
  hydraulics) ON="-D show_hydraulics=true" ;;
  platform)   ON="-D show_folding_platform=true" ;;
  *) echo "unknown group '$GROUP' (frame|wheels|arms|bucket|hydraulics|platform)"; exit 2 ;;
esac

STL="$OUT/${GROUP}_t${T}.stl"
LOG="$OUT/${GROUP}_t${T}.log"
START=$(date +%s)
timeout "${OPENSCAD_TIMEOUT:-1500}" "$BIN" -o "$STL" $FLAGS $ON -D "animation_time=$T" -D "\$t=$T" "$PROBE" > "$LOG" 2>&1
RC=$?
END=$(date +%s)
FACETS=$(grep -c 'facet normal' "$STL" 2>/dev/null || echo 0)
echo "group=$GROUP t=$T rc=$RC seconds=$((END-START)) facets=$FACETS" | tee -a "$OUT/timings.txt"
exit $RC
