#!/usr/bin/env bash
# Render the OpenSCAD intersection() of two rigid groups at one animation time.
#
# This is the "pure OpenSCAD" probe from issue #119: the result is empty (OpenSCAD 2021.01
# exits 1 and writes no file) when the two groups do not overlap, otherwise the STL is the
# exact overlap volume, which analyze_pairs.py measures and locates.
#
# Usage: export_pair.sh <t> <groupA> <groupB> [openscad-binary]
set -u
T="$1"; A="$2"; B="$3"; BIN="${4:-${OPENSCAD:-openscad}}"

HERE="$(cd "$(dirname "$0")" && pwd)"
SCAD_DIR="$(cd "$HERE/../../DESIGN-STRUCTURAL/openscad" && pwd)"
OUT="${COLLISION_OUT:-$HERE/out}"
mkdir -p "$OUT"

PROBE="$OUT/_probe_pairs.scad"
if [ ! -f "$PROBE" ] || [ "$SCAD_DIR/lifetrac_v25.scad" -nt "$PROBE" ]; then
  sed -e "s|^include <lifetrac_v25_params.scad>|include <$SCAD_DIR/lifetrac_v25_params.scad>|" \
      -e "s|^use <\(.*\)>|use <$SCAD_DIR/\1>|" \
      -e "s|^calculate_cog();|// calculate_cog(); // removed: draws a 100mm COG marker sphere|" \
      -e "s|^lifetrac_v25_assembly();|// lifetrac_v25_assembly(); // replaced by the pair probe below|" \
      "$SCAD_DIR/lifetrac_v25.scad" > "$PROBE"
  cat >> "$PROBE" <<'EOF'

// ---- collision pair probe (appended by tools/collision-check/export_pair.sh) ----
pair_a = "arms";
pair_b = "bucket";
module collision_group(name) {
    if (name == "arms") loader_arms();
    else if (name == "bucket") bucket_attachment();
    else if (name == "frame") base_frame();
    else if (name == "hydraulics") { lift_cylinders(); bucket_cylinders(); }
    else if (name == "wheels") { wheel_assemblies(); uwu_assemblies(); }
    else if (name == "platform") folding_platform_assembly(platform_fold_angle);
    else assert(false, str("unknown collision group: ", name));
}
intersection() { collision_group(pair_a); collision_group(pair_b); }
EOF
fi

STL="$OUT/pair_${A}_${B}_t${T}.stl"
LOG="$OUT/pair_${A}_${B}_t${T}.log"
START=$(date +%s)
timeout "${OPENSCAD_TIMEOUT:-1500}" "$BIN" -o "$STL" -D "pair_a=\"$A\"" -D "pair_b=\"$B\"" \
    -D "animation_time=$T" -D "\$t=$T" "$PROBE" > "$LOG" 2>&1
RC=$?
END=$(date +%s)
FACETS=$(grep -c 'facet normal' "$STL" 2>/dev/null || echo 0)
MSG=$(grep -i -E 'empty|ERROR' "$LOG" | head -1)
echo "pair=$A/$B t=$T rc=$RC seconds=$((END-START)) facets=$FACETS msg=$MSG" | tee -a "$OUT/pair_timings.txt"
exit 0
