#!/bin/sh
# Mutation check: build the NEW host tests against the OLD 1000 ms gap and
# confirm they fail (static asserts commented out in the copy only).
set -e
W=/c/Users/dorkm/Documents/GitHub/LifeTrac/.claude/worktrees/wf_adf2ad4c-341-4/LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072
S=/c/Users/dorkm/AppData/Local/Temp/claude/C--Users-dorkm-Documents-GitHub-LifeTrac/5eaec8c2-12ac-4272-80af-b19d1a563f48/scratchpad/imp/fhss-authority/mut
CC=${CC:-gcc}
rm -rf "$S"
mkdir -p "$S/include" "$S/radio" "$S/bench/host_proto"
cp -r "$W/include/." "$S/include/"
cp -r "$W/bench/host_proto/." "$S/bench/host_proto/"
for f in sx1276_fhss_authority.c sx1276_rx_grid_policy.c sx1276_fhss_clock.c sx1276_fhss.c sx1276_fhss_chantab.c; do
  cp "$W/radio/$f" "$S/radio/"
done
sed -i 's/STREAK_GAP_MS 1500U/STREAK_GAP_MS 1000U/' "$S/include/sx1276_fhss_authority.h"
grep -n "define SX1276_FHSS_AUTHORITY_STREAK_GAP_MS" "$S/include/sx1276_fhss_authority.h"
# drop the _Static_assert blocks in the copies
awk 'BEGIN{skip=0} /^_Static_assert\(SX1276_FHSS_AUTHORITY/{skip=1} { if(!skip) print; if(skip && /\);/) skip=0 }' "$S/radio/sx1276_fhss_authority.c" > "$S/radio/a.c" && mv "$S/radio/a.c" "$S/radio/sx1276_fhss_authority.c"
awk 'BEGIN{skip=0} /^_Static_assert\(SX1276_FHSS_AUTHORITY/{skip=1} { if(!skip) print; if(skip && /\);/) skip=0 }' "$S/radio/sx1276_rx_grid_policy.c" > "$S/radio/b.c" && mv "$S/radio/b.c" "$S/radio/sx1276_rx_grid_policy.c"
grep -c "_Static_assert" "$S/radio/sx1276_fhss_authority.c" "$S/radio/sx1276_rx_grid_policy.c" || true
cd "$S"
$CC -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto bench/host_proto/fhss_authority.c radio/sx1276_fhss_authority.c -o fa_old.exe
$CC -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto bench/host_proto/rx_grid_policy.c radio/sx1276_rx_grid_policy.c radio/sx1276_fhss_authority.c radio/sx1276_fhss_clock.c radio/sx1276_fhss.c radio/sx1276_fhss_chantab.c -o rg_old.exe
set +e
echo "=== fhss_authority vs OLD gap ==="
./fa_old.exe; echo "rc=$?"
echo "=== rx_grid_policy vs OLD gap ==="
./rg_old.exe; echo "rc=$?"
