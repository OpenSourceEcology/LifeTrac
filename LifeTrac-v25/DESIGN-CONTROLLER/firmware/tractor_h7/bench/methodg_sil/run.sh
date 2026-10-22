#!/bin/sh
# Build and run the Method G control-path SIL test on the host.
# Usage: run.sh [build_dir]   (from anywhere; needs cc and c++)
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
SKETCH=$(cd "$HERE/../.." && pwd)
COMMON=$(cd "$SKETCH/../common" && pwd)
OUT=${1:-"${TMPDIR:-/tmp}/lifetrac_methodg_sil"}
mkdir -p "$OUT/stage/src/lora_proto"

# Stage shared sources the way arduino-ci.yml does, with the all-zero
# placeholder key (stub crypto; never a real key).
cp "$COMMON"/lora_proto/*.h "$COMMON"/lora_proto/*.c "$OUT/stage/src/lora_proto/"
cp "$COMMON/shared_mem.h" "$COMMON/lifetrac_build_config.h" "$OUT/stage/src/"
cat > "$OUT/stage/src/lora_proto/key.h" <<'KEYEOF'
#ifndef LIFETRAC_LORA_KEY_H
#define LIFETRAC_LORA_KEY_H
#define LIFETRAC_KEY_ID 0x00000000UL
#define LIFETRAC_FLEET_KEY_BYTES { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 }
#endif
KEYEOF

DEFS="-DLIFETRAC_USE_METHOD_G_HOST=1 -DLIFETRAC_MH_SERIAL=Serial3 -DLIFETRAC_MH_BENCH_LOG=0 -DLIFETRAC_ALLOW_STUB_CRYPTO -DLIFETRAC_ALLOW_UNCONFIGURED_KEY"
INCS="-I$HERE/fake_arduino -I$OUT/stage -I$SKETCH -I$SKETCH/murata_host"

for f in mh_cobs mh_crc16 murata_host mh_stats mh_uart mh_stream mh_link mh_runtime_health mh_runtime; do
    cc -std=gnu11 -Wall -Wextra -Werror $DEFS $INCS -c "$SKETCH/murata_host/$f.c" -o "$OUT/$f.o"
done
c++ -std=gnu++17 -Wall -Wextra -Werror -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter $DEFS $INCS \
    -c "$SKETCH/murata_host/mh_uart_arduino.cpp" -o "$OUT/mh_uart_arduino.o"
cc -std=gnu11 $DEFS $INCS -c "$OUT/stage/src/lora_proto/lora_proto.c" -o "$OUT/lora_proto.o"
cc -std=gnu11 $DEFS $INCS -c "$OUT/stage/src/lora_proto/crypto_stub.c" -o "$OUT/crypto_stub.o"
c++ -std=gnu++17 -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-unused-parameter \
    -Wno-unused-but-set-variable $DEFS $INCS \
    "$HERE/tractor_methodg_sil.cpp" "$OUT"/*.o -lm -o "$OUT/tractor_methodg_sil"
"$OUT/tractor_methodg_sil"
