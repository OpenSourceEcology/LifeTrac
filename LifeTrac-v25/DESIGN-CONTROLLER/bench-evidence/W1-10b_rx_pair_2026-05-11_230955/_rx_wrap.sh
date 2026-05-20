# === FCC-B2-b ARTIFACT HEADER BEGIN (v1) ===
# firmware_git_sha: d4dfcb86cfd99bbcbd227844940a1f905336b356
# firmware_git_sha_short: d4dfcb86cfd9
# build_timestamp_utc: 2026-05-20T07:50:02Z
# profile_enum: 0
# profile_string: REG_PROFILE_BENCH_ONLY_FIXED_915
# rfco_summary_schema_ver: 1
# rfco_pertx_schema_ver: 1
# header_schema_ver: 1
# === FCC-B2-b ARTIFACT HEADER END ===

#!/bin/sh
echo fio | sudo -S -p '' env LIFETRAC_PROBE_MODE=rx_listen LIFETRAC_RX_WINDOW=56 bash /tmp/lifetrac_p0c/run_method_h_stage2_tx.sh
rc=$?
printf '__METHOD_H_RC__=%s\n' "$rc"
