# shellcheck shell=bash
# lib/env_file.sh -- the one bench.env loader. lib/bench_env.sh and the setup
# scripts (provision_bench_board.sh, flash_l072.sh, deploy_base.sh,
# build_tractor_image.sh, pull_board_state.sh) source this file and call
#
#     bench_load_env_file FILE
#
# so that every bench script applies the same rules (bench.env.example states
# them for the operator):
#   * CR bytes are dropped first, so a bench.env saved with CRLF endings works;
#   * every VAR=... line of FILE is applied, EXCEPT that a variable which is
#     already NON-EMPTY when the function runs keeps that value: the caller's
#     environment wins over the file (one-off override:
#     `TRACTOR_SERIAL=... bash flash_l072.sh ...`);
#   * old names: BASE_USER is BASE_SSH_USER and BENCH_ARCHIVE_DIR is ARCHIVE_DIR.
#     A caller's old name counts as the caller's new name; at the same level the
#     new name wins. Afterwards both names of a pair hold the same value (both
#     stay unset when neither is set).
# FILE is bash: plain VAR=value lines (a `declare` or `local` in it would stay
# inside the function). Nothing is exported here; each script decides what to
# export. Defines only the function; needs bash 3.2 or later.

bench_load_env_file() {
  local _blef_file=$1 _blef_v _blef_p _blef_keep="" _blef_new _blef_old
  local _blef_aliases="BASE_SSH_USER:BASE_USER ARCHIVE_DIR:BENCH_ARCHIVE_DIR"
  [ -f "$_blef_file" ] || { echo "bench_load_env_file: no such file: $_blef_file" >&2; return 1; }
  for _blef_p in $_blef_aliases; do               # the caller's old name = its new name
    _blef_new=${_blef_p%%:*}; _blef_old=${_blef_p#*:}
    if [ -z "${!_blef_new:-}" ] && [ -n "${!_blef_old:-}" ]; then printf -v "$_blef_new" '%s' "${!_blef_old}"; fi
  done
  # snapshot every non-empty variable the file would assign (and the new alias names)
  for _blef_v in $(tr -d '\r' < "$_blef_file" \
                   | sed -n 's/^[[:space:]]*\(export[[:space:]][[:space:]]*\)\{0,1\}\([A-Za-z_][A-Za-z0-9_]*\)=.*/\2/p') \
                 BASE_SSH_USER ARCHIVE_DIR; do
    case $_blef_v in _blef_*) continue ;; esac
    if [ -n "${!_blef_v:-}" ]; then
      _blef_keep="$_blef_keep $_blef_v"
      printf -v "_blef_pre_$_blef_v" '%s' "${!_blef_v}"
    fi
  done
  eval "$(tr -d '\r' < "$_blef_file")" || { echo "bench_load_env_file: error in $_blef_file" >&2; return 1; }
  for _blef_v in $_blef_keep; do                 # the caller's values win
    _blef_p=_blef_pre_$_blef_v
    if [ -n "${!_blef_p+x}" ]; then printf -v "$_blef_v" '%s' "${!_blef_p}"; unset "$_blef_p"; fi
  done
  for _blef_p in $_blef_aliases; do               # one value for both names
    _blef_new=${_blef_p%%:*}; _blef_old=${_blef_p#*:}
    if [ -n "${!_blef_new:-}" ]; then printf -v "$_blef_old" '%s' "${!_blef_new}"
    elif [ -n "${!_blef_old:-}" ]; then printf -v "$_blef_new" '%s' "${!_blef_old}"
    fi
  done
  return 0
}
