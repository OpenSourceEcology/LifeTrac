#!/usr/bin/env bash
# build_tractor_image.sh -- build the tractor's camera/encoder image
# (lifetrac-tractor-x8) from one commit, natively on the bench BASE X8, keep a
# copy on the PC, and load it on the offline TRACTOR over adb.
#
# Why the base: the image is linux/arm64, the bench PC has no Docker, and the
# tractor has no network (WiFi off), so it cannot pip-install the numpy/OpenCV
# wheels itself. The base X8 is aarch64 and online.
#
# Usage (Git Bash on the bench PC, or any bash with git + ssh/scp or adb):
#   bash build_tractor_image.sh [options] [<commit>]    # <commit> defaults to HEAD
#   bash build_tractor_image.sh --push-only <image.tar.gz>
#
# Options:
#   --ssh | --adb      transport to the BASE (same rule as deploy_base.sh). The
#                      tractor is always reached over adb (TRACTOR_SERIAL).
#   --unlocked         build without firmware/tractor_x8/requirements.lock.txt
#                      (pip resolves requirements.txt unpinned). Default:
#                      --build-arg LOCKED=1, the flown wheel versions.
#   --with-mosquitto   also export eclipse-mosquitto:2 from the base and load it
#                      on the tractor (the tractor cannot pull it).
#   --no-tractor       build and archive on the PC only; leave the tractor alone.
#   --push-only FILE   skip the build: load an archived image tarball (docker
#                      save, .tar or .tar.gz) on the tractor, for example to roll
#                      back to a flown image from the PC archive.
#   --keep-tarball     keep the tarball on the tractor after loading
#                      (default: removed once docker load succeeded).
#   --dry-run          make the build context and the board-side scripts
#                      locally, print the plan and stop. Never contacts a board.
#   -h | --help
#
# Settings: bench_tools/bench.env when it exists, else the environment, else
# the same defaults as deploy_base.sh (BASE_TRANSPORT, BASE_HOST, BASE_USER,
# BASE_SSH_KEY, BASE_SERIAL, TRACTOR_SERIAL, BENCH_SUDO_PW, BENCH_ARCHIVE_DIR).
# Image tarballs land in $BENCH_ARCHIVE_DIR/images (default
# $HOME/LifeTrac-bench-archive/images), which must be outside git.
#
# What it changes:
#   base:    /home/<user>/lifetrac_build/ (context + tarball, the tarball is
#            removed once the PC copy is verified), image tags
#            lifetrac-tractor-x8:<sha> and :latest, log
#            /home/<user>/docker_build_tractor_<sha>.log.
#   PC:      lifetrac-tractor-x8_<sha>.tar.gz + .manifest.txt + .pip-freeze.txt
#            + build log in the images folder.
#   tractor: /home/<user>/lifetrac_images/ (temporary), the loaded image, its
#            :latest tag, and lifetrac-tractor-x8:previous = the image :latest
#            pointed at before.
#
# RADIO SAFETY: nothing here opens /dev/ttymxc3 or restarts a container. The
# smoke tests run the image with --network none and no devices. A container
# already running the old image (for example tractor-camera, started by
# lifetrac-camera.service at boot) keeps running it until it is recreated;
# that unit maps /dev/ttymxc3, and the bench stops it before any radio work
# (BENCH_RUNBOOK prep 3). See DEPLOY.md.
set -euo pipefail

BT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DC_REL=LifeTrac-v25/DESIGN-CONTROLLER
CTX_REL=$DC_REL/firmware/tractor_x8
IMAGE=lifetrac-tractor-x8

# ---------------------------------------------------------------- settings --
BENCH_ENV=${BENCH_ENV:-${BENCH_ENV_FILE:-$BT/bench.env}}
if [ -f "$BENCH_ENV" ]; then
    set -a
    eval "$(tr -d '\r' < "$BENCH_ENV")"
    set +a
fi

COMMIT=HEAD
OPT_TRANSPORT=""
LOCKED=1
WITH_MQ=0
DO_TRACTOR=1
PUSH_ONLY=""
KEEP=0
DRY_RUN=0

usage() { sed -n '2,/^set -euo/p' "${BASH_SOURCE[0]}" | sed '$d; s/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case $1 in
        --ssh) OPT_TRANSPORT=ssh ;;
        --adb) OPT_TRANSPORT=adb ;;
        --unlocked) LOCKED=0 ;;
        --with-mosquitto) WITH_MQ=1 ;;
        --no-tractor) DO_TRACTOR=0 ;;
        --push-only) [ $# -ge 2 ] || { echo "--push-only needs a file" >&2; exit 2; }; PUSH_ONLY=$2; shift ;;
        --keep-tarball) KEEP=1 ;;
        --dry-run) DRY_RUN=1 ;;
        -h|--help) usage; exit 0 ;;
        -*) echo "build_tractor_image: unknown option $1 (see --help)" >&2; exit 2 ;;
        *) COMMIT=$1 ;;
    esac
    shift
done

if [ -n "$OPT_TRANSPORT" ]; then
    BASE_TRANSPORT=$OPT_TRANSPORT
elif [ -z "${BASE_TRANSPORT:-}" ]; then
    if [ -n "${BASE_HOST:-}" ]; then BASE_TRANSPORT=ssh
    elif [ -n "${BASE_SERIAL:-}" ]; then BASE_TRANSPORT=adb
    else BASE_TRANSPORT=ssh
    fi
fi
case $BASE_TRANSPORT in ssh|adb) ;; *) echo "build_tractor_image: BASE_TRANSPORT must be ssh or adb" >&2; exit 2 ;; esac
BASE_HOST=${BASE_HOST:-192.168.1.117}
BASE_USER=${BASE_USER:-${BASE_SSH_USER:-fio}}
BASE_SSH_KEY=${BASE_SSH_KEY:-$HOME/.ssh/lifetrac_base_ed25519}
BASE_SERIAL=${BASE_SERIAL:-2D0A1209DABC240B}
TRACTOR_SERIAL=${TRACTOR_SERIAL:-2E2C1209DABC240B}
TRACTOR_USER=${TRACTOR_USER:-fio}
BENCH_SUDO_PW=${BENCH_SUDO_PW-fio}
BENCH_ARCHIVE_DIR=${BENCH_ARCHIVE_DIR:-${ARCHIVE_DIR:-$HOME/Documents/LifeTrac-bench-archive}}
BASE_WORK=/home/$BASE_USER/lifetrac_build
TRACTOR_WORK=/home/$TRACTOR_USER/lifetrac_images   # on disk: /tmp is a RAM tmpfs aged at 5 d

# ----------------------------------------------------------------- helpers --
say() { printf '[build_tractor_image %s] %s\n' "$(date -u +%H:%M:%SZ)" "$*"; }
die() { printf '[build_tractor_image] ERROR: %s\n' "$*" >&2; exit 1; }
sq() { local s=${1//\'/\'\\\'\'}; printf "'%s'" "$s"; }
winpath() { if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else printf '%s' "$1"; fi; }
# tar and scp read "C:/..." as host "C"; Git Bash paths must look like /c/...
unixpath() { if command -v cygpath >/dev/null 2>&1; then cygpath -u "$1"; else printf '%s' "$1"; fi; }
md5_local() { if command -v md5sum >/dev/null 2>&1; then md5sum "$1" | cut -d' ' -f1; else md5 -q "$1"; fi; }
BENCH_ARCHIVE_DIR=$(unixpath "$BENCH_ARCHIVE_DIR")
BASE_SSH_KEY=$(unixpath "$BASE_SSH_KEY")
[ -z "$PUSH_ONLY" ] || PUSH_ONLY=$(unixpath "$PUSH_ONLY")

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new -o ServerAliveInterval=15)
[ -f "$BASE_SSH_KEY" ] && SSH_OPTS=(-i "$BASE_SSH_KEY" "${SSH_OPTS[@]}")

adb_sh() {    # adb_sh <serial> <command line>; returns the remote exit code
    local serial=$1 cmd=$2 line rc=""
    while IFS= read -r line || [ -n "$line" ]; do
        line=${line%$'\r'}
        case $line in
            *__LT_RC=*) rc=${line##*__LT_RC=}; line=${line%__LT_RC=*}; [ -z "$line" ] || printf '%s\n' "$line" ;;
            *) printf '%s\n' "$line" ;;
        esac
    done < <(MSYS_NO_PATHCONV=1 adb -s "$serial" shell "$cmd"'; echo "__LT_RC=$?"' < /dev/null 2>&1)
    [ -n "$rc" ] || { echo "adb shell on $serial returned no status (device gone?)" >&2; return 255; }
    return "$rc"
}
root_cmd() {  # wrap a command line (that reads no stdin) to run as root
    local pw; pw=$(sq "$BENCH_SUDO_PW")
    printf '%s' "if sudo -n true 2>/dev/null; then sudo -n $1; elif [ -n $pw ]; then printf '%s\n' $pw | sudo -S -p '' $1; else echo 'sudo needs a password: install the bench sudoers drop-in (BENCH_SETUP.md) or set BENCH_SUDO_PW' >&2; false; fi"
}

b_sh() {
    if [ "$BASE_TRANSPORT" = ssh ]; then
        ssh -n "${SSH_OPTS[@]}" "$BASE_USER@$BASE_HOST" "$1"
    else
        adb_sh "$BASE_SERIAL" "$1"
    fi
}
b_put() {     # b_put <local file> <remote dir>
    if [ "$BASE_TRANSPORT" = ssh ]; then
        scp -q "${SSH_OPTS[@]}" "$1" "$BASE_USER@$BASE_HOST:$2/"
    else
        MSYS_NO_PATHCONV=1 adb -s "$BASE_SERIAL" push "$(winpath "$1")" "$2/" > /dev/null
    fi
}
b_get() {     # b_get <remote file> <local dir>
    if [ "$BASE_TRANSPORT" = ssh ]; then
        scp -q "${SSH_OPTS[@]}" "$BASE_USER@$BASE_HOST:$1" "$2/"
    else
        MSYS_NO_PATHCONV=1 adb -s "$BASE_SERIAL" pull "$1" "$(winpath "$2")/" > /dev/null
    fi
}
t_sh() { adb_sh "$TRACTOR_SERIAL" "$1"; }
t_put() { MSYS_NO_PATHCONV=1 adb -s "$TRACTOR_SERIAL" push "$(winpath "$1")" "$2/" > /dev/null; }

case $(cd "$BENCH_ARCHIVE_DIR" 2>/dev/null && pwd -P || echo "$BENCH_ARCHIVE_DIR")/ in
    "$(cd "$(git -C "$BT" rev-parse --show-toplevel)" && pwd -P)"/*) die "BENCH_ARCHIVE_DIR must be outside the git repo" ;;
esac
IMG_DIR=$BENCH_ARCHIVE_DIR/images
STAGE=$BENCH_ARCHIVE_DIR/build
mkdir -p "$IMG_DIR" "$STAGE"

# ------------------------------------------------- board-side scripts (sh) --
# Both run as root. Values arrive as arguments; the quoted heredocs are verbatim.
BUILD_SCRIPT=$STAGE/build_tractor_remote.sh
cat > "$BUILD_SCRIPT" <<'REMOTE'
#!/bin/sh
# Board side of build_tractor_image.sh: build on the BASE X8 (aarch64), as root.
set -eu
CTX=$1 SHORT=$2 SHA=$3 LOCKED=$4 WITH_MQ=$5 WORK=$6 OWNER=$7
IMAGE=lifetrac-tractor-x8
say() { echo "[base $(date -u +%H:%M:%SZ)] $*"; }
die() { echo "[base] ERROR: $*" >&2; exit 1; }

command -v docker >/dev/null 2>&1 || die "docker not found"
st=$(systemctl is-enabled compose-apps-early-start-recovery.service 2>/dev/null || true)
case $st in
    masked*|"") ;;
    *) die "compose-apps-early-start-recovery.service is '$st', not masked: it wipes /var/lib/docker every 60 s on the LmP 934 image (BENCH_SETUP.md)." ;;
esac
free_kb=$(df -Pk "$WORK" | awk 'NR==2 {print $4}')
[ "$free_kb" -gt 3000000 ] || die "less than 3 GB free under $WORK"

dir=$WORK/tractor_x8_$SHORT
rm -rf "$dir"
mkdir -p "$dir"
tar -xzf "$CTX" -C "$dir"
cd "$dir"
build_args=""
locked_eff=0
if [ "$LOCKED" = 1 ]; then
    if [ -f requirements.lock.txt ] && grep -q '^ARG LOCKED' Dockerfile; then
        build_args="--build-arg LOCKED=1"
        locked_eff=1
    else
        say "WARNING: $SHORT has no lock-file support; building unlocked"
    fi
fi
log=/home/$OWNER/docker_build_tractor_$SHORT.log
say "docker build -t $IMAGE:$SHORT -t $IMAGE:latest $build_args . (log $log)"
# shellcheck disable=SC2086
if ! docker build $build_args --label "org.lifetrac.source_sha=$SHA" \
        -t "$IMAGE:$SHORT" -t "$IMAGE:latest" . > "$log" 2>&1; then
    tail -n 40 "$log"
    die "docker build failed (full log: $log)"
fi
cp "$log" "$WORK/"
id=$(docker image inspect -f '{{.Id}}' "$IMAGE:$SHORT")
say "built $id (locked=$locked_eff)"

# Smoke test without network or devices: the VECTOR encoder needs numpy + cv2.
docker run --rm --network none --entrypoint python3 "$IMAGE:$SHORT" -c \
    "import numpy, cv2; import x8_image_pipeline.encode_vector as e; print('numpy', numpy.__version__, 'cv2', cv2.__version__, 'encoder', e.VectorEncoder.__name__)" \
    > "$WORK/${IMAGE}_$SHORT.smoke.txt" 2>&1 || { cat "$WORK/${IMAGE}_$SHORT.smoke.txt"; die "smoke test failed"; }
cat "$WORK/${IMAGE}_$SHORT.smoke.txt"
docker run --rm --network none --entrypoint python3 "$IMAGE:$SHORT" -m pip freeze \
    > "$WORK/${IMAGE}_$SHORT.pip-freeze.txt" 2>&1 || say "WARNING: pip freeze failed"

# docker save to a file, then gzip: no pipe, so a failure cannot hide.
rm -f "$WORK/${IMAGE}_$SHORT.tar" "$WORK/${IMAGE}_$SHORT.tar.gz"
docker save -o "$WORK/${IMAGE}_$SHORT.tar" "$IMAGE:$SHORT" "$IMAGE:latest"
gzip -f "$WORK/${IMAGE}_$SHORT.tar"
md5sum "$WORK/${IMAGE}_$SHORT.tar.gz" | cut -d' ' -f1 > "$WORK/${IMAGE}_$SHORT.tar.gz.md5"
{
    echo "image=$IMAGE:$SHORT $id"
    echo "source_sha=$SHA"
    echo "build_locked=$locked_eff"
    echo "built_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ) on $(uname -n)"
    echo "build_log=docker_build_tractor_$SHORT.log"
    echo "tarball_md5=$(cat "$WORK/${IMAGE}_$SHORT.tar.gz.md5")"
} > "$WORK/${IMAGE}_$SHORT.manifest.txt"

if [ "$WITH_MQ" = 1 ]; then
    docker image inspect eclipse-mosquitto:2 > /dev/null 2>&1 || docker pull eclipse-mosquitto:2
    rm -f "$WORK/eclipse-mosquitto_2.tar" "$WORK/eclipse-mosquitto_2.tar.gz"
    docker save -o "$WORK/eclipse-mosquitto_2.tar" eclipse-mosquitto:2
    gzip -f "$WORK/eclipse-mosquitto_2.tar"
    md5sum "$WORK/eclipse-mosquitto_2.tar.gz" | cut -d' ' -f1 > "$WORK/eclipse-mosquitto_2.tar.gz.md5"
    echo "eclipse-mosquitto:2 $(docker image inspect -f '{{.Id}}' eclipse-mosquitto:2)" >> "$WORK/${IMAGE}_$SHORT.manifest.txt"
fi
rm -rf "$dir"
chown -R "$OWNER:$OWNER" "$WORK" "$log"
say "saved $WORK/${IMAGE}_$SHORT.tar.gz ($(wc -c < "$WORK/${IMAGE}_$SHORT.tar.gz") B)"
REMOTE

LOAD_SCRIPT=$STAGE/load_tractor_remote.sh
cat > "$LOAD_SCRIPT" <<'REMOTE'
#!/bin/sh
# Board side of build_tractor_image.sh: docker load on the TRACTOR, as root.
# Loads images only. It starts, stops and recreates no container.
set -eu
TGZ=$1 KEEP=$2 MQ_TGZ=$3
IMAGE=lifetrac-tractor-x8
say() { echo "[tractor $(date -u +%H:%M:%SZ)] $*"; }
die() { echo "[tractor] ERROR: $*" >&2; exit 1; }

command -v docker >/dev/null 2>&1 || die "docker not found"
if [ -f /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER/DEPLOYED_FROM.txt ]; then
    die "this board carries the base deploy tree: it looks like the BASE, not the tractor"
fi
[ -f "$TGZ" ] || die "missing $TGZ"
free_kb=$(df -Pk /var/lib/docker 2>/dev/null | awk 'NR==2 {print $4}')
[ -z "$free_kb" ] || [ "$free_kb" -gt 2000000 ] || die "less than 2 GB free for /var/lib/docker"

old=$(docker image inspect -f '{{.Id}}' "$IMAGE:latest" 2>/dev/null || true)
# -i, never stdin: `sudo -S` would eat a piped tarball.
docker load -i "$TGZ"
new=$(docker image inspect -f '{{.Id}}' "$IMAGE:latest" 2>/dev/null || true)
[ -n "$new" ] || die "no $IMAGE:latest after the load"
if [ -n "$old" ] && [ "$old" != "$new" ]; then
    docker tag "$old" "$IMAGE:previous"
    say "$IMAGE:previous = the old :latest ($(echo "$old" | cut -c8-19)); roll back with: docker tag $IMAGE:previous $IMAGE:latest"
elif [ "$old" = "$new" ]; then
    say "$IMAGE:latest did not change (same image, or the tarball carries no :latest tag); :previous untouched"
fi
docker images "$IMAGE"
docker run --rm --network none --entrypoint python3 "$IMAGE:latest" -c \
    "import numpy, cv2; import x8_image_pipeline.encode_vector as e; print('numpy', numpy.__version__, 'cv2', cv2.__version__, 'encoder', e.VectorEncoder.__name__)" \
    || die "smoke test of $IMAGE:latest failed on the tractor"
if [ -n "$MQ_TGZ" ]; then
    [ -f "$MQ_TGZ" ] || die "missing $MQ_TGZ"
    docker load -i "$MQ_TGZ"
fi
running=$(docker ps --filter "ancestor=$IMAGE" --format '{{.Names}} ({{.Image}})' | tr '\n' ' ' | sed 's/ *$//')
[ -z "$running" ] || say "NOTE: running now, not restarted, still on the image they started with: $running"
if [ "$KEEP" != 1 ]; then
    rm -f "$TGZ"
    [ -z "$MQ_TGZ" ] || rm -f "$MQ_TGZ"
fi
say "done"
REMOTE

for f in "$BUILD_SCRIPT" "$LOAD_SCRIPT"; do
    tr -d '\r' < "$f" > "$f.tmp" && mv "$f.tmp" "$f"
    sh -n "$f" || die "generated board script $f has a syntax error"
done

# ------------------------------------------------- local: the build context --
if [ -z "$PUSH_ONLY" ]; then
    REPO=$(git -C "$BT" rev-parse --show-toplevel)
    SHA=$(git -C "$REPO" rev-parse --verify --quiet "${COMMIT}^{commit}") || die "not a commit: $COMMIT"
    SHORT=${SHA:0:8}
    SUBJECT=$(git -C "$REPO" log -1 --format=%s "$SHA")
    git -C "$REPO" cat-file -e "$SHA:$CTX_REL/Dockerfile" 2>/dev/null || die "$SHORT has no $CTX_REL/Dockerfile"
    HAS_LOCK=0
    git -C "$REPO" cat-file -e "$SHA:$CTX_REL/requirements.lock.txt" 2>/dev/null && HAS_LOCK=1
    CTX=$STAGE/tractor_x8_ctx_$SHORT.tar.gz
    # The Dockerfile does `COPY . /app`: the context is firmware/tractor_x8 alone
    # (its VS1 codec is an in-tree mirror; it imports nothing from base_station).
    # LF forced: see the CRLF trap in DEPLOY.md.
    git -C "$REPO" -c core.autocrlf=false -c core.eol=lf archive --format=tar.gz -o "$CTX" "$SHA:$CTX_REL"
    TEXT_RE='(\.(sh|py|yml|conf|cfg|service|txt)|Dockerfile)$'
    CR_TAR=$(set +o pipefail; { tar -xzOf "$CTX" --wildcards '*.sh' '*.py' '*.yml' '*.conf' '*.cfg' \
        '*.service' '*.txt' '*Dockerfile' 2>/dev/null || true; } | tr -cd '\r' | wc -c)
    CR_GIT=$(set +o pipefail; tar -tzf "$CTX" | grep -E "$TEXT_RE" | sed "s|^|$SHA:$CTX_REL/|" \
        | git -C "$REPO" cat-file --batch | tr -cd '\r' | wc -c)
    [ "${CR_TAR// /}" -le "${CR_GIT// /}" ] \
        || die "the context tarball has CRLF the commit does not have; check .gitattributes / core.eol (DEPLOY.md)"
    CTX_MD5=$(md5_local "$CTX")
    TGZ_NAME=${IMAGE}_$SHORT.tar.gz
    say "commit   $SHA ($SUBJECT)"
    say "context  $CTX ($(wc -c < "$CTX") B, md5 $CTX_MD5)"
    if [ "$BASE_TRANSPORT" = ssh ]; then say "base     $BASE_USER@$BASE_HOST (ssh), work dir $BASE_WORK"; else say "base     adb $BASE_SERIAL, work dir $BASE_WORK"; fi
    say "build    $IMAGE:$SHORT + :latest, locked=$LOCKED (commit has lock file: $HAS_LOCK), mosquitto export=$WITH_MQ"
    say "PC copy  $IMG_DIR/$TGZ_NAME"
else
    [ -f "$PUSH_ONLY" ] || die "no such file: $PUSH_ONLY"
    TGZ_NAME=$(basename "$PUSH_ONLY")
    say "push-only: $PUSH_ONLY (md5 $(md5_local "$PUSH_ONLY"))"
fi
say "tractor  $([ "$DO_TRACTOR" = 1 ] && echo "adb $TRACTOR_SERIAL, docker load from $TRACTOR_WORK" || echo untouched)"

if [ "$DRY_RUN" = 1 ]; then
    say "dry run: board scripts $BUILD_SCRIPT and $LOAD_SCRIPT; nothing was sent"
    exit 0
fi
if [ "$DO_TRACTOR" = 1 ] && [ "$TRACTOR_SERIAL" = "$BASE_SERIAL" ] && [ "$BASE_TRANSPORT" = adb ]; then
    die "TRACTOR_SERIAL equals BASE_SERIAL"
fi

# ------------------------------------------------- build on the base, fetch --
if [ -z "$PUSH_ONLY" ]; then
    b_sh "true" > /dev/null || die "cannot reach the base"
    b_sh "mkdir -p $(sq "$BASE_WORK")"
    b_put "$CTX" "$BASE_WORK" || die "copy of the context failed"
    b_put "$BUILD_SCRIPT" "$BASE_WORK" || die "copy of the build script failed"
    remote_md5=$(b_sh "md5sum $(sq "$BASE_WORK/$(basename "$CTX")")" | tr -d '\r' | cut -d' ' -f1)
    [ "$remote_md5" = "$CTX_MD5" ] || die "context md5 mismatch on the base"
    say "building on the base (several minutes when the pip layer is not cached)"
    b_sh "$(root_cmd "sh $(sq "$BASE_WORK/build_tractor_remote.sh") $(sq "$BASE_WORK/$(basename "$CTX")") $SHORT $SHA $LOCKED $WITH_MQ $(sq "$BASE_WORK") $(sq "$BASE_USER")")" \
        || die "build on the base failed (see above)"

    say "fetching the image to $IMG_DIR"
    for f in "$TGZ_NAME" "$TGZ_NAME.md5" "${IMAGE}_$SHORT.manifest.txt" "${IMAGE}_$SHORT.pip-freeze.txt" \
             "${IMAGE}_$SHORT.smoke.txt" "docker_build_tractor_$SHORT.log"; do
        b_get "$BASE_WORK/$f" "$IMG_DIR" || die "fetch failed: $f"
    done
    want=$(tr -d '\r\n ' < "$IMG_DIR/$TGZ_NAME.md5")
    got=$(md5_local "$IMG_DIR/$TGZ_NAME")
    [ "$got" = "$want" ] || die "image tarball md5 mismatch after the fetch ($got vs $want)"
    if [ "$WITH_MQ" = 1 ]; then
        b_get "$BASE_WORK/eclipse-mosquitto_2.tar.gz" "$IMG_DIR" || die "fetch failed: eclipse-mosquitto_2.tar.gz"
        b_get "$BASE_WORK/eclipse-mosquitto_2.tar.gz.md5" "$IMG_DIR" || die "fetch failed: eclipse-mosquitto_2.tar.gz.md5"
        [ "$(md5_local "$IMG_DIR/eclipse-mosquitto_2.tar.gz")" = "$(tr -d '\r\n ' < "$IMG_DIR/eclipse-mosquitto_2.tar.gz.md5")" ] \
            || die "eclipse-mosquitto_2.tar.gz md5 mismatch after the fetch"
    fi
    {
        echo "subject=$SUBJECT"
        echo "context_md5=$CTX_MD5"
        echo "tool=bench_tools/build_tractor_image.sh"
    } >> "$IMG_DIR/${IMAGE}_$SHORT.manifest.txt"
    # The PC copy is verified: drop the big tarballs on the base (the image stays there).
    b_sh "rm -f $(sq "$BASE_WORK/$TGZ_NAME") $(sq "$BASE_WORK/eclipse-mosquitto_2.tar.gz")" || true
    say "PC copy verified (md5 $got)"
    TGZ_LOCAL=$IMG_DIR/$TGZ_NAME
else
    TGZ_LOCAL=$PUSH_ONLY
fi

# ------------------------------------------------------- load on the tractor --
if [ "$DO_TRACTOR" != 1 ]; then
    say "done; tractor untouched (--no-tractor). Load later with: bash build_tractor_image.sh --push-only $TGZ_LOCAL"
    exit 0
fi
state=$(MSYS_NO_PATHCONV=1 adb -s "$TRACTOR_SERIAL" get-state 2>/dev/null | tr -d '\r' || true)
[ "$state" = device ] || die "tractor $TRACTOR_SERIAL is not on adb (state '${state:-none}')"
t_sh "mkdir -p $(sq "$TRACTOR_WORK")"
say "pushing $TGZ_NAME to the tractor"
t_put "$TGZ_LOCAL" "$TRACTOR_WORK" || die "push to the tractor failed"
want=$(md5_local "$TGZ_LOCAL")
got=$(t_sh "md5sum $(sq "$TRACTOR_WORK/$TGZ_NAME")" | tr -d '\r' | cut -d' ' -f1)
[ "$got" = "$want" ] || die "md5 mismatch on the tractor ($got vs $want)"
MQ_REMOTE=""
if [ "$WITH_MQ" = 1 ] && [ -f "$IMG_DIR/eclipse-mosquitto_2.tar.gz" ]; then
    t_put "$IMG_DIR/eclipse-mosquitto_2.tar.gz" "$TRACTOR_WORK" || die "push of eclipse-mosquitto failed"
    MQ_REMOTE=$TRACTOR_WORK/eclipse-mosquitto_2.tar.gz
fi
t_put "$LOAD_SCRIPT" "$TRACTOR_WORK" || die "push of the load script failed"
t_sh "$(root_cmd "sh $(sq "$TRACTOR_WORK/load_tractor_remote.sh") $(sq "$TRACTOR_WORK/$TGZ_NAME") $KEEP $(sq "$MQ_REMOTE")")" \
    || die "docker load on the tractor failed (see above)"
say "tractor loaded $TGZ_NAME. No container was restarted and the radios were not touched."
