#!/usr/bin/env bash
# deploy_base.sh -- deploy one commit's base-station tree to the bench BASE X8,
# build lifetrac-v25:latest there, and (re)start ONLY the compose broker.
#
# Usage (Git Bash on the bench PC, or any bash with git + ssh/scp or adb):
#   bash deploy_base.sh [options] [<commit>]        # <commit> defaults to HEAD
#
# Options:
#   --ssh | --adb   transport to the base. Default: BASE_TRANSPORT from bench.env;
#                   else ssh when BASE_HOST is set; else adb when BASE_SERIAL is
#                   set; else ssh to 192.168.1.117.
#   --unlocked      build without base_station/requirements.lock.txt (pip
#                   resolves requirements.txt unpinned, as a plain build does).
#                   Default: --build-arg LOCKED=1, the flown wheel versions.
#   --no-build      deploy the tree only and keep the current image.
#   --no-broker     do not touch the broker container.
#   --no-clean      extract over the old tree. Default: base_station/ and
#                   firmware/ are replaced and an old deploy/ (no longer
#                   shipped) is removed, after a backup tarball of the old tree
#                   is saved on the base.
#   --dry-run       make the tarball and the board-side script locally, print the
#                   plan and stop. Never contacts a board.
#   -h | --help
#
# Settings: bench_tools/bench.env ($BENCH_ENV; see bench.env.example), loaded by
# lib/env_file.sh like every bench script -- CRLF is tolerated and a NON-EMPTY
# variable already in the environment wins over the file -- then the defaults
# below for anything still unset:
#   BASE_TRANSPORT  ssh | adb
#   BASE_HOST       192.168.1.117 (the bench base's DHCP lease)
#   BASE_SSH_USER   fio (old name BASE_USER)
#   BASE_SSH_KEY    ~/.ssh/lifetrac_base_ed25519
#   BASE_SERIAL     2D0A1209DABC240B (adb serial of the bench base)
#   TRACTOR_SERIAL  2E2C1209DABC240B (only used to refuse deploying to the tractor)
#   BENCH_SUDO_PW   fio, the LmP factory default; only used when passwordless
#                   sudo is not installed on the base
#   BASE_DEPLOY_DIR /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER (= /opt/lifetrac/...)
#   ARCHIVE_DIR     $HOME/Documents/LifeTrac-bench-archive (old name
#                   BENCH_ARCHIVE_DIR; outside git; the deploy tarballs are kept
#                   in its deploy/ folder)
#
# What it changes on the base: the tree in BASE_DEPLOY_DIR (backup first),
# DEPLOYED_FROM.txt, .env and secrets/ ONLY when missing (random PIN and fleet
# key, generated on the board, never printed), the lifetrac-v25:latest /
# :<sha> / :previous image tags, and the compose broker container. An old
# docker-compose.video-test.yml in the tree (the May video-test stack, no
# longer shipped) is renamed docker-compose.video-test.yml.RETIRED-<PC UTC
# date>, as the tractor's compose file was (BENCH_BOARDS.md, "Retired boot
# units"). Files go to /home/<user>/lifetrac_deploy/ and the build log to
# /home/<user>/docker_build_<sha>.log.
#
# RADIO SAFETY: nothing here opens /dev/ttymxc3 (the L072 radio UART) or starts
# a radio daemon. The board-side script refuses to run while a container maps
# /dev/ttymxc3, while a lifetrac-base* unit is anything but failed or inactive
# or has a job queued, or while that old video-test file is there and one of
# its containers is in any state but exited. It never starts lora_bridge (see
# the comment at the broker step) and never runs docker compose on the
# video-test file. See DEPLOY.md.
set -euo pipefail

BT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DC_REL=LifeTrac-v25/DESIGN-CONTROLLER

# ---------------------------------------------------------------- settings --
BENCH_ENV=${BENCH_ENV:-${BENCH_ENV_FILE:-$BT/bench.env}}
# shellcheck source=lib/env_file.sh
. "$BT/lib/env_file.sh"
if [ -f "$BENCH_ENV" ]; then
    # CR stripped; the environment wins; nothing is exported (the password stays
    # out of every child process's environment)
    bench_load_env_file "$BENCH_ENV" || exit 2
else
    echo "deploy_base: WARN: no $BENCH_ENV -- the original bench's address and serials apply unless set in the environment (cp bench.env.example bench.env)" >&2
fi

COMMIT=HEAD
OPT_TRANSPORT=""
LOCKED=1
DO_BUILD=1
DO_BROKER=1
DO_CLEAN=1
DRY_RUN=0

usage() { sed -n '2,/^set -euo/p' "${BASH_SOURCE[0]}" | sed '$d; s/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case $1 in
        --ssh) OPT_TRANSPORT=ssh ;;
        --adb) OPT_TRANSPORT=adb ;;
        --unlocked) LOCKED=0 ;;
        --no-build) DO_BUILD=0 ;;
        --no-broker) DO_BROKER=0 ;;
        --no-clean) DO_CLEAN=0 ;;
        --dry-run) DRY_RUN=1 ;;
        -h|--help) usage; exit 0 ;;
        -*) echo "deploy_base: unknown option $1 (see --help)" >&2; exit 2 ;;
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
case $BASE_TRANSPORT in ssh|adb) ;; *) echo "deploy_base: BASE_TRANSPORT must be ssh or adb" >&2; exit 2 ;; esac
BASE_HOST=${BASE_HOST:-192.168.1.117}
BASE_USER=${BASE_SSH_USER:-${BASE_USER:-fio}}
BASE_SSH_KEY=${BASE_SSH_KEY:-$HOME/.ssh/lifetrac_base_ed25519}
BASE_SERIAL=${BASE_SERIAL:-2D0A1209DABC240B}
TRACTOR_SERIAL=${TRACTOR_SERIAL:-2E2C1209DABC240B}
BENCH_SUDO_PW=${BENCH_SUDO_PW-fio}
BASE_DEPLOY_DIR=${BASE_DEPLOY_DIR:-/var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER}
BENCH_ARCHIVE_DIR=${ARCHIVE_DIR:-${BENCH_ARCHIVE_DIR:-$HOME/Documents/LifeTrac-bench-archive}}
COMPOSE_PROJECT=design-controller   # the running broker is design-controller-mosquitto-1
REMOTE_WORK=/home/$BASE_USER/lifetrac_deploy

# ----------------------------------------------------------------- helpers --
say() { printf '[deploy_base %s] %s\n' "$(date -u +%H:%M:%SZ)" "$*"; }
die() { printf '[deploy_base] ERROR: %s\n' "$*" >&2; exit 1; }
sq() { local s=${1//\'/\'\\\'\'}; printf "'%s'" "$s"; }          # shell-quote for the remote sh
winpath() { if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else printf '%s' "$1"; fi; }
# tar and scp read "C:/..." as host "C"; Git Bash paths must look like /c/...
unixpath() { if command -v cygpath >/dev/null 2>&1; then cygpath -u "$1"; else printf '%s' "$1"; fi; }
md5_local() { if command -v md5sum >/dev/null 2>&1; then md5sum "$1" | cut -d' ' -f1; else md5 -q "$1"; fi; }
BENCH_ARCHIVE_DIR=$(unixpath "$BENCH_ARCHIVE_DIR")
BASE_SSH_KEY=$(unixpath "$BASE_SSH_KEY")

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new -o ServerAliveInterval=15)
[ -f "$BASE_SSH_KEY" ] && SSH_OPTS=(-i "$BASE_SSH_KEY" "${SSH_OPTS[@]}")

# adb shell does not reliably return the remote exit code on every adbd, so the
# command prints a marker with its status and adb_sh returns that.
adb_sh() {
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

b_sh() {      # run a command line on the base as BASE_USER
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
# Run a command line as root on the base. The command must not read stdin:
# when a password is needed it arrives on sudo's stdin (the "sudo eats stdin"
# trap: never pipe data into a `sudo -S` command; copy the file first).
b_root() {
    local pw; pw=$(sq "$BENCH_SUDO_PW")
    b_sh "if sudo -n true 2>/dev/null; then sudo -n $1; elif [ -n $pw ]; then printf '%s\n' $pw | sudo -S -p '' $1; else echo 'sudo needs a password: install the bench sudoers drop-in (BENCH_SETUP.md) or set BENCH_SUDO_PW' >&2; false; fi"
}

# -------------------------------------------------------- local: the tarball --
REPO=$(git -C "$BT" rev-parse --show-toplevel)
SHA=$(git -C "$REPO" rev-parse --verify --quiet "${COMMIT}^{commit}") || die "not a commit: $COMMIT"
SHORT=${SHA:0:8}
SUBJECT=$(git -C "$REPO" log -1 --format=%s "$SHA")
git -C "$REPO" cat-file -e "$SHA:$DC_REL/Dockerfile" 2>/dev/null || die "$SHORT has no $DC_REL/Dockerfile"

# The image's build context needs base_station/ and firmware/ (the Dockerfile
# COPYs both); the broker step needs docker-compose.yml and
# base_station/mosquitto.conf. Nothing else is shipped: not
# docker-compose.video-test.yml (the May video-test stack, whose image_rx
# opens /dev/ttymxc3) and not deploy/ (its retired unit files), whatever the
# commit. Docs, bench-evidence, hil/ and tools/ stay home.
PATHS=()
while IFS= read -r p; do
    case $p in base_station|firmware|Dockerfile|docker-compose.yml) PATHS+=("$p") ;; esac
done < <(git -C "$REPO" ls-tree --name-only "$SHA:$DC_REL")
for need in base_station firmware Dockerfile docker-compose.yml; do
    found=0
    for p in "${PATHS[@]}"; do [ "$p" != "$need" ] || found=1; done
    [ "$found" = 1 ] || die "$SHORT: $DC_REL/$need missing"
done
HAS_LOCK=0
git -C "$REPO" cat-file -e "$SHA:$DC_REL/base_station/requirements.lock.txt" 2>/dev/null && HAS_LOCK=1

case $(cd "$BENCH_ARCHIVE_DIR" 2>/dev/null && pwd -P || echo "$BENCH_ARCHIVE_DIR")/ in
    "$(cd "$REPO" && pwd -P)"/*) die "BENCH_ARCHIVE_DIR must be outside the git repo" ;;
esac
STAGE=$BENCH_ARCHIVE_DIR/deploy
mkdir -p "$STAGE"
TGZ=$STAGE/dc_deploy_$SHORT.tar.gz
STAMP=$STAGE/DEPLOYED_FROM_$SHORT.txt
RSCRIPT=$STAGE/deploy_base_remote.sh

# CRLF trap: with core.autocrlf=true (Git for Windows' default) or a text=auto
# attribute and core.eol=crlf, git archive writes CRLF, and every shell script
# and config on the board then breaks. Force LF.
say "archive $SHORT ($SUBJECT): ${PATHS[*]}"
git -C "$REPO" -c core.autocrlf=false -c core.eol=lf archive --format=tar.gz -o "$TGZ" "$SHA:$DC_REL" -- "${PATHS[@]}"
# Check: the text files in the tarball must hold no more CR bytes than the same
# blobs in git (a few committed logs carry CRs of their own). More CRs means an
# eol conversion slipped in. (Needs GNU tar's --wildcards, else it is skipped.)
TEXT_RE='(\.(sh|py|yml|conf|cfg|service|txt)|Dockerfile)$'
CR_TAR=$(set +o pipefail; { tar -xzOf "$TGZ" --wildcards '*.sh' '*.py' '*.yml' '*.conf' '*.cfg' \
    '*.service' '*.txt' '*Dockerfile' 2>/dev/null || true; } | tr -cd '\r' | wc -c)
CR_GIT=$(set +o pipefail; tar -tzf "$TGZ" | grep -E "$TEXT_RE" | sed "s|^|$SHA:$DC_REL/|" \
    | git -C "$REPO" cat-file --batch | tr -cd '\r' | wc -c)
[ "${CR_TAR// /}" -le "${CR_GIT// /}" ] \
    || die "the tarball has CRLF the commit does not have; check .gitattributes / core.eol (DEPLOY.md, CRLF trap)"
TGZ_MD5=$(md5_local "$TGZ")

{
    echo "$SHA"
    echo "subject=$SUBJECT"
    echo "deployed_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "paths=${PATHS[*]}"
    echo "tarball=dc_deploy_$SHORT.tar.gz md5=$TGZ_MD5"
    echo "tool=bench_tools/deploy_base.sh"
} > "$STAMP"

# -------------------------------------------------- the board-side script --
# Plain POSIX sh, runs as root on the base. Arguments, not expansion, carry
# every value in, so the quoted heredoc below is copied verbatim.
cat > "$RSCRIPT" <<'REMOTE'
#!/bin/sh
# Board side of deploy_base.sh. Runs as root on the bench BASE X8.
# Never opens /dev/ttymxc3 and never starts lora_bridge.
set -eu
TGZ=$1 STAMP=$2 SHORT=$3 SHA=$4 DEST=$5 OWNER=$6 DO_CLEAN=$7 DO_BUILD=$8 LOCKED=$9
shift 9
DO_BROKER=$1 PROJECT=$2 WORK=$3 DAY=$4
VT=docker-compose.video-test.yml   # the May video-test stack: no longer shipped, an old copy is renamed

say() { echo "[base $(date -u +%H:%M:%SZ)] $*"; }
die() { echo "[base] ERROR: $*" >&2; exit 1; }

radio_containers() {   # running containers that map the L072 radio UART
    ids=$(docker ps -q)
    [ -n "$ids" ] || return 0
    docker inspect -f '{{.Name}} {{range .HostConfig.Devices}}{{.PathOnHost}} {{end}}' $ids \
        | grep ttymxc3 | cut -d' ' -f1 | tr -d / | tr '\n' ' ' | sed 's/ *$//'
}

mqtt_probe() {         # 0 = a broker on 127.0.0.1:1883 accepted an MQTT CONNECT
    if command -v python3 >/dev/null 2>&1; then
        python3 - <<'PY'
import socket, sys
try:
    s = socket.create_connection(("127.0.0.1", 1883), timeout=3)
    cid = b"lifetrac-deploy-probe"
    body = b"\x00\x04MQTT\x04\x02\x00\x0a" + len(cid).to_bytes(2, "big") + cid
    s.sendall(b"\x10" + bytes([len(body)]) + body)
    r = s.recv(4)
    s.sendall(b"\xe0\x00")
    s.close()
except OSError as e:
    print("mqtt probe: %s" % e)
    sys.exit(1)
if len(r) == 4 and r[0] == 0x20 and r[3] == 0:
    print("mqtt probe: CONNACK accepted on 127.0.0.1:1883")
    sys.exit(0)
print("mqtt probe: unexpected reply %r" % (r,))
sys.exit(2)
PY
    else
        docker run --rm --network host eclipse-mosquitto:2 \
            mosquitto_sub -h 127.0.0.1 -p 1883 -t '$SYS/broker/version' -C 1 -W 5
    fi
}

# ---- preconditions (read-only)
case $DEST in */DESIGN-CONTROLLER) ;; *) die "deploy dir must end in /DESIGN-CONTROLLER: $DEST" ;; esac
[ -f "$TGZ" ] || die "missing $TGZ"
command -v docker >/dev/null 2>&1 || die "docker not found"
if [ -f /etc/systemd/system/lifetrac-camera.service ]; then
    die "this board has lifetrac-camera.service: it looks like the TRACTOR, not the base"
fi
st=$(systemctl is-enabled compose-apps-early-start-recovery.service 2>/dev/null || true)
case $st in
    masked*|"") ;;
    *) die "compose-apps-early-start-recovery.service is '$st', not masked. On the LmP 934 image it runs 'systemctl stop docker; rm -rf /var/lib/docker' every 60 s. Mask it and compose-apps-early-start.service first (BENCH_SETUP.md, provision_bench_board.sh)." ;;
esac
st=$(systemctl is-enabled compose-apps-early-start.service 2>/dev/null || true)
case $st in masked*|"") ;; *) say "WARNING: compose-apps-early-start.service is '$st' (the bench masks it)" ;; esac
# systemd runs a unit's ExecStop only after a successful start, so a failed or
# still-starting unit needs no stop. An ACTIVE unit does, and the ExecStop of
# lifetrac-base.service is a bare `docker compose down`: it also removes the
# broker container, which step 4 recreates. Only failed or inactive passes,
# and only with no job queued: a deactivating unit is running that ExecStop
# right now, and early in a boot an enabled unit reads inactive while its
# start job still waits.
for u in lifetrac-base.service lifetrac-base-compose.service; do
    en=$(systemctl is-enabled "$u" 2>/dev/null || true)
    ac=$(systemctl is-active "$u" 2>/dev/null || true)
    job=$(systemctl show -p Job --value "$u" 2>/dev/null || echo unknown)
    case $ac in
        inactive|failed) ;;
        activating|reloading|deactivating)
            die "$u is $ac: a start or stop job is still running. Wait until it has failed or is inactive, then deploy again. Do not stop it and do not interrupt a build." ;;
        active)
            if [ "$u" = lifetrac-base.service ]; then
                svc="lora_bridge and web_ui"
                note="That stop runs a bare 'docker compose down', which also removes the broker container $PROJECT-mosquitto-1 (its volume stays); this deploy recreates it unless --no-broker is given."
            else
                svc="image_rx and web_ui"
                note="That stop removes only the containers and network of its own compose project, lifetrac-vtest."
            fi
            die "$u is active: its start succeeded, so its compose services were started and may be running ($svc; /dev/ttymxc3 is mapped). Stop those containers by name (docker stop <container>), then 'systemctl stop $u' and deploy again. $note Retire the unit afterwards (BENCH_BOARDS.md)." ;;
        *)
            die "$u: 'systemctl is-active' returned '$ac' (expected inactive or failed). Look at the unit by hand before deploying." ;;
    esac
    case $job in
        "") ;;
        unknown)
            die "$u: could not read whether systemd has a job queued for it ('systemctl show -p Job --value $u' failed)." ;;
        *)
            die "$u reads $ac, but systemd has a job queued for it (job $job): the base is still booting, or the unit has just been started or stopped. Wait until 'systemctl list-jobs' no longer lists it, then deploy again." ;;
    esac
    case $en in
        enabled*) say "WARNING: $u is $en ($ac). If it ever succeeds at boot it starts compose services that map /dev/ttymxc3, without a GO. Retire it: BENCH_BOARDS.md has the rules, and 'provision_bench_board.sh base' (with --ssh when the base is not on adb) does the disable and the move. A failed or inactive unit is disabled WITHOUT --now and needs no stop." ;;
    esac
done
busy=$(radio_containers)
[ -z "$busy" ] || die "running containers map /dev/ttymxc3 (a leg in progress?): $busy. Deploy only with the radio daemons stopped."
# Step 1 renames an old $VT in the tree, as the tractor's compose file was on
# 2026-10-12, so that no `docker compose up` there finds it. It does so only
# while every container made from that file is exited, as the tractor's were
# then. A running or created one (a radio container that is up, or the
# remains of an `up` that failed) needs the manual cleanup first:
# BENCH_BOARDS.md, "Retired boot units". The containers are found by their
# compose labels (project lifetrac-vtest, or $VT among the config files),
# without docker compose; a docker ps that fails stops the deploy too.
case $DAY in [0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]) ;; *) die "bad date argument '$DAY' (expected YYYY-MM-DD)" ;; esac
if [ -e "$DEST/$VT" ] || [ -L "$DEST/$VT" ]; then
    vt_ps=$(docker ps -a --format '{{.Names}}|{{.State}}|{{.Label "com.docker.compose.project"}}|{{.Label "com.docker.compose.project.config_files"}}') \
        || die "'docker ps -a' failed, so no container of $DEST/$VT could be ruled out. Nothing was changed."
    vt_ps=$(printf '%s\n' "$vt_ps" | awk -F'|' -v f="$VT" '$3 == "lifetrac-vtest" || index($4, f) {print $1 " (" $2 ")"}')
    vt_live=$(printf '%s\n' "$vt_ps" | grep -v ' (exited)$' | tr '\n' ' ' | sed 's/ *$//' || true)
    [ -z "$vt_live" ] || die "$DEST/$VT is still there, and containers made from it are not exited: $vt_live. Nothing was changed. Look at them first: if one is running, stop it by name (docker stop <container>; rule 3 in BENCH_BOARDS.md, \"Retired boot units\"); remove what is left as BENCH_BOARDS.md describes (no -f, no -v), then deploy again. Never bring that file up."
    [ -z "$vt_ps" ] || say "exited containers made from $VT are left (not removed here; BENCH_BOARDS.md): $(printf '%s\n' "$vt_ps" | tr '\n' ' ' | sed 's/ *$//')"
fi
free_kb=$(df -Pk "$WORK" | awk 'NR==2 {print $4}')
[ "$DO_BUILD" != 1 ] || [ "$free_kb" -gt 1500000 ] || die "less than 1.5 GB free under $WORK"

# ---- 1. disarm the old video-test file, back up the old tree, then replace it
mkdir -p "$DEST"
old_sha=$(head -n 1 "$DEST/DEPLOYED_FROM.txt" 2>/dev/null | cut -c1-8 || true)
old_conf_md5=$(md5sum "$DEST/base_station/mosquitto.conf" 2>/dev/null | cut -d' ' -f1 || true)
cd "$DEST"
# A rename, nothing is deleted; never onto an existing name. It comes before
# the backup, so no backup holds the file under its old name and restoring
# one cannot re-arm it.
if [ -e "$VT" ] || [ -L "$VT" ]; then
    n=1; retired=$VT.RETIRED-$DAY
    while [ -e "$retired" ] || [ -L "$retired" ]; do
        n=$((n + 1)); retired=$VT.RETIRED-$DAY-$n
    done
    mv "$VT" "$retired" || die "could not rename $DEST/$VT; nothing was changed"
    say "renamed $DEST/$VT to $retired, so no docker compose up here finds it"
fi
set --
for p in base_station firmware deploy Dockerfile DEPLOYED_FROM.txt docker-compose*.yml; do
    if [ -e "$p" ]; then set -- "$@" "$p"; fi
done
if [ $# -gt 0 ]; then
    backup="$WORK/prev_tree_${old_sha:-unknown}_$(date -u +%Y%m%dT%H%M%SZ).tgz"
    tar -czf "$backup" "$@"
    say "old tree (${old_sha:-no DEPLOYED_FROM}) saved to $backup"
fi
if [ "$DO_CLEAN" = 1 ]; then
    # deploy/ is no longer shipped: an old copy goes, the backup keeps it
    rm -rf "$DEST/base_station" "$DEST/firmware" "$DEST/deploy"
fi
tar -xzf "$TGZ" -C "$DEST"
cp "$STAMP" "$DEST/DEPLOYED_FROM.txt"
chown -R "$OWNER:$OWNER" "$DEST"
say "tree at $DEST is now $SHORT (clean=$DO_CLEAN)"

# ---- 2. .env and secrets/, only when missing; values never printed
umask 077
if [ ! -f .env ]; then
    cat > .env <<'ENV'
# Written by bench_tools/deploy_base.sh. web_ui reads the PIN from
# secrets/lifetrac_pin first, so LIFETRAC_PIN stays empty here.
LIFETRAC_PIN=
# Read only by lora_bridge and the video-test image_rx, which the bench
# deploy never starts.
LIFETRAC_LORA_DEVICE=/dev/ttymxc3
LIFETRAC_TRUSTED_PROXIES=
ENV
    say "created .env"
fi
mkdir -p secrets
if [ ! -s secrets/lifetrac_fleet_key ]; then
    head -c 16 /dev/urandom > secrets/lifetrac_fleet_key
    say "created secrets/lifetrac_fleet_key (16 random bytes, a bench-only key; not printed)"
fi
if [ ! -s secrets/lifetrac_pin ]; then
    n=$(od -An -N4 -tu4 /dev/urandom | tr -d ' \n')
    case $n in ''|*[!0-9]*) die "could not read /dev/urandom for the PIN" ;; esac
    printf '%d' $((100000 + n % 900000)) > secrets/lifetrac_pin
    say "created secrets/lifetrac_pin (6 random digits; read it on the base: sudo cat $DEST/secrets/lifetrac_pin)"
fi
chmod 700 secrets
chmod 600 secrets/lifetrac_fleet_key secrets/lifetrac_pin .env
chown -R "$OWNER:$OWNER" secrets .env
umask 022

# ---- 3. build the image (no radio involved)
if [ "$DO_BUILD" = 1 ]; then
    build_args=""
    locked_eff=0
    if [ "$LOCKED" = 1 ]; then
        if [ -f base_station/requirements.lock.txt ] && grep -q '^ARG LOCKED' Dockerfile; then
            build_args="--build-arg LOCKED=1"
            locked_eff=1
        else
            say "WARNING: $SHORT has no lock-file support; building unlocked"
        fi
    fi
    old_id=$(docker image inspect -f '{{.Id}}' lifetrac-v25:latest 2>/dev/null || true)
    log=/home/$OWNER/docker_build_$SHORT.log
    say "docker build -t lifetrac-v25:latest -t lifetrac-v25:$SHORT $build_args . (log $log)"
    # shellcheck disable=SC2086
    if ! docker build $build_args --label "org.lifetrac.source_sha=$SHA" \
            -t lifetrac-v25:latest -t "lifetrac-v25:$SHORT" . > "$log" 2>&1; then
        tail -n 40 "$log"
        die "docker build failed (full log: $log)"
    fi
    chown "$OWNER:$OWNER" "$log"
    new_id=$(docker image inspect -f '{{.Id}}' lifetrac-v25:latest)
    if [ -n "$old_id" ] && [ "$old_id" != "$new_id" ]; then
        docker tag "$old_id" lifetrac-v25:previous
        say "the image it replaced is kept as lifetrac-v25:previous ($(echo "$old_id" | cut -c8-19))"
    fi
    freeze=/home/$OWNER/pip_freeze_lifetrac-v25_$SHORT.txt
    docker run --rm --network none --entrypoint python3 lifetrac-v25:latest -m pip freeze > "$freeze" 2>&1 \
        || say "WARNING: pip freeze in the image failed (see $freeze)"
    chown "$OWNER:$OWNER" "$freeze"
    docker run --rm --network none --entrypoint python3 lifetrac-v25:latest -c \
        "import fastapi, uvicorn, paho.mqtt.client, cryptography, PIL, serial, jinja2; print('image deps import OK')" \
        || die "the new image cannot import its dependencies"
    {
        echo "image=lifetrac-v25:latest $new_id"
        echo "build_locked=$locked_eff build_log=$log pip_freeze=$freeze"
    } >> DEPLOYED_FROM.txt
    say "built $(echo "$new_id" | cut -c8-19) (locked=$locked_eff); wheels: $freeze"
fi

# ---- 4. the broker, and ONLY the broker
if [ "$DO_BROKER" = 1 ]; then
    docker compose version > /dev/null 2>&1 || die "the docker compose plugin is missing"
    docker compose -p "$PROJECT" -f "$DEST/docker-compose.yml" config --services | grep -qx mosquitto \
        || die "docker-compose.yml has no 'mosquitto' service"
    recreate=""
    new_conf_md5=$(md5sum base_station/mosquitto.conf | cut -d' ' -f1)
    [ "$new_conf_md5" = "$old_conf_md5" ] || recreate=--force-recreate
    # NEVER run `docker compose up` without a service name here, and never name
    # lora_bridge (or web_ui / audit_tail, which depend on nothing radio but are
    # not needed by the bench). lora_bridge maps ${LIFETRAC_LORA_DEVICE}, which is
    # /dev/ttymxc3 on this bench: the L072 radio UART, /dev/ttyACM0 inside the
    # container. As deployed it is told to open /dev/ttymxc3 there (the single
    # `$` at docker-compose.yml:54), so it would probably crash-loop; with that
    # `$` fixed it would drive the L072 with KISS framing at 115200 baud, which
    # the Method-G firmware does not speak. Never start it either way
    # (DEPLOY.md). Radio daemons are started only by the leg scripts, on an
    # explicit GO.
    # --no-deps keeps compose from starting anything mosquitto might depend on.
    # shellcheck disable=SC2086
    docker compose -p "$PROJECT" -f "$DEST/docker-compose.yml" up -d --no-deps $recreate mosquitto
    i=0
    until mqtt_probe; do
        i=$((i + 1))
        [ "$i" -lt 20 ] || die "nothing answers MQTT on 127.0.0.1:1883 after 20 s (docker compose -p $PROJECT logs mosquitto)"
        sleep 1
    done
    holder=$(docker ps --filter publish=1883 --format '{{.Names}}' | tr '\n' ' ')
    case " $holder " in
        *" $PROJECT-mosquitto-1 "*) say "broker $PROJECT-mosquitto-1 answers on 127.0.0.1:1883" ;;
        *) say "WARNING: 1883 is published by '$holder', not $PROJECT-mosquitto-1" ;;
    esac
    others=$(docker compose -p "$PROJECT" -f "$DEST/docker-compose.yml" ps --services --filter status=running | grep -vx mosquitto | tr '\n' ' ' || true)
    [ -z "$others" ] || say "WARNING: other services of $PROJECT are running (not started by this script): $others"
fi

busy=$(radio_containers)
[ -z "$busy" ] || say "WARNING: containers now map /dev/ttymxc3: $busy (this script started none of them)"
say "DEPLOYED_FROM.txt:"
sed 's/^/    /' "$DEST/DEPLOYED_FROM.txt"
say "done"
REMOTE
tr -d '\r' < "$RSCRIPT" > "$RSCRIPT.tmp" && mv "$RSCRIPT.tmp" "$RSCRIPT"
sh -n "$RSCRIPT" || die "generated board script has a syntax error"
tr -d '\r' < "$STAMP" > "$STAMP.tmp" && mv "$STAMP.tmp" "$STAMP"

DAY=$(date -u +%Y-%m-%d)   # the PC's date names the renamed video-test file (a board clock may not be wall time)
R_ARGS="$(sq "$REMOTE_WORK/dc_deploy_$SHORT.tar.gz") $(sq "$REMOTE_WORK/DEPLOYED_FROM_$SHORT.txt") $SHORT $SHA $(sq "$BASE_DEPLOY_DIR") $(sq "$BASE_USER") $DO_CLEAN $DO_BUILD $LOCKED $DO_BROKER $COMPOSE_PROJECT $(sq "$REMOTE_WORK") $DAY"

if [ "$BASE_TRANSPORT" = ssh ]; then TARGET="$BASE_USER@$BASE_HOST (ssh)"; else TARGET="adb $BASE_SERIAL"; fi
say "commit   $SHA"
say "target   $TARGET -> $BASE_DEPLOY_DIR"
say "tarball  $TGZ ($(wc -c < "$TGZ") B, md5 $TGZ_MD5)"
say "build    $([ "$DO_BUILD" = 1 ] && echo "yes, locked=$LOCKED (commit has lock file: $HAS_LOCK)" || echo no)"
say "broker   $([ "$DO_BROKER" = 1 ] && echo "docker compose -p $COMPOSE_PROJECT up -d --no-deps mosquitto" || echo untouched)"
say "disarm   an old docker-compose.video-test.yml in the tree is renamed docker-compose.video-test.yml.RETIRED-$DAY (refused while a container made from it is not exited)"
if [ "$DRY_RUN" = 1 ]; then
    say "dry run: board script $RSCRIPT"
    say "dry run: would copy the tarball, $STAMP and the script to $REMOTE_WORK, then run as root:"
    say "         sh $REMOTE_WORK/deploy_base_remote.sh $R_ARGS"
    exit 0
fi

# ------------------------------------------------------------ on the base --
if [ "$BASE_TRANSPORT" = adb ] && [ "$BASE_SERIAL" = "$TRACTOR_SERIAL" ]; then
    die "BASE_SERIAL equals TRACTOR_SERIAL"
fi
b_sh "true" > /dev/null || die "cannot reach the base ($TARGET)"
b_sh "mkdir -p $(sq "$REMOTE_WORK")"
for f in "$TGZ" "$STAMP" "$RSCRIPT"; do
    b_put "$f" "$REMOTE_WORK" || die "copy failed: $f"
done
remote_md5=$(b_sh "md5sum $(sq "$REMOTE_WORK/dc_deploy_$SHORT.tar.gz")" | tr -d '\r' | cut -d' ' -f1)
[ "$remote_md5" = "$TGZ_MD5" ] || die "tarball md5 mismatch on the base ($remote_md5 vs $TGZ_MD5)"
say "copied to $REMOTE_WORK (md5 OK); running the board-side script as root"
b_root "sh $(sq "$REMOTE_WORK/deploy_base_remote.sh") $R_ARGS" || die "board-side deploy failed (see above)"
say "base deployed from $SHORT. The radios were not touched."
