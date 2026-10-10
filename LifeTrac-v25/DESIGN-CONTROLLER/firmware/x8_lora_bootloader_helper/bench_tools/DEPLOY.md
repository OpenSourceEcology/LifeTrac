# Deploying code and images to the bench boards

Two scripts in this folder put a commit's software on the bench X8 boards:

| script | what it does | board(s) |
|---|---|---|
| [`deploy_base.sh`](deploy_base.sh) | copies the base-station tree of one commit to the base, builds `lifetrac-v25:latest` there, and (re)starts **only** the compose broker | base |
| [`build_tractor_image.sh`](build_tractor_image.sh) | builds `lifetrac-tractor-x8` from one commit **on the base**, keeps a copy on the PC, and loads it on the offline tractor over adb | base (build), tractor (load) |

Neither script touches the radios. Neither starts a radio daemon, and neither
counts as a radio GO. See [Radio safety](#radio-safety).

Both run from Git Bash on the bench PC (or any bash with `git`, `ssh`/`scp`
and `adb`). Both accept `--dry-run`, which builds everything locally, prints
the plan and contacts no board, and `--help`.

Related: [BENCH_QUICKSTART.md](BENCH_QUICKSTART.md) (the order of everything),
[BENCH_SETUP.md](BENCH_SETUP.md) (OS image and board provisioning, which comes
**before** this page), [PC_SETUP.md](PC_SETUP.md),
[BENCH_BOARDS.md](BENCH_BOARDS.md) (what is on each board now),
[BENCH_RUNBOOK.md](BENCH_RUNBOOK.md), [legs/README.md](legs/README.md).

## When to use them

- **`deploy_base.sh`**
  - after `main` (or the branch under test) changed anything the base runs:
    `base_station/`, `firmware/common/`, `Dockerfile`, `docker-compose.yml`;
  - after a base reflash and [provisioning](BENCH_SETUP.md), to install the
    tree, the image and the broker for the first time;
  - to roll the base back: deploy an older commit.

  The bench radio daemons (for example `rx_smoke` on the base) run in
  `lifetrac-v25:latest` but take their code from `/tmp/lifetrac_strict`,
  staged by the leg scripts ([legs/README.md](legs/README.md)). So the image
  supplies the Python dependencies, the staged tree supplies the code under
  test, and the compose broker is the base's MQTT bus.
- **`build_tractor_image.sh`**
  - when `firmware/tractor_x8/` changed in a way a leg needs inside the image:
    `requirements.txt`, the `Dockerfile`, or code that runs from the image and
    not from the staged tree;
  - after a tractor reflash (its images are gone and it cannot pull);
  - with `--push-only <tarball>` to put an archived image back, for example a
    rollback to the flown image.

  This automates route (a) of [RS13_VECTOR_LEG.md](RS13_VECTOR_LEG.md) step 0.
  Steps 2 and 3 of [README-DEPLOY.md](../../tractor_x8/README-DEPLOY.md) (the
  production camera unit) are **not** part of the bench.

## Settings

Both scripts read `bench_tools/bench.env` when it exists (copy
[`bench.env.example`](bench.env.example); `BENCH_ENV=<path>` points at another
file); otherwise the environment, then these defaults:

| variable | default | used for |
|---|---|---|
| `BASE_TRANSPORT` | `ssh` if `BASE_HOST` is set, else `adb` if `BASE_SERIAL` is set, else `ssh` | how to reach the base (`--ssh` / `--adb` override) |
| `BASE_HOST` | `192.168.1.117` | the base's DHCP lease on the bench; check it after a router change |
| `BASE_USER` / `TRACTOR_USER` | `fio` | login and file owner on the base / the tractor |
| `BASE_SSH_KEY` | `~/.ssh/lifetrac_base_ed25519` | ssh/scp key (omitted if the file is missing) |
| `BASE_SERIAL` | `2D0A1209DABC240B` | adb serial of the base |
| `TRACTOR_SERIAL` | `2E2C1209DABC240B` | adb serial of the tractor (the tractor is adb-only) |
| `BENCH_SUDO_PW` | `fio` (LmP factory default) | used only when the passwordless sudoers drop-in is missing |
| `BASE_DEPLOY_DIR` | `/var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER` | the base tree (`/opt` is a symlink to `/var/rootdirs/opt` on LmP) |
| `BENCH_ARCHIVE_DIR` | `$HOME/LifeTrac-bench-archive` | PC folder **outside git**: `deploy/` keeps every deploy tarball, `images/` every tractor image. On the original bench PC the private archive is `C:\Users\dorkm\Documents\LifeTrac-bench-archive`. |

The base has been reachable only over ssh since the 2026-10-10 power cycle (it
did not enumerate on USB), so ssh is the default.

## `deploy_base.sh [options] [<commit>]`

```sh
cd LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/bench_tools
bash deploy_base.sh --dry-run            # check the plan
bash deploy_base.sh                      # deploy HEAD
bash deploy_base.sh origin/main          # deploy another commit
```

Options: `--ssh | --adb`, `--unlocked`, `--no-build`, `--no-broker`,
`--no-clean`, `--dry-run`.

On the PC:
1. `git -c core.autocrlf=false -c core.eol=lf archive` the commit's
   `base_station/ firmware/ Dockerfile docker-compose*.yml deploy/` (the
   Dockerfile COPYs `base_station/` and `firmware/`; compose needs the yml files
   and `base_station/mosquitto.conf`). Docs, `bench-evidence/`, `hil/` and
   `tools/` stay on the PC. The tarball (about 2 MB) is kept as
   `$BENCH_ARCHIVE_DIR/deploy/dc_deploy_<sha8>.tar.gz`.
2. It refuses a tarball whose text files hold more CR bytes than the commit's
   blobs (the [CRLF trap](#traps)).
3. It writes the board-side script and `DEPLOYED_FROM_<sha8>.txt`, copies the
   three files to `/home/fio/lifetrac_deploy/`, checks the md5, and runs the
   script as root.

On the base, the script runs as root and:
1. **refuses** to run if `compose-apps-early-start-recovery.service` is not
   masked, if a `lifetrac-base*` unit is active, if any running container maps
   `/dev/ttymxc3`, or if the board looks like the tractor. It warns when a
   `lifetrac-base*` unit is enabled.
2. saves the old tree as `/home/fio/lifetrac_deploy/prev_tree_<oldsha>_<time>.tgz`,
   deletes `base_station/ firmware/ deploy/` (skip with `--no-clean`; stale files
   would otherwise end up in the image), and extracts the new tree.
3. writes `DEPLOYED_FROM.txt`: the full SHA on line 1 (as before), then the
   subject, time, paths, tarball md5, and after the build the image id.
4. creates `.env` and `secrets/` **only when missing**: a 16-byte random fleet
   key and a 6-digit random PIN, made on the board, mode 600, never printed.
   Read the PIN on the base with `sudo cat <BASE_DEPLOY_DIR>/secrets/lifetrac_pin`.
   An existing `.env` or `secrets/` is never changed.
5. `docker build --build-arg LOCKED=1 -t lifetrac-v25:latest -t lifetrac-v25:<sha8> .`,
   log in `/home/fio/docker_build_<sha8>.log`. The image it replaces is tagged
   `lifetrac-v25:previous`. The image's `pip freeze` goes to
   `/home/fio/pip_freeze_lifetrac-v25_<sha8>.txt`, and a dependency import
   check runs in a container with `--network none`.
6. `docker compose -p design-controller up -d --no-deps mosquitto` (with
   `--force-recreate` when `mosquitto.conf` changed), then waits until an MQTT
   CONNECT on `127.0.0.1:1883` is accepted, and names the container that
   publishes the port (`design-controller-mosquitto-1`).

The broker keeps its retained topics across a recreate (`persistence true`,
volume `design-controller_mosquitto_data`). Deploying does not clear them: the
leg prep's `clear_retained.py` does (BENCH_RUNBOOK prep 5).

## `build_tractor_image.sh [options] [<commit>]`

```sh
bash build_tractor_image.sh --dry-run
bash build_tractor_image.sh                       # build HEAD, archive, load on the tractor
bash build_tractor_image.sh --no-tractor          # build + archive only
bash build_tractor_image.sh --push-only ~/LifeTrac-bench-archive/images/lifetrac-tractor-x8_<sha8>.tar.gz
```

Options: `--ssh | --adb` (base transport), `--unlocked`, `--with-mosquitto`,
`--no-tractor`, `--push-only FILE`, `--keep-tarball`, `--dry-run`.

1. PC: `git archive` (LF forced) of `firmware/tractor_x8` alone. The
   Dockerfile does `COPY . /app`, and the encoder's codec is an in-tree mirror,
   so nothing else is needed.
2. Base, as root, in `/home/fio/lifetrac_build/`:
   `docker build --build-arg LOCKED=1 -t lifetrac-tractor-x8:<sha8> -t lifetrac-tractor-x8:latest .`
   (log `/home/fio/docker_build_tractor_<sha8>.log`). Then a smoke test without
   network or devices (`import numpy, cv2, x8_image_pipeline.encode_vector`),
   `pip freeze`, and `docker save -o … && gzip`. With `--with-mosquitto`,
   `eclipse-mosquitto:2` is saved too.
3. PC: the tarball, md5, manifest, pip freeze, smoke output and build log go to
   `$BENCH_ARCHIVE_DIR/images/`. Once the md5 matches, the tarball is deleted
   on the base; the image stays there.
4. Tractor, over adb: push to `/home/fio/lifetrac_images/` (on disk; `/tmp` is a
   RAM tmpfs), check the md5, then as root `docker load -i`. The image `:latest`
   pointed at before is tagged `lifetrac-tractor-x8:previous`. The same smoke
   test runs on the tractor, and the tarball is deleted (keep it with
   `--keep-tarball`). It refuses a board that carries the base's deploy tree.

The tractor also needs two images it cannot pull, which this script does not
cover: `hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44` (the
repo-root `foundries_python.tar`, `docker load -i`) and `eclipse-mosquitto:2`
(`--with-mosquitto` here). See [BENCH_SETUP.md](BENCH_SETUP.md).

## Lock files: the flown wheel versions

`requirements.txt` on both sides holds floors only, so a fresh build resolves
whatever is newest on PyPI that day. The two lock files record what the flown
images actually run:

| lock file | flown image | source of the list |
|---|---|---|
| [`base_station/requirements.lock.txt`](../../../base_station/requirements.lock.txt) | `lifetrac-v25` `4623980c2dac` (from `d3751286`, 2026-09-15) | its pip layer `9e3efb0b5438` was cached from the 2026-09-12 build; that build's log (`docker_build.log`) has the full list |
| [`firmware/tractor_x8/requirements.lock.txt`](../../tractor_x8/requirements.lock.txt) | `lifetrac-tractor-x8` `2727dfd36f9f` (from `65869517`, 2026-09-26) | `rs13_build.log`, an uncached build of that image |

Both logs are in
[`bench-evidence/board_state_2026-10-04/base/files/home_fio/`](../../../bench-evidence/board_state_2026-10-04/base/files/home_fio/).
The headers name the image ids, the python base image digests and the logs.

Both Dockerfiles gained an optional `ARG LOCKED` (default `0`). With
`--build-arg LOCKED=1` the lock file is applied as a pip **constraints** file on
top of `requirements.txt`. A dependency added later still installs, unpinned,
and a floor raised above a pin fails the build loudly; then refresh the lock
or deploy with `--unlocked`. The default build is unchanged: plain
`docker build` and `docker compose build` ignore the lock, and CI builds no
image. Because the pip `RUN` line changed, the first build after this change
re-runs pip once instead of using the cached layer. Unlocked, that resolves
the newest wheels; locked, it reproduces the flown set. The scripts pass
`LOCKED=1` unless `--unlocked` is given. A commit older than the lock files
builds unlocked with a warning.

`ARG PYTHON_IMAGE` (default: the old `FROM` tag) can pin the python base image
by digest. The digests are in the lock headers. Debian packages are not pinned;
the tractor log's versions are recorded in its lock header.

To refresh a lock after a deliberate unlocked rebuild, use the `pip freeze`
file the script saved and update the header.

## What the scripts change

| where | `deploy_base.sh` | `build_tractor_image.sh` |
|---|---|---|
| base | `BASE_DEPLOY_DIR` tree, `DEPLOYED_FROM.txt`, `.env` + `secrets/` if missing, `lifetrac-v25:latest/:<sha8>/:previous`, broker container, files in `/home/fio/lifetrac_deploy/`, `/home/fio/docker_build_<sha8>.log`, `/home/fio/pip_freeze_lifetrac-v25_<sha8>.txt` | `lifetrac-tractor-x8:<sha8>/:latest` (a build product; the base does not run it), `/home/fio/lifetrac_build/`, `/home/fio/docker_build_tractor_<sha8>.log` |
| tractor | nothing | loaded image, `:latest`, `:previous`; `/home/fio/lifetrac_images/` while loading |
| PC | `$BENCH_ARCHIVE_DIR/deploy/` | `$BENCH_ARCHIVE_DIR/images/`, `$BENCH_ARCHIVE_DIR/build/` |
| never | `/etc`, systemd units, the radios, the L072 firmware, `/tmp/lifetrac_strict`, any container other than the broker | `/etc`, systemd units, the radios, any running container |

## Rollback

- **Base tree:** deploy the older commit: `bash deploy_base.sh <old-sha>`. Or
  restore the backup by hand:
  ```sh
  cd /var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER
  sudo rm -rf base_station firmware deploy
  sudo tar -xzf /home/fio/lifetrac_deploy/prev_tree_<oldsha>_<time>.tgz
  ```
- **Base image:** `docker tag lifetrac-v25:previous lifetrac-v25:latest`, or any
  `lifetrac-v25:<sha8>`. The broker runs `eclipse-mosquitto:2`, so nothing
  needs a restart; the next leg's daemon containers start from the new
  `:latest`.
- **Tractor image:** `docker tag lifetrac-tractor-x8:previous lifetrac-tractor-x8:latest`
  on the tractor, or `bash build_tractor_image.sh --push-only <archived tarball>`.
  The flown RS-13 image `2727dfd36f9f` is also tagged `:rs13-65869517` on the
  tractor, and the older `:pre-rs13` (`9bfbbc8d06cb`) is still there. The first
  run of this script makes `:previous` = `2727dfd36f9f`.
- **Housekeeping:** remove old builds with `docker image rm lifetrac-v25:<sha8>`
  or `docker image prune` (dangling images only). **Never `docker image prune -a`
  or `docker system prune -a` on the tractor**: it deletes every image no
  container uses, including the Arduino OOTB images and `pre-rs13`, which the
  offline tractor cannot pull again.

## Traps

- **CRLF.** With Git for Windows' default `core.autocrlf=true`, `git archive`
  writes CRLF. For `base_station/` plus the Dockerfile that came to 47,419 CR
  bytes in the tarball against 0 in the commit, and every shell script and
  config on the board breaks. Both scripts pass `-c core.autocrlf=false -c core.eol=lf` and
  compare CR counts with the commit's blobs. The board-side scripts they
  generate are stripped of CR. For any file you push by hand, use
  `git archive` the same way or `sed -i 's/\r$//'` (`git diff` shows nothing,
  because the index is LF). The repo's `.gitattributes` keeps scripts LF in the
  working tree.
- **sudo eats stdin.** `git archive … | ssh base "echo pw | sudo -S tar -x"`
  fails with "This does not look like a tar archive": `sudo -S` reads the same
  stdin as tar. Copy the file first, then `tar -xzf FILE` or `docker load -i FILE`.
  The scripts do exactly that. They use `sudo -n` (the bench drop-in
  `99-lifetrac-bench-nopasswd`); the password fallback feeds only sudo.
- **The recovery unit wipes Docker.** On the LmP 934 image
  `compose-apps-early-start-recovery.service` runs
  `systemctl stop docker; rm -rf /var/lib/docker` every 60 s unless it is
  masked. Both scripts refuse to run while it is unmasked; masking is part of
  [provisioning](BENCH_SETUP.md).
- **`docker compose up -d` with no service name** on the base starts
  `lora_bridge` on `/dev/ttymxc3`. Always name `mosquitto`.
- **A moved `:latest` does not restart anything.** A container already running
  the old image keeps it until it is recreated.
- **`/tmp` is a RAM tmpfs** on both boards, aged at 5 days and emptied at
  reboot. Large files go to `/home/fio`.
- **adb from Git Bash** needs `MSYS_NO_PATHCONV=1` for board paths and a
  Windows path for the local file. The scripts set both per call. `adb shell`
  does not return the remote exit status on every adbd, so the scripts read a
  status marker instead.

## Radio safety

- Deploying never touches the radios. The scripts open no serial port, flash
  nothing, start no radio daemon and restart no container except the base
  broker. The L072s stay in whatever state they are in. After a power-up that
  is RXCONT, receive only; parking them is the job of the power-up guard
  ([legs/README.md](legs/README.md)), not of a deploy. A deploy is not a radio
  GO.
- **Never start `lora_bridge`** on the bench. `docker-compose.yml` maps
  `${LIFETRAC_LORA_DEVICE}`, which the bench `.env` sets to `/dev/ttymxc3`,
  the L072 radio UART. `lora_bridge` would open it and drive the radio with
  the old framing, which the Method-G firmware does not speak.
  `deploy_base.sh` starts only `mosquitto`, with `--no-deps`. It refuses to run
  while any container maps `/dev/ttymxc3` (a leg in progress) and warns if one
  appears afterwards.
- **The base's `lifetrac-base.service` and `lifetrac-base-compose.service` must
  stay disabled.** On the bench base both are currently **enabled but failing
  at every boot** (capture
  [`board_state_2026-10-04/base/10_systemd.txt`](../../../bench-evidence/board_state_2026-10-04/base/10_systemd.txt)).
  `lifetrac-base.service` runs `docker compose up -d --build --remove-orphans`,
  which starts `lora_bridge` on `/dev/ttymxc3` and rebuilds the image on the boot
  path. `lifetrac-base-compose.service` brings up the video-test stack, whose
  `image_rx` maps `/dev/ttymxc3`. If either ever succeeded, a radio daemon would
  start at boot without a GO. **Recommendation:** disable both, after the
  operator agrees (an open item in [BENCH_BOARDS.md](BENCH_BOARDS.md)):
  ```sh
  sudo systemctl disable lifetrac-base.service lifetrac-base-compose.service
  ```
  `deploy_base.sh` warns while they are enabled and refuses to run if either is
  active. It never installs, enables or starts them, and the bench kit does not
  install the unit files in `deploy/`.
- On the tractor, `lifetrac-camera.service` (started by udev at boot) runs
  `lifetrac-tractor-x8:latest` with `/dev/ttymxc3` mapped as its M7 port. Loading
  a new `:latest` does not start it. The next time it starts, it runs the new
  image, on the radio UART as before. The bench stops it before any radio
  work: BENCH_RUNBOOK prep 3, `systemctl stop lifetrac-camera.service;
  docker stop tractor-camera`.
