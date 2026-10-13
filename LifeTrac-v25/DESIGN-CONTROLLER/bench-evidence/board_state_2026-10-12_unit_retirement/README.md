# Boot-unit retirement on the bench X8s — 2026-10-12

Three LifeTrac systemd units were enabled on the bench boards and failed at
every boot that was captured: on the base at the 10-04 capture and at the
10-10 boot, on the tractor at the 10-10 boot. Earlier boots are not on
record. The operator approved retiring them on 2026-10-12. Radios stayed
parked throughout: nothing here opened `/dev/ttymxc3`, no board was rebooted,
and no L072 firmware was touched.

The procedure, the reasons and the rules that follow from it are in
[BENCH_BOARDS.md](../../firmware/x8_lora_bootloader_helper/bench_tools/BENCH_BOARDS.md).

## Why they were retired, not repaired

Each unit failed for an incidental reason, and that failure was the only thing
keeping a radio daemon from starting at boot without a GO.

| Unit | Board | What it is | Why it failed | If it had succeeded |
|---|---|---|---|---|
| `lifetrac-base.service` | base | Production compose stack (repo `base_station/systemd/`), installed 2026-05-26 | `docker compose up -d --build` at boot: three services export the same tag `lifetrac-v25:latest`, and all three exports die with `layer does not exist` (exit 17) | Moves the tag `lifetrac-v25:latest` off the flown image `4623980c2dac`; starts `web_ui`, which republishes a retained `radio_profile` into the harness broker; starts `lora_bridge` with the radio UART mapped |
| `lifetrac-base-compose.service` | base | May-2026 video-test demo (repo `deploy/`), installed 2026-05-27 | Its mosquitto wants `127.0.0.1:1883`, which the bench broker `design-controller-mosquitto-1` holds (the 10-04 capture records the failed bind); at the 10-10 boot it hung at "Starting" until the 180 s start timeout, and why it hung is not established | `image_rx` opens `/dev/ttymxc3` with `SKIP_RESET_REQ=0` (a reset that un-parks the L072) at `REG_PROFILE 0` |
| `lifetrac-tractor-compose.service` | tractor | The same May demo, run from the non-git `/opt/lifetrac/video-test` | It uses `-p tractor-vtest`, but the fixed container names were held by Exited containers of project `video-test` | `tractor-image-tx-v2` on `/dev/ttymxc3` at `REG_PROFILE 0` plus a camera with `LIFETRAC_USE_LORA_BRIDGE=1`: image TX at every boot |

The journals are in `base/00_failure_journal_2026-10-11.txt` and
`tractor/00_failure_journal_2026-10-11.txt`.

**Reading the base timestamps.** The base has no battery-backed clock, and
its wall clock is stepped forward early in a boot. At this boot (10-10) the
step was 239 s: `lifetrac-base-compose.service` has `TimeoutStartSec=180`,
which systemd counts on the monotonic clock, yet the journal shows 419 s
between its start and its timeout. Uptime puts the true boot at about
16:58:30 UTC, yet both units are logged as starting at 16:54:58. The journal
jumps from 16:55:00 to 16:59:04 inside `lifetrac-base.service`'s build, so
the step most likely fell there: the unit then failed about 7 s after it
started, not 4 minutes, and BuildKit's "exporting layers 243.4s" in the same
journal is the same artefact. That part is inferred; no monotonic timestamps
were captured. Only this one boot was measured, and the size of the step
presumably follows how long the board was off, so do not subtract 239 s from
another boot's journal. The tractor's clock is not wall time at all: it read
2026-09-23 when the PC read 2026-10-12.

## What was done

All times are PC UTC. The retire directory has the same name on both boards:
`/etc/lifetrac-retired/20261012T212515Z/`.

### Base (`2D0A1209DABC240B`, over ssh)

1. Preflight (`base/10_preflight.txt`): both units `failed`, nothing holding
   the UART, bench broker Up.
2. `systemctl disable` (no `--now`) and `reset-failed` for both units, then the
   two unit files and the `lifetrac-base.service.d/` drop-in moved to the
   retire directory, then `daemon-reload` (`base/11_disable_and_move.txt`).
3. The four never-started `lifetrac-vtest-*` containers and the
   `lifetrac-vtest_default` network removed, without `-f` or `-v`
   (`base/12_remove_dead_containers.txt`).

Left alone: `design-controller-mosquitto-1` (still Up), all volumes including
`lifetrac-vtest_*`, all images, `rx_smoke` / `bench_mqtt` / `f11_webui`.

### Tractor (`2E2C1209DABC240B`, over adb)

1. Preflight (`tractor/10_preflight.txt`): unit `failed`, nothing holding the
   UART, `lifetrac-camera.service` inactive.
2. `systemctl disable` (no `--now`) and `reset-failed`, the unit file moved to
   the retire directory, `daemon-reload`. `lifetrac-camera.service` was checked
   after each reload and stayed inactive (`tractor/11_disable_and_move.txt`).
3. `/opt/lifetrac/video-test` copied to the private archive
   (`board_state_2026-10-12/tractor/video-test`, 10 files) and verified against
   the board with sha256 (`tractor/12_video-test_tree.sha256`). This step, and
   the `docker inspect` saves that went with it, were typed by hand from the
   PC; no script was kept for them. The old containers had no logs to save:
   the board's docker log driver is `journald`, and the journal only holds
   the current boot.
4. `/opt/lifetrac/video-test/docker-compose.yml` renamed to
   `docker-compose.yml.RETIRED-2026-10-12`, so no `docker compose up` in that
   folder finds a file. Then the three Exited containers
   (`tractor-image-tx-v2`, `tractor-camera-v2`, `tractor-mosquitto-v2`) and the
   networks `video-test_default` and `tractor-vtest_default` removed, without
   `-f` or `-v` (`tractor/13_disarm_and_remove_containers.txt`).

Left alone: `lifetrac-camera.service` and its `tractor-camera` /
`tractor-mosquitto` containers, the harness containers, the `x8-*` containers,
all volumes, all images including `lifetrac-tractor-x8:pre-rs13`.

The order on the tractor matters: the unit was disabled before the containers
were removed. Removing them first would have let the unit succeed at the next
boot.

## End state

- `systemctl --failed` lists only the stock `psplash-start.service` on both
  boards.
- `fuser /dev/ttymxc3` shows no holder on either board, before and after.
- `provision_bench_board.sh <role> --check` (read-only), run against both
  boards after the retirement, reads `ok` for "old boot units retired"
  (`base/20_provision_check.txt`, `tractor/20_provision_check.txt`). The other
  TODO lines there (the probe image on the base; the NetworkManager WiFi flag,
  the stored WiFi profile and the camera provisioning on the tractor) are
  older and not part of this change.
- Not yet observed: the next boot. Expect the same failed list, and the base
  broker back Up through its own `unless-stopped` restart policy. The checks
  to run at that boot are in BENCH_BOARDS.md, "Not yet observed: the next
  boot".

## Undo

Move the files back from `/etc/lifetrac-retired/20261012T212515Z/` to
`/etc/systemd/system/`, run `systemctl daemon-reload`, then
`systemctl enable <unit>`. On the tractor the compose file must also be
renamed back from `docker-compose.yml.RETIRED-2026-10-12`; until then the
unit finds no compose file and fails.

The same unit text is in git at `cf71a587`
(`git show cf71a587:LifeTrac-v25/DESIGN-CONTROLLER/base_station/systemd/lifetrac-base.service`,
and `deploy/lifetrac-base-compose.service`,
`deploy/lifetrac-tractor-compose.service` likewise). It is not
byte-identical: the board copies are 480, 789 and 517 bytes
(`base/10_preflight.txt`, `tractor/10_preflight.txt`), each larger than its
git blob (463, 769, 500 bytes) by exactly its line count, so they have CRLF
line endings. The repo files have since gained a comment header. The
byte-exact originals are the retired files on the boards. The drop-in is in
[`board_state_2026-10-04`](../board_state_2026-10-04/base/files/etc/systemd/system/lifetrac-base.service.d/timeout.conf).

Do not undo any of this on a bench board without reading BENCH_BOARDS.md
first. With the leftover containers gone, the renamed compose file is now
the only thing between the tractor unit and the air: after a full undo the
unit would succeed and transmit at the next boot.

## Files

- `scripts/`: the board-side scripts that were run, as they were run
  (`diag_*` on 10-11, read-only; the rest on 10-12). The numbering has gaps
  because the steps in between were done from the PC by hand: the archive
  copy of `/opt/lifetrac/video-test`, its sha256 comparison and the
  `docker inspect` saves.
- `base/`, `tractor/`: their output, unedited except that the tractor
  preflight's list of anonymous docker volumes is replaced by a count.
- A false negative in that output: the `## listeners` line in
  `base/12_remove_dead_containers.txt` says "none on 1883/8080", but the
  bench broker was Up on `127.0.0.1:1883` at that moment (the `docker ps`
  list a few lines above it shows `design-controller-mosquitto-1` Up). The
  check in `scripts/base_3_containers.sh` uses `ss`, which does not work on
  the base (the 10-04 capture's `ss … || netstat` fell through to
  `netstat`), so it prints "none" whatever is listening. Use `netstat`
  there. The log is left as it was written.
