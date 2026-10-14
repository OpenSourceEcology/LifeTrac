# Bench boards: chips, firmware and board-resident software

State as of **2026-10-10**, plus the boot-unit retirement of **2026-10-12**.
Both boards are measured from read-only captures:
- base: [`bench-evidence/board_state_2026-10-04/base/`](../../../bench-evidence/board_state_2026-10-04/base/);
- tractor: [`bench-evidence/board_state_2026-10-10/tractor/`](../../../bench-evidence/board_state_2026-10-10/tractor/).

On 2026-10-12 three boot units were retired and their leftover containers
removed. That change is recorded in
[`bench-evidence/board_state_2026-10-12_unit_retirement/`](../../../bench-evidence/board_state_2026-10-12_unit_retirement/README.md)
and marked "Changed 2026-10-12" in the sections below; see
[Retired boot units](#retired-boot-units-2026-10-12).

The rest comes from a review of AI NOTES, bench-evidence,
X8_HEALTH_AND_RECOVERY and git history.

**To bring up a new pair to this state, see [BENCH_SETUP.md](BENCH_SETUP.md).**
It covers hardware, the OS image pinned by URL and sha256, the reflash,
first boot, verification and the L072 firmware.
[`provision_bench_board.sh`](provision_bench_board.sh) applies the `/etc`
and docker changes listed under *LifeTrac software living in the X8 Linux*
below. It never installs the three retired boot units. If it finds one that
is failed or inactive, it retires it; if one is active, starting or
stopping, that step fails and changes nothing
([Retired boot units](#retired-boot-units-2026-10-12)). `--check` reports
how far a board is from that state.

## The two boards

| | base | tractor |
|---|---|---|
| serial (adb) | `2D0A1209DABC240B` | `2E2C1209DABC240B` |
| hardware | Portenta X8 on a Portenta Max Carrier | same |
| OS (LmP) | **4.0.11-934-91**, kernel 6.1.24-lmp-standard (uuu reflash 2026-05-13 and 05-24). The bundle is `https://downloads.arduino.cc/portentax8image/934.tar.gz`; its sha256 list is in [T3a](../../../X8_HEALTH_AND_RECOVERY/recovery/T3a_sdp_uuu_reflash.md). | **4.0.3-674-88**, kernel 5.10.93-lmp-standard (factory, never reflashed). Arduino still hosts the image at `https://downloads.arduino.cc/portentax8image/674.tar.gz` (854,998,055 B). Nobody here has downloaded or byte-verified it, so there is no sha256 on record. Reflashing to 934 would move it to 6.1.24; BENCH_SETUP §3 compares the two. |
| access | adb; ethernet 192.168.1.117 with eth0 pinned to 100BASE-TX full (marginal cable); ssh key `~/.ssh/lifetrac_base_ed25519`. On 2026-10-10 it did not enumerate on USB after a power cycle, but ssh worked. | adb only (WiFi off by policy) |
| power-down | `systemctl poweroff` did **not** keep it off: it booted again about 4.5 min later (2026-10-04). Remove power to keep it down. | stayed off after `systemctl poweroff` |
| clock | NTP, but no battery-backed clock: the wall clock is stepped forward early in a boot (239 s at the 10-10 boot, the one boot measured; the size presumably follows how long the board was off), so journal time spans across that step are inflated | no time source; it lags by days to weeks (it read 2026-09-23 on 2026-10-12, 19 days behind) |

Pairing matters for recovery. The L072 firmware and option bytes, the
carrier's J-Link OB and the DIP switches belong to the **carrier**. The H747
flash, the eMMC/OS and the SE050 belong to the **X8 module**. Which carrier
goes with which module is not recorded.

## Programmable chips

| chip | where | runs | LifeTrac-custom? | saved | notes |
|---|---|---|---|---|---|
| STM32L072 in the Murata CMWX1ZZABZ (+ SX1276, no firmware) | carrier | `firmware/murata_l072` bench build **`0c1bb0a9`** since 2026-09-15 | **yes** | Source on `main`. Binaries and the full flashed-build inventory in [`RS_13…/firmware/`](../../../bench-evidence/RS_13_vector_scene_2026-09-26/firmware/README.md). `0c1bb0a9` rebuilds byte-identically from `main` with arm-none-eabi-gcc **12.2.1 (Arm GNU Toolchain 12.2.MPACBTI-Rel1)**. | The firmware never writes its own flash or EEPROM. Its option bytes were never recorded. The only proof of what is on the chip is the 09-15 flash logs (base copy now in the capture; `run_flash_bench.sh` deletes them at the next flash). |
| STM32H747 Cortex-M7 (flash bank 1) | X8 module | **stock Arduino x8h7** SPI bridge, programmed at boot by `stm32h7-program.service` from `/usr/arduino/extra/STM32H747AII6_CM7.bin`. The image differs per board: base sha256 `d81eaa81…`, tractor `0b03cd9c…`. | no | Part of each LmP image | **The tractor reflashes it at every boot.** Its `program-h7.sh` cannot read the version (the sysfs read times out), so it reprograms bank 1, `0x08000000`–`0x080709a4`, ending "Programming Finished … Verified OK"; bank 2 is not touched. The base's newer script prints "matches, No Update" after the same failed read, so on the base it is not proof. LifeTrac overwrote this bank with `tractor_h7` on 2026-05-04; it is stock again. |
| STM32H747 Cortex-M4 (flash bank 2, `0x08100000`) | X8 module | **unknown, probably a May 2026 LifeTrac bench sketch**, started at every boot by x8h7 | probably | Sources only: `firmware/x8_uart_route_probe`, `x8_lora_bootloader_helper`, `portenta_m7_l072_passthrough_ping`, `tractor_h7`. The flashed ELFs were never kept. | Last recorded uploads: x8_uart_route_probe → 2D0A, x8_lora_bootloader_helper → 2E2C (the May notes mix up COM ports, so the per-board mapping is uncertain). No erase is recorded. No visible effect, since both L072s boot normally, but it is undocumented code. Identify it with the openocd dump in *Open items*, then decide whether to blank the slot. |
| i.MX8M Mini boot chain, kernel, device tree | X8 module | stock LmP (U-Boot arms the 60 s WDOG) | no (two May bootargs/BLS experiments were removed by later reflashes) | 934 bundle on the PC (private archive, sha256) | The i.MX's own Cortex-M4 is unused. |
| SARA-R412M, SE050, Murata 1DX WiFi/BT, ANX7625, PMIC, CS42L52, USB2514, BQ24195, CAN/RS-485 transceivers; J-Link OB (STM32F405) on the carrier; Kurokesu C2 camera (tractor) | — | vendor firmware or none | no | n/a | The SE050 is unused by LifeTrac (ARCHITECTURE.md describes intent only); never export it. Do not accept J-Link OB update prompts. |

Not on the bench: `tractor_h7` / `tractor_h7_m4`, whose design target is the
X8's own H747; their linker script starts inside x8h7's region, an unresolved
design conflict. Also absent are the Opta (`tractor_opta`) and the MKR WAN
1310 handheld (`handheld_mkr`). Only their sources exist, and CI only
compiles them. The patched `libmbed_x8.a` needed to rebuild the X8 M7 image is
gitignored; a copy is in the private archive.

Repo L072 binaries that are on no board:
- `tools/mlm32l07x01.bin` is the stock MKRWAN AT firmware 1.2.3, the factory
  restore image.
- `x8_lora_bootloader_helper/mlm32l07x01.bin` is **misnamed**: it is LifeTrac
  "hello v0.2".
- `board2_l072_flash_dump.bin` is a readback of an early LifeTrac build, not
  the factory image.

## LifeTrac software living in the X8 Linux

### Base (measured 2026-10-04)

- **`/etc` changes** (`ostree admin config-diff`, `02_ostree.txt`):
  - `sudoers.d/99-lifetrac-bench-nopasswd` (`fio ALL=(ALL) NOPASSWD: ALL`).
  - `compose-apps-early-start.service` and `…-recovery.service` are masked.
    **Unmasking the recovery unit wipes `/var/lib/docker`.**
  - `lifetrac-base.service` (the production base compose stack, with the
    drop-in `lifetrac-base.service.d/timeout.conf`, `TimeoutStartSec=1800`)
    and `lifetrac-base-compose.service` (a May-2026 video-test demo unit).
    When measured, both were enabled and had failed at boot.
    **Changed 2026-10-12:** both are retired. systemd no longer lists them,
    and the files are in `/etc/lifetrac-retired/20261012T212515Z/`
    ([Retired boot units](#retired-boot-units-2026-10-12)).
  - NetworkManager `Wired connection 1` (the eth0 pin).
  - Removed wants: `lmp-auto-hostname`, `resize-helper`, `run-postinsts`.
  - `aktualizr-lite` is enabled but inactive, with no compose-apps registered.
- **Docker images:**
  - `lifetrac-v25:latest` `4623980c2dac`, built on the base from `d3751286`
    on 09-15;
  - `lifetrac-tractor-x8:latest` / `:rs13-65869517` `2727dfd36f9f`, built
    here for the tractor;
  - three dangling older base images (`ed7c0587`, `12fe97de`, `c6ded786`);
  - `python:3.11-slim(-bookworm)`, `eclipse-mosquitto:2`.

  None rebuilds byte-identically: the requirements are floors and the tags are
  mutable. The two LifeTrac images are exported to the private archive.
- **Containers:** `design-controller-mosquitto-1` runs (restart
  unless-stopped). That docker restart policy is what brings it up at boot;
  no systemd unit starts it. Leftovers: `rx_smoke`, `bench_mqtt`, `f11_webui`
  (Exited). **Changed 2026-10-12:** the four `lifetrac-vtest-*` containers
  (Created, never started) and the network `lifetrac-vtest_default` were
  removed. The `lifetrac-vtest_*` volumes were kept.
- **Retained topics** on that broker (`retained_topics.txt`) include a
  leftover `control/encode_mode_override {"mode":"mono_g4"}` from leg 2d_r4.
  The leg prep's `clear_retained.py` clears it.
- **`/var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER`** is a git subset of
  `d3751286` plus `DEPLOYED_FROM.txt`, `.env` (keys `LIFETRAC_PIN`,
  `LIFETRAC_LORA_DEVICE`, `LIFETRAC_TRUSTED_PROXIES`) and `secrets/`
  (`lifetrac_pin`, `lifetrac_fleet_key`, a random bench key). The secrets
  stay off git. The tree, minus secrets, is in the private archive.
- **`/home/fio`.** The unique files are now in the repo capture
  (`files/home_fio/`):
  - `wdog_regs.py`;
  - the 09-15 L072 flash logs (`pipeline_stamped.log`, `kmsg_flash.log`,
    `wdt_pet.log`);
  - the docker build logs, the only record of `4623980c2dac`'s wheel
    resolution.

  `rs13_build/`, `rs13_tests/` and `dc_deploy.tar.gz` are copies of git
  trees.

### Tractor (measured 2026-10-10)

- **`/etc` changes** (`ostree admin config-diff`):
  - `sudoers.d/99-lifetrac-bench-nopasswd`;
  - `modprobe.d/lifetrac-no-usb-audio.conf`;
  - `udev/rules.d/99-w2-01-c2.rules`, which starts `lifetrac-camera.service`
    on camera hotplug;
  - `systemd/system/lifetrac-camera.service`. This is the helper version,
    `firmware/x8_lora_bootloader_helper/lifetrac-camera.service`, not
    `tractor_x8/systemd/`. It reads "disabled", but udev starts it anyway.
    **It grabs `/dev/ttymxc3`, the radio UART, at every boot.** Stop it
    before any probe.
  - `lifetrac-tractor-compose.service`, a May-2026 video-test demo unit.
    When measured it was enabled and had failed at boot. **Changed
    2026-10-12:** retired; the file is in
    `/etc/lifetrac-retired/20261012T212515Z/`
    ([Retired boot units](#retired-boot-units-2026-10-12));
  - `disable-wifi.service`, enabled: an rfkill block at boot;
  - `wpa_supplicant.service` masked;
  - `fio` added to `video` / `docker`;
  - `docker/key.json`.

  These were installed by `provision_x8.sh` (2026-05-15) and the May
  WiFi-off steps, with one exception: `lifetrac-tractor-compose.service`.
  The only unit `provision_x8.sh` installs is `lifetrac-camera.service`.
  The compose unit was installed and enabled on 2026-05-27 06:36 UTC by the
  repo-root `tmp_install_tractor_unit.sh` (deleted on 2026-10-14, rule 1).

  The `disable-wifi.service` unit is now in git as
  [`units/disable-wifi.service`](units/disable-wifi.service), copied from
  the capture. `provision_bench_board.sh tractor` installs and enables it,
  masks `wpa_supplicant`, and reruns `provision_x8.sh` when the camera
  files differ.
- **WiFi:** soft-blocked and disabled in NetworkManager. **A stored
  `5star.nmconnection` holds the WiFi password** (not copied).
- **`/var/rootdirs/opt/lifetrac`:**
  - `compose-apps/lifetrac-camera`, which runs `lifetrac-tractor-x8:latest`
    with `/dev/video1` and `/dev/ttymxc3`;
  - the hand-edited May `video-test` stack, which is not in git. **Changed
    2026-10-12:** the folder is still on the board, but its
    `docker-compose.yml` is renamed `docker-compose.yml.RETIRED-2026-10-12`,
    so no `docker compose up` in that folder finds a file. A copy of the
    tree as it was before the rename (10 files, sha256 verified) is in the
    private archive;
  - `bin/ffmpeg`.
- **Containers** (2026-10-12; the 10-10 capture has the earlier list).
  Running: the Arduino `x8-devel`, `x8-provisioning`, `x8-webapp`. Exited:
  `tractor-camera` and `tractor-mosquitto` (the `lifetrac-camera` compose
  app), the harness containers `camera_svc`, `tx_smoke`, `bench_mqtt`, and
  `lifetrac-camera-x8`. **Changed 2026-10-12:** the three Exited video-test
  containers `tractor-image-tx-v2`, `tractor-camera-v2` and
  `tractor-mosquitto-v2` and the networks `video-test_default` and
  `tractor-vtest_default` were removed. The volumes
  `video-test_mosquitto_data` and `tractor-vtest_mosquitto_data` were kept.
  The board also holds 335 anonymous docker volumes (see *Open items*).
- **Images:**
  - `lifetrac-tractor-x8:latest` / `:rs13-65869517` `2727dfd36f9f`;
  - `lifetrac-tractor-x8:pre-rs13` `9bfbbc8d06cb` (594 MB);
  - `python:3.11-slim-bookworm`;
  - `eclipse-mosquitto:2`;
  - the Arduino OOTB images `arduino-ootb-python-devel:738bc44` (= the
    repo-root `foundries_python.tar`), `arduino-ootb-webapp`,
    `arduino-iot-cloud-provisioning`.

  Both LifeTrac images are now in the private archive; `pre-rs13`'s only
  off-board copy is there.
- **`/home/fio`.**
  - Unique, now in the repo capture: `wdog_regs.py`,
    `lifetrac_compact_at_probe.sh`, `at_probe.sh`, and the 09-15 flash logs.
  - `base_station/` is an old copy of the repo's `base_station` (with a
    `.pytest_cache`); `lifetrac_p0c/` is old flash staging.

## Retired boot units (2026-10-12)

Three LifeTrac units were enabled on the original pair and failed at every
boot that was captured: on the base at the 10-04 capture and at the 10-10
boot, on the tractor at the 10-10 boot. Earlier boots are not on record
(for `lifetrac-base.service` the July notes cited in the table say its build
path worked again on 07-26, and the broker container that blocked the
video-test unit was created that day). On 2026-10-12, with the operator's
approval, they were retired, not repaired: each one failed for an incidental
reason, and that failure was the only thing keeping a radio daemon from
starting at boot without a GO. Record, logs and the scripts that ran:
[`board_state_2026-10-12_unit_retirement/`](../../../bench-evidence/board_state_2026-10-12_unit_retirement/README.md).

| unit | what it was | why it failed | what it would have started |
|---|---|---|---|
| `lifetrac-base.service` (base), with the drop-in `lifetrac-base.service.d/timeout.conf` (`TimeoutStartSec=1800`) | The **production** base compose stack: [`base_station/systemd/lifetrac-base.service`](../../../base_station/systemd/lifetrac-base.service), commit `a90ae759` (2026-04-27). Installed on the base by hand on 2026-05-26 15:05 UTC; no script in git installs it. | Its `ExecStart` is `docker compose up -d --build --remove-orphans`. `lora_bridge`, `web_ui` and `audit_tail` each `build: .` to the same tag `lifetrac-v25:latest`, and all three BuildKit exports fail with `layer does not exist` (exit 17). Root cause unknown; the same error was diagnosed as docker layer-store corruption in July ([NOTES](../../../bench-evidence/RS_throughput_rebaseline_2026-07-25/NOTES.md), "Base redeploy"). | A retag of `lifetrac-v25:latest` away from the flown image `4623980c2dac`: the same `d3751286` tree, but a new image id that cannot be reproduced, and the base has no `:previous` tag. `web_ui`, which republishes a retained `control/radio_profile` into the harness broker at startup; on 2026-07-26 that caused a live FHSS→DTS switch on air (same NOTES, "Run R"). `lora_bridge` with `/dev/ttymxc3` mapped: as configured it is told to open `/dev/ttymxc3`, which inside the container is `/dev/ttyACM0`, so it would probably crash-loop (inferred, not observed). It speaks KISS at 115200, which the Method-G L072 firmware does not. |
| `lifetrac-base-compose.service` (base) | A **May-2026 video-test demo** unit: [`deploy/lifetrac-base-compose.service`](../../../deploy/lifetrac-base-compose.service), commit `03998936` (2026-05-27). Installed and enabled by the repo-root `tmp_install_base_unit.sh` (deleted on 2026-10-14, rule 1) on 2026-05-27 06:38 UTC. | Its mosquitto publishes `127.0.0.1:1883`, which the bench broker `design-controller-mosquitto-1` already holds. The 10-04 capture records `Bind for 127.0.0.1:1883 failed: port is already allocated`. At the 10-10 boot it hung at "Starting" instead, until its 180 s start timeout. | `image_rx` on `/dev/ttymxc3` with `LIFETRAC_SKIP_RESET_REQ=0`, a soft RESET_REQ that un-parks the L072, at `LIFETRAC_REG_PROFILE=0` (fixed 915 MHz). |
| `lifetrac-tractor-compose.service` (tractor) | The same May demo: [`deploy/lifetrac-tractor-compose.service`](../../../deploy/lifetrac-tractor-compose.service), commit `03998936`. Installed and enabled by the repo-root `tmp_install_tractor_unit.sh` (deleted on 2026-10-14, rule 1) on 2026-05-27 06:36 UTC, not by `provision_x8.sh`. It runs `docker compose -p tractor-vtest up -d` in the non-git `/opt/lifetrac/video-test`. | That compose file pins `container_name` to `tractor-*-v2`, and those names were held by Exited containers of compose project `video-test`. | `tractor-image-tx-v2` on `/dev/ttymxc3` at `LIFETRAC_REG_PROFILE=0`, plus `tractor-camera-v2` with `LIFETRAC_USE_LORA_BRIDGE=1`: image TX at every boot without a GO. |

Base journal times need care, because the wall clock is stepped forward
early in a boot. At the 10-10 boot the step was 239 s: the 419 s the journal
shows for `lifetrac-base-compose.service` is its 180 s start timeout, which
systemd counts on the monotonic clock. The same journal jumps from 16:55:00
to 16:59:04 inside `lifetrac-base.service`'s build, so the step most likely
fell there: the unit then failed about 7 s after it started, not the
4 minutes the journal shows, and BuildKit's "exporting layers 243.4s" is the
same artefact (inferred; no monotonic timestamps were captured). Only this
one boot was measured, so do not subtract 239 s from another boot's journal.

**What was done.** On each board: `systemctl disable` without `--now`,
`systemctl reset-failed`, the unit file (and the `.d` directory) moved to
`/etc/lifetrac-retired/20261012T212515Z/` with a `README.txt`, then
`systemctl daemon-reload`. Only after that were the leftover containers
removed; the *Base* and *Tractor* sections above list them. No board was
rebooted and nothing opened `/dev/ttymxc3`: the holder check showed nobody
on either board, before and after. On the tractor `lifetrac-camera.service`
stayed inactive through both `daemon-reload`s (systemd 250). End state:
`systemctl --failed` lists only the stock `psplash-start.service` on both
boards.

Left alone: `design-controller-mosquitto-1` (still Up),
`lifetrac-camera.service` with `tractor-camera` / `tractor-mosquitto`, the
harness containers, the `x8-*` containers, all volumes and all images.

[`provision_bench_board.sh`](provision_bench_board.sh) now enforces this
state on a re-provisioned board, in its item "old boot units retired":

- The item is `ok` only when systemd cannot load any unit of the board's
  role from a file. For each unit, either `systemctl is-enabled` finds no
  unit file and nothing named `<unit>` is left in `/etc/systemd/system`, or
  it reads `masked` and `/etc/systemd/system/<unit>` is a symlink to
  `/dev/null`. The unit must also be `inactive` or `failed`, with no
  `<unit>.d` left there and no job queued. On both boards (systemd 250 on the
  tractor) "no unit file" is no word at all, only `Failed to get unit file
  state for <unit>: No such file or directory`; later systemd versions print
  `not-found`. "No job" is an empty line from
  `systemctl show -p Job --value <unit>`, as both boards answered on
  2026-10-12. Every other `is-enabled` word names a unit file that systemd
  can still load, and start by hand, from udev or as another unit's
  dependency: `enabled`, `linked` (a `systemctl link` symlink), `alias`,
  `static`, `indirect`, `disabled`, `generated`, `transient` and the
  `-runtime` forms (systemctl(1), `is-enabled`). A board that never had the
  units passes.
- Otherwise the script retires them, but only while every listed unit is
  `failed` or `inactive`, has no start or stop job queued, and reads
  `enabled`, `disabled`, `static`, `linked`, `masked` or no unit file, with
  `/etc/systemd/system/<unit>` matching that word (a file or a symlink, a
  symlink to `/dev/null` for `masked`, nothing or a dangling symlink for no
  unit file). It runs `systemctl disable` without `--now` and
  `systemctl reset-failed` (not for a masked unit), moves the unit files,
  the `.d` directories and any symlink there that is not a mask into a new
  `/etc/lifetrac-retired/<PC UTC time>/`, writes a `README.txt` there, and
  runs `systemctl daemon-reload`. A mask stays. A unit that is already
  disabled but still has its file there is moved the same way.
- If a listed unit is `active`, `activating`, `deactivating` or `reloading`,
  has a job queued (early in a boot an enabled unit reads `inactive` while
  its start job waits), or reads any other `is-enabled` word or one that
  does not match its path, the item is `FAILED`, nothing is changed, and the
  script prints what to do (rule 3 for a unit that runs). A `-runtime` form
  lives in `/run` until the next boot and a plain `disable` does not touch
  it; `alias` and `indirect` would make `disable` act on other units;
  `generated` and `transient` units have no file in `/etc/systemd/system`.
  The board reads the state once more right before each unit's `disable`.
  If a unit has changed by then, it is refused there: units handled before
  it stay disabled, no file is moved, and the item is `FAILED`.
- With `--check` nothing is changed: the item reads `TODO` (exit 3), and
  info lines name the unit or the leftover file that is the reason.
- A query to the board that does not answer never reads as `ok`: the item
  is `TODO` or `FAILED`, and nothing is moved.
- It never starts, stops or masks a unit, never runs `docker compose`, and
  never stops or removes a container. It reports the leftover video-test
  containers and the tractor's `video-test/docker-compose.yml`; removing them
  stays manual, in the order given below.

### Rules

1. **Do not repair, reinstall or re-enable these units on a bench board.**
   Removing the incidental failure is what puts a radio daemon on
   `/dev/ttymxc3` at boot. Deleted on 2026-10-14 (the last commit that has
   them is `cfb93d8`): `tmp_install_base_unit.sh`,
   `tmp_install_tractor_unit.sh`, `start_base.sh`, `start_base_long.sh`,
   `bringup_retry.sh`.
2. **`systemctl mask` is not possible.** The real unit files sit in
   `/etc/systemd/system`, which is where `mask` wants to put its `/dev/null`
   symlink ("File exists"). Disable, then move the files out.
3. **Stopping.** systemd runs `ExecStop` only if the start succeeded. For a
   failed or activating unit, `stop` and `disable --now` run no `ExecStop`.
   `lifetrac-base.service`'s `ExecStop` is a bare `docker compose down` in
   `DESIGN-CONTROLLER`, which is compose project `design-controller`: it
   removes the bench broker container (rule 4 says how to recreate it; the
   broker's volume survives). So the unit's state decides:
   - `failed` or `inactive`: `systemctl disable` without `--now`, then move
     the files out. `provision_bench_board.sh` does both.
   - `activating`: wait until it is `failed` or `active`, then follow that
     branch. Do not interrupt the build.
   - `deactivating`: a stop is running. Wait until it is `inactive` or
     `failed`.
   - `active`: the start succeeded, so radio daemons may be up. There is one
     route, in this order; both scripts print it:
     1. `sudo docker stop` the unit's radio containers by name (`sudo docker
        ps` lists them): `design-controller-lora_bridge-1` and
        `design-controller-web_ui-1` for `lifetrac-base.service`,
        `lifetrac-vtest-image_rx-1` and `lifetrac-vtest-web_ui-1` for
        `lifetrac-base-compose.service`, `tractor-image-tx-v2` and
        `tractor-camera-v2` for `lifetrac-tractor-compose.service`.
     2. `sudo systemctl stop <unit>`. For `lifetrac-base.service`, and only
        for it, this removes the broker container; recreate it afterwards
        (rule 4). The two demo units' `ExecStop` names its own compose
        project (`-p lifetrac-vtest`, `-p tractor-vtest`), so it does not
        touch the broker.
     3. Retire the unit: run `provision_bench_board.sh` again.

     Stopping only the containers is not enough. The units are
     `Type=oneshot` with `RemainAfterExit=yes`, so the unit stays `active`
     and enabled, and `provision_bench_board.sh` and `deploy_base.sh` keep
     refusing.
4. **Never run a bare `docker compose down` in `DESIGN-CONTROLLER` on a
   bench base.** It removes `design-controller-mosquitto-1`. If the broker
   container is gone, recreate it alone, as root in the deployed tree, with
   the line `deploy_base.sh` uses:
   `docker compose -p design-controller -f docker-compose.yml up -d --no-deps mosquitto`.
   Deleted on 2026-10-14 (the last commit that has them is `cfb93d8`):
   `base_videotest_up.sh`, `_bench_up2.ps1`.
5. **Order on the tractor.** Never remove the old `tractor-*-v2` containers,
   and never "fix" the unit's `-p`, while the unit is enabled. That removes
   the only barrier, and the tractor transmits at the next boot. Disable
   first.
6. **The compose-file rename disarms the manual paths on the tractor too.**
   With `/opt/lifetrac/video-test/docker-compose.yml` renamed, compose finds
   no file in that folder, whatever `-p` is used. Do not rename it back, not
   even for a session with a radio GO: the containers have
   `restart: unless-stopped`, so one successful `up` comes back at every
   boot. Deleted on 2026-10-14 (the last commit that has them is
   `cfb93d8`): `tmp_tractor_vtest_up.sh`, `tmp_recreate_both.sh`,
   `tmp_recreate_camera.sh`, `tmp_recreate_image_tx.sh`,
   `tmp_short_keyframe.sh`,
   `firmware/x8_lora_bootloader_helper/tractor_restart_video_test.sh`,
   `_tractor_flip.ps1`, `_tractor_flip2.ps1`, `_tractor_flip_remote.sh`.
7. **The base's copy is renamed by `deploy_base.sh`.** Since 2026-10-14
   `deploy_base.sh` ships neither `docker-compose.video-test.yml` nor
   `deploy/`, and it renames an old `docker-compose.video-test.yml` in the
   deployed tree to `docker-compose.video-test.yml.RETIRED-<PC UTC date>`
   (`-2`, `-3` … when that name is taken), as on the tractor: a rename,
   nothing deleted, done before it backs up the old tree, so no backup holds
   the file under its old name. It renames only while every container made
   from that file (compose project `lifetrac-vtest`, or that file among a
   container's compose config files) is exited. One in any other state
   (running, created, restarting, paused, dead), or a `docker ps` that
   fails, stops the deploy before anything changes; clean up as in [Doing
   the same on another board pair](#doing-the-same-on-another-board-pair),
   step 2. It never runs `docker compose` on that file. It also leaves the
   tractor's compose files in `firmware/tractor_x8/` out of the tarball
   (RS-5.20) and removes an old copy of them from the deployed tree. Until
   that first deploy the file on the base is armed: never bring it up. Its
   `image_rx` maps `/dev/ttymxc3`, sends a soft RESET_REQ on open that un-parks the
   L072, and has `restart: unless-stopped`, so after one successful `up` it
   comes back at every boot, with no unit involved, until the container is
   removed. Only a broker on `127.0.0.1:1883` blocks a manual `up` of it:
   the bench broker, or the desk broker while the
   [RS13_2_DESK_CHECK.md](RS13_2_DESK_CHECK.md) stack is up. Do not rename
   it back. Deleted on 2026-10-14 (the last commit that has them is
   `cfb93d8`): `tmp_base_up.sh`, `tmp_restart_image_rx.sh`,
   `tmp_hot_patch.sh`, `_bench_up.ps1`, and the two in rule 4.
8. **Build on the base only with `deploy_base.sh`.** It keeps the image it
   replaces as `lifetrac-v25:previous` and labels the new one with its
   commit. A plain `docker compose build` in the deployed tree that succeeds
   moves `lifetrac-v25:latest` off the flown image and keeps nothing.
   Deleted on 2026-10-14 (the last commit that has them is `cfb93d8`):
   `base_build_bg.sh`, `tmp_base_build.sh`, `tmp_base_prune_build.sh`,
   `_bench_build3.ps1`, `_bench_build4.ps1`.

### Doing the same on another board pair

This changes board configuration, so it needs the operator's approval. It
needs no radio GO: nothing opens `/dev/ttymxc3`.

**Step 1: retire the units with the script.** `provision_bench_board.sh` is
the one implementation of the disable and the move, and its guards refuse
(see above). From Git Bash on the PC, in `bench_tools/`:

```sh
bash provision_bench_board.sh base --ssh --check      # leave out --ssh when the base is on adb
bash provision_bench_board.sh tractor --check
```

Read both reports first: without `--check` the script applies every
provisioning step that reads `TODO`, not only this one
([BENCH_SETUP.md](BENCH_SETUP.md), step 6). Then run each line again
without `--check`. The item "old boot units retired" must read `ok` or
`applied`. If it reads `FAILED`, follow the printed message (rule 3) and run
the script again. Go on to step 2 only after the item has read `ok` or
`applied` on that board.

The scripts that ran by hand on 2026-10-12 are in
[`scripts/`](../../../bench-evidence/board_state_2026-10-12_unit_retirement/scripts/).
They are the record of that day and carry its names and dates; they are not
a tool for another pair.

**Step 2: remove the leftover containers by hand.** Run these as `fio` on
the board, the base over ssh and the tractor in `adb shell`. Each cleanup is
a shell function, so a guard that fails returns before anything is removed.
The commands and guards are those of the 10-12 scripts, except that
`units_gone` now accepts only the `is-enabled` replies the script's item
accepts (2026-10-14); wrapped in functions like this they have not been run
on a board. The container lists and the states the guards expect are those
of the original pair; if a guard aborts on another pair, stop and look.

On both boards, first:

```sh
export PATH=$PATH:/usr/sbin:/sbin
uart_chk() { sudo -n sh -c 'if command -v fuser >/dev/null; then fuser /dev/ttymxc3; echo; echo __UART_CHK_DONE__; else echo __UART_NO_FUSER__; fi' 2>&1; }
units_gone() {   # units_gone <unit>...: fails unless systemd finds no unit file for each unit (or a /dev/null mask), each is inactive or failed, and no wants link is left
  for u in "$@"; do
    e=$(systemctl is-enabled "$u" 2>&1); a=$(systemctl is-active "$u" 2>/dev/null)
    echo "$u: is-enabled=[$e] is-active=[$a]"
    case $e in
      "Failed to get unit file state for $u: No such file or directory"|not-found) ;;
      masked) [ "$(readlink "/etc/systemd/system/$u")" = /dev/null ] || { echo "ABORT: $u is masked, but not by a symlink to /dev/null"; return 1; } ;;
      *) echo "ABORT: systemd can still load $u"; return 1 ;;
    esac
    case $a in inactive|failed) ;; *) echo "ABORT: $u is not inactive or failed"; return 1 ;; esac
    if [ -L "/etc/systemd/system/multi-user.target.wants/$u" ]; then echo "ABORT: $u still has a wants link"; return 1; fi
  done
}
```

`uart_chk` is the marker form of the UART holder check. Only
`__UART_CHK_DONE__` with nothing before it means nobody holds the port.
`__UART_NO_FUSER__`, or no marker at all, means the check did not run.

**Base:**

```sh
base_cleanup() {
  units_gone lifetrac-base.service lifetrac-base-compose.service || return 1
  C="lifetrac-vtest-mosquitto-1 lifetrac-vtest-image_rx-1 lifetrac-vtest-web_ui-1 lifetrac-vtest-audit_tail-1"
  for c in $C; do   # guard: each container was created and never started
    s=$(sudo -n docker inspect -f '{{.State.Status}} started={{.State.StartedAt}} project={{index .Config.Labels "com.docker.compose.project"}}' $c 2>&1)
    echo "$c: $s"
    case $s in "created started=0001-01-01T00:00:00Z project=lifetrac-vtest") ;; *) echo "ABORT: unexpected state"; return 1 ;; esac
  done
  sudo -n docker rm $C && sudo -n docker network rm lifetrac-vtest_default    # no -f, no -v: the volumes stay
}
uart_chk
base_cleanup
# verify
systemctl --failed --no-legend --no-pager                  # expect only psplash-start.service
sudo -n docker ps --format '{{.Names}}\t{{.Status}}'       # design-controller-mosquitto-1 Up
uart_chk
```

The base's `docker-compose.video-test.yml` is renamed by the next
`deploy_base.sh` run (rule 7), which refuses while a container made from it
is in any state but exited, so run `base_cleanup` first.

**Tractor.** First save what the cleanup removes for good. From Git Bash on
the PC, with `TRACTOR_SERIAL` and `ARCHIVE_DIR` set as in `bench.env`
(`ARCHIVE_DIR` is a folder outside git). The commands used on 2026-10-12
were typed by hand and not kept; these are the equivalent, and they have not
been run:

```sh
S=${TRACTOR_SERIAL:?set TRACTOR_SERIAL first}
A=${ARCHIVE_DIR:?set ARCHIVE_DIR first}/board_state_$(date -u +%Y-%m-%d)/tractor
mkdir -p "$A"
MSYS_NO_PATHCONV=1 adb -s "$S" pull /var/rootdirs/opt/lifetrac/video-test "$(cygpath -m "$A")"
MSYS_NO_PATHCONV=1 adb -s "$S" shell 'cd /var/rootdirs/opt/lifetrac && find video-test -type f -exec sha256sum {} \;' | tr -d '\r' > "$A/video-test.board.sha256"
(cd "$A" && sha256sum -c video-test.board.sha256)     # every line must end in OK
for c in tractor-image-tx-v2 tractor-camera-v2 tractor-mosquitto-v2; do
  MSYS_NO_PATHCONV=1 adb -s "$S" shell "sudo -n docker inspect $c" | tr -d '\r' > "$A/inspect_$c.json"
done
```

Without `MSYS_NO_PATHCONV=1` Git Bash rewrites the board path into a
Windows path. Then, in `adb shell` on the tractor, set `DAY` to today's UTC
date as the PC shows it, for example `DAY=2026-10-12`. The tractor's clock
is not wall time, and `tractor_cleanup` refuses without `DAY`. Then:

```sh
U=lifetrac-tractor-compose.service
VT=/var/rootdirs/opt/lifetrac/video-test
camchk() { echo "lifetrac-camera.service=$(systemctl is-active lifetrac-camera.service) running=[$(sudo -n docker ps --format '{{.Names}}' | grep -x -E 'tractor-camera|tractor-camera-v2|tractor-image-tx-v2|camera_svc|tx_smoke' | tr '\n' ' ')]"; }
tractor_cleanup() {
  [ -n "${DAY:-}" ] || { echo "ABORT: set DAY first"; return 1; }
  units_gone $U || return 1
  C="tractor-image-tx-v2 tractor-camera-v2 tractor-mosquitto-v2"
  for c in $C; do   # guard: each container is Exited and belongs to project video-test
    s=$(sudo -n docker inspect -f '{{.State.Status}} project={{index .Config.Labels "com.docker.compose.project"}}' $c 2>&1)
    echo "$c: $s"
    case $s in "exited project=video-test") ;; *) echo "ABORT: unexpected state"; return 1 ;; esac
  done
  # disarm the compose file: a rename, nothing is deleted
  [ -f $VT/docker-compose.yml ] || { echo "ABORT: no $VT/docker-compose.yml"; return 1; }
  sudo -n mv $VT/docker-compose.yml $VT/docker-compose.yml.RETIRED-$DAY || { echo "ABORT: rename failed"; return 1; }
  (cd $VT && sudo -n docker compose -p tractor-vtest config --services 2>&1 | head -n 3)    # expect: no configuration file provided: not found
  # remove the three Exited containers and their two networks (no -f, no -v)
  sudo -n docker rm $C && sudo -n docker network rm video-test_default tractor-vtest_default
}
camchk; uart_chk
tractor_cleanup
# verify
systemctl --failed --no-legend --no-pager                  # expect only psplash-start.service
camchk; uart_chk
```

`camchk` must show the camera unit `inactive` and `running=[]`. If it ever
shows the camera unit active or a container running, apply the standard
bench stop before going on:
`sudo -n systemctl stop lifetrac-camera.service; sudo -n docker stop <name>`.
On 2026-10-12 there were no container logs to save: the tractor's docker
log driver is `journald`, and its journal holds only the current boot.

### Undo

Move the files back from `/etc/lifetrac-retired/<TS>/` to
`/etc/systemd/system/`, run `systemctl daemon-reload`, then
`systemctl enable <unit>`. On the tractor also rename the compose file
back, and on the base `docker-compose.video-test.yml` once `deploy_base.sh`
has renamed it (rule 7). The same unit text is in git at the commits named
in the table; the repo files have since gained a comment header. The board
copies are 480, 789 and 517 bytes, each larger than its git blob at those
commits by exactly its line count, so they have CRLF line endings: the
byte-exact originals are the retired files on the boards. The drop-in is in the
[10-04 capture](../../../bench-evidence/board_state_2026-10-04/base/files/etc/systemd/system/lifetrac-base.service.d/timeout.conf).
The removed containers cannot be restored exactly. `docker inspect` output
of the tractor's three is in the private archive and in the 10-10 capture.
The base's four had never started; they came from
`docker-compose.video-test.yml`, which is in git.

Do not undo any of this on a bench board. After a full undo on the tractor
(unit file back and enabled, compose file renamed back) the unit would
succeed and transmit at the next boot: the old containers that blocked it
are gone, so the rename is now the only barrier. With
`lifetrac-base-compose.service` enabled again and its compose file in place,
stopping the bench broker (as [RS13_2_DESK_CHECK.md](RS13_2_DESK_CHECK.md)
Step 3 does) would let it start `image_rx` at the next base boot.

### Not yet observed: the next boot

No board was rebooted on 2026-10-12. At the next power-up of each board, do
these checks first, before any probe, and record the output:

1. `bash provision_bench_board.sh base --ssh --check` and
   `bash provision_bench_board.sh tractor --check`. The item "old boot units
   retired" must read `ok`, with no info line about a unit or a leftover
   file. (The tractor's "radio UART free" item reads `TODO` until the camera
   is stopped, point 3.)
2. `sudo docker ps` on each board. On the base expect only
   `design-controller-mosquitto-1`, Up again through its `unless-stopped`
   restart policy. On the tractor expect the three `x8-*` containers, plus
   `tractor-camera` and `tractor-mosquitto` until the camera is stopped. No
   `lifetrac-vtest-*` and no `tractor-*-v2` container may be listed.
3. On the tractor `lifetrac-camera.service` comes up at boot, started by
   udev, and holds `/dev/ttymxc3`. That is the known fourth path (*Open
   items*), not a failed retirement. Stop it as at every power-up
   ([BENCH_SETUP.md](BENCH_SETUP.md#after-every-power-up)). After that,
   `uart_chk` (defined above) must print `__UART_CHK_DONE__` alone, on both
   boards.
4. `systemctl --failed` lists only `psplash-start.service`. On its own this
   proves little: a retired unit that had started successfully would not be
   listed either.

If a radio container is up (`image_rx`, `image_tx`, `lora_bridge` or a
`tractor-*-v2` container), `sudo docker stop` it by name at once. Then
record what ran and since when (`sudo docker ps -a`,
`journalctl -b -u <unit>`) before changing anything else. This is an open
item below.

## Where everything is saved

- **Repo:**
  - L072 source and bench binaries (PR #140);
  - provisioning scripts, unit files and the sudoers installer, now driven
    by [`provision_bench_board.sh`](provision_bench_board.sh) and
    [BENCH_SETUP.md](BENCH_SETUP.md);
  - every sketch source;
  - the base capture (this folder's sibling);
  - the capture tools [`capture_board_state.sh`](capture_board_state.sh)
    (read-only, secrets withheld) and [`pull_board_state.sh`](pull_board_state.sh).
- **Private archive, outside git, on this PC only:**
  `C:\Users\dorkm\Documents\LifeTrac-bench-archive\board_state_2026-10-04\`
  and `…\board_state_2026-10-10\` (the tractor capture, its
  `opt_lifetrac_no_secrets.tgz`, and image `pre-rs13` `9bfbbc8d06cb`, the
  only off-board copy), and `…\board_state_2026-10-12\` (the tractor's
  `/opt/lifetrac/video-test` tree as it was before its compose file was
  renamed, and the `docker inspect` output of the three removed containers
  and the two removed networks).
  - `images/`: base `lifetrac-v25` `4623980c2dac` and tractor
    `2727dfd36f9f` (docker save).
  - `base/`: the full capture tarball and the deployed tree without
    secrets.
  - `pc_only/`: the patched `libmbed_x8.a`, the LmP 934 bundle's sha256
    list (now also in T3a), and copies of the Claude and Copilot memory
    notes. Those notes were the only written record of several provisioning
    steps, which are now in BENCH_SETUP.md.
  - `scripts/`.

  **Make a second copy on an external disk or NAS.**
- **Keep in a password manager, never in git:**
  - the PC's gitignored `key.h` (a real fleet key);
  - `~/.ssh/lifetrac_base_ed25519`;
  - `~/.android/adbkey`;
  - the base's `secrets/`.

## Capture at a power-up (radios stay off)

1. As each board comes up, stop the tractor's `lifetrac-camera.service` and
   `tractor-camera`, and check that `fuser /dev/ttymxc3` is empty.
2. Stage only the probe tools (no `push_fix_to_board.sh`). Then run
   `radio_state.py`. At boot the L072 sits in **RXCONT**, listening only;
   `radio_park.py` puts it to `0x80` SLEEP.
3. Run `bash pull_board_state.sh <base|tractor> [--images]` (adb only). It
   writes to `ARCHIVE`, else `ARCHIVE_DIR` from `bench.env`: any folder
   outside git, in any path form (the script converts it for adb;
   [BENCH_QUICKSTART.md](BENCH_QUICKSTART.md) step 9). Review the
   reports for secrets before copying them into `bench-evidence/`; the
   2026-10-04 run caught a Dropbear host key, and the filter now excludes it.

## Open items

- [x] **Tractor capture.** Done 2026-10-10 with images, `pre-rs13`
  included (`board_state_2026-10-10/`).
- [ ] The base did not enumerate on USB/adb after the 2026-10-10 power cycle
  (ssh works). Check its USB cable and port.
- [x] **Decide on the tractor's `lifetrac-tractor-compose.service`.** Done
  2026-10-12: retired, its compose file disarmed and the three old
  `tractor-*-v2` containers removed
  ([record](../../../bench-evidence/board_state_2026-10-12_unit_retirement/README.md)).
- [ ] **Confirm the retirement at the next boot of each board,** with the
  four checks under
  [Not yet observed: the next boot](#not-yet-observed-the-next-boot). No
  board was rebooted on 2026-10-12.
- [x] **Decide on the base's `docker-compose.video-test.yml`** (rule 7).
  Decided with the operator's OK and done in the bench kit on 2026-10-14:
  `deploy_base.sh` no longer ships it or `deploy/`, and renames the copy on
  the base as on the tractor (TODO RS-5.18). Not yet run on the base: the
  file there stays armed until the next `deploy_base.sh` run.
- [ ] **Decide on the tractor's udev-started `lifetrac-camera.service`.** It
  is a fourth path that maps `/dev/ttymxc3` at boot: it reads "disabled",
  but the udev rule's `SYSTEMD_WANTS` starts it, and it is bound (`BindsTo`)
  to the C2 camera device. The bench stops it at every power-up. It needs
  its own operator decision.
- [ ] The tractor holds 335 anonymous docker volumes, probably leftovers of
  repeated mosquitto `docker run`s (cause unverified; only the count was
  captured). A cleanup candidate; it needs an OK first.
- [ ] Delete `/opt/lifetrac/video-test` on the tractor and the volumes
  `lifetrac-vtest_*` (base), `video-test_*` and `tractor-vtest_*` (tractor)
  later, only on an explicit OK. Deleting them is permanent.
- [ ] Delete the stored `5star` WiFi profile on the tractor once the password
  is rotated (`nmcli con delete 5star`). `provision_bench_board.sh tractor`
  backs up the stored WiFi profiles to a root-only directory on the board,
  then deletes them and prints only their names. Pass `--keep-wifi-profiles`
  to keep them until then.
- [ ] **H747 M4 bank 2: identify, then decide.** An openocd
  `dump_image` of banks 1 and 2 halts the H7 and drops the x8h7 bridge, and
  needs a deliberate reboot (FLASH_RUNBOOK precautions; never re-insmod x8h7
  on the base). **Explicit GO only.** Then upload an empty M4 sketch or erase
  bank 2, and record which.
- [ ] **Rotate the WiFi password.** It was committed in seven repo-root
  scripts on the public `main` (`connect_wpa.sh`, `do_wifi.sh`,
  `setup_5star_wifi.sh`, `wifi_connect.sh`, `wifi_connect_fast.sh`,
  `wifi_persist_and_reboot.sh`, `wifi_rescan_connect.sh`) and in
  `.vscode/tasks.json`.
  - [x] ~~Replace the literal with an environment variable.~~ Done in the
    bench kit (2026-10-10): the scripts take the SSID and passphrase from
    `LIFETRAC_WIFI_SSID` / `LIFETRAC_WIFI_PSK`, and the VS Code tasks that
    carried the passphrase were removed (it would appear in the task echo).
  - [ ] **Change the passphrase on the access point.** The old value is
    still in the public git history, so treat it as compromised. Rewriting a
    public history with forks is not practical; rotation is the fix.
- [x] **Decide on the base's `lifetrac-base.service` /
  `lifetrac-base-compose.service`.** Done 2026-10-12: both retired, with the
  `timeout.conf` drop-in, and the four never-started `lifetrac-vtest-*`
  containers removed
  ([record](../../../bench-evidence/board_state_2026-10-12_unit_retirement/README.md)).
- [ ] **Base docker store.** The broker container
  `design-controller-mosquitto-1` references image `sha256:2a23a3e427e4`,
  which is no longer in the base's image list (`eclipse-mosquitto:2` is now
  `0db28dca6f11`), so the layer store is not fully consistent. The first
  `deploy_base.sh` run is expected to recreate the broker container on the
  current image; its volume is kept. Before any image purge, make sure the
  flown image `4623980c2dac` is saved (a `docker save` of it is in the
  private archive).
- [ ] Optional, on GO: read back the L072 option bytes; their current values
  are recorded nowhere.
- [ ] Record the carrier ↔ module pairing, both carriers' DIP switch
  positions and the J-Link OB serials (1078222309 / 1078180658).
- [ ] Fix the doc errors the review found:
  - ~~T3a verdict row :66 says 2E2C for the 05-13 reflash; it was 2D0A.~~
    Fixed 2026-10-10. The row now matches the session log: `BOOT SEL` only,
    USB-C only, PID 0134.
  - `setup_x8_mqtt_link.ps1:9` names the wrong board.
  - HC-02:13 uses the old name `board1_healthcheck.sh`.
  - FIRMWARE_UPDATES.md:53 calls the handheld a Portenta H747.
  - BUILD-CONTROLLER step 2 would overwrite x8h7.
  - The 05-14 and 05-17 L072 unbrick notes disagree about which board.
