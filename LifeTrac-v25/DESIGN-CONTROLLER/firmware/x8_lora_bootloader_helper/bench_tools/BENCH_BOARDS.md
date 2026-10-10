# Bench boards: chips, firmware and board-resident software

State as of **2026-10-10**. Both boards are measured from read-only
captures:
- base: [`bench-evidence/board_state_2026-10-04/base/`](../../../bench-evidence/board_state_2026-10-04/base/);
- tractor: [`bench-evidence/board_state_2026-10-10/tractor/`](../../../bench-evidence/board_state_2026-10-10/tractor/).

The rest comes from a review of AI NOTES, bench-evidence,
X8_HEALTH_AND_RECOVERY and git history.

**To bring up a new pair to this state, see [BENCH_SETUP.md](BENCH_SETUP.md).**
It covers hardware, the OS image pinned by URL and sha256, the reflash,
first boot, verification and the L072 firmware.
[`provision_bench_board.sh`](provision_bench_board.sh) applies the `/etc`
and docker changes listed under *LifeTrac software living in the X8 Linux*
below, except the production units, which it never installs. `--check`
reports how far a board is from that state.

## The two boards

| | base | tractor |
|---|---|---|
| serial (adb) | `2D0A1209DABC240B` | `2E2C1209DABC240B` |
| hardware | Portenta X8 on a Portenta Max Carrier | same |
| OS (LmP) | **4.0.11-934-91**, kernel 6.1.24-lmp-standard (uuu reflash 2026-05-13 and 05-24). The bundle is `https://downloads.arduino.cc/portentax8image/934.tar.gz`; its sha256 list is in [T3a](../../../X8_HEALTH_AND_RECOVERY/recovery/T3a_sdp_uuu_reflash.md). | **4.0.3-674-88**, kernel 5.10.93-lmp-standard (factory, never reflashed). Arduino still hosts the image at `https://downloads.arduino.cc/portentax8image/674.tar.gz` (854,998,055 B). Nobody here has downloaded or byte-verified it, so there is no sha256 on record. Reflashing to 934 would move it to 6.1.24; BENCH_SETUP §3 compares the two. |
| access | adb; ethernet 192.168.1.117 with eth0 pinned to 100BASE-TX full (marginal cable); ssh key `~/.ssh/lifetrac_base_ed25519`. On 2026-10-10 it did not enumerate on USB after a power cycle, but ssh worked. | adb only (WiFi off by policy) |
| power-down | `systemctl poweroff` did **not** keep it off: it booted again about 4.5 min later (2026-10-04). Remove power to keep it down. | stayed off after `systemctl poweroff` |
| clock | NTP | no time source; it lags by hours to days |

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
  - `lifetrac-base.service` and `lifetrac-base-compose.service` are
    **enabled but fail at every boot**. If one ever succeeded, compose would
    start `lora_bridge` on `/dev/ttymxc3`.
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
  unless-stopped). Leftovers: `lifetrac-vtest-*` (Created), `rx_smoke`,
  `bench_mqtt`, `f11_webui` (Exited).
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
  - `lifetrac-tractor-compose.service`, **enabled, failing at boot**;
  - `disable-wifi.service`, enabled: an rfkill block at boot;
  - `wpa_supplicant.service` masked;
  - `fio` added to `video` / `docker`;
  - `docker/key.json`.

  All of these were installed by `provision_x8.sh` (2026-05-15) and the May
  WiFi-off steps. The `disable-wifi.service` unit is now in git as
  [`units/disable-wifi.service`](units/disable-wifi.service), copied from
  the capture. `provision_bench_board.sh tractor` installs and enables it,
  masks `wpa_supplicant`, and reruns `provision_x8.sh` when the camera
  files differ.
- **WiFi:** soft-blocked and disabled in NetworkManager. **A stored
  `5star.nmconnection` holds the WiFi password** (not copied).
- **`/var/rootdirs/opt/lifetrac`:**
  - `compose-apps/lifetrac-camera`, which runs `lifetrac-tractor-x8:latest`
    with `/dev/video1` and `/dev/ttymxc3`;
  - the hand-edited May `video-test` stack;
  - `bin/ffmpeg`.
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
  only off-board copy).
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
- [ ] **Decide on the tractor's `lifetrac-tractor-compose.service`.** It is
  enabled and fails at every boot, like the base's units.
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
- [ ] **Decide on the base's `lifetrac-base.service` /
  `lifetrac-base-compose.service`.** Both are enabled and fail at every boot;
  they would start `lora_bridge` on the radio UART if they ever succeeded.
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
