# Bench boards: chips, firmware and board-resident software

State as of **2026-10-04**. The base is measured from its capture
([`bench-evidence/board_state_2026-10-04/base/`](../../../bench-evidence/board_state_2026-10-04/base/)).
The tractor is from records only: it was powered off and is not captured yet
(see *Open items*). The record review behind this page covered AI NOTES,
bench-evidence, X8_HEALTH_AND_RECOVERY and git history.

## The two boards

| | base | tractor |
|---|---|---|
| serial (adb) | `2D0A1209DABC240B` | `2E2C1209DABC240B` |
| hardware | Portenta X8 on a Portenta Max Carrier | same |
| OS (LmP) | **4.0.11-934-91**, kernel 6.1.24-lmp-standard (uuu reflash 2026-05-13 and 05-24; bundle sha256 in the private archive) | **4.0.3-674-88**, kernel 5.10.93-lmp-standard (factory, never reflashed). **No copy of the 674 image exists.** A reflash would move it to 934/6.1.24. |
| access | adb; ethernet 192.168.1.117 with eth0 pinned to 100BASE-TX full (marginal cable); ssh key `lifetrac-bench-pc` | adb only (WiFi off by policy) |
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
| STM32H747 Cortex-M7 (flash bank 1) | X8 module | **stock Arduino x8h7** SPI bridge, programmed at boot by `stm32h7-program.service` from `/usr/arduino/extra/STM32H747AII6_CM7.bin` (base: sha256 `d81eaa81…`) | no | Part of the LmP image | LifeTrac overwrote it with `tractor_h7` at `0x08040000` on both boards on 2026-05-04. It has been stock again since at least 05-13. The version string cannot be read: `/sys/kernel/x8h7_firmware/version` times out, so `program-h7.sh`'s "matches, No Update" is not proof. |
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

### Tractor (from records; capture pending)

- **`provision_x8.sh`** (2026-05-15) installed:
  - `/etc/udev/rules.d/99-w2-01-c2.rules`, which starts
    `lifetrac-camera.service` on camera hotplug;
  - `/etc/modprobe.d/lifetrac-no-usb-audio.conf`;
  - `/etc/systemd/system/lifetrac-camera.service`. This is the helper
    version, `firmware/x8_lora_bootloader_helper/lifetrac-camera.service`,
    not `tractor_x8/systemd/`.

  **It grabs `/dev/ttymxc3`, the radio UART, at every boot.** Stop it before
  any probe.
- **WiFi off:**
  - `nmcli radio wifi off` plus rfkill;
  - `99-disable-wifi.conf`;
  - a masked `wpa_supplicant` and a oneshot `disable-wifi.service`, neither
    in git;
  - possibly a stored WiFi profile with its password.
- **`/var/rootdirs/opt/lifetrac`:**
  - `compose-apps/lifetrac-camera`, which runs `lifetrac-tractor-x8:latest`
    with `/dev/video1` and `/dev/ttymxc3`;
  - the hand-edited May `video-test` stack;
  - `bin/ffmpeg`.
- **Images:**
  - `lifetrac-tractor-x8:latest` `2727dfd36f9f` (archived);
  - **`lifetrac-tractor-x8:pre-rs13` `9bfbbc8d06cb`, on the board only**;
  - the Arduino OOTB `x8-devel`, `x8-provisioning` and `x8-webapp`;
  - `eclipse-mosquitto:2`.

  The probe image `arduino-ootb-python-devel:738bc44` is the repo-root
  `foundries_python.tar`.
- **`/home/fio`.** Unique files:
  - `wdog_regs.py`;
  - `lifetrac_compact_at_probe.sh`;
  - `base_station/` (contents unknown);
  - the 09-15 flash logs.

## Where everything is saved

- **Repo:**
  - L072 source and bench binaries (PR #140);
  - provisioning scripts, unit files and the sudoers installer;
  - every sketch source;
  - the base capture (this folder's sibling);
  - the capture tools [`capture_board_state.sh`](capture_board_state.sh)
    (read-only, secrets withheld) and [`pull_board_state.sh`](pull_board_state.sh).
- **Private archive, outside git, on this PC only:**
  `C:\Users\dorkm\Documents\LifeTrac-bench-archive\board_state_2026-10-04\`.
  - `images/`: base `lifetrac-v25` `4623980c2dac` and tractor
    `2727dfd36f9f` (docker save).
  - `base/`: the full capture tarball and the deployed tree without
    secrets.
  - `pc_only/`: the patched `libmbed_x8.a`, the LmP 934 bundle's sha256
    list, and copies of the Claude and Copilot memory notes, which are the
    only written record of several provisioning steps.
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
3. Run `bash pull_board_state.sh <base|tractor> [--images]`. Review the
   reports for secrets before copying them into `bench-evidence/`; the
   2026-10-04 run caught a Dropbear host key, and the filter now excludes it.

## Open items

- [ ] **Tractor capture.** It needs a power cycle, then step 3 with
  `--images` (for `pre-rs13`, which exists only on the board).
- [ ] **H747 M4 bank 2: identify, then decide.** An openocd
  `dump_image` of banks 1 and 2 halts the H7 and drops the x8h7 bridge, and
  needs a deliberate reboot (FLASH_RUNBOOK precautions; never re-insmod x8h7
  on the base). **Explicit GO only.** Then upload an empty M4 sketch or erase
  bank 2, and record which.
- [ ] **Rotate the WiFi password.** It is committed in seven repo-root
  scripts on the public `main` (`connect_wpa.sh`, `do_wifi.sh`,
  `setup_5star_wifi.sh`, `wifi_connect.sh`, `wifi_connect_fast.sh`,
  `wifi_persist_and_reboot.sh`, `wifi_rescan_connect.sh`). Then replace the
  literal with an environment variable.
- [ ] **Decide on the base's `lifetrac-base.service` /
  `lifetrac-base-compose.service`.** Both are enabled and fail at every boot;
  they would start `lora_bridge` on the radio UART if they ever succeeded.
- [ ] Optional, on GO: read back the L072 option bytes; their current values
  are recorded nowhere.
- [ ] Record the carrier ↔ module pairing, both carriers' DIP switch
  positions and the J-Link OB serials (1078222309 / 1078180658).
- [ ] Fix the doc errors the review found:
  - T3a verdict row :66 says 2E2C for the 05-13 reflash; it was 2D0A.
  - `setup_x8_mqtt_link.ps1:9` names the wrong board.
  - HC-02:13 uses the old name `board1_healthcheck.sh`.
  - FIRMWARE_UPDATES.md:53 calls the handheld a Portenta H747.
  - BUILD-CONTROLLER step 2 would overwrite x8h7.
  - The 05-14 and 05-17 L072 unbrick notes disagree about which board.
