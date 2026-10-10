# Bench setup: from two new boards to a bench-ready pair

This page takes two new Portenta X8 boards on Max Carriers to the state the
LifeTrac radio bench runs in: OS image, board settings, firmware and docker
images. Work through it in order. Each step names the doc that has the details.

It reproduces the state measured on the original pair. The base was captured
in [`board_state_2026-10-04/base/`](../../../bench-evidence/board_state_2026-10-04/base/)
and the tractor in [`board_state_2026-10-10/tractor/`](../../../bench-evidence/board_state_2026-10-10/tractor/);
[BENCH_BOARDS.md](BENCH_BOARDS.md) describes both captures.

| step | what | where |
|---|---|---|
| 0 | Radio safety rules | below |
| 1 | Hardware | below |
| 2 | DIP switches | below |
| 3 | Choose the OS image | below |
| 4 | Reflash both boards | [T3a](../../../X8_HEALTH_AND_RECOVERY/recovery/T3a_sdp_uuu_reflash.md) |
| 5 | First boot: adb, password, base network, ssh key | below |
| 6 | Provision: `provision_bench_board.sh base`, then `tractor` | below |
| 7 | Verify: HC-01, HC-02, `--check` | below |
| 8 | L072 radio firmware, health probe, park | [FLASH_RUNBOOK.md](../FLASH_RUNBOOK.md) |
| 9 | Deploy the base tree, the images and the broker | [DEPLOY.md](DEPLOY.md) |
| 10 | Staging and radio legs | [BENCH_QUICKSTART.md](BENCH_QUICKSTART.md) |

You need a Windows PC set up as in [PC_SETUP.md](PC_SETUP.md): adb, Git Bash,
PowerShell, `py -3`, the Arm GCC toolchain, `mingw32-make` and `uuu`.
Run the shell commands on this page in **Git Bash**, from this directory
(`LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/bench_tools`),
unless a step says PowerShell.

Who is the base and who is the tractor: the **base** is on ethernet and is
reached over ssh or adb. The **tractor** has its WiFi off for good, has the
camera, and is reached over adb only (USB). Put the serials in `bench.env`
(step 5).

---

## 0. Radio safety (read this first)

The bench has two LoRa radios in the 902–928 MHz ISM band. The repo's
references are US rules (FCC §15.247); check your local rules before any
radio test.

1. **Antennas first.** Screw the antenna onto each carrier's LoRa connector
   before you power the board. Never let a radio transmit without one.
2. **The LifeTrac L072 firmware boots listening.** It powers up in RXCONT,
   which is receive-only. It transmits only when a host program on the X8
   asks it to: the bench daemons, the harness, or a probe that sends TX
   requests. If the radios must stay off, park them with
   `radio_park.py` (step 8). Its `0x80` readback is not a lasting guarantee
   while the firmware's scan is still running, so confirm a few minutes
   later with the read-only `radio_state.py`.
3. **Flashing the L072 turns its receiver on.** A probe that connects to
   it (rs116, radio_park, radio_state) also wakes it.
4. **The tractor's camera unit takes the radio UART.** At every boot, and
   whenever the camera is plugged in, `lifetrac-camera.service` and its
   `tractor-camera` container grab `/dev/ttymxc3`. Stop both before any
   probe or flash:
   `sudo -n systemctl stop lifetrac-camera.service; sudo -n docker stop tractor-camera`.
   `sudo -n fuser /dev/ttymxc3` must then print nothing.
5. **The base does not stay off.** After `systemctl poweroff` the base
   booted again about 4.5 minutes later (2026-10-04). To keep it down,
   remove its power. The tractor stays off after `systemctl poweroff`.
6. **Never start the production units on the bench.** These are
   `lifetrac-base.service`, `lifetrac-base-compose.service` and
   `lifetrac-tractor-compose.service`. They would start `lora_bridge` or the
   tractor stack on the radio UART. This kit does not install them. The
   original pair has them enabled, and they fail at every boot.
7. **Radio tests need a GO.** On a shared bench, every round of radio
   tests needs the operator's explicit GO, and both radios are parked
   after it.

## 1. Hardware

| item | qty | notes |
|---|---|---|
| Arduino Portenta X8 (ABX00049) | 2 | The OS, eMMC, H747 and SE050 belong to the module. |
| Arduino Portenta Max Carrier (ABX00043) | 2 | Carries the Murata CMWX1ZZABZ LoRa module (STM32L072 + SX1276), the BOOT DIP switches and an on-board J-Link. The L072 firmware belongs to the **carrier**. |
| 915 MHz antenna | 2 | Bench parts: Taoglas TD.95.6H31 Blade (868/915 MHz dual-band dipole, SMA), one on each carrier's LoRa antenna connector. |
| 12 V DC supply | 2 | One per carrier (barrel jack). |
| USB-A to USB-C data cable | 2 | Goes into the X8 module's own USB-C port. For a reflash, plug it straight into the PC: no hub, and no USB-C to USB-C cable. |
| Ethernet cable + LAN with DHCP | 1 | For the base. A marginal cable at gigabit forced the optional 100BASE-TX pin (step 6). |
| Kurokesu C2 USB camera (16d0:0ed4) | 1 | Goes on the tractor carrier's USB-A host port. Needed only for camera legs. |
| Windows 10/11 PC | 1 | Set up as in [PC_SETUP.md](PC_SETUP.md). The PC has no Docker: images are built on the base X8. |
| Optional: a second USB cable per carrier | 2 | For the carrier's own USB-C (on-board J-Link, `VID_1366&PID_0105`). Only the T2.5 and T4 recovery steps use it. |

Label each module and each carrier with its serial, and write down which
module sits on which carrier. Recovery depends on that pairing: the L072
firmware, its option bytes and the DIP switches stay with the carrier.

## 2. DIP switches

Each Max Carrier has a `BOOT SEL` and a `BOOT` DIP switch.

- **Normal boot: both OFF.** Check this on new carriers before first power.
- **Reflash (SDP mode):** set `BOOT SEL` ON. If the SDP device does not
  appear, power down and set `BOOT` ON as well. Both reflashes on this bench
  needed only `BOOT SEL`. See [T3a](../../../X8_HEALTH_AND_RECOVERY/recovery/T3a_sdp_uuu_reflash.md).
- After the reflash, set both back to OFF.

Write down each carrier's switch positions, as BENCH_BOARDS asks.

## 3. Choose the OS image

| image | kernel | URL | status |
|---|---|---|---|
| **LmP 4.0.11-934-91** | 6.1.24 | `https://downloads.arduino.cc/portentax8image/934.tar.gz` (1,097,240,589 B, Last-Modified 2025-08-11) | **The base image.** Flashed with `uuu` 2026-05-13 and 05-24. Pinned by sha256 in T3a. **Use this on both boards.** |
| LmP 4.0.3-674-88 | 5.10.93 | `https://downloads.arduino.cc/portentax8image/674.tar.gz` (854,998,055 B) | The tractor's factory image; that board was never reflashed. It is still hosted at this URL, but nobody here has downloaded or byte-checked it, so there is no sha256 on record. |
| `image-latest.tar.gz` | — | `https://downloads.arduino.cc/portentax8image/image-latest.tar.gz` | **Not the bench image.** This URL moves to newer builds. The bench bundle was downloaded under this name on 2026-05-08, when it still served 934. |

The 934 bundle used on the base has this sha256:
`2f2065c7b10ce33d8444a138353187c507727b5109ddca325506ecaea1efe221`.
It was saved as `image-latest.tar.gz`. Today's `934.tar.gz` has the same
size and date, but nobody has compared it byte for byte. If your download
hashes the same, it is identical. If not, check the extracted files against
the full list in T3a; those files are what `uuu` writes.

**Recommendation for a new bench: 934 on both boards.** That gives one image
to keep and one set of quirks. Be aware that **the tractor role has only ever
run on 674**. Treat the first tractor-on-934 legs as a new configuration and
record it in the evidence. The 934 differences that matter for the tractor
are known and handled:

- The flash wrapper must reboot instead of re-inserting `x8h7`, which crashes
  the 6.1 kernel. `REVIVE_MODE` now defaults to `reboot` in the flash wrapper, and
  FLASH_RUNBOOK passes it explicitly too.
- The compose-apps recovery unit wipes `/var/lib/docker`.
  `provision_bench_board.sh` masks it.
- The factory docker images differ. The probe image comes from the repo's
  `foundries_python.tar`, which `provision_bench_board.sh` loads.

To match the original pair exactly, keep the tractor on its factory 674.
That works only if the board still runs 674; check
`cat /etc/os-release` once you have adb.

## 4. Reflash both boards

Follow [T3a_sdp_uuu_reflash.md](../../../X8_HEALTH_AND_RECOVERY/recovery/T3a_sdp_uuu_reflash.md).
It covers the download and sha256 check, the extracted layout, `uuu` 1.5.243,
the DIP switches, SDP detection and the first boot. Budget 60–80 minutes per
board for the eMMC write.

Reflash even brand-new boards: the factory image depends on the production
batch. A reflash erases the whole eMMC (OS, `/etc`, docker images,
`/home/fio`). It does not touch the L072 firmware on the carrier.

## 5. First boot

1. Set both DIP switches OFF. Connect 12 V, then USB-C. First boot takes
   about 60 s. If the LEDs go red or adb has not appeared after 2 minutes,
   power-cycle once more; the 2026-05-13 reflash needed two cycles.
2. Check the board is visible: `adb devices -l` must list its serial as
   `device`. **Do not run `adb kill-server`.** On a first-boot board adbd is
   fragile, and a kill-server can leave it invisible until you replug the
   USB-C ([ADB_TIPS_AND_TRICKS.md](../../../X8_HEALTH_AND_RECOVERY/recovery/ADB_TIPS_AND_TRICKS.md)).
3. **Record the serials.** Copy [`bench.env.example`](bench.env.example) to
   `bench.env` in this directory and set `BASE_SERIAL`, `TRACTOR_SERIAL`
   and `BASE_HOST`, plus `BENCH_SUDO_PW` if you change the password.
   `bench.env` is your local file: keep it out of git.
4. **Password.** The image's user is `fio` with password `fio`; adb opens a
   shell as `fio` without asking for it. Changing it is recommended on the
   base, which sits on your LAN with ssh. Run `adb -s <serial> shell`, then
   `passwd`. If you change it, put it in `bench.env` as `BENCH_SUDO_PW`.
   The provisioning script uses it once, to install the sudoers drop-in;
   after that `sudo -n` needs no password.
5. **Sudo without a password.** Step 6 installs this with the repo's
   [`install_lifetrac_nopasswd.sh`](../install_lifetrac_nopasswd.sh), which
   writes `/etc/sudoers.d/99-lifetrac-bench-nopasswd` =
   `fio ALL=(ALL) NOPASSWD: ALL`. It is bench-only: it gives anyone with a
   shell as `fio` full root. To install it by hand instead:

   ```bash
   export MSYS_NO_PATHCONV=1
   sed 's/\r$//' ../install_lifetrac_nopasswd.sh > /tmp/install_nopasswd.sh   # LF only
   adb -s <serial> push "$(cygpath -m /tmp/install_nopasswd.sh)" /tmp/install_lifetrac_nopasswd.sh
   adb -s <serial> shell   # then on the board: sudo sh /tmp/install_lifetrac_nopasswd.sh
   ```

6. **Base network.** Plug the base into the LAN and read its address:
   `adb -s <base serial> shell ip -br addr show eth0`. Reserve that DHCP
   lease in your router (the original bench uses `192.168.1.117`) and set
   `BASE_HOST` in `bench.env`.
7. **Base ssh key.** Create a key on the PC, in Git Bash:

   ```bash
   ssh-keygen -t ed25519 -f ~/.ssh/lifetrac_base_ed25519 -C lifetrac-bench-pc
   ```

   Step 6 appends `~/.ssh/lifetrac_base_ed25519.pub` to the base's
   `/home/fio/.ssh/authorized_keys`. Afterwards this must work:
   `ssh -i ~/.ssh/lifetrac_base_ed25519 fio@<BASE_HOST> uptime`. Back up
   the private key in a password manager, never in git.
8. **Tractor network: none, on purpose.** Leave its ethernet unplugged.
   Step 6 turns its WiFi off for good.

## 6. Provision

[`provision_bench_board.sh`](provision_bench_board.sh) runs on the PC and
applies every board-side customization the original pair has. It reads
`bench.env` and talks to one board over adb. Every step checks first, so it
is safe to re-run. `--check` reports without changing anything. Do the base
first, because the tractor's broker image comes from it.

```bash
bash provision_bench_board.sh base --check       # what is missing
bash provision_bench_board.sh base               # add --nic-100 only for a marginal cable
bash provision_bench_board.sh tractor --check
bash provision_bench_board.sh tractor --mosquitto-from-base
```

What it applies, and why:

| step | board | why |
|---|---|---|
| sudoers drop-in (`install_lifetrac_nopasswd.sh`) | both | Every bench script uses `sudo -n`. |
| mask `compose-apps-early-start-recovery.service` and `compose-apps-early-start.service` | base, and the tractor on any image other than 674 | On 934 `compose-apps-early-start` fails at boot. Its `OnFailure` recovery unit then runs `systemctl stop docker` and deletes `/var/lib/docker`, about every 60 s, so every image you load disappears. Mask both before relying on docker. On 674 the unit succeeds and is left alone (`--mask-compose` masks it anyway). |
| optional `--nic-100`: `Wired connection 1` set to speed 100, duplex full, auto-negotiate yes | base | The original base's cable loses about half its RX frames at gigabit and none at 100. Auto-negotiation stays on, as on the base, so it only advertises 100/full. Skip this on a good cable. To undo it: `sudo nmcli connection modify 'Wired connection 1' 802-3-ethernet.speed 0 802-3-ethernet.duplex '' 802-3-ethernet.auto-negotiate yes`. |
| PC ssh key appended to `~/.ssh/authorized_keys` | base | Base work goes over ssh. Its USB has failed to enumerate after a power cycle (2026-10-10) while ssh kept working. |
| [`units/disable-wifi.service`](units/disable-wifi.service) installed and enabled, which runs `rfkill block wifi` at boot | tractor | The tractor has no IP path by policy. This is the unit text from the tractor capture. |
| `wpa_supplicant.service` masked; `nmcli radio wifi off` | tractor | Same reason. |
| stored NetworkManager WiFi profiles deleted (only their names are printed) | tractor | The original tractor stores the `5star` profile with its password. `--keep-wifi-profiles` keeps them. |
| camera USB provisioning: runs the repo's [`provision_x8.sh`](../provision_x8.sh) unchanged | tractor | Installs the `snd_usb_audio` blacklist, the C2 udev rules (autosuspend off, `/dev/lifetrac-c2`) and `lifetrac-camera.service`, and adds `fio` to `video`. The script skips this when all three files already match. It refuses when a camera compose app is installed and the C2 is attached, because `provision_x8.sh` would restart the camera, whose container maps the radio UART; pass `--allow-camera-restart` to accept that. |
| docker running; probe image `hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44` loaded from the repo-root `foundries_python.tar` (image id `9a454afe48f7`) | both | `radio_park.py`, `radio_state.py` and `rs116_health_probe.py` run in it. The tractor cannot pull images, and the base's own `lifetrac-v25` image exists only after step 9. |
| `eclipse-mosquitto:2` | base: `docker pull`. Tractor: `--mosquitto-from-base` (docker save on the base, then through the PC) or `--mosquitto-tar FILE` | The base broker and the tractor's local broker. The tag moves, so note the image id the script prints. |
| `lifetrac-camera.service` and `tractor-camera` stopped | tractor | Keeps the radio UART free (rule 4). Both come back at the next boot. |

It never opens `/dev/ttymxc3` and never starts the production units. It
reports any of them it finds, and who holds the radio UART. It ends with a
checklist of what is left. Exit codes: 0 done, 1 a step failed, 2 usage or
board unreachable, 3 `--check` found work to do. Run
`bash provision_bench_board.sh --help` for all options.

It does not touch the L072 firmware or option bytes, the H747, the deployed
tree or the production units. It does not rotate the WiFi password
committed in the old repo-root scripts.

## 7. Verify

1. **HC-01 (adb and USB enumeration, PowerShell):**
   [HC-01](../../../X8_HEALTH_AND_RECOVERY/routines/HC-01_adb_and_usb_enumeration.md).
   Each serial must show `device`, stable over 30 s.
2. **HC-02 (Linux, x8h7 bridge, GPIO, UART):**
   [HC-02](../../../X8_HEALTH_AND_RECOVERY/routines/HC-02_linux_bridge_full.md).
   The helper is [`x8_max_carrier_healthcheck.sh`](../x8_max_carrier_healthcheck.sh).
   On the tractor, stop the camera unit first (rule 4).

   ```bash
   export MSYS_NO_PATHCONV=1
   sed 's/\r$//' ../x8_max_carrier_healthcheck.sh > /tmp/hc.sh
   adb -s <serial> push "$(cygpath -m /tmp/hc.sh)" /tmp/x8_max_carrier_healthcheck.sh
   adb -s <serial> shell "sh /tmp/x8_max_carrier_healthcheck.sh"
   ```

   Its L072 section resets the UART's line settings and sends a two-byte
   ROM-bootloader probe (`00 FF` at 19200 8E1). That does not reach the
   radio, and a running firmware ignores it, but do not run it while
   anything else is using `/dev/ttymxc3`. Expected on a fresh board: the x8h7
   modules load (934 lists 10), `m4-proxy` and `stm32h7-program` are active,
   and `/dev/ttymxc3` and `/dev/watchdog0` exist. A
   `/sys/kernel/x8h7_firmware/version` timeout is normal. `gpio8/10/15` are
   exported only after a flash-pipeline run.
3. `bash provision_bench_board.sh <role> --check` must report no TODO.
   After a reboot, the tractor's camera line reads TODO again until you stop
   the camera.

## 8. L072 radio firmware

A new carrier's L072 normally runs Arduino's stock AT firmware.
`LifeTrac-v25/tools/mlm32l07x01.bin` is that image (MKRWAN 1.2.3), kept for
a factory restore. The same-named file in `x8_lora_bootloader_helper/` is a
different build. The stock firmware does not answer the LifeTrac probes
until you flash the LifeTrac build.

1. **Build** in PowerShell, in `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072`:
   `mingw32-make bench` makes `build/firmware_bench_diag.bin`, md5
   `0c1bb0a9573f813137f941dfa47177d0`. Use arm-none-eabi-gcc 12.2.1
   (Arm GNU Toolchain 12.2.MPACBTI-Rel1); the toolchain and the expected md5
   are in [`murata_l072/README.md`](../../murata_l072/README.md). The
   committed `build/firmware.bin` (md5 `589c120323c2d5e7ef9f459d7a4ba42d`)
   is the production image. Do not put it on a bench board: it refuses the
   bench's diagnostic register writes.
2. **Flash** with [FLASH_RUNBOOK.md](../FLASH_RUNBOOK.md), which uses
   `run_flash_bench.sh` with `REVIVE_MODE=reboot`. U-Boot arms the X8 watchdog
   (60 s, then a PMIC power-cycle); the runbook says why the scripts pet it
   and reboot. **Flashing turns the receiver on** (rule 3).
3. **Health probe and park.** Stage the probe files to
   `/tmp/lifetrac_strict` and run them in the probe image. The scripts in
   [`legs/`](legs/README.md) automate this; by hand, for the tractor:

   ```bash
   export MSYS_NO_PATHCONV=1; S=<serial>; H=$(cygpath -m "$PWD/..")
   adb -s $S shell "sudo -n systemctl stop lifetrac-camera.service; sudo -n docker stop tractor-camera; mkdir -p /tmp/lifetrac_strict"
   for f in method_g_stage1_probe.py method_h_stage2_tx_probe_v2.py rs116_health_probe.py bench_tools/radio_park.py bench_tools/radio_state.py; do
     adb -s $S push "$H/$f" /tmp/lifetrac_strict/ >/dev/null; done
   R="sudo -n docker run --rm --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44"
   adb -s $S shell "$R -u /work/rs116_health_probe.py"   # STATS-OK, RS115-INSTRUMENTED-FIRMWARE=YES, RS12-URC-COUNTERS=YES, radio_state=4
   adb -s $S shell "$R -u /work/radio_park.py"           # PARK_OK {"opmode_readback": "0x80"}
   adb -s $S shell "$R -u /work/radio_state.py"          # read-only; repeat a few minutes later
   ```

   Do the same on the base, over adb or ssh; it has no camera to stop.
   `/tmp` is tmpfs: a reboot (and so every flash) wipes the staging, and
   `systemd-tmpfiles` also ages it out after 5 days.

## 9. Deploy

[DEPLOY.md](DEPLOY.md) covers it:

- **Base** (`deploy_base.sh`): it copies a `git archive` subset of a commit
  to `/var/rootdirs/opt/lifetrac/DESIGN-CONTROLLER`, builds `lifetrac-v25`
  there, and starts the broker `design-controller-mosquitto-1` on
  `127.0.0.1:1883`. It never starts `lora_bridge`.
- **Tractor** (`build_tractor_image.sh`): it builds `lifetrac-tractor-x8`
  natively on the base X8, then moves it with `docker save` and
  `docker load`. The PC has no Docker.

## 10. Staging and radio legs

Continue with [BENCH_QUICKSTART.md](BENCH_QUICKSTART.md), which covers
staging, a first leg and parking. The other leg docs are:

- the parameterised leg scripts and the power-up guard: [`legs/README.md`](legs/README.md);
- the per-session checklist: [BENCH_RUNBOOK.md](BENCH_RUNBOOK.md);
- the RS-13 procedure: [RS13_VECTOR_LEG.md](RS13_VECTOR_LEG.md).

## After every power-up

- **Tractor:** stop `lifetrac-camera.service` and `tractor-camera`, then
  check that `sudo -n fuser /dev/ttymxc3` prints nothing.
- **Both:** `/tmp` staging is gone; stage again before any probe or flash.
- **Radios:** read them with `radio_state.py`. If they must stay off and
  the readback is not `0x80`, run `radio_park.py`.
- **Base:** if it does not show up on USB, use ssh
  (`provision_bench_board.sh base --ssh --check` works over ssh).

## Troubleshooting

| symptom | cause | fix |
|---|---|---|
| Docker images disappear about every minute on a 934 board | The compose-apps recovery unit deletes `/var/lib/docker` | `provision_bench_board.sh <role>` masks it |
| A probe prints `timeout waiting for response` on the tractor | The camera unit or container holds `/dev/ttymxc3` | Rule 4 |
| A flash fails with `1: image` (rc 127) | A CRLF script, or staging that aged out of `/tmp` | Strip CR (`sed 's/\r$//'`) and stage again ([FLASH_RUNBOOK.md](../FLASH_RUNBOOK.md) §1) |
| `adb devices` is empty right after a first boot | adbd is fragile on first boot | Wait, then replug the USB-C; do not `adb kill-server` ([T0](../../../X8_HEALTH_AND_RECOVERY/recovery/T0_adb_daemon_kick.md)) |
| `provision_bench_board.sh` stops at the sudoers step | Wrong `BENCH_SUDO_PW` | Fix it in `bench.env` |
| `$'\r': command not found` when you start a kit script on the PC | git `core.autocrlf` checked the script out with CRLF | The repo's `.gitattributes` keeps `*.sh` LF. On an older checkout, run `git add --renormalize .` or `sed -i 's/\r$//' <script>` |
| The board is unreachable or hung | — | [X8_HEALTH_AND_RECOVERY](../../../X8_HEALTH_AND_RECOVERY/README.md) recovery tiers T0–T4 |
