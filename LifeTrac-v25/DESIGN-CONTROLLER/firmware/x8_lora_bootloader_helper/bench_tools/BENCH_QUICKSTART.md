# Radio bench quickstart

Start here to reproduce the LifeTrac v25 LoRa bench from GitHub: two
Portenta X8 boards on Max Carriers talking over their Murata L072 radios, as
in the RS-12 and RS-13 campaigns. This page lists the steps in order with the
commands. The linked documents explain each step.

**The GO rule.** Radios run only on an explicit operator GO, given for each
round. Without that GO, nothing that opens `/dev/ttymxc3` runs: no flash
(the L072 boots into RXCONT, so a flash turns the receiver on), no probe, no
channel spot-check, no leg. There is one exception: `radio_park.py` and
`radio_state.py`, which put a radio to sleep or check that it is asleep.
Power-up, staging, deploys and image builds do not touch the radios.

## What you need

- **Boards.** Two Arduino Portenta X8 (ABX00049), each on a Portenta Max
  Carrier (ABX00043). The carrier holds the Murata CMWX1ZZABZ (L072 + SX1276).
  You also need a 12 V supply per carrier and a USB-C data cable from each
  board straight to the PC (no hub).
- **Antennas.** A 915 MHz antenna on each carrier's LoRa SMA (the bench uses
  Taoglas TD.95.6H31). Never key a radio without one.
- **Network.** Ethernet for the **base** to a LAN with DHCP and internet; it
  builds the Docker images. The **tractor** has WiFi off and is reached over
  adb only.
- **Camera legs.** A USB UVC camera on the tractor (it enumerates as
  `/dev/video1`), aimed at the PC screen.
- **PC.** Windows 11 on the same LAN as the base. Set it up with
  [PC_SETUP.md](PC_SETUP.md).

Board roles on this bench: the **base** is `2D0A1209DABC240B` and the
**tractor** is `2E2C1209DABC240B`. [BENCH_BOARDS.md](BENCH_BOARDS.md) lists
what is on every chip.

## 0. PC setup

Follow [PC_SETUP.md](PC_SETUP.md): Git for Windows (long paths), Python
through `py -3` plus [`requirements-pc.txt`](requirements-pc.txt), the Arm GNU
Toolchain 12.2.MPACBTI-Rel1, WinLibs `mingw32-make`, adb, and optionally `uuu`
and arduino-cli.

The commands below run in **Git Bash from the repo root**, except those marked
PowerShell. They use these shell variables. The values shown are this bench's;
replace them with yours from `bench.env` (step 4).

```bash
export MSYS_NO_PATHCONV=1                       # adb from Git Bash: no MSYS path rewriting
DC=LifeTrac-v25/DESIGN-CONTROLLER
BT=$DC/firmware/x8_lora_bootloader_helper/bench_tools
BASE="ssh -i $HOME/.ssh/lifetrac_base_ed25519 fio@192.168.1.117"   # base: ethernet + ssh
TRAC="adb -s 2E2C1209DABC240B shell"                                # tractor: adb only
R="docker run --rm --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho"
BI=lifetrac-v25:latest                                              # base daemon image
TI=hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44       # tractor probe image
```

## 1. Boards (once per board)

[BENCH_SETUP.md](BENCH_SETUP.md) covers this step:

- the OS image, pinned by URL and sha256 (base LmP 4.0.11-934-91; the tractor
  runs factory 4.0.3-674-88);
- reflashing through
  [T3a](../../../X8_HEALTH_AND_RECOVERY/recovery/T3a_sdp_uuu_reflash.md) if
  you need to;
- first boot;
- [`provision_bench_board.sh`](provision_bench_board.sh), which installs the
  sudoers drop-in, masks the compose early-start units on the base **before
  any docker use**, and on the tractor adds WiFi-off
  ([`units/disable-wifi.service`](units/disable-wifi.service)) and the camera
  unit handling.

## 2. Radio firmware (L072)

Both boards run the bench build **`0c1bb0a9`**. The production
`build/firmware.bin` (`589c1203`) refuses the bench carrier pin, so do not put
it on a bench board.

```powershell
# PowerShell (mingw32-make does not work from Git Bash)
cd LifeTrac-v25\DESIGN-CONTROLLER\firmware\murata_l072
mingw32-make check
mingw32-make bench
(Get-FileHash build\firmware_bench_diag.bin -Algorithm MD5).Hash   # 0C1BB0A9573F813137F941DFA47177D0
```

A different md5 means a different toolchain; see the toolchain section of
[murata_l072/README.md](../../murata_l072/README.md). If you have no
toolchain, the same bytes are committed as
[`firmware_bench_diag_0c1bb0a9.bin`](../../../bench-evidence/RS_13_vector_scene_2026-09-26/firmware/).

To flash (**needs the GO**), push the tooling LF-clean and run
`REVIVE_MODE=reboot` exactly as [FLASH_RUNBOOK.md](../FLASH_RUNBOOK.md) §1–§4
says. Success is `Verify OK` and `flash_rc=0` in
`/home/fio/pipeline_stamped.log`. The flash reboots the board, which wipes
`/tmp`, so stage again afterwards (step 5).

## 3. Board software

[DEPLOY.md](DEPLOY.md) explains both scripts and their arguments:

- [`deploy_base.sh`](deploy_base.sh) puts a commit's `DESIGN-CONTROLLER`
  subset on the base and builds `lifetrac-v25:latest` there. It never starts
  `lora_bridge`. The base broker `design-controller-mosquitto-1` must be
  running.
- [`build_tractor_image.sh`](build_tractor_image.sh) builds
  `lifetrac-tractor-x8` natively on the base, then loads it onto the offline
  tractor. It also loads the two images the tractor cannot pull:
  `foundries_python.tar` (repo root) and `eclipse-mosquitto:2`.

Wheel versions are pinned in the `requirements.lock.txt` files next to each
`requirements.txt`.

## 4. Configure `bench.env`

Copy [`bench.env.example`](bench.env.example) to `bench.env` as its header
says, then fill in your values: the board serials, the base address, the ssh
key path, the board password and the PC's LAN address. The `legs/` scripts
read it through [`lib/bench_env.sh`](lib/bench_env.sh). Do not commit
`bench.env`.

## 5. Every session

1. **Power-up radio safety.** Start the guard, then power the boards:

   ```bash
   bash "$BT/legs/power_up_guard.sh"
   ```

   As each board comes up, the guard stops the tractor's
   `lifetrac-camera.service` and the `tractor-camera` container (both grab
   `/dev/ttymxc3` at every boot) and reports any UART holder. It sends nothing
   to the radio. Each L072 boots into RXCONT, which only listens. If the
   radios must stay quiet, park them (step 8).
2. **Staging.**

   ```bash
   bash "$BT/legs/stage_boards.sh"
   ```

   This re-pushes `/tmp/lifetrac_strict` on both boards. Run it after any
   reboot (and every flash reboots) or when the last push is 5 days old,
   because `systemd-tmpfiles` ages `/tmp`. Afterwards,
   `ls /tmp/lifetrac_strict | wc -l` must be at least 19. Flash staging in
   `/tmp/lifetrac_p0c` must count 11 files (FLASH_RUNBOOK).
3. **Moving scene (camera legs).** Play the railroad video in a **normal**
   Firefox window of a separate profile, with the player's own fullscreen
   (Esc leaves it). **Never use `firefox --kiosk`**: a kiosk locked the
   operator out of the PC. Before **every** camera run, check the scene: at
   least 8 % of pixels must change over 10 s (`legs/scene_check.sh`; see
   [legs/README.md](legs/README.md)).
4. **Channel spot-check (needs the GO; receive only) and the DTS carrier
   pin.** Park the tractor, then sniff the candidate channel on the base,
   where the known emitter is about 20 dB hotter:

   ```bash
   scp -i $HOME/.ssh/lifetrac_base_ed25519 $DC/firmware/x8_lora_bootloader_helper/channel_survey_sniff.py fio@192.168.1.117:/tmp/lifetrac_strict/
   $BASE "sudo docker rm -f rx_smoke 2>/dev/null; sudo $R -e LIFETRAC_REG_PROFILE=2 $BI -u /work/channel_survey_sniff.py --start-hz 927500000 --stop-hz 927500000 --step-hz 500000 --dwell-s 45 --interval-s 0.05" | tee spot_$(date +%F).txt
   py -3 $DC/tools/survey_compare.py spot_$(date +%F).txt
   ```

   A hot sample rules a channel out. A clean one does not prove the link;
   only the leg does. To sweep instead of spot-checking, use
   `--start-hz 902500000 --stop-hz 927500000 --dwell-s 30`, which takes about
   26 minutes. Pass `-ForceFrfHz <the pick>` on **every** profile-2 leg and
   record it in RESULTS. The default, `-ForceFrfHz 0`, tunes 915.000 MHz,
   which is the channel of an external emitter (RS-11.6). Picks hold for
   hours, not days.

## 6. Run a leg (needs the GO)

[legs/README.md](legs/README.md) lists the per-leg prep and post scripts and
their arguments. They are parameterised copies of RS-13's `leg_prep.sh` and
`leg_post.sh`:

- **prep:** scene check, health probes, `clear_retained.py`, pre-brackets,
  base-side capture;
- **post:** post-brackets, reports, park.

Then launch the harness,
[`run_live_radio_monitor.ps1`](../run_live_radio_monitor.ps1), from
PowerShell in the helper directory. This is RS-13 leg 2a, VECTOR at boot on
DTS:

```powershell
cd LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper
.\run_live_radio_monitor.ps1 -TxFeed camera -RegProfile 2 -ForceFrfHz 927500000 -DurationS 300 -SynthFps 2 `
   -KfRequestDisable 1 -ProbeEcho 0 -LogFragArrivals 1 -TxBatch 0 `
   -CamExtraEnv "-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80" -HostIp <this PC's LAN IP> -Archive
```

- 2b is the same leg on FHSS (`-RegProfile 1`, no `-ForceFrfHz`).
- 2c is the `mono_g4` control (`-CamExtraEnv "-e LIFETRAC_ENCODE_MODE=6"`).
- 2d is the over-the-air switch.

[RS13_VECTOR_LEG.md](RS13_VECTOR_LEG.md) gives each leg and the pass criteria
P1–P8. Start the session's `RESULTS.md` from
[RS13_RESULTS_TEMPLATE.md](RS13_RESULTS_TEMPLATE.md). For RS-12-style tile
legs, see [BENCH_RUNBOOK.md](BENCH_RUNBOOK.md), "A leg". The archive goes to
`DESIGN-CONTROLLER/bench-evidence/radio_monitor_<stamp>_<sha>/`, and its
`params.txt` records what actually ran.

## 7. Analyse

The tools are
[`rs12_leg_report.py`](../../../tools/rs12_leg_report.py),
[`frag_gap_report.py`](frag_gap_report.py) and
[`vector_dry_run.py`](../../../tools/vector_dry_run.py). Run them from
`DESIGN-CONTROLLER`. From Git Bash, give them Windows-style `C:/...` paths,
not `/c/...`:

```powershell
py -3 tools/rs12_leg_report.py <archive> --pre <legs>/leg<X>_pre_base.txt --post <legs>/leg<X>_post_base.txt --capture <legs>/leg<X>_base.jsonl
py -3 firmware/x8_lora_bootloader_helper/bench_tools/frag_gap_report.py <archive>
py -3 tools/vector_dry_run.py replay <legs>/leg<X>_base.jsonl --profile image_bw500
py -3 tools/vector_dry_run.py tractor-log <archive>/camera_service.log
```

Then run `legs/leg_replay.py` for the post-lock sequence gaps and the P1
resync episodes. Its arguments are in [legs/README.md](legs/README.md).

The loss of record is the report's `capture seq gaps` line. The counter
`loss` line can be stale (RS13_VECTOR_LEG A14) or frozen by a dead stats
thread (RS-12.17).

## 8. Park and power down

About 2 minutes after the daemons stop, park both radios with
[`radio_park.py`](radio_park.py). The firmware's scan needs that long to fail
out.

```bash
$BASE "sudo $R $BI -u /work/radio_park.py"      # want PARK_OK, both readbacks 0x80
$TRAC "sudo -n $R $TI -u /work/radio_park.py"
```

`PARK_TRANSIENT` (exit 5) means the scan was still walking: wait 60 s and
repeat. The park readback does not guarantee the radio stays off, so before
you call the bench off, re-check both boards with the read-only
[`radio_state.py`](radio_state.py) (same command line).

To power down:

- **tractor:** `$TRAC "sudo -n systemctl poweroff"`; it stays off.
- **base:** `$BASE "sudo systemctl poweroff"`, then **remove its power**.
  Otherwise it boots again about 4.5 minutes later.

## 9. Capture board state

```bash
ARCHIVE=<a folder outside git> bash "$BT/pull_board_state.sh" tractor   # add --images for docker saves
ARCHIVE=<a folder outside git> bash "$BT/pull_board_state.sh" base
```

The capture is read-only and withholds secrets. First stop the tractor's
camera unit (step 5.1). Review the reports before you copy text into
`bench-evidence/`, and never commit images or secrets. See
[BENCH_BOARDS.md](BENCH_BOARDS.md), "Capture at a power-up".

## Troubleshooting

[BENCH_RUNBOOK.md](BENCH_RUNBOOK.md) has the board facts and the full trap
list. These are the common symptoms:

| symptom | cause | fix |
|---|---|---|
| probe or daemon gets no HostLink answer although the L072 transmits | tractor camera unit or container holds `/dev/ttymxc3` | `systemctl stop lifetrac-camera.service; docker stop tractor-camera`; `fuser /dev/ttymxc3` must be empty |
| flash fails with `run_flash_bench.sh: line 5: 1: image` (rc 127) | `/tmp/lifetrac_p0c` aged out, or a script was pushed with CRLF | re-push LF-clean, count 11 files (FLASH_RUNBOOK §1) |
| `ERR_PROTO FORBIDDEN detail=6` on `-ForceFrfHz` or a survey | production firmware on the board | flash the bench build (step 2) |
| DTS loss folds on a ~7.08 s period | the leg flew on 915.000 MHz | spot-check and pin `-ForceFrfHz` (step 5.4) |
| camera changes mode or a profile is commanded mid-leg | retained control topics | `clear_retained.py` on the base broker **and** `clear_retained_host.py` on the PC broker |
| report says ~99.8 % loss and `published=1` | stats thread died (RS-12.17) | use `capture seq gaps` / count `published frame_id` lines |
| docker images vanish on the base | `compose-apps-early-start-recovery.service` is unmasked: it runs `rm -rf /var/lib/docker` every 60 s | mask it ([BENCH_SETUP.md](BENCH_SETUP.md)) |
| board resets about 60 s into a long operation | the U-Boot-armed X8 watchdog | FLASH_RUNBOOK §3; close `/dev/watchdog0` with `V` |
| a harness parameter seems ignored | misspelled name | the harness now rejects unknown names; read the archive's `params.txt` |
| adb lost the board after `adb kill-server` | the board's adbd hung | run `adb -s <serial> wait-for-device` once; if that fails, replug USB-C or press RESET. Avoid `kill-server` while the boards are enumerating ([ADB_TIPS_AND_TRICKS.md](../../../X8_HEALTH_AND_RECOVERY/recovery/ADB_TIPS_AND_TRICKS.md)) |
