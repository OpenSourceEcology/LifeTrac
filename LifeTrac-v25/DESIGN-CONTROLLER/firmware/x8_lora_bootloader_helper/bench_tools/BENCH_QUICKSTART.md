# Radio bench quickstart

Start here to reproduce the LifeTrac v25 LoRa bench from GitHub: two
Portenta X8 boards on Max Carriers talking over their Murata L072 radios, as
in the RS-12 and RS-13 campaigns. This page lists the steps in order with the
commands. The linked documents explain each step.

**The GO rule.** Radios run only on an explicit operator GO, given for each
round. Without that GO, nothing that opens `/dev/ttymxc3` runs: no flash
(the L072 boots into RXCONT, so a flash turns the receiver on), no probe
(the L072 section of the HC-02 health check writes to that UART too,
[BENCH_SETUP.md](BENCH_SETUP.md) §7), no channel spot-check, no leg. There is
one exception: `radio_state.py`, which reads whether a radio is asleep, and
`radio_park.py`, which puts it to sleep. The power-up guard runs both on its
own (step 5.1). Apart from the guard's `radio_state.py` / `radio_park.py`,
power-up, staging, deploys and image builds do not touch the radios.

## What you need

- **Boards.** Two Arduino Portenta X8 (ABX00049), each on a Portenta Max
  Carrier (ABX00043). The carrier holds the Murata CMWX1ZZABZ (L072 + SX1276).
  You also need a 12 V supply per carrier and a USB-A to USB-C data cable from
  each X8 straight to the PC: no hub, and no USB-C to USB-C cable
  ([BENCH_SETUP.md](BENCH_SETUP.md) §1).
- **Antennas.** A 915 MHz antenna on each carrier's LoRa SMA (the bench uses
  Taoglas TD.95.6H31). Never key a radio without one.
- **Network.** Ethernet for the **base** to a LAN with DHCP and internet; it
  builds the Docker images. The **tractor** has WiFi off and is reached over
  adb only.
- **Camera legs.** A USB UVC camera on the tractor (it enumerates as
  `/dev/video1`), aimed at the PC screen.
- **PC.** Windows 11 on the same LAN as the base. Set it up with
  [PC_SETUP.md](PC_SETUP.md).

On the original bench the **base** is `2D0A1209DABC240B` and the **tractor**
is `2E2C1209DABC240B`; those are the values in `bench.env.example`. Your
boards' serials go into `bench.env` (step 1).
[BENCH_BOARDS.md](BENCH_BOARDS.md) lists what is on every chip of the
original pair.

## 0. PC setup

Follow [PC_SETUP.md](PC_SETUP.md): Git for Windows (long paths), Python
through `py -3` plus [`requirements-pc.txt`](requirements-pc.txt), the Arm GNU
Toolchain 12.2.MPACBTI-Rel1, WinLibs `mingw32-make`, adb, a PowerShell
execution policy that lets the kit's `.ps1` scripts run
([PC_SETUP.md](PC_SETUP.md#powershell-execution-policy)), and optionally
`uuu` and arduino-cli.

## 1. Configure `bench.env`

Every bench script reads `bench_tools/bench.env`: provisioning, the L072
flash, the deploys and the leg scripts. Create it now, before any script
touches a board. A script that finds no `bench.env` falls back to the
original bench's serials and base address (`192.168.1.117`), which on your
LAN may be some other host.

The commands on this page run in **Git Bash from the repo root**, except
those marked PowerShell. Start every Git Bash window with:

```bash
export MSYS_NO_PATHCONV=1                       # adb from Git Bash: no MSYS path rewriting
DC=LifeTrac-v25/DESIGN-CONTROLLER
BT=$DC/firmware/x8_lora_bootloader_helper/bench_tools
```

Create the file once, then edit it:

```bash
cp "$BT/bench.env.example" "$BT/bench.env"      # gitignored; never commit it
```

Fill in, following the comments in the file:

- `BASE_SERIAL`, `TRACTOR_SERIAL`: the adb serials. On new boards you read
  them at first boot ([BENCH_SETUP.md](BENCH_SETUP.md) §5, before
  provisioning).
- `BASE_HOST`: the base's reserved DHCP lease. `BASE_SSH_KEY` only if your
  key is not `~/.ssh/lifetrac_base_ed25519`.
- `BENCH_SUDO_PW` if you changed the `fio` password.
- `PC_HOST` (**required** for legs): this PC's LAN IPv4 address as the boards
  see it. The harness gets it as `-HostIp` and the base rx daemon uses it as
  its control broker.
- `BASE_TRANSPORT`: `adb` (radio legs need the base on USB) or `ssh`.
- `EVIDENCE_DIR`: where the leg scripts write their records. Left empty it
  is `bench-evidence/RS_13_vector_scene_<UTC date>/legs`. Give each session
  a new dated folder: since 2026-10-22 the leg scripts refuse a folder that
  already holds files tracked by git (a committed evidence record), and warn
  when the last date in its path is not today (step 6).

Check what the scripts will use:

```bash
bash -c '. "$0/lib/bench_env.sh" && bench_show_env' "$BT"
```

Then load the board values into this shell. The commands below use these
variables. Paste the block into each new Git Bash window, and again after you
edit `bench.env`:

```bash
# board values from bench.env, read in a subshell (the loader is not meant for an interactive shell)
eval "$(bash -c '. "$0/lib/bench_env.sh" || exit 1
  for v in BASE_SERIAL TRACTOR_SERIAL BASE_HOST BASE_SSH_USER BASE_SSH_KEY BASE_IMAGE TRACTOR_PROBE_IMAGE ARCHIVE_DIR; do
    printf "%s=%q\n" "$v" "${!v}"; done' "$BT")"
BASE="ssh -i $BASE_SSH_KEY $BASE_SSH_USER@$BASE_HOST"   # base: ethernet + ssh
TRAC="adb -s $TRACTOR_SERIAL shell"                     # tractor: adb only
R="docker run --rm --network=host --entrypoint python3 --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work -w /work -e PYTHONPATH=/work:/work/paho"
BI=$BASE_IMAGE                                          # base daemon image
TI=$TRACTOR_PROBE_IMAGE                                 # tractor probe image
```

## 2. Boards (once per board)

[BENCH_SETUP.md](BENCH_SETUP.md) covers this step:

- the OS image, pinned by URL and sha256 in T3a: LmP 4.0.11-934-91 on both
  boards. The original tractor still runs its factory 4.0.3-674-88; keep 674
  only to match that pair exactly (BENCH_SETUP §3);
- reflashing through
  [T3a](../../../X8_HEALTH_AND_RECOVERY/recovery/T3a_sdp_uuu_reflash.md),
  brand-new boards included, because the factory image depends on the
  production batch (BENCH_SETUP §4);
- first boot, where the serials and the base address go into `bench.env`;
- [`provision_bench_board.sh`](provision_bench_board.sh), which installs the
  sudoers drop-in, masks the compose early-start units on the base **before
  any docker use**, loads the probe image from the repo-root
  `foundries_python.tar` on both boards, and on the tractor adds WiFi-off
  ([`units/disable-wifi.service`](units/disable-wifi.service)), the camera
  unit handling and `eclipse-mosquitto:2` (`--mosquitto-from-base`).

## 3. Radio firmware (L072)

Both boards run the bench build **`e083ae46`** (since 2026-10-18). The
production `build/firmware.bin` refuses the bench carrier pin, so do not put
it on a bench board. The source now builds the RS-12.21 image (scan option
(b) + L + d2, the firmware park command, the 0x0D split): bench
**`0daae279`**, production `2c5e055a` (25 324 B, built 2026-10-25). It is
**built, not flashed**; flashing it needs its own GO
([Expected md5s](../../murata_l072/README.md#expected-md5s)). The flash and
the checks after it are planned step by step in
[FHSS_SCAN_FAILED_OPTIONS.md](../../murata_l072/FHSS_SCAN_FAILED_OPTIONS.md)
§6.1. On that image the VER capability bitmap reads `0x000000FF` (step 8:
the firmware park).

```powershell
# PowerShell (mingw32-make does not work from Git Bash)
cd LifeTrac-v25\DESIGN-CONTROLLER\firmware\murata_l072
mingw32-make check
mingw32-make bench
(Get-FileHash build\firmware_bench_diag.bin -Algorithm MD5).Hash   # 0DAAE27978089EB97AB35C18718B62B0 (e083ae46 before RS-12.21)
```

A different md5 means a different toolchain; see the toolchain section of
[murata_l072/README.md](../../murata_l072/README.md). If you have no
toolchain, the same bytes are committed as
[`firmware_bench_diag_e083ae46.bin`](../../../bench-evidence/RS_12_15_reverse_delivery_2026-10-18/firmware_flash/firmware_bench_diag_e083ae46.bin).
The previous build, `0c1bb0a9` (on both boards 2026-09-15 → 2026-10-18), is
the rollback image:
[`firmware_bench_diag_0c1bb0a9.bin`](../../../bench-evidence/RS_13_vector_scene_2026-09-26/firmware/).

Flash with [`flash_l072.sh`](flash_l072.sh), which is
[FLASH_RUNBOOK.md](../FLASH_RUNBOOK.md) §1–§2 as one script and reads
`bench.env`. One board at a time:

```bash
FW=$DC/firmware/murata_l072/build/firmware_bench_diag.bin
bash "$BT/flash_l072.sh" tractor "$FW"        # stage LF-clean + preflight only: want PREFLIGHT-OK
bash "$BT/flash_l072.sh" tractor "$FW" --go   # NEEDS THE GO: stops the camera unit, then flashes
```

Then the same with `base`. Without `--go` the script copies files and runs
the wrapper's preflight; it touches no radio.

- **New carriers** still run Arduino's stock MKRWAN firmware. Work through
  [FLASH_RUNBOOK.md §5](../FLASH_RUNBOOK.md#5-first-flash-from-a-factory-board),
  "First flash from a factory board", first. It starts with a read-only
  first contact (also on the GO):
  `bash "$BT/flash_l072.sh" <board> LifeTrac-v25/tools/mlm32l07x01.bin --go --verify-only`,
  then flashes the bench build as above.
- **Success** is `Verify OK` and `flash_rc=0` in
  `/home/fio/pipeline_stamped.log`. The flash reboots the board
  (`REVIVE_MODE=reboot`), which wipes `/tmp`, and the L072 comes back
  listening in RXCONT.
- **Afterwards** run the health probe and park of FLASH_RUNBOOK §4; the
  commands are in [BENCH_SETUP.md](BENCH_SETUP.md) §8, step 3.
  `bash "$BT/legs/power_up_guard.sh" --only <board>` does the park part on
  its own: it stages the probe tools, reads the radio and parks it.

## 4. Board software

[DEPLOY.md](DEPLOY.md) explains both scripts and their arguments:

- [`deploy_base.sh`](deploy_base.sh) puts a commit's `DESIGN-CONTROLLER`
  subset on the base and builds `lifetrac-v25:latest` there. It never starts
  `lora_bridge`. The base broker `design-controller-mosquitto-1` must be
  running.
- [`build_tractor_image.sh`](build_tractor_image.sh) builds
  `lifetrac-tractor-x8` natively on the base, then loads it onto the offline
  tractor. The two images the tractor cannot pull came with provisioning
  (step 2): `provision_bench_board.sh tractor` loads the probe image from the
  repo-root `foundries_python.tar` and, with `--mosquitto-from-base`,
  `eclipse-mosquitto:2`. `build_tractor_image.sh --with-mosquitto` can reload
  mosquitto.

Wheel versions are pinned in the `requirements.lock.txt` files next to each
`requirements.txt`.

## 5. Every session

1. **Power-up radio safety.** Arm the guard, then switch the boards on:

   ```bash
   bash "$BT/legs/power_up_guard.sh" --wait-power-cycle   # boards already up: leave out the flag
   ```

   As each board comes up, the guard stops the tractor's
   `lifetrac-camera.service` and the `tractor-camera` container (both grab
   `/dev/ttymxc3` at every boot) and reports any UART holder. It then stages
   the probe tools, reads each L072 with `radio_state.py`, and parks any
   radio that is not in `0x80` SLEEP with `radio_park.py` (3 tries, 60 s
   apart). Those two scripts are the GO-rule exception; the guard never
   transmits. Each L072 boots into RXCONT, which only listens, and the guard
   is what puts it to sleep. `--no-park` only reads; `--check-only` is the
   end-of-session check (step 8).
2. **Staging.**

   ```bash
   bash "$BT/legs/stage_boards.sh"
   ```

   This re-pushes `/tmp/lifetrac_strict` on both boards. Run it after any
   reboot (and every flash reboots) or when the last push is 5 days old,
   because `systemd-tmpfiles` ages `/tmp`. It writes its md5 record into
   `EVIDENCE_DIR`, so it stops with `REFUSED: EVIDENCE_DIR=...` (exit 7),
   before any board is touched, when that folder holds committed files:
   set a new folder in `bench.env` first (step 1). Afterwards,
   `ls /tmp/lifetrac_strict | wc -l` must be at least 19 (23 on a fresh full
   staging, `channel_survey_sniff.py` included). The flash staging
   in `/tmp/lifetrac_p0c` is separate: `flash_l072.sh <board> <bin>` without
   `--go` re-stages it and runs the wrapper's preflight, and `PREFLIGHT-OK`
   means it is complete.
3. **Moving scene (camera legs).** Play the railroad video in a **normal**
   Firefox window of a separate profile, with the player's own fullscreen
   (Esc leaves it). **Never use `firefox --kiosk`**: a kiosk locked the
   operator out of the PC. Before **every** camera run, check the scene: at
   least 8 % of pixels must change over 10 s (`legs/scene_check.sh`; see
   [legs/README.md](legs/README.md)).
4. **Channel spot-check (needs the GO; receive only) and the DTS carrier
   pin.** The tractor must be parked. The guard parked it at power-up; park
   it again if anything has woken it since (a probe, a flash, a leg prep):

   ```bash
   $TRAC "sudo -n $R $TI -u /work/radio_park.py"     # want PARK_OK, both readbacks 0x80
   ```

   Then sniff the candidate channel on the base, where the known emitter is
   about 20 dB hotter. `CAND` is the carrier you want to try, in Hz: inside
   902250000..927750000, and not 915000000.

   ```bash
   CAND=<candidate carrier in Hz>
   # channel_survey_sniff.py is staged by legs/stage_boards.sh (step 2); after a
   # --probes-only or power_up_guard.sh staging run: bash "$BT/legs/stage_boards.sh" base
   $BASE "sudo docker rm -f rx_smoke 2>/dev/null; sudo $R -e LIFETRAC_REG_PROFILE=2 $BI -u /work/channel_survey_sniff.py --start-hz $CAND --stop-hz $CAND --step-hz 500000 --dwell-s 45 --interval-s 0.05" | tee spot_$(date +%F).txt
   py -3 $DC/tools/survey_compare.py spot_$(date +%F).txt
   ```

   A hot sample rules a channel out. A clean one does not prove the link;
   only the leg does. To sweep instead of spot-checking, use
   `--start-hz 902500000 --stop-hz 927500000 --dwell-s 30`, which takes about
   26 minutes. Picks hold for hours, not days.

   **Write the pick into `bench.env`** and record it in RESULTS:

   ```bash
   DTS_CARRIER_HZ=<the pick, in Hz>
   DTS_CARRIER_DATE=<today, YYYY-MM-DD>
   ```

   `leg_prep.sh` refuses a profile-2 leg without `DTS_CARRIER_HZ`, on
   915.000 MHz, and on any day other than `DTS_CARRIER_DATE`. It puts the
   pick into the harness command as `-ForceFrfHz`. The harness itself
   refuses `-RegProfile 2` without `-ForceFrfHz`, because the profile
   default, 915.000 MHz, is the channel of an external emitter (RS-11.6).
   `-AllowDefaultCarrier` overrides that refusal; use it only for a
   deliberate 915 MHz control leg.

## 6. Run a leg (needs the GO)

[legs/README.md](legs/README.md) lists the per-leg scripts and their
arguments. They are parameterised copies of RS-13's `leg_prep.sh` and
`leg_post.sh`:

- **prep:** scene check, health probes, `clear_retained.py`, pre-brackets,
  base-side capture. It ends by writing the harness command for this leg;
- **post:** post-brackets, reports (each written whole to `legs/`), the
  archive's reset and injector logs, the RS-13 scorers and
  `bracket_deltas.py`, park.

RS-13 leg 2a, VECTOR at boot on DTS, goes like this:

```bash
bash "$BT/legs/leg_prep.sh" 2a image_bw500    # prints the wrapper path (--print-harness: preview, touches no board)
```

```powershell
# PowerShell, right away: the base capture is already listening.
# leg_prep.sh printed the exact path of the wrapper.
& "<BENCH_SCRATCH>/leg2a_harness.ps1"
```

```bash
bash "$BT/legs/link_sample.sh" 2a     # while frames flow
bash "$BT/legs/leg_post.sh" 2a        # after the harness prints "[EVIDENCE] archived to ..."
```

The wrapper runs
[`run_live_radio_monitor.ps1`](../run_live_radio_monitor.ps1) from the
helper directory with the serials, `-HostIp` and `-ForceFrfHz` from
`bench.env`. It tees the transcript and records the archive folder for
`leg_post.sh`. If PowerShell says that running scripts is disabled, see
[PC_SETUP.md](PC_SETUP.md#powershell-execution-policy).

The harness line carries `-StopDrainS` (since 2026-10-22): 4 s on FHSS
(`image_bw250`), 2 s on DTS (`image_bw500`). `--stop-drain S` (0 < S ≤ 30)
overrides it; fly both codecs of a round (its `mono_g4` and VECTOR legs) with
the same value, and the prep prints a NOTE when it is not the default.

**Evidence folder** (2026-10-22). `leg_prep.sh` refuses an `EVIDENCE_DIR`
that already holds files tracked by git, a committed evidence record: it
prints `REFUSED: EVIDENCE_DIR=...` and exits 7 before any board action
(`--print-harness` shows the same verdict and still writes the preview).
Point `EVIDENCE_DIR` in `bench.env` at a new folder. To append to a
committed record on purpose, put `BENCH_EVIDENCE_APPEND=1` in front of
**every** script of the leg (`BENCH_EVIDENCE_APPEND=1 bash ...`): the
follow-up scripts refuse the folder without it, and the follow-up lines
the prep prints then carry it. It is an environment setting only; a
`BENCH_EVIDENCE_APPEND` line in `bench.env` is ignored with a WARN. A
folder whose last date is not today gets a WARNING, not a refusal.
`leg_post.sh` never refuses the folder: it parks both radios at the end, so
it only prints a WARNING into its transcript and goes on to the park; move
the files afterwards.

The harness output ends with `=== LEG CHECKS ===`, just above the
`[EVIDENCE]` line. `TRACTOR_L072_RESET landed` is the normal line; `failed`
or `unknown` means score the tractor brackets with
`bench_tools/bracket_deltas.py` (reset-aware; `leg_post.sh` writes
`leg<X>_deltas.txt`; BENCH_RUNBOOK, "A leg"). `BASE_L072_RESET unverified`
prints on every leg until the RS-5.2 base follow-up is done.

Since the bench kit of 2026-10-19 (not yet flown) the harness ends a leg by
stopping the frame source, the injector, then tx_smoke (after `-StopDrainS`)
and rx_smoke, and only then reads the daemon logs. LEG CHECKS adds
`CLOCKS board minus PC: ...` (read only; the offsets are in `params.txt`) and
a `LOG_WINDOW` line, checked since 2026-10-22:

- green `LOG_WINDOW after_stop (...)`: both daemon stops worked and both
  daemon logs hold their SIGTERM shutdown line (`params.txt`:
  `log_window=after_stop`);
- red `LOG_WINDOW after_stop_unverified (...)`: a daemon stop was not
  confirmed. That daemon may have run while its log was read, and may still
  run: `leg_post.sh` stops before the brackets and the park while a leg
  daemon runs (exit 3); `bash "$BT/legs/power_up_guard.sh" --stop-leg-daemons`
  stops it and parks, then re-run `leg_post.sh`;
- yellow `LOG_WINDOW after_stop_unverified (...)`: both stops worked, but a
  daemon log has no shutdown line. The daemon may have died before the stop
  (look for a traceback in its final log above LEG CHECKS), or its log did
  not come back whole: the log ends early.

An unverified window puts `log_window=after_stop_unverified` and the reason
(`log_window_detail=`) in `params.txt`, with a hint line under LOG_WINDOW.
`feed_stop=` records the frame source's own stop. It does not change the
colour: if the LOG_WINDOW line shows `feed: camera_svc` (or `synth_pub`)
with a docker stop exit other than 0, or no exit status, that container
may still run, and `leg_post.sh` stops at exit 3; run
`bash "$BT/legs/power_up_guard.sh" --stop-leg-daemons`, then re-run
`leg_post.sh`.

Every adb push is checked and md5-compared with the PC copy
(`deploy_md5.txt` in the archive). A leg that stops at `[DEPLOY]` with
`adb push FAILED` or `md5 MISMATCH` and `No daemon was started.`
transmitted nothing: look for a
root-owned file or folder in `/tmp/lifetrac_strict` or `/tmp/lifetrac_p0c`
on that board, remove it with sudo and re-run the leg. Details: BENCH_RUNBOOK,
"A leg".

If you must type the harness command yourself, it is the line below, with
every `<...>` replaced by your `bench.env` values. Typed by hand, it skips
`leg_prep.sh`'s carrier gate, its evidence-folder refusal (`leg_post.sh`
only warns about a committed `EVIDENCE_DIR`, and writes there) and the
archive record, so check `EVIDENCE_DIR` first and pass the archive to
`leg_post.sh 2a <archive dir>`:

```powershell
cd LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper
.\run_live_radio_monitor.ps1 -TxAdbSerial <TRACTOR_SERIAL> -RxAdbSerial <BASE_SERIAL> -HostIp <PC_HOST> `
   -TxFeed camera -RegProfile 2 -ForceFrfHz <DTS_CARRIER_HZ from today's spot-check> -DurationS 300 -StopDrainS 2 -SynthFps 2 `
   -KfRequestDisable 1 -ReactiveFire 0 -ProbeEcho 0 -LogFragArrivals 1 -TxBatch 0 `
   -CamExtraEnv "-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80" -Archive
```

The other RS-13 legs (legs/README.md, "The four RS-13 legs"):

- 2b is the same leg on FHSS: `leg_prep.sh 2b image_bw250` (`-RegProfile 1`,
  no `-ForceFrfHz`, `-StopDrainS 4`).
- 2c is the `mono_g4` control: `leg_prep.sh 2c image_bw500`
  (`-CamExtraEnv "-e LIFETRAC_ENCODE_MODE=6"`).
- 2d is the over-the-air switch: `leg_prep.sh 2d image_bw500`, then
  `leg2d_switch.sh 2d` right after the harness launch.
- The range-edge procedure ([RS13_RANGE_EDGE_LEG.md](RS13_RANGE_EDGE_LEG.md))
  uses its own leg names (legs/README.md, "Range-edge legs"). The tag sets
  the profile, codec, probes and length. Since 2026-10-19 only `A2c`, `A2a`,
  `A2c2` and `A2a2` are planned, to fly on the antennas (`leg_prep.sh A2c`).
  The attenuator legs (`leak_*`, `B*`, `C*`, `*close`) are not planned: the
  bench has no attenuator path.
- A command-plane leg adds `-InjectCmd "<injector> <args>"` to the harness
  line; the harness starts the injector on the base at the first published
  frame (BENCH_RUNBOOK, "A leg").

**Not flying after the prep after all** (no GO, the harness refused, the
round was aborted)? `leg_prep.sh` has woken both receivers and left the base
capture running. `bash "$BT/legs/power_up_guard.sh" --stop-leg-daemons`
stops the leftover containers and parks both radios.

[RS13_VECTOR_LEG.md](RS13_VECTOR_LEG.md) gives each leg and the pass criteria
P1–P8. Start the session's `RESULTS.md` from
[RS13_RESULTS_TEMPLATE.md](RS13_RESULTS_TEMPLATE.md), and fill its RF setup
(antennas, cables, positions) and its pass rule (margin and legs per codec)
before leg 1 (sections added 2026-10-22). For RS-12-style tile
legs, see [BENCH_RUNBOOK.md](BENCH_RUNBOOK.md), "A leg". The archive goes to
`DESIGN-CONTROLLER/bench-evidence/radio_monitor_<stamp>_<sha>/`, and its
`params.txt` records what actually ran.

## 7. Analyse

`leg_post.sh` runs the reports for you. To run them by hand, the tools are
[`rs12_leg_report.py`](../../../tools/rs12_leg_report.py),
[`frag_gap_report.py`](frag_gap_report.py),
[`vector_dry_run.py`](../../../tools/vector_dry_run.py) and
[`legs/leg_replay.py`](legs/leg_replay.py). Run them from
`DESIGN-CONTROLLER`. From Git Bash, give them Windows-style `C:/...` paths,
not `/c/...`:

```powershell
py -3 tools/rs12_leg_report.py <archive> --pre <legs>/leg<X>_pre_base.txt --post <legs>/leg<X>_post_base.txt --capture <legs>/leg<X>_base.jsonl
py -3 firmware/x8_lora_bootloader_helper/bench_tools/frag_gap_report.py <archive>
py -3 tools/vector_dry_run.py replay <legs>/leg<X>_base.jsonl --profile image_bw500
py -3 tools/vector_dry_run.py tractor-log <archive>/camera_service.log
py -3 firmware/x8_lora_bootloader_helper/bench_tools/legs/leg_replay.py <X> image_bw500 --evidence-dir <legs>
py -3 firmware/x8_lora_bootloader_helper/bench_tools/bracket_deltas.py <legs> <X>
```

`leg_replay.py <leg> <image_bw500|image_bw250> [--evidence-dir DIR | --capture FILE]`
gives the post-lock sequence gaps and the P1 resync episodes; it reads
`<DIR>/leg<leg>_base.jsonl`. On a capture with no VECTOR frame (`mono_g4`)
it prints `digest_ok n/a (no VECTOR frame)`.

`rs12_leg_report.py` prints two fragment-loss lines. `loss … [whole logs]`
compares every TX-log fragment with every RX-log fragment. That old line keeps
its old figure (it now ends `[whole logs]`), so old reports compare directly. `shared-window loss …`
matches the `txdone_arrival` trains to the `frag_arrival` trains over the
window both logs cover (RS_12_15_clock_authority_2026-09-14/ERRATA.md §3).
Read that one when the whole-log line says received > sent. It prints
`shared-window loss: n/a, withheld (<reason>)` instead of a figure when the
two logs cannot be lined up: an outage of about one u8 seq wrap, or a train
with key-frame copies or parity. (With
`--capture`, the warning points at the capture seq-gap line, which is the
loss of record on an RS-13 leg.) `FIRMWARE DROP` subtracts the base's own
fragment count. The report's first line repeats params.txt's `log_window`.
On `log_window=after_stop` there is no log-window note. When params.txt has
no `log_window` (every archive flown so far), a note says the logs were read
before the daemons stopped. Every 10-18/10-19 leg shows 4-5 there:
`rx_urc_lost` (0-2 on those legs, on the URC-path line) is the part the L072
really lost, and the rest is expected to be that window (first leg with
`log_window=after_stop` to confirm). On `log_window=after_stop_unverified`
(2026-10-22) the report prints `!! LOG WINDOW UNVERIFIED (log_window_detail:
...)` after the loss lines, worded by the reason (a stop not confirmed: the
old artefact is possible; a missing shutdown line after good stops: a log
that ends early), points to it from a `received > sent` line, and adds a note
under FIRMWARE DROP. That note can fall below the 8 lines `leg_post.sh` shows
on the console, so read the report file.

The loss of record is the report's `capture seq gaps` line. The counter
`loss` line can be stale (RS13_VECTOR_LEG A14) or frozen by a dead stats
thread (RS-12.17).

**The head and the gaps** (2026-10-24; tested offline, not yet used on a
flown leg). The `first frame heard at seq N` line under `capture seq gaps`
is now placed on the tractor's TX log, by the frame's transport seq (read off
the base's `published` line at its capture time; the TileDeltaFrame seq is
the camera's own counter). After a u8 seq wrap it names the tractor's frame:
on 2026-10-24 leg 6 it reads `first frame heard at seq 8 is the tractor's
frame #264 on air (1 u8 seq wrap before it, by its TX log)`, where the old
line counted the head as 7 frames (98/103). When the frame cannot be placed,
or its two seqs differ (a codec switch numbers afresh), the line reads `the
head is AMBIGUOUS`, whatever the seq (seq 1 can be frame #257), and gives no
figure counting from seq 1. A `tractor TX log:` line under it gives the share
of the tractor's frames on air that never reached the capture, head and tail
included (leg 6: 601/606), unless the TX log shows a batched train (a
`batching:` line; the line then counts trains and gives no share). A capture
with no clean +1 seq step is timed by
the leg's `synth_fps`, so a long outage is counted (leg 6: 226/231; the old
line said 91/96). `frag_gap_report.py`'s gaps still run from the first
fragment to the last; its new `head:` and `tail:` lines give the silence
before and after them.

**Acquisition** (2026-10-24; not yet used on a flown leg). On an FHSS leg the
report prints an `acquisition:` line: the base's first image reception after
its profile-1 activation, against the 120 s cold-start scan budget. A frame
received is not a lock, so the line under it gives the verdict, on positive
evidence only: `confirmed acquisition: ACQUIRED` (a lock shown),
`confirmed acquisition: NOT DETERMINED`, or `!! ACQUISITION FAILED` (a cold
or early final RX_SCAN_FAILED, or an instrument known to be on that saw no
frame within 120 s, when no lock shown, FAULT_URC-logging daemon without an
RX_SCAN_FAILED line or base bracket contradicts that silence; the check stops
at a profile switch). When the report's first line ends with `!! ACQUISITION
FAILED (see the acquisition line below)`, the base's scan ran out into
FAILED: it never followed the hop grid, and the leg's loss and command
figures measure that, not the link.
`bracket_deltas.py` should then print `!! BASE NEVER LOCKED` (its STOP RULE
line points at it), and `rs13_probe_score.py` prints `!! base ACQUISITION
FAILED`. Finish the leg with `leg_post.sh` (park included), record it, and
void or re-fly it only as the pass rule written before leg 1 says (the
template's "never locks" row is a proposal until the operator agrees it). A
re-fly needs a GO. Details: BENCH_RUNBOOK, "A leg", Base acquisition.

On the RS-12.21 image (built 2026-10-25, not flashed yet) the base's scan
does not stay in FAILED: after its fourth failed attempt it scans on and
sends a final RX_SCAN_FAILED every 30 s until it locks, so a final there is
a 30 s report and a lock after it is a late acquisition. The report and
`rs13_probe_score.py` tell the images apart by `--scan-image` (default
`auto`: the base daemon's `L072 VER warm-up ok ... cap=0x000000FF` line);
give `--scan-image restarts` or `absorbing` when the log cannot say.

**FAULT lines** (2026-10-24; not yet flown). Both daemon logs now carry one
`FAULT_URC: code=0x.. sub=0x.. name=...` line per FAULT the L072 sends
(WARNING; INFO for 0x09 and 0x0C), with `uptime_ms=`, the L072's uptime.
Code 0x0D is a failed scan attempt (`name=RX_SCAN_FAILED`, with `attempt=`
and `final=`; `final=1` means the retries are spent: FAILED on `e083ae46`,
another final every 30 s on the RS-12.21 image). The TX FIFO readback
mismatch is code **0x0F** (`name=TX_FIFO_RB_MISMATCH tx_id=0x..`) on the
RS-12.21 image, which split the two; `e083ae46` and older still send it as
0x0D, and the daemon then writes `name=TX_FIFO_RB_MISMATCH tx_id=0x..
fw=old` (any 0x0D sub that is not a scan encoding). `name=AMBIGUOUS` comes
only from the daemons of 2026-10-24 before the split. Count faults on these
lines only: the same URC can also leave an older line. The harness trigger
`-InjectOn fault:0x0D:final` fires on the scan's attempt-3 fault only
(not on bit 7 alone). Details: BENCH_RUNBOOK, "A leg", FAULT lines and
command injectors.

`bracket_deltas.py <legs> <leg>` scores the L072 counter brackets (reset-aware,
cross-checked against `leg<X>_gaps.txt`; the RS-12.15 criteria on FHSS legs
only; BENCH_RUNBOOK, "A leg").

The range-edge session adds three scoring helpers in `bench_tools/`, run per
leg by `leg_post.sh` (files `leg<X>_{probe_score,rf,digest_trace}.txt`); the
pooled runs are by hand: `rs13_rf_extract.py` (received level, R0, leakage,
tracking and closing checks; `--path antenna` for legs on the antennas gives
the level only), `rs13_probe_score.py` (command delivery, pooled per profile
and codec, with a `tail:` line per leg) and `rs13_digest_trace.py` (A16
confirmation). They take paths, so they need no copy; each exits non-zero on
missing or garbled input. RS13_RANGE_EDGE_LEG.md says which to run per leg.

Since 2026-10-22 `rs13_probe_score.py` also:

- refuses to pool legs whose params `log_window` values differ (exit 1),
  and names a leg whose window is not `after_stop` in its block;
- prints a `!! base command TX attempts other than PROBE (0x69)` line on a
  `-ReactiveFire 1` leg whose base sent other opcodes (control or profile
  traffic among the probes; the 0x63 commands of a 2d switch leg are
  expected);
- takes `--margin PP`: under the unchanged Fisher lines it prints VECTOR
  minus `mono_g4` delivery, its one-sided 95 % lower bound (Newcombe hybrid
  score interval) and `NOT WORSE (margin rule)` when that bound is above
  −PP pp. A two-sided Fisher p ≥ 0.05 passes whenever the sample is too
  small, so write the margin and the legs per codec into RESULTS before leg
  1 ([RS13_RESULTS_TEMPLATE.md](RS13_RESULTS_TEMPLATE.md), "Pass rule").

Since 2026-10-24 (not yet used on a flown leg) each FHSS leg's block ends
with a `base acquisition:` line by the report's rule (ACQUIRED, NOT
DETERMINED, or `!! base ACQUISITION FAILED`, then the first image reception
and the evidence). By default such a leg still pools, and a leg that fired no
probe still withholds the pooled verdict (exit 1). `--void-unlocked` leaves
the legs FAILED on positive evidence out of the pool and lists them (never a
leg NOT DETERMINED); use it only when the pass rule written before leg 1 says
so (BENCH_RUNBOOK, `rs13_probe_score.py`).

## 8. Park and power down

Park both radios with [`radio_park.py`](radio_park.py). `leg_post.sh` does
this at the end of a leg; by hand:

```bash
$BASE "sudo $R $BI -u /work/radio_park.py"      # want PARK_OK, both readbacks 0x80
$TRAC "sudo -n $R $TI -u /work/radio_park.py"
```

The script picks one of two paths from the VER reply (its JSON says which,
`"path"`):

- **firmware** (the RS-12.21 image, `"cap_bitmap": "0x000000FF"`; built
  2026-10-25, not flashed yet): the park command. The L072 stops every
  receiver re-arm and sleeps until a host RXCONT write, a profile
  activation (the next daemon start) or a reset: `"persistent": true`. It
  works at once, with no wait. A `PARK_FAIL` (no answer, or refused while a
  TX is on air) or a `PARK_TRANSIENT` on this path is a fault to report,
  not something to wait out.
- **register** (`e083ae46` and older, `"cap_bitmap": "0x0000007F"`): the
  old SLEEP write, which holds only once the firmware's scan has failed out,
  about 2 minutes after the daemons stop. Park after that. `PARK_TRANSIENT`
  (exit 5) means the scan was still walking: wait 60 s and repeat.

Either way, before you call the bench off, re-check both boards at least
2 minutes later with the read-only [`radio_state.py`](radio_state.py) (same
command line), or run `bash "$BT/legs/power_up_guard.sh" --check-only`.

To power down:

- **tractor:** `$TRAC "sudo -n systemctl poweroff"`; it stays off.
- **base:** `$BASE "sudo systemctl poweroff"`, then **remove its power**.
  Otherwise it boots again about 4.5 minutes later.

## 9. Capture board state

```bash
ARCHIVE=$ARCHIVE_DIR bash "$BT/pull_board_state.sh" tractor   # add --images for docker saves
ARCHIVE=$ARCHIVE_DIR bash "$BT/pull_board_state.sh" base
```

`ARCHIVE_DIR` comes from `bench.env` through the block in step 1, already in
the `C:/...` form adb needs. To write somewhere else, set `ARCHIVE` to any
folder outside git (`C:/...`, `~/...` or `/c/...` all work: the script
converts it with `cygpath -m`). The capture is adb-only, so the base must be
on USB.

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
| flash stops with `PREFLIGHT-FAIL: missing or empty` or `PREFLIGHT-FAIL: CRLF line endings in:` | `/tmp/lifetrac_p0c` aged out or was wiped by a reboot, or a script went up with CRLF | re-run `flash_l072.sh <board> <bin>` without `--go`: it re-stages LF-clean and repeats the preflight; `PREFLIGHT-OK` means the staging is complete |
| `ERR_PROTO FORBIDDEN detail=6` on `-ForceFrfHz` or a survey | production firmware on the board | flash the bench build (step 3) |
| harness throws `Refusing a profile-2 (DTS) leg with -ForceFrfHz 0` | a DTS leg without a carrier pin | spot-check (step 5.4) and launch the wrapper `leg_prep.sh` writes |
| `leg_prep.sh` prints `REFUSED: DTS_CARRIER_HZ ...` | no pick in `bench.env`, 915.000 MHz, or a pick from another day | spot-check today and set `DTS_CARRIER_HZ` / `DTS_CARRIER_DATE` (step 5.4) |
| DTS loss folds on a ~7.08 s period | the leg flew on 915.000 MHz | spot-check and pin `-ForceFrfHz` (step 5.4) |
| `running scripts is disabled on this system` in PowerShell | execution policy `Restricted` | [PC_SETUP.md](PC_SETUP.md#powershell-execution-policy) |
| camera changes mode or a profile is commanded mid-leg | retained control topics | `clear_retained.py` on the base broker **and** `clear_retained_host.py` on the PC broker |
| report says ~99.8 % loss and `published=1` | stats thread died (RS-12.17) | use `capture seq gaps` / count `published frame_id` lines |
| the report's first line ends `!! ACQUISITION FAILED`; `leg<X>_deltas.txt` says `!! BASE NEVER LOCKED` | positive evidence (the `!!` line under `acquisition:` names it) that the base's FHSS scan ran out into FAILED and never followed the hop grid: a cold or early final RX_SCAN_FAILED, or no frame within 120 s of its profile-1 activation on an instrument known to be on, with nothing contradicting it (2026-10-24 leg 6: its stats counter and its frag_arrival lines) | the leg measures that, not the link: finish it with `leg_post.sh`, record it, and void or re-fly it only as the pass rule written before leg 1 says (BENCH_RUNBOOK, "A leg", Base acquisition); a re-fly needs a GO |
| FAULT counts differ between tools, or a tractor fault shows twice | since 2026-10-24 a URC leaves a `FAULT_URC:` line and can also leave a legacy line (`FAULT during pipelined TX:`, or stdout `FAULT_URC during TX wait:` / `during settle`) | count only the `FAULT_URC: code=0x.. sub=0x.. name=` lines |
| docker images vanish on the base | `compose-apps-early-start-recovery.service` is unmasked: it runs `rm -rf /var/lib/docker` every 60 s | mask it ([BENCH_SETUP.md](BENCH_SETUP.md)) |
| board resets about 60 s into a long operation | the U-Boot-armed X8 watchdog | FLASH_RUNBOOK §3; close `/dev/watchdog0` with `V` |
| a harness parameter seems ignored | misspelled name | the harness now rejects unknown names; read the archive's `params.txt` |
| harness stops at `[DEPLOY] adb push FAILED ...` or `[DEPLOY] md5 MISMATCH ...`, `No daemon was started.` | since the 2026-10-19 bench kit every push is checked; usually a root-owned file or folder in `/tmp/lifetrac_strict` or `/tmp/lifetrac_p0c` on that board | remove it with sudo on that board and re-run the leg; nothing was transmitted |
| a leg script prints `REFUSED: EVIDENCE_DIR=... already holds N file(s) tracked by git` (`leg_prep.sh` / `stage_boards.sh` exit 7) | `EVIDENCE_DIR` in `bench.env` points at a committed evidence record (since 2026-10-22) | point it at a new folder for this session; for a deliberate append, run every script of the leg with `BENCH_EVIDENCE_APPEND=1` in front (step 6) |
| LEG CHECKS prints `LOG_WINDOW after_stop_unverified` in red | a daemon's `docker stop` at leg end was not confirmed (2026-10-22 check); the reason is in `params.txt` `log_window_detail=` | run `leg_post.sh` as usual: if a leg daemon still runs it stops before the brackets and the park (exit 3); then `bash "$BT/legs/power_up_guard.sh" --stop-leg-daemons` (it parks too) and re-run `leg_post.sh` |
| LEG CHECKS prints `LOG_WINDOW after_stop_unverified` in yellow | a daemon log has no SIGTERM shutdown line: the daemon may have died before the stop, or its log did not come back whole | look for a traceback in that daemon's final log; its figures cover less than the leg |
| adb lost the board after `adb kill-server` | the board's adbd hung | run `adb -s <serial> wait-for-device` once; if that fails, replug USB-C or press RESET. Avoid `kill-server` while the boards are enumerating ([ADB_TIPS_AND_TRICKS.md](../../../X8_HEALTH_AND_RECOVERY/recovery/ADB_TIPS_AND_TRICKS.md)) |
