# Portenta X8 OS image: suggested improvements

**Status:** proposal, 2026-10-13. Not yet started. Task list: [TODO.md § X8 Linux OS image](TODO.md#x8-linux-os-image--improvements-x8os).

**Origin:** an analysis of whether a custom Gentoo build for the Portenta X8
would speed LifeTrac up or help the STM32H747. It would not. This page records
that conclusion and the four changes that would actually pay off. The full
write-up is the project document "Gentoo on the Portenta X8: feasibility and
payoff" (2026-10-13).

## 1. Decision: stay on LmP, do not build Gentoo

Gentoo would boot on the X8, but the board support (imx-boot, the Portenta X8
device tree, the nine `x8h7` kernel modules, `stm32h7-program.service`, the
Murata 1DX firmware) exists only as Yocto recipes in Arduino's and Foundries'
layers. Porting it is several person-weeks (estimate, not measured), then
ongoing upkeep with no vendor behind it, and it gives up OSTree rollback.

It would not make LifeTrac measurably faster:

- The CPU-heavy code (OpenCV, NumPy, Pillow/WebP, `cryptography`, ffmpeg)
  runs in `python:3.11-slim-bookworm` containers from PyPI aarch64 wheels.
  The host distro's compiler flags never reach it.
- aarch64 generic builds already use NEON, OpenCV dispatches optimized
  kernels at runtime, and the official Python images are built with PGO and
  LTO. Tuning for Cortex-A53 on top is low single-digit percent (inferred).
- Linux is not the bottleneck. A 44 B control frame is ~92 ms on air at
  SF7/BW125 ([DECISIONS.md](DECISIONS.md) D-A2). In RS-13 the vector-mode
  switch took 2.55 s, of which encode was 0.42 s p50
  ([RESULTS.md](bench-evidence/RS_13_vector_scene_2026-09-26/RESULTS.md) P6).
  Code changes already beat compiler flags: `c7f3926e` halved encode time.

It does nothing for the H747. Its M7 and M4 have no MMU and run bare-metal
firmware; no Linux distro changes their speed. That also holds for a
standalone Portenta H7 board.

Revisit Gentoo only if a measurement (X8OS-2) shows a CPU-bound hot spot that
a tuned container cannot fix.

## 2. Current state (2026-10-10)

From [BENCH_BOARDS.md](firmware/x8_lora_bootloader_helper/bench_tools/BENCH_BOARDS.md):

| | base (2D0A) | tractor (2E2C) |
|---|---|---|
| LmP | 4.0.11-934-91 | 4.0.3-674-88 (factory) |
| kernel | 6.1.24-lmp-standard | 5.10.93-lmp-standard |
| H747 bank 1 (M7) | stock x8h7, programmed at boot | stock x8h7, **reprogrammed at every boot** |
| H747 bank 2 (M4) | unknown May bench sketch | unknown May bench sketch |

OS-level pain points already on record:

- The LmP host Python is stripped (no `termios`):
  [HC-04](X8_HEALTH_AND_RECOVERY/routines/HC-04_stage1_standard_quant.md).
- `adb reverse` is broken on 934:
  [ADB_TIPS_AND_TRICKS.md §3.1](X8_HEALTH_AND_RECOVERY/recovery/ADB_TIPS_AND_TRICKS.md).
- USB autosuspend needs a kernel argument to be off before enumeration:
  TODO W2-01.D3.
- MIPI CSI needs a device-tree overlay, and the Coral PCIe card needs the
  `gasket`/`apex` driver: TODO Phase 0 and Phase 1.
- `tractor_h7`'s M7 linker script starts at `0x08040000`
  ([linker_script_x8_m7.ld](firmware/tractor_h7/linker_script_x8_m7.ld)),
  inside the region stock x8h7 occupies (`0x08000000`–`0x080709a4` on the
  tractor).

## 3. Improvements, in order

### X8OS-1 Put both boards on LmP 934

The tractor is the only board still on 674 / kernel 5.10. One image means one
set of quirks. [BENCH_SETUP.md §3](firmware/x8_lora_bootloader_helper/bench_tools/BENCH_SETUP.md)
already recommends 934 on both boards and lists the 934 differences that
matter for the tractor; the reflash is
[T3a](X8_HEALTH_AND_RECOVERY/recovery/T3a_sdp_uuu_reflash.md).

- Capture the tractor's board state first with
  [`capture_board_state.sh`](firmware/x8_lora_bootloader_helper/bench_tools/capture_board_state.sh)
  (as for `board_state_2026-10-10`), so 674 behaviour stays on record.
- Treat the first tractor-on-934 radio legs as a new configuration and say so
  in the evidence.

**Done when:** both boards report `4.0.11-934-91` / `6.1.24-lmp-standard`, the
HC checks pass on the tractor, and one RS-13-style leg has run on the new pair.

### X8OS-2 Measure before tuning anything

Get numbers for the three things a different OS could change.

- **CPU:** run `tools_offline/encoder_perf`
  ([README](bench-evidence/RS_13_vector_scene_2026-09-26/tools_offline/README.md))
  on the tractor with the live pipeline running, and record per-stage times
  and per-core load.
- **Boot:** power-on to "tractor services ready" and to "first LoRa frame",
  plus `systemd-analyze blame`.
- **RAM:** `free -m` and per-container RSS at steady state.

Then test the Gentoo performance claim without changing the OS: rebuild the
tractor container with NumPy, OpenCV and libwebp compiled for
`-mcpu=cortex-a53`, and run the same encoder bench. Adopt it only if it moves
a number that matters, and pin it in `requirements.lock.txt` like any other
build input.

**Done when:** a dated `bench-evidence/` folder holds the baseline and the
tuned-container A/B.

### X8OS-3 Settle who owns the H747 flash

This is the item that matters for the H7. Today Linux overwrites bank 1 with
stock x8h7 at boot, so any LifeTrac M7 image would be lost on the next reboot.

1. Decide whether the tractor M7 runs stock x8h7 (Linux keeps its GPIO, CAN
   and UART bridges) or `tractor_h7` (safety loop independent of Linux, per
   [TRACTOR_NODE.md](TRACTOR_NODE.md) and [MASTER_PLAN.md §8.9](MASTER_PLAN.md)).
2. If `tractor_h7`: fix the linker overlap, choose the Linux↔H747 link that
   replaces the x8h7 bridge (a UART or an RPMsg-style shared-memory channel),
   and override `stm32h7-program.service` in `/etc` so it no longer reflashes
   bank 1. `/etc` is writable under OSTree, so this needs no new image.
3. Identify and either document or blank the unknown M4 sketch in bank 2
   (the openocd dump in BENCH_BOARDS *Open items*).
4. Record the decision in [DECISIONS.md](DECISIONS.md).

**Done when:** a reboot leaves the chosen M7 and M4 images in place, verified
by readback, and the decision is in DECISIONS.md.

### X8OS-4 Build a custom LmP image when an item forces it

Don't rebuild the OS for its own sake. Build one custom image from Arduino's
LmP manifest (Yocto) when the first of these is needed, and fold in the rest
at the same time:

- `OSTREE_KERNEL_ARGS=usbcore.autosuspend=-1` (W2-01.D3);
- the MIPI CSI camera device-tree overlay;
- the Coral `gasket`/`apex` driver for the base;
- a full host Python (including `termios`);
- the `stm32h7-program.service` change from X8OS-3, if it should ship in the
  image rather than as an `/etc` override;
- trimming services LifeTrac does not use (aktualizr-lite, the compose-apps
  units), to cut boot time and RAM against the X8OS-2 baseline.

A custom LmP keeps OSTree rollback, the uuu recovery path and the vendor BSP.
Pin the manifest revision and publish the image's sha256 the way T3a pins
934, so the image can be rebuilt.

**Done when:** a pinned, rebuildable image boots on both boards, the items it
carries are verified, and BENCH_SETUP §3 names it as the bench image.

## 4. Comparison

| | Stock LmP (today) | Custom LmP / trimmed Yocto | Custom Gentoo |
|---|---|---|---|
| Effort to adopt | none | moderate: bitbake and a build host | high: port BSP, drivers, boot chain |
| Ongoing upkeep | Arduino ships images | rebase on Arduino's manifest | all ours, rolling upstream |
| Speed of LifeTrac code | baseline | same (code runs in containers) | same unless containers go; then a few percent at best |
| Boot time and RAM | heaviest | trim to need | trim to need |
| Kernel args, DT overlays, PREEMPT_RT | no | yes | yes |
| Full host Python | no | yes | yes |
| Atomic update and rollback | OSTree | OSTree | none unless we build A/B |
| x8h7 drivers and H7 programming | included | included, configurable | port ourselves |
| Vendor support | Arduino and Foundries | partial (same layers) | none |
| Effect on H747 firmware | none | none | none |
