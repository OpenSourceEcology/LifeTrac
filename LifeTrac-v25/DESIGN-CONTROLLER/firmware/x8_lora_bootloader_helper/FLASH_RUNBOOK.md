# L072 flash runbook (Portenta X8 → Murata CMWX1ZZABZ)

How to put a `murata_l072` build on a bench board without a JTAG pod. The
X8 drives the L072's ROM bootloader (AN3155 over `/dev/ttymxc3`) while
openocd holds BOOT0/NRST through the H7's GPIOs. Written 2026-09-12 after
a session that met every trap below; the scripts live in this directory
and are pushed to `/tmp/lifetrac_p0c` on the board.

- **Scripted form (2026-10-10):** [`bench_tools/flash_l072.sh`](bench_tools/flash_l072.sh)
  does sections 1–2 from the PC. It reads the serials, base address, ssh key
  and sudo password from `bench_tools/bench.env` (template
  `bench_tools/bench.env.example`); the defaults are the current bench.
  Without `--go` it only stages and preflights; `--go` flashes.
- **A board that has never been flashed** (stock MKRWAN AT firmware on the
  L072): read [section 5](#5-first-flash-from-a-factory-board) first.
- **A flash is a radio-on event.** The L072 boots into RXCONT. Flash only on
  the operator's explicit GO for that round.

## 0. Which binary

| build | command | binary | use |
|---|---|---|---|
| production | `mingw32-make all` (PowerShell) | `build/firmware.bin` (committed) | field boards. `HOST_ALLOW_REG_WRITE_DIAG=0`: refuses diagnostic register writes, including the bench carrier pin (`-ForceFrfHz`, `channel_survey_sniff.py` → `ERR_PROTO FORBIDDEN detail=6`). |
| bench | `mingw32-make bench` | `build/firmware_bench_diag.bin` (untracked) | **both bench boards**. Same source, `HOST_ALLOW_REG_WRITE_DIAG=1`. |

Run `mingw32-make check` before flashing anything; `check-stats-layout`
pins the STATS wire against `host_types.h` (an additive-tail slip shifted
four fields by one slot on 2026-09-12 — the test exists because of it).

### Byte-identical builds and the expected md5s

The build is reproducible byte for byte, but only with the exact toolchain:
**Arm GNU Toolchain 12.2.MPACBTI-Rel1** (`arm-none-eabi-gcc --version` →
`12.2.1 20230214`). On the bench PC it is installed at
`C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\12.2 mpacbti-rel1\`,
with WinLibs `mingw32-make` run from PowerShell. Expected md5s at `main`
`bb4a2071` (`firmware/murata_l072` unchanged since):

| build | md5 | size |
|---|---|---|
| bench `build/firmware_bench_diag.bin` (on both boards since 2026-09-15) | `0c1bb0a9573f813137f941dfa47177d0` | 24 860 B |
| production `build/firmware.bin` (committed) | `589c120323c2d5e7ef9f459d7a4ba42d` | 24 860 B |

Other compilers (Arduino's bundled 7-2017q4, which `build.ps1` picks first;
Ubuntu's `gcc-arm-none-eabi`, which the main CI job uses) build working
firmware with **different bytes**, so a matching md5 also proves the
toolchain. Details and make targets:
[`../murata_l072/README.md`](../murata_l072/README.md#building). The
non-blocking CI job `.github/workflows/l072-bench-md5.yml` rebuilds both
images on Linux with the same Arm release and prints the md5s against these
values. `flash_l072.sh` names every recorded md5 before it stages. The older
builds are listed in
[`bench-evidence/RS_13_vector_scene_2026-09-26/firmware/README.md`](../../bench-evidence/RS_13_vector_scene_2026-09-26/firmware/README.md).

## 1. Push the tooling (LF!)

git autocrlf rewrites `*.sh`/`*.cfg` to CRLF in the working tree whenever
git touches them (a pull that changed them, a fresh worktree). Bash on the
board then sees `\r` in every path (`/tmp/lifetrac_p0c\r/pipeline.log: No
such file`). Normalize before pushing:

```bash
mkdir -p /tmp/p0c_lf && for f in full_flash_pipeline.sh run_flash_l072.sh prep_bridge.sh revive_bridge.sh wdt_pet.sh stm32_an3155_flasher.py 07_assert_pa11_pf4_long.cfg 08_boot_user_app.cfg 99_release_and_reset.cfg; do sed 's/\r$//' "$f" > /tmp/p0c_lf/$f; done
```

Base (ethernet): `scp /tmp/p0c_lf/* firmware_bench_diag.bin fio@192.168.1.117:/tmp/lifetrac_p0c/`
Tractor (adb, Windows-style local paths with `MSYS_NO_PATHCONV=1`):
`adb -s 2E2C1209DABC240B push C:/…/firmware_bench_diag.bin /tmp/lifetrac_p0c/`
Verify on the board: `grep -l $'\r' /tmp/lifetrac_p0c/*.sh | wc -l` → 0.

The instrumented wrapper `run_flash_bench.sh` plus `stamp.py` and
`kmsg_log.py` go to `/home/fio` (persistent) — they write monotonic-stamped
pipeline, pet and kernel logs under `/home/fio` so a reboot cannot eat the
evidence.

`flash_l072.sh <base|tractor> <bin>` does all of this section (LF-clean
copies of the same nine pipeline files plus the bin, the three wrapper files
to `/home/fio`) and then runs the wrapper's preflight
(`FLASH_PREFLIGHT_ONLY=1`). The wrapper itself now refuses to start — before
touching openocd, the UART or the watchdog — when a pipeline file or the
image is missing or a `.sh`/`.cfg` has CRLF (`PREFLIGHT-FAIL`, exit 3).

## 2. Flash

```bash
# tractor (adb) — stop the unit AND the container first, or the UART is stolen
adb -s 2E2C1209DABC240B shell "echo fio | sudo -S -p '' systemctl stop lifetrac-camera.service; echo fio | sudo -S -p '' docker stop tractor-camera; echo fio | sudo -S -p '' env REVIVE_MODE=reboot bash /home/fio/run_flash_bench.sh /tmp/lifetrac_p0c/firmware_bench_diag.bin"
# base (ssh) — root's SSH shell has no sbin on PATH; the wrapper exports it
ssh -i ~/.ssh/lifetrac_base_ed25519 fio@192.168.1.117 "REVIVE_MODE=reboot bash /home/fio/run_flash_bench.sh /tmp/lifetrac_p0c/firmware_bench_diag.bin"
# the same from the PC, one board at a time (stages, preflights, then flashes)
bash bench_tools/flash_l072.sh tractor ../murata_l072/build/firmware_bench_diag.bin --go
bash bench_tools/flash_l072.sh base    ../murata_l072/build/firmware_bench_diag.bin --go
```

**`REVIVE_MODE` now defaults to `reboot`** (2026-10-10) in both
`run_flash_bench.sh` and `full_flash_pipeline.sh`; it used to default to
`full`, so every flash had to pass `REVIVE_MODE=reboot` by hand. Passing it
explicitly, as above, still works. `REVIVE_MODE=full` remains selectable for
a 5.10 board where a reboot is unwanted; on the base's 6.1 kernel it oopses
(below). `revive_bridge.sh` treats `reboot` as `reset_run_only`, and it warns
before a module reload on any 6.x kernel. Other wrapper variables:
`FLASH_VERIFY_ONLY=1` (read back and compare, write nothing),
`BENCH_SUDO_PW` and `BENCH_HOME` (defaults `fio`, `/home/fio`).

What happens: `wdt_pet start` (pets `/dev/watchdog0` every 10 s, fsync'd
log) → `prep_bridge` (unbind consumers, rmmod `x8h7_*`) → `run_flash_l072`
(openocd holds BOOT0 high + NRST, AN3155 write + `Verify OK`, ~45 s) →
`revive_bridge` up to `openocd reset run` (H7 back in its firmware) →
`wdt_pet stop` (writes `'V'` then closes) → `systemctl reboot`. The board
is back in 12–24 s. `PIPELINE-EXIT=0` and `flash_rc=0` in
`/home/fio/pipeline_stamped.log` are the record.

Why the reboot is deliberate: the old revive re-inserted the `x8h7`
modules, which OOPSes the base's 6.1.24 kernel in `spi_probe →
of_irq_get` and drops the board anyway (twice on 2026-09-12, petter alive 2 s
before). `REVIVE_MODE=full` keeps the old behaviour for the tractor's 5.10
kernel if ever wanted; nothing needs it, which is why `reboot` is now the
default.

## 3. Watchdog facts (why the scripts look like this)

- WDOG1 is armed by **u-boot** on both boards: `WCR=0x773d` (60 s,
  `WDOG_B` → PMIC). A timeout is a full power-cycle and `WRSR` reads POR,
  never TOUT — the register cannot distinguish a watchdog reboot.
- The kernel core keeps it alive until userspace opens `/dev/watchdog0`.
  **Close it only after writing `'V'`**; a plain close leaves the dog active
  with no petter → reboot 60 s later (tractor, 5.10.93, 2026-09-12).
- No `/sys/class/watchdog/*` attributes, no `devmem`, host python has no
  `mmap`: read registers with `wdog_regs.py` inside the docker image
  (`--privileged -v /dev/mem:/dev/mem`). The script now lives in the repo at
  [`bench_tools/wdog_regs.py`](bench_tools/wdog_regs.py). Until 2026-10-10
  it existed only in `/home/fio` on the boards; it was recovered from the
  board captures. It is read-only (`O_RDONLY`, `PROT_READ`). Push it to
  `/tmp/lifetrac_strict`, then:
  `echo fio | sudo -S -p '' docker run --rm --privileged -v /dev/mem:/dev/mem -v /tmp/lifetrac_strict:/work --entrypoint python3 lifetrac-v25:latest /work/wdog_regs.py`
  (on the tractor use the `arduino-ootb-python-devel:738bc44` image).
  Expected: `WCR=0x773d WDE=1 WT=119 (60.0 s)`.

## 4. After the reboot

`/tmp` is tmpfs — re-push `/tmp/lifetrac_strict` (the harness does its own
list at launch; `radio_state.py`, `radio_park.py`, `kf_inject.py`,
`clear_retained.py`, `channel_survey_sniff.py` are manual) and
`/tmp/lifetrac_p0c` if another flash is coming. On the tractor the
production `tractor-camera` container returns holding `/dev/ttymxc3`
(probe symptom: `timeout waiting for response`): stop the unit, then the
container. On the base restart the bench web UI
(`bench_webui`, port 8090, PIN 2525) if it was running.

Then `rs116_health_probe.py` on each board: `RS115-INSTRUMENTED-FIRMWARE=YES`,
`RS12-URC-COUNTERS=YES`, `RS12-10-COUNTERS=YES`, counters 0,
`radio_state=4`. The probe helpers (`method_g_stage1_probe.py`,
`method_h_stage2_tx_probe_v2.py`) must carry the label list matching the
flashed wire — the harness re-pushes them from the branch it runs from at
every launch, so fly a wire change from that branch or every post-leg
bracket silently parses only the old fields.

Park at the end of the session: `radio_park.py` → `PARK_OK
{"opmode_readback": "0x80"}` on both boards.

## 5. First flash from a factory board

This section collects guidance from the 2026-05 bring-up notes that was not
in one place before. Both bench L072s have been flashed many times since
May. Nobody has run this first-flash procedure from scratch since then, so
treat it as consolidated guidance and record what you see.

### What a factory board has

- The L072 inside the Murata CMWX1ZZABZ runs Arduino's stock **MKRWAN AT
  modem firmware**. The repo keeps 1.2.3 as the factory restore image,
  [`../../../tools/mlm32l07x01.bin`](../../../tools/mlm32l07x01.bin)
  (83 032 B). The file of the same name in this directory is misnamed: it is
  LifeTrac "hello v0.2".
- **Nothing on the carrier needs to change for this path.** The X8 reaches
  the L072's ROM bootloader with parts it already has. `/dev/ttymxc3` (i.MX
  UART4, wired straight to the L072, not through the H7 bridge) carries
  AN3155 at **19200 8E1**. The X8's own openocd
  (`/usr/arduino/extra/openocd_script-imx_gpio.cfg`) halts the H747 and
  drives the L072's **BOOT0 = H7 PA11** and **NRST = H7 PF4**.
  [`07_assert_pa11_pf4_long.cfg`](07_assert_pa11_pf4_long.cfg) holds BOOT0
  high for a 600 s window. [`stm32_an3155_flasher.py`](stm32_an3155_flasher.py)
  is stdlib-only, because LmP's python has no `termios`; `stty` sets up the
  UART in the shell. Pin map: [Phase 0 crack note](../../../AI%20NOTES/2026-05-07_Portenta_X8_LoRa_Phase0_Crack_and_Phase1_Options_Copilot_v1_0.md).
  First end-to-end flash, which wrote the MKRWAN image itself:
  [Method G Phase 1 note](../../../AI%20NOTES/2026-05-08_Method_G_Phase1_End_to_End_Flash_Success_Copilot_v1_0.md).
- No J-Link or ST-Link is needed. The carrier's L072 SWD header `CN2` is
  **not populated** by default
  ([BRINGUP_MAX_CARRIER.md](../murata_l072/BRINGUP_MAX_CARRIER.md) §2); it
  only matters for the last-resort recovery below.

### Before the first flash

1. Set up the X8 first ([`bench_tools/BENCH_SETUP.md`](bench_tools/BENCH_SETUP.md)):
   the sudoers drop-in, docker usable, and on the tractor
   `lifetrac-camera.service` / `tractor-camera` stopped. Both grab
   `/dev/ttymxc3`, and `run_flash_l072.sh` refuses while anything holds it
   (exit 90).
2. **Freshly reflashed X8 (uuu):** `gpio10` (H7 NRST) boots LOW, so openocd
   reports `cannot read IDR` (README_P0C TT-1). `run_flash_l072.sh` already
   exports gpio8/10/15, drives gpio10 high and unexports them before
   openocd attaches. If attach still fails, compare against a good board
   ([HC-03](../../X8_HEALTH_AND_RECOVERY/routines/HC-03_h7_swd_attach_diff.md)).
   `SWD DPIDR 0xdeadbeef` on a newer openocd is the sysfs/mmap race, **not a
   bricked L072** ([T4](../../X8_HEALTH_AND_RECOVERY/recovery/T4_lora_jlink_hardware_unbrick.md)
   "When NOT to use").
3. Build with the exact toolchain and check the md5 (section 0).

### The first flash, step by step

1. **Read-only first contact.** Run
   `bash bench_tools/flash_l072.sh <board> ../../../tools/mlm32l07x01.bin --go --verify-only`.
   It enters the ROM bootloader, reads the flash back and compares it with
   the stock image without writing anything, then reboots the X8.
   - `READY: L072 in STM32 ROM bootloader` (openocd) followed by the
     flasher's `Bootloader version: 31` means ROM entry and the UART path
     work.
   - `Verify-only OK` means the chip carries stock 1.2.3.
   - A mismatch only means a different stock version. The real question is
     whether ROM entry works.
   - This is also how to capture what was on the chip before LifeTrac
     touched it.
2. **Flash the bench build:**
   `bash bench_tools/flash_l072.sh <board> ../murata_l072/build/firmware_bench_diag.bin --go`.
   - Erase: the flasher first tries Extended Erase mass erase (`0x44`,
     `0xFFFF`). On the Murata factory part that NACKs, so it falls back to
     **page-by-page erase** of only the pages the image needs. The rest of
     the flash and the option bytes are untouched.
   - Success means `Verify OK`, `flash_rc=0` and `PIPELINE-EXIT=0` in
     `/home/fio/pipeline_stamped.log`. The X8 reboots
     (`REVIVE_MODE=reboot`, section 2).
3. After the reboot, follow section 4: re-stage `/tmp`, stop the tractor
   camera, run `rs116_health_probe.py`, and park the radio.

### Option bytes and readout protection: do not touch them

- **Never send these AN3155 commands from the X8:** Write Unprotect `0x73`,
  Readout Protect `0x82`, Readout Unprotect `0x92`, or any Write Memory to
  the option bytes at `0x1FF80000`. The pipeline's flasher sends none of
  them.
  - On 2026-05-14, `flash_l072_via_uart.py wunprot` (through
    `recover_l072_opt.sh`) was ACKed, and then the chip went silent. Its
    option-byte complement pairs no longer matched, and it would not run its
    flash again without SWD
    ([timeline](../../../AI%20NOTES/2026-05-14_W2-02_Board2_L072_wunprot_Brick_Timeline_Copilot_v1_0.md),
    [root cause](../../../AI%20NOTES/2026-05-14_W2-02_Board2_L072_OPT_Bytes_Root_Cause_Copilot_v1_0.md)).
  - That script, `flash_l072_via_uart.py`, `l072_unprotect.py`,
    `run_l072_unprotect.sh` and `recover_l072_opt.sh` are still in this
    directory **as history. Do not run them.**
- Readout Protect sets RDP level 1. The ROM then refuses reads, so the
  pipeline can no longer verify. Going back to level 0 mass-erases the chip.
  Level 2 is permanent. Neither has any use on the bench.
- An option-byte change needs a full power-on reset to load; NRST alone is
  not enough (root-cause note §4.2).
- Factory-clean option bytes read `0x807800AA` at `0x1FF80000` (RDP `0xAA` =
  level 0). Source: the
  [J-Link note §3a](../../../AI%20NOTES/2026-05-17_Murata_L072_LoRa_Unbrick_JLink_Hardware.md).
  The bench L072s' current option bytes were never recorded. Reading them
  (AN3155 Read Memory, read-only) is an open item in
  [`BENCH_BOARDS.md`](bench_tools/BENCH_BOARDS.md), to be done only on a GO.

### When it goes wrong (2026-05 lessons)

- **Openocd never prints `READY: L072 in STM32 ROM bootloader`.** This is
  the H7 side (gpio10, the openocd race, or a wedged bridge), not the L072.
  Power-cycle the X8 and carrier with the 12 V barrel and USB-C unplugged
  for 10 s
  ([T2](../../X8_HEALTH_AND_RECOVERY/recovery/T2_cold_power_cycle.md))
  before you suspect the chip.
- **Do not use the PWM/gpio163 "SWD bypass"**
  (`swd_bypass_pa11_pf4_launcher.sh`, sysfs PWM4 + gpio163). It was shown
  on 2026-05-17 not to enter the ROM. Only the openocd path does.
- **Last resort: an external J-Link on `CN2`**
  ([T4](../../X8_HEALTH_AND_RECOVERY/recovery/T4_lora_jlink_hardware_unbrick.md),
  [J-Link note](../../../AI%20NOTES/2026-05-17_Murata_L072_LoRa_Unbrick_JLink_Hardware.md)).
  1. Solder a 2×5 1.27 mm header. Use the standard Cortex 10-pin zig-zag
     cable; the Segger needle adapter does not fit.
  2. VTref (~3.3 V) needs the 12 V barrel jack **and** the X8 seated and
     powered.
  3. Run `JLink.exe -device STM32L072CZ -if SWD -speed 1000 -autoconnect 1`.
  4. `erase`: a mass erase that also restores the option-byte defaults.
  5. `mem32 0x1FF80000 4` must show `0x807800AA`.
  6. `loadbin <bin>, 0x08000000`, then `r` and `go`.

  Once the option bytes are clean, the X8 pipeline above works again.
- **Which board was bricked is uncertain.** The 05-14 brick ran on adb
  serial `2E2C…`, but the 05-17 unbrick note says "Board 1 (`2D0A…`)". The
  notes disagree, which [`BENCH_BOARDS.md`](bench_tools/BENCH_BOARDS.md)
  lists as an open item.
- Do not accept J-Link OB firmware-update prompts. An X8 OS reflash (T3a)
  never fixes an L072 problem.
