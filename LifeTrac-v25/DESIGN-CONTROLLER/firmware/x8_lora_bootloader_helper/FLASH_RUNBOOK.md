# L072 flash runbook (Portenta X8 → Murata CMWX1ZZABZ)

How to put a `murata_l072` build on a bench board without a JTAG pod. The
X8 drives the L072's ROM bootloader (AN3155 over `/dev/ttymxc3`) while
openocd holds BOOT0/NRST through the H7's GPIOs. Written 2026-09-12 after
a session that met every trap below; the scripts live in this directory
and are pushed to `/tmp/lifetrac_p0c` on the board.

## 0. Which binary

| build | command | binary | use |
|---|---|---|---|
| production | `mingw32-make all` (PowerShell) | `build/firmware.bin` (committed) | field boards. `HOST_ALLOW_REG_WRITE_DIAG=0`: refuses diagnostic register writes, including the bench carrier pin (`-ForceFrfHz`, `channel_survey_sniff.py` → `ERR_PROTO FORBIDDEN detail=6`). |
| bench | `mingw32-make bench` | `build/firmware_bench_diag.bin` (untracked) | **both bench boards**. Same source, `HOST_ALLOW_REG_WRITE_DIAG=1`. |

Run `mingw32-make check` before flashing anything; `check-stats-layout`
pins the STATS wire against `host_types.h` (an additive-tail slip shifted
four fields by one slot on 2026-09-12 — the test exists because of it).

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

## 2. Flash

```bash
# tractor (adb) — stop the unit AND the container first, or the UART is stolen
adb -s 2E2C1209DABC240B shell "echo fio | sudo -S -p '' systemctl stop lifetrac-camera.service; echo fio | sudo -S -p '' docker stop tractor-camera; echo fio | sudo -S -p '' env REVIVE_MODE=reboot bash /home/fio/run_flash_bench.sh /tmp/lifetrac_p0c/firmware_bench_diag.bin"
# base (ssh) — root's SSH shell has no sbin on PATH; the wrapper exports it
ssh -i ~/.ssh/lifetrac_base_ed25519 fio@192.168.1.117 "REVIVE_MODE=reboot bash /home/fio/run_flash_bench.sh /tmp/lifetrac_p0c/firmware_bench_diag.bin"
```

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
kernel if ever wanted; nothing needs it.

## 3. Watchdog facts (why the scripts look like this)

- WDOG1 is armed by **u-boot** on both boards: `WCR=0x773d` (60 s,
  `WDOG_B` → PMIC). A timeout is a full power-cycle and `WRSR` reads POR,
  never TOUT — the register cannot distinguish a watchdog reboot.
- The kernel core keeps it alive until userspace opens `/dev/watchdog0`.
  **Close it only after writing `'V'`**; a plain close leaves the dog active
  with no petter → reboot 60 s later (tractor, 5.10.93, 2026-09-12).
- No `/sys/class/watchdog/*` attributes, no `devmem`, host python has no
  `mmap`: read registers with `wdog_regs.py` inside the docker image
  (`--privileged -v /dev/mem:/dev/mem`).

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
