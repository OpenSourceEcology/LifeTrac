# Portenta X8 — OpenOCD Register-Poke (Option P0-C) toolkit

This directory contains TCL scripts for the **OpenOCD register-poke** strategy
documented in
`LifeTrac-v25/AI NOTES/2026-05-07_Portenta_X8_Murata_OpenOCD_Register_Poke_Plan_Copilot_v1.0.md`
plus the corrections in the assistant's review (RMW writes, IDR readback,
halted-mode pin sweep).

The scripts must run **on the X8 Linux host**, not on the dev PC. They use the
on-board OpenOCD + `imx_gpio` adapter that already programs the M4 via
`monitor-m4-elf-file.service`.

## Files

| File | Purpose |
|---|---|
| `01_halted_dump.cfg` | Halt M7, dump every GPIO bank's MODER + IDR to stdout. Zero side effects. |
| `02_assert_pg7_pc7.cfg` | Halt M7, RMW-enable GPIOC/G clocks, set PG_7 high (BOOT0), pulse PC_7 (NRST). Reads IDR back to confirm. Then sleeps so a second SSH session can run `stm32flash`. |
| `03_boot0_hunt.cfg` | Halt M7. For every output-capable pin in GPIOA..K (skipping PC_7 which is NRST), set HIGH, prompt the operator to send 0x7F via ttymxc3 in a second shell, log result. Slow but definitive identifier of the real BOOT0 net. |
| `99_release_and_reset.cfg` | Halt M7, release all writes, `reset run`. Use this after a successful flash (or as cleanup) to bring the H7 back online. The cleanest recovery is still a power cycle. |
| `verify_l072_rom.sh` | (Linux-side) Probe `/dev/ttymxc3` — first sends AT (should now be silent if BOOT0 took effect), then sends 0x7F at 8E1 (should ACK with 0x79). |

## Usage on the X8

Push this whole directory to the X8 once:

```powershell
adb -s 2E2C1209DABC240B push LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper /tmp/lifetrac_p0c
```

Then on the X8 (SSH or `adb shell`):

```bash
sudo openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
             -f /tmp/lifetrac_p0c/01_halted_dump.cfg
```

`/usr/arduino/extra/openocd_script-imx_gpio.cfg` ends with `init; targets; reset
halt`, so by the time our overlay runs the M7 is already halted at the System
Bootloader (PC=0x1ff09abc per the v1.0 RTT note). Our scripts assume that and
do not call `init` themselves.

## Important caveats (carried over from the assistant's review)

1. **The H7 is halted in System Bootloader, not mid-x8h7-firmware.** This is
   actually better — bootloader doesn't touch GPIOC/G. But it also means that
   if the L072 still answers AT after we drive PG_7 high, it's because PG_7 is
   *not* the L072 BOOT0 net on the X8 + Max Carrier SKU pair, not because we
   lost a race with x8h7.
2. **Always RMW.** Bare `mww` of MODER would clobber Ethernet RMII pins on
   GPIOG (PG_11/13/14) and SDMMC1 pins on GPIOC (PC_8..12). The scripts use
   read-modify-write throughout.
3. **Always read IDR after BSRR.** This is the single best diagnostic — it
   tells us whether the bit went high electrically (vs. MODER lock, external
   pull, or wrong pin).
4. **Keep openocd alive for the entire flash.** `02_assert_pg7_pc7.cfg` ends
   with a `gets stdin` that holds the session open until the operator hits
   Enter. Don't Ctrl-C until stm32flash has finished and verified.
5. **x8h7 bridge will stall** once openocd halts the H7. `gpio167`, `gpio163`,
   etc. will return `Connection timed out`. This is expected. Plan a power
   cycle as the final cleanup step.

## Bench tips & tricks (post-recovery gotchas, 2026-05-12)

These are bench failures that have bitten us repeatedly. Each entry documents
the symptom, the root cause, and the fix already wired into the launcher
scripts in this directory. **If you hit one of these again, do not re-debug
from scratch — apply the listed fix and move on.**

### TT-1: `OpenOCD: Error connecting DP: cannot read IDR` after a fresh `uuu` reflash

**Symptom.** First `imx_gpio` OpenOCD attach on a freshly reflashed X8 fails
with `Error connecting DP: cannot read IDR`. `run_stage1_standard_quant_end_to_end.ps1`
reports `final_result=FAIL_SYNC` on every cycle. A side-by-side `cat
/sys/class/gpio/gpio10/value` shows `0` on the freshly flashed board and `1`
on a known-good reference board.

**Root cause.** A freshly flashed LMP image leaves H7 NRST asserted: kernel
GPIO line `gpio10` (which is wired to H7 NRST via the `imx_gpio`
`reset_config srst_only srst_push_pull` pin map at `srst_num 10`) boots LOW.
With NRST held low, the H7 SWD DP cannot respond, so OpenOCD reads 0 from
DPIDR. Reference boards have `gpio8/10/15` pre-exported with `gpio10=1` as a
side effect of prior workflows — that state does **not** persist across
reflash.

**Fix (already in `run_stage1_standard_quant_end_to_end.ps1`).** Run a GPIO
preflight before the cycle loop:

```bash
echo fio | sudo -S -p '' bash -c '
  if [ ! -d /sys/class/gpio/gpio10 ]; then echo 10 > /sys/class/gpio/export; fi
  echo out > /sys/class/gpio/gpio10/direction
  echo 1 > /sys/class/gpio/gpio10/value
  for n in 8 15; do
    if [ ! -d /sys/class/gpio/gpio$n ]; then echo $n > /sys/class/gpio/export; fi
  done
'
```

After this, `cat /sys/class/gpio/gpio10/value` returns `1`, NRST releases,
and OpenOCD reads `Info : SWD DPIDR 0x6ba02477`. **Mirror this preflight in
any other launcher you write that runs OpenOCD against a freshly reflashed
X8** (e.g. future stage-2/stage-3 quants).

### TT-2: PowerShell `Invoke-Adb` wrapper gives false-fail on chmod / fire-and-forget calls

**Symptom.** `run_stage1_standard_quant_end_to_end.ps1` aborts with `chmod
failed` even though the script ran fine on the X8. Re-running by hand works.
The "rc" printed by the wrapper looks like an array, not an int.

**Root cause.** The local `Invoke-Adb` helper returns
`(stdout-array, $LASTEXITCODE)`. When the wrapped command emits ANSI / sudo
TTY escape sequences to stdout (e.g. `adb exec-out 'echo fio | sudo -S
chmod ...'`), the captured "rc" becomes a `[string[]]` containing the
escapes, not an `[int]`. `$rc -ne 0` is then always true.

**Fix.** Bypass the wrapper for fire-and-forget calls — discard stdout and
read `$LASTEXITCODE` directly:

```powershell
& adb @prefix exec-out $cmd | Out-Null
$rc = $LASTEXITCODE
if ($rc -ne 0) { throw "command failed: rc=$rc" }
```

The chmod call in `run_stage1_standard_quant_end_to_end.ps1` already does
this. **If you copy/paste a new `Invoke-Adb` call for a no-output command,
prefer this pattern.** A long-term fix would be to make `Invoke-Adb` always
return only `[int]` and route stdout through `Out-String` separately.

### TT-3: `uuu` recovery — version + warm-reset traps

**Symptom A.** Bundled `uuu 1.5.109` (from older NXP `MfgTool` releases)
fails partway through the recovery flow on Win11 against the current X8 LMP
image.

**Fix A.** Use **`uuu 1.5.243`** (or newer). Verified end-to-end (12/12
stages `Okay`) on Board 1 2026-05-12.

**Symptom B.** After `FB: ucmd reset` from a temp `.uuu` script, host returns
`LIBUSB_ERROR_PIPE` (expected) but the board never re-enumerates as fastboot
*or* adb. Polling for ≥55 minutes does nothing.

**Fix B.** Don't poll forever. If the board hasn't re-enumerated within
~2 minutes of the warm reset, escalate to a **physical USB-C + 12 V
power-cycle** (unplug both, wait 10 s, replug). The post-cold-boot adb
attach is reliable.

### TT-4: i.MX 8M Mini SDP PID is not always `012B`

**Symptom.** `diagnose_x8_recovery.ps1` reports "no SDP device present" or
the recovery plan's "is the mask ROM alive?" check returns empty even though
the BOOT DIPs are set correctly and `uuu` itself can see the device.

**Root cause.** The SDP PID varies by mask-ROM revision: older boards
enumerate as `VID_1FC9&PID_012B`, newer boards (observed on Board 1
2026-05-12) as `VID_1FC9&PID_0134`. The original regex only matched `012B`.

**Fix.** Already patched: `diagnose_x8_recovery.ps1` and the recovery plan
both now match `VID_1FC9&PID_(012B|0134)`. **If you write a new SDP-presence
check, use the same widened regex.**

### TT-5: Always reach for the reference-board diff before debugging OpenOCD config

If the L072 flash pipeline regresses on one board but works on another,
compare these *before* touching `imx_gpio` config, OpenOCD versions, or Tcl
scripts:

```bash
# On both boards:
cat /sys/class/gpio/gpio8/value /sys/class/gpio/gpio10/value /sys/class/gpio/gpio15/value
ls /sys/class/gpio/
uname -r              # kernel version
openocd --version     # OpenOCD version + build date
```

A `gpio10` mismatch (1 on good, 0 on bad) is the canonical TT-1 signature
and saves an hour of OpenOCD verbose-log archaeology.

### TT-6: PowerShell `Start-Process -ArgumentList` mangles nested-quoted shell strings

**Symptom.** A two-board orchestrator (e.g.
`run_w1_10b_rx_pair_end_to_end.ps1`) launches an `adb exec-out` background
process via `Start-Process`. Stdout file shows
`fio: -c: line 1: unexpected EOF while looking for matching ''` and the
remote command never starts. The *same* `sh -lc '...'` string works fine
when invoked via the PowerShell call operator (`& adb ... exec-out
$wrapped`) elsewhere in the codebase.

**Root cause.** `Start-Process -ArgumentList @(...)` re-quotes args
containing single quotes/spaces using Windows arg-quoting rules, which
silently breaks any nested `sh -lc 'echo ... | sudo -S -p '\'\'' ...'`
embedded escape sequence. The PowerShell call operator `&` uses a
different (native-style) quoting path that preserves the inner quotes.

**Fix (already applied to W1-10b).** Don't pass a multi-quoted shell
string through `Start-Process -ArgumentList`. Instead, write the command
to a tiny `.sh` wrapper file (with **LF line endings** — `[System.IO.File]::WriteAllText`
plus `-replace "`r`n", "`n"`), `adb push` it to `/tmp/lifetrac_p0c/_xxx_wrap.sh`,
and let `Start-Process` invoke a single, quote-free arg:

```powershell
$wrapBody = "#!/bin/sh`necho fio | sudo -S -p '' env FOO=bar bash /tmp/.../runner.sh`n"
[System.IO.File]::WriteAllText($wrapLocal, ($wrapBody -replace "`r`n", "`n"))
& adb -s $serial push $wrapLocal /tmp/lifetrac_p0c/_wrap.sh | Out-Null
& adb -s $serial exec-out "chmod +x /tmp/lifetrac_p0c/_wrap.sh" | Out-Null
$proc = Start-Process -FilePath "adb" `
    -ArgumentList @("-s", $serial, "exec-out", "bash /tmp/lifetrac_p0c/_wrap.sh") `
    -RedirectStandardOutput $stdoutFile -RedirectStandardError $stderrFile `
    -PassThru -NoNewWindow
```

Foreground sync calls can keep using `& adb @prefix exec-out $wrapped` —
that path is fine.

### TT-7: `Start-Process -NoNewWindow` and `-WindowStyle Hidden` are mutually exclusive

PS5.1 throws `Parameters "-NoNewWindow" and "-WindowStyle" cannot be
specified at the same time.` Pick one. For a hidden background `adb`
helper, use **`-NoNewWindow`** (no `-WindowStyle`).

### TT-8: Bench EMC hygiene — always park SX1276 in SLEEP after a probe run

Per the 2026-05-10 "Radio Shutdown for EM Reduction (Bench-Only)" rule, the
SX1276 must be in LoRa SLEEP (`RegOpMode=0x80`) whenever the controller is
not actively transmitting or receiving on the bench. RXCONTINUOUS
(`0x85`) draws ~10 mA continuously and radiates. SLEEP drops to <1 µA
with effectively zero RF emission.

**Layer 1 — probe-side cleanup (active by default, no reflash needed).**
[`method_h_stage2_tx_probe.py`](method_h_stage2_tx_probe.py) `main()` finally:
calls `sleep_radio_safely(link, label=args.probe)` for **every** probe
mode (`tx`, `rx`, `rx_listen`, `tx_burst`, `regversion`, `fsk`,
`opmode_walk`). Opt-out: `--no-sleep-on-exit` (only use this if a
follow-on probe needs the radio left awake). On success the probe emits:

```
__RADIO_SLEEP_ON_EXIT__ (rx_listen) ack=0180 RegOpMode(post)=0x80 (target=0x80)
```

**Layer 2 — wrapper-level audit (enforces Layer 1 across all orchestrators).**
[`run_method_h_stage2_tx.sh`](run_method_h_stage2_tx.sh) tail-greps the
probe log for the SLEEP marker after every run and emits one of three
audit verdicts:

| Marker present | Audit verdict | Meaning |
|---|---|---|
| `__RADIO_SLEEP_ON_EXIT__` | `__BENCH_RADIO_SLEEP_AUDIT__=PASS` | Radio parked at 0x80, OK to leave bench |
| `__RADIO_SLEEP_ON_EXIT_WARN__` | `__BENCH_RADIO_SLEEP_AUDIT__=DEGRADED` | Best-effort sleep write failed (link/firmware fault); investigate radio state manually |
| neither | `__BENCH_RADIO_SLEEP_AUDIT__=MISSING` | Probe bypassed cleanup (new mode without finally, crash, link open failure, or `--no-sleep-on-exit`); **bench EMC rule violated**, do not start next test until resolved |

Because every higher-level orchestrator
([`run_w1_10b_rx_pair_end_to_end.ps1`](run_w1_10b_rx_pair_end_to_end.ps1),
[`run_method_h_stage2_tx_end_to_end.ps1`](run_method_h_stage2_tx_end_to_end.ps1),
[`run_stage1_standard_quant_end_to_end.ps1`](run_stage1_standard_quant_end_to_end.ps1),
ad-hoc `LIFETRAC_PROBE_MODE=...` invocations) routes through
`run_method_h_stage2_tx.sh`, the audit line lands in every evidence log
automatically — no per-orchestrator change needed when adding new bench tests.

**Layer 3 — firmware idle (opt-in, requires reflash).** Build with
`make CFLAGS=-DLIFETRAC_BENCH_RADIO_IDLE_SLEEP=1` to make the firmware
boot into LoRa SLEEP instead of RXCONTINUOUS. Default is `0` so
production binaries are unchanged. Do **not** enable this for RX-side
probes (rx_listen, rx_liveness) without first wiring an explicit wake
step — they assume `0x85` at entry.

**Adding a new bench probe?** As long as the new mode dispatches through
`main()` in `method_h_stage2_tx_probe.py`, the `finally:` covers it for
free. If you write a new entry-point script that talks to the radio
outside `method_h_stage2_tx_probe.py`, end it with the same
`write_reg(SX1276_REG_OP_MODE, 0x80)` call and emit
`__RADIO_SLEEP_ON_EXIT__` so the wrapper audit can verify it.


