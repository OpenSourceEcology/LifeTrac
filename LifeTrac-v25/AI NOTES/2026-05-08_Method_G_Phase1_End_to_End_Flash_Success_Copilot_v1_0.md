# Method G — Phase 1 End-to-End Flash SUCCESS

**Date:** 2026-05-08  
**Author:** Copilot (Claude Opus 4.7), bench-driven by user  
**Version:** v1.0  
**Status:** ★★ COMPLETE ★★ — first custom-firmware flash to Murata
CMWX1ZZABZ-078 from Portenta X8 Linux, no SWD on the L072.

This document closes the loop on Method G Phase 1. It supersedes nothing —
the Phase 0 breakthrough doc
(`2026-05-07_Portenta_X8_LoRa_Phase0_Crack_and_Phase1_Options_Copilot_v1_0.md`)
remains the authoritative pin-mapping reference.

---

## 1. TL;DR

A pure-Python AN3155 client (`LifeTrac-v25/tools/stm32_an3155_flasher.py`,
stdlib-only, 270 LOC) running on the Portenta X8 Linux side now reliably:

1. Enters the L072 system bootloader (PA_11=BOOT0 HIGH, PF_4=NRST pulsed
   LOW→HIGH, both poked from the H7 via internal `imx_gpio` openocd while
   the H7 stays halted in its own System Bootloader).
2. Speaks AN3155 over `/dev/ttymxc3` at 19200 8E1.
3. **Erases per-page** (mass erase NACKs on this Murata-flashed L072).
4. Writes 83 032 B of `mlm32l07x01.bin` in 325 × 256 B blocks at 1 340 B/s.
5. Verifies byte-for-byte by Read Memory.
6. Releases BOOT0 and pulses NRST so the freshly-flashed firmware boots.

End-to-end timing on the bench: **erase 4.6 s, write 62.0 s, verify 57.6 s,
total ≈ 124 s** (excluding openocd attach overhead).

Method G Phase 0 (qualify the bootloader pipeline) and the
flash-engine portion of Phase 1 are both done. We now own the path; we can
flash any L072 binary we build.

---

## 2. What was new today vs. yesterday

| Yesterday (2026-05-07) | Today (2026-05-08) |
|---|---|
| Pin map cracked, 0x79 ACK proven | Same map, full AN3155 protocol stack |
| openocd cfg holds BOOT0 for 60 s | Long-hold cfg `07_…_long.cfg` (600 s) |
| No flasher exists | `stm32_an3155_flasher.py`, stdlib-only |
| `python -c` smoke test only | `--verify` round-trip passes |
| BOOT0 left HIGH after probe | `08_boot_user_app.cfg` releases it cleanly |

---

## 3. The four hard problems that had to be solved

### 3.1 LmP Python lacks `termios` and `fcntl`

The X8 Linux Python 3.10.4 ships without the `termios`/`fcntl` C-extensions
(no `pyserial` in opkg, no apt, no gcc). Workaround:

- UART configured externally via `stty -F /dev/ttymxc3 19200 cs8 parenb -parodd -cstopb raw -echo`
  in the wrapper shell script.
- Python uses only `os.open(O_RDWR | O_NOCTTY)`, `select.select`, `os.read`,
  `os.write`. No mode changes from Python.
- `tcdrain` is replaced by a byte-count sleep:
  `time.sleep(len(data) * 11.0 / 19200 + 0.01)`
  (8 data + 1 start + 1 parity + 1 stop = 11 bit-times per byte).

Result: the flasher imports nothing outside `argparse, os, select, struct,
sys, time`.

### 3.2 Mass Extended Erase NACKs even after Write Unprotect

`dev_table.c` in the MKRWAN reference says STM32L07xxx (`0x447`) supports
mass erase (`F_NO_ME = 0`). On our Murata-flashed L072, the `0x44 / 0xFF 0xFF
/ XOR` sequence NACKs every time, and a `0x73` Write Unprotect followed by
re-sync and retry NACKs again. The `stm32.cpp` source from the same project
has the canonical guidance:

> Not all chips support mass erase. ... Erasing the flash page-by-page is the
> safer way to go.

Implementation (`cmd_extended_erase_pages`):

- Send `0x44 / ~0x44`, expect ACK.
- Build payload: `BE16(N-1) + N × BE16(page#) + XOR(payload)`.
- Wait ACK with timeout `N × 0.5 s`.

Batched as 100 pages per call (200 + 3 bytes payload, well below the AN3155
practical limit). 649 pages cover the 83 032-byte image; total erase time
4.6 s. We deliberately erase only what we write — the rest of the 1 536-page
chip is left intact for any factory data the Murata module might expect.

### 3.3 Output invisibility from buffered Python pipes

`tee` was capturing the bash side fine but the Python progress dots never
appeared because `python3` block-buffers stdout when piped. Fix: launch as
`python3 -u`. Now every `print()` flushes immediately and the log shows the
full erase / write / verify trace in real time.

### 3.4 `set -e` + `kill -9 + wait` race

The first AT probe scripts hung after the first `kill -9 $PID; wait $PID`.
SIGKILL produces exit status 137, which `set -e` treats as a failure. Fix:
remove `set -e` from any script that intentionally signals a child.

---

## 4. Bench results (verbatim)

```
=== running flasher (with --verify) ===
loaded /tmp/lifetrac_p0c/mlm32l07x01.bin: 83032 bytes
[1/6] sync (0x7F)
[2/6] Get
       bootloader version=0x31 cmds=00 01 02 11 21 31 44 63 73 82 92
[3/6] Get ID
       PID=0x0447
[4/6] Read Memory probe @ 0x08000000
       first16=00 50 00 20 25 2A 01 08 75 2A 01 08 75 2A 01 08
[5/6] Erase: try mass first, fall back to per-page (649 pages needed)
       mass erase NACKed -> Write Unprotect, re-sync, page-erase
       erased pages 0..99 (2.1 s elapsed)
       erased pages 0..199 (2.6 s elapsed)
       erased pages 0..299 (3.0 s elapsed)
       erased pages 0..399 (3.5 s elapsed)
       erased pages 0..499 (3.9 s elapsed)
       erased pages 0..599 (4.4 s elapsed)
       erased pages 0..648 (4.6 s elapsed)
       page erase OK in 4.6 s
[6/6] Write Memory: 83032 bytes in 325 blocks of <=256 B
       block   16/ 325    4096 B  3.1 s  1341 B/s
       ...
       block  325/ 325   83032 B  62.0 s  1340 B/s
       write OK in 62.0 s
[+]  Verify (read-back compare)
       verify OK in 57.6 s
DONE.
flasher exit code = 0
```

The `00 50 00 20 25 2A 01 08` first-16 read is the *real* Murata vector
table (SP=0x20005000, reset vector = 0x0801 2A25). Compare with yesterday's
read which was `... A1 FF 00 08 F1 FF 00 08 ...`-style — yesterday the
reset vector area wasn't fully populated, today it is the live image we just
verified. (Both are valid; what matters is that read-back and write-back
agree byte-for-byte.)

After flash, `boot_and_probe.sh` ran `08_boot_user_app.cfg`:

```
Phase C: drop BOOT0 (PA_11 -> LOW) so L072 boots from flash
  GPIOA_IDR.PA_11 = 0   (expect 0)
Phase D: pulse NRST (PF_4 LOW 250 ms then HIGH)
  GPIOF_IDR.PF_4 (low)  = 0   (expect 0)
  GPIOF_IDR.PF_4 (high) = 1   (expect 1)
DONE: BOOT0 LOW, NRST released. L072 should now run user firmware.
```

The follow-up AT probe captured 29 bytes
(`Error when receiving\n+ERR_RX\r`). The user firmware is the MKRWAN modem
build, which uses a binary CMW frame protocol on top of the UART, not raw
text AT — so it complains about our `AT\r\n` then ignores subsequent
malformed bytes until power-cycle. **This response is positive evidence the
chip booted from user flash**, not from the ROM bootloader (which would have
sent nothing in response to `AT\r\n`).

---

## 5. Reproducible recipe

From a Windows PowerShell with `adb` on PATH:

```powershell
$DEV = "2E2C1209DABC240B"   # X8 ADB serial
$REPO = "C:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25"
$HELPER = "$REPO\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper"

# Push the toolkit (one-time per change)
adb -s $DEV push "$HELPER\07_assert_pa11_pf4_long.cfg"  /tmp/lifetrac_p0c/
adb -s $DEV push "$HELPER\08_boot_user_app.cfg"         /tmp/lifetrac_p0c/
adb -s $DEV push "$HELPER\run_flash_l072.sh"            /tmp/lifetrac_p0c/
adb -s $DEV push "$HELPER\boot_and_probe.sh"            /tmp/lifetrac_p0c/
adb -s $DEV push "$REPO\tools\stm32_an3155_flasher.py"  /tmp/lifetrac_p0c/
adb -s $DEV push "$REPO\tools\mlm32l07x01.bin"          /tmp/lifetrac_p0c/

# Flash + verify (~3 min wall clock)
adb -s $DEV exec-out "echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/run_flash_l072.sh"

# After flash, release BOOT0 and reset
adb -s $DEV exec-out "echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/boot_and_probe.sh"
```

To flash a *different* binary, just replace `mlm32l07x01.bin` and re-run.
The flasher takes the path as `argv[2]`.

---

## 6. Known caveats and follow-ups

1. **Bridge stalls after every openocd run.** `cat /sys/kernel/x8h7_firmware/version`
   returns `Connection timed out` afterwards. This does NOT block the L072
   path because `/dev/ttymxc3` is i.MX-direct (not bridged). A power-cycle
   restores everything. Acceptable for development; will need to revisit
   when we want CAN/SPI/etc. usable in the same session.

2. **Long-hold cfg keeps re-asserting BOOT0 HIGH every 1 s.** The flasher
   tolerates this fine (the reassertion writes the same bit), but
   `08_boot_user_app.cfg` MUST run as a fresh openocd invocation —
   otherwise the holder script fights the release.

3. **Murata WRP state.** Mass erase NACKs because the Murata factory image
   appears to set Write Protection bits that survive Write Unprotect on this
   chip variant. Per-page erase works around it transparently. If we ever
   want full-chip wipe (e.g. to clear the option-byte mirror), we'll need
   `Readout Protect → Readout Unprotect` cycle (which DOES mass-erase as a
   side effect per ST AN3155). Not needed yet.

4. **AT probe shows binary modem.** The MKRWAN firmware is *not* a
   text-AT modem in the MKRWANv2 builds; it uses the CMW binary frame
   protocol. To validate a freshly-flashed image we should either (a) flash
   our own minimal hello-world binary that does emit text on UART, or (b)
   speak the CMW protocol from the X8. Option (a) is the next concrete
   bench step (covered by Phase 1 of `03_Bringup_Roadmap.md`).

5. **No `Go` command issued.** The flasher deliberately does NOT issue the
   AN3155 `Go` command after write. We always reset via NRST instead, which
   gives a clean POR-equivalent state for the user firmware. This matches
   the MKRWAN reference flasher's behaviour.

---

## 7. What this unlocks

- We can now flash *any* L072 image we build, in ~2 min, with no SWD on the
  Murata module, using only the parts of the X8 that are nominally
  customer-accessible (ADB + sudo + `/dev/ttymxc3` + internal openocd).
- The Method G Phase 1 deliverable "qualify the bootloader pipeline" is
  done. Phase 1 hello-world (compile a minimal `main.c` that emits
  `LIFETRAC L072 v0.1\r\n` at 19200 8N1) is the next concrete step.
- Phase 6 (signed boot + A/B slots + field updates) is now *plausible*
  because we've proven the chip can be reflashed without manual intervention
  and without SWD.

---

## 8. Files of record

| Path | Role |
|---|---|
| `LifeTrac-v25/tools/stm32_an3155_flasher.py` | Pure-Python AN3155 client |
| `LifeTrac-v25/tools/mlm32l07x01.bin`         | MKRWAN reference image (83 032 B) |
| `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/06_assert_pa11_pf4.cfg`      | 60 s bootloader-entry hold (Phase 0 cfg) |
| `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/07_assert_pa11_pf4_long.cfg` | 600 s flash-window hold |
| `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/08_boot_user_app.cfg`        | BOOT0 LOW + NRST pulse (post-flash boot) |
| `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_flash_l072.sh`           | End-to-end orchestrator |
| `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/boot_and_probe.sh`           | Post-flash boot + AT probe |
| `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/probe_at_clean.sh`           | Standalone AT probe (no openocd) |

Memory updates:
- `/memories/repo/lifetrac-portenta-x8-lora.md` — appended `2026-05-08
  METHOD G PHASE 0/1 COMPLETE` entry with full reproducible recipe and
  per-page erase rationale.

---

## 9. Decision log (for the architects)

- **Per-page erase, not mass-erase-via-RDP-cycle.** Faster (4.6 s vs the
  RDP-cycle's required 35 s mass-erase timeout), preserves option bytes,
  and works on Murata's WRP state without needing to know its details.
  We accept the cost of erasing 1.5× the bytes we write (we erase *all*
  pages we touch with a per-image granularity, not the whole flash).
- **No `Go` command.** NRST pulse is more deterministic and avoids the
  bootloader-vs-user-app SP/PC divergence that has bitten several STM32
  bootloader users in the wild.
- **Pure stdlib, no Docker.** A 270-LOC stdlib script is something we can
  read, audit, and modify in seconds. A Dockerised `stm32flash` would have
  required ~600 MB of image pull and a containerd dependency we don't want
  in the production gateway flow.
- **External `stty`, not `termios` from Python.** Lets us run on the
  factory LmP Python without any user-space additions. The cost is one
  extra line in the wrapper script.
- **openocd holds the H7 halted while we flash the L072.** The H7 is in its
  own System Bootloader anyway during this whole sequence; we haven't lost
  any H7 functionality we wouldn't have lost from the bridge stall. After
  power-cycle the H7 boots normally.

---

*End of document.*
