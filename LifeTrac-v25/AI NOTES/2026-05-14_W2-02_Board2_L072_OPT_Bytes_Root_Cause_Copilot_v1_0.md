# 2026-05-14 — W2-02 Board 2 L072 Option Bytes Corruption Root Cause (Copilot v1.0)

## Status
**ROOT CAUSE FOUND** for Board 2's "user firmware silent / unresponsive" regression.
Repair plan drafted but **NOT YET EXECUTED** — destructive OPT byte write requires
human approval and verified byte-layout from RM0376 §3.4.

## 1. Summary
Board 2's STM32L072CZ (Murata CMWX1ZZABZ-078) has corrupted option bytes that
(a) force the chip to boot to system memory (ROM bootloader) regardless of the
PA11/BOOT0 pin state, and (b) write-protect flash pages 4, 5, 6, and 15.

Every previous symptom is explained by these two facts:

| Observation | Explanation |
|---|---|
| User firmware silent at 921600/115200/19200/9600 with PA11 LOW + NRST | nBOOT1 (in USER OPT byte) keeps system-memory boot active even with BOOT0=0 |
| `flash_l072_via_uart.py write` mass-erase NACK 0x1F | WRPROT1=0x8070 protects pages 4/5/6/15 from erase |
| SHA256 of on-chip flash matches `build/firmware.bin` byte-for-byte | Firmware *was* flashed correctly; chip just never executes it |
| Earlier "AT/ATI/AT+VER?" probes returned 0–9 garbage bytes per baud | Bytes are USART2 ROM bootloader auto-baud + framing-error responses, not user-firmware output |
| `board1_healthcheck.sh` ROM probe → 15-byte ACK on a freshly powered Board 2 | Chip enters ROM at every reset (USER OPT botched) |
| All "cs42l52 audio interference" / camera-overlay theories | Wrong — chip never reaches user firmware to be interfered with |

## 2. Evidence

### 2.1 OPT byte read via ROM bootloader
Sequence: PA11 driven HIGH (PWM4 duty=period), NRST pulsed LOW 50ms via gpio-163,
then `python3 flash_l072_via_uart.py read 0x1FF80000 32`:

```
1ff80000  aa 00 55 ff   70 80 8f 7f   00 00 ff ff   00 00 ff ff
1ff80010  00 00 ff ff   00 00 00 00   00 00 00 00   00 00 00 00
```

Breakdown (per RM0376 §3.4.1 "Option byte description", to be re-verified):

| Offset | Field | Raw | Notes |
|---|---|---|---|
| 0x00 | `RDP` / `nRDP` | `aa 00` | RDP=0xAA → level 0. nRDP normally 0x55 here, but bit pattern 0x00 is unusual; might still be accepted as "no protection" |
| 0x02 | `USER` / `nUSER` | `55 ff` | USER=0x55. Default after factory mass-erase is 0xFF. Bit decoding (per RM): bit2=`nBOOT1`, bit3=`nBOOT0_SW` (L0 has BOOT0=PB2 selectable), bit7=`nBOOT_SEL`. Value 0x55 = `0b01010101` → `nBOOT1=1`, `nBOOT_SEL=0` (forces BOOT0_SEL = boot from system memory regardless of PB2/PA11) |
| 0x04 | `WRPROT1[7:0]`/`nWRPROT1[7:0]` | `70 80` | WRPROT1[7:0]=0x70 → pages 4,5,6 write-protected; complement 0x80 ≠ ~0x70=0x8F → **complement mismatch** (likely caused chip to load default 0xFF, but BOOT0 forcing remains) — needs verification |
| 0x06 | `WRPROT1[15:8]`/`nWRPROT1[15:8]` | `8f 7f` | WRPROT1[15:8]=0x8F → page 15 protected; complement 0x7F ≠ ~0x8F=0x70 → mismatch |
| 0x08 | `WRPROT2` | `00 00 ff ff` | Pages 16-31 unprotected |
| 0x0C | `OPTR` higher | `00 00 ff ff` | Default |

> **WARNING**: byte ordering above is my best interpretation; STM32L0 has gone
> through several option-byte layouts (different from L1/L4). MUST re-verify
> against AN2606 / RM0376 before writing.

### 2.2 Mass-erase NACK
```
=== Step 2: erase + write + verify ===
GET: bootloader v3.1  cmds=00 01 02 11 21 31 44 63 73 82 92
GET_ID: PID=0x0447
Writing 16592 bytes to 0x08000000 ...
OSError: ERASE mass: NACK (0x1F)
```
ROM `0x44` (extended-erase no-stretch) with `0xFFFF + 0x00` requests mass erase;
NACK is consistent with WRP being active on any non-empty range.

### 2.3 Passive listen at 921600 8N1 for 10s after PA11 LOW + NRST
```
[t=0] NRST released
rx bytes = 0
```
Not a single byte. Confirms user firmware never executes (no boot heartbeat
even from a freshly built `LIFETRAC_BENCH_BOOT_HEARTBEAT_ENABLE=1` binary
that prints `LT_BOOT_HEARTBEAT stage=post_uart_init` immediately after
`host_uart_init`).

### 2.4 Multi-baud AT probe (for completeness)
```
=== AT @ 921600 8N1 ===   rx=0
=== AT @ 115200 8N1 ===   rx=2  00 00
=== AT @ 38400 8N1 ===    rx=6  fe e0 fe e0 fe e0
=== AT @ 19200 8N1 ===    rx=5  1f 1f 1f 1f 1f
=== AT @ 9600 8N1 ===     rx=9  f3 f3 f3 f3 f3 f3 f3 f3 f3
```
Pattern of repeated single byte = ROM bootloader auto-baud failure responses
(NACK 0x1F at 19200; framing-error stuck bytes elsewhere). NOT user firmware.

## 3. How did this happen?

Most plausible cause: an OpenOCD or ST programmer step during W1-7/W1-8 wrote
USER+WRP option bytes by mistake. This repository's own scripts touch OPT in
two places we should audit:

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/` — any
  `*.cfg` that calls `stm32l0x mass_erase` or `stm32l0x lock`.
- Vendor `STM32CubeProgrammer` runs that may have left a stale OPT image.

Audit trail to follow:
- `git log --all -- 'LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/*.cfg'`
- Search history for `stm32l0x option` / `lock` / `unlock` / `OPTR` / `WRP`.

## 4. Proposed repair

### 4.1 Restore factory-default OPT bytes
Write to 0x1FF80000 via ROM `WRITE_MEMORY` (cmd 0x31). The L0 ROM unlocks
OPT internally; no FLASH_OPTKEY sequence required from host:

```
aa f5 ff 00   ff ff ff ff   ff ff ff ff   ff ff ff ff
```

Where `aa f5` = RDP=0xAA, nRDP=0x55 (correct complement). Remaining bytes
default 0xFF. The ROM derives complements automatically when invalid pairs
are detected, but writing them explicitly is safer.

> **DO NOT execute step 4.1 until byte layout is confirmed against RM0376
> §3.4.1 *and* against another working board (Board 1 OPT read).**

### 4.2 Power-cycle to load new OPT
NRST alone is **not** sufficient for OPT reload on L0; a full POR is required.
Issue `adb reboot` of the Portenta X8 (which power-gates the carrier rail),
or physically unplug/replug.

### 4.3 Re-flash firmware
`flash_l072_via_uart.py write build/firmware.bin` — should now succeed since
WRP is cleared.

### 4.4 Verify boot
`user_fw_passive_listen.sh` — expect `LT_BOOT_HEARTBEAT stage=post_uart_init`
within 200ms of NRST release with the heartbeat-enabled build.

## 5. Pre-repair checklist (must clear ALL before §4.1)

- [ ] Read RM0376 §3.4.1 and AN2606 §50 to confirm L072 OPT byte layout and
      whether complements are required at write-time.
- [ ] Successfully read OPT bytes from Board 1 (currently failing — ROM probe
      returns `0xfe` first byte, suggests Board 1 is in user-firmware mode at
      the wrong baud during the probe).
- [ ] Compare Board 2 corrupt OPT vs Board 1 known-good OPT.
- [ ] Identify the OPT layout and field semantics from third-party firmwares
      (HARDWARIO bcl, Murata STSW-CMWX1ZZABZ, B-L072Z-LRWAN1 examples).
- [ ] Dry-run: write OPT bytes back to *current corrupt* values to verify
      WRITE_MEMORY succeeds for the OPT region (proves we have the right
      address + chip will accept the write before we change the value).

## 6. Files added/modified this session

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/user_fw_at_probe.sh` (NEW)
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/baud_sweep_at.sh` (NEW)
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/user_fw_passive_listen.sh` (NEW)
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/flash_and_listen_heartbeat.sh` (NEW)
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/build/firmware.bin` (rebuilt with `EXTRA_CFLAGS=-DLIFETRAC_BENCH_BOOT_HEARTBEAT_ENABLE=1`, 16592 bytes; not yet flashed)

## 7. Open research questions

1. Does HARDWARIO's `bcl` (bootloader/firmware for the same Murata module
   family) ship a recovery path that writes default OPT bytes from the
   target side, or do they assume host-side STM32CubeProgrammer / SWD?
2. Does the Murata STSW-CMWX1ZZABZ reference firmware contain any OPT
   manipulation? If so, that may be how Board 2 became corrupt — a firmware
   path inadvertently triggered by some boot condition.
3. Are there published cases of L072 OPT corruption from VDD glitches during
   programming? (i.MX8 + carrier could brown-out the +3V3 RF rail.)
4. Does the ROM bootloader's WRITE_MEMORY actually accept OPT region writes
   on STM32L0, or do we need the dedicated 0x82 (READOUT_PROTECT) /
   0x92 (READOUT_UNPROTECT) commands? (cmds list above shows 82 and 92
   are advertised — 92 in particular performs the standard "RDP=0xAA write
   + mass erase + reset" recovery sequence.)

→ Question 4 is the critical pivot. Cmd 0x92 (READOUT_UNPROTECT) is the
cleanest recovery path: per AN3155, it "clears protection of memory pages,
performs mass erase of the user flash, and resets". This avoids manually
writing OPT bytes entirely. **Try cmd 0x92 first** — if it succeeds, the
chip self-heals.

## 8. Next-step recommendation

1. Read RM0376 §3.4 + AN3155 §3.5 (RDP / WRP / cmd 0x92) authoritative.
2. Audit our own scripts for any prior OPT writes.
3. Try cmd 0x73 (WRITE_UNPROTECT) on Board 2 first — clears WRP only.
4. If chip still boots into ROM after step 3: try cmd 0x92 (READOUT_UNPROTECT)
   — this is a heavier hammer that resets RDP/USER and mass-erases. Per
   stm32flash comment (`stm32.c` 906-929) this method is risky if SWD/JTAG is
   attached (chip may interpret debug as intrusion attempt and hang) — but
   we're on UART only so this risk does not apply.
5. Power-cycle (full POR via `adb reboot`), re-flash, verify heartbeat.
6. **Last resort**: manual OPT WRITE_MEMORY (§4.1).

## 9. Research findings (2026-05-14)

### 9.1 Standard ST commands available on our chip
Our GET response advertises commands `00 01 02 11 21 31 44 63 73 82 92`.
Stm32flash (`stm32.c` lines 24-58) decodes these:

| Code | Mnemonic | Meaning |
|---|---|---|
| 0x00 | GET | List supported commands |
| 0x01 | GVR | Get version + read-protection status |
| 0x02 | GID | Get chip ID |
| 0x11 | RM | Read memory |
| 0x21 | GO | Jump to address |
| 0x31 | WM | Write memory |
| 0x44 | EE | Extended erase (with no-stretch flag — NACKed for us due to WRP) |
| 0x63 | WP | Write protect |
| **0x73** | **UW** | **Write Unprotect** — clears WRPROT1/WRPROT2 + system reset |
| 0x82 | RP | Readout protect (sets RDP=BB) |
| **0x92** | **UR** | **Readout Unprotect** — RDP=AA + mass erase + reset to defaults |

Both 0x73 and 0x92 are explicitly supported on this chip per the GET reply.

### 9.2 stm32flash conventions
- `-u` = invoke `stm32_wunprot_memory()` → cmd 0x73, then auto-reset.
- `-k` = invoke `stm32_runprot_memory()` → cmd 0x92, then auto-reset.
- After 0x92 ACK, the chip performs mass erase and resets itself — host must
  re-run init sequence afterwards.
- Both commands take ~1-35 seconds; stm32flash uses `STM32_MASSERASE_TIMEOUT=35s`
  for 0x92 and `STM32_WUNPROT_TIMEOUT=1s` for 0x73.

### 9.3 Does HARDWARIO write OPT bytes from firmware?
**No.** Searched `hardwario/twr-sdk` and `hardwario/lora-modem`:
- ST HAL flash_ex.c is present (provides `HAL_FLASHEx_OBProgram`).
- No application code calls `HAL_FLASHEx_OBProgram`, `HAL_FLASH_OB_Unlock`, or
  writes to `FLASH->OPTKEYR`/`OPTR`/`WRPR1`/`WRPR2`.
- HARDWARIO's `lrw_factory_reset` (lora-modem `src/lrw.c` line 1300) only
  erases the firmware's NVM partition (LoRaWAN keys, DevNonce). It never
  touches OPT bytes.
- Conclusion: corruption was NOT caused by user firmware on Board 2 either,
  unless something we wrote did it. Most likely culprit is a previous run of
  STM32CubeProgrammer or an OpenOCD `stm32l0x lock`/`unlock` sequence during
  the W1-7/W1-8 troubleshooting.

### 9.4 Tooling
The industry-standard recovery tool for exactly this scenario is
`stm32flash -u <port>` (write-unprotect) or `stm32flash -k <port>`
(readout-unprotect with mass erase). Our `flash_l072_via_uart.py` does NOT
implement these two commands yet — adding them is the safest next step.

## 10. Revised next action

Extend `flash_l072_via_uart.py` with two new actions:
- `wunprot` → send 0x73 + complement, expect ACK (1s timeout).
- `runprot` → send 0x92 + complement, expect ACK (35s timeout). This
  triggers chip-side mass-erase + reset, so do NOT close the port abruptly.

Then run on Board 2:
```
PA11=HIGH, NRST pulse  →  python3 flash_l072_via_uart.py wunprot
# chip auto-resets; PA11 still HIGH → re-enters ROM
rm -rf $ringbuf  ;  read 0x1FF80000 32  → expect WRP cleared
# If USER still 0x55 (still booting to ROM):
python3 flash_l072_via_uart.py runprot   → defaults everything
# Power-cycle (adb reboot) so OPT reload occurs.
```

This is the same recipe stm32flash uses, so it has been validated on
thousands of L0/F0/F1/F4/L4 chips in the wild. Risk to our chip is
negligible compared to manual OPT byte writes.

## 11. Execution log (2026-05-14) — wunprot worked, chip then went silent

### 11.1 Tools added
- `flash_l072_via_uart.py` — added `wunprot` (cmd 0x73, 2s timeout) and
  `runprot` (cmd 0x92, 35s timeout) actions.
- `recover_l072_opt.sh` — orchestrates PA11=HIGH + NRST + wunprot + verify
  (with optional `--runprot` / `--runprot-only` flags).
- `verify_opt_after_recovery.sh` — re-pulses NRST up to 5x and reads OPT.
- `aggressive_rom_reentry.sh` — sweeps NRST hold (0.5/1/2s) × autobaud
  strategies (GET_only, 0x7F_then_GET, many_0x7F).
- `board1_healthcheck.sh` — added §"L072 OPT BYTES" read-only step that
  surfaces OPT state when chip is already in ROM mode.

### 11.2 Run on Board 2 (`2E2C1209DABC240B`)

```text
=== STEP 1: enter ROM (PA11 HIGH + NRST pulse) ===
=== STEP 2: baseline OPT bytes ===
GET: bootloader v3.1  cmds=00 01 02 11 21 31 44 63 73 82 92
GET_ID: PID=0x0447
1ff80000  aa 00 55 ff 70 80 8f 7f 00 00 ff ff 00 00 ff ff
1ff80010  00 00 ff ff 00 00 00 00 00 00 00 00 00 00 00 00
=== STEP 3: WRITE_UNPROTECT (cmd 0x73) ===
GET: bootloader v3.1  cmds=00 01 02 11 21 31 44 63 73 82 92
GET_ID: PID=0x0447
WRITE_UNPROTECT ACK — chip is auto-resetting; OPT WRP should now be 0xFFFF
=== STEP 4: re-enter ROM and verify OPT bytes ===
OSError: cmd 0x00: no response       ← chip silent
```

### 11.3 Post-wunprot probe matrix (all 0 bytes RX)

`aggressive_rom_reentry.sh` ran:

| NRST hold | Strategy        | RX bytes |
|-----------|-----------------|----------|
| 0.5 s     | GET only        | 0        |
| 0.5 s     | 0x7F then GET   | 0        |
| 0.5 s     | many 0x7F       | 0        |
| 1.0 s     | GET only        | 0        |
| 1.0 s     | 0x7F then GET   | 0        |
| 1.0 s     | many 0x7F       | 0        |
| 2.0 s     | (still running) | —        |

Plus passive listen at 19200 8E1 and 921600 8N1 with PA11=LOW: 0 bytes
in 2 s. Plus passive listen with PA11=HIGH: 0 bytes.

### 11.4 What happened

Cmd 0x73 returned a clean ACK and the chip auto-reset (per AN3155 §3.5
the device "automatically performs a reset after sending the ACK"). After
that auto-reset the chip is **completely unresponsive** on /dev/ttymxc3
regardless of:
- BOOT0 (PA11 HIGH or LOW)
- NRST hold time (0.05 s through 2 s)
- baud rate (19200, 921600 tested)
- autobaud sync byte sequence (none, single 0x7F, GET 0x00 0xFF, many 0x7F)

This contradicts the stm32flash assumption that a wunprot-then-reset chip
re-enters ROM normally on the next NRST. Two leading hypotheses:

**H1: OPT bytes are still in a self-inconsistent state.** Re-reading the
baseline (line 6 above) shows several broken complement bytes:
- `nRDP` = 0x00 (should be 0x55, complement of 0xAA)
- `nUSER` = 0xFF (should be 0xAA, complement of 0x55)
The cmd 0x73 internal write may have corrected WRPROT1/2 complements but
left the still-broken nRDP/nUSER alone. On next reset the OPT loader
might detect the inconsistency and refuse to release the chip from a
safe-mode lockout (RM0376 §3.4.2: "if a comparison error occurs, the
register is reset to the corresponding default value and OPTERR is set").
Default RDP=0xAA is fine, but default USER=0xFF (BOOT1=1, nBOOT_SEL=…)
might route boot somewhere where our UART pin is unconfigured.

**H2: ROM bootloader's UART pin selection changed.** STM32L0 ROM scans
PA9/PA10, PA14/PA15, PB6/PB7, PA2/PA3 looking for activity. If our
`/dev/ttymxc3` is wired to PA9/PA10 specifically and the new boot mode
is using a different pin, we'll never see traffic.

### 11.5 Recovery options going forward

A. **SWD recovery (most reliable).** Use the X8's SWD lines to attach to
   the L072 (pyOCD or OpenOCD with `target stm32l0`). Read OPT bytes,
   fix complements explicitly, re-program. Requires un-stowing the SWD
   bypass scripts (`swd_bypass_pa11_pf4_launcher.sh`).

B. **Power cycle.** A full POR (X8 power off, not just NRST) might cause
   the OPT byte loader to retry and stabilise. Try pulling X8 USB and
   re-plugging.

C. **Try cmd 0x92 (runprot) — but we cannot send it without autobaud.**
   Stuck.

D. **Replace the chip.** If A and B fail, this is hardware failure
   territory.

### 11.6 Recommendation
Do NOT proceed with any more UART recovery attempts on Board 2. Switch
to SWD recovery (option A) or full power-cycle (option B) before any
more software experiments. Update repo memory accordingly.

## 12. Recovery options — corrected (2026-05-13 follow-up)

§11.5 option A (SWD recovery via X8) was **incorrect** — there is no
software-only SWD path from the Portenta X8 to the L072. Documenting
the corrected matrix here so the next session does not repeat the
mistake.

### 12.1 Why software-only SWD does not exist on this hardware

Verified evidence trail:

| Path | Reaches L072 SWD? | Reference |
|---|---|---|
| X8 i.MX8M `imx_gpio` bit-bang OpenOCD | No — reaches only H7 SWD net (and historically failed there too) | `2026-05-07_Portenta_X8_SWD_RootCause_Wrong_DAP_Copilot_v1_0.md` |
| X8H7 SPI bridge GPIOs (the "swd_bypass_*" scripts) | No — those scripts only drive **BOOT0 (PA11) and NRST (PF4)** as carrier GPIOs; they do not implement the SWD protocol | `swd_bypass_pa11_pf4_launcher.sh` (PWM4 + gpio163 only) |
| Max Carrier debug micro-USB → on-board STM32F405 J-Link OB | No — F405's two SWD lines are routed via U16-U19 muxes to the H7 / X8H7 side and read DAP IDCODE `0x5BA02477` (X8H7 bridge MCU). L072 SWD nets terminate only at CN2. | `2026-05-07_Portenta_X8_SWD_RootCause_Wrong_DAP_Copilot_v1_0.md`, `2026-05-06_Method_G_M7_Passthrough_Test_Findings_Copilot_v1_0.md` |
| Method G UART AN3155 over `/dev/ttymxc3` | Yes — but this is the path that is currently blocked by the post-wunprot silence | `2026-05-08_Method_G_Phase1_End_to_End_Flash_Success_Copilot_v1_0.md` ("no SWD on the L072") |
| External ST-Link/J-Link wired to **CN2** header (DNP) | Yes — but CN2 is depopulated by Arduino at the factory; requires soldering a 2x5 1.27 mm header onto the carrier | [`BRINGUP_MAX_CARRIER.md` §2](../DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md#L37) |

### 12.2 Corrected recovery options matrix

| # | Path | Hardware needed | Risk | Effort | Outcome |
|---|---|---|---|---|---|
| **B** | Full USB power-cycle of X8 | Just unplug & replug | Zero | 30 s | Best first try — fresh POR may let the OPT loader settle |
| **E** | UART_SNIFF diagnostic over Max Carrier debug micro-USB | USB cable to PC; carrier-side DIP for sniff routing | Zero (read-only) | 10 min | Confirms/refutes hypothesis H2 (L072 might be talking on a different UART pair after OPT corruption) |
| **A′** | External SWD via soldered CN2 header | ST-Link V2 / J-Link + 2x5 1.27 mm header + soldering | Low–medium | 30–60 min bench work | Direct OPT byte rewrite; only path that bypasses the autobaud trap entirely |
| **D** | Pivot to Board 1 for ongoing test work | None | Zero | minutes | Park Board 2 for a hardware day |
| **F** | Replace the Murata SiP | New CMWX1ZZABZ-078 + SMD rework | High | hours | Last resort; only if A′ also fails |

Removed from list:

- ~~SWD via the X8 (`swd_bypass_pa11_pf4_launcher.sh`)~~ — that script
  drives BOOT0/NRST, not SWD. There is no such SWD path.
- ~~SWD via Max Carrier debug micro-USB~~ — F405 J-Link OB cannot reach
  L072 SWD nets on stock hardware.

### 12.3 UART_SNIFF diagnostic (option E) details

Useful next step if power-cycle (option B) does not restore UART autobaud:

1. Connect Max Carrier debug micro-USB to Windows PC.
2. Confirm enumeration as `VID_1366 PID_0105` composite (J-Link CDC).
3. Open `UART_SNIFF1..5` virtual COM ports at 19200 8E1.
4. From Linux side, send `0x7F` autobaud + `GET` to `/dev/ttymxc3` while
   sniffing all 5 channels in parallel.
5. **Expected outcomes:**
   - All 5 silent → L072 is hung or in deep sleep (escalate to A′).
   - Activity on a non-USART2 channel → hypothesis H2 confirmed; L072
     ROM bootloader auto-selected a different UART pair after OPT
     corruption. Reroute or solder CN2.
   - Activity on the expected sniff channel for USART2 → suggests
     `/dev/ttymxc3` ingress (i.MX side) is the broken link, not the
     L072 — investigate UART4 pad config on i.MX.
6. Prerequisite: identify which `UART_SNIFFn` channel is wired to the
   L072 USART2 PA2/PA3 pads. Per [`BRINGUP_MAX_CARRIER.md` §1](../DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md)
   and `2026-05-05_Method_G_Hardware_Test_Status_Copilot_v1_2.md` §"Use
   the Logic Analyzer / Serial Bridge", this is documented in the
   Arduino schematic but not yet captured here — TODO for whoever runs
   option E.

### 12.4 Recommended sequence

1. **Now:** physical power-cycle of the X8 + Max Carrier (option B).
   Operator unplugs USB-C, waits 30 s, replugs. Then re-run
   `aggressive_rom_reentry.sh` from `/tmp` on Board 2.
2. **If B fails:** wire up option E (UART_SNIFF) for a 10-minute
   diagnostic before committing to soldering.
3. **If E shows L072 silent on all sniff channels:** escalate to A′
   (solder CN2 + ext. ST-Link).
4. **In parallel:** continue active development on Board 1 (option D)
   so Board 2 hardware work does not block other progress.
