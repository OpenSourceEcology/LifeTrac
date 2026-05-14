# Portenta X8 Board 1 (`2D0A1209DABC240B`) — Recovery Plan & Tiered Playbook

**Author:** Copilot · **Version:** 1.0 · **Date:** 2026-05-12

---

## 1. Question (verbatim from user)

> "can we research ways to save the x8 that wont connect? can we use the reset command?"

**Direct answer to the reset question:** No software-only `reset` / `reboot` command from
the host PC will help in the current state. `adb reboot` requires a functional `adbd` on
the X8 — and `adbd` is exactly what's not responding. The X8 itself has Linux-level
`reboot` / `systemctl reboot`, but they are unreachable when ADB is down. **Reset only
helps when something on the host can still talk to the device.** When neither ADB nor a
serial console responds, only physical/electrical reset paths remain (Max Carrier
button, USB-C+12 V power-cycle, or `uuu` re-flash with the BOOT DIPs set).

---

## 2. Diagnosis (run 2026-05-12)

| Check | Board 1 (`2D0A1209DABC240B`) | Board 2 (`2E2C1209DABC240B`) |
|---|---|---|
| Windows USB enumeration (`Get-PnpDevice`) | **OK** — `USB Composite Device`, `USB Serial Device (COM11)`, `ADB Interface` MI_02 — `Status=OK`, `Problem=CM_PROB_NONE` | **OK** — same set, COM12 |
| Driver binding | WINUSB.sys, `DriverDesc=ADB Device` (identical to Board 2) | WINUSB.sys, `DriverDesc=ADB Device` |
| `adb devices` after `adb kill-server; adb start-server` | **NOT listed** | listed `device` |
| `adb -s 2D0A... get-state` | `error: device not found` | `device` |
| COM11 open + write at 115 200 8N1 | port opens, **`WriteLine` times out** (no Linux getty consuming bytes) | n/a |

**Conclusion.** USB layer is healthy on both boards. On Board 1 the i.MX 8M Mini Linux
kernel still presents the USB-gadget endpoints (composite `acm + adb`) to the host, but
**no userland service is responding** — neither `adbd` (MI_02) nor the serial console
getty (MI_00). This is a classic *soft hang* of the X8's Linux user-space (or a stuck
shutdown/halt waiting on a watchdog bite). The host PC has no remaining channel to
drive Linux, so nothing here can be fixed from the keyboard.

---

## 3. Recovery hierarchy (try in order — cheapest first)

### Tier 0 — Already attempted, no effect
- [x] `adb kill-server; adb start-server; adb devices` → still missing
- [x] `adb -s 2D0A1209DABC240B reconnect` → device-not-found
- [x] COM11 probe at 115200 8N1 → opens, no response from getty
- [x] `revive_bridge.sh` (would be next) — **documented INEFFECTIVE** in repo memory; skip

> Helper script `diagnose_x8_recovery.ps1` re-runs all of Tier 0 + Tier 1 in one shot
> (see §5). Run it first whenever a board "drops out" — it tells you *which* tier is the
> first one that requires hardware contact.

### Tier 1 — Host-side, no hardware contact
1. **Different USB cable.** The Arduino docs explicitly call out USB-C↔USB-C cables as
   problematic for the X8 (the data-role negotiation can leave the gadget half-up).
   Use a USB-C↔USB-A cable from a known-good supply.
2. **Different USB-A port on the host.** Prefer a chipset-root-hub port (rear panel on
   a desktop). Avoid hubs entirely during recovery.
3. **PnP cycle (admin shell required).** Disable + enable the broken interface from
   Device Manager / `Disable-PnpDevice` to force Windows to re-bind WinUSB.sys. This
   only helps if the driver bind is what broke; in our diagnosis the bind is healthy on
   both boards, so this is unlikely to recover the current state but is harmless.

### Tier 2 — On-bench reset (no flash)
4. **Press the Max Carrier RESET button.** Pulls the i.MX `nRST` net low; the SoC
   re-boots from eMMC. This is the cleanest non-destructive reset for a healthy image
   that's just hung.
5. **USB-C only re-plug.** Useful if the X8 is being powered from the 12 V barrel and
   the USB issue is only on the data link. Wait ≥5 s before re-plugging.
6. **Full power-cycle.** Unplug **both** USB-C and the 12 V barrel for ≥10 s, then
   re-power. This is the canonical recovery for any X8 bring-up stall (already
   documented in repo memory as the only reliable path after openocd/M7 contention).

> **2026-05-12 update — Tier 2 has been attempted multiple times and never
> recovers Board 1.** The hung state is reproducible across power cycles. This means
> something *during the Linux boot sequence itself* is wedging userland (corrupt
> systemd unit, exhausted /var, half-finished OTA, etc.). Repeating the same boot is
> wasted time. Skip directly to Tier 2.5 / Tier 3 below.

### Tier 2.5 — Carrier sanity check (DO THIS BEFORE Tier 3)
The 2026-05-12 diagnostic run found **zero** Max Carrier on-board J-Link CDC ports
(`VID_1366&PID_0105`) enumerated, even though the X8 itself enumerated. The OB BMP
runs on the carrier independent of the X8 module, so its absence is a strong signal
that **the carrier is not fully powered** or its USB host port is not the one
plugged into the host PC.

Before assuming Linux is hung, verify:
- The Max Carrier is on its **own** USB-C cable into the host (not just the X8's
  USB-C). The carrier's USB port is what brings up the BMP CDC ports.
- The 12 V barrel jack is plugged in **and** the carrier's main power LED is lit.
- After plugging in, `Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -match 'VID_1366&PID_0105' }`
  returns at least 2 CDC interfaces. If it doesn't, there's no point trying Tier 3
  on this carrier — the BMP is not booting either.

### Tier 3 — Interrupt boot at the mask-ROM with SDP + `uuu`

**This is the answer to "do we need to disrupt the Linux bootup before it gets
stuck?" — yes, and this tier does exactly that, one stage earlier than U-Boot.**

The i.MX 8M Mini's *Serial Download Protocol* lives in **mask ROM** — burned into
silicon at the foundry, untouchable by any image corruption. When the BOOT DIPs are
set to SDP at power-on, the SoC executes ROM code only and waits for a host
(`uuu`) to push it a payload over USB. **U-Boot, the Linux kernel, systemd, and
every userspace daemon are bypassed entirely.** The hung state cannot recur in
this mode because none of the hung software has been loaded.

`uuu` then offers two distinct recovery options:

- **3a. Full reflash (definitive fix).** Run `uuu full_image.uuu` from the
  `mfgtool-files-portenta-x8` bundle to overwrite eMMC with a known-good factory
  image. Use this when you want a clean slate and don't need anything from the
  current eMMC contents.
- **3b. RAM-only rescue boot (preserves data).** Run `uuu` with just the imx-boot
  + u-boot.itb + a small initramfs to bring the SoC up entirely from RAM, drop to
  a U-Boot or Linux shell, mount the eMMC read-only, capture logs (`/var/log/journal`,
  `/etc/systemd`), find the wedge, fix it, then reboot with DIPs OFF. Use this when
  you want forensic data on *why* Linux hung rather than nuking it.

#### Tier 3 procedure (3a — full reflash)

7. **Power off the X8 completely** (USB-C unplug AND 12 V barrel unplug).
8. **Set DIP switches on the Max Carrier:** both `BOOT SEL` and `BOOT` to the
   **ON** position (per Arduino docs *09. How To Update Your Portenta X8 → Update
   Using uuu*). This is the line that "disrupts the Linux bootup" — the SoC will
   not even attempt to load anything from eMMC.
9. **Cable:** USB-C ↔ USB-A from host to the X8's USB-C port (NOT the carrier's).
   Do not use USB-C↔USB-C.
10. **Install `uuu`** if not already present:
    `winget install --id NXP.uuu` (or download
    <https://github.com/nxp-imx/mfgtools/releases> → `uuu.exe`).
11. **Download the latest factory image** from
    <https://downloads.arduino.cc/portentax8image/image-latest.tar.gz>.
    Extract; then decompress `mfgtool-files-portenta-x8.tar.gz` and
    `lmp-factory-image-portenta-x8.wic.gz` so the directory contains:
    ```
    imx-boot-portenta-x8
    lmp-factory-image-portenta-x8.wic
    mfgtool-files-portenta-x8/
    sit-portenta-x8.bin
    u-boot-portenta-x8.itb
    ```
12. **Start `uuu`:** `cd mfgtool-files-portenta-x8`, then `.\uuu.exe .\full_image.uuu`.
    `uuu` will print `Waiting for Known USB Device Appear...`.
13. **Power the X8 on.** With the BOOT DIPs ON, the i.MX comes up in SDP mode and
    enumerates as a Freescale `0x1fc9` SDP device — `uuu` immediately starts
    pushing the bootloader, then the wic image. End-to-end takes ~3–5 minutes.
14. **After "Done" appears:** power off, set both BOOT DIPs back to **OFF**,
    power on. Wait ~10 s for the blue LED to start blinking — Linux is alive
    again. Verify with `adb devices`.
15. **Re-flash the L072 LoRa firmware** with the existing
    `run_stage1_standard_quant_end_to_end.ps1` pipeline. The L072 lives on the
    Murata module and is not touched by `uuu`, but the W4-pre era firmware on
    Board 1 (if any) is older than W1-9d and should be replaced with the current
    16,440 B build for Phase B testing.

#### How to confirm SDP is working before flashing

After step 13, before `uuu` begins pushing, you can verify in another shell:
```powershell
Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -match 'VID_1FC9&PID_(012B|0134)' }
```
The i.MX SDP PID varies by mask-ROM revision: `PID_012B` (older) or `PID_0134`
(newer; observed on Board 1 during the 2026-05-12 recovery). A present
`Status=OK` entry confirms the i.MX mask ROM is alive and accepting SDP
commands. If this never appears with the DIPs set correctly, the issue is
hardware (power rail, broken USB-C connector, dead i.MX) — not Linux at all.

#### Host `uuu` version note

The bundled `uuu 1.5.109` (from older MfgTool releases) was unstable on Win11
with the current X8 LMP image. Use **`uuu 1.5.243`** (or newer) — verified
working end-to-end (12/12 stages Okay) on Board 1.

#### Post-reflash GPIO preflight (REQUIRED before L072 reflash)

A freshly flashed LMP image leaves H7 NRST asserted: kernel `gpio10` boots
LOW, holding the H7 in reset. The very first `imx_gpio` OpenOCD attach used
by the L072 flash pipeline will fail with `Error connecting DP: cannot read
IDR`. The `run_stage1_standard_quant_end_to_end.ps1` launcher now exports
`gpio8/10/15` and drives `gpio10=1` before the cycle loop; any other launcher
that runs OpenOCD against a freshly reflashed X8 must do the same.

### Tier 3 — (legacy bullets, retained for cross-reference)
The i.MX 8M Mini has a *Serial Download Protocol* mode in mask ROM. With the carrier's
BOOT DIPs set, the SoC bypasses eMMC entirely and waits for `uuu` over USB. **This
recovers the board even if the entire eMMC root filesystem is corrupt.** It is the
final, always-works recovery path.

7. **Set DIP switches on the Max Carrier:** both `BOOT SEL` and `BOOT` to the **ON**
   position (per Arduino docs, `09. How To Update Your Portenta X8 → Update Using uuu`).
8. **Cable:** USB-C ↔ USB-A from host to X8 (do not use USB-C↔USB-C).
9. **Download the latest factory image** from
   <https://downloads.arduino.cc/portentax8image/image-latest.tar.gz>.
   Extract; then decompress `mfgtool-files-portenta-x8.tar.gz` and
   `lmp-factory-image-portenta-x8.wic.gz` so the directory contains:
   ```
   imx-boot-portenta-x8
   lmp-factory-image-portenta-x8.wic
   mfgtool-files-portenta-x8/
   sit-portenta-x8.bin
   u-boot-portenta-x8.itb
   ```
10. **Run `uuu full_image.uuu`** (Windows: `.\uuu.exe .\full_image.uuu`) inside the
    `mfgtool-files-portenta-x8/` directory. Power-cycle the X8 once when the tool is
    waiting; the i.MX SDP enumerates and `uuu` flashes through to completion.
11. **After flash succeeds, set both BOOT DIPs back to OFF**, power-cycle, wait ~10 s
    for the blue LED to start blinking — that is the "Linux booted" signal.
12. **Re-flash the L072 LoRa firmware** with the existing
    `run_stage1_standard_quant_end_to_end.ps1` pipeline once `adb devices` lists the
    board again. Stage 1 is required because uuu wipes the carrier-side stack but the
    L072 firmware lives on the Murata module which is *not* touched by uuu.

### Tier 4 — Hardware contact (only if Tier 3 fails to enumerate even in SDP)
- Inspect the Max Carrier DIPs visually (correct switches set ON?).
- Try a different USB-A port / different host PC.
- Inspect the X8 module for a stuck or shorted load on the 12 V rail.

---

## 4. What NOT to do
- Do **not** run `revive_bridge.sh` — repo memory entries from 2026-05-08 already
  document it as ineffective in practice. Auto-reboot via the H7 watchdog is what
  actually recovers a bridge stall, but that path requires a working Linux to start.
- Do **not** stop the watchdog daemon (`wdt_pet.sh stop`) hoping the X8 will
  self-reboot — that trick also requires a working `adbd` to *start* the daemon.
- Do **not** attempt SWD recovery via the on-board J-Link. It only reaches the H747
  M4 SW-DP (`0x5BA02477`); there is no external path to the M7 or to the i.MX SoC on
  the Max Carrier (memory entry 2026-05-07 architectural-block).

---

## 5. Helper script

`LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/diagnose_x8_recovery.ps1`

Run with the broken board's serial:

```powershell
cd LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper
./diagnose_x8_recovery.ps1 -AdbSerial 2D0A1209DABC240B
```

The script:
1. Runs `adb kill-server` + `adb start-server` and re-checks `adb devices`.
2. Enumerates Windows PnP for `VID_2341&PID_0061` (Arduino X8 ADB) and
   `VID_1366&PID_0105` (Max Carrier OB J-Link CDC) and reports which interfaces are
   present for the requested serial.
3. If a `USB Serial Device` appears for that serial, opens the COM port at 115 200 8N1,
   sends `\r\nuname -a\r\n`, and reports whether bytes come back.
4. Prints a one-line **TIER verdict** indicating which tier of the §3 hierarchy the
   user must escalate to.

The script is **read-only** with respect to the X8 — it never reboots, flashes, or
sends destructive commands. It exists so any future "board dropped out" event can be
triaged in one command rather than reconstructing the diagnosis by hand.

---

## 6. References
- Arduino docs · *09. How To Update Your Portenta X8* — Update Using uuu (DIP-switch
  positions, `full_image.uuu` workflow).
- Arduino docs · *Portenta Max Carrier* — pinout & DIP-switch silkscreen.
- Repo memory `/memories/repo/lifetrac-portenta-x8-lora.md` — entries 2026-05-07
  through 2026-05-12 covering bridge stall recovery and openocd/H7 architectural
  block.
- NXP application note AN12805 — *i.MX 8M Mini Serial Download Protocol*.

---

## 6. 2026-05-13 update — L072 ROM auto-baud trap (Stack C recovery path)

### 6.1 The discovery

While debugging the **swd_bypass_pa11_pf4_launcher.sh** flow on Board 2, I confirmed
the launcher *does* enter ROM bootloader mode but every probe returned `0x1F` NACK
(or no response at 8N1). Root cause: the STM32 ROM bootloader auto-bauds on the FIRST
byte after NRST release. A glitch byte during the NRST rising edge — most likely from
the `cs42l52` audio codec's repeated `cannot obtain reset-gpio` i2c retries (see
[2026-05-08_Method_G_Avoiding_Power_Cycles_Copilot_v2_0.md §1](2026-05-08_Method_G_Avoiding_Power_Cycles_Copilot_v2_0.md)) —
consumes the auto-baud slot. After that, the ROM is locked at 19200 8E1 and treats
the standard `0x7F` sync byte as an invalid command, returning `0x1F` NACK.

**Fix:** never re-send `0x7F`. Configure UART to 19200 8E1 and send commands directly.
Validated on Board 2:

| Command | TX | RX | Decoded |
|---|---|---|---|
| GET    | `00 FF` | `79 0b 31 00 01 02 11 21 31 44 63 73 82 92 79` | bootloader **v3.1**, 11 commands |
| GET_ID | `02 FD` | `79 01 04 47 79`                                | PID **0x0447** (L072) |
| READ_MEMORY @ 0x08000000 | `11 EE / addr+csum / FF 00` | 256 valid bytes | vector table SP=0x20005000, Reset=0x08000579 (Thumb-aligned ✓) |

### 6.2 New tooling

Added to `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/`:

- **flash_l072_via_uart.py** — minimal AN3155 client for LmP. Avoids `0x7F` entirely.
  Supports `probe`, `read [addr] [n]`, `erase`, `write <bin>`, `go [addr]`. Uses
  shell `stty` for UART config (LmP Python lacks termios/fcntl/ctypes/pyserial).
- **rom_clean_sync.sh** — re-asserts PWM4 HIGH (BOOT0), pulses PF4 (NRST), proves
  liveness without expecting an ACK from the autobaud byte.
- **rom_get_cmd.sh** / **rom_get_id.sh** — minimal shell-only confirmations.

### 6.3 New Tier 0.5 in the recovery hierarchy

Insert between Tier 0 (host-side already attempted) and Tier 1 (USB cable swaps):

> **Tier 0.5 — L072-only recovery via UART4 (no SWD, no DFU).**
> If the H7 / X8 stack is healthy but the L072 user firmware is broken (no AT
> response, stuck in OPMODE, etc.), the entire L072 can be re-flashed *over the
> existing UART4 link* via the bridge. Steps:
>
> 1. Drive PA11 HIGH and pulse PF4 NRST low/high using
>    `swd_bypass_pa11_pf4_launcher.sh` (writes via `pwmchip0/pwm4` and
>    `gpio-163`). PA11 must already be high *before* NRST releases.
> 2. Run `python3 flash_l072_via_uart.py probe`. Expect `bootloader v3.1` and
>    `PID=0x0447`. **Do not panic if the very first 0x7F probe NACKs** — that
>    is now expected; the Python flasher never sends 0x7F.
> 3. `read 0x08000000 256` to dump the vector table — confirms READ_MEMORY
>    works and shows what is currently flashed.
> 4. `write <fw>.bin` to ERASE + WRITE_MEMORY + verify the new image.
> 5. `go 0x08000000` to start it (or just pulse NRST again with PA11 LOW).

### 6.4 Healthcheck addition

`board1_healthcheck.sh` now ends with an **L072 GET probe** section that:

- Sends `GET (00 FF)` at 19200 8E1 to `/dev/ttymxc3` (read-only, never `0x7F`).
- Reports `silent` (chip running user firmware — the normal state),
  `ACK` (chip in ROM mode), or `NACK 0x1F` (in ROM but auto-baud consumed,
  use `flash_l072_via_uart.py`).

This is non-destructive in all cases — `GET` is a query command and does not
alter chip state.

### 6.5 What the discovery does NOT explain

The L072 user firmware itself is still uncooperative — it does not respond on
`/dev/ttymxc3` at 921600 8N1 (expected Stack C host-protocol baud). That means:

- The chip is healthy (ROM proves it).
- The wiring is healthy (READ_MEMORY full pages succeed).
- **The user firmware on flash is not initializing the host UART correctly**, OR
  **the W1-9 fallback path leaves the chip in a non-host-protocol state**.

Next step (separate work item): use the new `read 0x08000000 16440` capability
to dump the *currently flashed* image off Board 2 and bindiff it against
`build/firmware.bin` to find out what is actually on the chip vs. what we last
intentionally flashed.
