# Murata `CMWX1ZZABZ-078` UART Firmware Probe — Bench Results

**Date:** 2026-05-04
**Author:** GitHub Copilot (Claude Opus 4.7)
**Version:** v1.0
**Boards:** Portenta X8 Board2 (`2E2C1209DABC240B`) and Board1 (`2D0A1209DABC240B`) on Max Carrier
**Tools used:** [`tools/at_probe.sh`](../tools/at_probe.sh), [`tools/at_probe2.sh`](../tools/at_probe2.sh), [`tools/rx_listen.sh`](../tools/rx_listen.sh), [`tools/tx_burst.sh`](../tools/tx_burst.sh)

This note records what the on-modem firmware does when poked from Linux user-space, so the next bench session can decide whether to reflash the Murata or commit to an external SX1276.

---

## 1. Probe environment

- Modem reset line: `gpiochip5` line 3 → sysfs `gpio163`. Verified writable as root, drives modem reset (low/high cycle observed to clear modem state).
- Modem UART: `/dev/ttymxc3` (`crw-rw---- root:dialout`). The `fio` user is **not** in `dialout`, so all probes ran under `sudo` with password `fio` (Arduino X8 default).
- `adb root` is rejected: "adbd cannot run as root in production builds". Sudo with password is the only path to the modem.
- No `gpioset`/`pip3`/`python3-serial` installed. Probes are pure POSIX shell using `stty` + `cat` + `printf`.
- Kernel-side `dmesg` shows nothing about the LoRa modem; the only Murata mention is the BCM4343WA1 BT/Wi-Fi combo. The LoRa SiP is invisible to the OS — it's just a peripheral hanging off `ttymxc3`.

## 2. Baud / framing

| Baud / framing | Modem behaviour |
|---|---|
| **19200 8N1** | Clean ASCII responses |
| 19200 8N2 (`cstopb`) | Garbled — emits truncated `+ERR\r` per input |
| 57600 8N1 | Silent (only NULs back) |
| 115200 8N1 | Silent (only NULs back) |

**Conclusion:** Modem operates at **19200 8N1**, *not* 8N2 (despite some Arduino docs suggesting otherwise for stock MKRWAN firmware). All further probing used 19200 8N1.

## 3. Command-response identity

Every line the host sends terminated with `\r` or `\r\n` produces **exactly one response**:

```
Error when receiving\n+ERR_RX\r        (29 bytes)
```

Tested commands (all gave the same response):

| Family | Examples | Response |
|---|---|---|
| Hayes | `AT`, `AT?`, `ATI` | `+ERR_RX` |
| MKRWAN AT | `AT+VER?`, `AT+DEV?`, `AT+APPEUI?`, `AT+CGMR` | `+ERR_RX` |
| Hayes escape | `+++` then `AT` | `+ERR_RX` |
| RAK / RUI3 | `AT+RFCFG=...`, `AT+SEND=hello`, `AT+P2P_VER` | mostly `+ERR_RX`, one `+ERR` for the long form |
| Semtech AT_Slave | `AT+UTX=5`, `AT+RX=1` | `+ERR_RX` |
| Special | single `\0` byte, `BREAK` line condition | no response (input swallowed) |

**With no input, the modem is silent (zero bytes captured during a 12 s passive listen).** The earlier `+ERR_RX` storm seen in [`at_probe.sh`](../tools/at_probe.sh) was 1 response per command, not autonomous chatter.

## 4. Cross-board over-air test

- Board1 listener: `cat /dev/ttymxc3` for 12 s (after modem reset) — **0 bytes received.**
- Board2 transmitter: 5×`printf 'CROSSBOARD_TEST_n\r\n' > /dev/ttymxc3` over 2.5 s.

**Result:** Board1 captured nothing. Either the firmware does not actually transmit over the air on UART input, or it transmits but the receiving end's firmware does not surface received frames on UART. Either way, **no bidirectional LoRa link can be established with the firmware that is currently flashed.**

## 5. Identification attempt

The literal string `"Error when receiving"` paired with the `+ERR_RX` URC code does **not** appear in any source under the local Arduino tree (`C:\Users\dorkm\Documents\Arduino`, `C:\Users\dorkm\AppData\Local\Arduino15`). It is **not** the stock MKRWAN/`mkrwan1300-fw` firmware (which uses `+OK`/`+ERR=N`), not RadioLib (which doesn't speak AT), and not the standard Semtech AT_Slave reference (which uses `AT_OK`/`AT_ERROR=N`).

Most likely candidates (in order of probability):

1. **A custom or vendor-customised diagnostic firmware** flashed at OSE/Arduino factory or by a previous experimenter, whose only visible behaviour is to print this fixed error string.
2. **A partially-initialised firmware** stuck in a reset/error loop, where the LoRa stack failed to start and the only thing reaching the UART is an error handler.
3. **A locked-down Murata firmware** (e.g. an LoRaWAN-Edge variant) that requires a session-key handshake before responding to AT.

The differentiation is academic — in all three cases the path forward is the same.

## 6. Hard conclusions for the v25 design

1. **The Murata UART is alive and bidirectional from Linux.** Method D (raw-P2P AT over UART) remains *hardware-feasible*; the obstruction is purely the firmware currently flashed.
2. **The Murata's currently-flashed firmware is unusable for v25.** It does not expose `AT+VER?`, `AT+SEND`, `AT+P2P_*`, or any obvious command; it does not surface received frames; and it does not propagate `\0` or `BREAK` for bootloader entry.
3. **No autonomous chatter** means there is no diagnostic / heartbeat we can read to identify the firmware further without invasive action.
4. **Reset works** — `gpio163` properly cycles the modem; this is the handle we'll need for any future reflash sequence.
5. **Baud is 19200 8N1**, definitively. `LORA_PROTOCOL.md` and any future AT driver should hardcode this and stop testing 8N2 / 115200.

## 7. Decision-tree update for [Method D](2026-05-04_LoRa_AT_vs_SPI_Method_Comparison_Copilot_v1_0.md)

The earlier comparison note made Method D conditional on *"identifying / sourcing a P2P-capable AT firmware for the Murata."* This bench session **rules out** the optimistic case where the stock firmware just needs the right command syntax — but a follow-up reading of Arduino's official documentation (see §7a below) **also rules out** the pessimistic case that SWD access is required.

### 7a. Arduino-supported in-system reflash (no SWD needed)

Per [Arduino docs (Connecting Max Carrier to TTN, §2.1)](https://docs.arduino.cc/tutorials/portenta-max-carrier/connecting-to-ttn) and the [official LoRa modem firmware update guide](https://support.arduino.cc/hc/en-us/articles/4405107258130-How-to-update-the-LoRa-modem-firmware), the Murata `CMWX1ZZABZ-078` on the Max Carrier is **field-flashable from the host MCU** using the `MKRWANFWUpdate_standalone` example sketch from the `MKRWAN` Arduino library, by adding `#define PORTENTA_CARRIER` before `#include <MKRWAN.h>`. The sketch:

1. Toggles the BOOT0 / RESET lines on the Murata's internal STM32L072 to enter its built-in **STM32 system bootloader** over UART. (Those control lines must be wired from the H7 to the SiP through the Max Carrier — the existence of this supported workflow proves it.)
2. Pushes a packed firmware blob (LoRaWAN AT) over `Serial3` at the bootloader's baud.
3. Reboots the modem and verifies via `AT+VER?`.

So our "firmware unusable" finding is exactly the symptom Arduino warns about: *"the module arrives with an earlier version of the Murata proprietary stack and must be updated before first use."* The current `Error when receiving / +ERR_RX` URC is consistent with a stale or mis-region-configured stock firmware.

**Implications:**

- **No SWD needed.** The earlier conclusion that "Method D = multi-day SWD work with brick risk" is wrong; the Arduino-blessed path is a one-click flash from the host MCU.
- **The first reflash will produce a LoRaWAN-AT firmware** (Method B/C territory), not raw P2P. To get raw P2P (Method D), we'd flash a different blob using the **same bootloader entry sequence** — e.g. the Semtech `I-CUBE-LRWAN AT_Slave` binary built for STM32L072 — but that requires building/sourcing a compatible image.
- Step 1 (LoRaWAN-AT reflash) is **safe and fast** because it brings the modem to a known-good state we can probe from `Serial3` / `/dev/ttymxc3` and from which we can then evaluate Method B/C empirically. Step 2 (custom P2P binary) becomes a follow-on choice.
- The X8 boards run the H7 M7 core too; the same sketch should compile under `arduino:mbed_portenta:envie_m7` via `arduino-cli`. The X8 Linux side stays out of the flash path entirely.

### 7b. Revised cost picture

| Method | Pre-Arduino-doc estimate | Post-Arduino-doc estimate |
|---|---|---|
| D (raw-P2P AT) — Step 1: bring up LoRaWAN-AT first | multi-day, brick risk | **~1 hour** (one sketch upload) |
| D — Step 2: source/build P2P binary | included above | days, but now optional / lazy |
| E (external SX1276) | ~1 week HW + driver | unchanged |
| F (two-radio split) | ~1.5 weeks | unchanged |

The rational sequencing is now: **flash the LoRaWAN-AT firmware first, then re-evaluate.** If the round-tripped LoRaWAN AT response times look survivable for our latency goals (they probably won't for control, but should for telemetry), we can commit to Method F where the LoRaWAN-AT-Murata handles telemetry/image and an external SX1276 handles control. If we find a P2P binary, full Method D is back on the table.

## 8. Next bench actions (revised)

1. **Install** Arduino IDE + `MKRWAN` (v1) library on the dev workstation; or set up `arduino-cli` with the `arduino:mbed_portenta` core.
2. **Build** `MKRWANFWUpdate_standalone.ino` with `#define PORTENTA_CARRIER` for `arduino:mbed_portenta:envie_m7`.
3. **Flash and run** on Board2 first (slate of choice for risk). Capture serial-monitor output of the update progress; archive any printed pre-update version string.
4. **Re-run** [`tools/at_probe.sh`](../tools/at_probe.sh) and [`tools/at_probe2.sh`](../tools/at_probe2.sh) afterward. Expected post-flash response: `AT+VER?` returns a real version string and `+OK`. Document the actual version.
5. **Repeat** on Board1 only after Board2 is confirmed alive.
6. **If the post-flash modem speaks MKRWAN AT correctly:** measure round-trip AT latency at 19200 8N1 and 115200 8N1 (if the new firmware exposes a baud-change command), and feed those numbers back into the latency-budget table in [LORA_AT_vs_SPI_Method_Comparison](2026-05-04_LoRa_AT_vs_SPI_Method_Comparison_Copilot_v1_0.md).
7. **If the post-flash modem still misbehaves:** then the failure isn't stale firmware and we should fall back to the SWD path or commit to Method E.

*Side task (still useful):* a UART-passthrough sketch on `tractor_h7` to expose `Serial3` ↔ USB CDC for quick laptop probing during bench sessions.

## 9. Files touched / produced this session

- New: [`LifeTrac-v25/tools/at_probe.sh`](../tools/at_probe.sh), [`LifeTrac-v25/tools/at_probe2.sh`](../tools/at_probe2.sh), [`LifeTrac-v25/tools/rx_listen.sh`](../tools/rx_listen.sh), [`LifeTrac-v25/tools/tx_burst.sh`](../tools/tx_burst.sh) — keep as bench utilities.
- Existing [`LifeTrac-v25/tools/at_probe.py`](../tools/at_probe.py) is now redundant on Yocto X8 (no `pyserial`); leave for future hosts that have it.

## 10. One-line summary

> **Hardware path to the Murata is good (UART live, reset works, bidirectional confirmed at 19200 8N1). The currently-flashed firmware matches Arduino's documented "stale stock firmware" failure mode and can be reflashed in-system from the host MCU using the `MKRWANFWUpdate_standalone` sketch with `#define PORTENTA_CARRIER` — no SWD required. Next bench step is to run that update on Board2.**
