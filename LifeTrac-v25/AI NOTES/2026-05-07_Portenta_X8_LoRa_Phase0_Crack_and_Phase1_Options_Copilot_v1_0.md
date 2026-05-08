# Portenta X8 LoRa — Phase 0 Crack & Phase 1 Options

**Date:** 2026-05-07 (EOD #2 follow-up)
**Author:** Copilot (Claude Opus 4.7)
**Version:** v1.0
**Status:** Phase 0 ✅ COMPLETE — bootloader entry achieved. Phase 1 options open.
**Supersedes the working hypothesis in:** [2026-05-07_Portenta_X8_LoRa_Phase0_Status_EOD2_Copilot_v1_0.md](2026-05-07_Portenta_X8_LoRa_Phase0_Status_EOD2_Copilot_v1_0.md)

---

## 1. Executive Summary

After two days of pin-mapping dead ends (the `PG_7=BOOT0` / `PC_7=NRST` collaborator hypothesis was disproven both by exhaustive GPIO sweep and by reverse-engineering the upstream `x8h7` firmware `GPIO_pinmap[]`), the user supplied schematic-derived pin labels that, combined with the firmware pinmap, produced a definitive mapping. The mapping was tested by direct H747 register-poke from internal `openocd` and produced a **`0x79` STM32 ROM bootloader ACK on the very first probe attempt** at `/dev/ttymxc3` 19200 8E1.

**Authoritative pin mapping (Portenta X8 + Max Carrier, this build):**

| LoRa function | LoRa module pin | Schematic label | H747 net | x8h7 GPIO_pinmap[] index |
|---|---|---|---|---|
| **BOOT0** | 43 | J2-67 / PWM4 | **PA_11** | PWM[4] |
| **NRST**  | (different, near GPIO 3) | — | **PF_4** | GPIO[3] |
| IRQ / DIO | 22 | J2-69 / PWM5 | PD_15 | PWM[5] |
| UART RX (L072 ← H7/i.MX) | 23 | J2-25 / SERIAL3_TX | — | (wired to `/dev/ttymxc3` directly via i.MX, NOT bridged) |
| UART TX (L072 → H7/i.MX) | 24 | J2-27 / SERIAL3_RX | — | (same path) |

Earlier (incorrect) hypotheses for posterity:

| Iteration | Hypothesis | Disproof |
|---|---|---|
| 1 | `LORA_BOOT0 = PG_7` (collaborator) | GPIOG entirely absent from x8h7 GPIO_pinmap. Driving PG_7 HIGH at the H747 pad (IDR confirmed) caused no behavior change on L072. Exhaustive GPIOG NRST sweep (12 candidates) returned 0 ACKs. |
| 2 | `LORA_NRST = PC_7` | Reverse-engineered `gpio.c` shows PC_7 is PWM channel 0, not a reset line. |
| 3 | `BOOT0=PA_11, NRST=PD_15` (after first user schematic dump) | PA_11 IDR=1 confirmed, PD_15 LOW/HIGH pulse confirmed, but no ACK. User then clarified PD_15 is IRQ/DIO (an L072 *output*). Pulsing it from the H7 side did nothing useful. |
| 4 ✅ | `BOOT0=PA_11, NRST=PF_4` (GPIO3 from x8h7 pinmap) | **0x79 ACK on attempt #1.** |

---

## 2. The Crack — Reproducible Recipe

### 2.1 Test artifact

`LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/06_assert_pa11_pf4.cfg`

OpenOCD TCL that:

1. Halts M7 at SWD attach (lands in System Bootloader at `PC=0x1ff09abc`, RCC clean).
2. Enables `RCC_AHB4ENR` bits 0 (GPIOA) + 5 (GPIOF).
3. Configures `PA_11` as push-pull output, drives **HIGH** (BOOT0 latched).
4. Configures `PF_4` as push-pull output, drives **LOW for 250 ms**, then **HIGH** (NRST pulse — release while BOOT0 still high).
5. Re-asserts `PA_11=HIGH` every 1 s for 60 s to keep BOOT0 latched during the flash window.

### 2.2 Wrapper

`LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_pa11_pf4_test.sh`

- Launches `openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -f 06_assert_pa11_pf4.cfg` in background.
- Waits 5 s for openocd to assert pins.
- Probes `/dev/ttymxc3` at 19200 **8E1** with a single `\x7F` sync byte; reads up to 1 byte; checks for `0x79`.
- Repeats up to 4 times.

### 2.3 Observed result (2026-05-07)

```
Phase A: enable GPIOA + GPIOF clocks (bits 0 + 5)
  RCC_AHB4ENR before = 0x00000000
  RCC_AHB4ENR after  = 0x00000021
Phase B: drive PA_11 HIGH (LoRa BOOT0)
  GPIOA_IDR.PA_11 = 1   (expect 1)
Phase C: pulse PF_4 LOW(250ms) then HIGH (LoRa NRST)
  GPIOF_IDR.PF_4 (low)  = 0   (expect 0)
  GPIOF_IDR.PF_4 (high) = 1   (expect 1)
Phase D: re-assert PA_11 HIGH
  GPIOA_IDR.PA_11 (post-reset) = 1   (expect 1)

=== probing /dev/ttymxc3 8E1 for ROM ACK 0x79 ===
--- attempt 1 ---
  size=1 hex=79
  *** GOT 0x79 ACK — L072 IS IN STM32 ROM BOOTLOADER ***
```

### 2.4 What this proves

- L072 is alive, electrically reachable, and **its STM32 system bootloader is intact** (i.e., we have not bricked it; the stale `+ERR_RX` firmware is still on top, but the ROM under it works).
- The host-side I/O path from X8 Linux → `/dev/ttymxc3` → L072 UART1 is functional at 19200 8E1.
- We have a **deterministic** way to put the L072 into the STM32 ROM bootloader from pure software on X8 (no hardware mod, no soldering, no DIP switches, no carrier swap, no M4 sketch needed).

### 2.5 What this does NOT yet prove

- That we can complete a full AN3155 bootloader handshake (`Get`, `Get ID`, `Read Memory`, `Write Memory`, `Erase`, `Go`).
- That we can write a 257 KB image without timing out (60 s hold is enough only for sync; we'll extend to ≥ 600 s for a full flash).
- That the pin mapping is identical on every Max Carrier revision (we've validated one specific board: ABX00043 with serial `2E2C1209DABC240B`).

---

## 3. Phase 1 Options — How to Actually Flash the L072

We need an AN3155 client. `stm32flash` is the obvious choice but **is not installed on the X8** and there is no package manager (LmP/OE image, no `apt`/`opkg`/`dnf`). Available on the X8:

- `python3 3.10.4` ✅
- `docker` ✅
- `bash`, `dd`, `xxd`, `stty` ✅
- `gcc` / `make` ❌
- `pyserial` ❌

### Option A — Pure-Python AN3155 flasher  ★ RECOMMENDED

**Approach.** ~150 LoC implementing AN3155 directly over `os.read`/`os.write`/`termios` + `fcntl.ioctl` for parity flips. No third-party dependencies. Runs in-place on X8.

| Pros | Cons |
|---|---|
| Zero provisioning. We already have python3. | We own the protocol — bugs are on us. |
| Deterministic, single source of truth. | Slow at 19200 baud (~135 s for 257 KB). |
| Easy to add diagnostics (Get/GID/ReadOut for verify). | Needs careful parity handling (tcsetattr on each direction). |
| Trivial to integrate with the openocd hold (just call from a sibling shell). | |
| We can extend to do byte-by-byte verify via Read Memory. | |

**Effort:** modest — half a day to write + bench-validate.
**Risk to L072:** very low; AN3155 is a public, well-specified, ROM-resident protocol. Worst case: we leave the chip in bootloader and re-pulse NRST without BOOT0 → boots whatever is at 0x08000000.

### Option B — Docker container with stm32flash

**Approach.** Pull a small image (e.g., `alpine` or `debian:slim`) with `stm32flash` on the X8 and run with `--device=/dev/ttymxc3 --network host`.

| Pros | Cons |
|---|---|
| Battle-tested AN3155 implementation. | Requires internet from X8 + image pull (~50 MB). |
| One-shot CLI; trivial flash + verify + go. | Adds Docker layer to a security-sensitive path. |
| | Docker on i.MX8MM may be slow to start (~3–5 s container spin-up). |
| | Need to ensure container can see `/dev/ttymxc3` and that no Linux process holds it. |

**Effort:** small — most time is the image pull. Existing Alpine packages `stm32flash` in `community`.
**Risk:** container plumbing edge cases.

### Option C — Cross-compile stm32flash on Windows host, push static binary

**Approach.** Build `stm32flash` (single C file + Makefile, ~3 KLOC) with an aarch64-linux-musl toolchain, push the resulting static binary to `/var/rootdirs/home/fio/`.

| Pros | Cons |
|---|---|
| Native binary, no Python, no Docker. | We don't have an aarch64 toolchain on this Windows workstation today. |
| Familiar CLI. | Setup time (install zig / WSL + cross-toolchain) likely exceeds Option A's flasher implementation. |

**Effort:** medium (toolchain provisioning).
**Risk:** glibc/musl mismatches if not statically linked.

### Option D — Cross-compile stm32flash inside a Docker container on the X8 itself

**Approach.** Use a build-toolchain image (`debian:bookworm` + `build-essential`) on X8, mount source, output binary, copy out, drop image.

| Pros | Cons |
|---|---|
| Yields a native binary like Option C. | Larger image pull (~200+ MB). |
| One-shot; binary then lives on X8 forever. | Slowest end-to-end of all four options. |

**Effort:** medium-large.
**Risk:** disk space on X8.

### Recommendation

**Go with Option A.** It is the smallest, fastest, most observable path. We have everything we need on the X8 already, and writing AN3155 in Python keeps the entire flash pipeline transparent and step-debuggable. We can revisit Option B if Option A hits a parity-flip / blocking-IO snag.

If Option A produces a `Get` command success (chip ID = `0x447` for L072), we cleanly progress to:

1. Erase + Write Memory of `mlm32l07x01.bin` (extracted from `MKRWAN/examples/MKRWANFWUpdate_standalone/fw.h`).
2. Verify (Read Memory + compare).
3. `Go 0x08000000` to boot the new firmware.
4. Drop openocd hold; pulse NRST without BOOT0 (release `PF_4` to input, then pulse it).
5. Probe `printf 'AT+VER?\r\n' > /dev/ttymxc3` at 19200 **8N1** — expect a real version string instead of `+ERR_RX`.

---

## 4. Outstanding Questions Before Phase 1 Starts

1. **Hold-time extension.** `06_assert_pa11_pf4.cfg` currently holds for 60 s. For a full flash + verify at 19200 baud we need ≥ 600 s. Trivial change: `for {set i 0} {$i < 600} {incr i}`. Will produce as `07_assert_pa11_pf4_long.cfg`.
2. **Concurrent UART access.** Some Linux processes may hold `/dev/ttymxc3` open. We should `fuser /dev/ttymxc3` before starting and either kill the holder or accept the conflict. Worth adding to the wrapper.
3. **NRST release polarity.** After flash + Go, we want the L072 to run user firmware, not stay in bootloader. The cleanest sequence is: drop BOOT0 to LOW (or release to input pull-down), then pulse NRST LOW/HIGH. Need to confirm whether PA_11 has an external pull-down — if not, we must explicitly drive it LOW before NRST release.
4. **Recovery path.** If we corrupt the user firmware, the ROM bootloader is untouchable — we always have Phase 0 to re-enter and re-flash. So this is genuinely brick-resistant.
5. **Bridge stall.** Every openocd cycle stalls the x8h7 bridge (`/sys/kernel/x8h7_firmware/version` = "Connection timed out"). This does NOT block Phase 1 because the L072 path is *not* through the bridge. But it does mean we should plan a power cycle after Phase 1 success.

---

## 5. Decision Log Update

| Decision | Status |
|---|---|
| Method G commitment (custom firmware on L072) | Ratified 2026-05-04, unchanged. |
| Phase 0 ROM-bootloader entry path | **CLOSED** — `06_assert_pa11_pf4.cfg`. |
| Phase 1 flasher | **OPEN** — recommend Option A (pure-Python AN3155). |
| Carrier identity assumption (Max Carrier ABX00043) | Confirmed via successful PA_11/PF_4 reset on this hardware. |

---

## 6. Files Touched This Session

| File | Purpose |
|---|---|
| [06_assert_pa11_pf4.cfg](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/06_assert_pa11_pf4.cfg) | ✅ Working bootloader-entry openocd cfg. |
| [run_pa11_pf4_test.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_pa11_pf4_test.sh) | Wrapper that runs the cfg + probes for `0x79`. |
| [04_assert_pa11_pd15.cfg](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/04_assert_pa11_pd15.cfg) | Superseded (PD_15 is IRQ, not NRST). |
| [05_assert_pd15_pa11.cfg](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/05_assert_pd15_pa11.cfg) | Superseded (swap polarity test, never executed). |
| [run_pa11_pd15_test.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_pa11_pd15_test.sh) | Superseded. |
| [run_pd15_pa11_test.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_pd15_pa11_test.sh) | Superseded. |
| `/memories/repo/lifetrac-portenta-x8-lora.md` | Updated with the breakthrough mapping. |

---

## 7. Next Action

Pending user go-ahead on the flasher choice. If Option A is approved, the next deliverable is `flash_l072.py` plus an extended-hold cfg (`07_assert_pa11_pf4_long.cfg`) and an end-to-end wrapper that:

1. Extracts `mlm32l07x01.bin` from MKRWAN `fw.h` on the workstation.
2. Pushes `.bin` + `flash_l072.py` to X8.
3. Starts openocd with the long-hold cfg.
4. Runs `flash_l072.py /dev/ttymxc3 mlm32l07x01.bin` (Get → Erase → Write → Verify → Go).
5. Releases BOOT0, pulses NRST, probes `AT+VER?` at 8N1, prints the real version string.
