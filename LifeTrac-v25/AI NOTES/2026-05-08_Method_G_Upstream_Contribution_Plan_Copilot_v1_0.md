# Method G — Upstream Contribution Plan (Portenta X8 LoRa custom-firmware path)

**Date:** 2026-05-08
**Author:** GitHub Copilot (Claude Opus 4.7)
**Version:** 1.0
**Status:** PLANNING — no upstream PRs opened yet. This doc proposes the work; review/edit before action.
**Audience:** LifeTrac maintainers and prospective Arduino-side reviewers.

## TL;DR

LifeTrac has now end-to-end proven a path to flash arbitrary user firmware to the Murata CMWX1ZZABZ-078 (STM32L072) on the Arduino Portenta X8 + Max Carrier, using only what already ships on the X8 (`openocd 0.11.0-dirty` with the `imx_gpio` driver, `python3`, `bash`). See:

- [2026-05-08_Method_G_Phase1_End_to_End_Flash_Success_Copilot_v1_0.md](2026-05-08_Method_G_Phase1_End_to_End_Flash_Success_Copilot_v1_0.md)
- [2026-05-08_Method_G_HelloWorld_Validation_Copilot_v1_0.md](2026-05-08_Method_G_HelloWorld_Validation_Copilot_v1_0.md)

This is a capability that **does not exist for any other Portenta X8 user today** (or at least is not documented anywhere we've found). The pieces we built are general — they have nothing LifeTrac-specific in them apart from filesystem paths. Arduino's user community would benefit if we upstream the work in a small series of focused contributions.

This doc proposes a 4–5 PR sequence across three Arduino-owned repos, with explicit scope, files, tests, and risks for each.

## Why upstream this

**Today's pain (from Arduino forum threads and our own experience):**

- The Portenta X8 ships with `mlm32l07x01.bin` (Murata stock AT modem) burned into the L072 by Arduino at the factory. There is **no documented user-facing way** to update or replace it from the X8 (Linux) side.
- Existing flashers (`stm32flash`, ST's CubeProgrammer) assume direct UART access to the L072 with manual BOOT0/NRST control. On Portenta X8, those pins are owned by the H7 (`PA_11`/`PF_4`) and not exposed to Linux through any documented API.
- The Arduino IDE's Portenta workflow targets the H7/M4 pair only; the L072 is treated as an opaque modem.

**What our work provides:**

1. A reproducible recipe for taking the H7 momentarily out of `x8h7-firmware` mode, asserting BOOT0+NRST via raw register pokes through openocd, talking AN3155 over `/dev/ttymxc3`, then dropping BOOT0 to boot user firmware.
2. A stdlib-only Python implementation of the AN3155 protocol with a per-page erase fallback that works around L072 mass-erase NACKs (which we observed even after Write Unprotect).
3. A "hold H7 halted with `PA_11`=LOW" pattern that lets user firmware actually run without reverting to the ROM bootloader.

## Target repositories and proposed PRs

| # | Repo | PR title | Scope |
|---|------|----------|-------|
| 1 | `arduino/portentax8-stm32h7-fw` | Add LoRa BOOT0 / NRST control to x8h7 bridge sysfs | Replace openocd hack with a clean kernel-API path |
| 2 | `arduino/portentax8-stm32h7-fw` | Boot policy: drive `PA_11` LOW at H7 startup | Make user-app boot the default; bootloader requires opt-in |
| 3 | `arduino/lmp-manifest` (Portenta X8 image) | Ship `lora-fw-update` userland tool + `python3` AN3155 lib | Make `lora-fw-update <path-to-bin>` a one-liner on the X8 |
| 4 | `arduino-libraries/MKRWAN` | Document Murata-flashed L072 mass-erase NACK + per-page fallback | Help anyone else writing an L072 flasher hit the same wall |
| 5 | Arduino docs site (PR to `arduino/docs-content` or successor) | Tutorial: "Flashing custom firmware to the Portenta Max Carrier LoRa modem" | Make the capability discoverable |

PRs 1 + 2 are the load-bearing ones. PR 3 is a packaging convenience that depends on the openocd fallback path remaining available until 1 + 2 land. PRs 4 + 5 are documentation.

A reasonable contribution order: **3 → 4 → 5 → 1 → 2**. PR 3 makes the openocd path official without changing H7 firmware (low blast radius); 4–5 are docs (no code risk); 1–2 are the H7-firmware changes that need actual review attention from Arduino's embedded team.

---

## PR 1 — `portentax8-stm32h7-fw`: expose LoRa BOOT0 + NRST via x8h7 bridge

**Repo:** https://github.com/arduino/portentax8-stm32h7-fw
**Branch:** `feature/lora-boot-control`

**Motivation:** Today, controlling `PA_11`/`PF_4` from Linux requires hijacking the H7 via openocd System Bootloader. That's hostile to the running x8h7 bridge (it stalls — see "Known side-effect" in the Phase 0 notes). A first-class kernel-side API removes the hostility.

**Proposed shape:**

- Add two new x8h7 bridge protocol opcodes:
  - `X8H7_LORA_BOOT0` (1-byte payload: 0=LOW, 1=HIGH) — drives `PA_11`.
  - `X8H7_LORA_NRST_PULSE` (1-byte payload: pulse width in ms, e.g. 250) — drives `PF_4` LOW for N ms then HIGH.
- Expose via existing `x8h7_firmware` sysfs interface; recommended path:
  - `/sys/kernel/x8h7_firmware/lora/boot0` (echo 0/1)
  - `/sys/kernel/x8h7_firmware/lora/nrst_pulse_ms` (echo 250)

**Files touched (best estimate, to be confirmed by reading the repo):**
- `firmware/x8h7_lora.c` (new) — opcode handlers driving GPIOA/GPIOF.
- `firmware/x8h7_proto.h` — opcode IDs.
- Linux side: `linux-x8h7/x8h7_firmware.c` — sysfs entries.

**Tests:** Bench script that drives boot0=1, sends 0x7F at 8E1, expects 0x79 ACK. Then boot0=0, nrst pulse 250, sends an AT command at 8N1, expects "OK".

**Risks:**
- Requires bumping x8h7 protocol version. Backward-compat: old bridge ignores the new opcode but should NACK gracefully. Need to confirm.
- Arduino may not want to expose raw modem pins via sysfs; may prefer a dedicated `lora-update` daemon. We should ASK in an issue first.

---

## PR 2 — `portentax8-stm32h7-fw`: drive `PA_11` LOW at H7 startup

**Repo:** same as PR 1.
**Branch:** `fix/lora-boot0-default-low`

**Motivation:** Our hello-world testing showed that `PA_11` (BOOT0) cannot be left floating. If left floating, an external pull-up (or H7 ROM behavior) eventually drives it HIGH, and the L072 falls back to ROM bootloader on next NRST. Today the x8h7 bridge does not actively drive `PA_11`; it only does so when we hijack it via openocd.

**Proposed change:** In the x8h7 bridge init code, after GPIOA clock enable, set `PA_11` to push-pull output and drive it LOW. Make this the default. Only change to HIGH on explicit `X8H7_LORA_BOOT0=1` from Linux (PR 1).

**Files touched:**
- `firmware/board_init.c` (or equivalent) — add `HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)` after MX_GPIO_Init.

**Tests:** Power-cycle X8. Without any openocd hijack, send AT to `/dev/ttymxc3` at 19200 8N1; expect "OK" from MKRWAN modem. Today this works only after manual openocd `08_boot_user_app.cfg`. After this PR it should work cold.

**Risks:**
- If Arduino's factory test fixture relies on `PA_11`=HIGH at boot to put L072 in bootloader during board test, this would break their factory flow. Need to confirm in issue discussion. (Mitigation: gate on a build flag, default LOW.)

---

## PR 3 — `lmp-manifest` (Portenta X8 image): ship `lora-fw-update` tool

**Repo:** https://github.com/arduino/lmp-manifest (or the Portenta X8 image recipe layer therein)
**Branch:** `feature/lora-fw-update-tool`

**Motivation:** Even with PRs 1–2 merged, users still need an AN3155 flasher. We have one (Python, stdlib-only, ~12 KB). Ship it as `/usr/bin/lora-fw-update`.

**Proposed shape:**
- Add a Yocto recipe `recipes-arduino/lora-fw-update/lora-fw-update.bb`.
- Source: a generalized version of [LifeTrac-v25/tools/stm32_an3155_flasher.py](../tools/stm32_an3155_flasher.py).
- Wrapper script `/usr/bin/lora-fw-update` that:
  1. Calls the new sysfs API (PR 1) to assert BOOT0=HIGH + NRST pulse.
  2. Configures `/dev/ttymxc3` to 19200 8E1.
  3. Runs the AN3155 flasher with `--verify`.
  4. Calls sysfs to drop BOOT0=LOW + NRST pulse.

**Fallback path (today, before PRs 1–2 land):** the wrapper script can detect the absence of the sysfs API and fall back to invoking openocd with the configs we ship (`06`, `09`). This makes PR 3 useful immediately.

**Files added:**
- `recipes-arduino/lora-fw-update/lora-fw-update.bb`
- `recipes-arduino/lora-fw-update/files/lora-fw-update`
- `recipes-arduino/lora-fw-update/files/an3155_flasher.py`
- `recipes-arduino/lora-fw-update/files/openocd-cfg/{06,07,08,09}_*.cfg`

**Tests:** `lora-fw-update mlm32l07x01.bin` should produce verify-OK in <5 s. Also test with arbitrary user binary (e.g. our `hello.bin`).

**Risks:**
- Image size impact: ~30 KB total. Negligible.
- License: our flasher is freshly-written by us for LifeTrac. Need to assign an Apache-2.0 or MIT header to make it acceptable upstream.

---

## PR 4 — `MKRWAN`: document mass-erase NACK quirk

**Repo:** https://github.com/arduino-libraries/MKRWAN
**Branch:** `docs/mass-erase-nack-l072`

**Motivation:** MKRWAN's reference flasher in `stm32.cpp` sets `F_NO_ME=0` (mass-erase enabled) for L072. We observed empirically that mass erase NACKs on Murata-flashed L072s **even after Write Unprotect**. Per-page Extended Erase works.

**Proposed change:**
- Add a paragraph to `extras/firmware/README.md` documenting the symptom and the per-page fallback.
- Optional code change: change `F_NO_ME` to `1` and switch to per-page erase by default (more reliable, marginally slower — 1.7 s vs aspirational <1 s).

**Files touched:**
- `extras/firmware/README.md` (docs)
- `extras/firmware/stm32.cpp` (optional code path)

**Risks:** Code change might affect other boards using the same flasher logic (MKRWAN1300/1310 are SAMD-based and use this for the Murata L0; they should behave identically but worth a regression run on MKRWAN hardware).

---

## PR 5 — Arduino docs: tutorial for custom L072 firmware on Portenta X8

**Repo:** wherever Arduino's docs live currently (was `arduino/docs-content`; verify before opening PR)
**Branch:** `tutorial/portenta-x8-lora-custom-fw`

**Motivation:** The capability is invisible to users unless documented. A SparkFun-tutorial-style walkthrough (similar to what we adopted in [2026-04-26_SparkFun_Tutorial_Style_Guide.md](2026-04-26_SparkFun_Tutorial_Style_Guide.md)) makes adoption frictionless.

**Outline:**
1. **Why you might want this** — replace AT modem with bare-metal LoRaWAN MAC, use raw SX1276 radio, etc.
2. **What you need** — Portenta X8 + Max Carrier, ARM toolchain (or use Arduino's bundled gcc), an L072 binary.
3. **Quick path** (post-PRs 1–3): `lora-fw-update mybin.bin`. Done.
4. **Manual path** (today): walkthrough of the openocd + AN3155 sequence using the cfgs and python flasher we ship via PR 3.
5. **Restoring the stock modem image**: keep a copy of `mlm32l07x01.bin` from `~/.arduino15/packages/arduino/hardware/mbed_portenta/.../firmwares/`; reflash with the same command.
6. **Pitfalls** — the BOOT0-must-be-held-LOW trap (our `09_boot_user_app_hold.cfg` lesson).
7. **Reference** — link to MKRWAN, AN3155, RM0367 (L0 reference manual).

**Tests:** A second engineer who has not used Method G should reproduce the tutorial end-to-end on a fresh Portenta X8 and submit edits where the steps are unclear.

**Risks:** None (docs only).

---

## Cross-cutting concerns

### License & attribution
- Our hello-world (`murata_l072_hello/main.c`, `stm32l072.ld`, `build.ps1`) is freshly-written by Copilot for LifeTrac. We can relicense to Apache-2.0 or BSD-3 for upstream — confirm with project lead.
- Our AN3155 flasher (`tools/stm32_an3155_flasher.py`) is also freshly-written. Same.
- The openocd cfgs (`06`/`07`/`08`/`09`) are similarly fresh.
- We did NOT copy code from MKRWAN's `stm32.cpp` or ST's CubeProgrammer; we re-implemented from AN3155 spec only. Confirm before upstream.

### Backward compatibility
- PR 1 adds new opcodes; old bridge versions reject them — Linux side must check version and fall back to the openocd path. PR 3's wrapper already supports both.
- PR 2 changes default boot pin state. Document loudly in release notes; provide a build flag to revert.

### Security implications
- A new sysfs API to pulse NRST + control BOOT0 is, by definition, an unrestricted-rewrite path to the L072 from any process with write access to those sysfs nodes. Default permissions should be `root:dialout 0660` (matching `/dev/ttyACM*` convention). Document this in the README of PR 1.
- The L072 has no secure boot in our product; firmware on the L072 can be replaced at will. This is consistent with the rest of the Arduino ecosystem — not a regression.

### Getting Arduino review attention
- Open a single discussion issue on `arduino/portentax8-stm32h7-fw` first ("Proposal: first-class L072 firmware update path from Linux"), summarizing this plan and linking back to LifeTrac's notes.
- Tag maintainers identified from recent PR history (avoid naming individuals here; check at PR-open time).
- Be ready to chunk further: each of PRs 1/2 is reviewable in isolation but should reference a shared design discussion.

## Open questions to resolve before opening PRs

1. **Has Arduino published any internal/private path for L072 updates?** Search forum + GitHub for "MKRWAN factory firmware". If yes, our PRs may be redundant or need to integrate with that path.
2. **Is the `x8h7-firmware` bridge protocol versioned and extensible?** Read `linux-x8h7` repo before drafting PR 1.
3. **Does the H7 factory firmware actively manage `PA_11` for any other purpose?** PR 2 must not break it. Read `board_init.c` and grep for `GPIO_PIN_11` on GPIOA.
4. **Mass-erase NACK reproduction**: confirm on a second Portenta X8 unit (we've only seen our one bench unit). If it's only ours, PR 4 should be downgraded to a comment in `MKRWAN/extras/firmware/README.md`.

## Recommended next concrete step

Open issue: `arduino/portentax8-stm32h7-fw#NEW` titled
**"Proposal: first-class L072 firmware update path from Linux on Portenta X8 + Max Carrier"**.
Body should be a 1-page summary linking to the two LifeTrac AI NOTES docs (Phase 1 + Hello-World Validation) and this plan. Wait for maintainer response (~1 week) before drafting any code PR.
