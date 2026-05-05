# 04 — Hardware Interface & Recovery

**Date:** 2026-05-04
**Status:** Spec — defines the public contract between the H7 host and the L072 firmware, plus brick-recovery design.
**Author:** GitHub Copilot (Claude Opus 4.7)

---

## 1. Pin / signal map

These are the L072 pins exposed off the Murata SiP and wired through to the Portenta Max Carrier and the Portenta H7 / X8.

| Function | L072 pin | Carrier net | H7 / X8 endpoint | Notes |
|---|---|---|---|---|
| Host UART RX (L072 ← H7) | PA3 (USART2_RX) | `LORA_RX` | `Serial3 TX` on H7 / `/dev/ttymxc3` TX on X8 | proven 19200 8N1 today; we will run 921600 8N1 |
| Host UART TX (L072 → H7) | PA2 (USART2_TX) | `LORA_TX` | `Serial3 RX` / `/dev/ttymxc3` RX | same |
| NRST | NRST | `LORA_NRST` | H7 GPIO / X8 `/dev/gpiochip5` line 3 (`gpio163`) | proven on bench |
| BOOT0 | BOOT0 | `LORA_BOOT0` | H7 GPIO (TBD — confirm in Phase 0/1 against `MKRWANFWUpdate_standalone` source) | drives the L072 into ROM bootloader |
| (Optional) Wake/IRQ | PB? | `LORA_IRQ` | H7 GPIO | for N-31 wake-on-LoRa; confirm whether routed |
| (Optional) BOOT DIP | — | SW1 on Max Carrier | physical switch | must be **OFF** for normal app boot, **ON** for ROM-bootloader entry without H7 BOOT0 control. We rely on H7-driven BOOT0 in production. |
| Internal SX1276 SPI | PA5/PA6/PA7 (SPI1) | (in-package) | not exposed | the bus we wanted; only L072 firmware can reach it |
| Internal SX1276 DIO0..3 | PB4/PB1/PB0/PC13 | (in-package) | not exposed | radio interrupts |
| Internal RF switch / PA control | PA1/PC1/PC2 | (in-package) | not exposed | per Murata datasheet |

The four lines that matter to the H7-side software are: **RX, TX, NRST, BOOT0**. Everything else is internal to the SiP.

## 2. Host UART transport

### 2.1 Link parameters

- **Baud:** 921600 8N1 (target). Phase 1 may bring up at 115200 first; production is 921600.
- **Flow control:** none. The DMA-on-IDLE pattern + length-prefixed frames remove the need for HW flow control.
- **Direction:** full-duplex. Host can pipeline TX commands without waiting for responses.

### 2.2 Frame format

Every frame on the wire, in either direction:

```
+------+--------------------------------+------+
| 0x00 |  COBS-encoded inner frame       | 0x00 |
+------+--------------------------------+------+

Inner frame (before COBS):
+------+--------+------+----------+-----------------+--------+
| ver  |  type  | flag |   seq    |   payload_len   |  ...   |
| 1 B  |  1 B   | 1 B  |   2 B    |     2 B         |  N B   |
+------+--------+------+----------+-----------------+--------+
+----------+
|   CRC16  |
|   2 B    |
+----------+

ver         = 0x01 for the launch protocol
type        = command/URC enum (table below)
flag        = bitmap (priority, ack-requested, encrypted-already, ...)
seq         = host-assigned sequence number; URCs echo the seq of the request that produced them, or 0 for unsolicited URCs
payload_len = little-endian u16
payload     = type-specific bytes
CRC16       = CCITT-FALSE polynomial 0x1021, init 0xFFFF, over [ver..payload]
```

Properties:
- COBS framing means there are **no escape characters in the payload** — every payload byte is opaque, including `0x00`.
- Length-prefixed → the L072 knows exactly how many bytes to expect, so DMA-on-IDLE is sufficient and no parser state machine is needed in the receive path.
- Per-frame CRC16 catches single-bit UART errors that would otherwise corrupt a frame.

### 2.3 Initial command/URC table

| `type` | Direction | Name | Purpose |
|---|---|---|---|
| `0x00` | both | `PING` | liveness probe; URC echoes the seq |
| `0x01` | H→L | `VER_REQ` | request firmware identity and version |
| `0x81` | L→H | `VER_URC` | reply with product name + firmware version + git SHA + capability bitmap |
| `0x02` | H→L | `UID_REQ` | request L072 96-bit unique device ID |
| `0x82` | L→H | `UID_URC` | reply with UID |
| `0x03` | H→L | `RESET` | soft reset request (firmware does an orderly NVIC_SystemReset) |
| `0x10` | H→L | `TX_FRAME` | transmit a v25 frame; payload is the on-air bytes (already encrypted if H7 owns crypto) plus a small header (channel hint, SF hint, profile hint, priority) |
| `0x90` | L→H | `TX_DONE_URC` | TX completed; payload includes timing metadata |
| `0x91` | L→H | `RX_FRAME_URC` | a frame was received; payload includes radio metadata + bytes |
| `0x20` | H→L | `CFG_SET` | set a runtime config value (e.g. fixed-channel mode for testing) |
| `0xA0` | L→H | `CFG_OK_URC` | config applied |
| `0x30` | H→L | `REG_READ` | read SX1276 register(s) — diagnostic only |
| `0xB0` | L→H | `REG_DATA_URC` | register read result |
| `0x31` | H→L | `REG_WRITE` | write SX1276 register — diagnostic, gated by build flag in production |
| `0x40` | H→L | `STATS_RESET` | clear counters |
| `0x41` | H→L | `STATS_DUMP` | request counter dump |
| `0xC1` | L→H | `STATS_URC` | counter dump payload |
| `0xF0` | L→H | `BOOT_URC` | sent once when the L072 finishes boot; carries safe-mode-was-considered flag, golden-jump-was-considered flag |
| `0xF1` | L→H | `FAULT_URC` | unsolicited fault notification |
| `0xFE` | L→H | `ERR_PROTO` | host sent something we don't understand; carries the offending type and ver |
| `0xFF` | both | `RESERVED` | never sent |

This table grows over the bring-up roadmap; every new command is documented here at the same time it lands in firmware.

### 2.4 Safe-mode magic

Within the **first 500 ms after boot**, if the L072 sees the byte sequence

```
0xA5 0x5A 0xA5 0x5A 'S' 'A' 'F' 'E'
```

on USART2 (uninterpreted by COBS — pre-protocol), it jumps to the STM32 ROM bootloader at `0x1FF00000` after deinit. This is the brick-recovery escape hatch that does not require BOOT0/NRST cycling. The H7 always knows how to send this sequence, and so does any laptop with a USB-UART adapter.

## 3. Brick-recovery design (defence in depth)

Five independent layers. Any one of them is enough to recover a wedged L072.

1. **Layer 1 — H7-driven NRST.** Always available. Recovers from a runaway main loop.
2. **Layer 2 — UART safe-mode magic** (§2.4). Recovers from a wedged main loop that's still servicing UART interrupts. Does not require BOOT0 control.
3. **Layer 3 — H7-driven BOOT0 + NRST.** Forces ROM bootloader on next boot, regardless of what the L072 firmware does or doesn't do. Recovers from a binary that doesn't even reach `safe_mode_listen()`. This is the path the Arduino `MKRWANFWUpdate_standalone` sketch uses.
4. **Layer 4 — Carrier BOOT DIP switch (SW1 = ON).** Manual fallback for an unattended unit whose H7 has also failed. Requires opening the enclosure but no soldering or special tools.
5. **Layer 5 — SWD via test points.** Last resort. The Murata exposes SWDIO/SWCLK on internal pads; the Max Carrier may or may not bring them out (confirm in Phase 1). Requires a J-Link / ST-Link and a physical adapter. Expected to be needed approximately never in production.

**Golden-binary policy:** the H7 firmware always carries a known-good L072 binary in its own Flash (or eMMC on the X8). A `RECOVER_LORA` operator command invokes Layer 3 to flash this golden binary unconditionally. The golden binary is the latest validated launch firmware, not the development tip.

## 4. Sequence diagrams

### 4.1 Normal boot

```
H7                                L072
 |                                   |
 |  --- NRST low/high pulse --->     |
 |                                   |  reset vector
 |                                   |  hal_init()
 |  --- (no safe-mode magic) --->    |
 |                                   |  safe_mode_listen() expires after 500 ms
 |                                   |  application starts
 |  <-------- BOOT_URC --------      |
 |                                   |
 |  --- VER_REQ --->                 |
 |  <----- VER_URC ------            |
 |                                   |
 |  --- TX_FRAME (ControlFrame) ---> |
 |                                   |  -> SX1276 -> air
 |  <----- TX_DONE_URC -------       |
```

### 4.2 Brick recovery via UART safe-mode

```
H7                                L072 (running misbehaving binary)
 |                                   |
 |  --- NRST low/high pulse --->     |
 |                                   |  reset vector
 |                                   |  hal_init() (USART2 up)
 |  --- 0xA5 0x5A 0xA5 0x5A "SAFE" -->
 |                                   |  safe_mode_listen() matches
 |                                   |  jump to 0x1FF00000 (ROM bootloader)
 |  --- STM32 ROM bootloader UART --->
 |  <--------- ACK ---------         |
 |  ... flash a new binary ...       |
```

### 4.3 Brick recovery via BOOT0 (last-ditch)

```
H7                                L072 (binary doesn't even reach safe_mode_listen)
 |                                   |
 |  --- BOOT0 high --->              |
 |  --- NRST low/high pulse --->     |
 |                                   |  reset vector enters ROM bootloader directly
 |  --- STM32 ROM bootloader UART --->
 |  <--------- ACK ---------         |
 |  ... flash a new binary ...       |
 |  --- BOOT0 low --->               |
 |  --- NRST low/high pulse --->     |
 |                                   |  normal boot
```

## 5. H7-side responsibilities (binding)

The H7 firmware **must** maintain these invariants for brick-resistance to hold:

- H1: Keep BOOT0 line always under H7 control. Never repurpose the GPIO. Never tri-state it during normal operation (drive low for app, drive high for bootloader entry, never floating).
- H2: Keep NRST line always under H7 control. Same rules.
- H3: Implement a `recover_lora_modem()` function that drives Layer 3 from §3 and flashes the golden binary. Accessible from the operator UI.
- H4: Maintain the golden binary in H7 Flash (or eMMC), updated only by a deliberate developer action (never by the field-update mechanism — that updates the *application* binary on the L072, not the golden recovery copy).
- H5: Validate the L072 emits a `BOOT_URC` within 1 s of NRST release, and the `VER_URC` checksum matches the expected value. If not, schedule one Layer 3 recovery automatically; if still failing, surface a fault to the operator and refuse to operate.
- H6: Throttle automatic recovery attempts (max 1 per minute, max 5 per hour) to avoid Flash-wear loops.

## 6. Configuration & feature flags exposed to the H7

The L072 firmware exposes a small set of runtime knobs via `CFG_SET`. Defaults are chosen so that the H7 doesn't need to configure anything for normal operation; configuration is only for testing, certification work, and the post-launch features.

| Key | Type | Default | Purpose |
|---|---|---|---|
| `tx_power_dbm` | i8 | 14 | Max TX power in dBm |
| `tx_power_adapt_enable` | bool | true | Enable N-07 |
| `lbt_enable` | bool | true | Enable N-04 |
| `lbt_threshold_dbm` | i8 | -90 | CCA threshold |
| `fhss_enable` | bool | true | Enable per-frame FHSS |
| `fhss_quality_aware` | bool | true | Enable N-06 |
| `fhss_channel_mask` | u64 | (region-default) | Allowed channels |
| `deep_sleep_enable` | bool | false | Enable N-09 (handheld profile turns on, tractor leaves off) |
| `beacon_enable` | bool | true | Enable N-30 emergency beacon |
| `beacon_channel_idx` | u8 | (region-default) | Emergency beacon channel |
| `host_baud` | u32 | 921600 | UART baud (changes apply on next reset) |
| `replay_window` | u8 | 64 | AES-GCM replay window size |
| `iwdg_window_ms` | u16 | 100 | IWDG kick deadline |
| `crypto_in_l072` | bool | false | N-12 — move AES-GCM into L072 |

A future `cfg get` command lets the H7 read back the active config to detect drift after a recovery.

## 7. Things the H7 must *not* do

- Must not reflash the L072 while an operator command is active. `recover_lora_modem()` requires the operator to confirm a "modem will be unavailable for ~5 s" prompt unless the modem is already faulted.
- Must not toggle BOOT0 mid-operation just to "see what happens" — it will reset the L072.
- Must not write to undocumented `REG_WRITE` addresses in production builds — the L072 firmware refuses anything outside an allowlist.
- Must not assume `RX_FRAME_URC` ordering matches air-arrival order under heavy load — the priority queue may reorder; the URC carries an `air_timestamp_us` field for ordering at the application layer.

## 8. Versioning & compatibility

- The firmware product name is `OSE-LifeTracLORA-MurataFW`.
- The firmware build version uses `vMAJOR.MINOR.PATCH`, starting at `v0.0.0` for initial bring-up builds.
- The protocol version is `ver = 0x01` for v25 launch.
- Adding a new command type increments the firmware build version but **not** the protocol version.
- Changing the meaning of an existing command type or removing one is a protocol-version change. The H7 detects this from the `VER_URC` capability bitmap and refuses to operate against an incompatible L072 firmware.

## 9. One-line summary

> **Four wires between H7 and L072 (RX/TX/NRST/BOOT0); 921600 8N1 with COBS-framed length-prefixed CRC-checked binary frames; five independent brick-recovery layers (NRST, UART safe-mode magic, H7-driven BOOT0+NRST with golden binary, manual DIP switch, SWD); H7 carries the golden binary and is the only path that field-updates the L072.**
