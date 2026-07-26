# Method G Firmware Analysis (murata_l072) — Claude v1.0

**Date:** 2026-07-25 (bench session)
**Scope:** `DESIGN-CONTROLLER/firmware/murata_l072/` — all production translation
units (~10 KLOC production + ~10 KLOC `bench/host_proto` SIL tests, 20,201 total)
plus the Python reference client (`x8_lora_bootloader_helper/method_g_stage1_probe.py`).
**Method:** seven parallel subsystem readers (host-uart, host-cmd, host-cfg,
radio-tx, radio-rx, radio-core-fhss, core-boot; ~808 K tokens read), followed by
**manual source verification of every headline claim**. Markers:
✅ = hand-verified against source by the session author; ⚠️ = agent-reported,
spot-checked plausible, not independently re-read.
**Feeds:** RS-9.3(c) bridge port, RS-2.3 TX idle, RS-1.2 freeze, RS-4.x
hardening, RS-7.4 EIRP, RS-8.2 crypto scope — see
[DESIGN-CONTROLLER/TODO.md](../DESIGN-CONTROLLER/TODO.md).

---

## 1. Architecture

**Model:** the L072 is a radio coprocessor with *no crypto, no persistence, no
watchdog*. One foreground loop; interrupts only drain hardware into rings and
set event bits.

**Boot** (`startup.c`, `boot/safe_mode.c`, `main.c`): reset → ~500 ms blocking
safe-mode listen window (host pins muxed away; wire silent) → clock init
(HSI16 SYSCLK always — deliberately, post-W1-9d; HSE/TCXO probed only for
`clock_source_id` reporting) → `host_uart_init(921600)` → radio init (failure
does **not** halt: FAULT 0x03 + `M:RADIO_BYPASS`, host link continues) →
BOOT_URC → main loop.

**Main loop order** (`main.c`): `host_uart_poll_dma` → `host_uart_service_rx`
(COBS parse + dispatch) → `flush_diag_traces` → TX mailbox service (:159) →
radio event service → `sx1276_rx_scan_tick` (:168) → `sx1276_rx_slot_follow`
(:173) → RX tick → `host_rfco_summary_tick` (:196, 60 s cadence).

**ISR side** (`host_uart.c:1037-1109`): drain-only. On line error (PE/FE/NE/ORE):
count per-flag, discard byte, `s_stats_errors++` — **one error event per ISR
entry**. Good bytes → 512 B software ring. IDLE sets a pending flag used by
foreground COBS-state recovery. Radio DIO ISRs (`sx1276.c:504`) only set bits
in an event mask.

**Key structural facts:**
- TX pipeline is depth-2 total: one frame in the radio + one parked in the
  single-slot mailbox `s_tx_pending` (`host_cmd.c:21-25`). ✅
- All URC emission is synchronous foreground busy-wait on **both** UART lanes
  (LPUART1 + USART1 mirror), no timeout (`host_uart.c:564-600`). ⚠️
- SPI1 runs at ~250 kHz with per-byte busy-wait — a 255 B FIFO load costs
  ~8 ms of main-loop time (`sx1276.c:231`). ⚠️
- The parsed-request dispatch queue is depth 4 and drops overflow **silently**
  (no ERR_PROTO) (`host_uart.c:253`). ⚠️
- FHSS: 200 ms slots, 12 ms TX head-start + 15 ms guard = 27 ms (13.5%) fixed
  overhead per slot; 50-channel table; blacklist can never trigger (legal floor
  = table size) (`sx1276_fhss*.c`). ⚠️
- The DMA RX scaffold is dead code: channel never configured, HT/TC/TE
  counters permanently 0 (`host_uart.c:560`). ⚠️

---

## 2. Wire contract (reference for the RS-9.3(c) bridge port)

**Line:** 921600 8N1, no flow control, LPUART1 (PA2/PA3). Clock source is
HSI16-derived in all cases (±1% class) — the link is marginal by design at this
baud; the W1-7 history is the proof it works. ASCII diagnostic traces share the
wire and appear **between** frames — clients must tolerate them (0x00-delimiter
framing makes this safe).

**Outer framing:** `0x00 | COBS(inner) | 0x00`. Any 0x00 terminates an
accumulation; empty chunks ignored (0x00 is a safe resync preamble). Encoded
chunk cap 323 B; overflow discards until next 0x00.

**Inner frame** (little-endian):

| off | size | field |
|---|---|---|
| 0 | 1 | `ver` = 0x01 |
| 1 | 1 | `type` |
| 2 | 1 | `flags` |
| 3 | 2 | `seq` (LE) |
| 5 | 2 | `payload_len` (LE, must match exactly) |
| 7 | N | payload, N ≤ 311 |
| 7+N | 2 | CRC-16/CCITT-FALSE (init 0xFFFF, poly 0x1021, no reflect/xorout) over bytes [0..7+N-1], appended LE |

**Types:** host→L072: PING 0x00, VER 0x01, UID 0x02, RESET 0x03, TX_FRAME 0x10,
CFG_SET 0x20, CFG_GET 0x21, REG_READ 0x30, REG_WRITE 0x31, STATS_RESET 0x40,
STATS_DUMP 0x41. L072→host: VER_URC 0x81, UID_URC 0x82, TX_DONE 0x90,
RX_FRAME 0x91, CFG_OK 0xA0, CFG_DATA 0xA1, REG_DATA 0xB0, REG_WRITE_ACK 0xB1,
RADIO_IRQ 0xC0, STATS 0xC1, AIRTIME_REJECT 0xC2, RFCO_PERTX 0xC3,
RFCO_SUMMARY 0xC4, BOOT 0xF0, FAULT 0xF1, READY 0xF2, ERR_PROTO 0xFE.

**Key payloads:**
- `TX_FRAME_REQ`: `[tx_id u8][length u8][payload]` — **accepted silently**;
  outcome arrives later as TX_DONE_URC `{tx_id, status, time_on_air_us u32,
  tx_power_dbm}` (status only ever 0=OK/1=TIMEOUT on this wire; LBT/BUSY codes
  are dead values — refusals surface as ERR_PROTO FORBIDDEN instead). ⚠️
- `RX_FRAME_URC`: `{len u8, snr_db i8, rssi_dbm i16, timestamp_us u32,
  payload[len]}`.
- `FAULT_URC` (24 B): `{code, sub, reserved u16, pc u32, lr u32, psr u32,
  bfar u32, uptime_ms u32}` — note uptime already exists here (RS-4.7).
- `ERR_PROTO` (5 B): `{offending_type, offending_ver, err_code, detail u16}`;
  err codes: BAD_VERSION 1, UNKNOWN_TYPE 2, BAD_LENGTH 3, BAD_CRC 4,
  BAD_COBS 5, TOO_LARGE 6, QUEUE_FULL 7, FORBIDDEN 8.
- `STATS_URC`: 132 B of u32-LE counters at fixed offsets (host_types.h:119-155);
  offsets 16/20/24 (irq_ht/tc/te) are permanently 0 (dead DMA scaffold).
- `BOOT_URC` ≥6 B: `{reset_cause, radio_ok, radio_version, proto_ver,
  schema_ver, clock_source_id}`.

**Client rules a port must honor** (each burned the reference client once):
1. **No transport ACK, no NAK for malformed frames** — bad COBS/CRC/length are
   dropped with counters only. Error model = timeouts.
2. **Three requests produce no reply**: RESET_REQ, STATS_RESET_REQ, and any
   *accepted* TX_FRAME_REQ (reply comes later as TX_DONE).
3. **Match VER_URC by type only**, not (type, seq) — the reference client's
   special case (probe:324).
4. **Dispatch-queue overflow is silent**; TX-mailbox overflow is an explicit
   ERR_PROTO QUEUE_FULL(7). Distinguish these when diagnosing timeouts.
5. **All TX-begin refusals collapse to FORBIDDEN(8), detail=0** — LBT busy,
   dwell cap, airtime budget, scheduler not-init are indistinguishable on-wire.
6. **Boot contract:** ~500 ms silence after reset, then BOOT_URC; gate on
   `reset_cause` + `clock_source_id` (this is the RS-5.1 post-reset check).
7. **No persistence:** every reset reverts all CFG (profile 0, mask 0xFF,
   power 14). The host must replay its full CFG recipe after every reset —
   including the 50-ch mask *before* selecting profile 1.
8. **CFG_OK does not echo the value**; TX_POWER out-of-range is silently
   clamped with status 0. Never assume set == requested.
9. **AT-shell autodetect** can steal a frame whose COBS bytes spell
   `A/a`+`T/t`+printables+CR/LF when the accumulator is empty — vanishingly
   rare for ver=0x01 frames, but a resync preamble of 0x00 before bursts is
   cheap insurance. ⚠️
10. A mid-frame pause ≥1 char time can trigger IDLE-based discard of a partial
    frame — write frames in one syscall, never byte-dribble. ⚠️

---

## 3. Findings

### Verified by hand ✅

| # | Severity | Where | Finding |
|---|---|---|---|
| F1 | **BUG** | `sx1276_lbt.c:122-127` + `sx1276_cad.c:14` | **CAD timeout permanently wedges all LBT TX.** The LBT timeout path goes to standby and returns ERROR but never clears `s_cad_active`; no abort API exists — only `cad_poll()` seeing CAD_DONE clears it. After one CAD timeout, every `sx1276_cad_begin()` returns false → every LBT-enabled TX refused (as indistinguishable FORBIDDEN) until reboot. |
| F2 | **RISK** | `config.h:88-89` et al. | **No watchdog exists.** `IWDG_BOOT/RUN_WINDOW_MS` constants, the CFG key (0x0D), and RESET_CAUSE_IWDG decoding all exist, but the IWDG peripheral is never started — no KR writes anywhere. Any foreground wedge (see F4) is permanent until manual reset. |
| F3 | **BUG** | `stm32l072_regs.h:239-241` | **Interrupt priorities are silently all 0.** `nvic_enable_irq` writes `priority << 4`, but Cortex-M0+ implements only bits [7:6]; both values in use (1 = host UART, 3 = radio DIO) decode to hardware priority 0. The intended UART-over-radio prioritization does not exist. (Also: NVIC_IPR written with byte access, out of ARMv6-M contract — works on this silicon.) |
| F4 | **RISK** (chain) | `host_uart.c:564-600` + F1 + F2 | **Compounding availability hole:** URC emission is blocking busy-wait on both UART lanes with no timeout; with no watchdog (F2), a wedged lane wedges the firmware forever; with F1, one CAD timeout silences TX forever. Three independent designed-in single-points, no recovery path. |
| F5 | note | `config.h:50` | `HOST_TXQ_DEPTH 8` (and `HOST_TXQ_P0_RESERVED 1`) are defined and referenced **nowhere** — the depth-8 TX queue was planned, RAM-budgeted, and never built. RS-2.3's ring has a reserved slot waiting. |
| F6 | **RISK** | `sx1276_tx.c:259` | **Run-31's root cause is still open in firmware.** `s_rearm_rx` keys off *tracked* state (`state_before`); RXCONT armed via raw REG_WRITE (opmode 0x01 is on the diag allowlist) leaves tracked state STANDBY, so post-TX the modem parks in STANDBY — deaf. `faed3f02` fixed this **host-side only** (per-fragment `_ensure_rxcont`). Every future HostLink client (the ported bridge!) must either replicate the workaround or the firmware must track raw-armed state. |

### Agent-reported, high-consequence ⚠️

- **RS-1.2 mechanism candidate:** TX-begin can stall the main loop up to
  ~25 ms/attempt (1 ms PLL spin + 20 ms CAD poll + RSSI settle); blocking URC
  bursts add ~3 ms/lane; during stalls inbound bytes accumulate in the 512 B
  ring and overflow **silently**, corrupting subsequent host frames
  (`sx1276_tx.c:80`, `host_uart.c:134`). Firmware-side contribution to the
  freeze signature; host remains prime suspect.
- **Run-33 LBT mechanism confirmed in code:** once CAD detects busy,
  exponential backoff (10 ms << attempt, cap 500 ms) refuses every attempt
  *without sampling the channel* (`sx1276_lbt.c:97`).
- **QoS airtime gate unsigned underflow** if the budget cap is lowered below
  a channel's already-booked usage mid-window (e.g. DTS→FHSS profile switch)
  (`sx1276_airtime.c:211`).
- **Slot-clock drift assumption wrong by ~3 orders of magnitude:** the ±2 ppm
  TCXO figure (≈0.12 ms/min) describes a timebase the code doesn't use —
  `platform_now_ms()` is SysTick off HSI16 (±1% class). RS-4.3's coasting
  budget must be derived from the real clock (`sx1276_fhss_clock.h:18`).
- **`sx1276_reg_dump()` reads RegFifo (0x00)** in its 0x00–0x42 sweep — a
  host-triggered dump during an in-FIFO payload pops a byte and corrupts it
  (`sx1276.c:497`).
- **Profile activation never programs FRF** and (for bench/DTS) leaves RX
  disarmed by design — the host raw-arm + force-FRF workarounds are
  load-bearing (`host_cfg_profile.c:192-226`).
- **RS-7.4 inversion:** the exact §15.247(b)(4) clamp *function* exists —
  `Pmax = min(tier − max(0, gain−6), hw_ceiling)`, floor 2 dBm
  (`host_cfg_profile.c:103`) — but it is used only as a pass/fail headroom
  check at profile-set; the computed ceiling is **never applied** to TX power,
  and gain/ceiling keys accept writes *after* activation with no re-check.
- **Deferred baud switch unimplemented:** CFG_KEY_HOST_BAUD is stored and
  acked DEFERRED(5) but nothing ever consumes it (`host_cfg.c:115`).
- Size-budget CI script evaluates deleted `MM_BOOT_*` symbols and always
  exits nonzero (`tools/check_size_budget.py:162`); Makefile `size` banner
  shows the stale pre-unification memory map.
- Reset-cause decode checks PINRSTF before PORRSTF, so a true power-on reset
  reports as pin reset in BOOT_URC (`platform.c:37`) — weakens RS-5.1's
  reset-verification gate.
- `RADIO_IRQ_URC` debug emission is compiled on in production (~16 wire bytes
  per radio-event batch) (`host_cmd.c:841`).
- Historical trap for the RS-2.3 ring refactor: the 2026-07-24 fix moved the
  payload memcpy to cover the *parked* path — a ring must copy at park time
  or re-introduce the transmit-uninitialized-stack bug (`host_cmd.c:454`).

---

## 4. RS-item map

| RS | What the code says |
|---|---|
| **RS-2.3** | Single-slot mailbox confirmed (✅). But the 25% idle has *five* candidate contributors, not one: (1) depth-2 pipeline refill latency; (2) SPI ~8 ms per 255 B FIFO load; (3) blocking URC emission 3–6 ms; (4) FHSS 13.5% slot overhead; (5) LBT stalls ≤25 ms. The ring fix: replace three statics at `host_cmd.c:21-25`, drain in `service_tx_mailbox`, busy-reply becomes QUEUE_FULL at depth N; `HOST_TXQ_DEPTH=8` is already defined and RAM-budgeted (F5). Preserve memcpy-at-park. Instrument per-contributor before sizing N. |
| **RS-1.2** | Firmware cannot deadlock while UARTs clock (TXE always eventually sets) but can stall ≥25 ms and then silently lose RX bytes to ring overflow — a concrete corruption mechanism to test for in the base worker's input stream. |
| **RS-4.3** | `LOCK_LOSS_MS=2000`; demotion hard-resets the slot clock. Coasting hooks exist, but the drift budget must be recomputed for HSI16 (±1%), not TCXO 2 ppm. |
| **RS-4.4** | CFG keys (0x09/0x0A) + `LORA_FW_BEACON_ENABLE=1` exist with zero consumers. RX side already anchors on 8 B header-only frames — beacon is TX-side-only work. |
| **RS-4.5** | Slot-clock arithmetic already has TU coverage; the follower needs a scripted-sequence test. The scan-SM transition list to cover is enumerated in the radio-rx reader output. |
| **RS-4.6** | `freq_hz` is already latched per-TX and carried in RFCO_PERTX (0xC3); bench/DTS profiles deliberately zero it. Fix = populate from the host-pinned FRF, or emit `sx1276_fhss_chantab_center_hz(idx)`. |
| **RS-4.7** | `uptime_ms` already ships in FAULT_URC offset 20; VER_URC byte 7 is an explicit reserved=0 and the payload is additive-friendly. **But no boot counter is possible without a flash write path, and none exists** — scope RS-4.7 to uptime-only or add a flash writer first. |
| **RS-7.4** | Rewrite the item: the arithmetic exists; the *enforcement* is missing (see §3). Fix = apply the clamp output as the TX ceiling + re-validate on antenna-gain/hw-ceiling writes post-activation. |
| **RS-8.2** | **Reframe: the L072 needs no crypto work at all.** Crypto Profile A is explicit (`config.h:24`): the X8/H7 owns AES-GCM; L072 transports opaque bytes; `CFG_KEY_CRYPTO_IN_L072` is permanently unwritable; the LoRa header reserves `schema_ver=2` for a future trailing MIC. The D13 GCM-64 codec lands in the **host-side** code (Python `lora_proto.py` + the three vendored H7/handheld `lp_crypto` trees) — RS-8.2's vendored-trees warning stands, its murata_l072 implication does not. |
| **RS-9.3(c)** | §2 is the port contract. Add to the port checklist: replay full CFG after every reset (no persistence); replicate `_ensure_rxcont` (F6) until the firmware tracks raw-armed state; never await the three no-reply requests. |
| **RS-9.7/9.8** | FHSS numerology confirmed: 200 ms slot, 12 ms head-start, 15 ms guard, 380 ms per-frame legal cap, 400 ms/10 s per-channel dwell — consistent with the cap analysis pinned in RS-9.7. |
| **RS-5.1** | BOOT_URC (`reset_cause` + `clock_source_id`) after the 500 ms safe-mode window is exactly the post-reset gate the harness needs — but note the PINRSTF/PORRSTF mis-order above. |

---

## 5. Corrections to prior session assumptions

1. **T0's HOST_ERRORS interpretation is confirmed:** the 1.77 M count is
   framing-error *service events* (one per ISR entry, errored byte discarded
   pre-ring) — the baud-mismatch signature. `HOST_PARSE_ERR` stayed 0 because
   garbage never reaches the COBS parser as clean bytes.
2. **One reader claimed a "100 ms IWDG" resets the chip — false** (it read the
   config constants). Verified: no watchdog is ever started (F2).
3. **One reader claimed the TX path already sees raw-armed RXCONT — false**
   (it read the tracked-state snapshot as a register read). Verified still
   open (F6).
4. RS-7.4 and RS-8.2 in TODO.md carry now-outdated framings; see §4.

*Raw reader outputs (7 subsystems, structured JSON) live in the session task
output; this document is the curated durable record.*
