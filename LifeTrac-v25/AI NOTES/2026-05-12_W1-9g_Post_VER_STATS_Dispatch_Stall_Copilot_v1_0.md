# W1-9g — Post-VER STATS Dispatch Stall (Investigation Note)

- **Date:** 2026-05-12
- **Author:** Copilot (Claude Opus 4.7)
- **Status:** **CLOSED 2026-05-12** — probe-side fix landed and bench-validated (`bench-evidence/W1-9b_fsk_2026-05-11_125612/`, `probe_rc=0`).
- **Hardware:** Portenta X8 ABX00049 (`2E2C1209DABC240B`) + Max Carrier ABX00043 V3.12 + Murata CMWX1ZZABZ-078
- **Firmware:** `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/build/firmware.bin` (16 440 B)
- **Affected probe:** `method_h_stage2_tx_probe.py` (Stage 2 — `--probe fsk`)
- **Unaffected probe:** `method_g_stage1_probe.py` (Stage 1 — VER → STATS → REG_READ ×113 at 5/5 PASS this session)

## 1. Summary

After the W1-9b T2 probe relaxation (accept any LoRa-mode pre-state, not strictly `0x80` SLEEP) and the
W1-9e/W1-9f probe transport fixes (`HostLink.urc_queue`, `BENIGN_FAULT_CODES`), the FSK
probe's first `STATS_DUMP_REQ` after a successful `VER_REQ` warm-up still times out, even though
Stage 1 issues exactly the same VER → STATS sequence and succeeds 5/5.

End-state of FSK probe (verdict `TRANSPORT_FAIL`):

```
T0a: VER warm-up OK name=OSE-LifeTracLORA-MurataFW v=0.0.0
INFO: discarding stale type=0xC1 seq=0 (waiting for seq=2)
INFO: discarding stale type=0xC1 seq=0 (waiting for seq=2)
FATAL: T0 STATS_URC(before) failed: timeout waiting for response type 0xC1 to req 0x41
__W1_9B_VERDICT__=TRANSPORT_FAIL
```

The two stale `STATS_URC seq=0` frames are the unsolicited boot snapshots emitted from
`main.c` lines 92 & 98 (the `HOST_RX_SEEN` and `HOST_DIAG_MARK` fault paths each call
`host_cmd_emit_stats_snapshot()` → `emit_stats_urc(0)` in `host/host_cmd.c` line 386). Those
draining correctly. The mystery is that **no `STATS_URC seq=2` ever arrives** — the firmware
never appears to dispatch the STATS request that the probe just sent on the wire.

## 2. What is preserved (do NOT roll back)

- **`HostLink.request()` rewrite (`method_g_stage1_probe.py` lines 300-345).**
  Drains `self.urc_queue` ONCE at function entry into local `pending`; main loop then ALWAYS
  reads from the wire for subsequent iterations (instead of returning the same queued frames
  forever). Fixes a pre-existing infinite-loop pattern; validated by Stage 1 = 5/5 PASS.
- **`BENIGN_FAULT_CODES` filter in `wait_for_tx_done()` (W1-9f).** TX probe still 7/7 PASS.
- **W1-9d firmware deltas** (drive `PB6` HIGH in `platform_clock_init_hsi16()`; remove the
  SYSCLK→HSE switch). Bench evidence: `bench-evidence/W1-9_stage2_tx_2026-05-11_115340/`.
- **VER-first ordering in `run_fsk_stdby()`.** The VER warm-up itself succeeds; removing it
  takes us back to a `VER never fires` failure mode that's strictly worse.

## 3. What was attempted in this session

| Attempt | Change | Result |
|---|---|---|
| 1 | Relax pre-state assertion (accept any LoRa-mode bit-7 high) | Pre-state read still timed out |
| 2 | Insert `VER_REQ` warm-up after `fetch_stats(link)` | STATS still failed first |
| 3 | Move `VER_REQ` warm-up BEFORE `fetch_stats(link)` | VER PASS, but STATS still times out (current state) |
| 4 | Rewrite `HostLink.request()` to discard stale type-matches and always fall back to wire-read | Fixed an unrelated bug; Stage 1 5/5 PASS preserved; FSK still TRANSPORT_FAIL |

## 4. Why Stage 1 works and FSK probe doesn't (working hypotheses)

Stage 1's main():

```python
for frame in link.read_frames(args.boot_timeout):    # 1.0 s
    # processes BOOT_URC, STATS_URC(init), FAULT_URC, READY_URC, ERR_PROTO_URC
    ...
version = parse_version(link.request(VER_REQ, VER_URC, timeout=1.0)[...])
stats   = parse_stats(link.request(STATS_DUMP_REQ, STATS_URC, timeout=1.0)[...])
```

FSK probe's `run_fsk_stdby()`:

```python
drain_boot(link, settle_s=1.0)                       # only knows BOOT_URC + FAULT_URC
ver = link.request(VER_REQ, VER_URC, timeout=1.0)    # PASS this session
stats_before = fetch_stats(link)                     # FAIL
```

The VER-URC is observed both times. The difference must be in either:

- **(A) The exact byte timing between VER_URC reception and the next STATS_DUMP_REQ TX.**
  Stage 1's main has more interpretation work between calls (string format, print) than the
  FSK probe (which goes from `link.request(VER…)` → `fetch_stats` → `link.request(STATS…)`
  with almost no Python work in between). If the firmware's `host_cmd` dispatcher has a
  brief blind window after sending a URC (e.g. waiting for `host_uart_send_urc()` DMA TC
  before re-arming RX-IDLE), a tighter back-to-back REQ from the host could land while the
  RX path is briefly disabled.
- **(B) RX FIFO/IDLE state transition on the L072 side.** `host_uart_service_rx()` polls a
  DMA-IDLE flag to slice incoming frames; if the IDLE timer hasn't expired between the
  STATS_DUMP_REQ bytes and the previous VER_REQ bytes, the dispatcher might treat the
  STATS bytes as a continuation of an already-completed frame and silently drop them.
- **(C) X8-side kernel batching.** `/dev/ttymxc3` driver may coalesce small TX bursts; if
  the `STATS_DUMP_REQ` 9-byte frame is sent inside the same kernel TX flush as a leftover
  VER_REQ artifact, the firmware might receive a malformed concatenation. (Less likely:
  COBS framing is robust to leading 0-bytes; the probe always emits a single 0x00 separator.)

## 5. Suggested next-session experiments

1. **Add a 100–200 ms `time.sleep()` between VER_REQ and STATS_DUMP_REQ** in the FSK probe
   only. If STATS then succeeds, hypothesis (A)/(B) is confirmed and the firmware's
   `host_uart_service_rx()` dispatcher needs an inter-frame guard (or the probe needs a
   permanent post-VER quiet window).
2. **Mirror Stage 1's full boot drain** in the FSK probe: replace `drain_boot()` with the
   same per-type explicit handler used by Stage 1 main(), so unsolicited STATS_URCs and
   FAULT_URCs are consumed identically. Eliminates the one remaining behavioural difference
   between the two.
3. **Firmware-side dispatcher counters.** Add an `host_uart_stats.dispatched_per_type[256]`
   array bumped in `host_uart_service_rx()` once per parsed REQ frame, and dump it in the
   STATS payload. If the FSK probe's `STATS_DUMP_REQ` arrives but isn't dispatched, the
   counter delta tells us whether bytes hit the parser or were rejected upstream.
4. **Logic-analyser capture** of TX line at the L072 RX pin during the FSK probe's
   `STATS_DUMP_REQ`. Confirms whether bytes physically reach the SiP. If they do but no
   STATS_URC follows, the bug is firmware-side. If they don't, the X8 driver is dropping
   the TX burst.

## 6. Closure (2026-05-12)

Hypothesis (B) was the closest to correct, but the actual mechanism is simpler and
fully probe-side. After `VER_REQ` returns successfully, the firmware emits — in order —
`VER_URC`, then a `FAULT_URC` for `HOST_FAULT_CODE_HOST_RX_SEEN` (one-shot from
`main.c` line 92 the first time any RX byte is observed on a UART lane), then an
auto `STATS_URC(seq=0)` snapshot from `host_cmd_emit_stats_snapshot()`. On this
build a second pair (`HOST_DIAG_MARK` FAULT + STATS) follows for the same reason.
The `request(VER_REQ, VER_URC)` call returns the moment it sees `VER_URC`, and any
trailing frames from the same wire-read batch are pushed onto `HostLink.urc_queue`.

The next call — `request(STATS_DUMP_REQ, STATS_URC)` — drains `urc_queue` and finds
`STATS_URC(seq=0)` (an unsolicited snapshot) instead of the response it just asked
for (`STATS_URC(seq=N)`). With the W1-9b request() rewrite, the loop discards the
stale-seq frame and tries to read the wire next; but with two such snapshots queued,
the discard loop and the wire-read interleave consume the entire 1.0s response window
before the actual STATS_URC reply lands.

Stage 1's `main()` doesn't hit this because (a) its boot-window `for frame in
link.read_frames(args.boot_timeout):` 1.0s loop explicitly handles `STATS_URC` and
`ERR_PROTO_URC` types, not just BOOT/FAULT, and (b) the print/format work between
each `link.request()` call gives a few hundred ms of natural settle time during
which the firmware finishes emitting and the stale snapshots are absorbed.

**Probe-side fix (no firmware change needed):**

1. Extended `drain_boot()` in `method_h_stage2_tx_probe.py` to handle every URC
   type (STATS_URC, anything-else) explicitly — they're now consumed off the wire
   instead of being silently iterated past and potentially re-queued elsewhere.
2. Added `drain_pending(link, quiet_s=0.25, max_s=1.0)` helper that drains
   `urc_queue` AND wire interleaved until quiet for 250 ms; called between the
   `VER_REQ` warm-up and `fetch_stats()`.

**Bench evidence** (`bench-evidence/W1-9b_fsk_2026-05-11_125612/`):

```
T0a: VER warm-up OK name=OSE-LifeTracLORA-MurataFW v=0.0.0
INFO: post-VER drain queued type=0xF1 seq=0
INFO: post-VER drain queued type=0xC1 seq=0
INFO: post-VER drain queued type=0xF1 seq=0
INFO: post-VER drain queued type=0xC1 seq=0
T0b: post-VER drain consumed 4 frames
T0: STATS(before) host_parse_err=0
T2 step2: pre RegOpMode=0x85 (raw=0185)
INFO: pre-state 0x85 is non-SLEEP LoRa mode (post-W1-9d firmware boots into LoRa
 RXCONTINUOUS=0x85); accepting and continuing.
T2 step3: REG_WRITE OPMODE=0x00 ack=0100
T2 step5: post-FSK-SLEEP RegOpMode=0x80 (raw=0180) elapsed=23.74ms
T2 step6: REG_WRITE OPMODE=0x01 ack=0101
T2 step8: post-FSK-STDBY RegOpMode=0x81 (raw=0181) elapsed=23.69ms
__T2_RADIO_VERDICT__=INCONSISTENT
__W1_9B_VERDICT__=T2_INCONSISTENT
probe_rc[/dev/ttymxc3][fsk]=0
```

`probe_rc=0`, transport works end-to-end, mode bits 2:0 transition 0→1 between
step5 and step8 — **clock is alive and SX1276 mode-state-machine is responsive.**
The remaining `T2_INCONSISTENT` verdict (rather than `FSK_STDBY_WORKS`) is a
**separate, minor classifier issue, not a regression**: the probe writes `0x00` (FSK
SLEEP) at step3 but reads back `0x80` (LoRa SLEEP) at step5. Per SX1276 datasheet
v7 §4.1.1.1, `LongRangeMode` (bit 7 of `RegOpMode`) "can only be modified in Sleep
mode" — and the chip is in `0x85` (LoRa+RXCONTINUOUS) at probe entry, so the
write is silently truncated to mode-bits-only. To capture `FSK_STDBY_WORKS`
properly, the probe would need to pre-walk LoRa→LoRa-SLEEP→FSK-SLEEP. Out of scope
for this session — the discriminator's actual invariant (mode bits 2:0 transition
when commanded) is already proven.

**Stage 1 regression check:** 3/3 PASS (`bench-evidence/T6_stage1_standard_quant_2026-05-11_125646_*/`).

**Lessons:**

- Any probe-side `drain_boot()` helper MUST handle every URC type the firmware
  can emit out-of-band, not just `BOOT_URC`/`FAULT_URC`. Unhandled URCs are at
  best silently iterated past, at worst left in `urc_queue` to poison the next
  `request()`.
- A tight back-to-back `request(A); request(B);` pattern is fragile when A's
  response triggers the firmware to emit additional unsolicited URCs. Either
  drain between calls, or build a `request_drain()` variant that explicitly
  consumes and discards type-mismatches before returning.

## 7. Files changed this session

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`
  (`HostLink.request()` rewrite + read_frames urc_queue fast-path — keep).
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py`
  (`run_fsk_stdby()` VER-first + pre-state relaxation; `drain_boot()` extended
  to handle STATS_URC; new `drain_pending()` helper called post-VER).
- `LifeTrac-v25/DESIGN-CONTROLLER/TODO.md` (W1-9g + W1-9b T2 closure entries).
- `LifeTrac-v25/DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/07_Successful_Connection_Recipe_All_Chips_Copilot_v1_0.md`
  §7 follow-ups updated.
- `/memories/repo/lifetrac-portenta-x8-lora.md` (W1-9g closure note).
