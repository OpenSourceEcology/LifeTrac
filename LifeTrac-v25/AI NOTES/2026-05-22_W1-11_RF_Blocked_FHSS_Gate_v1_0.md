# W1-11 ping_pong RF blocked by post-FHSS firmware gate

**Date:** 2026-05-22
**Author:** assistant (autonomous closure attempt — needs user sign-off before any fix)
**Scope:** Why "resume RF testing" failed and what the next decision is.

---

## TL;DR

After today's HC-04 reflash of both X8/L072 boards (TX = `2E2C1209DABC240B`,
RX = `2D0A1209DABC240B`) the post-flash W1-11 ping_pong regression failed
**5/7 gates** with zero radio activity (`radio_tx_ok=0/100`, no `__RX_FRAME__`,
no `__TX_DONE__`). Root cause is **not** an RF/hardware fault and **not** a
bench-probe URC-drain regression — it is a firmware-vs-host-probe contract
break introduced by yesterday's FHSS routing commit:

| Item | Detail |
|---|---|
| Commit | `d4dfcb8` (2026-05-20 02:24) "Add FHSS, RFCO summary, RX-scan docs & radio" |
| Build flag | `-DLIFETRAC_FHSS_TX_ROUTED=1` (now unconditional in `murata_l072/Makefile`) |
| Firmware change | `sx1276_tx_begin()` and the RX path call `sx1276_fhss_next_channel()` before TX/RX start; if the FHSS scheduler is not initialised it returns non-OK and TX/RX are refused |
| Host contract gap | No bench probe (`method_h_stage2_tx_probe.py`, `method_h_stage2_tx_probe_v2.py`, `method_g_stage1_probe.py`) issues `CFG_SET CFG_KEY_REG_PROFILE` (key `0x14`) before its first `TX_FRAME_REQ` |
| Yesterday's PASS (2026-05-20 18:38) | Boards still ran pre-`d4dfcb8` firmware loaded before the FHSS landing — they were not reflashed yesterday |
| Today's reflash effect | HC-04 wrote the post-`d4dfcb8` `firmware.bin` for the first time → first run after reflash exposed the gate |

The right fix is non-trivial (touches FCC-relevant code paths) so this note
**documents** the diagnosis and stops the bench rather than guessing a fix.

---

## Evidence

Run dir: `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/W1-11_pingpong_2026-05-22_165144/`

Both X8 boards present in `adb devices`; HC-04 had passed on both <1 h earlier;
P1 cold-boot discriminator was 5/5 clean on RX immediately before this run.

### TX side (board 2E2C, post-reflash firmware)

`summary.json`:

```
tx_done_ok       = 0
tx_done_rate     = 0
tx_timeouts      = 100
rx_frames        = 0
rssi_median_dbm  = null
rx_real_faults   = 2
verdict          = RX_PAIR_FAIL_5_GATES
```

`2E2C1209DABC240B_method_h_stage2_ping_pong.log` (tail, repeated 100×):

```
INFO: unrelated frame during TX wait type=0xC3
__TX_ERR__ idx=N ERR_PROTO during TX wait: 1001080000
...
PING_PONG: STATS(after) radio_tx_ok=0 (delta=0) radio_tx_abort_lbt=0 (delta=0) radio_tx_abort_airtime=0 (delta=0)
__W1_10B_BURST_DONE__ tx_count=100 tx_done_ok=0 tx_done_fail=0 tx_timeout=100 ...
__W1_11_PINGPONG_DONE__ tx_count=100 tx_done_ok=0 pong_received=0 pong_timeout=0
__RADIO_SLEEP_ON_EXIT__ (ping_pong) ack=0180 RegOpMode(post)=0x80 (target=0x80)
__BENCH_RADIO_SLEEP_AUDIT__=PASS
```

ERR_PROTO payload `1001 08 0000` decodes (per `format_err_proto_payload`):
- `offending_type=0x10` = `HOST_TYPE_TX_FRAME_REQ`
- `offending_ver=0x01`
- `err_code=0x08` = `HOST_ERR_PROTO_FORBIDDEN`
- `detail=0`

The 0xC3 logged as "unrelated frame" is `HOST_TYPE_RFCO_PERTX_URC` —
firmware emits it once per refused TX with status `INTERNAL` (see
`sx1276_tx.c:139,167`). It is *informational*, not the cause; the cause is
the FORBIDDEN ERR_PROTO that immediately follows.

### RX side (board 2D0A, post-reflash firmware)

`2D0A1209DABC240B_method_h_stage2_rx_echo.log` (tail):

```
__W1_10B_LISTEN_READY__
__RX_FAULT__ code=0x0D sub=0x20 raw=0d20...3e750000
__RX_FAULT__ code=0x0D sub=0x21 raw=0d21...6eea0000
RX_ECHO: STATS(after) radio_rx_ok=0 (delta=0) radio_crc_err=0 (delta=0)
__W1_10B_LISTEN_DONE__ rx_frames=0 ... real_faults=2 invariants_violated=0
```

`HOST_FAULT_CODE = 0x0D` is `HOST_FAULT_CODE_RX_SCAN_FAILED` (see
`murata_l072/include/host_types.h:198`). Sub-byte fields are defined in
`murata_l072/include/sx1276_rx_scan_fail.h`; the codes shown
(0x20 = SUB_BIT_HW_SUSPECT bit, 0x21 = + CRC_SEEN) confirm the RX scan
setup itself is refused, not the antenna front-end.

---

## Why the 2026-05-20 ping_pong was fine

`bench-evidence/W1-11_pingpong_2026-05-20_183815/summary.json`:

```
tx_done_ok=100  tx_done_rate=1  pong_received=97  rssi_median_dbm=-114
rx_real_faults=0  verdict=RX_PAIR_FAIL_2_GATES   (only B2 and B7 failed)
```

That run executed at 18:38; commit `d4dfcb8` (FHSS routing) is dated
02:24 the same day. The bench L072s were last flashed on 2026-05-18 (HC-04
verdict matrix) — before the FHSS landing. So the boards were still
running the **pre-FHSS** firmware until today's HC-04 reflash uploaded the
post-FHSS `firmware.bin`. The 2026-05-20 PASS therefore does not
disprove the FHSS-gate diagnosis; it predates it.

---

## Decision points (require user input)

This is a contract change between firmware and host probes; both fixes are
plausible and they have different cost / risk / regulatory footprints. I'm
listing them in order of cheapest-to-revert:

### Option A — Probe-side: send `cfg_set(CFG_KEY_REG_PROFILE)` at probe start

- Probes already declare `CFG_KEY_REG_PROFILE = 0x14` and know the enum
  table (`BENCH_ONLY_FIXED_915=0`, `FCC_FHSS=1`, `FCC_DTS=2`). They
  currently only use it for `CFG_GET` readout (FCC-B3-1).
- Add a `cfg_set` call before the boot-drain + STATS snapshot in every
  TX/RX entrypoint (`run_tx_probe`, `run_rx_listen`, `run_rx_echo`,
  `run_rx_pair`, `run_ping_pong`, `run_w2_02_*`). Profile choice should
  default to `FCC_FHSS=1` for bench because that is what the gated path
  expects; bench fixed-frequency operation would need `BENCH_ONLY_FIXED_915`
  AND a firmware change so the fixed-915 case bypasses the FHSS scheduler.
- Risk: every probe needs the same change; if the profile request itself
  has a multi-field payload (channel mask, etc., per `host_cfg_profile.h`),
  the probe also has to build that payload correctly. Not a one-liner.

### Option B — Firmware-side: make `BENCH_ONLY_FIXED_915` an auto-init no-op for the FHSS scheduler

- Today the firmware boots with `host_cfg_profile_active() == NULL` so
  every gated call falls back to "scheduler not ready → refuse". If
  `BENCH_ONLY_FIXED_915` is the documented default for unflashed/lab use,
  the firmware should plant a 1-channel scheduler on boot for that enum
  and only require an explicit `cfg_set` to switch to `FCC_FHSS` or
  `FCC_DTS`.
- Risk: FCC-adjacent change. Needs author of the FHSS work to confirm the
  bench-only profile is supposed to bypass the scheduler. Also requires a
  rebuild + new HC-04 reflash cycle to validate.

### Option C — Test-only build flag

- Add `-DLIFETRAC_FHSS_TX_ROUTED_BENCH_BYPASS=1` (or similar) that, when
  set, lets `BENCH_ONLY_FIXED_915` skip `sx1276_fhss_next_channel()`. Two
  firmware variants then: production (`firmware.bin`) and bench
  (`firmware_bench.bin`). HC-04 picks the bench one for regression runs.
- Risk: more permanent than B but introduces variant management. Avoids
  changing the production wire contract.

My recommendation if forced to choose autonomously: **Option A**, because
it is reversible at the host-probe layer alone and doesn't touch FCC code
paths. But I am **not** applying it without user confirmation because
multiple probes need it AND the payload format for `cfg_set(0x14, …)` is
not obvious from a five-minute read of `host_cfg_profile.h`.

---

## What I am NOT doing

- Not re-running RF tests with the existing probes (already proven to
  fail; pointless bench cycles).
- Not modifying firmware or probe source code (high-risk autonomous edit;
  needs sign-off).
- Not reflashing TX or RX with an older `firmware.bin` (would erase
  today's proof of the gate and reopen the question of which build is on
  each board).

---

## Cross-links

- HC-04 verdict (today, both boards PASS): `DESIGN-CONTROLLER/X8_HEALTH_AND_RECOVERY/routines/HC-04_stage1_standard_quant.md` 2026-05-22 rows.
- P1 cold-boot discriminator (today, 5/5 PASS): `AI NOTES/2026-05-22_RX_Reflash_And_P1_Resolution_v1_0.md`.
- Reference good ping_pong (pre-FHSS firmware): `bench-evidence/W1-11_pingpong_2026-05-20_183815/`.
- Failed run (this note): `bench-evidence/W1-11_pingpong_2026-05-22_165144/`.
- Repo memory: `/memories/repo/lifetrac-x8-l072-bootloader.md` (NEW BLOCKER 2026-05-22 section).
