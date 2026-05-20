# FCC-B1-SUMMARY Wire Layout and Cadence Design

**Document:** `2026-05-19_FCC_B1_SUMMARY_Wire_Layout_Design_Copilot_v1_0.md`
**Author:** Copilot (Claude Opus 4.7)
**Date:** 2026-05-19
**Status:** DRAFT — proposes a concrete payload layout + cadence design for
[`FCC-B1-SUMMARY`](../TODO.md) to be ratified by user; once ratified, B1-SUMMARY-a/b/c
sub-increments are mechanical.
**Scope:** L072 firmware only. Defines (a) the per-minute `RFCO_SUMMARY` URC
wire payload, (b) the snapshot-and-reset semantics, (c) the 1-minute cadence
trigger, (d) which existing counter TUs the snapshot builder reads, and
(e) the `HOST_TYPE_RFCO_SUMMARY_URC` type-code allocation.

> **Precedence rule (inherited from 2026-05-18 §0.7).**
> Current code constants > bench evidence > `DECISIONS.md` >
> `LORA_PROTOCOL.md` > `IMAGE_PIPELINE.md` > this document. This document
> consolidates B1-SUMMARY wire format. If a later code change requires
> changing it, bump `HOST_RFCO_SUMMARY_SCHEMA_VER` and update both this doc
> AND the bench parser pin per [host_rfco.h L23–L27](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco.h#L23).

## 0. TL;DR

* The naive payload (50×u32 dwell-max + 50×u16 hop-count + headers) is
  ~330 B and **exceeds** `HOST_PAYLOAD_MAX_LEN = 320 − 7 − 2 = 311 B`
  ([host_uart.h L12](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_uart.h#L12),
  [config.h L48](../DESIGN-CONTROLLER/firmware/murata_l072/config.h#L48)).
* Resolution: store dwell-max as **u16 ms** (cap is 400 ms, range 0–65535
  ms saturates harmlessly above 65.5 s) and per-channel hop-count as
  **u8 saturating**. This drops the payload to **191 B** with room for
  additive tail growth (~120 B headroom).
* Cadence trigger: main loop polls `platform_now_ms()` against a
  `s_rfco_summary_last_emit_ms` static; if `now - last >= 60_000U`,
  fire `host_rfco_summary_emit()` and update last. **No new HAL timer.**
  This mirrors the existing `emit_stats_urc()` polling pattern
  ([host_cmd.c L386](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c#L386)).
* Snapshot semantics: **snapshot-and-reset** for the per-channel
  hop-count and blocked-attempts histograms (the "what happened in the
  last minute" intent). Dwell-max is **peak-since-last-summary** (not
  rolling 10 s — that data already lives in the per-TX URC). active_count,
  blacklist_size, last_clamp_reason are **instantaneous-at-emit** values.
* `HOST_TYPE_RFCO_SUMMARY_URC = 0xC4U` (next free in the C-range; per
  the 0xC3 PERTX comment block at
  [host_types.h L86–L93](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h#L86)).
* `HOST_RFCO_SUMMARY_SCHEMA_VER = 1U`. Tail-additive. Bench parsers
  refuse unfamiliar schema_ver (matches PERTX rule).

## 1. Hard constraints established before this design

| # | Constraint | Source |
|---|---|---|
| 1 | Max URC payload = **311 B** | [host_uart.h L10–L12](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_uart.h#L10), [config.h L48](../DESIGN-CONTROLLER/firmware/murata_l072/config.h#L48) |
| 2 | FCC-FHSS active channel count = **50** | [sx1276_fhss_chantab.c L11](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_fhss_chantab.c#L11) `_Static_assert(SX1276_FHSS_CHANNEL_COUNT == 50U)` |
| 3 | Legal-dwell cap = **400 000 µs** = 400 ms | [sx1276_legal_dwell.h L72](../DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_legal_dwell.h#L72) |
| 4 | Legal-dwell channel count = **64** (table size, only 50 referenced by FHSS) | [sx1276_legal_dwell.h L69](../DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_legal_dwell.h#L69) |
| 5 | `platform_now_ms()` exists; main loop already polls it for `lora_ping` | [lora_ping.c L386](../DESIGN-CONTROLLER/firmware/murata_l072/lora_ping.c#L386) |
| 6 | URC framing additive-tail rule, schema_ver byte 0 | [host_rfco.h L23–L27](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco.h#L23) |
| 7 | Naming discipline: `legal_dwell_used_us_10s` / `_20s` must not be conflated with `qos_used_us_1s` | [host_rfco.h L19–L22](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco.h#L19) + [tools/lint_artifact_naming.py](../tools/lint_artifact_naming.py) |
| 8 | Next free URC type code = **0xC4** | [host_types.h L80–L93](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h#L80) |

## 2. Open design questions and resolutions

### Q1. Per-channel hop-count bucket width (u8 vs u16)?

**Resolution: u8 saturating (50 bytes).**

Worst-case TX rate at +17 dBm, SF7/BW250, payload-min: ToA ~6 ms; QoS
1 s cap = 400 ms airtime ⇒ ~66 TX/s max system-wide. Per-channel
average over 1 min: ~80 / 50 ≈ 1.6 TX/channel/min. Even under a
SAFETY-burst hotspot (N=5 copies on the same hop slot, per 2026-05-18
S0.9.a) and adversarial fairness loss, **a single channel exceeding
255 hops/min would itself be a fairness-equality violation flagged by
the D1 evidence gate** (±10 % equal-use per epoch, see 2026-05-19 FCC
plan §3 D1). Saturation at 255 is therefore a useful **out-of-range
signal**, not data loss. Reserve byte value `0xFF` as
"saturated — exceeded 255 hops in this minute, investigate fairness".

### Q2. Dwell-max field width (u32 µs vs u16 ms)?

**Resolution: u16 ms (100 bytes for 50 channels).**

The interesting range is **0 ms to 400 ms** (the §15.247 cap). u16 ms
gives 0–65 535 ms = 0–65.5 s resolution-1ms, which is **165× wider
than the cap** — saturation impossible under any realistic
configuration. u32 µs would be 200 B and gain only sub-ms resolution
that bench analysis does not consume (the cap is a 1 ms-scale rule).

Per-TX URC already ships the exact-µs value at TX-time
([host_rfco.h L42–L48](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco.h#L42)),
so callers wanting µs precision should reconcile against the per-TX
stream; SUMMARY is a roll-up surface.

### Q3. Blocked-attempts histogram — how many reasons?

**Resolution: 8 reasons, 8×u16 = 16 bytes.** Mirrors `host_rfco_tx_status_t`
([host_rfco.h L80+](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco.h#L80))
non-OK values + 1 spare. Layout (indexed by enum value, **NOT** by name —
parsers MUST use the enum so additions are detected via schema_ver bumps):

| idx | reason | source |
|----:|---|---|
| 0 | `OK` (always 0 here — sanity gate; parsers MUST assert) | n/a |
| 1 | `ABORT_AIRTIME_INVARIANT` | A1b reject (FCC notes §14.2 step 1) |
| 2 | `ABORT_LBT` | LBT/CAD busy |
| 3 | `ABORT_LEGAL_DWELL` | per-channel 400 ms/10 s cap |
| 4 | `ABORT_QOS` | 1 s/400 ms QoS gate |
| 5 | `TX_TIMEOUT` | TX_DONE watchdog (post-RF) |
| 6 | `TX_FAIL` | downstream NACK / link error (post-RF) |
| 7 | `INTERNAL` | bug bucket |

Per-counter width: **u16 saturated**. Same fairness rationale as Q1:
>65535 of any single reason in 1 min implies broken loop / runaway
retry, which is itself the signal we want.

### Q4. Snapshot semantics — reset vs rolling vs cumulative?

**Resolution: snapshot-and-reset** for `per_channel_hop_count[50]` and
`blocked_attempts[8]`. Both are **delta counters** over the elapsed
1-minute window, computed by `counters_after − counters_at_last_emit`.
Implementation is "read raw → diff against snapshot stamp → write
current snapshot as next stamp" — **no destructive reset of the
underlying counter TU**, so per-TX RFCO and stats URCs keep their
cumulative semantics.

Rationale: 60 s of total bandwidth (50 hops × 1 byte + 8 reasons × 2 B
= 66 B for the delta surface) is the right granularity for D1 fairness
and D5 burst analysis. Rolling-window semantics would require per-TX
ring buffers (RAM-prohibitive at 50-channel × 60 s × ~10 TX/s ≈ ~30 KB,
exceeds ~20 KB SRAM budget). Cumulative-since-boot is what
`emit_stats_urc()` already provides for the radio counters
([host_cmd.c L386](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c#L386));
duplicating that surface adds no information.

Dwell-max: **peak-since-last-summary** (not cumulative; otherwise it
becomes a flat ≥400 ms after the first 10 s window saturates and
loses fairness signal). Reset to 0 immediately after emit.

`active_count`, `blacklist_size`, `last_clamp_reason`: **instantaneous
read at emit-time** from the live `sx1276_fhss` state.

### Q5. Cadence trigger source

**Resolution: main-loop poll, file-static last-emit timestamp.**

```c
/* In main super-loop, near the existing host_cmd_poll() / sx1276_*_poll() */
static uint32_t s_rfco_summary_last_emit_ms = 0U;
const uint32_t now = platform_now_ms();
if ((now - s_rfco_summary_last_emit_ms) >= HOST_RFCO_SUMMARY_PERIOD_MS) {
    host_rfco_summary_emit(s_rfco_summary_seq++, now);
    s_rfco_summary_last_emit_ms = now;
}
```

`HOST_RFCO_SUMMARY_PERIOD_MS = 60000U`. First emit is at boot+60 000 ms
(not boot+0); the first window's delta is therefore well-defined as
"counters since boot, capped at 1 min". Wrap-around safe via the
canonical `(now - last_emit_ms) >= period` idiom (uses `uint32_t`
subtraction modular arithmetic — same idiom as
[lora_ping.c L386](../DESIGN-CONTROLLER/firmware/murata_l072/lora_ping.c#L386)).

Drift acceptance: ±1 main-loop iteration (well under 60 s); B1-SUMMARY
does not advertise a wall-clock-accurate cadence — bench post-
processing must use the included `uptime_ms` field, not assume
exactly-60 000 ms spacing.

### Q6. Single-frame vs multi-frame payload

**Resolution: single frame.** Final layout (§3 below) is **191 B**,
inside the 311 B budget with 120 B headroom. Multi-frame would require
a reassembly contract on the X8 host side and is unjustified at this
budget.

If a future schema-ver bump pushes past 311 B, the precedent is to
*split into two URC types* (e.g. `RFCO_SUMMARY_HIST` for the
histograms and `RFCO_SUMMARY_STATE` for the scalars), each
schema-versioned independently. Multi-frame reassembly remains out of
scope.

### Q7. `rfco_source_schema_ver` field — which schema does it pin?

**Resolution: pin the per-TX RFCO schema** that fed the snapshot
builder. Field name: `pertx_schema_ver_at_emit`, byte width 1.
Rationale: the SUMMARY URC's *own* schema_ver pins the payload layout
(byte 0); the source pin tells bench analysis "the deltas in this
SUMMARY were aggregated from per-TX URCs of schema X" so a mid-run
schema-bump of the PERTX URC is detectable as a SUMMARY discontinuity.
Today both schemas are at version 1; the field is forward-looking.

**This also unblocks FCC-B2-b** (artifact stamping): `rfco_schema_ver`
in the artifact header takes the SUMMARY URC's value at the START of
the run (the first SUMMARY emit's `schema_ver` byte), not a build-time
constant. The orchestrator script must capture the first SUMMARY URC
and write it into the artifact header before any subsequent SUMMARY is
processed.

### Q8. Unused/inactive channels (active_count < 50)?

**Resolution: emit all 50 slots unconditionally.** Per Q4 deltas, an
inactive channel emits 0 hops / 0 dwell, which is the correct signal
("we did not transmit on this channel in this minute"). Blacklisted
channels also emit 0; `blacklist_size` already tells parsers how many
channels are blacklisted; the per-channel histogram does not need
in-band blacklist annotation. Future schema-bump can add a 50-bit
blacklist bitmap (7 bytes) if D-Gate analysis demands it.

## 3. Final payload layout (HOST_RFCO_SUMMARY_SCHEMA_VER = 1)

All little-endian. Total = **191 bytes** (well under 311 B limit).

```
off  size  field                                  notes
---- ----  -------------------------------------  ---------------------------------
  0    1   schema_ver                             == HOST_RFCO_SUMMARY_SCHEMA_VER
  1    1   pertx_schema_ver_at_emit               == HOST_RFCO_PERTX_SCHEMA_VER at emit
  2    1   profile_id                             REG_PROFILE_* enum
  3    1   active_count                           sx1276_fhss_active_count()  (instantaneous)
  4    1   blacklist_size                         50 - active_count           (instantaneous)
  5    1   last_clamp_reason                      host_rfco_tx_status_t of most-recent
                                                  non-OK tx; 0xFF = "none in this window"
  6    2   _reserved_align                        must be 0; pads to 4 B alignment for histogram
  8    4   uptime_ms_le                           platform_now_ms() at emit
 12    4   summary_seq_le                         monotonic, supplied by caller
 16    4   window_elapsed_ms_le                   now_ms - last_emit_ms (≈ 60_000; not pinned)
 20   50   per_channel_hop_count[50]              u8 each; 0xFF = saturated (>255 hops/min)
 70  100   per_channel_dwell_max_ms[50]           u16 each; ms-resolution peak in this window
170   16   blocked_attempts_by_reason[8]          u16 each; indexed by host_rfco_tx_status_t enum
186    1   pertx_count_in_window                  u8 saturated; how many PERTX URCs fed deltas
187    1   summary_emit_count                     u8 saturated; this URC's index since boot
188    1   _flags                                 bit0=first_summary_since_boot, bits 1-7 reserved
189    2   payload_crc16_le                       CRC-16/CCITT-FALSE over bytes [0..188]; this
                                                  is the URC-internal CRC, NOT the host-frame CRC
                                                  (which adds another 2 B at frame layer).
                                                  Catches in-flight bit-flip of the snapshot
                                                  builder while it walks 191 B of fresh-read state.
191   --   (end of payload)
```

**Static asserts (compile-time pins):**

```c
_Static_assert(HOST_RFCO_SUMMARY_PAYLOAD_LEN == 191U,
               "B1-SUMMARY wire layout drift");
_Static_assert(SX1276_FHSS_CHANNEL_COUNT == 50U,
               "B1-SUMMARY layout assumes 50 active channels");
_Static_assert(HOST_RFCO_SUMMARY_PAYLOAD_LEN <= HOST_PAYLOAD_MAX_LEN,
               "B1-SUMMARY payload exceeds URC frame limit");
```

## 4. Counter TU sources for the snapshot builder

The snapshot builder is HW-free policy — same pattern as
`sx1276_rx_scan_fail_eval()` / `host_rfco_pertx_pack()`. It reads from:

| Field | Source TU | Function called |
|---|---|---|
| `per_channel_hop_count[idx]` | `sx1276_fhss` or new bookkeeping in `sx1276_tx.c` | TBD: B1-SUMMARY-b will add `sx1276_fhss_hop_count_snapshot(uint8_t out[50])` if no caller-side counter exists; otherwise read existing tx-OK counter |
| `per_channel_dwell_max_ms[idx]` | new sidecar in `sx1276_legal_dwell.c` | TBD: B1-SUMMARY-b adds `sx1276_legal_dwell_peak_us_snapshot_and_clear(uint32_t out[50])` |
| `blocked_attempts_by_reason[i]` | per-TX RFCO emission path in `sx1276_tx.c` | TBD: B1-SUMMARY-b adds an 8-slot counter array updated alongside each `host_rfco_pertx_emit()` |
| `active_count` | `sx1276_fhss` | [`sx1276_fhss_active_count()`](../DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_fhss.h#L178) — already exists |
| `blacklist_size` | derived | `SX1276_FHSS_CHANNEL_COUNT - active_count` |
| `last_clamp_reason` | per-TX RFCO emission path | TBD: B1-SUMMARY-b stores the last non-OK `host_rfco_tx_status_t` seen since last summary |
| `pertx_count_in_window` | per-TX RFCO emission path | TBD: B1-SUMMARY-b increments on every `host_rfco_pertx_emit()` |

Three new sidecar TUs land in B1-SUMMARY-b alongside the pack helper.
None of them require new HW touchpoints; all are pure RAM bookkeeping
attached to existing emit/commit paths.

## 5. Decomposition into atomic sub-increments

Mirrors the FCC-A6c-3 a/b/c pattern.

* **B1-SUMMARY-a (declaration-only).** New file
  `include/host_rfco_summary.h` with: `HOST_RFCO_SUMMARY_SCHEMA_VER`,
  `HOST_RFCO_SUMMARY_PAYLOAD_LEN`, `HOST_RFCO_SUMMARY_PERIOD_MS`,
  `host_rfco_summary_t` payload struct **documenting every byte
  offset**. Add `HOST_TYPE_RFCO_SUMMARY_URC = 0xC4U` to
  `include/host_types.h` with comment block citing this analysis doc.
  No emitter, no pack helper, no main-loop integration. `mingw32-make
  check` should remain green (declaration-only).

* **B1-SUMMARY-b (pure pack helper + 3 sidecar counter TUs).** Add
  `host/host_rfco_summary.c` with `host_rfco_summary_pack(const
  host_rfco_summary_t *snap, uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN])`
  pure helper. Add three sidecar counter TUs as listed in §4. Add
  `bench/host_proto/rfco_summary.c` with ≥10 byte-by-byte wire-vector
  cases (schema_ver pin, little-endian encoding, hop-count saturation,
  dwell-max saturation, blocked-attempts indexing-by-enum,
  CRC-16/CCITT-FALSE matches a known-answer test vector, NULL-safe,
  `_reserved_align` must-be-zero, payload_len exact).

* **B1-SUMMARY-c (emit wrapper + cadence integration).** Add
  `host_rfco_summary_emit(uint32_t seq, uint32_t now_ms)` calling
  `host_uart_send_urc(HOST_TYPE_RFCO_SUMMARY_URC, ...)`. Wire main-loop
  cadence per §Q5. Bench test: stub `host_uart_send_urc` and confirm
  emit timing logic dispatches at ≥60 s and not before. Update
  `tools/lint_artifact_naming.py` is **not** required (the new payload
  field names all use the canonical `_us_1s`/`_us_10s` family or
  unambiguous `_ms` units).

## 6. Out of scope (deferred to later increments)

* Multi-channel SUMMARY split (Q6 fallback path).
* 50-bit blacklist bitmap (Q8 future-additive).
* SUMMARY-driven base-station auto-blacklist response.
* Time-aligned SUMMARY across the FHSS epoch boundary (current design
  is wall-clock 60 s, not epoch-aligned).

## 7. Ratification checklist (before B1-SUMMARY-a code lands)

- [ ] User confirms 191 B payload size is acceptable.
- [ ] User confirms snapshot-and-reset (not rolling) semantics.
- [ ] User confirms main-loop polling cadence is acceptable (no
      dedicated HAL timer).
- [ ] User confirms `0xC4` URC type-code allocation.
- [ ] User confirms ratifying this analysis is sufficient to proceed
      autonomously through B1-SUMMARY-a/b/c.

Signed: GitHub Copilot
Date: 2026-05-19
