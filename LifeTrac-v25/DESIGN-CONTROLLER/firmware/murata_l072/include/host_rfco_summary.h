#ifndef LIFETRAC_MURATA_L072_HOST_RFCO_SUMMARY_H
#define LIFETRAC_MURATA_L072_HOST_RFCO_SUMMARY_H

/*
 * Per-minute RFCO_SUMMARY URC (FCC-B1-SUMMARY)
 * --------------------------------------------
 *
 * Plan / design ref:
 *   AI NOTES/2026-05-19_FCC_B1_SUMMARY_Wire_Layout_Design_Copilot_v1_0.md
 *   (§3 final payload layout, §4 counter TU sources, §5 a/b/c
 *    decomposition).
 *
 * Cross-refs to the existing per-TX RFCO URC (FCC-B1-PERTX):
 *   - include/host_rfco.h           (per-TX schema + tx-status enum
 *                                   shared as the indexing key for
 *                                   blocked_attempts_by_reason[])
 *   - include/host_types.h          (URC type-code allocation:
 *                                   0xC3 = PERTX, 0xC4 = SUMMARY)
 *
 * This URC is the HEAVY snapshot. It complements the LIGHT per-TX
 * RFCO_PERTX URC (one per TX result) by carrying:
 *
 *   - per_channel_hop_count[50]            u8 saturated
 *   - per_channel_dwell_max_ms[50]         u16 ms-resolution peak
 *   - blocked_attempts_by_reason[8]        u16 indexed by
 *                                          host_rfco_tx_status_t
 *   - active_count / blacklist_size        instantaneous
 *   - last_clamp_reason                    most-recent non-OK tx status
 *   - uptime_ms / summary_seq / window_elapsed_ms
 *   - pertx_count_in_window / summary_emit_count / _flags
 *   - payload_crc16 (CRC-16/CCITT-FALSE)   URC-internal integrity
 *
 * Cadence: emitted at HOST_RFCO_SUMMARY_PERIOD_MS (60 000 ms) from
 * the main loop, polled against platform_now_ms() — no new HAL timer.
 * Snapshot-and-reset semantics: histogram + peak-dwell counters are
 * sampled then cleared at emit; instantaneous fields (active_count,
 * blacklist_size, last_clamp_reason) are read live.
 *
 * Wire layout: see §3 of the design doc, reproduced inline below
 * with byte offsets. All multi-byte fields are little-endian.
 *
 *   off  size  field
 *   ---- ----  -----------------------------------------------------
 *     0   1    schema_ver                  (== HOST_RFCO_SUMMARY_SCHEMA_VER)
 *     1   1    pertx_schema_ver_at_emit    (== HOST_RFCO_PERTX_SCHEMA_VER at emit)
 *     2   1    profile_id                  (REG_PROFILE_* enum from host_cfg_keys.h)
 *     3   1    active_count                (sx1276_fhss_active_count(), 0..50)
 *     4   1    blacklist_size              (50 - active_count)
 *     5   1    last_clamp_reason           (host_rfco_tx_status_t of most-recent
 *                                          non-OK TX; 0xFF = "none in window")
 *     6   2    _reserved_align             (must be 0; aligns histogram to 4 B)
 *     8   4    uptime_ms_le                (platform_now_ms() at emit)
 *    12   4    summary_seq_le              (monotonic, caller-supplied)
 *    16   4    window_elapsed_ms_le        (now_ms - last_emit_ms, ~60_000)
 *    20  50    per_channel_hop_count[50]   (u8 each; 0xFF = saturated >255/min)
 *    70 100    per_channel_dwell_max_ms[50] (u16 each; ms-resolution peak)
 *   170  16    blocked_attempts_by_reason[8] (u16 each; indexed by
 *                                            host_rfco_tx_status_t)
 *   186   1    pertx_count_in_window       (u8 saturated; PERTX URCs since
 *                                          last SUMMARY emit)
 *   187   1    summary_emit_count          (u8 saturated; index since boot)
 *   188   1    _flags                      (bit0 = first_summary_since_boot;
 *                                          bits 1..7 reserved, must be 0)
 *   189   2    payload_crc16_le            (CRC-16/CCITT-FALSE over [0..188];
 *                                          URC-internal, NOT the host-frame CRC)
 *   191  --   (end of payload)
 *
 * Bench parsers MUST refuse unfamiliar schema_ver rather than silently
 * mis-decoding the tail (matches the PERTX rule documented in
 * host_rfco.h L23-L27).
 *
 * This header is DECLARATION-ONLY: no pack helper, no emit wrapper,
 * no main-loop integration. Those land as FCC-B1-SUMMARY-b and
 * FCC-B1-SUMMARY-c respectively (see TODO.md).
 */

#include <stdint.h>

#include "config.h"
#include "host_rfco.h"
#include "host_uart.h"
#include "sx1276_fhss.h"

/* Schema + layout pins. Bump SCHEMA_VER on any payload change. */
#define HOST_RFCO_SUMMARY_SCHEMA_VER         1U
#define HOST_RFCO_SUMMARY_PAYLOAD_LEN        191U

/* Emit cadence. Polled from the main loop against platform_now_ms(). */
#define HOST_RFCO_SUMMARY_PERIOD_MS          60000U

/* Channel count + per-reason counter slot count (mirrors the §3 layout). */
#define HOST_RFCO_SUMMARY_CHANNEL_COUNT      50U
#define HOST_RFCO_SUMMARY_REASON_SLOTS       8U

/* Field byte offsets — kept here so the pack helper and bench
 * wire-vector tests share one source of truth. */
#define HOST_RFCO_SUMMARY_OFF_SCHEMA_VER             0U
#define HOST_RFCO_SUMMARY_OFF_PERTX_SCHEMA_VER       1U
#define HOST_RFCO_SUMMARY_OFF_PROFILE_ID             2U
#define HOST_RFCO_SUMMARY_OFF_ACTIVE_COUNT           3U
#define HOST_RFCO_SUMMARY_OFF_BLACKLIST_SIZE         4U
#define HOST_RFCO_SUMMARY_OFF_LAST_CLAMP_REASON      5U
#define HOST_RFCO_SUMMARY_OFF_RESERVED_ALIGN         6U
#define HOST_RFCO_SUMMARY_OFF_UPTIME_MS              8U
#define HOST_RFCO_SUMMARY_OFF_SUMMARY_SEQ            12U
#define HOST_RFCO_SUMMARY_OFF_WINDOW_ELAPSED_MS      16U
#define HOST_RFCO_SUMMARY_OFF_PER_CHANNEL_HOP_COUNT  20U
#define HOST_RFCO_SUMMARY_OFF_PER_CHANNEL_DWELL_MAX  70U
#define HOST_RFCO_SUMMARY_OFF_BLOCKED_ATTEMPTS       170U
#define HOST_RFCO_SUMMARY_OFF_PERTX_COUNT_IN_WINDOW  186U
#define HOST_RFCO_SUMMARY_OFF_SUMMARY_EMIT_COUNT     187U
#define HOST_RFCO_SUMMARY_OFF_FLAGS                  188U
#define HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16          189U

/* Sentinel + flag bit definitions. */
#define HOST_RFCO_SUMMARY_HOP_COUNT_SATURATED        0xFFU
#define HOST_RFCO_SUMMARY_LAST_CLAMP_REASON_NONE     0xFFU
#define HOST_RFCO_SUMMARY_FLAG_FIRST_SINCE_BOOT      (1U << 0)

/*
 * Snapshot struct for the pack helper (FCC-B1-SUMMARY-b will add the
 * pack helper; this struct documents the field set the helper will
 * consume). Sizes mirror the wire layout exactly so the helper can
 * be a straight serialize, but the struct is NOT memcpy-able to the
 * wire (multi-byte fields are host-endian here and serialized LE on
 * the wire).
 */
typedef struct host_rfco_summary_s {
    uint8_t  schema_ver;
    uint8_t  pertx_schema_ver_at_emit;
    uint8_t  profile_id;
    uint8_t  active_count;
    uint8_t  blacklist_size;
    uint8_t  last_clamp_reason;
    uint16_t _reserved_align;
    uint32_t uptime_ms;
    uint32_t summary_seq;
    uint32_t window_elapsed_ms;
    uint8_t  per_channel_hop_count[HOST_RFCO_SUMMARY_CHANNEL_COUNT];
    uint16_t per_channel_dwell_max_ms[HOST_RFCO_SUMMARY_CHANNEL_COUNT];
    uint16_t blocked_attempts_by_reason[HOST_RFCO_SUMMARY_REASON_SLOTS];
    uint8_t  pertx_count_in_window;
    uint8_t  summary_emit_count;
    uint8_t  flags;
    /* payload_crc16 is computed by the pack helper, not stored here. */
} host_rfco_summary_t;

/* Compile-time pins per §3 of the design doc. */
_Static_assert(HOST_RFCO_SUMMARY_PAYLOAD_LEN == 191U,
               "B1-SUMMARY wire layout drift: payload_len must stay 191");
_Static_assert(SX1276_FHSS_CHANNEL_COUNT == HOST_RFCO_SUMMARY_CHANNEL_COUNT,
               "B1-SUMMARY layout assumes 50 active channels");
_Static_assert(HOST_RFCO_SUMMARY_PAYLOAD_LEN <= HOST_PAYLOAD_MAX_LEN,
               "B1-SUMMARY payload exceeds URC frame limit");
_Static_assert(HOST_RFCO_SUMMARY_REASON_SLOTS >= 7U,
               "blocked_attempts_by_reason[] must cover all host_rfco_tx_status_t");

#include <stdbool.h>

/*
 * FCC-B1-SUMMARY-b-2 pure pack helper. Serializes a host-endian
 * snapshot into the 191-byte little-endian wire payload. The schema
 * byte (offset 0) is hard-pinned to HOST_RFCO_SUMMARY_SCHEMA_VER and
 * the _reserved_align bytes (offsets 6..7) are hard-zeroed regardless
 * of the struct field values; the trailing CRC (offsets 189..190) is
 * CRC-16/CCITT-FALSE over bytes [0..188]. NULL inputs are rejected.
 * Returns true on success, false on NULL.
 *
 * No HAL dependencies — exercised against golden vectors by
 * bench/host_proto/rfco_summary.c. */
bool host_rfco_summary_pack(const host_rfco_summary_t *in,
                            uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN]);

/*
 * FCC-B1-SUMMARY-c-1 emit wrapper. Builds the snapshot from the live
 * sidecar sources, calls host_rfco_summary_pack(), then forwards the
 * 191-byte payload to host_uart_send_urc() as a
 * HOST_TYPE_RFCO_SUMMARY_URC (0xC4).
 *
 * Caller responsibilities (kept out of the wrapper so it stays pure
 * of HAL/cfg dependencies):
 *   - seq             : monotonic SUMMARY sequence number (caller-owned).
 *   - now_ms          : platform_now_ms() at the emit instant; doubles
 *                       as uptime_ms_le on the wire.
 *   - window_elapsed_ms: now_ms - last_emit_ms (caller-tracked).
 *   - profile_id      : REG_PROFILE_* enum of the active profile.
 *
 * Wrapper-managed state (drained at emit):
 *   - FIRST_SINCE_BOOT flag (set on first call, cleared after).
 *   - summary_emit_count (u8 saturating).
 *   - per-channel hop_count, dwell_peak_us, blocked_attempts,
 *     pertx_count_in_window, last_clamp_reason — all
 *     snapshot_and_clear'd.
 *
 * Dwell peak conversion: sidecar tracks µs; wire field is u16 ms.
 * Ceiling-division (µs + 999) / 1000 — under-reporting a peak would
 * mask compliance margin. Saturates to 0xFFFF on overflow.
 *
 * Channel-count truncation: legal_dwell tracks 64 channels but
 * SUMMARY only carries 50. The first SX1276_FHSS_CHANNEL_COUNT
 * entries are copied; tail entries are dropped (the dwell module's
 * 50-channel sub-range is the same indexing used by FHSS).
 *
 * Returns true on a successful send_urc forward; false if pack()
 * fails (should be unreachable — sidecars produce a valid snapshot).
 */
bool host_rfco_summary_emit(uint32_t seq,
                            uint32_t now_ms,
                            uint32_t window_elapsed_ms,
                            uint8_t  profile_id);

/* Pure cadence helper: returns true iff (now_ms - last_emit_ms) has
 * reached HOST_RFCO_SUMMARY_PERIOD_MS. u32 subtraction wraps
 * correctly across platform_now_ms() rollover. */
bool host_rfco_summary_should_emit(uint32_t now_ms, uint32_t last_emit_ms);

/* FCC-B1-SUMMARY-c-2: main-loop integration helper.
 *
 * Owns the cadence state (last_emit_ms + summary_seq) internally so
 * the caller (main.c) only has to invoke this once per loop tick:
 *
 *     (void)host_rfco_summary_tick(platform_now_ms(),
 *                                   host_cfg_profile_active()->profile_id);
 *
 * Behaviour:
 *   - On first call after boot or wrapper-state reset, last_emit_ms
 *     is 0, so the first SUMMARY fires only when now_ms reaches
 *     HOST_RFCO_SUMMARY_PERIOD_MS (60 000 ms). The first emit
 *     carries the FIRST_SINCE_BOOT flag (driven by the underlying
 *     host_rfco_summary_emit()).
 *   - On a successful emit, last_emit_ms advances to now_ms and the
 *     internal seq increments. window_elapsed_ms passed downstream
 *     is (now_ms - last_emit_ms_before_advance), so jitter / late
 *     ticks are reported accurately.
 *   - If host_rfco_summary_emit() returns false (pack failure —
 *     unreachable in practice), last_emit_ms and seq are NOT
 *     advanced so the next tick retries with the same window.
 *
 * Returns true iff an emit was forwarded to host_uart_send_urc().
 */
bool host_rfco_summary_tick(uint32_t now_ms, uint8_t profile_id);

/* Test-only: resets the FIRST_SINCE_BOOT latch, summary_emit_count,
 * cadence last_emit_ms, and cadence seq. NOT called by production
 * code. */
void host_rfco_summary_reset_wrapper_state(void);

#endif /* LIFETRAC_MURATA_L072_HOST_RFCO_SUMMARY_H */
