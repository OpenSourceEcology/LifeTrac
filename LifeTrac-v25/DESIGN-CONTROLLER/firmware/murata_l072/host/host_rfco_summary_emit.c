/*
 * FCC-B1-SUMMARY-c-1 (plan §14.2 step 11, sub-step c-1):
 * emit wrapper + cadence helper + per-window sidecar drain.
 *
 * Split rationale: kept separate from host_rfco_summary.c (which is
 * the pure pack helper from b-2) so the pack-only bench
 * (bench/host_proto/rfco_summary.c) does not have to link
 * host_uart_send_urc(), host_rfco.c, or the radio counter modules.
 * This file pulls those dependencies in for the c-1 emit bench
 * (bench/host_proto/rfco_summary_emit.c) and for the production
 * firmware link.
 *
 * No main-loop wiring is performed here — that lands as
 * FCC-B1-SUMMARY-c-2 (caller responsibilities: seq, now_ms,
 * window_elapsed_ms, profile_id, and last_emit_ms tracking).
 */

#include "host_rfco_summary.h"

#include "host_types.h"           /* HOST_TYPE_RFCO_SUMMARY_URC */
#include "host_uart.h"            /* host_uart_send_urc()       */
#include "sx1276_fhss.h"          /* hop_count_snapshot, active_count */
#include "sx1276_legal_dwell.h"   /* peak_us_snapshot           */

/* Wrapper-managed state. The FIRST_SINCE_BOOT latch flips false
 * after the first successful emit; summary_emit_count is u8
 * saturating per the wire schema. */
static bool     s_first_since_boot = true;
static uint8_t  s_summary_emit_count;

/* FCC-B1-SUMMARY-c-2 cadence state — owned by the tick helper so
 * main.c does not have to thread per-emit timing manually. */
static uint32_t s_tick_last_emit_ms;
static uint32_t s_tick_seq;

bool host_rfco_summary_should_emit(uint32_t now_ms, uint32_t last_emit_ms) {
    /* u32 subtraction is well-defined modulo 2^32 — works across
     * platform_now_ms() wraparound. */
    return ((uint32_t)(now_ms - last_emit_ms)) >= HOST_RFCO_SUMMARY_PERIOD_MS;
}

void host_rfco_summary_reset_wrapper_state(void) {
    s_first_since_boot   = true;
    s_summary_emit_count = 0U;
    s_tick_last_emit_ms  = 0U;
    s_tick_seq           = 0U;
}

/* µs → ms ceiling-div with u16 saturation. Under-reporting a peak
 * would mask compliance margin, so round UP and clamp to 0xFFFF. */
static uint16_t dwell_us_to_ms_sat(uint32_t us) {
    if (us == 0U) {
        return 0U;
    }
    const uint32_t ms = (us + 999U) / 1000U;
    return (ms > 0xFFFFU) ? 0xFFFFU : (uint16_t)ms;
}

bool host_rfco_summary_emit(uint32_t seq,
                            uint32_t now_ms,
                            uint32_t window_elapsed_ms,
                            uint8_t  profile_id) {
    host_rfco_summary_t snap;

    /* Header bytes (pack helper hard-pins schema_ver at byte 0 +
     * zeros _reserved_align; we still populate the struct fields for
     * documentation symmetry). */
    snap.schema_ver               = HOST_RFCO_SUMMARY_SCHEMA_VER;
    snap.pertx_schema_ver_at_emit = HOST_RFCO_PERTX_SCHEMA_VER;
    snap.profile_id               = profile_id;
    snap.active_count             = sx1276_fhss_active_count();
    /* 50-channel plan: blacklist_size = 50 - active_count. Clamp in
     * case active_count somehow exceeds the channel count (would be
     * a bug elsewhere, but never under-flow the u8). */
    snap.blacklist_size = (snap.active_count > HOST_RFCO_SUMMARY_CHANNEL_COUNT)
                              ? 0U
                              : (uint8_t)(HOST_RFCO_SUMMARY_CHANNEL_COUNT -
                                          snap.active_count);
    snap.last_clamp_reason  = host_rfco_last_clamp_reason_snapshot_and_clear();
    snap._reserved_align    = 0U;
    snap.uptime_ms          = now_ms;
    snap.summary_seq        = seq;
    snap.window_elapsed_ms  = window_elapsed_ms;

    /* per_channel_hop_count[50]: u8 already saturated to 0xFF by the
     * sidecar. */
    sx1276_fhss_hop_count_snapshot_and_clear(snap.per_channel_hop_count);

    /* per_channel_dwell_max_ms[50]: legal_dwell sidecar tracks µs over
     * 64 channels; the first 50 entries align with the FHSS channel
     * indexing. Convert µs → ms (ceiling, saturating). */
    uint32_t dwell_us[SX1276_DWELL_CHANNEL_COUNT];
    sx1276_legal_dwell_peak_us_snapshot_and_clear(dwell_us);
    for (uint8_t i = 0; i < HOST_RFCO_SUMMARY_CHANNEL_COUNT; ++i) {
        snap.per_channel_dwell_max_ms[i] = dwell_us_to_ms_sat(dwell_us[i]);
    }

    /* blocked_attempts_by_reason[8]: u16 already saturated by the
     * sidecar. */
    host_rfco_blocked_attempts_snapshot_and_clear(snap.blocked_attempts_by_reason);

    /* Per-window tail. */
    snap.pertx_count_in_window = host_rfco_pertx_count_in_window_snapshot_and_clear();
    snap.summary_emit_count    = s_summary_emit_count;
    snap.flags                 = s_first_since_boot
                                     ? (uint8_t)HOST_RFCO_SUMMARY_FLAG_FIRST_SINCE_BOOT
                                     : 0U;

    /* Pack + send. */
    uint8_t payload[HOST_RFCO_SUMMARY_PAYLOAD_LEN];
    if (!host_rfco_summary_pack(&snap, payload)) {
        /* Defensive: pack only returns false on NULL, which is
         * unreachable here. Leave wrapper state untouched so a
         * retry on the next tick would produce the same snapshot. */
        return false;
    }

    /* seq is u32 on the API but the UART URC frame seq is u16 — the
     * SUMMARY URC carries its own u32 summary_seq inside the
     * payload, so the frame-level seq is just the low 16 bits for
     * frame deduplication purposes. */
    host_uart_send_urc(HOST_TYPE_RFCO_SUMMARY_URC,
                       (uint16_t)(seq & 0xFFFFU),
                       snap.flags,
                       payload,
                       (uint16_t)HOST_RFCO_SUMMARY_PAYLOAD_LEN);

    /* Latch + counter advance happen AFTER the send so a pack failure
     * leaves the FIRST_SINCE_BOOT bit available for the retry. */
    s_first_since_boot = false;
    if (s_summary_emit_count != 0xFFU) {
        ++s_summary_emit_count;
    }
    return true;
}

bool host_rfco_summary_tick(uint32_t now_ms, uint8_t profile_id) {
    if (!host_rfco_summary_should_emit(now_ms, s_tick_last_emit_ms)) {
        return false;
    }
    /* window_elapsed_ms = wall-clock interval between last emit and
     * now. Using u32 wrap math so a SysTick rollover during a long
     * uptime still produces the correct delta. */
    const uint32_t window = (uint32_t)(now_ms - s_tick_last_emit_ms);
    if (!host_rfco_summary_emit(s_tick_seq, now_ms, window, profile_id)) {
        /* Preserve cadence + seq on transient failure so the next
         * tick retries with the accumulated window. */
        return false;
    }
    s_tick_last_emit_ms = now_ms;
    ++s_tick_seq;
    return true;
}
