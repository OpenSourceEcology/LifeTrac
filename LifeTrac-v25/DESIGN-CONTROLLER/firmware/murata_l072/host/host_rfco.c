/*
 * Per-TX RFCO URC pack + emit (plan §14.2 step 7, FCC-B1-PERTX).
 * See include/host_rfco.h for full payload semantics.
 *
 * Split rationale: pack() is pure C with no HAL dependencies so it can
 * be exercised against golden vectors by the host toolchain
 * (bench/host_proto/rfco_pertx.c). emit() is a thin wrapper that calls
 * host_uart_send_urc(); the host test stubs host_uart_send_urc and
 * checks that emit forwards the correct (type, payload, len) triple.
 */

#include "host_rfco.h"

#include <stddef.h>

#include "host_types.h"
#include "host_uart.h"
#include "sx1276_fhss.h"  /* sx1276_fhss_record_hop() for SUMMARY sidecar */

static void put_u32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v        & 0xFFU);
    p[1] = (uint8_t)((v >>  8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

bool host_rfco_pertx_pack(const host_rfco_pertx_t *in, uint8_t *out) {
    if ((in == NULL) || (out == NULL)) {
        return false;
    }
    /* Offsets match the table in host_rfco.h — any change here must be
     * matched in the header doc AND in the schema_ver bump. */
    out[0] = HOST_RFCO_PERTX_SCHEMA_VER;
    out[1] = in->profile_id;
    out[2] = in->tx_status;
    out[3] = in->hop_idx;
    out[4] = in->channel_idx;
    put_u32_le(&out[5],  in->epoch);
    put_u32_le(&out[9],  in->freq_hz);
    put_u32_le(&out[13], in->pkt_toa_us);
    put_u32_le(&out[17], in->legal_dwell_used_us_10s);
    return true;
}

bool host_rfco_pertx_emit(uint16_t seq,
                          uint8_t flags,
                          const host_rfco_pertx_t *snapshot) {
    if (snapshot == NULL) {
        return false;
    }
    uint8_t payload[HOST_RFCO_PERTX_PAYLOAD_LEN];
    if (!host_rfco_pertx_pack(snapshot, payload)) {
        return false;
    }
    /* FCC-B1-SUMMARY-b-3: feed the per-minute SUMMARY URC sidecars at
     * the per-TX emit boundary. record_blocked() handles the OK skip
     * internally (slot 0 is reserved); record_hop() is conditioned on
     * a successful TX so blocked attempts do not inflate the
     * per-channel hop histogram. record_hop is OOR-safe so an
     * out-of-range hop_idx is a no-op.
     *
     * FCC-B1-SUMMARY-c-1: also feed the per-window pertx_count +
     * last_clamp_reason sidecars. window_record() saturates the count
     * and only updates last_clamp on non-OK statuses. */
    host_rfco_blocked_attempts_record(
        (host_rfco_tx_status_t)snapshot->tx_status);
    if (snapshot->tx_status == (uint8_t)HOST_RFCO_TX_STATUS_OK) {
        sx1276_fhss_record_hop(snapshot->hop_idx);
    }
    host_rfco_pertx_window_record(snapshot->tx_status);
    host_uart_send_urc(HOST_TYPE_RFCO_PERTX_URC,
                       seq,
                       flags,
                       payload,
                       (uint16_t)HOST_RFCO_PERTX_PAYLOAD_LEN);
    return true;
}

/* FCC-B1-SUMMARY-b sidecar: blocked-attempts-by-reason counter.
 * Slot mapping is documented in host_rfco.h. */
static uint16_t s_blocked_attempts[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS];

void host_rfco_blocked_attempts_reset(void) {
    for (uint8_t i = 0; i < HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS; ++i) {
        s_blocked_attempts[i] = 0U;
    }
}

void host_rfco_blocked_attempts_record(host_rfco_tx_status_t reason) {
    uint8_t slot;
    if (reason == HOST_RFCO_TX_STATUS_OK) {
        /* OK is not a blocked outcome — slot 0 reserved. */
        return;
    } else if (reason == HOST_RFCO_TX_STATUS_INTERNAL) {
        slot = 7U;
    } else if (((uint8_t)reason) >= 1U && ((uint8_t)reason) <= 6U) {
        slot = (uint8_t)reason;
    } else {
        /* Defensive: unallocated enum value, drop. Schema bump is the
         * correct response to an enum addition. */
        return;
    }
    if (s_blocked_attempts[slot] != UINT16_MAX) {
        ++s_blocked_attempts[slot];
    }
}

void host_rfco_blocked_attempts_snapshot_and_clear(
    uint16_t out[HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS]) {
    if (out == NULL) {
        /* Preserve counters: a caller bug must not silently lose a
         * minute of compliance evidence. */
        return;
    }
    for (uint8_t i = 0; i < HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS; ++i) {
        out[i] = s_blocked_attempts[i];
        s_blocked_attempts[i] = 0U;
    }
}

/* FCC-B1-SUMMARY-c-1: per-window pertx_count + last_clamp_reason.
 * Sentinel 0xFFU mirrors HOST_RFCO_SUMMARY_LAST_CLAMP_REASON_NONE
 * from host_rfco_summary.h (kept as a literal here to avoid pulling
 * the SUMMARY header into the per-TX module). */
#define HOST_RFCO_LAST_CLAMP_NONE  0xFFU

static uint16_t s_pertx_count_in_window;
static uint8_t  s_last_clamp_reason = HOST_RFCO_LAST_CLAMP_NONE;

void host_rfco_pertx_window_record(uint8_t tx_status) {
    if (s_pertx_count_in_window != UINT16_MAX) {
        ++s_pertx_count_in_window;
    }
    if (tx_status != (uint8_t)HOST_RFCO_TX_STATUS_OK) {
        s_last_clamp_reason = tx_status;
    }
}

uint8_t host_rfco_pertx_count_in_window_snapshot_and_clear(void) {
    const uint16_t v = s_pertx_count_in_window;
    s_pertx_count_in_window = 0U;
    return (v > 0xFFU) ? 0xFFU : (uint8_t)v;
}

uint8_t host_rfco_last_clamp_reason_snapshot_and_clear(void) {
    const uint8_t v = s_last_clamp_reason;
    s_last_clamp_reason = HOST_RFCO_LAST_CLAMP_NONE;
    return v;
}

void host_rfco_pertx_window_reset(void) {
    s_pertx_count_in_window = 0U;
    s_last_clamp_reason = HOST_RFCO_LAST_CLAMP_NONE;
}
