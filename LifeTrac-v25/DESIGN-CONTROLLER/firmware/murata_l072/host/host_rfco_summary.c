/*
 * FCC-B1-SUMMARY-b-2 (plan §14.2 step 11): pure pack helper for the
 * per-minute RFCO_SUMMARY URC.
 *
 * Wire layout pinned by include/host_rfco_summary.h (191 bytes total,
 * little-endian, CRC-16/CCITT-FALSE over bytes [0..188]).
 *
 * Split rationale (mirrors host_rfco.c per-TX precedent): pack() is
 * pure C with no HAL dependencies so it can be exhaustively exercised
 * against golden byte vectors by the host toolchain
 * (bench/host_proto/rfco_summary.c). emit() lands in B1-SUMMARY-c as
 * a thin wrapper that builds a host_rfco_summary_t from the live
 * snapshot-and-clear sources and forwards to host_uart_send_urc().
 *
 * Invariants enforced here:
 *   - byte 0 is HARD-CODED to HOST_RFCO_SUMMARY_SCHEMA_VER. The
 *     struct field `schema_ver` is for documentation symmetry only;
 *     bench parsers MUST refuse unfamiliar schema_ver rather than
 *     mis-decoding the tail (see host_rfco_summary.h §preamble).
 *   - bytes 6..7 (_reserved_align) are HARD-CODED to 0x00, regardless
 *     of the struct field value. This actively suppresses uninitialized-
 *     struct leaks; any future use of those 2 bytes MUST bump
 *     HOST_RFCO_SUMMARY_SCHEMA_VER.
 *   - the CRC at bytes 189..190 is CRC-16/CCITT-FALSE
 *     (poly 0x1021, init 0xFFFF, no reflection, xorout 0x0000)
 *     computed over the 189 preceding bytes.
 */

#include "host_rfco_summary.h"

#include <stddef.h>

static void put_u16_le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v        & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void put_u32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v        & 0xFFU);
    p[1] = (uint8_t)((v >>  8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect, xorout 0. */
static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t b = 0; b < 8; ++b) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

bool host_rfco_summary_pack(const host_rfco_summary_t *in,
                            uint8_t out[HOST_RFCO_SUMMARY_PAYLOAD_LEN]) {
    if ((in == NULL) || (out == NULL)) {
        return false;
    }

    /* Header bytes 0..7. schema_ver hard-pinned; _reserved_align
     * hard-zeroed to suppress uninit-struct leaks. */
    out[HOST_RFCO_SUMMARY_OFF_SCHEMA_VER]        = HOST_RFCO_SUMMARY_SCHEMA_VER;
    out[HOST_RFCO_SUMMARY_OFF_PERTX_SCHEMA_VER]  = in->pertx_schema_ver_at_emit;
    out[HOST_RFCO_SUMMARY_OFF_PROFILE_ID]        = in->profile_id;
    out[HOST_RFCO_SUMMARY_OFF_ACTIVE_COUNT]      = in->active_count;
    out[HOST_RFCO_SUMMARY_OFF_BLACKLIST_SIZE]    = in->blacklist_size;
    out[HOST_RFCO_SUMMARY_OFF_LAST_CLAMP_REASON] = in->last_clamp_reason;
    out[HOST_RFCO_SUMMARY_OFF_RESERVED_ALIGN]      = 0x00U;
    out[HOST_RFCO_SUMMARY_OFF_RESERVED_ALIGN + 1U] = 0x00U;

    /* u32 LE block: uptime / seq / window_elapsed. */
    put_u32_le(&out[HOST_RFCO_SUMMARY_OFF_UPTIME_MS],         in->uptime_ms);
    put_u32_le(&out[HOST_RFCO_SUMMARY_OFF_SUMMARY_SEQ],       in->summary_seq);
    put_u32_le(&out[HOST_RFCO_SUMMARY_OFF_WINDOW_ELAPSED_MS], in->window_elapsed_ms);

    /* per_channel_hop_count[50]: u8 each. Struct already pre-saturates
     * to 0xFFU at sx1276_fhss_hop_count_snapshot_and_clear() time. */
    for (uint8_t i = 0; i < HOST_RFCO_SUMMARY_CHANNEL_COUNT; ++i) {
        out[HOST_RFCO_SUMMARY_OFF_PER_CHANNEL_HOP_COUNT + i] =
            in->per_channel_hop_count[i];
    }

    /* per_channel_dwell_max_ms[50]: u16 LE each. The struct field is
     * already in ms (us->ms conversion happens in the snapshot consumer
     * before populating the struct, so the wire field stays a pure
     * serialize). */
    for (uint8_t i = 0; i < HOST_RFCO_SUMMARY_CHANNEL_COUNT; ++i) {
        put_u16_le(&out[HOST_RFCO_SUMMARY_OFF_PER_CHANNEL_DWELL_MAX + (uint16_t)(i * 2U)],
                   in->per_channel_dwell_max_ms[i]);
    }

    /* blocked_attempts_by_reason[8]: u16 LE each. */
    for (uint8_t i = 0; i < HOST_RFCO_SUMMARY_REASON_SLOTS; ++i) {
        put_u16_le(&out[HOST_RFCO_SUMMARY_OFF_BLOCKED_ATTEMPTS + (uint16_t)(i * 2U)],
                   in->blocked_attempts_by_reason[i]);
    }

    /* Tail bytes 186..188. */
    out[HOST_RFCO_SUMMARY_OFF_PERTX_COUNT_IN_WINDOW] = in->pertx_count_in_window;
    out[HOST_RFCO_SUMMARY_OFF_SUMMARY_EMIT_COUNT]    = in->summary_emit_count;
    out[HOST_RFCO_SUMMARY_OFF_FLAGS]                 = in->flags;

    /* CRC over bytes [0..188]. */
    const uint16_t crc = crc16_ccitt_false(out, HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16);
    put_u16_le(&out[HOST_RFCO_SUMMARY_OFF_PAYLOAD_CRC16], crc);

    return true;
}

