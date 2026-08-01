/*
 * F8 (2026-07-30): pure RX_FRAME_URC payload packer. See host_rx_wire.h
 * for the byte layout and the hdr_valid gate rationale. Kept free of
 * HW/UART calls so the check-rx-frame-urc bench target can pin the wire
 * bytes with host gcc.
 */
#include "host_rx_wire.h"

#include <string.h>

static void rxw_put_u16_le(uint8_t *out, uint16_t v) {
    out[0] = (uint8_t)(v & 0xFFU);
    out[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void rxw_put_u32_le(uint8_t *out, uint32_t v) {
    out[0] = (uint8_t)(v & 0xFFU);
    out[1] = (uint8_t)((v >> 8) & 0xFFU);
    out[2] = (uint8_t)((v >> 16) & 0xFFU);
    out[3] = (uint8_t)((v >> 24) & 0xFFU);
}

uint16_t host_rx_frame_urc_pack(const sx1276_rx_frame_t *f,
                                uint8_t *out, uint16_t out_cap) {
    uint16_t tail;

    if (f == NULL || out == NULL) {
        return 0U;
    }
    if ((uint16_t)(8U + (uint16_t)f->length + HOST_RX_FRAME_URC_TAIL_LEN)
            > out_cap) {
        return 0U;
    }

    out[0] = f->length;
    out[1] = (uint8_t)f->snr_db;
    rxw_put_u16_le(&out[2], (uint16_t)f->rssi_dbm);
    rxw_put_u32_le(&out[4], f->timestamp_us);
    if (f->length > 0U) {
        memcpy(&out[8], f->payload, f->length);
    }

    tail = (uint16_t)(8U + f->length);
#ifdef LIFETRAC_FHSS_TX_ROUTED
    if (f->hdr_valid) {
        out[tail + 0U] = HOST_RX_FRAME_URC_PHASE_VALID;
        out[tail + 1U] = f->hdr.profile_id;
        out[tail + 2U] = f->hdr.hop_idx;
        out[tail + 3U] = f->hdr.slot_offset_ms;
        rxw_put_u32_le(&out[tail + 4U], f->hdr.epoch);
        return (uint16_t)(tail + HOST_RX_FRAME_URC_TAIL_LEN);
    }
#endif
    /* Unrouted build, or routed with an invalid header: zero tail.
     * NEVER read f->hdr here — in unrouted builds it is uninitialized. */
    memset(&out[tail], 0, HOST_RX_FRAME_URC_TAIL_LEN);
    return (uint16_t)(tail + HOST_RX_FRAME_URC_TAIL_LEN);
}
