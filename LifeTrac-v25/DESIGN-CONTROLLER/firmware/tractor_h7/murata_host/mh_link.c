#include "mh_link.h"

#if defined(LIFETRAC_USE_METHOD_G_HOST) && (LIFETRAC_USE_METHOD_G_HOST)

#include <stddef.h>
#include <string.h>

#define MH_LINK_RESERVED_TX_ID 0xFEU

static uint16_t read_u16_le(const uint8_t *src) {
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *src) {
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

bool mh_link_rx_parse(const murata_host_frame_t *frame, mh_link_rx_t *out) {
    uint8_t rx_len;

    if (frame == NULL || out == NULL ||
        frame->type != HOST_TYPE_RX_FRAME_URC ||
        frame->payload_len < 8U) {
        return false;
    }

    rx_len = frame->payload[0];
    if (frame->payload_len < (uint16_t)(8U + rx_len)) {
        return false;
    }

    out->len = rx_len;
    out->snr_db = (int8_t)frame->payload[1];
    out->rssi_dbm = (int16_t)read_u16_le(&frame->payload[2]);
    out->timestamp_us = read_u32_le(&frame->payload[4]);
    out->data = &frame->payload[8];
    return true;
}

void mh_link_gate_init(mh_link_gate_t *gate) {
    if (gate == NULL) {
        return;
    }
    memset(gate, 0, sizeof(*gate));
    gate->next_tx_id = 1U;
}

bool mh_link_gate_busy(const mh_link_gate_t *gate) {
    return gate != NULL && gate->in_flight;
}

uint8_t mh_link_gate_next_tx_id(mh_link_gate_t *gate) {
    uint8_t id;

    if (gate == NULL) {
        return 0U;
    }
    id = gate->next_tx_id++;
    if (gate->next_tx_id == MH_LINK_RESERVED_TX_ID) {
        gate->next_tx_id++;
    }
    return id;
}

void mh_link_gate_on_submit(mh_link_gate_t *gate,
                            uint8_t tx_id,
                            uint16_t seq,
                            uint32_t now_ms,
                            uint32_t timeout_ms) {
    if (gate == NULL) {
        return;
    }
    gate->in_flight = true;
    gate->tx_id = tx_id;
    gate->seq = seq;
    gate->deadline_ms = now_ms + timeout_ms;
    gate->submitted++;
}

mh_link_event_t mh_link_gate_on_frame(mh_link_gate_t *gate,
                                      const murata_host_frame_t *frame) {
    if (gate == NULL || frame == NULL || !gate->in_flight) {
        return MH_LINK_EV_NONE;
    }

    switch (frame->type) {
        case HOST_TYPE_TX_DONE_URC:
            if (frame->payload_len < 2U || frame->payload[0] != gate->tx_id) {
                return MH_LINK_EV_NONE;
            }
            gate->in_flight = false;
            gate->last_status = frame->payload[1];
            if (frame->payload[1] == 0U) {
                gate->sent++;
                return MH_LINK_EV_SENT;
            }
            gate->aborted++;
            return MH_LINK_EV_ABORTED;

        case HOST_TYPE_ERR_PROTO_URC:
            /* payload: offending_type, offending_ver, err_code, detail(2);
             * the URC header carries the offending frame's seq. */
            if (frame->payload_len < 3U ||
                frame->payload[0] != HOST_TYPE_TX_FRAME_REQ ||
                frame->seq != gate->seq) {
                return MH_LINK_EV_NONE;
            }
            gate->in_flight = false;
            gate->last_status = frame->payload[2];
            gate->rejected++;
            return MH_LINK_EV_REJECTED;

        case HOST_TYPE_BOOT_URC:
            gate->in_flight = false;
            gate->reset++;
            return MH_LINK_EV_RESET;

        default:
            /* FAULT_URC included: diagnostic, the TX ring survives it. */
            return MH_LINK_EV_NONE;
    }
}

mh_link_event_t mh_link_gate_tick(mh_link_gate_t *gate, uint32_t now_ms) {
    if (gate == NULL || !gate->in_flight) {
        return MH_LINK_EV_NONE;
    }
    if ((int32_t)(now_ms - gate->deadline_ms) < 0) {
        return MH_LINK_EV_NONE;
    }
    gate->in_flight = false;
    gate->timeouts++;
    return MH_LINK_EV_TIMEOUT;
}

#else

typedef int mh_link_disabled_translation_unit;

#endif
