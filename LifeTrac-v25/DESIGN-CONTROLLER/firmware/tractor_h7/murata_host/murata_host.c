#include "murata_host.h"

#include "mh_crc16.h"

#include <stddef.h>
#include <string.h>

static uint16_t read_u16_le(const uint8_t *in) {
    return (uint16_t)in[0] | ((uint16_t)in[1] << 8);
}

uint16_t murata_host_build_inner_frame(uint8_t type,
                                       uint8_t flags,
                                       uint16_t seq,
                                       const uint8_t *payload,
                                       uint16_t payload_len,
                                       uint8_t *out,
                                       uint16_t out_cap) {
    uint16_t idx;
    uint16_t frame_len;
    uint16_t crc;

    if (out == NULL) {
        return 0U;
    }

    if (payload_len > HOST_MAX_PAYLOAD_LEN) {
        return 0U;
    }

    if (payload_len > 0U && payload == NULL) {
        return 0U;
    }

    frame_len = (uint16_t)(HOST_HEADER_LEN + payload_len + HOST_CRC_LEN);
    if (frame_len > out_cap) {
        return 0U;
    }

    out[0] = HOST_PROTOCOL_VER;
    out[1] = type;
    out[2] = flags;
    out[3] = (uint8_t)(seq & 0xFFU);
    out[4] = (uint8_t)((seq >> 8) & 0xFFU);
    out[5] = (uint8_t)(payload_len & 0xFFU);
    out[6] = (uint8_t)((payload_len >> 8) & 0xFFU);

    idx = HOST_HEADER_LEN;
    if (payload_len > 0U) {
        memcpy(&out[idx], payload, payload_len);
        idx = (uint16_t)(idx + payload_len);
    }

    crc = mh_crc16_ccitt(out, idx);
    out[idx++] = (uint8_t)(crc & 0xFFU);
    out[idx++] = (uint8_t)((crc >> 8) & 0xFFU);

    return idx;
}

bool murata_host_parse_inner_frame(const uint8_t *inner,
                                   uint16_t inner_len,
                                   murata_host_frame_t *out_frame) {
    uint16_t expected_len;
    uint16_t frame_crc;
    uint16_t calc_crc;
    uint16_t payload_len;

    if (inner == NULL || out_frame == NULL) {
        return false;
    }

    if (inner_len < (HOST_HEADER_LEN + HOST_CRC_LEN)) {
        return false;
    }

    payload_len = read_u16_le(&inner[5]);
    if (payload_len > HOST_MAX_PAYLOAD_LEN) {
        return false;
    }

    expected_len = (uint16_t)(HOST_HEADER_LEN + payload_len + HOST_CRC_LEN);
    if (inner_len != expected_len) {
        return false;
    }

    frame_crc = read_u16_le(&inner[(uint16_t)(inner_len - HOST_CRC_LEN)]);
    calc_crc = mh_crc16_ccitt(inner, (size_t)(inner_len - HOST_CRC_LEN));
    if (frame_crc != calc_crc) {
        return false;
    }

    out_frame->ver = inner[0];
    out_frame->type = inner[1];
    out_frame->flags = inner[2];
    out_frame->seq = read_u16_le(&inner[3]);
    out_frame->payload_len = payload_len;
    if (payload_len > 0U) {
        memcpy(out_frame->payload, &inner[HOST_HEADER_LEN], payload_len);
    }

    return true;
}
