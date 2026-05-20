#include "lora_pkt_hdr.h"

#include <stddef.h>

bool lora_pkt_hdr_pack(const lora_pkt_hdr_t *in, uint8_t *out) {
    if ((in == NULL) || (out == NULL)) {
        return false;
    }
    out[0] = (uint8_t)LORA_PKT_HDR_SCHEMA_VER;
    out[1] = in->profile_id;
    out[2] = in->hop_idx;
    out[3] = 0U;
    out[4] = (uint8_t)((in->epoch      ) & 0xFFU);
    out[5] = (uint8_t)((in->epoch >>  8) & 0xFFU);
    out[6] = (uint8_t)((in->epoch >> 16) & 0xFFU);
    out[7] = (uint8_t)((in->epoch >> 24) & 0xFFU);
    return true;
}

lora_pkt_hdr_status_t lora_pkt_hdr_unpack(const uint8_t *in,
                                          uint16_t in_len,
                                          lora_pkt_hdr_t *out) {
    if ((in == NULL) || (out == NULL)) {
        return LORA_PKT_HDR_ERR_NULL_ARG;
    }
    if (in_len < LORA_PKT_HDR_LEN) {
        return LORA_PKT_HDR_ERR_SHORT_INPUT;
    }
    if (in[0] != (uint8_t)LORA_PKT_HDR_SCHEMA_VER) {
        return LORA_PKT_HDR_ERR_BAD_SCHEMA;
    }
    out->profile_id = in[1];
    out->hop_idx    = in[2];
    /* in[3] is _reserved — additive-evolution friendly, ignored. */
    out->epoch = ((uint32_t)in[4])
               | ((uint32_t)in[5] <<  8)
               | ((uint32_t)in[6] << 16)
               | ((uint32_t)in[7] << 24);
    return LORA_PKT_HDR_OK;
}
