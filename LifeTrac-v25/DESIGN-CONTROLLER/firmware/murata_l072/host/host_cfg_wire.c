#include "host_cfg_wire.h"

#include <stddef.h>

bool host_cfg_wire_encode_ok(uint8_t key,
                             cfg_status_t status,
                             uint8_t actual_len,
                             uint8_t *out_payload,
                             uint8_t out_cap,
                             uint8_t *out_len) {
    if (out_payload == NULL || out_len == NULL || out_cap < HOST_CFG_OK_PAYLOAD_LEN) {
        return false;
    }

    out_payload[0] = key;
    out_payload[1] = (uint8_t)status;
    out_payload[2] = actual_len;
    out_payload[3] = 0U;
    *out_len = HOST_CFG_OK_PAYLOAD_LEN;
    return true;
}

bool host_cfg_wire_encode_data(uint8_t key,
                               const uint8_t *value,
                               uint8_t value_len,
                               uint8_t *out_payload,
                               uint8_t out_cap,
                               uint8_t *out_len) {
    if (value == NULL || out_payload == NULL || out_len == NULL) {
        return false;
    }

    if (value_len > (HOST_CFG_DATA_PAYLOAD_MAX_LEN - HOST_CFG_DATA_HEADER_LEN)) {
        return false;
    }

    if (out_cap < (uint8_t)(HOST_CFG_DATA_HEADER_LEN + value_len)) {
        return false;
    }

    out_payload[0] = key;
    out_payload[1] = value_len;

    for (uint8_t i = 0U; i < value_len; ++i) {
        out_payload[(uint8_t)(HOST_CFG_DATA_HEADER_LEN + i)] = value[i];
    }

    *out_len = (uint8_t)(HOST_CFG_DATA_HEADER_LEN + value_len);
    return true;
}
