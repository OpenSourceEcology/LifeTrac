#ifndef LIFETRAC_MURATA_L072_HOST_CFG_WIRE_H
#define LIFETRAC_MURATA_L072_HOST_CFG_WIRE_H

#include "host_cfg.h"

#include <stdbool.h>
#include <stdint.h>

#define HOST_CFG_OK_PAYLOAD_LEN            4U
#define HOST_CFG_DATA_HEADER_LEN           2U
#define HOST_CFG_DATA_PAYLOAD_MAX_LEN      10U

bool host_cfg_wire_encode_ok(uint8_t key,
                             cfg_status_t status,
                             uint8_t actual_len,
                             uint8_t *out_payload,
                             uint8_t out_cap,
                             uint8_t *out_len);

bool host_cfg_wire_encode_data(uint8_t key,
                               const uint8_t *value,
                               uint8_t value_len,
                               uint8_t *out_payload,
                               uint8_t out_cap,
                               uint8_t *out_len);

#endif /* LIFETRAC_MURATA_L072_HOST_CFG_WIRE_H */
