#ifndef LIFETRAC_TRACTOR_H7_MURATA_HOST_H
#define LIFETRAC_TRACTOR_H7_MURATA_HOST_H

#include "mh_wire.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct murata_host_frame_s {
    uint8_t ver;
    uint8_t type;
    uint8_t flags;
    uint16_t seq;
    uint16_t payload_len;
    uint8_t payload[HOST_MAX_PAYLOAD_LEN];
} murata_host_frame_t;

uint16_t murata_host_build_inner_frame(uint8_t type,
                                       uint8_t flags,
                                       uint16_t seq,
                                       const uint8_t *payload,
                                       uint16_t payload_len,
                                       uint8_t *out,
                                       uint16_t out_cap);

bool murata_host_parse_inner_frame(const uint8_t *inner,
                                   uint16_t inner_len,
                                   murata_host_frame_t *out_frame);

#ifdef __cplusplus
}
#endif

#endif /* LIFETRAC_TRACTOR_H7_MURATA_HOST_H */
