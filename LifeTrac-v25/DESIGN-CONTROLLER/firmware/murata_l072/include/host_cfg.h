#ifndef LIFETRAC_MURATA_L072_HOST_CFG_H
#define LIFETRAC_MURATA_L072_HOST_CFG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum cfg_status_e {
    CFG_STATUS_OK = 0,
    CFG_STATUS_UNKNOWN_KEY = 1,
    CFG_STATUS_BAD_LENGTH = 2,
    CFG_STATUS_OUT_OF_RANGE = 3,
    CFG_STATUS_APPLY_FAILED = 4,
    CFG_STATUS_DEFERRED = 5,
    CFG_STATUS_READ_ONLY = 6
} cfg_status_t;

void cfg_init(void);
cfg_status_t cfg_set(uint8_t key, const uint8_t *value, uint8_t len);
cfg_status_t cfg_get(uint8_t key, uint8_t *out, uint8_t out_cap, uint8_t *out_len);
bool cfg_is_dirty(void);

#endif /* LIFETRAC_MURATA_L072_HOST_CFG_H */
