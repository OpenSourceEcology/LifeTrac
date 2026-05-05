#ifndef LIFETRAC_MURATA_L072_SX1276_LBT_H
#define LIFETRAC_MURATA_L072_SX1276_LBT_H

#include <stdint.h>

typedef enum sx1276_lbt_result_e {
    SX1276_LBT_RESULT_CLEAR = 0,
    SX1276_LBT_RESULT_BUSY = 1,
    SX1276_LBT_RESULT_DISABLED = 2,
    SX1276_LBT_RESULT_BACKOFF = 3,
    SX1276_LBT_RESULT_ERROR = 4
} sx1276_lbt_result_t;

sx1276_lbt_result_t sx1276_lbt_check_and_backoff(void);
uint16_t sx1276_lbt_backoff_remaining_ms(void);

#endif /* LIFETRAC_MURATA_L072_SX1276_LBT_H */
