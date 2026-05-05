#ifndef LIFETRAC_MURATA_L072_SX1276_CAD_H
#define LIFETRAC_MURATA_L072_SX1276_CAD_H

#include <stdbool.h>

typedef enum sx1276_cad_result_e {
    SX1276_CAD_RESULT_PENDING = 0,
    SX1276_CAD_RESULT_CLEAR = 1,
    SX1276_CAD_RESULT_DETECTED = 2,
    SX1276_CAD_RESULT_ERROR = 3
} sx1276_cad_result_t;

bool sx1276_cad_begin(void);
sx1276_cad_result_t sx1276_cad_poll(void);

#endif /* LIFETRAC_MURATA_L072_SX1276_CAD_H */
