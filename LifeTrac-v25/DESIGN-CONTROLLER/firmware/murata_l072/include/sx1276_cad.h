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

/* RS-4.9 (2026-07-25): abandon an in-flight CAD without waiting for
 * CAD_DONE. Clears the active latch, clears IRQ flags, parks the modem
 * in STANDBY. Safe no-op when no CAD is active. Callers that stop
 * polling before CAD_DONE (e.g. the LBT deadline path) MUST call this,
 * or every later sx1276_cad_begin() refuses until reboot. */
void sx1276_cad_abort(void);

#endif /* LIFETRAC_MURATA_L072_SX1276_CAD_H */
