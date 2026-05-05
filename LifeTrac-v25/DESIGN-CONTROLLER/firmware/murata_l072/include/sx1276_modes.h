#ifndef LIFETRAC_MURATA_L072_SX1276_MODES_H
#define LIFETRAC_MURATA_L072_SX1276_MODES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum sx1276_state_e {
    SX1276_STATE_UNINIT = 0,
    SX1276_STATE_SLEEP = 1,
    SX1276_STATE_STANDBY = 2,
    SX1276_STATE_TX = 3,
    SX1276_STATE_RX_CONT = 4,
    SX1276_STATE_RX_SINGLE = 5,
    SX1276_STATE_CAD = 6,
    SX1276_STATE_FAULT = 0xFFU
} sx1276_state_t;

bool sx1276_modes_init(void);
sx1276_state_t sx1276_modes_get_state(void);

bool sx1276_modes_to_sleep(void);
bool sx1276_modes_to_standby(void);
bool sx1276_modes_to_tx(void);
bool sx1276_modes_to_rx_cont(void);
bool sx1276_modes_to_rx_single(uint16_t timeout_symbols);
bool sx1276_modes_to_cad(void);

#endif /* LIFETRAC_MURATA_L072_SX1276_MODES_H */
