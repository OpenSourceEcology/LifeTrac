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

/* RS-4.12 (2026-07-25): fold a host-issued raw RegOpMode write into the
 * tracked state. Pure bookkeeping — NO SPI traffic. This is what makes a
 * raw-armed RXCONT visible to the TX path's s_rearm_rx snapshot, so the
 * firmware re-arms the receiver after every TX instead of parking the
 * modem deaf in STANDBY (run-31 root cause; measured 0/22 command
 * delivery at DTS saturation before this fix). */
void sx1276_modes_sync_external(uint8_t raw_opmode);

bool sx1276_modes_to_sleep(void);
bool sx1276_modes_to_standby(void);
bool sx1276_modes_to_tx(void);
bool sx1276_modes_to_rx_cont(void);
bool sx1276_modes_to_rx_single(uint16_t timeout_symbols);
bool sx1276_modes_to_cad(void);

#endif /* LIFETRAC_MURATA_L072_SX1276_MODES_H */
