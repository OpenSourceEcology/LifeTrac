#ifndef LIFETRAC_MURATA_L072_HOST_STATS_H
#define LIFETRAC_MURATA_L072_HOST_STATS_H

#include "host_types.h"

#include <stdint.h>

void host_stats_reset(void);
void host_stats_note_radio_events(uint32_t events);
void host_stats_radio_rx_ok(void);
void host_stats_radio_crc_err(void);
void host_stats_radio_tx_ok(void);
void host_stats_radio_tx_abort_lbt(void);
void host_stats_radio_tx_abort_airtime(void);
uint16_t host_stats_serialize(uint8_t *out, uint16_t out_cap);

#endif /* LIFETRAC_MURATA_L072_HOST_STATS_H */
