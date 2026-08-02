#ifndef LIFETRAC_MURATA_L072_HOST_STATS_H
#define LIFETRAC_MURATA_L072_HOST_STATS_H

#include "host_types.h"

#include <stdint.h>

void host_stats_reset(void);
void host_stats_note_radio_events(uint32_t events);

/* RS-11.5 TX FIFO readback discriminator (sx1276_tx.c). */
void host_stats_tx_fifo_rb_ok(void);
void host_stats_tx_fifo_rb_bad(void);
void host_stats_tx_done_early(void);
void host_stats_radio_rx_ok(void);
void host_stats_radio_crc_err(void);
void host_stats_radio_tx_ok(void);
void host_stats_radio_tx_abort_lbt(void);
void host_stats_radio_tx_abort_airtime(void);
uint16_t host_stats_serialize(uint8_t *out, uint16_t out_cap);

#endif /* LIFETRAC_MURATA_L072_HOST_STATS_H */
