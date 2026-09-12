#ifndef LIFETRAC_MURATA_L072_HOST_RX_URC_LOST_H
#define LIFETRAC_MURATA_L072_HOST_RX_URC_LOST_H

#include <stdbool.h>
#include <stdint.h>

/*
 * RS-12 (2026-09-12): how many DIO0 edges a main-loop pass failed to
 * service. Pure function of what the pass observed:
 *   dio0_edges       edges since the previous take (sx1276_take_dio0_edges)
 *   tx_done_consumed sx1276_tx_poll() consumed a TxDone edge this pass
 *   rx_opportunity   the pass ran sx1276_rx_service() (DIO0 set, TX idle)
 * Each of the two consumers accounts for at most one edge; anything
 * beyond that was a demodulated packet nobody read before the SX1276
 * FIFO was rewritten. Saturating, never negative.
 */
uint32_t host_rx_urc_lost_eval(uint32_t dio0_edges,
                               bool tx_done_consumed,
                               bool rx_opportunity);

#endif /* LIFETRAC_MURATA_L072_HOST_RX_URC_LOST_H */
