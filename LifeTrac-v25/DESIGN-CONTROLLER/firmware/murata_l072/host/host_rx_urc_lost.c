#include "host_rx_urc_lost.h"

uint32_t host_rx_urc_lost_eval(uint32_t dio0_edges,
                               bool tx_done_consumed,
                               bool rx_opportunity) {
    uint32_t consumed = 0U;

    if (tx_done_consumed) {
        consumed++;
    }
    if (rx_opportunity) {
        consumed++;
    }
    return (dio0_edges > consumed) ? (dio0_edges - consumed) : 0U;
}
