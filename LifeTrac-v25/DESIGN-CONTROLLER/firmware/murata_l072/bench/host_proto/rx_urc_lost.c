/*
 * rx_urc_lost — pins host_rx_urc_lost_eval(), the RS-12 URC-path loss
 * arithmetic the main loop runs once per pass. The cases are the ones
 * the bench actually produces:
 *   - one RxDone serviced normally                       -> 0
 *   - two RxDone edges coalesced before one pass         -> 1
 *   - TxDone + RxDone landing in the same take           -> 0 (both consumed)
 *   - a frame clobbered by the TX FIFO load: next pass sees DIO0 set with
 *     TX busy, no TxDone yet, no RX opportunity          -> 1
 *   - the following pass consumes the TxDone alone       -> 0
 *   - saturation / never negative
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>

#include "host_rx_urc_lost.h"

static int s_fail;
static int s_cases;

static void expect_eq(uint32_t got, uint32_t want, const char *what) {
    s_cases++;
    if (got != want) {
        s_fail++;
        printf("[FAIL] %s: got %" PRIu32 " want %" PRIu32 "\n", what, got, want);
    }
}

int main(void) {
    expect_eq(host_rx_urc_lost_eval(0U, false, false), 0U, "idle pass");
    expect_eq(host_rx_urc_lost_eval(1U, false, true),  0U, "one RxDone serviced");
    expect_eq(host_rx_urc_lost_eval(2U, false, true),  1U, "two RxDone coalesced");
    expect_eq(host_rx_urc_lost_eval(3U, false, true),  2U, "three RxDone coalesced");
    expect_eq(host_rx_urc_lost_eval(1U, true,  false), 0U, "TxDone only, TX still busy after");
    expect_eq(host_rx_urc_lost_eval(1U, true,  true),  0U, "TxDone, then idle RX opportunity");
    expect_eq(host_rx_urc_lost_eval(2U, true,  true),  0U, "TxDone + RxDone same take");
    expect_eq(host_rx_urc_lost_eval(3U, true,  true),  1U, "TxDone + two RxDone same take");
    /* Clobber case: the pass after tx_begin() sees the edge from the
     * frame it overwrote — DIO0 set, TX busy, TxDone not yet. */
    expect_eq(host_rx_urc_lost_eval(1U, false, false), 1U, "clobbered frame, TX busy");
    expect_eq(host_rx_urc_lost_eval(0U, true,  true),  0U, "never negative");
    expect_eq(host_rx_urc_lost_eval(UINT32_MAX, false, false), UINT32_MAX, "saturating input");
    expect_eq(host_rx_urc_lost_eval(UINT32_MAX, true, true), UINT32_MAX - 2U, "large input");

    if (s_fail != 0) {
        printf("[FAIL] rx_urc_lost: %d of %d cases\n", s_fail, s_cases);
        return EXIT_FAILURE;
    }
    printf("[PASS] rx_urc_lost: %d cases\n", s_cases);
    return EXIT_SUCCESS;
}
