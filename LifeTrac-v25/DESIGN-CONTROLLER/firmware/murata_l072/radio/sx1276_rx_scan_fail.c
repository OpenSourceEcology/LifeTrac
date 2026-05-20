/*
 * FCC-A6c-3-c (analysis doc §13.1 #2 + §13.6 #3): pure helper that
 * builds the HOST_FAULT_CODE_RX_SCAN_FAILED sub-byte and the
 * retry-vs-final decision from the A6c-3-b file-static snapshot.
 * See sx1276_rx_scan_fail.h for the bit layout and contract.
 */

#include "sx1276_rx_scan_fail.h"

sx1276_rx_scan_fail_eval_t sx1276_rx_scan_fail_eval(
    const sx1276_rx_scan_fail_input_t *in) {
    sx1276_rx_scan_fail_eval_t out;

    if (in == NULL) {
        /* Defensive: caller bug or zeroed pointer. Force final so
         * the dispatcher absorbs in FAILED and never enters a
         * retry loop driven by garbage state. The 0x80 marker
         * (final=1, all other bits zero, attempt=0) is observable
         * on the wire — see header docstring. */
        out.sub      = (uint8_t)(1U << SX1276_RX_SCAN_FAIL_SUB_BIT_FINAL);
        out.is_final = true;
        return out;
    }

    /* Final iff the caller has already exhausted the retry budget.
     * `retries` is the count BEFORE this FAIL — i.e. on the very
     * first FAIL of a cold start retries=0 and is_final iff
     * max_retries==0. */
    const bool is_final = (in->retries >= in->max_retries);

    /* hw_suspect: the just-failed scan window observed no IRQ
     * activity at all (neither preamble/header/payload-done nor
     * payload-CRC-error). Distinct from crc_seen which is an
     * independent signal — see header docstring. */
    const bool hw_suspect = (!in->got_any_irq) && (!in->crc_seen);

    /* Saturate the attempt nibble at 0x0F. Real configs never
     * exceed MAX_RETRIES=3, but a future bump (or a corrupted
     * counter) must not bleed into the upper status bits. */
    const uint8_t attempt =
        (in->retries > SX1276_RX_SCAN_FAIL_SUB_MASK_ATTEMPT)
            ? (uint8_t)SX1276_RX_SCAN_FAIL_SUB_MASK_ATTEMPT
            : in->retries;

    uint8_t sub = 0U;
    if (is_final) {
        sub |= (uint8_t)(1U << SX1276_RX_SCAN_FAIL_SUB_BIT_FINAL);
    }
    if (in->ever_locked) {
        sub |= (uint8_t)(1U << SX1276_RX_SCAN_FAIL_SUB_BIT_WARM);
    }
    if (hw_suspect) {
        sub |= (uint8_t)(1U << SX1276_RX_SCAN_FAIL_SUB_BIT_HW_SUSPECT);
    }
    if (in->crc_seen) {
        sub |= (uint8_t)(1U << SX1276_RX_SCAN_FAIL_SUB_BIT_CRC_SEEN);
    }
    sub |= (uint8_t)(attempt & SX1276_RX_SCAN_FAIL_SUB_MASK_ATTEMPT);

    out.sub      = sub;
    out.is_final = is_final;
    return out;
}
