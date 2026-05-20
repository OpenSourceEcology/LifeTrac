#ifndef LIFETRAC_MURATA_L072_SX1276_RX_SCAN_FAIL_H
#define LIFETRAC_MURATA_L072_SX1276_RX_SCAN_FAIL_H

/*
 * FCC-A6c-3-c (analysis doc §13.1 #2 + §13.6 #3): pure helper that
 * turns the A6c-3-b file-static snapshot taken on the FAIL edge of
 * the A6c-1 Scanning SM into:
 *
 *   (a) the 8-bit `sub` payload byte for the HOST_FAULT_CODE_RX_
 *       SCAN_FAILED URC,
 *   (b) the `is_final` boolean that tells scan_dispatch_action()
 *       whether to retry (SCANNING re-entry with anchor reset) or
 *       absorb in FAILED.
 *
 * Concern split mirrors the A6c-1 (policy) / A6c-2 (counters TU) /
 * A6c-2-c (HW dispatch) split:
 *
 *   - This file: pure decision (sub-byte assembly + retry gate).
 *   - radio/sx1276_rx.c scan_dispatch_action(FAIL): caller — emits
 *     the URC, increments the retry counter, performs the HW
 *     re-entry sequence on non-final, or standby-and-absorb on
 *     final.
 *
 * Keeping the assembly + retry decision in a pure HW-free helper
 * lets bench/host_proto/rx_scan_fail.c lock the §13.1 #2 bit layout
 * with golden vectors that never touch the modem, the host_cmd
 * wire layer, or the file-statics inside sx1276_rx.c — same pattern
 * as the γ-1 RX retune policy + counters TU.
 *
 * §13.1 #2 sub-byte layout (MSB → LSB):
 *
 *     bit 7 : final       — 1 ⇒ this is the absorbing FAIL,
 *                            0 ⇒ a retry follows.
 *     bit 6 : warm        — 1 ⇒ this boot has been LOCKED at least
 *                            once (latched, never cleared).
 *     bit 5 : hw_suspect  — 1 ⇒ the just-failed scan window observed
 *                            zero DIO0 IRQs AND zero CRC errors;
 *                            consistent with a silent / dead radio.
 *     bit 4 : crc_seen    — 1 ⇒ at least one payload-CRC-error IRQ
 *                            arrived during the just-failed scan
 *                            window (suggests interference or wrong
 *                            profile rather than dead modem).
 *     bits 3..0 : attempt — current retry count (0..MAX), saturated
 *                            at 0x0F. On the first FAIL of a cold
 *                            start this is 0.
 *
 * Note: bit 5 and bit 4 are independent — `crc_seen=1` always
 * forces `hw_suspect=0`, but `crc_seen=0` does NOT force
 * `hw_suspect=1` (the window may have seen only valid-but-stale
 * frames that the SM treated as FRAME_INVALID without a CRC IRQ).
 *
 * No HW deps. Includes only <stdbool.h> + <stdint.h>.
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * Total number of cold-start scan attempts allowed before the
 * absorbing FAILED state is final = MAX_RETRIES + 1 (initial +
 * retries). Set to 3 retries (4 total attempts) per v5.0 §13.1.
 */
#define SX1276_RX_SCAN_MAX_RETRIES 3U

/* Bit positions in the sub-byte. Pinned by the host-side test. */
#define SX1276_RX_SCAN_FAIL_SUB_BIT_FINAL       7U
#define SX1276_RX_SCAN_FAIL_SUB_BIT_WARM        6U
#define SX1276_RX_SCAN_FAIL_SUB_BIT_HW_SUSPECT  5U
#define SX1276_RX_SCAN_FAIL_SUB_BIT_CRC_SEEN    4U
#define SX1276_RX_SCAN_FAIL_SUB_MASK_ATTEMPT    0x0FU

typedef struct {
    uint8_t retries;     /* current retry count BEFORE this FAIL */
    uint8_t max_retries; /* SX1276_RX_SCAN_MAX_RETRIES (injected
                          * so the test can sweep boundary values
                          * without touching the live constant). */
    bool    got_any_irq; /* any DIO0 IRQ observed this scan window */
    bool    crc_seen;    /* any payload-CRC IRQ observed this window */
    bool    ever_locked; /* sticky: this boot has been LOCKED ≥ once */
} sx1276_rx_scan_fail_input_t;

typedef struct {
    uint8_t sub;       /* §13.1 #2 sub-byte, ready for emit_fault() */
    bool    is_final;  /* true ⇒ absorb FAILED; false ⇒ retry */
} sx1276_rx_scan_fail_eval_t;

/*
 * Defensive contract: NULL `in` yields {sub=0x80, is_final=true} so
 * a corrupted caller never accidentally triggers a retry loop. The
 * 0x80 marker (final=1, all other bits zero) makes the bug visible
 * over the wire without crashing the firmware.
 */
sx1276_rx_scan_fail_eval_t sx1276_rx_scan_fail_eval(
    const sx1276_rx_scan_fail_input_t *in);

#endif /* LIFETRAC_MURATA_L072_SX1276_RX_SCAN_FAIL_H */
