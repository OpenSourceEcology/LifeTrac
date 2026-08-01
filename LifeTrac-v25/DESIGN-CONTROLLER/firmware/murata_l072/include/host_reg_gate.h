#ifndef LIFETRAC_MURATA_L072_HOST_REG_GATE_H
#define LIFETRAC_MURATA_L072_HOST_REG_GATE_H

/*
 * F9 (2026-07-30): REG_WRITE acceptance policy, hoisted out of
 * host_cmd.c into a PURE function so the bench can pin it
 * (check-reg-write-gate; pattern of sx1276_rx_retune_policy).
 *
 * Why this exists: RS-4.12's opmode-sync — the fix that makes host
 * RXCONT arming visible to the firmware's mode tracker — was reachable
 * only through the HOST_ALLOW_REG_WRITE_DIAG=1 register surface, whose
 * own comment says keep conservative in production. Setting the flag
 * to 0 didn't just deafen RX (nothing could ever re-arm after a
 * profile activation parks the modem in STANDBY, the run-31
 * signature): a refused re-arm inside _apply_profile also made the
 * base daemon REVERT profile switches that had succeeded on the wire.
 * Production was therefore forced to ship with the whole 13-register
 * diagnostic surface open just to keep RX arming alive.
 *
 * The gate decouples them. Production traffic to RegOpMode (0x01) is
 * exactly three values — 0x80 SLEEP, 0x81 STANDBY, 0x85 RXCONT, all
 * LoRa-mode — and those are accepted UNCONDITIONALLY. Everything else
 * (the other 12 diagnostic registers, and opmode values that would key
 * the PA around LBT/airtime/dwell accounting — 0x83 TX — or drop to
 * FSK mode) stays behind the diagnostic flag, which can finally be 0
 * in production.
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * True iff a REG_WRITE of `value` to `reg_addr` is acceptable.
 * `diag_enabled` mirrors HOST_ALLOW_REG_WRITE_DIAG at the call site;
 * it is a parameter (not an #if) so the bench pins BOTH policies.
 * Pure, no side effects.
 */
bool host_reg_write_allowed(uint8_t reg_addr, uint8_t value,
                            bool diag_enabled);

#endif /* LIFETRAC_MURATA_L072_HOST_REG_GATE_H */
