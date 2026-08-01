/*
 * F9 (2026-07-30): pure REG_WRITE acceptance policy. See host_reg_gate.h
 * for the rationale. The diagnostic address list below is moved VERBATIM
 * from host_cmd.c's former static reg_write_allowed(); the only new
 * policy is the unconditional opmode value-allowlist above it.
 */
#include "host_reg_gate.h"

#define REG_GATE_OPMODE_ADDR      0x01U
#define REG_GATE_OPMODE_SLEEP     0x80U  /* LoRa SLEEP   (radio_sleep.py) */
#define REG_GATE_OPMODE_STANDBY   0x81U  /* LoRa STANDBY (probe restore)  */
#define REG_GATE_OPMODE_RXCONT    0x85U  /* LoRa RXCONT  (the RS-4.12 arm) */

bool host_reg_write_allowed(uint8_t reg_addr, uint8_t value,
                            bool diag_enabled) {
    /*
     * Production path: the three LoRa-mode opmode values every daemon
     * actually sends are always acceptable. The value gate is what
     * makes this safe without the flag: raw 0x83 (TX) would key the PA
     * bypassing LBT/airtime/dwell accounting, and FSK-mode values
     * (bit7 clear — the T2 TCXO probe's 0x00/0x01) change clocking
     * assumptions; both stay diagnostic-only.
     */
    if (reg_addr == REG_GATE_OPMODE_ADDR &&
        (value == REG_GATE_OPMODE_SLEEP ||
         value == REG_GATE_OPMODE_STANDBY ||
         value == REG_GATE_OPMODE_RXCONT)) {
        return true;
    }

    if (!diag_enabled) {
        return false;
    }

    switch (reg_addr) {
        case 0x01U:  /* W1-9 diag: allow OPMODE for SPI-write isolation */
        case 0x06U:
        case 0x07U:
        case 0x08U:
        case 0x09U:
        case 0x1DU:
        case 0x1EU:
        case 0x26U:
        case 0x31U:
        case 0x33U:  /* 2026-05-25: RegInvertIQ -- host-side IQ-normalize diag */
        case 0x37U:
        case 0x3BU:  /* 2026-05-25: RegInvertIQ2 -- companion to 0x33 */
        case 0x40U:  /* W1-9 diag: allow DIO_MAPPING1 */
            return true;
        default:
            return false;
    }
}
