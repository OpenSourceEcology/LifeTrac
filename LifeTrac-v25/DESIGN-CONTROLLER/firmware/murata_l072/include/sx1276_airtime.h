#ifndef LIFETRAC_MURATA_L072_SX1276_AIRTIME_H
#define LIFETRAC_MURATA_L072_SX1276_AIRTIME_H

#include <stdbool.h>
#include <stdint.h>

typedef enum sx1276_airtime_result_e {
    SX1276_AIRTIME_OK = 0,
    SX1276_AIRTIME_OVER_BUDGET = 1,
    SX1276_AIRTIME_BAD_INPUT = 2
} sx1276_airtime_result_t;

/*
 * FCC-A1a (plan §14.2 step 3): config-time airtime invariant.
 *
 * SX1276_AIRTIME_DWELL_CAP_US is the per-channel ToA cap a single TX
 * may consume in one hop dwell. It is intentionally below the 400 ms
 * legal-dwell window by a 20 ms guard band so that scheduler jitter,
 * preamble extension, and host->L072 latency never push an otherwise
 * "legal" frame over the line.
 *
 * SX1276_AIRTIME_DWELL_GUARD_US = 20 000  (margin reserved for jitter)
 * SX1276_AIRTIME_DWELL_CAP_US   = 380 000 (= 400 000 - 20 000)
 */
#define SX1276_AIRTIME_DWELL_WINDOW_MS        400U
#define SX1276_AIRTIME_DWELL_GUARD_US         20000U
#define SX1276_AIRTIME_DWELL_CAP_US           380000UL

uint32_t sx1276_airtime_estimate_toa_us(uint8_t payload_len);
sx1276_airtime_result_t sx1276_airtime_reserve(uint8_t channel_idx,
                                                uint8_t payload_len,
                                                uint32_t now_ms,
                                                uint32_t *out_reserved_us);
void sx1276_airtime_release(uint8_t channel_idx, uint32_t reserved_us);
void sx1276_airtime_commit(uint8_t channel_idx, uint32_t used_us, uint32_t now_ms);

/*
 * D4 (2026-07-24): per-profile QoS budget. Default 400 000 µs per
 * rolling 1 s per channel (P0-latency policy gate). DTS BW500
 * activation raises it to SX1276_AIRTIME_BUDGET_DTS_US — PSD-based
 * compliance has no duty limit and the image strict path carries no
 * P0 traffic. Clamped to [10 000, 950 000] µs inside the setter.
 */
#define SX1276_AIRTIME_BUDGET_DEFAULT_US      400000UL
#define SX1276_AIRTIME_BUDGET_DTS_US          950000UL
void sx1276_airtime_set_budget_us(uint32_t budget_us);
uint32_t sx1276_airtime_get_budget_us(void);

/*
 * Pure ToA calculator (no register reads, no globals). Used by the
 * config-time invariant check and by host-side mirrors. Returns the
 * computed time-on-air in microseconds for the supplied tuple.
 *
 * sf:                spreading factor 6..12 (clamped)
 * bw_hz:             channel bandwidth in Hz (e.g. 125000/250000/500000)
 * cr_den:            coding-rate denominator 5..8 (CR = 4/cr_den, clamped)
 * payload_len:       PHY payload byte count
 * crc_on:            1 = CRC field present, 0 = none
 * implicit_header:   1 = implicit header mode (no PHY header), 0 = explicit
 * preamble_symbols:  preamble symbol count (defaults to 8 if 0)
 * low_dr_opt:        1 = low-data-rate optimisation enabled (SF11/SF12 @ 125 kHz)
 */
uint32_t sx1276_airtime_compute_toa_us(uint8_t sf,
                                       uint32_t bw_hz,
                                       uint8_t cr_den,
                                       uint8_t payload_len,
                                       uint8_t crc_on,
                                       uint8_t implicit_header,
                                       uint16_t preamble_symbols,
                                       uint8_t low_dr_opt);

/*
 * Config-time airtime invariant (FCC-A1a).
 *
 * Returns true iff a frame of `max_payload_len` bytes transmitted with
 * the supplied (sf, bw_khz, cr_den) tuple, using the firmware's standard
 * framing assumptions (CRC on, explicit header, 8-symbol preamble,
 * low-DR-opt auto-enabled at SF11/12 over 125 kHz), would complete in
 * <= `cap_us` microseconds.
 *
 * out_computed_toa_us (optional) receives the worst-case ToA the helper
 * computed for the tuple, regardless of accept/reject. Callers MAY use
 * the value to populate the AIRTIME_INVARIANT_REJECT_URC payload on a
 * rejection.
 */
bool sx1276_airtime_config_invariant_ok(uint8_t sf,
                                        uint16_t bw_khz,
                                        uint8_t cr_den,
                                        uint8_t max_payload_len,
                                        uint32_t cap_us,
                                        uint32_t *out_computed_toa_us);

#endif /* LIFETRAC_MURATA_L072_SX1276_AIRTIME_H */
