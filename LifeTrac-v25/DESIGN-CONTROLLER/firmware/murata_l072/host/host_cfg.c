#include "host_cfg.h"

#include "config.h"
#include "host_cfg_keys.h"
#include "host_cfg_profile.h"
#include "host_types.h"
#include "sx1276.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define CFG_FLAG_READ_ONLY                  (1U << 0)
#define CFG_FLAG_DEFERRED                   (1U << 1)

#define CFG_KIND_RAW                        0U
#define CFG_KIND_BOOL                       1U
#define CFG_KIND_I8                         2U
#define CFG_KIND_U8                         3U
#define CFG_KIND_U16                        4U
#define CFG_KIND_U32                        5U
#define CFG_KIND_U64                        6U

#define DEFAULT_TX_POWER_DBM                14U
#define DEFAULT_LBT_THRESHOLD_DBM           ((uint8_t)0xA6U) /* -90 dBm */
#define DEFAULT_LBT_MAX_BACKOFF_MS          500U
#define DEFAULT_CAD_SYMBOLS                 4U
#define DEFAULT_FHSS_CHANNEL_MASK           0x00000000000000FFULL
#define DEFAULT_FHSS_DWELL_MS               400U
#define DEFAULT_BEACON_CHANNEL_IDX          0U
#define DEFAULT_REPLAY_WINDOW               16U
/*
 * FCC-EVID-D6-2-a-ii defaults. Antenna gain 2 dBi matches the
 * existing host TU make_*_req() helpers (cfg_profile.c). HW
 * ceiling 17 dBm matches the bench tier and the existing
 * sx1276 PA clamp; production hardware can re-program it via
 * cfg_set up to 30 dBm.
 */
#define DEFAULT_ANTENNA_GAIN_DBI            ((uint8_t)(int8_t)2)
#define DEFAULT_HW_CEILING_DBM              17U

#define LE16_0(v)                           ((uint8_t)((v) & 0xFFU))
#define LE16_1(v)                           ((uint8_t)(((v) >> 8) & 0xFFU))

#define LE32_0(v)                           ((uint8_t)((v) & 0xFFU))
#define LE32_1(v)                           ((uint8_t)(((v) >> 8) & 0xFFU))
#define LE32_2(v)                           ((uint8_t)(((v) >> 16) & 0xFFU))
#define LE32_3(v)                           ((uint8_t)(((v) >> 24) & 0xFFU))

#define LE64_0(v)                           ((uint8_t)((v) & 0xFFU))
#define LE64_1(v)                           ((uint8_t)(((v) >> 8) & 0xFFU))
#define LE64_2(v)                           ((uint8_t)(((v) >> 16) & 0xFFU))
#define LE64_3(v)                           ((uint8_t)(((v) >> 24) & 0xFFU))
#define LE64_4(v)                           ((uint8_t)(((v) >> 32) & 0xFFU))
#define LE64_5(v)                           ((uint8_t)(((v) >> 40) & 0xFFU))
#define LE64_6(v)                           ((uint8_t)(((v) >> 48) & 0xFFU))
#define LE64_7(v)                           ((uint8_t)(((v) >> 56) & 0xFFU))

typedef cfg_status_t (*cfg_apply_fn_t)(const uint8_t *value, uint8_t len);

typedef struct cfg_key_desc_s {
    uint8_t key;
    uint8_t len;
    uint8_t kind;
    uint8_t flags;
    cfg_apply_fn_t apply;
    uint8_t default_value[CFG_KEY_MAX_VALUE_LEN];
} cfg_key_desc_t;

typedef struct cfg_value_s {
    uint8_t key;
    uint8_t len;
    uint8_t bytes[CFG_KEY_MAX_VALUE_LEN];
} cfg_value_t;

static cfg_status_t cfg_apply_tx_power_dbm(const uint8_t *value, uint8_t len) {
    if (len != 1U) {
        return CFG_STATUS_BAD_LENGTH;
    }

    sx1276_set_tx_power_dbm(value[0]);
    return CFG_STATUS_OK;
}

static cfg_status_t cfg_apply_unsupported(const uint8_t *value, uint8_t len) {
    (void)value;
    (void)len;
    return CFG_STATUS_APPLY_FAILED;
}

static const cfg_key_desc_t k_cfg_desc[CFG_KEY_COUNT] = {
    { CFG_KEY_TX_POWER_DBM,          1U, CFG_KIND_U8,   0U,                cfg_apply_tx_power_dbm,
      { DEFAULT_TX_POWER_DBM, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_TX_POWER_ADAPT_ENABLE, 1U, CFG_KIND_BOOL, 0U,                NULL,
      { LORA_FW_TX_POWER_ADAPT, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_LBT_ENABLE,            1U, CFG_KIND_BOOL, 0U,                NULL,
      { LORA_FW_LBT_ENABLE, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_LBT_THRESHOLD_DBM,     1U, CFG_KIND_I8,   0U,                NULL,
      { DEFAULT_LBT_THRESHOLD_DBM, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_FHSS_ENABLE,           1U, CFG_KIND_BOOL, 0U,                NULL,
      { 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_FHSS_QUALITY_AWARE,    1U, CFG_KIND_BOOL, 0U,                cfg_apply_unsupported,
      { LORA_FW_QUALITY_AWARE_FHSS, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_FHSS_CHANNEL_MASK,     8U, CFG_KIND_U64,  0U,                NULL,
      { LE64_0(DEFAULT_FHSS_CHANNEL_MASK), LE64_1(DEFAULT_FHSS_CHANNEL_MASK),
        LE64_2(DEFAULT_FHSS_CHANNEL_MASK), LE64_3(DEFAULT_FHSS_CHANNEL_MASK),
        LE64_4(DEFAULT_FHSS_CHANNEL_MASK), LE64_5(DEFAULT_FHSS_CHANNEL_MASK),
        LE64_6(DEFAULT_FHSS_CHANNEL_MASK), LE64_7(DEFAULT_FHSS_CHANNEL_MASK) } },
    { CFG_KEY_DEEP_SLEEP_ENABLE,     1U, CFG_KIND_BOOL, 0U,                cfg_apply_unsupported,
      { LORA_FW_DEEP_SLEEP_BUILD, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_BEACON_ENABLE,         1U, CFG_KIND_BOOL, 0U,                NULL,
      { LORA_FW_BEACON_ENABLE, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_BEACON_CHANNEL_IDX,    1U, CFG_KIND_U8,   0U,                NULL,
      { DEFAULT_BEACON_CHANNEL_IDX, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_HOST_BAUD,             4U, CFG_KIND_U32,  CFG_FLAG_DEFERRED, NULL,
      { LE32_0(HOST_BAUD_DEFAULT), LE32_1(HOST_BAUD_DEFAULT),
        LE32_2(HOST_BAUD_DEFAULT), LE32_3(HOST_BAUD_DEFAULT), 0U, 0U, 0U, 0U } },
    { CFG_KEY_REPLAY_WINDOW,         1U, CFG_KIND_U8,   0U,                NULL,
      { DEFAULT_REPLAY_WINDOW, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_IWDG_WINDOW_MS,        2U, CFG_KIND_U16,  0U,                NULL,
      { LE16_0(IWDG_RUN_WINDOW_MS), LE16_1(IWDG_RUN_WINDOW_MS), 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_CRYPTO_IN_L072,        1U, CFG_KIND_BOOL, 0U,                cfg_apply_unsupported,
      { LORA_FW_CRYPTO_IN_L072, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
        { CFG_KEY_LBT_MAX_BACKOFF_MS,    2U, CFG_KIND_U16,  0U,                NULL,
            { LE16_0(DEFAULT_LBT_MAX_BACKOFF_MS), LE16_1(DEFAULT_LBT_MAX_BACKOFF_MS), 0U, 0U, 0U, 0U, 0U, 0U } },
        { CFG_KEY_CAD_SYMBOLS,           1U, CFG_KIND_U8,   0U,                NULL,
            { DEFAULT_CAD_SYMBOLS, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
        { CFG_KEY_FHSS_DWELL_MS,         2U, CFG_KIND_U16,  0U,                NULL,
            { LE16_0(DEFAULT_FHSS_DWELL_MS), LE16_1(DEFAULT_FHSS_DWELL_MS), 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_REG_PROFILE,           1U, CFG_KIND_U8,   0U,                NULL,
      { REG_PROFILE_BENCH_ONLY_FIXED_915, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_ANTENNA_GAIN_DBI,      1U, CFG_KIND_I8,   0U,                NULL,
      { DEFAULT_ANTENNA_GAIN_DBI, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_HW_CEILING_DBM,        1U, CFG_KIND_U8,   0U,                NULL,
      { DEFAULT_HW_CEILING_DBM, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    /*
     * FHSS seed identity. Default 0 reproduces the pre-2026-07-29
     * hardcoded (0,0,0) permutation exactly, so adding these keys
     * changes no behaviour until a host provisions them. No apply
     * hook: the seed is consumed at profile activation, not on write
     * (see the ORDERING CONTRACT in host_cfg_keys.h).
     */
    { CFG_KEY_FHSS_FARM_ID,          8U, CFG_KIND_U64,  0U,                NULL,
      { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_FHSS_NODE_ID,          8U, CFG_KIND_U64,  0U,                NULL,
      { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_PROTOCOL_VERSION,      1U, CFG_KIND_U8,   CFG_FLAG_READ_ONLY, NULL,
      { HOST_PROTOCOL_VER, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_WIRE_SCHEMA_VERSION,   1U, CFG_KIND_U8,   CFG_FLAG_READ_ONLY, NULL,
      { HOST_WIRE_SCHEMA_VER, 0U, 0U, 0U, 0U, 0U, 0U, 0U } },
    { CFG_KEY_CFG_DIRTY,             1U, CFG_KIND_BOOL, CFG_FLAG_READ_ONLY, NULL,
      { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U } }
};

_Static_assert((sizeof(k_cfg_desc) / sizeof(k_cfg_desc[0])) == CFG_KEY_COUNT,
               "CFG key table must match CFG_KEY_COUNT");

static cfg_value_t s_cfg_values[CFG_KEY_COUNT];
static bool s_cfg_dirty;

static uint16_t read_u16_le(const uint8_t *in) {
    return (uint16_t)in[0] | ((uint16_t)in[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *in) {
    return (uint32_t)in[0] |
           ((uint32_t)in[1] << 8) |
           ((uint32_t)in[2] << 16) |
           ((uint32_t)in[3] << 24);
}

static uint64_t read_u64_le(const uint8_t *in) {
    return (uint64_t)in[0] |
           ((uint64_t)in[1] << 8) |
           ((uint64_t)in[2] << 16) |
           ((uint64_t)in[3] << 24) |
           ((uint64_t)in[4] << 32) |
           ((uint64_t)in[5] << 40) |
           ((uint64_t)in[6] << 48) |
           ((uint64_t)in[7] << 56);
}

static int cfg_find_index(uint8_t key) {
    for (uint8_t i = 0U; i < CFG_KEY_COUNT; ++i) {
        if (k_cfg_desc[i].key == key) {
            return (int)i;
        }
    }
    return -1;
}

static bool cfg_value_is_default(int index, const uint8_t *value) {
    return memcmp(value, k_cfg_desc[index].default_value, k_cfg_desc[index].len) == 0;
}

/*
 * FCC §A5 ERP enforcement (2026-07-29).
 *
 * Maximum conducted dBm the ACTIVE profile permits. Before this
 * existed, host_cfg_profile_power_clamp() was computed once per
 * staging attempt, compared against 0 to accept or reject the profile,
 * and then thrown away — so the ERP ceiling the profile system appears
 * to enforce was never enforced in hardware. Two exceedance routes
 * were reachable: declare a high antenna gain (or a low hw ceiling),
 * let the profile be accepted at a reduced ceiling, then write
 * CFG_KEY_TX_POWER_DBM up to 17 dBm unopposed.
 *
 * No new state is needed. s_active inside host_cfg_profile.c retains
 * every clamp input the validator actually approved, and the tier
 * ceiling is derivable from its profile_id.
 *
 * Returns 0 for "no ceiling known" — which is also power_clamp()'s
 * no-headroom sentinel and is never a valid transmit power, so callers
 * must test for it rather than clamping to it. Reachable when
 * host_cfg_profile_active() is NULL, which the tx_len_guard bench TU
 * deliberately stubs.
 */
static uint8_t cfg_active_erp_max_dbm(void) {
    const host_cfg_profile_req_t *active = host_cfg_profile_active();
    if (active == NULL) {
        return 0U;
    }
    return host_cfg_profile_power_clamp(
        host_cfg_profile_tier_ceiling_dBm(active->profile_id),
        active->hw_ceiling_dBm,
        active->antenna_gain_dBi);
}

static cfg_status_t cfg_validate_and_normalize(uint8_t key, uint8_t *value, uint8_t len) {
    (void)len;

    switch (key) {
        case CFG_KEY_TX_POWER_DBM: {
            if (value[0] < 2U) {
                value[0] = 2U;
            } else if (value[0] > 17U) {
                value[0] = 17U;
            }
            /*
             * FCC §A5 ERP enforcement (2026-07-29): also clamp to what
             * the ACTIVE profile allows. Without this a host could
             * declare 24 dBi of antenna gain, have the profile accepted
             * at a 12 dBm allowance, then write 17 dBm here and
             * transmit 5 dB over the ERP limit — with cfg_get showing
             * nothing anomalous.
             *
             * Normalise rather than reject, matching this key's
             * existing contract (0 -> 2, 30 -> 17 are both OK, not
             * OUT_OF_RANGE) so the wire behaviour and the golden
             * vectors in cfg_contract.c stay consistent.
             *
             * A 0 answer means "no ceiling known" and is deliberately
             * NOT applied as a ceiling of 0 — doing so would clamp
             * every write to the 2 dBm floor and effectively brick the
             * key in any TU where host_cfg_profile_active() is NULL.
             */
            const uint8_t erp_max = cfg_active_erp_max_dbm();
            if (erp_max != 0U && value[0] > erp_max) {
                value[0] = erp_max;
            }
            return CFG_STATUS_OK;
        }

        case CFG_KEY_TX_POWER_ADAPT_ENABLE:
        case CFG_KEY_LBT_ENABLE:
        case CFG_KEY_FHSS_ENABLE:
        case CFG_KEY_FHSS_QUALITY_AWARE:
        case CFG_KEY_DEEP_SLEEP_ENABLE:
        case CFG_KEY_BEACON_ENABLE:
        case CFG_KEY_CRYPTO_IN_L072:
            return (value[0] <= 1U) ? CFG_STATUS_OK : CFG_STATUS_OUT_OF_RANGE;

        case CFG_KEY_LBT_THRESHOLD_DBM: {
            const int8_t thr = (int8_t)value[0];
            if (thr < -120 || thr > 0) {
                return CFG_STATUS_OUT_OF_RANGE;
            }
            return CFG_STATUS_OK;
        }

        case CFG_KEY_BEACON_CHANNEL_IDX:
            return (value[0] <= 7U) ? CFG_STATUS_OK : CFG_STATUS_OUT_OF_RANGE;

        case CFG_KEY_HOST_BAUD: {
            const uint32_t baud = read_u32_le(value);
            if (baud == 9600UL || baud == 115200UL || baud == 921600UL) {
                return CFG_STATUS_OK;
            }
            return CFG_STATUS_OUT_OF_RANGE;
        }

        case CFG_KEY_REPLAY_WINDOW:
            return (value[0] >= 1U && value[0] <= 64U) ? CFG_STATUS_OK : CFG_STATUS_OUT_OF_RANGE;

        case CFG_KEY_IWDG_WINDOW_MS: {
            const uint16_t ms = read_u16_le(value);
            if (ms < 50U || ms > 5000U) {
                return CFG_STATUS_OUT_OF_RANGE;
            }
            return CFG_STATUS_OK;
        }

        case CFG_KEY_LBT_MAX_BACKOFF_MS: {
            const uint16_t max_backoff_ms = read_u16_le(value);
            if (max_backoff_ms < 10U || max_backoff_ms > 5000U) {
                return CFG_STATUS_OUT_OF_RANGE;
            }
            return CFG_STATUS_OK;
        }

        case CFG_KEY_CAD_SYMBOLS:
            return (value[0] >= 1U && value[0] <= 10U) ? CFG_STATUS_OK : CFG_STATUS_OUT_OF_RANGE;

        case CFG_KEY_FHSS_DWELL_MS: {
            const uint16_t dwell_ms = read_u16_le(value);
            if (dwell_ms < 50U || dwell_ms > 2000U) {
                return CFG_STATUS_OUT_OF_RANGE;
            }
            return CFG_STATUS_OK;
        }

        case CFG_KEY_FHSS_CHANNEL_MASK:
            return (read_u64_le(value) != 0ULL) ? CFG_STATUS_OK : CFG_STATUS_OUT_OF_RANGE;

        case CFG_KEY_REG_PROFILE: {
            /*
             * FCC-EVID-D6-2-a-iii: route cfg_set(REG_PROFILE) through
             * the host_cfg_profile two-phase commit. Wire layer
             * synthesises a host_cfg_profile_req_t from the candidate
             * profile_id plus the current cfg state (FHSS channel
             * mask, antenna gain, HW ceiling); modem BW is fixed per
             * profile via host_cfg_profile_default_bw_hz so the wire
             * path never produces BW_MISMATCH. tx_routed mirrors the
             * LIFETRAC_FHSS_TX_ROUTED build symbol exactly as
             * documented in host_cfg_profile.h.
             *
             * The legacy >REG_PROFILE_MAX early-return is preserved
             * for backward compat with the cfg_contract.c
             * reg_profile_unknown_oor case; it short-circuits before
             * we touch the profile state machine, matching the prior
             * wire contract that an unknown profile id is OUT_OF_RANGE
             * (not the BAD_PROFILE alias).
             */
            if (value[0] > REG_PROFILE_MAX) {
                return CFG_STATUS_OUT_OF_RANGE;
            }

            host_cfg_profile_req_t req;
            memset(&req, 0, sizeof(req));
            req.profile_id   = value[0];
            req.modem_bw_hz  = host_cfg_profile_default_bw_hz(value[0]);

            {
                int idx = cfg_find_index(CFG_KEY_FHSS_CHANNEL_MASK);
                if (idx < 0) {
                    return CFG_STATUS_APPLY_FAILED;
                }
                req.channel_mask = read_u64_le(s_cfg_values[idx].bytes);
            }
            {
                int idx = cfg_find_index(CFG_KEY_ANTENNA_GAIN_DBI);
                if (idx < 0) {
                    return CFG_STATUS_APPLY_FAILED;
                }
                req.antenna_gain_dBi = (int8_t)s_cfg_values[idx].bytes[0];
            }
            {
                int idx = cfg_find_index(CFG_KEY_HW_CEILING_DBM);
                if (idx < 0) {
                    return CFG_STATUS_APPLY_FAILED;
                }
                req.hw_ceiling_dBm = s_cfg_values[idx].bytes[0];
            }
            /*
             * FHSS hop-sequence identity (2026-07-29). Collected here
             * so activate() can forward it to sx1276_fhss_init(), which
             * used to receive a hardcoded (0, 0, 0) and therefore gave
             * every radio in the fleet the same channel permutation.
             * Zero defaults reproduce that historical seed exactly.
             */
            {
                int idx = cfg_find_index(CFG_KEY_FHSS_FARM_ID);
                if (idx < 0) {
                    return CFG_STATUS_APPLY_FAILED;
                }
                req.farm_id = read_u64_le(s_cfg_values[idx].bytes);
            }
            {
                int idx = cfg_find_index(CFG_KEY_FHSS_NODE_ID);
                if (idx < 0) {
                    return CFG_STATUS_APPLY_FAILED;
                }
                req.node_id = read_u64_le(s_cfg_values[idx].bytes);
            }

#ifdef LIFETRAC_FHSS_TX_ROUTED
            const bool tx_routed = true;
#else
            const bool tx_routed = false;
#endif

            host_cfg_profile_reject_t r =
                host_cfg_profile_stage(&req, tx_routed);
            if (r != HOST_CFG_PROFILE_REJECT_NONE) {
                return host_cfg_profile_reject_to_cfg_status(r);
            }
            r = host_cfg_profile_activate();
            if (r != HOST_CFG_PROFILE_REJECT_NONE) {
                return host_cfg_profile_reject_to_cfg_status(r);
            }

            /*
             * FCC §A5 ERP enforcement (2026-07-29): program the PA to
             * the newly-active allowance.
             *
             * activate() previously left TX power completely untouched,
             * which broke the guarantee in both directions: the ERP
             * clamp never reached RegPaConfig at all, and switching to a
             * MORE restrictive profile did not lower an already-high
             * setting.
             *
             * Semantics: effective = min(configured, erp_max). This
             * only ever LOWERS power — it deliberately does not raise a
             * deliberately-reduced setting up to the new ceiling,
             * because the operator's chosen power is intent while the
             * clamp is a limit.
             *
             * We program unconditionally rather than only when clamping
             * is required, so that after any activation the hardware and
             * the cfg table are guaranteed to agree. At boot they agree
             * only by coincidence: sx1276_init() hard-codes 14 dBm and
             * cfg_init() seeds the table without invoking any apply
             * hook, so nothing had ever reconciled them.
             *
             * erp_max == 0 means the clamp found no headroom above the
             * +2 dBm floor. That cannot happen here — validate()
             * rejects such a request before activate() can run — but if
             * it ever did, passing the sentinel to
             * sx1276_set_tx_power_dbm() would be silently raised to
             * 2 dBm and TRANSMIT. Fail closed: leave the PA alone and
             * report the failure.
             */
            {
                const uint8_t erp_max = cfg_active_erp_max_dbm();
                const int pidx = cfg_find_index(CFG_KEY_TX_POWER_DBM);
                if (pidx < 0) {
                    return CFG_STATUS_APPLY_FAILED;
                }
                if (erp_max == 0U) {
                    return CFG_STATUS_APPLY_FAILED;
                }

                uint8_t effective = s_cfg_values[pidx].bytes[0];
                if (effective > erp_max) {
                    effective = erp_max;
                }
                s_cfg_values[pidx].bytes[0] = effective;
                sx1276_set_tx_power_dbm(effective);

                if (!s_cfg_dirty &&
                        !cfg_value_is_default(pidx, s_cfg_values[pidx].bytes)) {
                    s_cfg_dirty = true;
                }
            }

            return CFG_STATUS_OK;
        }

        default:
            return CFG_STATUS_OK;
    }
}

bool cfg_is_dirty(void) {
    return s_cfg_dirty;
}

void cfg_init(void) {
    for (uint8_t i = 0U; i < CFG_KEY_COUNT; ++i) {
        s_cfg_values[i].key = k_cfg_desc[i].key;
        s_cfg_values[i].len = k_cfg_desc[i].len;
        memset(s_cfg_values[i].bytes, 0, sizeof(s_cfg_values[i].bytes));
        memcpy(s_cfg_values[i].bytes,
               k_cfg_desc[i].default_value,
               k_cfg_desc[i].len);
    }

    /*
     * FCC-EVID-D6-2-a-iii: seed the profile two-phase-commit state
     * machine with the bench-default req so cfg_set(REG_PROFILE)
     * after boot sees a consistent active profile and so the host
     * TU run_case()->cfg_init() cycle resets the state machine
     * between cases.
     */
    {
        host_cfg_profile_req_t initial;
        memset(&initial, 0, sizeof(initial));
        initial.profile_id      = REG_PROFILE_BENCH_ONLY_FIXED_915;
        initial.channel_mask    = DEFAULT_FHSS_CHANNEL_MASK;
        initial.modem_bw_hz     = host_cfg_profile_default_bw_hz(
                                      REG_PROFILE_BENCH_ONLY_FIXED_915);
        initial.antenna_gain_dBi = (int8_t)DEFAULT_ANTENNA_GAIN_DBI;
        initial.hw_ceiling_dBm  = DEFAULT_HW_CEILING_DBM;
        host_cfg_profile_reset(&initial);
    }

    s_cfg_dirty = false;
}

cfg_status_t cfg_set(uint8_t key, const uint8_t *value, uint8_t len) {
    uint8_t new_value[CFG_KEY_MAX_VALUE_LEN] = {0};
    uint8_t old_value[CFG_KEY_MAX_VALUE_LEN] = {0};
    cfg_status_t status;

    const int index = cfg_find_index(key);
    if (index < 0) {
        return CFG_STATUS_UNKNOWN_KEY;
    }

    if ((k_cfg_desc[index].flags & CFG_FLAG_READ_ONLY) != 0U) {
        return CFG_STATUS_READ_ONLY;
    }

    if (len != k_cfg_desc[index].len) {
        return CFG_STATUS_BAD_LENGTH;
    }

    if (value == NULL) {
        return CFG_STATUS_BAD_LENGTH;
    }

    memcpy(new_value, value, len);
    status = cfg_validate_and_normalize(key, new_value, len);
    if (status != CFG_STATUS_OK) {
        return status;
    }

    memcpy(old_value, s_cfg_values[index].bytes, k_cfg_desc[index].len);
    memcpy(s_cfg_values[index].bytes, new_value, k_cfg_desc[index].len);

    if (k_cfg_desc[index].apply != NULL) {
        status = k_cfg_desc[index].apply(s_cfg_values[index].bytes, k_cfg_desc[index].len);
        if (status != CFG_STATUS_OK) {
            memcpy(s_cfg_values[index].bytes, old_value, k_cfg_desc[index].len);
            return status;
        }
    }

    if (!s_cfg_dirty && !cfg_value_is_default(index, s_cfg_values[index].bytes)) {
        s_cfg_dirty = true;
    }

    if ((k_cfg_desc[index].flags & CFG_FLAG_DEFERRED) != 0U) {
        return CFG_STATUS_DEFERRED;
    }

    return CFG_STATUS_OK;
}

cfg_status_t cfg_get(uint8_t key, uint8_t *out, uint8_t out_cap, uint8_t *out_len) {
    const int index = cfg_find_index(key);

    if (index < 0) {
        return CFG_STATUS_UNKNOWN_KEY;
    }

    if (out == NULL || out_len == NULL || out_cap < k_cfg_desc[index].len) {
        return CFG_STATUS_BAD_LENGTH;
    }

    if (key == CFG_KEY_CFG_DIRTY) {
        out[0] = s_cfg_dirty ? 1U : 0U;
        *out_len = 1U;
        return CFG_STATUS_OK;
    }

    memcpy(out, s_cfg_values[index].bytes, k_cfg_desc[index].len);
    *out_len = k_cfg_desc[index].len;
    return CFG_STATUS_OK;
}
