#include "host_cfg.h"

#include "config.h"
#include "host_cfg_keys.h"
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

static cfg_status_t cfg_validate_and_normalize(uint8_t key, uint8_t *value, uint8_t len) {
    (void)len;

    switch (key) {
        case CFG_KEY_TX_POWER_DBM:
            if (value[0] < 2U) {
                value[0] = 2U;
            } else if (value[0] > 17U) {
                value[0] = 17U;
            }
            return CFG_STATUS_OK;

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
