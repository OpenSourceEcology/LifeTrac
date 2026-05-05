#include "config.h"
#include "host_cfg.h"
#include "host_cfg_keys.h"
#include "host_types.h"
#include "sx1276_stub.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct cfg_case_s {
    const char *name;
    uint8_t key;
    uint8_t input[CFG_KEY_MAX_VALUE_LEN];
    uint8_t input_len;
    cfg_status_t expect_status;
    bool expect_get_valid;
    uint8_t expect_get[CFG_KEY_MAX_VALUE_LEN];
    uint8_t expect_get_len;
    bool expect_dirty;
    bool expect_tx_power_call;
    uint8_t expect_tx_power_dbm;
} cfg_case_t;

static uint32_t g_failures;

#define LE16_LO(v) ((uint8_t)((v) & 0xFFU))
#define LE16_HI(v) ((uint8_t)(((v) >> 8) & 0xFFU))
#define LE32_B0(v) ((uint8_t)((v) & 0xFFU))
#define LE32_B1(v) ((uint8_t)(((v) >> 8) & 0xFFU))
#define LE32_B2(v) ((uint8_t)(((v) >> 16) & 0xFFU))
#define LE32_B3(v) ((uint8_t)(((v) >> 24) & 0xFFU))

static void fail_case(const char *name, const char *reason) {
    printf("[FAIL] %s: %s\n", name, reason);
    g_failures++;
}

static bool bytes_equal(const uint8_t *a, const uint8_t *b, uint8_t len) {
    return memcmp(a, b, len) == 0;
}

static void run_case(const cfg_case_t *tc) {
    uint8_t got[CFG_KEY_MAX_VALUE_LEN] = {0};
    uint8_t got_len = 0U;

    cfg_init();
    sx1276_stub_reset();

    {
        const cfg_status_t status = cfg_set(tc->key, tc->input, tc->input_len);
        if (status != tc->expect_status) {
            fail_case(tc->name, "unexpected cfg_set status");
            return;
        }
    }

    if (tc->expect_get_valid) {
        if (cfg_get(tc->key, got, (uint8_t)sizeof(got), &got_len) != CFG_STATUS_OK) {
            fail_case(tc->name, "cfg_get did not return OK");
            return;
        }

        if (got_len != tc->expect_get_len) {
            fail_case(tc->name, "cfg_get length mismatch");
            return;
        }

        if (!bytes_equal(got, tc->expect_get, got_len)) {
            fail_case(tc->name, "cfg_get value mismatch");
            return;
        }
    } else {
        if (cfg_get(tc->key, got, (uint8_t)sizeof(got), &got_len) != CFG_STATUS_UNKNOWN_KEY) {
            fail_case(tc->name, "cfg_get did not report UNKNOWN_KEY");
            return;
        }
    }

    if (cfg_is_dirty() != tc->expect_dirty) {
        fail_case(tc->name, "cfg_dirty mismatch");
        return;
    }

    if (tc->expect_tx_power_call) {
        if (sx1276_stub_tx_power_call_count() != 1U) {
            fail_case(tc->name, "tx power apply call count mismatch");
            return;
        }
        if (sx1276_stub_last_tx_power_dbm() != tc->expect_tx_power_dbm) {
            fail_case(tc->name, "tx power apply value mismatch");
            return;
        }
    } else if (sx1276_stub_tx_power_call_count() != 0U) {
        fail_case(tc->name, "unexpected tx power apply call");
    }
}

static void test_cfg_dirty_live(void) {
    uint8_t dirty = 0U;
    uint8_t dirty_len = 0U;
    uint8_t beacon_ch = 7U;

    cfg_init();
    if (cfg_set(CFG_KEY_BEACON_CHANNEL_IDX, &beacon_ch, 1U) != CFG_STATUS_OK) {
        fail_case("cfg_dirty_live", "beacon cfg_set failed");
        return;
    }

    if (cfg_get(CFG_KEY_CFG_DIRTY, &dirty, 1U, &dirty_len) != CFG_STATUS_OK) {
        fail_case("cfg_dirty_live", "cfg_get cfg_dirty failed");
        return;
    }

    if (dirty_len != 1U || dirty != 1U) {
        fail_case("cfg_dirty_live", "cfg_dirty did not reflect live state");
    }
}

int main(void) {
    static const cfg_case_t k_cases[] = {
        {
            "tx_power_clamp_low",
            CFG_KEY_TX_POWER_DBM,
            { 0U },
            1U,
            CFG_STATUS_OK,
            true,
            { 2U },
            1U,
            true,
            true,
            2U
        },
        {
            "tx_power_clamp_high",
            CFG_KEY_TX_POWER_DBM,
            { 30U },
            1U,
            CFG_STATUS_OK,
            true,
            { 17U },
            1U,
            true,
            true,
            17U
        },
        {
            "tx_power_in_range",
            CFG_KEY_TX_POWER_DBM,
            { 10U },
            1U,
            CFG_STATUS_OK,
            true,
            { 10U },
            1U,
            true,
            true,
            10U
        },
        {
            "tx_power_adapt_bool_oor",
            CFG_KEY_TX_POWER_ADAPT_ENABLE,
            { 2U },
            1U,
            CFG_STATUS_OUT_OF_RANGE,
            true,
            { LORA_FW_TX_POWER_ADAPT },
            1U,
            false,
            false,
            0U
        },
        {
            "lbt_threshold_min_ok",
            CFG_KEY_LBT_THRESHOLD_DBM,
            { (uint8_t)-120 },
            1U,
            CFG_STATUS_OK,
            true,
            { (uint8_t)-120 },
            1U,
            true,
            false,
            0U
        },
        {
            "lbt_threshold_plus1_oor",
            CFG_KEY_LBT_THRESHOLD_DBM,
            { 1U },
            1U,
            CFG_STATUS_OUT_OF_RANGE,
            true,
            { (uint8_t)0xA6U },
            1U,
            false,
            false,
            0U
        },
        {
            "fhss_quality_aware_apply_failed",
            CFG_KEY_FHSS_QUALITY_AWARE,
            { 1U },
            1U,
            CFG_STATUS_APPLY_FAILED,
            true,
            { LORA_FW_QUALITY_AWARE_FHSS },
            1U,
            false,
            false,
            0U
        },
        {
            "fhss_channel_mask_zero_oor",
            CFG_KEY_FHSS_CHANNEL_MASK,
            { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
            8U,
            CFG_STATUS_OUT_OF_RANGE,
            true,
            { 0xFFU, 0U, 0U, 0U, 0U, 0U, 0U, 0U },
            8U,
            false,
            false,
            0U
        },
        {
            "deep_sleep_apply_failed",
            CFG_KEY_DEEP_SLEEP_ENABLE,
            { 0U },
            1U,
            CFG_STATUS_APPLY_FAILED,
            true,
            { LORA_FW_DEEP_SLEEP_BUILD },
            1U,
            false,
            false,
            0U
        },
        {
            "beacon_channel_max_ok",
            CFG_KEY_BEACON_CHANNEL_IDX,
            { 7U },
            1U,
            CFG_STATUS_OK,
            true,
            { 7U },
            1U,
            true,
            false,
            0U
        },
        {
            "beacon_channel_oor",
            CFG_KEY_BEACON_CHANNEL_IDX,
            { 8U },
            1U,
            CFG_STATUS_OUT_OF_RANGE,
            true,
            { 0U },
            1U,
            false,
            false,
            0U
        },
        {
            "host_baud_deferred_stored",
            CFG_KEY_HOST_BAUD,
            { LE32_B0(115200UL), LE32_B1(115200UL), LE32_B2(115200UL), LE32_B3(115200UL) },
            4U,
            CFG_STATUS_DEFERRED,
            true,
            { LE32_B0(115200UL), LE32_B1(115200UL), LE32_B2(115200UL), LE32_B3(115200UL) },
            4U,
            true,
            false,
            0U
        },
        {
            "host_baud_oor",
            CFG_KEY_HOST_BAUD,
            { LE32_B0(19200UL), LE32_B1(19200UL), LE32_B2(19200UL), LE32_B3(19200UL) },
            4U,
            CFG_STATUS_OUT_OF_RANGE,
            true,
            { LE32_B0(HOST_BAUD_DEFAULT), LE32_B1(HOST_BAUD_DEFAULT), LE32_B2(HOST_BAUD_DEFAULT), LE32_B3(HOST_BAUD_DEFAULT) },
            4U,
            false,
            false,
            0U
        },
        {
            "replay_window_oor",
            CFG_KEY_REPLAY_WINDOW,
            { 0U },
            1U,
            CFG_STATUS_OUT_OF_RANGE,
            true,
            { 16U },
            1U,
            false,
            false,
            0U
        },
        {
            "iwdg_window_ok",
            CFG_KEY_IWDG_WINDOW_MS,
            { LE16_LO(5000U), LE16_HI(5000U) },
            2U,
            CFG_STATUS_OK,
            true,
            { LE16_LO(5000U), LE16_HI(5000U) },
            2U,
            true,
            false,
            0U
        },
        {
            "iwdg_window_oor",
            CFG_KEY_IWDG_WINDOW_MS,
            { LE16_LO(49U), LE16_HI(49U) },
            2U,
            CFG_STATUS_OUT_OF_RANGE,
            true,
            { LE16_LO(IWDG_RUN_WINDOW_MS), LE16_HI(IWDG_RUN_WINDOW_MS) },
            2U,
            false,
            false,
            0U
        },
        {
            "crypto_apply_failed",
            CFG_KEY_CRYPTO_IN_L072,
            { 1U },
            1U,
            CFG_STATUS_APPLY_FAILED,
            true,
            { LORA_FW_CRYPTO_IN_L072 },
            1U,
            false,
            false,
            0U
        },
        {
            "protocol_version_read_only",
            CFG_KEY_PROTOCOL_VERSION,
            { 42U },
            1U,
            CFG_STATUS_READ_ONLY,
            true,
            { HOST_PROTOCOL_VER },
            1U,
            false,
            false,
            0U
        },
        {
            "wire_schema_read_only",
            CFG_KEY_WIRE_SCHEMA_VERSION,
            { 42U },
            1U,
            CFG_STATUS_READ_ONLY,
            true,
            { HOST_WIRE_SCHEMA_VER },
            1U,
            false,
            false,
            0U
        },
        {
            "cfg_dirty_read_only",
            CFG_KEY_CFG_DIRTY,
            { 1U },
            1U,
            CFG_STATUS_READ_ONLY,
            true,
            { 0U },
            1U,
            false,
            false,
            0U
        },
        {
            "unknown_key",
            0xFEU,
            { 1U },
            1U,
            CFG_STATUS_UNKNOWN_KEY,
            false,
            { 0U },
            0U,
            false,
            false,
            0U
        },
        {
            "bad_length_precedes_range",
            CFG_KEY_TX_POWER_DBM,
            { 0U, 0U },
            2U,
            CFG_STATUS_BAD_LENGTH,
            true,
            { 14U },
            1U,
            false,
            false,
            0U
        }
    };

    for (size_t i = 0U; i < (sizeof(k_cases) / sizeof(k_cases[0])); ++i) {
        run_case(&k_cases[i]);
    }

    test_cfg_dirty_live();

    if (g_failures != 0U) {
        printf("[FAIL] cfg_contract: %lu failures\n", (unsigned long)g_failures);
        return 1;
    }

    printf("[PASS] cfg_contract: %lu cases\n",
           (unsigned long)((sizeof(k_cases) / sizeof(k_cases[0])) + 1U));
    return 0;
}
