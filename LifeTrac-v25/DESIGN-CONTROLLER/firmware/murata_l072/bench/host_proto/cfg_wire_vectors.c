#include "host_cfg.h"
#include "host_cfg_keys.h"
#include "host_cfg_wire.h"
#include "host_types.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t g_failures;

static void fail_case(const char *name, const char *reason) {
    printf("[FAIL] %s: %s\n", name, reason);
    g_failures++;
}

static bool build_cfg_set_urc(const uint8_t *req,
                              uint8_t req_len,
                              uint8_t *out_type,
                              uint8_t *out_payload,
                              uint8_t *out_len) {
    uint8_t key;
    uint8_t value_len;
    cfg_status_t status;
    uint8_t actual_len = 0U;
    uint8_t scratch[CFG_KEY_MAX_VALUE_LEN] = {0};

    if (req == NULL || out_type == NULL || out_payload == NULL || out_len == NULL || req_len < 2U) {
        return false;
    }

    key = req[0];
    value_len = req[1];
    if (req_len != (uint8_t)(2U + value_len)) {
        return false;
    }

    status = cfg_set(key, &req[2], value_len);
    if (status == CFG_STATUS_OK ||
        status == CFG_STATUS_DEFERRED ||
        status == CFG_STATUS_APPLY_FAILED) {
        if (cfg_get(key, scratch, (uint8_t)sizeof(scratch), &actual_len) != CFG_STATUS_OK) {
            actual_len = 0U;
        }
    }

    if (!host_cfg_wire_encode_ok(key,
                                 status,
                                 actual_len,
                                 out_payload,
                                 HOST_CFG_OK_PAYLOAD_LEN,
                                 out_len)) {
        return false;
    }

    *out_type = HOST_TYPE_CFG_OK_URC;
    return true;
}

static bool build_cfg_get_urc(const uint8_t *req,
                              uint8_t req_len,
                              uint8_t *out_type,
                              uint8_t *out_payload,
                              uint8_t *out_len) {
    uint8_t key;
    cfg_status_t status;
    uint8_t value_len = 0U;

    if (req == NULL || out_type == NULL || out_payload == NULL || out_len == NULL || req_len != 1U) {
        return false;
    }

    key = req[0];
    status = cfg_get(key,
                     &out_payload[HOST_CFG_DATA_HEADER_LEN],
                     (uint8_t)(HOST_CFG_DATA_PAYLOAD_MAX_LEN - HOST_CFG_DATA_HEADER_LEN),
                     &value_len);
    if (status == CFG_STATUS_OK) {
        if (!host_cfg_wire_encode_data(key,
                                       &out_payload[HOST_CFG_DATA_HEADER_LEN],
                                       value_len,
                                       out_payload,
                                       HOST_CFG_DATA_PAYLOAD_MAX_LEN,
                                       out_len)) {
            return false;
        }
        *out_type = HOST_TYPE_CFG_DATA_URC;
        return true;
    }

    if (!host_cfg_wire_encode_ok(key,
                                 status,
                                 0U,
                                 out_payload,
                                 HOST_CFG_OK_PAYLOAD_LEN,
                                 out_len)) {
        return false;
    }

    *out_type = HOST_TYPE_CFG_OK_URC;
    return true;
}

static void assert_vector(const char *name,
                          bool ok,
                          uint8_t got_type,
                          const uint8_t *got_payload,
                          uint8_t got_len,
                          uint8_t exp_type,
                          const uint8_t *exp_payload,
                          uint8_t exp_len) {
    if (!ok) {
        fail_case(name, "builder returned false");
        return;
    }

    if (got_type != exp_type) {
        fail_case(name, "URC type mismatch");
        return;
    }

    if (got_len != exp_len) {
        fail_case(name, "URC payload length mismatch");
        return;
    }

    if (memcmp(got_payload, exp_payload, exp_len) != 0) {
        fail_case(name, "URC payload bytes mismatch");
    }
}

int main(void) {
    bool ok;
    uint8_t out_type = 0U;
    uint8_t out_payload[HOST_CFG_DATA_PAYLOAD_MAX_LEN] = {0};
    uint8_t out_len = 0U;

    static const uint8_t k_get_tx_power_req[] = { CFG_KEY_TX_POWER_DBM };
    static const uint8_t k_get_tx_power_urc[] = { CFG_KEY_TX_POWER_DBM, 0x01U, 0x0EU };

    static const uint8_t k_set_clamp_req[] = { CFG_KEY_TX_POWER_DBM, 0x01U, 0x12U };
    static const uint8_t k_set_clamp_urc[] = { CFG_KEY_TX_POWER_DBM, HOST_CFG_STATUS_OK, 0x01U, 0x00U };

    static const uint8_t k_set_ro_req[] = { CFG_KEY_PROTOCOL_VERSION, 0x01U, 0x2AU };
    static const uint8_t k_set_ro_urc[] = { CFG_KEY_PROTOCOL_VERSION, HOST_CFG_STATUS_READ_ONLY, 0x00U, 0x00U };

    static const uint8_t k_set_baud_req[] = {
        CFG_KEY_HOST_BAUD,
        0x04U,
        0x00U, 0xC2U, 0x01U, 0x00U
    };
    static const uint8_t k_set_baud_urc[] = { CFG_KEY_HOST_BAUD, HOST_CFG_STATUS_DEFERRED, 0x04U, 0x00U };

    cfg_init();
    ok = build_cfg_get_urc(k_get_tx_power_req,
                           (uint8_t)sizeof(k_get_tx_power_req),
                           &out_type,
                           out_payload,
                           &out_len);
    assert_vector("get_tx_power_default",
                  ok,
                  out_type,
                  out_payload,
                  out_len,
                  HOST_TYPE_CFG_DATA_URC,
                  k_get_tx_power_urc,
                  (uint8_t)sizeof(k_get_tx_power_urc));

    cfg_init();
    ok = build_cfg_set_urc(k_set_clamp_req,
                           (uint8_t)sizeof(k_set_clamp_req),
                           &out_type,
                           out_payload,
                           &out_len);
    assert_vector("set_tx_power_clamp",
                  ok,
                  out_type,
                  out_payload,
                  out_len,
                  HOST_TYPE_CFG_OK_URC,
                  k_set_clamp_urc,
                  (uint8_t)sizeof(k_set_clamp_urc));

    cfg_init();
    ok = build_cfg_set_urc(k_set_ro_req,
                           (uint8_t)sizeof(k_set_ro_req),
                           &out_type,
                           out_payload,
                           &out_len);
    assert_vector("set_protocol_read_only",
                  ok,
                  out_type,
                  out_payload,
                  out_len,
                  HOST_TYPE_CFG_OK_URC,
                  k_set_ro_urc,
                  (uint8_t)sizeof(k_set_ro_urc));

    cfg_init();
    ok = build_cfg_set_urc(k_set_baud_req,
                           (uint8_t)sizeof(k_set_baud_req),
                           &out_type,
                           out_payload,
                           &out_len);
    assert_vector("set_host_baud_deferred",
                  ok,
                  out_type,
                  out_payload,
                  out_len,
                  HOST_TYPE_CFG_OK_URC,
                  k_set_baud_urc,
                  (uint8_t)sizeof(k_set_baud_urc));

    if (g_failures != 0U) {
        printf("[FAIL] cfg_wire_vectors: %lu failures\n", (unsigned long)g_failures);
        return 1;
    }

    printf("[PASS] cfg_wire_vectors: 4 vectors\n");
    return 0;
}
