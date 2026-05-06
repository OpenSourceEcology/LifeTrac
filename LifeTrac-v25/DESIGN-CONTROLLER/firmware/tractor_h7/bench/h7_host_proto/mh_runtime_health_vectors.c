#include "mh_runtime_health.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int g_failures;

static void expect_true(bool cond, const char *msg) {
    if (!cond) {
        fprintf(stderr, "[FAIL] %s\n", msg);
        g_failures++;
    }
}

static void put_u32_le(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static murata_host_frame_t make_frame(uint8_t type, const uint8_t *payload, uint16_t payload_len) {
    murata_host_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.ver = HOST_PROTOCOL_VER;
    frame.type = type;
    frame.payload_len = payload_len;
    if (payload != NULL && payload_len > 0U) {
        memcpy(frame.payload, payload, payload_len);
    }
    return frame;
}

static void test_boot_offsets(void) {
    mh_runtime_health_t health;
    const uint8_t payload[6] = {
        9U, /* reset_cause */
        0U, /* radio_ok */
        7U, /* radio_version */
        1U, /* protocol_ver */
        1U, /* wire_schema_ver */
        2U  /* clock_source_id */
    };
    murata_host_frame_t frame = make_frame(HOST_TYPE_BOOT_URC, payload, (uint16_t)sizeof(payload));

    mh_runtime_health_reset(&health);
    expect_true(mh_runtime_health_on_frame(&health, &frame, 1234U), "BOOT_URC should parse");
    expect_true(health.boot_seen, "BOOT_URC should mark boot_seen");
    expect_true(health.boot_radio_ok == 0U, "BOOT_URC radio_ok must come from payload[1]");
    expect_true(health.boot_clock_source_id == 2U, "BOOT_URC clock_source_id must parse");
    expect_true(health.boot_seen_at_ms == 1234U, "BOOT_URC timestamp must be recorded");
}

static void test_tx_done_and_rx_frame(void) {
    mh_runtime_health_t health;
    const uint8_t tx_done_payload[7] = {
        0x34U,
        0x01U,
        0x78U,
        0x56U,
        0x34U,
        0x12U,
        (uint8_t)0xFD /* -3 dBm */
    };
    const uint8_t rx_payload[11] = {
        3U,
        (uint8_t)0xFE, /* -2 dB SNR */
        0xA9U,
        0xFFU,         /* -87 dBm RSSI */
        0x04U,
        0x03U,
        0x02U,
        0x01U,
        0xAAU,
        0xBBU,
        0xCCU
    };
    murata_host_frame_t tx_done = make_frame(HOST_TYPE_TX_DONE_URC,
                                             tx_done_payload,
                                             (uint16_t)sizeof(tx_done_payload));
    murata_host_frame_t rx = make_frame(HOST_TYPE_RX_FRAME_URC,
                                        rx_payload,
                                        (uint16_t)sizeof(rx_payload));

    mh_runtime_health_reset(&health);
    expect_true(mh_runtime_health_on_frame(&health, &tx_done, 2000U), "TX_DONE_URC should parse");
    expect_true(health.tx_done_count == 1U, "TX_DONE counter should increment");
    expect_true(health.last_tx_id == 0x34U, "TX_DONE tx_id should parse");
    expect_true(health.last_tx_status == 0x01U, "TX_DONE status should parse");
    expect_true(health.last_tx_time_on_air_us == 0x12345678UL, "TX_DONE time_on_air should parse");
    expect_true(health.last_tx_power_dbm == -3, "TX_DONE power should parse as signed");

    expect_true(mh_runtime_health_on_frame(&health, &rx, 2100U), "RX_FRAME_URC should parse");
    expect_true(health.rx_frame_count == 1U, "RX_FRAME counter should increment");
    expect_true(health.last_rx_len == 3U, "RX_FRAME len should parse");
    expect_true(health.last_rx_snr_db == -2, "RX_FRAME SNR should parse as signed");
    expect_true(health.last_rx_rssi_dbm == -87, "RX_FRAME RSSI should parse as signed");
    expect_true(health.last_rx_timestamp_us == 0x01020304UL, "RX_FRAME timestamp should parse");
    expect_true(health.last_rx_payload[0] == 0xAAU &&
                health.last_rx_payload[1] == 0xBBU &&
                health.last_rx_payload[2] == 0xCCU,
                "RX_FRAME payload bytes should copy");
}

static void test_stats_compatibility(void) {
    mh_runtime_health_t health;
    uint8_t legacy[HOST_STATS_LEGACY_PAYLOAD_LEN];
    uint8_t additive[HOST_STATS_PAYLOAD_LEN];
    murata_host_frame_t legacy_frame;
    murata_host_frame_t additive_frame;

    memset(legacy, 0, sizeof(legacy));
    memset(additive, 0, sizeof(additive));

    for (uint16_t off = 0; off <= HOST_STATS_LEGACY_OFFSET_RADIO_STATE; off = (uint16_t)(off + 4U)) {
        put_u32_le(&legacy[off], (uint32_t)(1000U + off));
    }
    for (uint16_t off = 0; off <= HOST_STATS_OFFSET_RADIO_STATE; off = (uint16_t)(off + 4U)) {
        put_u32_le(&additive[off], (uint32_t)(2000U + off));
    }

    legacy_frame = make_frame(HOST_TYPE_STATS_URC, legacy, (uint16_t)sizeof(legacy));
    additive_frame = make_frame(HOST_TYPE_STATS_URC, additive, (uint16_t)sizeof(additive));

    mh_runtime_health_reset(&health);
    expect_true(mh_runtime_health_on_frame(&health, &legacy_frame, 3000U), "Legacy STATS should parse");
    expect_true(health.has_stats, "Legacy STATS should set has_stats");
    expect_true(health.last_stats_len == HOST_STATS_LEGACY_PAYLOAD_LEN, "Legacy STATS len should record");
    expect_true(!health.last_stats.has_tx_abort_airtime, "Legacy STATS should report no airtime field");

    expect_true(mh_runtime_health_on_frame(&health, &additive_frame, 3100U), "Additive STATS should parse");
    expect_true(health.last_stats_len == HOST_STATS_PAYLOAD_LEN, "Additive STATS len should record");
    expect_true(health.last_stats.has_tx_abort_airtime, "Additive STATS should report airtime field");
}

static void test_ver_urc(void) {
    mh_runtime_health_t health;
    const char name[] = "lifetraclora-l072";
    const char git[] = "deadbeef";
    const uint8_t name_len = (uint8_t)(sizeof(name) - 1U);
    const uint8_t git_len = (uint8_t)(sizeof(git) - 1U);
    uint8_t payload[12 + 40 + 16];
    uint16_t idx = 0U;
    murata_host_frame_t frame;

    memset(payload, 0, sizeof(payload));
    payload[idx++] = HOST_PROTOCOL_VER;
    payload[idx++] = HOST_WIRE_SCHEMA_VER;
    payload[idx++] = 1U;
    payload[idx++] = 2U;
    payload[idx++] = 3U;
    payload[idx++] = name_len;
    payload[idx++] = git_len;
    payload[idx++] = 0U;
    put_u32_le(&payload[idx], 0x000000FFUL);
    idx = (uint16_t)(idx + 4U);
    memcpy(&payload[idx], name, name_len);
    idx = (uint16_t)(idx + name_len);
    memcpy(&payload[idx], git, git_len);
    idx = (uint16_t)(idx + git_len);

    frame = make_frame(HOST_TYPE_VER_URC, payload, idx);

    mh_runtime_health_reset(&health);
    expect_true(mh_runtime_health_on_frame(&health, &frame, 5000U), "VER_URC should parse");
    expect_true(health.ver_seen, "VER_URC should set ver_seen");
    expect_true(health.ver_count == 1U, "VER_URC count should increment");
    expect_true(health.ver_protocol_ver == HOST_PROTOCOL_VER, "VER_URC protocol must parse");
    expect_true(health.ver_wire_schema_ver == HOST_WIRE_SCHEMA_VER, "VER_URC wire must parse");
    expect_true(health.ver_fw_major == 1U && health.ver_fw_minor == 2U && health.ver_fw_patch == 3U,
                "VER_URC fw triplet must parse");
    expect_true(health.ver_capability_bitmap == 0x000000FFUL, "VER_URC capability bitmap must parse");
    expect_true(health.ver_fw_name_len == name_len, "VER_URC name_len must parse");
    expect_true(health.ver_fw_git_len == git_len, "VER_URC git_len must parse");
    expect_true(strcmp(health.ver_fw_name, name) == 0, "VER_URC name must copy and null-terminate");
    expect_true(strcmp(health.ver_fw_git, git) == 0, "VER_URC git must copy and null-terminate");
    expect_true(health.ver_seen_at_ms == 5000U, "VER_URC timestamp must be recorded");
}

static void test_ver_urc_malformed(void) {
    mh_runtime_health_t health;
    uint8_t short_payload[8] = {0};
    uint8_t bad_lengths[12] = {1U, 1U, 0U, 0U, 0U, 41U, 0U, 0U, 0U, 0U, 0U, 0U};
    murata_host_frame_t too_short = make_frame(HOST_TYPE_VER_URC,
                                               short_payload,
                                               (uint16_t)sizeof(short_payload));
    murata_host_frame_t bad = make_frame(HOST_TYPE_VER_URC,
                                         bad_lengths,
                                         (uint16_t)sizeof(bad_lengths));

    mh_runtime_health_reset(&health);
    expect_true(!mh_runtime_health_on_frame(&health, &too_short, 5100U),
                "VER_URC shorter than header must reject");
    expect_true(!mh_runtime_health_on_frame(&health, &bad, 5110U),
                "VER_URC name_len > 40 must reject");
    expect_true(!health.ver_seen, "Malformed VER_URC must not set ver_seen");
}

static void test_rejects(void) {
    mh_runtime_health_t health;
    const uint8_t bad_rx_payload[8] = {3U, 0U, 0U, 0U, 0U, 0U, 0U, 0x11U};
    murata_host_frame_t unknown = make_frame(0x55U, NULL, 0U);
    murata_host_frame_t bad_rx = make_frame(HOST_TYPE_RX_FRAME_URC,
                                            bad_rx_payload,
                                            (uint16_t)sizeof(bad_rx_payload));

    mh_runtime_health_reset(&health);
    expect_true(!mh_runtime_health_on_frame(&health, &unknown, 4000U), "Unknown frame type should reject");
    expect_true(!mh_runtime_health_on_frame(&health, &bad_rx, 4010U), "Malformed RX_FRAME should reject");
}

int main(void) {
    test_boot_offsets();
    test_tx_done_and_rx_frame();
    test_stats_compatibility();
    test_ver_urc();
    test_ver_urc_malformed();
    test_rejects();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] mh_runtime_health_vectors: %d failures\n", g_failures);
        return 1;
    }

    printf("[PASS] mh_runtime_health_vectors: 6 vectors\n");
    return 0;
}