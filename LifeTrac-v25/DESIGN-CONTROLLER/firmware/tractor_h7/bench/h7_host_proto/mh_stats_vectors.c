#include "mh_stats.h"
#include "mh_wire.h"
#include "murata_host.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t g_failures;

static void fail_case(const char *name, const char *reason) {
    printf("[FAIL] %s: %s\n", name, reason);
    g_failures++;
}

static void put_u32_le(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void build_legacy_payload(uint8_t *out64) {
    memset(out64, 0, HOST_STATS_LEGACY_PAYLOAD_LEN);

    put_u32_le(&out64[HOST_STATS_OFFSET_HOST_DROPPED], 1U);
    put_u32_le(&out64[HOST_STATS_OFFSET_HOST_ERRORS], 2U);
    put_u32_le(&out64[HOST_STATS_OFFSET_HOST_QUEUE_FULL], 3U);
    put_u32_le(&out64[HOST_STATS_OFFSET_HOST_IRQ_IDLE], 4U);
    put_u32_le(&out64[HOST_STATS_OFFSET_HOST_IRQ_HT], 5U);
    put_u32_le(&out64[HOST_STATS_OFFSET_HOST_IRQ_TC], 6U);
    put_u32_le(&out64[HOST_STATS_OFFSET_HOST_IRQ_TE], 7U);
    put_u32_le(&out64[HOST_STATS_OFFSET_RADIO_DIO0], 8U);
    put_u32_le(&out64[HOST_STATS_OFFSET_RADIO_DIO1], 9U);
    put_u32_le(&out64[HOST_STATS_OFFSET_RADIO_DIO2], 10U);
    put_u32_le(&out64[HOST_STATS_OFFSET_RADIO_DIO3], 11U);
    put_u32_le(&out64[HOST_STATS_OFFSET_RADIO_CRC_ERR], 12U);
    put_u32_le(&out64[HOST_STATS_OFFSET_RADIO_RX_OK], 13U);
    put_u32_le(&out64[HOST_STATS_OFFSET_RADIO_TX_OK], 14U);
    put_u32_le(&out64[HOST_STATS_OFFSET_RADIO_TX_ABORT_LBT], 15U);
    put_u32_le(&out64[HOST_STATS_LEGACY_OFFSET_RADIO_STATE], 16U);
}

static void build_extended_payload(uint8_t *out68) {
    memset(out68, 0, HOST_STATS_PAYLOAD_LEN);

    put_u32_le(&out68[HOST_STATS_OFFSET_HOST_DROPPED], 101U);
    put_u32_le(&out68[HOST_STATS_OFFSET_HOST_ERRORS], 102U);
    put_u32_le(&out68[HOST_STATS_OFFSET_HOST_QUEUE_FULL], 103U);
    put_u32_le(&out68[HOST_STATS_OFFSET_HOST_IRQ_IDLE], 104U);
    put_u32_le(&out68[HOST_STATS_OFFSET_HOST_IRQ_HT], 105U);
    put_u32_le(&out68[HOST_STATS_OFFSET_HOST_IRQ_TC], 106U);
    put_u32_le(&out68[HOST_STATS_OFFSET_HOST_IRQ_TE], 107U);
    put_u32_le(&out68[HOST_STATS_OFFSET_RADIO_DIO0], 108U);
    put_u32_le(&out68[HOST_STATS_OFFSET_RADIO_DIO1], 109U);
    put_u32_le(&out68[HOST_STATS_OFFSET_RADIO_DIO2], 110U);
    put_u32_le(&out68[HOST_STATS_OFFSET_RADIO_DIO3], 111U);
    put_u32_le(&out68[HOST_STATS_OFFSET_RADIO_CRC_ERR], 112U);
    put_u32_le(&out68[HOST_STATS_OFFSET_RADIO_RX_OK], 113U);
    put_u32_le(&out68[HOST_STATS_OFFSET_RADIO_TX_OK], 114U);
    put_u32_le(&out68[HOST_STATS_OFFSET_RADIO_TX_ABORT_LBT], 115U);
    put_u32_le(&out68[HOST_STATS_OFFSET_RADIO_TX_ABORT_AIRTIME], 116U);
    put_u32_le(&out68[HOST_STATS_OFFSET_RADIO_STATE], 117U);
}

static void test_stats_legacy_64(void) {
    uint8_t payload[HOST_STATS_LEGACY_PAYLOAD_LEN];
    mh_stats_t parsed;

    build_legacy_payload(payload);

    if (!mh_stats_parse(payload, (uint16_t)sizeof(payload), &parsed)) {
        fail_case("stats_legacy_64", "parse failed");
        return;
    }

    if (parsed.host_dropped != 1U || parsed.host_errors != 2U || parsed.radio_tx_ok != 14U) {
        fail_case("stats_legacy_64", "core field mismatch");
        return;
    }

    if (parsed.radio_state != 16U) {
        fail_case("stats_legacy_64", "legacy radio_state mismatch");
        return;
    }

    if (parsed.has_tx_abort_airtime) {
        fail_case("stats_legacy_64", "legacy payload incorrectly marked as extended");
        return;
    }

    if (parsed.radio_tx_abort_airtime != 0U) {
        fail_case("stats_legacy_64", "legacy payload airtime abort should be zero");
    }
}

static void test_stats_extended_68(void) {
    uint8_t payload[HOST_STATS_PAYLOAD_LEN];
    mh_stats_t parsed;

    build_extended_payload(payload);

    if (!mh_stats_parse(payload, (uint16_t)sizeof(payload), &parsed)) {
        fail_case("stats_extended_68", "parse failed");
        return;
    }

    if (parsed.host_dropped != 101U || parsed.host_errors != 102U || parsed.radio_tx_ok != 114U) {
        fail_case("stats_extended_68", "core field mismatch");
        return;
    }

    if (parsed.radio_tx_abort_airtime != 116U) {
        fail_case("stats_extended_68", "airtime abort field mismatch");
        return;
    }

    if (parsed.radio_state != 117U) {
        fail_case("stats_extended_68", "extended radio_state mismatch");
        return;
    }

    if (!parsed.has_tx_abort_airtime) {
        fail_case("stats_extended_68", "extended payload not marked");
    }
}

static void test_stats_too_short_rejected(void) {
    uint8_t payload[63];
    mh_stats_t parsed;

    memset(payload, 0, sizeof(payload));
    if (mh_stats_parse(payload, (uint16_t)sizeof(payload), &parsed)) {
        fail_case("stats_too_short_rejected", "short payload should fail");
    }
}

static void test_stats_urc_inner_frame_legacy_and_extended(void) {
    uint8_t payload64[HOST_STATS_LEGACY_PAYLOAD_LEN];
    uint8_t payload68[HOST_STATS_PAYLOAD_LEN];
    uint8_t inner[HOST_MAX_INNER_LEN];
    uint16_t inner_len;
    murata_host_frame_t frame;
    mh_stats_t parsed;

    build_legacy_payload(payload64);
    inner_len = murata_host_build_inner_frame(HOST_TYPE_STATS_URC,
                                              0U,
                                              0x1234U,
                                              payload64,
                                              (uint16_t)sizeof(payload64),
                                              inner,
                                              (uint16_t)sizeof(inner));
    if (inner_len == 0U) {
        fail_case("stats_urc_inner_legacy", "failed to build inner frame");
        return;
    }
    if (!murata_host_parse_inner_frame(inner, inner_len, &frame)) {
        fail_case("stats_urc_inner_legacy", "failed to parse inner frame");
        return;
    }
    if (frame.type != HOST_TYPE_STATS_URC || frame.payload_len != HOST_STATS_LEGACY_PAYLOAD_LEN) {
        fail_case("stats_urc_inner_legacy", "frame metadata mismatch");
        return;
    }
    if (!mh_stats_parse(frame.payload, frame.payload_len, &parsed)) {
        fail_case("stats_urc_inner_legacy", "payload parse failed");
        return;
    }
    if (parsed.radio_state != 16U || parsed.has_tx_abort_airtime) {
        fail_case("stats_urc_inner_legacy", "parsed legacy data mismatch");
        return;
    }

    build_extended_payload(payload68);
    inner_len = murata_host_build_inner_frame(HOST_TYPE_STATS_URC,
                                              0U,
                                              0x2345U,
                                              payload68,
                                              (uint16_t)sizeof(payload68),
                                              inner,
                                              (uint16_t)sizeof(inner));
    if (inner_len == 0U) {
        fail_case("stats_urc_inner_extended", "failed to build inner frame");
        return;
    }
    if (!murata_host_parse_inner_frame(inner, inner_len, &frame)) {
        fail_case("stats_urc_inner_extended", "failed to parse inner frame");
        return;
    }
    if (frame.type != HOST_TYPE_STATS_URC || frame.payload_len != HOST_STATS_PAYLOAD_LEN) {
        fail_case("stats_urc_inner_extended", "frame metadata mismatch");
        return;
    }
    if (!mh_stats_parse(frame.payload, frame.payload_len, &parsed)) {
        fail_case("stats_urc_inner_extended", "payload parse failed");
        return;
    }
    if (parsed.radio_tx_abort_airtime != 116U || parsed.radio_state != 117U || !parsed.has_tx_abort_airtime) {
        fail_case("stats_urc_inner_extended", "parsed extended data mismatch");
    }
}

int main(void) {
    test_stats_legacy_64();
    test_stats_extended_68();
    test_stats_too_short_rejected();
    test_stats_urc_inner_frame_legacy_and_extended();

    if (g_failures != 0U) {
        printf("[FAIL] mh_stats_vectors: %lu failures\n", (unsigned long)g_failures);
        return 1;
    }

    printf("[PASS] mh_stats_vectors: 4 vectors\n");
    return 0;
}
