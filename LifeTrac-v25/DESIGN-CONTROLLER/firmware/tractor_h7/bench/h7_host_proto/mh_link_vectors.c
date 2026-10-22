/* H7-4: vectors for the Method G TX gate and RX_FRAME_URC parser
 * (murata_host/mh_link.c). Build: see .github/workflows/arduino-ci.yml. */
#include "mh_link.h"

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

static murata_host_frame_t make_frame(uint8_t type, uint16_t seq,
                                      const uint8_t *payload, uint16_t payload_len) {
    murata_host_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.ver = HOST_PROTOCOL_VER;
    frame.type = type;
    frame.seq = seq;
    frame.payload_len = payload_len;
    if (payload != NULL && payload_len > 0U) {
        memcpy(frame.payload, payload, payload_len);
    }
    return frame;
}

static murata_host_frame_t tx_done(uint8_t tx_id, uint8_t status) {
    const uint8_t p[7] = { tx_id, status, 0x10U, 0x27U, 0U, 0U, 14U };
    return make_frame(HOST_TYPE_TX_DONE_URC, 0U, p, (uint16_t)sizeof(p));
}

static murata_host_frame_t err_proto(uint16_t seq, uint8_t offending_type, uint8_t code) {
    const uint8_t p[5] = { offending_type, HOST_PROTOCOL_VER, code, 0U, 0U };
    return make_frame(HOST_TYPE_ERR_PROTO_URC, seq, p, (uint16_t)sizeof(p));
}

static void submit(mh_link_gate_t *g, uint16_t seq, uint32_t now) {
    mh_link_gate_on_submit(g, mh_link_gate_next_tx_id(g), seq, now, 300U);
}

static void test_tx_done_releases_only_matching_id(void) {
    mh_link_gate_t g;
    murata_host_frame_t f;

    mh_link_gate_init(&g);
    expect_true(!mh_link_gate_busy(&g), "fresh gate is idle");
    submit(&g, 10U, 0U);
    expect_true(mh_link_gate_busy(&g), "submit arms the gate");

    f = tx_done((uint8_t)(g.tx_id + 1U), 0U);
    expect_true(mh_link_gate_on_frame(&g, &f) == MH_LINK_EV_NONE, "other tx_id is ignored");
    expect_true(mh_link_gate_busy(&g), "other tx_id keeps the gate");

    f = tx_done(g.tx_id, 0U);
    expect_true(mh_link_gate_on_frame(&g, &f) == MH_LINK_EV_SENT, "matching TX_DONE OK -> SENT");
    expect_true(!mh_link_gate_busy(&g) && g.sent == 1U, "SENT releases and counts");

    submit(&g, 11U, 0U);
    f = tx_done(g.tx_id, 2U);   /* LBT abort */
    expect_true(mh_link_gate_on_frame(&g, &f) == MH_LINK_EV_ABORTED, "non-zero status -> ABORTED");
    expect_true(!mh_link_gate_busy(&g) && g.aborted == 1U && g.last_status == 2U,
                "ABORTED releases and records the status");
}

static void test_err_proto_releases_only_our_seq(void) {
    mh_link_gate_t g;
    murata_host_frame_t f;

    mh_link_gate_init(&g);
    submit(&g, 42U, 0U);

    f = err_proto(41U, HOST_TYPE_TX_FRAME_REQ, 0x05U);
    expect_true(mh_link_gate_on_frame(&g, &f) == MH_LINK_EV_NONE, "ERR_PROTO for another seq is ignored");
    f = err_proto(42U, HOST_TYPE_CFG_SET_REQ, 0x05U);
    expect_true(mh_link_gate_on_frame(&g, &f) == MH_LINK_EV_NONE, "ERR_PROTO for another type is ignored");
    expect_true(mh_link_gate_busy(&g), "unrelated ERR_PROTO keeps the gate");

    /* Drain-time FORBIDDEN from host_cmd_service_tx_mailbox(). */
    f = err_proto(42U, HOST_TYPE_TX_FRAME_REQ, 0x06U);
    expect_true(mh_link_gate_on_frame(&g, &f) == MH_LINK_EV_REJECTED, "our ERR_PROTO -> REJECTED");
    expect_true(!mh_link_gate_busy(&g) && g.rejected == 1U && g.last_status == 0x06U,
                "REJECTED releases and records err_code");
}

static void test_boot_releases_fault_does_not(void) {
    mh_link_gate_t g;
    murata_host_frame_t f;
    const uint8_t fault[2] = { 0x08U, 0x01U };
    const uint8_t boot[6] = { 1U, 1U, 0x12U, 1U, 2U, 0U };

    mh_link_gate_init(&g);
    submit(&g, 7U, 0U);
    f = make_frame(HOST_TYPE_FAULT_URC, 0U, fault, (uint16_t)sizeof(fault));
    expect_true(mh_link_gate_on_frame(&g, &f) == MH_LINK_EV_NONE, "FAULT_URC releases nothing");
    expect_true(mh_link_gate_busy(&g), "gate held through FAULT_URC");

    f = make_frame(HOST_TYPE_BOOT_URC, 0U, boot, (uint16_t)sizeof(boot));
    expect_true(mh_link_gate_on_frame(&g, &f) == MH_LINK_EV_RESET, "BOOT_URC -> RESET");
    expect_true(!mh_link_gate_busy(&g) && g.reset == 1U, "RESET releases");

    f = make_frame(HOST_TYPE_BOOT_URC, 0U, boot, (uint16_t)sizeof(boot));
    expect_true(mh_link_gate_on_frame(&g, &f) == MH_LINK_EV_NONE, "BOOT_URC while idle is not an event");
}

static void test_timeout_and_late_done(void) {
    mh_link_gate_t g;
    murata_host_frame_t f;
    uint8_t first_id;

    mh_link_gate_init(&g);
    submit(&g, 1U, 0xFFFFFF00U);   /* deadline wraps past zero */
    first_id = g.tx_id;
    expect_true(mh_link_gate_tick(&g, 0xFFFFFF00U + 299U) == MH_LINK_EV_NONE, "no timeout before deadline");
    expect_true(mh_link_gate_tick(&g, 0xFFFFFF00U + 300U) == MH_LINK_EV_TIMEOUT, "timeout at deadline (wrap-safe)");
    expect_true(!mh_link_gate_busy(&g) && g.timeouts == 1U, "TIMEOUT releases and counts");

    submit(&g, 2U, 1000U);
    f = tx_done(first_id, 0U);
    expect_true(mh_link_gate_on_frame(&g, &f) == MH_LINK_EV_NONE, "late TX_DONE of a timed-out frame is ignored");
    expect_true(mh_link_gate_busy(&g), "late TX_DONE does not release the next frame");
}

static void test_tx_id_skips_reserved(void) {
    mh_link_gate_t g;
    int i;
    bool saw_reserved = false;

    mh_link_gate_init(&g);
    for (i = 0; i < 600; i++) {
        if (mh_link_gate_next_tx_id(&g) == 0xFEU) {
            saw_reserved = true;
        }
    }
    expect_true(!saw_reserved, "tx_id never uses 0xFE");
}

static void test_rx_parse(void) {
    uint8_t p[8 + 5 + 8];
    murata_host_frame_t f;
    mh_link_rx_t rx;

    memset(p, 0, sizeof(p));
    p[0] = 5U;                  /* len */
    p[1] = (uint8_t)(int8_t)-7; /* snr dB */
    p[2] = (uint8_t)(-91 & 0xFF);
    p[3] = (uint8_t)((-91 >> 8) & 0xFF);
    p[4] = 0x78U; p[5] = 0x56U; p[6] = 0x34U; p[7] = 0x12U;
    p[8] = 0xC0U; p[9] = 1U; p[10] = 2U; p[11] = 3U; p[12] = 0xC0U;

    f = make_frame(HOST_TYPE_RX_FRAME_URC, 0U, p, 8U + 5U);
    expect_true(mh_link_rx_parse(&f, &rx), "legacy-length RX_FRAME_URC parses");
    expect_true(rx.len == 5U && rx.snr_db == -7 && rx.rssi_dbm == -91 &&
                rx.timestamp_us == 0x12345678U, "RX header fields");
    expect_true(rx.data == &f.payload[8] && rx.data[0] == 0xC0U && rx.data[4] == 0xC0U,
                "RX data points at the packet");

    f = make_frame(HOST_TYPE_RX_FRAME_URC, 0U, p, (uint16_t)sizeof(p));
    expect_true(mh_link_rx_parse(&f, &rx) && rx.len == 5U, "F8 extended RX_FRAME_URC parses");

    f = make_frame(HOST_TYPE_RX_FRAME_URC, 0U, p, 8U + 4U);
    expect_true(!mh_link_rx_parse(&f, &rx), "truncated RX_FRAME_URC is rejected");

    f = make_frame(HOST_TYPE_TX_DONE_URC, 0U, p, (uint16_t)sizeof(p));
    expect_true(!mh_link_rx_parse(&f, &rx), "other frame types are not RX");
}

int main(void) {
    test_tx_done_releases_only_matching_id();
    test_err_proto_releases_only_our_seq();
    test_boot_releases_fault_does_not();
    test_timeout_and_late_done();
    test_tx_id_skips_reserved();
    test_rx_parse();

    if (g_failures != 0) {
        fprintf(stderr, "mh_link_vectors: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("mh_link_vectors: all checks passed\n");
    return 0;
}
