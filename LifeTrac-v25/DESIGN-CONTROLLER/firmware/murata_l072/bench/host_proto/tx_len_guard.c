/*
 * C1/F7 regression pin for sx1276_tx_begin length guard
 * ======================================================
 *
 * sx1276_tx_begin() must refuse req->length + LORA_PKT_HDR_LEN > 255
 * BEFORE any FIFO write or scheduler side effect. This pins the in-tree
 * guard so a refactor cannot resurrect the narrowing-cast overflow.
 */

#include "sx1276_tx.h"
#include "sx1276_stub.h"
#include "platform.h"
#include "host_stats.h"
#include "host_cfg_profile.h"
#include "lora_pkt_hdr.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>   /* RS-11.5 stub: memset in sx1276_read_burst */
#include <stdint.h>

#define CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            printf("[FAIL] %s (line %d)\n", msg, __LINE__); \
            return 1; \
        } \
    } while (0)

#include "sx1276_lbt.h"

/* Minimal stubs required by sx1276_tx.c for host unit test */
uint32_t platform_now_us(void) { return 1000000UL; }
uint32_t platform_now_ms(void) { return 1000UL; }
uint8_t sx1276_read_reg(uint8_t reg_addr) {
    if (reg_addr == 0x1DU) return 0x82U; /* BW250, CR4/5 */
    if (reg_addr == 0x1EU) return 0x74U; /* SF7, CRC on */
    if (reg_addr == 0x21U) return 8U;    /* Preamble 8 */
    return 0U;
}
void sx1276_write_reg(uint8_t reg_addr, uint8_t value) { (void)reg_addr; (void)value; }
void sx1276_write_burst(uint8_t reg_addr, const uint8_t *buf, uint8_t len) { (void)reg_addr; (void)buf; (void)len; }
/* RS-11.5: readback returns zeros; on-host the checksum check is inert
 * (load ck over zero-writes vs read ck over zeros both start at the DJB
 * seed only when length is 0 — the guard test never reaches TX_DONE). */
void sx1276_read_burst(uint8_t start_reg, uint8_t *dst, size_t len) {
    (void)start_reg;
    memset(dst, 0, len);
}
bool sx1276_lbt_check_clear(uint32_t freq_hz, uint32_t toa_us) { (void)freq_hz; (void)toa_us; return true; }
sx1276_lbt_result_t sx1276_lbt_check_and_backoff(void) { return SX1276_LBT_RESULT_CLEAR; }
void host_stats_radio_tx_ok(void) {}
void host_stats_radio_tx_abort_airtime(void) {}
void host_stats_radio_tx_ok_inc(void) {}
void host_stats_radio_tx_abort_airtime_inc(void) {}
void host_stats_radio_tx_abort_lbt_inc(void) {}
/* RS-11.5 TX FIFO readback discriminator stubs. */
void host_stats_tx_fifo_rb_ok(void) {}
void host_stats_tx_fifo_rb_bad(void) {}
void host_stats_tx_done_early(void) {}
void host_cmd_emit_fault(uint8_t code, uint8_t sub) { (void)code; (void)sub; }
const host_cfg_profile_req_t *host_cfg_profile_active(void) { return NULL; }
/* sx1276_modes_to_standby / sx1276_set_sf_bw_cr / sx1276_set_tx_power_dbm
 * come from bench/host_proto/sx1276_stub.c — do not redefine here. */
bool sx1276_modes_to_tx(void) { return true; }
uint8_t sx1276_modes_get_state(void) { return 0U; }
void sx1276_rx_arm(uint8_t mode) { (void)mode; }
void sx1276_rx_disarm(void) {}
void sx1276_set_frequency_hz(uint32_t freq) { (void)freq; }
bool lora_pkt_hdr_pack(const lora_pkt_hdr_t *in, uint8_t *out) {
    (void)in; (void)out; return true;
}
void host_uart_send_urc(uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t len) {
    (void)type; (void)seq; (void)payload; (void)len;
}

int main(void) {
    sx1276_stub_reset();
    sx1276_tx_request_t req = {0};
    req.tx_id = 0x42U;

    req.length = 248U;                       /* 248+8 = 256 > 255: refuse */
    CHECK(!sx1276_tx_begin(&req), "len=248 must be refused");

    req.length = 247U;                       /* 247+8 = 255: exactly legal */
    CHECK(sx1276_tx_begin(&req), "len=247 must be admitted");
    printf("PASS tx_len_guard\n");
    return 0;
}
