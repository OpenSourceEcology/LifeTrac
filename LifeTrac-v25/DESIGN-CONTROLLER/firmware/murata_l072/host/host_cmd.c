#include "host_cmd.h"

#include "config.h"
#include "host_cfg.h"
#include "host_cfg_wire.h"
#include "host_stats.h"
#include "host_types.h"
#include "platform.h"
#include "sx1276.h"
#include "stm32l072_regs.h"
#include "version.h"

#include <stddef.h>
#include <string.h>

#define STM32_UID_BASE               0x1FF80050UL

static bool s_radio_ok;
static uint8_t s_radio_version;

#if HOST_AT_SHELL_ENABLE
static char at_upper(char ch) {
    if (ch >= 'a' && ch <= 'z') {
        return (char)(ch - ('a' - 'A'));
    }

    return ch;
}

static bool at_line_equals(const char *line, uint16_t len, const char *cmd) {
    uint16_t start = 0U;
    uint16_t end = len;
    uint16_t idx = 0U;

    while (start < end && (line[start] == ' ' || line[start] == '\t')) {
        start++;
    }

    while (end > start && (line[end - 1U] == ' ' || line[end - 1U] == '\t')) {
        end--;
    }

    while (cmd[idx] != '\0') {
        if ((uint16_t)(start + idx) >= end) {
            return false;
        }

        if (at_upper(line[start + idx]) != at_upper(cmd[idx])) {
            return false;
        }

        idx++;
    }

    return (uint16_t)(start + idx) == end;
}

static void at_send_u32(uint32_t value) {
    char buf[10];
    uint8_t idx = 0U;

    if (value == 0UL) {
        host_uart_send_ascii("0");
        return;
    }

    while (value != 0UL && idx < (uint8_t)sizeof(buf)) {
        buf[idx++] = (char)('0' + (value % 10UL));
        value /= 10UL;
    }

    while (idx > 0U) {
        char out[2];
        out[0] = buf[--idx];
        out[1] = '\0';
        host_uart_send_ascii(out);
    }
}

static void at_send_hex_nibble(uint8_t value) {
    char out[2];
    value &= 0x0FU;
    if (value < 10U) {
        out[0] = (char)((uint8_t)'0' + value);
    } else {
        out[0] = (char)((uint8_t)'A' + (value - 10U));
    }
    out[1] = '\0';
    host_uart_send_ascii(out);
}

static void at_send_hex_u8(uint8_t value) {
    at_send_hex_nibble((uint8_t)(value >> 4));
    at_send_hex_nibble(value);
}

static void at_send_hex_u32(uint32_t value) {
    for (int8_t shift = 28; shift >= 0; shift = (int8_t)(shift - 4)) {
        at_send_hex_nibble((uint8_t)(value >> (uint8_t)shift));
    }
}

static uint32_t at_get_u32_le(const uint8_t *src, uint16_t offset) {
    return (uint32_t)src[offset] |
           ((uint32_t)src[offset + 1U] << 8) |
           ((uint32_t)src[offset + 2U] << 16) |
           ((uint32_t)src[offset + 3U] << 24);
}

static void at_send_key_u32(const char *key, uint32_t value) {
    host_uart_send_ascii(key);
    host_uart_send_ascii("=");
    at_send_u32(value);
}

static void at_send_ok(void) {
    host_uart_send_ascii("OK\r\n");
}

static void at_send_error(void) {
    host_uart_send_ascii("ERROR\r\n");
}

static void at_send_version_triplet(void) {
    at_send_u32(FW_VERSION_MAJOR);
    host_uart_send_ascii(".");
    at_send_u32(FW_VERSION_MINOR);
    host_uart_send_ascii(".");
    at_send_u32(FW_VERSION_PATCH);
}

static void at_handle_identity(void) {
    host_uart_send_ascii(OSE_LIFETRACLORA_FW_NAME);
    host_uart_send_ascii(" ");
    at_send_version_triplet();
    host_uart_send_ascii(" ");
    host_uart_send_ascii(FW_GIT_SHA);
    host_uart_send_ascii("\r\n");
    at_send_ok();
}

static void at_handle_version(void) {
    host_uart_send_ascii("FW=");
    host_uart_send_ascii(OSE_LIFETRACLORA_FW_NAME);
    host_uart_send_ascii(" VERSION=");
    at_send_version_triplet();
    host_uart_send_ascii(" GIT=");
    host_uart_send_ascii(FW_GIT_SHA);
    host_uart_send_ascii(" PROTO=");
    at_send_u32(HOST_PROTOCOL_VER);
    host_uart_send_ascii(" SCHEMA=");
    at_send_u32(HOST_WIRE_SCHEMA_VER);
    host_uart_send_ascii(" CAP=0x");
    at_send_hex_u32(HOST_CAPABILITY_BITMAP);
    host_uart_send_ascii("\r\n");
    at_send_ok();
}

static bool at_take_stats(uint8_t *stats) {
    return host_stats_serialize(stats, HOST_STATS_PAYLOAD_LEN) == HOST_STATS_PAYLOAD_LEN;
}

static void at_handle_radio(void) {
    uint8_t stats[HOST_STATS_PAYLOAD_LEN];

    if (!at_take_stats(stats)) {
        at_send_error();
        return;
    }

    host_uart_send_ascii("RADIO=");
    host_uart_send_ascii(s_radio_ok ? "OK" : "FAIL");
    host_uart_send_ascii(" VERSION=0x");
    at_send_hex_u8(s_radio_version);
    host_uart_send_ascii(" STATE=");
    at_send_u32(at_get_u32_le(stats, HOST_STATS_OFFSET_RADIO_STATE));
    host_uart_send_ascii("\r\n");
    at_send_ok();
}

static void at_handle_stats(void) {
    uint8_t stats[HOST_STATS_PAYLOAD_LEN];

    if (!at_take_stats(stats)) {
        at_send_error();
        return;
    }

    at_send_key_u32("HOST_DROPPED", at_get_u32_le(stats, HOST_STATS_OFFSET_HOST_DROPPED));
    host_uart_send_ascii(" ");
    at_send_key_u32("HOST_ERRORS", at_get_u32_le(stats, HOST_STATS_OFFSET_HOST_ERRORS));
    host_uart_send_ascii(" ");
    at_send_key_u32("HOST_QUEUE_FULL", at_get_u32_le(stats, HOST_STATS_OFFSET_HOST_QUEUE_FULL));
    host_uart_send_ascii("\r\n");

    at_send_key_u32("HOST_IRQ_IDLE", at_get_u32_le(stats, HOST_STATS_OFFSET_HOST_IRQ_IDLE));
    host_uart_send_ascii(" ");
    at_send_key_u32("HOST_IRQ_HT", at_get_u32_le(stats, HOST_STATS_OFFSET_HOST_IRQ_HT));
    host_uart_send_ascii(" ");
    at_send_key_u32("HOST_IRQ_TC", at_get_u32_le(stats, HOST_STATS_OFFSET_HOST_IRQ_TC));
    host_uart_send_ascii(" ");
    at_send_key_u32("HOST_IRQ_TE", at_get_u32_le(stats, HOST_STATS_OFFSET_HOST_IRQ_TE));
    host_uart_send_ascii("\r\n");

    at_send_key_u32("RADIO_DIO0", at_get_u32_le(stats, HOST_STATS_OFFSET_RADIO_DIO0));
    host_uart_send_ascii(" ");
    at_send_key_u32("RADIO_DIO1", at_get_u32_le(stats, HOST_STATS_OFFSET_RADIO_DIO1));
    host_uart_send_ascii(" ");
    at_send_key_u32("RADIO_DIO2", at_get_u32_le(stats, HOST_STATS_OFFSET_RADIO_DIO2));
    host_uart_send_ascii(" ");
    at_send_key_u32("RADIO_DIO3", at_get_u32_le(stats, HOST_STATS_OFFSET_RADIO_DIO3));
    host_uart_send_ascii("\r\n");

    at_send_key_u32("RADIO_RX_OK", at_get_u32_le(stats, HOST_STATS_OFFSET_RADIO_RX_OK));
    host_uart_send_ascii(" ");
    at_send_key_u32("RADIO_TX_OK", at_get_u32_le(stats, HOST_STATS_OFFSET_RADIO_TX_OK));
    host_uart_send_ascii(" ");
    at_send_key_u32("RADIO_CRC_ERR", at_get_u32_le(stats, HOST_STATS_OFFSET_RADIO_CRC_ERR));
    host_uart_send_ascii(" ");
    at_send_key_u32("STATE", at_get_u32_le(stats, HOST_STATS_OFFSET_RADIO_STATE));
    host_uart_send_ascii("\r\n");
    at_send_ok();
}

static void at_handle_help(void) {
    host_uart_send_ascii("AT ATI AT+VER? AT+RADIO? AT+STAT? AT+HELP AT+BIN\r\n");
    at_send_ok();
}
#endif

static void put_u16_le(uint8_t *dst, uint16_t value) {
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void put_u32_le(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void host_send_fault_urc(const platform_fault_record_t *record) {
    uint8_t payload[HOST_FAULT_URC_PAYLOAD_LEN];

    if (record == NULL) {
        return;
    }

    memset(payload, 0, sizeof(payload));
    payload[0] = record->code;
    payload[1] = record->sub;
    put_u32_le(&payload[4], record->pc);
    put_u32_le(&payload[8], record->lr);
    put_u32_le(&payload[12], record->psr);
    put_u32_le(&payload[16], record->bfar);
    put_u32_le(&payload[20], platform_now_ms());

    host_uart_send_urc(HOST_TYPE_FAULT_URC,
                       0U,
                       0U,
                       payload,
                       (uint16_t)sizeof(payload));
}

static void host_send_boot_urc(void) {
    uint8_t payload[6];

    payload[0] = (uint8_t)platform_reset_cause_take();
    payload[1] = s_radio_ok ? 1U : 0U;
    payload[2] = s_radio_version;
    payload[3] = HOST_PROTOCOL_VER;
    payload[4] = HOST_WIRE_SCHEMA_VER;
    payload[5] = platform_clock_source_id();

    host_uart_send_urc(HOST_TYPE_BOOT_URC,
                       0U,
                       0U,
                       payload,
                       (uint16_t)sizeof(payload));
}

static void handle_ping(const host_frame_t *frame) {
    host_uart_send_urc(HOST_TYPE_PING_REQ,
                       frame->seq,
                       frame->flags,
                       frame->payload,
                       frame->payload_len);
}

static void handle_version(const host_frame_t *frame) {
    uint8_t payload[96];
    const char *name = OSE_LIFETRACLORA_FW_NAME;
    const char *git = FW_GIT_SHA;
    uint8_t name_len = (uint8_t)strlen(name);
    uint8_t git_len = (uint8_t)strlen(git);
    uint16_t idx = 0U;

    if (name_len > 40U) {
        name_len = 40U;
    }
    if (git_len > 16U) {
        git_len = 16U;
    }

    payload[idx++] = HOST_PROTOCOL_VER;
    payload[idx++] = HOST_WIRE_SCHEMA_VER;
    payload[idx++] = FW_VERSION_MAJOR;
    payload[idx++] = FW_VERSION_MINOR;
    payload[idx++] = FW_VERSION_PATCH;
    payload[idx++] = name_len;
    payload[idx++] = git_len;
    payload[idx++] = 0U;

    put_u32_le(&payload[idx], HOST_CAPABILITY_BITMAP);
    idx = (uint16_t)(idx + 4U);

    memcpy(&payload[idx], name, name_len);
    idx = (uint16_t)(idx + name_len);

    memcpy(&payload[idx], git, git_len);
    idx = (uint16_t)(idx + git_len);

    host_uart_send_urc(HOST_TYPE_VER_URC,
                       frame->seq,
                       0U,
                       payload,
                       idx);
}

static void handle_uid(const host_frame_t *frame) {
    uint8_t payload[12];
    const uint32_t uid0 = MMIO32(STM32_UID_BASE + 0x00UL);
    const uint32_t uid1 = MMIO32(STM32_UID_BASE + 0x04UL);
    const uint32_t uid2 = MMIO32(STM32_UID_BASE + 0x08UL);

    put_u32_le(&payload[0], uid0);
    put_u32_le(&payload[4], uid1);
    put_u32_le(&payload[8], uid2);

    host_uart_send_urc(HOST_TYPE_UID_URC,
                       frame->seq,
                       0U,
                       payload,
                       (uint16_t)sizeof(payload));
}

static void handle_stats_dump(const host_frame_t *frame) {
    uint8_t payload[HOST_STATS_PAYLOAD_LEN];
    const uint16_t payload_len = host_stats_serialize(payload, (uint16_t)sizeof(payload));

    if (payload_len == 0U) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_BAD_LENGTH,
                                 HOST_STATS_PAYLOAD_LEN);
        return;
    }

    host_uart_send_urc(HOST_TYPE_STATS_URC,
                       frame->seq,
                       0U,
                       payload,
                       payload_len);
}

static void handle_tx_frame(const host_frame_t *frame) {
    sx1276_tx_request_t req;

    if (frame->payload_len < 2U) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_BAD_LENGTH,
                                 2U);
        return;
    }

    req.tx_id = frame->payload[0];
    req.length = frame->payload[1];

    if (frame->payload_len != (uint16_t)(2U + req.length)) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_BAD_LENGTH,
                                 (uint16_t)(2U + req.length));
        return;
    }

    if (sx1276_tx_busy()) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_QUEUE_FULL,
                                 0U);
        return;
    }

    if (req.length > 0U) {
        memcpy(req.payload, &frame->payload[2], req.length);
    }

    if (!sx1276_tx_begin(&req)) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_FORBIDDEN,
                                 0U);
    }
}

static bool reg_write_allowed(uint8_t reg_addr) {
#if HOST_ALLOW_REG_WRITE_DIAG
    switch (reg_addr) {
        case 0x06U:
        case 0x07U:
        case 0x08U:
        case 0x09U:
        case 0x1DU:
        case 0x1EU:
        case 0x26U:
        case 0x31U:
        case 0x37U:
            return true;
        default:
            return false;
    }
#else
    (void)reg_addr;
    return false;
#endif
}

static void handle_reg_read(const host_frame_t *frame) {
    uint8_t payload[2];

    if (frame->payload_len != 1U) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_BAD_LENGTH,
                                 1U);
        return;
    }

    payload[0] = frame->payload[0];
    payload[1] = sx1276_read_reg(frame->payload[0]);

    host_uart_send_urc(HOST_TYPE_REG_DATA_URC,
                       frame->seq,
                       0U,
                       payload,
                       (uint16_t)sizeof(payload));
}

static void handle_reg_write(const host_frame_t *frame) {
    if (frame->payload_len != 2U) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_BAD_LENGTH,
                                 2U);
        return;
    }

    if (!reg_write_allowed(frame->payload[0])) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_FORBIDDEN,
                                 frame->payload[0]);
        return;
    }

    sx1276_write_reg(frame->payload[0], frame->payload[1]);

    host_uart_send_urc(HOST_TYPE_REG_WRITE_ACK_URC,
                       frame->seq,
                       0U,
                       frame->payload,
                       2U);
}

static void handle_cfg_set(const host_frame_t *frame) {
    cfg_status_t status;
    uint8_t key;
    uint8_t in_len;
    uint8_t actual_len = 0U;
    uint8_t urc_len;
    uint8_t urc_payload[HOST_CFG_OK_PAYLOAD_LEN];
    uint8_t scratch[8];

    if (frame->payload_len < 2U) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_BAD_LENGTH,
                                 2U);
        return;
    }

    key = frame->payload[0];
    in_len = frame->payload[1];

    if (frame->payload_len != (uint16_t)(2U + in_len)) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_BAD_LENGTH,
                                 (uint16_t)(2U + in_len));
        return;
    }

    status = cfg_set(key, &frame->payload[2], in_len);

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
                                 urc_payload,
                                 (uint8_t)sizeof(urc_payload),
                                 &urc_len)) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_BAD_LENGTH,
                                 HOST_CFG_OK_PAYLOAD_LEN);
        return;
    }

    host_uart_send_urc(HOST_TYPE_CFG_OK_URC,
                       frame->seq,
                       0U,
                       urc_payload,
                       urc_len);
}

static void handle_cfg_get(const host_frame_t *frame) {
    cfg_status_t status;
    uint8_t key;
    uint8_t val_len = 0U;
    uint8_t urc_len;
    uint8_t urc_payload[HOST_CFG_DATA_PAYLOAD_MAX_LEN];

    if (frame->payload_len != 1U) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_BAD_LENGTH,
                                 1U);
        return;
    }

    key = frame->payload[0];
    status = cfg_get(key,
                     &urc_payload[HOST_CFG_DATA_HEADER_LEN],
                     (uint8_t)(sizeof(urc_payload) - HOST_CFG_DATA_HEADER_LEN),
                     &val_len);
    if (status != CFG_STATUS_OK) {
        if (!host_cfg_wire_encode_ok(key,
                                     status,
                                     0U,
                                     urc_payload,
                                     (uint8_t)sizeof(urc_payload),
                                     &urc_len)) {
            host_uart_send_err_proto(frame->seq,
                                     frame->type,
                                     frame->ver,
                                     HOST_ERR_PROTO_BAD_LENGTH,
                                     HOST_CFG_OK_PAYLOAD_LEN);
            return;
        }

        host_uart_send_urc(HOST_TYPE_CFG_OK_URC,
                           frame->seq,
                           0U,
                           urc_payload,
                           urc_len);
        return;
    }

    if (!host_cfg_wire_encode_data(key,
                                   &urc_payload[HOST_CFG_DATA_HEADER_LEN],
                                   val_len,
                                   urc_payload,
                                   (uint8_t)sizeof(urc_payload),
                                   &urc_len)) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_BAD_LENGTH,
                                 (uint16_t)(HOST_CFG_DATA_HEADER_LEN + val_len));
        return;
    }

    host_uart_send_urc(HOST_TYPE_CFG_DATA_URC,
                       frame->seq,
                       0U,
                       urc_payload,
                       urc_len);
}

void host_cmd_init(bool radio_ok, uint8_t radio_version) {
    platform_fault_record_t replay;

    host_type_compile_time_uniqueness_check();

    s_radio_ok = radio_ok;
    s_radio_version = radio_version;

    cfg_init();
    host_stats_reset();
    host_send_boot_urc();

    if (platform_fault_take_replay(&replay)) {
        host_send_fault_urc(&replay);
    }
}

void host_cmd_dispatch(const host_frame_t *frame) {
    if (frame == NULL) {
        return;
    }

    if (frame->ver != HOST_PROTOCOL_VER) {
        host_uart_send_err_proto(frame->seq,
                                 frame->type,
                                 frame->ver,
                                 HOST_ERR_PROTO_BAD_VERSION,
                                 HOST_PROTOCOL_VER);
        return;
    }

    switch (frame->type) {
        case HOST_TYPE_PING_REQ:
            handle_ping(frame);
            break;

        case HOST_TYPE_VER_REQ:
            handle_version(frame);
            break;

        case HOST_TYPE_UID_REQ:
            handle_uid(frame);
            break;

        case HOST_TYPE_RESET_REQ:
            platform_system_reset();
            break;

        case HOST_TYPE_TX_FRAME_REQ:
            handle_tx_frame(frame);
            break;

        case HOST_TYPE_STATS_RESET_REQ:
            host_stats_reset();
            break;

        case HOST_TYPE_STATS_DUMP_REQ:
            handle_stats_dump(frame);
            break;

        case HOST_TYPE_REG_READ_REQ:
            handle_reg_read(frame);
            break;

        case HOST_TYPE_REG_WRITE_REQ:
            handle_reg_write(frame);
            break;

        case HOST_TYPE_CFG_SET_REQ:
            handle_cfg_set(frame);
            break;

        case HOST_TYPE_CFG_GET_REQ:
            handle_cfg_get(frame);
            break;

        default:
            host_uart_send_err_proto(frame->seq,
                                     frame->type,
                                     frame->ver,
                                     HOST_ERR_PROTO_UNKNOWN_TYPE,
                                     frame->type);
            break;
    }
}

void host_cmd_dispatch_at_line(const char *line, uint16_t len) {
#if HOST_AT_SHELL_ENABLE
    if (line == NULL) {
        return;
    }

    if (at_line_equals(line, len, "AT")) {
        at_send_ok();
    } else if (at_line_equals(line, len, "ATI")) {
        at_handle_identity();
    } else if (at_line_equals(line, len, "AT+VER?")) {
        at_handle_version();
    } else if (at_line_equals(line, len, "AT+RADIO?")) {
        at_handle_radio();
    } else if (at_line_equals(line, len, "AT+STAT?")) {
        at_handle_stats();
    } else if (at_line_equals(line, len, "AT+HELP") ||
               at_line_equals(line, len, "AT+HELP?")) {
        at_handle_help();
    } else if (at_line_equals(line, len, "AT+BIN")) {
        host_uart_send_ascii("OK BIN\r\n");
    } else {
        at_send_error();
    }
#else
    (void)line;
    (void)len;
#endif
}

void host_cmd_on_radio_events(uint32_t radio_events) {
    host_stats_note_radio_events(radio_events);

#if HOST_EMIT_RADIO_IRQ_DEBUG_URC
    if (radio_events != 0U) {
        uint8_t payload[4];
        put_u32_le(payload, radio_events);
        host_uart_send_urc(HOST_TYPE_RADIO_IRQ_URC,
                           0U,
                           0U,
                           payload,
                           (uint16_t)sizeof(payload));
    }
#else
    (void)radio_events;
#endif
}

void host_cmd_emit_tx_done(const sx1276_tx_result_t *result) {
    uint8_t payload[7];

    if (result == NULL) {
        return;
    }

    payload[0] = result->tx_id;
    payload[1] = result->status;
    put_u32_le(&payload[2], result->time_on_air_us);
    payload[6] = result->tx_power_dbm;

    host_uart_send_urc(HOST_TYPE_TX_DONE_URC,
                       0U,
                       0U,
                       payload,
                       (uint16_t)sizeof(payload));
}

void host_cmd_emit_fault(uint8_t code, uint8_t sub) {
    platform_fault_record_t fault;

    fault.code = code;
    fault.sub = sub;
    fault.pc = 0U;
    fault.lr = 0U;
    fault.psr = 0U;
    fault.bfar = 0U;

    host_send_fault_urc(&fault);
}

void host_cmd_emit_rx_frame(const sx1276_rx_frame_t *frame) {
    uint8_t payload[264];

    if (frame == NULL) {
        return;
    }

    payload[0] = frame->length;
    payload[1] = (uint8_t)frame->snr_db;
    put_u16_le(&payload[2], (uint16_t)frame->rssi_dbm);
    put_u32_le(&payload[4], frame->timestamp_us);
    if (frame->length > 0U) {
        memcpy(&payload[8], frame->payload, frame->length);
    }

    host_uart_send_urc(HOST_TYPE_RX_FRAME_URC,
                       0U,
                       0U,
                       payload,
                       (uint16_t)(8U + frame->length));
}
