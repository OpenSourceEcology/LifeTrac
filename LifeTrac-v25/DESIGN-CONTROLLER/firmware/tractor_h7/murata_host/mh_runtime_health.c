#include "mh_runtime_health.h"

#if defined(LIFETRAC_USE_METHOD_G_HOST) && (LIFETRAC_USE_METHOD_G_HOST)

#include <stddef.h>
#include <string.h>

static uint16_t read_u16_le(const uint8_t *src) {
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *src) {
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

void mh_runtime_health_reset(mh_runtime_health_t *health) {
    if (health == NULL) {
        return;
    }
    memset(health, 0, sizeof(*health));
}

bool mh_runtime_health_on_frame(mh_runtime_health_t *health,
                                const murata_host_frame_t *frame,
                                uint32_t now_ms) {
    if (health == NULL || frame == NULL) {
        return false;
    }

    switch (frame->type) {
        case HOST_TYPE_BOOT_URC:
            if (frame->payload_len < 6U) {
                return false;
            }
            health->boot_seen = true;
            health->boot_reset_cause = frame->payload[0];
            health->boot_radio_ok = frame->payload[1];
            health->boot_radio_version = frame->payload[2];
            health->boot_protocol_ver = frame->payload[3];
            health->boot_wire_schema_ver = frame->payload[4];
            health->boot_clock_source_id = frame->payload[5];
            health->boot_seen_at_ms = now_ms;
            return true;

        case HOST_TYPE_FAULT_URC:
            if (frame->payload_len < 2U) {
                return false;
            }
            health->fault_seen = true;
            health->fault_code = frame->payload[0];
            health->fault_sub = frame->payload[1];
            health->fault_seen_at_ms = now_ms;
            return true;

        case HOST_TYPE_STATS_URC:
            health->has_stats = mh_stats_parse(frame->payload,
                                               frame->payload_len,
                                               &health->last_stats);
            if (!health->has_stats) {
                return false;
            }
            health->last_stats_len = frame->payload_len;
            health->last_stats_seen_at_ms = now_ms;
            return true;

        case HOST_TYPE_PING_REQ:
            health->ping_echo_count++;
            return true;

        case HOST_TYPE_CFG_OK_URC:
            health->cfg_ok_count++;
            return true;

        case HOST_TYPE_CFG_DATA_URC:
            health->cfg_data_count++;
            return true;

        case HOST_TYPE_ERR_PROTO_URC:
            health->err_proto_count++;
            return true;

        case HOST_TYPE_TX_DONE_URC:
            if (frame->payload_len < 7U) {
                return false;
            }
            health->tx_done_count++;
            health->last_tx_id = frame->payload[0];
            health->last_tx_status = frame->payload[1];
            health->last_tx_time_on_air_us = read_u32_le(&frame->payload[2]);
            health->last_tx_power_dbm = (int8_t)frame->payload[6];
            health->tx_done_seen_at_ms = now_ms;
            return true;

        case HOST_TYPE_RX_FRAME_URC: {
            uint8_t rx_len;

            if (frame->payload_len < 8U) {
                return false;
            }

            rx_len = frame->payload[0];
            if (frame->payload_len != (uint16_t)(8U + rx_len)) {
                return false;
            }

            health->rx_frame_count++;
            health->last_rx_len = rx_len;
            health->last_rx_snr_db = (int8_t)frame->payload[1];
            health->last_rx_rssi_dbm = (int16_t)read_u16_le(&frame->payload[2]);
            health->last_rx_timestamp_us = read_u32_le(&frame->payload[4]);
            if (rx_len > 0U) {
                memcpy(health->last_rx_payload, &frame->payload[8], rx_len);
            }
            health->rx_frame_seen_at_ms = now_ms;
            return true;
        }

        case HOST_TYPE_VER_URC: {
            uint8_t name_len;
            uint8_t git_len;
            uint16_t idx;

            if (frame->payload_len < 12U) {
                return false;
            }

            name_len = frame->payload[5];
            git_len = frame->payload[6];
            if (name_len > 40U || git_len > 16U) {
                return false;
            }
            if (frame->payload_len < (uint16_t)(12U + name_len + git_len)) {
                return false;
            }

            health->ver_seen = true;
            health->ver_count++;
            health->ver_protocol_ver = frame->payload[0];
            health->ver_wire_schema_ver = frame->payload[1];
            health->ver_fw_major = frame->payload[2];
            health->ver_fw_minor = frame->payload[3];
            health->ver_fw_patch = frame->payload[4];
            health->ver_fw_name_len = name_len;
            health->ver_fw_git_len = git_len;
            health->ver_capability_bitmap = read_u32_le(&frame->payload[8]);

            idx = 12U;
            memset(health->ver_fw_name, 0, sizeof(health->ver_fw_name));
            if (name_len > 0U) {
                memcpy(health->ver_fw_name, &frame->payload[idx], name_len);
            }
            idx = (uint16_t)(idx + name_len);
            memset(health->ver_fw_git, 0, sizeof(health->ver_fw_git));
            if (git_len > 0U) {
                memcpy(health->ver_fw_git, &frame->payload[idx], git_len);
            }
            health->ver_seen_at_ms = now_ms;
            return true;
        }

        default:
            return false;
    }
}

#else

typedef int mh_runtime_health_disabled_translation_unit;

#endif