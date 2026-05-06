#ifndef LIFETRAC_TRACTOR_H7_MH_RUNTIME_HEALTH_H
#define LIFETRAC_TRACTOR_H7_MH_RUNTIME_HEALTH_H

#include "mh_stats.h"
#include "mh_wire.h"
#include "murata_host.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mh_runtime_health_s {
    bool boot_seen;
    uint8_t boot_reset_cause;
    uint8_t boot_radio_ok;
    uint8_t boot_radio_version;
    uint8_t boot_protocol_ver;
    uint8_t boot_wire_schema_ver;
    uint8_t boot_clock_source_id;
    uint32_t boot_seen_at_ms;

    bool fault_seen;
    uint8_t fault_code;
    uint8_t fault_sub;
    uint32_t fault_seen_at_ms;

    bool has_stats;
    mh_stats_t last_stats;
    uint16_t last_stats_len;
    uint32_t last_stats_seen_at_ms;

    uint32_t ping_echo_count;
    uint32_t cfg_ok_count;
    uint32_t cfg_data_count;
    uint32_t err_proto_count;

    uint32_t tx_done_count;
    uint8_t last_tx_id;
    uint8_t last_tx_status;
    uint32_t last_tx_time_on_air_us;
    int8_t last_tx_power_dbm;
    uint32_t tx_done_seen_at_ms;

    uint32_t rx_frame_count;
    uint8_t last_rx_len;
    int8_t last_rx_snr_db;
    int16_t last_rx_rssi_dbm;
    uint32_t last_rx_timestamp_us;
    uint8_t last_rx_payload[HOST_MAX_PAYLOAD_LEN];
    uint32_t rx_frame_seen_at_ms;

    bool ver_seen;
    uint32_t ver_count;
    uint8_t ver_protocol_ver;
    uint8_t ver_wire_schema_ver;
    uint8_t ver_fw_major;
    uint8_t ver_fw_minor;
    uint8_t ver_fw_patch;
    uint32_t ver_capability_bitmap;
    uint8_t ver_fw_name_len;
    uint8_t ver_fw_git_len;
    char ver_fw_name[41];
    char ver_fw_git[17];
    uint32_t ver_seen_at_ms;
} mh_runtime_health_t;

void mh_runtime_health_reset(mh_runtime_health_t *health);

bool mh_runtime_health_on_frame(mh_runtime_health_t *health,
                                const murata_host_frame_t *frame,
                                uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* LIFETRAC_TRACTOR_H7_MH_RUNTIME_HEALTH_H */