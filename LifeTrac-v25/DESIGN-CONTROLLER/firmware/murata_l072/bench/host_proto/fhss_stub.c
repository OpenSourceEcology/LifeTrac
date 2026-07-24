/*
 * Host-side link stub for sx1276_fhss_init(), needed by the four
 * cfg-profile/cfg-contract bench targets that link host_cfg_profile.c
 * with sx1276_stub.c instead of the real radio objects.
 *
 * Kept OUT of sx1276_stub.c on purpose: check-tx-len-guard (and any
 * future target) links sx1276_stub.c TOGETHER WITH the real
 * radio/sx1276_fhss.c, and a stub definition there would collide with
 * the real symbol at link time. Only link this file where the real
 * FHSS scheduler is absent.
 */
#include "sx1276_fhss.h"

sx1276_fhss_status_t sx1276_fhss_init(uint64_t farm_id,
                                      uint64_t node_id,
                                      uint32_t initial_epoch) {
    (void)farm_id;
    (void)node_id;
    (void)initial_epoch;
    return SX1276_FHSS_OK;
}
