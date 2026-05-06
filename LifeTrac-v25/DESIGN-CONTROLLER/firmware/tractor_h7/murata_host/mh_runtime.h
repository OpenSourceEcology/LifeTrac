#ifndef LIFETRAC_TRACTOR_H7_MH_RUNTIME_H
#define LIFETRAC_TRACTOR_H7_MH_RUNTIME_H

#include "mh_runtime_health.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*mh_runtime_log_sink_t)(const char *line, void *ctx);

bool mh_runtime_begin(void *serial_port, uint32_t baud);
void mh_runtime_loop(uint32_t now_ms);
bool mh_runtime_get_health(mh_runtime_health_t *out_health);
void mh_runtime_set_log_sink(mh_runtime_log_sink_t sink, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* LIFETRAC_TRACTOR_H7_MH_RUNTIME_H */
