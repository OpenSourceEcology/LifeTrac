#ifndef LIFETRAC_TRACTOR_H7_MH_RUNTIME_H
#define LIFETRAC_TRACTOR_H7_MH_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool mh_runtime_begin(void *serial_port, uint32_t baud);
void mh_runtime_loop(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* LIFETRAC_TRACTOR_H7_MH_RUNTIME_H */
