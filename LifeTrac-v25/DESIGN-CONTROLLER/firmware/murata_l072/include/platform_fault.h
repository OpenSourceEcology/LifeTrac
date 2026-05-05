#ifndef LIFETRAC_MURATA_L072_PLATFORM_FAULT_H
#define LIFETRAC_MURATA_L072_PLATFORM_FAULT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct platform_fault_record_s {
    uint8_t code;
    uint8_t sub;
    uint32_t pc;
    uint32_t lr;
    uint32_t psr;
    uint32_t bfar;
} platform_fault_record_t;

void platform_fault_record(uint8_t code,
                           uint8_t sub,
                           uint32_t pc,
                           uint32_t lr,
                           uint32_t psr,
                           uint32_t bfar);
bool platform_fault_take_replay(platform_fault_record_t *out_record);

#endif /* LIFETRAC_MURATA_L072_PLATFORM_FAULT_H */