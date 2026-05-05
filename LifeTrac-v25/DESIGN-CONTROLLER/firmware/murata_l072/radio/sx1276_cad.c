#include "sx1276_cad.h"

#include "sx1276.h"
#include "sx1276_modes.h"

#define SX1276_REG_IRQ_FLAGS               0x12U

#define SX1276_IRQ_CAD_DETECTED            0x01U
#define SX1276_IRQ_CAD_DONE                0x04U

static uint8_t s_cad_active;

bool sx1276_cad_begin(void) {
    if (s_cad_active != 0U) {
        return false;
    }

    sx1276_write_reg(SX1276_REG_IRQ_FLAGS, 0xFFU);
    if (!sx1276_modes_to_cad()) {
        return false;
    }

    s_cad_active = 1U;
    return true;
}

sx1276_cad_result_t sx1276_cad_poll(void) {
    uint8_t irq_flags;

    if (s_cad_active == 0U) {
        return SX1276_CAD_RESULT_ERROR;
    }

    irq_flags = sx1276_read_reg(SX1276_REG_IRQ_FLAGS);
    if ((irq_flags & SX1276_IRQ_CAD_DONE) == 0U) {
        return SX1276_CAD_RESULT_PENDING;
    }

    sx1276_write_reg(SX1276_REG_IRQ_FLAGS, irq_flags);
    s_cad_active = 0U;
    if (!sx1276_modes_to_standby()) {
        return SX1276_CAD_RESULT_ERROR;
    }

    if ((irq_flags & SX1276_IRQ_CAD_DETECTED) != 0U) {
        return SX1276_CAD_RESULT_DETECTED;
    }

    return SX1276_CAD_RESULT_CLEAR;
}
