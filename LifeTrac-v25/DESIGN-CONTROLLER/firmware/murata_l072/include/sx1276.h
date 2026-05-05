#ifndef LIFETRAC_MURATA_L072_SX1276_H
#define LIFETRAC_MURATA_L072_SX1276_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SX1276_EVT_DIO0              (1UL << 0)
#define SX1276_EVT_DIO1              (1UL << 1)
#define SX1276_EVT_DIO2              (1UL << 2)
#define SX1276_EVT_DIO3              (1UL << 3)

typedef struct sx1276_profile_s {
    uint32_t freq_hz;
    uint8_t sf;
    uint16_t bw_khz;
    uint8_t cr_den;
    uint8_t tx_power_dbm;
} sx1276_profile_t;

bool sx1276_init(void);
void sx1276_radio_reset(void);

uint8_t sx1276_read_reg(uint8_t reg_addr);
void sx1276_write_reg(uint8_t reg_addr, uint8_t value);
void sx1276_read_burst(uint8_t start_reg, uint8_t *dst, size_t len);
void sx1276_write_burst(uint8_t start_reg, const uint8_t *src, size_t len);

uint8_t sx1276_read_version(void);
void sx1276_set_frequency_hz(uint32_t freq_hz);
void sx1276_set_tx_power_dbm(uint8_t dbm);
void sx1276_set_sf_bw_cr(uint8_t sf, uint16_t bw_khz, uint8_t cr_den);
void sx1276_apply_profile_full(const sx1276_profile_t *profile);

uint32_t sx1276_take_irq_events(void);
bool sx1276_reg_dump(uint8_t *out_regs, size_t out_len);

#endif /* LIFETRAC_MURATA_L072_SX1276_H */
