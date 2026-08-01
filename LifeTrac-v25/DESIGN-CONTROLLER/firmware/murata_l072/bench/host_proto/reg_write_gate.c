/*
 * F9 (2026-07-30): pins the REG_WRITE acceptance policy in
 * host_reg_gate.c, in BOTH flag states — diag_enabled is a function
 * parameter precisely so this test can drive each without a -D.
 *
 * The diag=false rows are the F9 contract: the three production opmode
 * values pass, everything else is refused — including the values real
 * probes send (T2's FSK 0x00/0x01, SPI-isolation's 0x83), which is
 * what keeps the flag meaningful. The diag=true rows pin the 13-address
 * legacy list verbatim so drift is loud.
 */
#include <stdio.h>

#include "host_reg_gate.h"

static int g_failures = 0;

#define CHECK(cond, ...) do {                                          \
    if (!(cond)) {                                                     \
        ++g_failures;                                                  \
        fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);           \
        fprintf(stderr, __VA_ARGS__);                                  \
        fputc('\n', stderr);                                           \
    }                                                                  \
} while (0)

static void test_production_policy_diag_off(void) {
    /* The three values every daemon actually sends: always allowed. */
    CHECK(host_reg_write_allowed(0x01U, 0x85U, false), "RXCONT allowed");
    CHECK(host_reg_write_allowed(0x01U, 0x80U, false), "SLEEP allowed");
    CHECK(host_reg_write_allowed(0x01U, 0x81U, false), "STANDBY allowed");

    /* Dangerous / diagnostic opmode values: refused without the flag. */
    CHECK(!host_reg_write_allowed(0x01U, 0x83U, false),
          "TX refused — raw key-up would bypass LBT/airtime/dwell");
    CHECK(!host_reg_write_allowed(0x01U, 0x87U, false), "CAD refused");
    CHECK(!host_reg_write_allowed(0x01U, 0x86U, false), "RXSINGLE refused");
    CHECK(!host_reg_write_allowed(0x01U, 0x00U, false),
          "FSK SLEEP refused — the T2 probe really sends this");
    CHECK(!host_reg_write_allowed(0x01U, 0x01U, false),
          "FSK STDBY refused — the T2 probe really sends this");
    CHECK(!host_reg_write_allowed(0x01U, 0x05U, false),
          "FSK RXCONT (bit7 clear) refused");

    /* Non-opmode registers: all refused without the flag. */
    CHECK(!host_reg_write_allowed(0x1DU, 0x72U, false), "ModemConfig1 refused");
    CHECK(!host_reg_write_allowed(0x1EU, 0x74U, false), "ModemConfig2 refused");
    CHECK(!host_reg_write_allowed(0x40U, 0x00U, false), "DioMapping1 refused");
    CHECK(!host_reg_write_allowed(0x00U, 0x00U, false), "FIFO refused");
    CHECK(!host_reg_write_allowed(0xFFU, 0xFFU, false), "0xFF refused");
}

static void test_diag_list_pinned(void) {
    static const uint8_t legacy[13] = {
        0x01U, 0x06U, 0x07U, 0x08U, 0x09U, 0x1DU, 0x1EU,
        0x26U, 0x31U, 0x33U, 0x37U, 0x3BU, 0x40U
    };
    for (unsigned i = 0; i < 13U; ++i) {
        /* Arbitrary value — the diag surface is address-gated only,
         * which is what keeps T2 (0x00/0x01) and SPI-isolation (0x83)
         * working in diag builds. */
        CHECK(host_reg_write_allowed(legacy[i], 0x5AU, true),
              "diag addr 0x%02X allowed", legacy[i]);
    }
    CHECK(!host_reg_write_allowed(0x02U, 0x5AU, true),
          "non-listed addr refused even in diag builds");
}

static void test_exhaustive_property_diag_off(void) {
    /* diag=false: allowed iff reg==0x01 && val in {0x80,0x81,0x85}. */
    unsigned allowed = 0;
    for (unsigned reg = 0; reg < 256U; ++reg) {
        for (unsigned val = 0; val < 256U; ++val) {
            const bool got = host_reg_write_allowed(
                (uint8_t)reg, (uint8_t)val, false);
            const bool want = (reg == 0x01U) &&
                (val == 0x80U || val == 0x81U || val == 0x85U);
            if (got != want) {
                CHECK(false, "policy mismatch reg=0x%02X val=0x%02X "
                             "got=%d want=%d",
                      reg, val, (int)got, (int)want);
                return;
            }
            if (got) { ++allowed; }
        }
    }
    CHECK(allowed == 3U, "exactly 3 (reg,val) pairs allowed, got %u", allowed);
}

int main(void) {
    test_production_policy_diag_off();
    test_diag_list_pinned();
    test_exhaustive_property_diag_off();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] reg_write_gate: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("[PASS] reg_write_gate: 3 test fns, 65536-point property\n");
    return 0;
}
