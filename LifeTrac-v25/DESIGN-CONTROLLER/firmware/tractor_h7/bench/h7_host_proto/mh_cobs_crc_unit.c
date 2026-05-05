#include "mh_cobs.h"
#include "mh_crc16.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t g_failures;

static void fail_case(const char *name, const char *reason) {
    printf("[FAIL] %s: %s\n", name, reason);
    g_failures++;
}

static void test_crc_reference(void) {
    static const uint8_t k_vec[] = { '1','2','3','4','5','6','7','8','9' };
    const uint16_t crc = mh_crc16_ccitt(k_vec, sizeof(k_vec));

    if (crc != 0x29B1U) {
        fail_case("crc_reference", "CRC16-CCITT mismatch for 123456789");
    }
}

static void assert_cobs_roundtrip(const char *name, const uint8_t *in, size_t in_len) {
    uint8_t enc[512];
    uint8_t dec[512];
    size_t enc_len;
    size_t dec_len;

    memset(enc, 0, sizeof(enc));
    memset(dec, 0, sizeof(dec));

    enc_len = mh_cobs_encode(in, in_len, enc, sizeof(enc));
    if (enc_len == 0U) {
        fail_case(name, "encode failed");
        return;
    }

    dec_len = mh_cobs_decode(enc, enc_len, dec, sizeof(dec));
    if (dec_len != in_len) {
        fail_case(name, "decoded length mismatch");
        return;
    }

    if (memcmp(in, dec, in_len) != 0) {
        fail_case(name, "decoded bytes mismatch");
    }
}

static void test_cobs_roundtrip(void) {
    static const uint8_t k_case_a[] = { 0x01U, 0x02U, 0x03U };
    static const uint8_t k_case_b[] = { 0x00U, 0x01U, 0x00U, 0x02U, 0x00U };
    static const uint8_t k_case_c[] = {
        0x11U, 0x00U, 0x22U, 0x33U, 0x00U, 0x44U, 0x55U, 0x66U,
        0x77U, 0x00U, 0x88U, 0x99U
    };

    assert_cobs_roundtrip("cobs_roundtrip_simple", k_case_a, sizeof(k_case_a));
    assert_cobs_roundtrip("cobs_roundtrip_with_zeros", k_case_b, sizeof(k_case_b));
    assert_cobs_roundtrip("cobs_roundtrip_mixed", k_case_c, sizeof(k_case_c));
}

int main(void) {
    test_crc_reference();
    test_cobs_roundtrip();

    if (g_failures != 0U) {
        printf("[FAIL] mh_cobs_crc_unit: %lu failures\n", (unsigned long)g_failures);
        return 1;
    }

    printf("[PASS] mh_cobs_crc_unit: 4 checks\n");
    return 0;
}
