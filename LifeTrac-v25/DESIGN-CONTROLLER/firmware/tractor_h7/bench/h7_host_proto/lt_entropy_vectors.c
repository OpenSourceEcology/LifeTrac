/* H7-13: vectors for the nonce entropy pool (lt_entropy.c). */
#include "lt_entropy.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int g_failures;

static void expect_true(bool cond, const char *msg) {
    if (!cond) {
        fprintf(stderr, "[FAIL] %s\n", msg);
        g_failures++;
    }
}

typedef struct fake_src_s {
    uint32_t state;
    uint32_t calls;
    int fail_call;          /* 1-based call number that errors, 0 = never */
    int stuck_after_bytes;  /* after this many bytes, repeat one word; -1 = never */
    uint32_t bytes;
} fake_src_t;

static bool fake_source(uint8_t *out, size_t len, void *ctx) {
    fake_src_t *s = (fake_src_t *)ctx;
    size_t i;

    s->calls++;
    if (s->fail_call > 0 && (int)s->calls == s->fail_call) {
        return false;
    }
    for (i = 0; i < len; i += 4U) {
        uint32_t w;
        if (s->stuck_after_bytes >= 0 && (int)s->bytes >= s->stuck_after_bytes) {
            w = 0xDEADBEEFU;
        } else {
            s->state = s->state * 1664525U + 1013904223U;
            w = s->state;
        }
        memcpy(&out[i], &w, 4);
        s->bytes += 4U;
    }
    return true;
}

static void test_bytes_flow_and_differ(void) {
    fake_src_t src = { 12345U, 0U, 0, -1, 0U };
    lt_entropy_t e;
    uint8_t a[5], b[5];
    uint8_t big[70];

    lt_entropy_init(&e, fake_source, &src);
    expect_true(lt_entropy_get(&e, a, sizeof(a)), "healthy source yields bytes");
    expect_true(lt_entropy_get(&e, b, sizeof(b)), "second draw yields bytes");
    expect_true(memcmp(a, b, sizeof(a)) != 0, "consecutive draws differ");
    expect_true(src.calls == 2U, "first refill reads a seed word plus one pool");
    expect_true(lt_entropy_get(&e, big, sizeof(big)), "draws larger than the pool span refills");
    expect_true(e.refills == 3U, "70 + 10 bytes take three 32-byte pools");
    expect_true(lt_entropy_get(&e, NULL, 0U), "zero-length draw is fine");
}

static void test_source_error_is_not_latched(void) {
    fake_src_t src = { 7U, 0U, 1, -1, 0U };
    lt_entropy_t e;
    uint8_t out[5];

    lt_entropy_init(&e, fake_source, &src);
    expect_true(!lt_entropy_get(&e, out, sizeof(out)), "source error fails the draw");
    expect_true(e.source_errors == 1U && !e.failed, "source error is counted, not latched");
    expect_true(lt_entropy_get(&e, out, sizeof(out)), "a recovered source works again");
}

static void test_stuck_source_latches(void) {
    fake_src_t src = { 99U, 0U, 0, 40, 0U };   /* seed word + 1 pool + 1 word ok */
    lt_entropy_t e;
    uint8_t out[32];

    lt_entropy_init(&e, fake_source, &src);
    expect_true(lt_entropy_get(&e, out, sizeof(out)), "first pool is healthy");
    expect_true(!lt_entropy_get(&e, out, 1U), "a repeated word fails the refill");
    expect_true(e.failed && e.repeat_errors == 1U, "repeated word latches the pool");
    src.stuck_after_bytes = -1;
    expect_true(!lt_entropy_get(&e, out, 1U), "latched pool stays failed");
}

static void test_no_source(void) {
    lt_entropy_t e;
    uint8_t out[5];

    lt_entropy_init(&e, NULL, NULL);
    expect_true(!lt_entropy_get(&e, out, sizeof(out)), "no source, no bytes");
}

int main(void) {
    test_bytes_flow_and_differ();
    test_source_error_is_not_latched();
    test_stuck_source_latches();
    test_no_source();
    if (g_failures != 0) {
        fprintf(stderr, "lt_entropy_vectors: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("lt_entropy_vectors: all checks passed\n");
    return 0;
}
