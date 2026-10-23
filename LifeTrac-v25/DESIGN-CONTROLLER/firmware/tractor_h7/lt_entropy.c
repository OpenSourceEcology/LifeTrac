#include "lt_entropy.h"

#include <string.h>

static uint32_t read_u32(const uint8_t *src) {
    uint32_t v;
    memcpy(&v, src, sizeof(v));
    return v;
}

void lt_entropy_init(lt_entropy_t *e, lt_entropy_source_t source, void *ctx) {
    if (e == NULL) {
        return;
    }
    memset(e, 0, sizeof(*e));
    e->source = source;
    e->ctx = ctx;
    e->pos = LT_ENTROPY_POOL_LEN;   /* empty: first get refills */
}

static bool refill(lt_entropy_t *e) {
    uint8_t first[4];
    size_t i;

    if (e->source == NULL) {
        e->source_errors++;
        return false;
    }
    /* The first word ever read only seeds the continuous test. */
    if (!e->have_last) {
        if (!e->source(first, sizeof(first), e->ctx)) {
            e->source_errors++;
            return false;
        }
        e->last_word = read_u32(first);
        e->have_last = true;
    }
    if (!e->source(e->pool, sizeof(e->pool), e->ctx)) {
        e->source_errors++;
        return false;
    }
    for (i = 0; i < sizeof(e->pool); i += 4U) {
        uint32_t w = read_u32(&e->pool[i]);
        if (w == e->last_word) {
            e->repeat_errors++;
            e->failed = true;
            memset(e->pool, 0, sizeof(e->pool));
            return false;
        }
        e->last_word = w;
    }
    e->pos = 0U;
    e->refills++;
    return true;
}

bool lt_entropy_get(lt_entropy_t *e, uint8_t *out, size_t len) {
    size_t done = 0U;

    if (e == NULL || (out == NULL && len > 0U) || e->failed) {
        return false;
    }
    while (done < len) {
        size_t avail;
        size_t take;

        if (e->pos >= LT_ENTROPY_POOL_LEN && !refill(e)) {
            return false;
        }
        avail = (size_t)(LT_ENTROPY_POOL_LEN - e->pos);
        take = (len - done) < avail ? (len - done) : avail;
        memcpy(&out[done], &e->pool[e->pos], take);
        /* Hand each byte out once. */
        memset(&e->pool[e->pos], 0, take);
        e->pos = (uint8_t)(e->pos + take);
        done += take;
    }
    return true;
}
