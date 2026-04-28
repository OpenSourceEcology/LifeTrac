// shared_mem.h — Portenta H747 M7↔M4 shared SRAM4 layout.
//
// Both cores agree on this layout so the M4 watchdog can read the M7's
// liveness tick without a second IPC channel. SRAM4 (0x38000000) is shared
// across both cores on the H747; the M7 stamps every loop iteration, the
// M4 polls every 20 ms.
//
// Per TRACTOR_NODE.md § Wiring overview and MASTER_PLAN.md §8.4 (M4 acts
// as the independent safety watchdog feeding the PSR safety relay).
//
// IMPORTANT: do NOT add fields without bumping LIFETRAC_SHARED_VERSION.
// A stale M4 binary against a new M7 layout would silently misread a
// safety-critical field.

#ifndef LIFETRAC_SHARED_MEM_H
#define LIFETRAC_SHARED_MEM_H

#include <stdint.h>

#define LIFETRAC_SHARED_VERSION   2u
#define LIFETRAC_SHARED_ADDR      0x38000000u

// Seqlock convention (IP-105):
//   * Writer (M7) increments `seq` to an ODD value, writes the payload
//     fields, then increments `seq` to the next EVEN value.
//   * Reader (M4) snapshots `seq` (must be EVEN), reads the payload, then
//     re-snapshots `seq` and confirms it is unchanged. If it changed (or
//     was odd on the first read) the reader retries.
//   * No locks needed across cores; works because the cortex-M cores
//     have coherent SRAM4 visibility for word-aligned 32-bit writes.
//   * `seq` itself is the only field that needs atomic write on each
//     update; the rest of the struct is a normal copy.

struct SharedM7M4 {
    // Bumped on every layout change; M4 refuses to arm the safety output
    // until it sees the version it was compiled for.
    volatile uint32_t version;
    // Seqlock counter \u2014 odd while writer is mid-update, even otherwise.
    volatile uint32_t seq;
    // M7 writes millis() each loop iteration. M4 trips if (now - tick)
    // exceeds LIFETRAC_M4_WATCHDOG_MS.
    volatile uint32_t alive_tick_ms;
    // M7 sets this to LIFETRAC_ESTOP_MAGIC when its E-stop is latched; M4
    // mirrors by dropping the safety output even if alive_tick is current.
    // Magic value (not just non-zero) so SRAM corruption / stale boot
    // garbage cannot trip the safety chain (IP-106).
    volatile uint32_t estop_request;
    // Bumped on every M7 loop so M4 can compute "loop frequency too low"
    // even when the timestamp is current (e.g. RTC stuck).
    volatile uint32_t loop_counter;
    volatile uint32_t reserved[11];
};

// IP-106: any value other than this constant is treated as "no E-stop".
// Chosen as 0xA5A5A5A5 (alternating bit pattern) so neither all-zero RAM
// nor all-ones flash erase pattern can match.
#define LIFETRAC_ESTOP_MAGIC      0xA5A5A5A5u

#define LIFETRAC_M4_WATCHDOG_MS   200u

#endif  // LIFETRAC_SHARED_MEM_H
