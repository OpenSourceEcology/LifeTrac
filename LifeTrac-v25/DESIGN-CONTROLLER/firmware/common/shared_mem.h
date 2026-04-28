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

#define LIFETRAC_SHARED_VERSION   1u
#define LIFETRAC_SHARED_ADDR      0x38000000u

struct SharedM7M4 {
    // Bumped on every layout change; M4 refuses to arm the safety output
    // until it sees the version it was compiled for.
    volatile uint32_t version;
    // M7 writes millis() each loop iteration. M4 trips if (now - tick)
    // exceeds LIFETRAC_M4_WATCHDOG_MS.
    volatile uint32_t alive_tick_ms;
    // M7 sets this true when its E-stop is latched; M4 mirrors by
    // dropping the safety output even if alive_tick is current. This is
    // belt-and-suspenders: the Opta gets the same signal via Modbus.
    volatile uint32_t estop_request;
    // Bumped on every M7 loop so M4 can compute "loop frequency too low"
    // even when the timestamp is current (e.g. RTC stuck).
    volatile uint32_t loop_counter;
    volatile uint32_t reserved[12];
};

#define LIFETRAC_M4_WATCHDOG_MS   200u

#endif  // LIFETRAC_SHARED_MEM_H
