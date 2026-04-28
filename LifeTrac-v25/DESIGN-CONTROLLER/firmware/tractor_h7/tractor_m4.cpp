// tractor_m4.cpp — Portenta H747 M4 core: independent valve safety watchdog.
//
// The M4 runs alongside the M7 on the same H747 silicon. It does ONE thing:
// observe the "alive tick" the M7 publishes via shared SRAM4, and pull a
// GPIO low (de-energising the PSR safety relay) if any of:
//
//   * the M7 stops ticking for >LIFETRAC_M4_WATCHDOG_MS,
//   * the M7's loop_counter stops advancing for the same window
//     (catches RTC-stuck-but-everything-else-frozen),
//   * the M7 explicitly raises estop_request (mirror of CMD_ESTOP latch).
//
// Why bother, when the Opta also has a Modbus-side watchdog?
//   Defense in depth. M4 watchdog catches "M7 is alive but the Modbus link
//   is wedged"; Opta watchdog catches "M7 *and* M4 are both dead."
//
// Per TRACTOR_NODE.md § Wiring overview and MASTER_PLAN.md §8.4.
//
// SAFETY NOTE: once tripped, this loop latches forever; only an MCU reset
// re-arms. That is intentional — automatic re-arming after a missed tick
// would mask intermittent firmware faults that the operator must see.

#include "Arduino.h"
#include "mbed.h"
#include "../common/shared_mem.h"

static volatile SharedM7M4* const SHARED =
    reinterpret_cast<volatile SharedM7M4*>(LIFETRAC_SHARED_ADDR);

// "Alive" GPIO into the PSR safety relay's monitored channel.
// Active-high = healthy; pulled low on any trip.
#define PIN_SAFETY_ALIVE  PG_3

// Heartbeat LED so a tech with no scope can see the M4 is running.
#define PIN_M4_HEARTBEAT  LEDB

static uint32_t s_last_loop_counter = 0;
static uint32_t s_last_loop_change_ms = 0;
static uint32_t s_hb_toggle_ms = 0;
static bool     s_hb_state = false;

static inline void trip(const char* /*reason*/) {
    digitalWrite(PIN_SAFETY_ALIVE, LOW);
    // Latch low until reset. Heartbeat LED held solid-on so a tech can
    // distinguish "M4 dead" (LED off) from "M4 tripped" (LED on solid).
    digitalWrite(PIN_M4_HEARTBEAT, HIGH);
    while (1) { delay(1000); }
}

void setup() {
    pinMode(PIN_SAFETY_ALIVE, OUTPUT);
    pinMode(PIN_M4_HEARTBEAT, OUTPUT);
    digitalWrite(PIN_SAFETY_ALIVE, LOW);    // start de-energised
    digitalWrite(PIN_M4_HEARTBEAT, LOW);

    // Wait for the M7 to publish its shared-mem header. We refuse to
    // arm until the version matches what we were compiled for — a stale
    // M4 against a new M7 layout would silently misread a safety field.
    uint32_t deadline = millis() + 3000;
    while (millis() < deadline) {
        if (SHARED->version == LIFETRAC_SHARED_VERSION &&
            SHARED->alive_tick_ms != 0) {
            break;
        }
        delay(10);
    }
    if (SHARED->version != LIFETRAC_SHARED_VERSION) {
        // Cannot trust shared memory — leave the safety output low and
        // halt. Operator will see the heartbeat LED dark.
        while (1) { delay(1000); }
    }
    s_last_loop_counter   = SHARED->loop_counter;
    s_last_loop_change_ms = millis();
    digitalWrite(PIN_SAFETY_ALIVE, HIGH);   // arm
}

void loop() {
    uint32_t now = millis();

    // 1. Liveness tick check.
    uint32_t last_tick = SHARED->alive_tick_ms;
    if (now >= last_tick && (now - last_tick) > LIFETRAC_M4_WATCHDOG_MS) {
        trip("alive_tick_stale");
    }

    // 2. Loop-counter advance check (separate witness; catches "millis()
    //    is stuck so alive_tick keeps reading fresh by accident").
    uint32_t lc = SHARED->loop_counter;
    if (lc != s_last_loop_counter) {
        s_last_loop_counter   = lc;
        s_last_loop_change_ms = now;
    } else if ((now - s_last_loop_change_ms) > LIFETRAC_M4_WATCHDOG_MS) {
        trip("loop_counter_stuck");
    }

    // 3. M7 explicitly requested E-stop mirror.
    if (SHARED->estop_request != 0) {
        trip("estop_request");
    }

    // 4. Heartbeat: blink at ~5 Hz so a quick glance confirms M4 is alive
    //    and not in the trip loop.
    if ((now - s_hb_toggle_ms) >= 100) {
        s_hb_toggle_ms = now;
        s_hb_state = !s_hb_state;
        digitalWrite(PIN_M4_HEARTBEAT, s_hb_state ? HIGH : LOW);
    }

    delay(20);
}
