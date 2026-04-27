// tractor_m4.cpp — Portenta H747 M4 core: independent valve safety watchdog.
//
// DRAFT FOR REVIEW. Not compiled or tested.
//
// The M4 runs alongside the M7 on the same H747 silicon. It does ONE thing:
// observe an "alive tick" the M7 publishes via shared SRAM, and pull a GPIO
// low if the M7 stops ticking for >200 ms. That GPIO feeds the PSR safety
// relay's monitored channel — see TRACTOR_NODE.md § Wiring overview.
//
// Why bother, when the Opta also has a Modbus-side watchdog?
//   Defense in depth. M4 watchdog catches "M7 is alive but the Modbus link
//   is wedged"; Opta watchdog catches "M7 *and* M4 are both dead."

#include "Arduino.h"
#include "mbed.h"

// Shared memory layout — MUST match the M7 side. We'll formalize this in
// firmware/common/shared_mem.h once we choose the layout.
struct SharedM7M4 {
    volatile uint32_t alive_tick_ms;   // M7 writes millis() each loop iteration
    volatile uint32_t reserved[15];
};

// Place at a fixed SRAM4 address visible to both cores.
static volatile SharedM7M4* const SHARED =
    reinterpret_cast<volatile SharedM7M4*>(0x38000000);

// "Alive" GPIO into the PSR safety relay's monitored channel.
// Active-high = healthy; pulled low on watchdog trip.
#define PIN_SAFETY_ALIVE  PG_3

static const uint32_t WATCHDOG_TIMEOUT_MS = 200;

void setup() {
    pinMode(PIN_SAFETY_ALIVE, OUTPUT);
    digitalWrite(PIN_SAFETY_ALIVE, HIGH);   // assume healthy at boot
    // Give the M7 ~500 ms to come up before we start checking.
    delay(500);
}

void loop() {
    uint32_t now = millis();
    uint32_t last = SHARED->alive_tick_ms;
    if ((now - last) > WATCHDOG_TIMEOUT_MS) {
        digitalWrite(PIN_SAFETY_ALIVE, LOW);   // PSR drops out → valves dead
        // Latch low until reset — only an MCU reset re-arms.
        while (1) { delay(1000); }
    }
    delay(20);
}
