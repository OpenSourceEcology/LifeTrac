// tractor_h7_m4.ino — Portenta H747 M4 core: independent valve safety watchdog.
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
//
// SKETCH LAYOUT (Round 17): the M4 firmware lives in this dedicated sketch
// folder, separate from ../tractor_h7/. Reason: arduino-cli scans the .ino
// for #include directives during library auto-discovery REGARDLESS of the
// selected target_core, so building the M7 sketch (which pulls in RadioLib
// + ArduinoModbus + ArduinoRS485) for the M4 core fails in those library
// sources. Splitting into two sketch folders gives each core its own
// minimal include set. The Arduino IDE's stock "PortentaDualCore" example
// uses the same two-folder layout.
//
// Build: open this folder in the IDE, select board "Portenta H7" with
// menu "Target core: M4 Co-processor" (FQBN
// ``arduino:mbed_portenta:envie_m7:target_core=cm4``), upload. CI builds
// it via the ``firmware-compile-tractor-m4`` job in arduino-ci.yml.
//
// Shared sources (``shared_mem.h``) are staged into ``src/`` by
// ``tools/stage_firmware_sketches.ps1`` (local) and by the workflow
// (CI). The canonical copy lives at
// ``DESIGN-CONTROLLER/firmware/common/shared_mem.h``.

#include "Arduino.h"
#include "mbed.h"
#include "src/shared_mem.h"

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

    // IP-105: take a coherent snapshot of the M7-side fields using the
    // shared_mem.h seqlock. Retry up to a few times if the writer is
    // mid-update; bail out and trip if we can't get a consistent read.
    uint32_t snap_alive   = 0;
    uint32_t snap_loop_ct = 0;
    uint32_t snap_estop   = 0;
    bool     snap_ok      = false;
    for (int attempt = 0; attempt < 4; attempt++) {
        uint32_t s0 = SHARED->seq;
        if (s0 & 1u) { continue; }                 // writer in progress
        snap_alive   = SHARED->alive_tick_ms;
        snap_loop_ct = SHARED->loop_counter;
        snap_estop   = SHARED->estop_request;
        __DMB();                                    // observe writes-in-order
        uint32_t s1 = SHARED->seq;
        if (s0 == s1) { snap_ok = true; break; }
    }
    if (!snap_ok) {
        // Writer is hammering the struct (or RAM is corrupt); treat as
        // unsafe and trip rather than acting on a torn read.
        trip("seqlock_torn");
        return;
    }

    // 1. Liveness tick check.
    if (now >= snap_alive && (now - snap_alive) > LIFETRAC_M4_WATCHDOG_MS) {
        trip("alive_tick_stale");
    }

    // 2. Loop-counter advance check (separate witness; catches "millis()
    //    is stuck so alive_tick keeps reading fresh by accident").
    if (snap_loop_ct != s_last_loop_counter) {
        s_last_loop_counter   = snap_loop_ct;
        s_last_loop_change_ms = now;
    } else if ((now - s_last_loop_change_ms) > LIFETRAC_M4_WATCHDOG_MS) {
        trip("loop_counter_stuck");
    }

    // 3. M7 explicitly requested E-stop mirror. IP-106: gate on the magic
    //    constant so SRAM noise / stale boot bytes cannot trip the chain.
    if (snap_estop == LIFETRAC_ESTOP_MAGIC) {
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
