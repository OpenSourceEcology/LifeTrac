// opta_modbus_slave.ino — Arduino Opta WiFi + D1608S + A0602.
//
// DRAFT FOR REVIEW. Not compiled or tested.
//
// SCOPE: Hydraulic-system I/O ONLY, over Modbus-RTU/RS-485.
//   - The Opta's WiFi and BLE radios are intentionally NOT initialised here.
//   - No WiFi.begin(), no BLE.begin(), no MQTT client, no HTTP server.
//   - The legacy Opta firmware that exposed those surfaces is archived in
//     RESEARCH-CONTROLLER/arduino_opta_controller/ and is NOT this file.
//   - The only command/telemetry path in/out of the Opta is the RS-485
//     Modbus link to the Portenta Max Carrier (see MASTER_PLAN.md).
//
// Modbus-RTU SLAVE on RS-485 at 115200 8N1, slave id 0x01.
// Register map matches TRACTOR_NODE.md § Modbus RTU register map.
//
// Owns:
//   - 8 directional valve coils  (4× onboard relays + 4× D1608S relays)
//   - 2× 0–10 V flow set-points  (A0602 AO1/AO2 → Burkert 8605s)
//   - 6× analog telemetry inputs (A0602 AI1..AI6)
//   - independent 200 ms watchdog on REG_WATCHDOG_CTR
//
// Safety:
//   - If REG_WATCHDOG_CTR does not change for > 200 ms → drop ALL coils + zero AOs.
//   - If the external E-stop loop (PIN_ESTOP_LOOP) goes open → same.
//   - If REG_ARM_ESTOP becomes non-zero → same + energize engine-kill.
//   - Boot self-test: cycle each relay briefly, verify A0602 hits midpoint.

#include <Arduino.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
// Real expansion-bus driver from the Arduino_Opta_Blueprints repository.
// On a workstation without the library installed, arduino-cli pulls it via:
//   arduino-cli lib install "Arduino_Opta_Blueprints"
#include <OptaBlue.h>
using namespace Opta;

#define SLAVE_ID 0x01

// ---- Holding registers (master writes) — matches TRACTOR_NODE.md ----
#define REG_VALVE_COILS    0x0000
#define REG_FLOW_SP_1      0x0001
#define REG_FLOW_SP_2      0x0002
#define REG_AUX_OUTPUTS    0x0003
#define REG_WATCHDOG_CTR   0x0004
#define REG_CMD_SOURCE     0x0005
#define REG_ARM_ESTOP      0x0006
#define HOLDING_BLOCK_LEN  7

// ---- Input registers (slave writes) ----
#define REG_SAFETY_STATE   0x0100
#define REG_DIGITAL_INPUTS 0x0101
#define REG_BATTERY_MV     0x0102
#define REG_HYD_SUPPLY     0x0103
#define REG_HYD_RETURN     0x0104
#define REG_COOLANT_C      0x0105
#define REG_OIL_C          0x0106
#define REG_AUX_ANALOG     0x0107
#define REG_RELAY_FAULTS   0x0108
#define REG_UPTIME_LO      0x0109
#define REG_UPTIME_HI      0x010A
#define REG_FW_VERSION     0x010B
#define INPUT_BLOCK_LEN    12

// ---- safety_state values ----
enum SafetyState : uint16_t {
    SAFETY_NORMAL          = 0,
    SAFETY_WATCHDOG_TRIPPED = 1,
    SAFETY_ESTOP_LATCHED   = 2,
    SAFETY_IGNITION_OFF    = 3,
};

// ---- Bit positions inside REG_VALVE_COILS (must match tractor_m7.ino) ----
#define VB_DRIVE_LF     0
#define VB_DRIVE_LR     1
#define VB_DRIVE_RF     2
#define VB_DRIVE_RR     3
#define VB_BOOM_UP      4
#define VB_BOOM_DN      5
#define VB_BUCKET_CURL  6
#define VB_BUCKET_DUMP  7

// ---- Bit positions inside REG_AUX_OUTPUTS (matches TRACTOR_NODE.md §4) ----
// IP-303: bits 0-2 drive the spare D1608S SSR channels reserved for R10-R12.
// Bit 3 ("engine-kill arm") is intentionally NOT honoured here — the live
// engine-kill chain is gated by REG_ARM_ESTOP (0x0006) and the PSR. Mapping
// it twice would let a stale aux-outputs write re-arm the kill solenoid.
#define AUX_R10_BIT     0
#define AUX_R11_BIT     1
#define AUX_R12_BIT     2
#define AUX_OUTPUTS_VALID_MASK 0x0007u   // anything outside this is rejected
// D1608S SSR channels carrying the aux relays. Channels 0-3 are taken by
// the boom/bucket valve coils above (see ``apply_valve_coils``); 4-6 are
// the spare bank documented in TRACTOR_NODE.md.
#define D1608S_CH_R10   4
#define D1608S_CH_R11   5
#define D1608S_CH_R12   6

// Pin map (placeholder — verify against actual Opta + expansion pinout).
// Onboard relays R1..R4 → drive LF/LR/RF/RR (valve bits 0..3).
#define PIN_R1 D0
#define PIN_R2 D1
#define PIN_R3 D2
#define PIN_R4 D3
// D1608S relays R5..R8 → boom up/down + bucket curl/dump (valve bits 4..7).
// A0602 owns AO1/AO2 (flow set-points) and AI1..AI6 (analog telemetry).
// Both expansions are addressed by their position in the chain (0 = closest
// to the Opta). Adjust if the wiring order changes.
#define D1608S_INDEX 0
#define A0602_INDEX  1

#define PIN_ESTOP_LOOP   D4    // digital input, HIGH = healthy
#define PIN_MODE_SW_A    D5    // 2-bit mode switch
#define PIN_MODE_SW_B    D6
#define PIN_PSR_ALIVE    D7    // GPIO into PSR safety relay's monitored channel
#define PIN_ENGINE_KILL  D8    // engine-kill solenoid driver

#define FW_VERSION 0x0100   // BCD: v1.00

static uint32_t g_last_alive_change_ms = 0;
static uint16_t g_last_alive_value     = 0;
static bool     g_alive_seen           = false;   // first write seen yet?
static uint16_t g_safety_state         = SAFETY_NORMAL;
static uint16_t g_relay_fault_flags    = 0;

// ---- expansion-bus driver wrappers ----
// Thin wrappers over Arduino_Opta_Blueprints. Each call asserts the device
// type defensively: a swapped expansion order on the chain would otherwise
// silently send relay commands to the analog board.
static void d1608s_set(uint8_t ch, bool on) {
    Expansion& exp = OptaController.getExpansion(D1608S_INDEX);
    if (exp.getType() != EXPANSION_OPTA_DIGITAL_MEC &&
        exp.getType() != EXPANSION_OPTA_DIGITAL_STS) {
        return;   // wrong board at this index — fail safe (relay stays off)
    }
    DigitalExpansion de = (DigitalExpansion&)exp;
    de.digitalWrite(ch, on ? HIGH : LOW, true /* update now */);
}
static void a0602_write_mv(uint8_t ch, uint16_t value_0_10000) {
    Expansion& exp = OptaController.getExpansion(A0602_INDEX);
    if (exp.getType() != EXPANSION_OPTA_ANALOG) return;
    AnalogExpansion ae = (AnalogExpansion&)exp;
    // The A0602 analog-out range is 0..10000 mV. Library takes mV directly.
    ae.pinVoltage(ch, value_0_10000, true /* update now */);
}
static uint16_t a0602_read(uint8_t ch) {
    Expansion& exp = OptaController.getExpansion(A0602_INDEX);
    if (exp.getType() != EXPANSION_OPTA_ANALOG) return 0;
    AnalogExpansion ae = (AnalogExpansion&)exp;
    // Return raw 16-bit ADC code; downstream scales per channel role
    // (battery / supply / coolant / etc).
    return (uint16_t)ae.pinCurrent(ch);
}

static void all_coils_off() {
    // Onboard relays R1..R4 → bits 0..3
    digitalWrite(PIN_R1, LOW);
    digitalWrite(PIN_R2, LOW);
    digitalWrite(PIN_R3, LOW);
    digitalWrite(PIN_R4, LOW);
    // D1608S relays R5..R8 → bits 4..7
    for (uint8_t ch = 0; ch < 4; ch++) d1608s_set(ch, false);
    // IP-303: aux relays R10..R12 also drop on safe-state.
    d1608s_set(D1608S_CH_R10, false);
    d1608s_set(D1608S_CH_R11, false);
    d1608s_set(D1608S_CH_R12, false);
    // Both flow set-points to 0 V.
    a0602_write_mv(0, 0);
    a0602_write_mv(1, 0);
}

// Apply the full 8-bit valve_coils bitfield in one call.
static void apply_valve_coils(uint16_t coils) {
    digitalWrite(PIN_R1, (coils >> VB_DRIVE_LF) & 1 ? HIGH : LOW);
    digitalWrite(PIN_R2, (coils >> VB_DRIVE_LR) & 1 ? HIGH : LOW);
    digitalWrite(PIN_R3, (coils >> VB_DRIVE_RF) & 1 ? HIGH : LOW);
    digitalWrite(PIN_R4, (coils >> VB_DRIVE_RR) & 1 ? HIGH : LOW);
    d1608s_set(0, (coils >> VB_BOOM_UP)     & 1);
    d1608s_set(1, (coils >> VB_BOOM_DN)     & 1);
    d1608s_set(2, (coils >> VB_BUCKET_CURL) & 1);
    d1608s_set(3, (coils >> VB_BUCKET_DUMP) & 1);
}

static uint16_t read_analog(uint8_t i) { return a0602_read(i); }

static uint8_t read_mode_switch() {
    return (digitalRead(PIN_MODE_SW_A) << 1) | digitalRead(PIN_MODE_SW_B);
}

static void enter_safe_state(SafetyState why) {
    g_safety_state = why;
    all_coils_off();
    digitalWrite(PIN_PSR_ALIVE, LOW);   // drop the PSR
}

static void check_safety() {
    uint32_t now = millis();
    // 1. Watchdog: master must change REG_WATCHDOG_CTR within 200 ms.
    if (g_alive_seen && (now - g_last_alive_change_ms) > 200) {
        if (g_safety_state == SAFETY_NORMAL) enter_safe_state(SAFETY_WATCHDOG_TRIPPED);
    }
    // 2. External E-stop loop open.
    if (digitalRead(PIN_ESTOP_LOOP) == LOW) {
        if (g_safety_state == SAFETY_NORMAL) enter_safe_state(SAFETY_ESTOP_LATCHED);
    }
}

// Modbus write callback. Called ONLY when a register actually transitions
// (see loop() below for how we gate this) — so REG_WATCHDOG_CTR side effects
// only fire on real master writes, not on every poll iteration.
static void on_holding_change(uint16_t address, uint16_t value) {
    // Refuse motion writes while latched in a fault state. The master can
    // still write REG_WATCHDOG_CTR (we want to keep observing master health)
    // and the operator's clear command lands as a future CMD over the air,
    // not via Modbus, so nothing on the bus can clear safety state.
    if (g_safety_state != SAFETY_NORMAL &&
        address != REG_WATCHDOG_CTR) {
        return;
    }

    switch (address) {
        case REG_VALVE_COILS:
            apply_valve_coils(value);
            break;
        case REG_FLOW_SP_1:
            a0602_write_mv(0, value > 10000 ? 10000 : value);
            break;
        case REG_FLOW_SP_2:
            a0602_write_mv(1, value > 10000 ? 10000 : value);
            break;
        case REG_AUX_OUTPUTS:
            // IP-303: drive only the documented bits (R10..R12). Anything
            // outside ``AUX_OUTPUTS_VALID_MASK`` is silently dropped so a
            // future expansion with a typo'd bitfield can't unexpectedly
            // energize R5..R8 (those are owned by the valve manifold above).
            {
                uint16_t aux = value & AUX_OUTPUTS_VALID_MASK;
                d1608s_set(D1608S_CH_R10, (aux >> AUX_R10_BIT) & 1);
                d1608s_set(D1608S_CH_R11, (aux >> AUX_R11_BIT) & 1);
                d1608s_set(D1608S_CH_R12, (aux >> AUX_R12_BIT) & 1);
            }
            break;
        case REG_WATCHDOG_CTR:
            g_last_alive_value     = value;
            g_last_alive_change_ms = millis();
            g_alive_seen           = true;
            break;
        case REG_CMD_SOURCE:
            // Logging only — not safety-relevant.
            break;
        case REG_ARM_ESTOP:
            if (value != 0) {
                digitalWrite(PIN_ENGINE_KILL, HIGH);
                if (g_safety_state == SAFETY_NORMAL) {
                    enter_safe_state(SAFETY_ESTOP_LATCHED);
                }
            } else {
                digitalWrite(PIN_ENGINE_KILL, LOW);
            }
            break;
    }
}

// ---------- Arduino entry points ----------
void setup() {
    Serial.begin(115200);

    // Expansion bus must come up BEFORE we touch any d1608s_* / a0602_*
    // helper, otherwise getExpansion() returns an UNKNOWN stub.
    OptaController.begin();
    OptaController.update();

    pinMode(PIN_R1, OUTPUT);
    pinMode(PIN_R2, OUTPUT);
    pinMode(PIN_R3, OUTPUT);
    pinMode(PIN_R4, OUTPUT);
    pinMode(PIN_ESTOP_LOOP,  INPUT);
    pinMode(PIN_MODE_SW_A,   INPUT_PULLUP);
    pinMode(PIN_MODE_SW_B,   INPUT_PULLUP);
    pinMode(PIN_PSR_ALIVE,   OUTPUT);
    pinMode(PIN_ENGINE_KILL, OUTPUT);
    digitalWrite(PIN_PSR_ALIVE,   HIGH);   // assume healthy at boot
    digitalWrite(PIN_ENGINE_KILL, LOW);

    all_coils_off();

    // RS-485 + Modbus slave (115200 8N1)
    if (!ModbusRTUServer.begin(SLAVE_ID, 115200)) {
        Serial.println("Modbus slave begin failed");
        while (1) {}
    }
    // Allocate the holding window 0x0000..0x0006 + the input window
    // 0x0100..0x010B. (configureInputRegisters takes start + count.)
    ModbusRTUServer.configureHoldingRegisters(0x0000, HOLDING_BLOCK_LEN);
    ModbusRTUServer.configureInputRegisters(0x0100, INPUT_BLOCK_LEN);
    ModbusRTUServer.inputRegisterWrite(REG_FW_VERSION, FW_VERSION);

    // Boot self-test stub: blink each onboard relay 50 ms.
    // IP-308: iterate over an explicit pin table rather than ``PIN_R1 + i``.
    // The Rev-A board happens to have D0..D3 contiguous, but a future board
    // revision may break that assumption silently and this loop would then
    // pulse the wrong GPIOs.
    static const uint8_t kRelayPins[4] = { PIN_R1, PIN_R2, PIN_R3, PIN_R4 };
    for (uint8_t i = 0; i < 4; i++) {
        digitalWrite(kRelayPins[i], HIGH); delay(50); digitalWrite(kRelayPins[i], LOW);
    }
}

// Last-seen value for every writable holding register, so we can detect a
// real master write versus our own polling reads. This is the core of the
// fix for the "watchdog refreshed by error scan" bug — we only invoke side
// effects when the value transitions.
static uint16_t s_last_seen[HOLDING_BLOCK_LEN];
static bool     s_seen_init = false;

void loop() {
    // Pump expansion-bus traffic every loop iteration — the library queues
    // analog reads / digital writes here.
    OptaController.update();

    int n = ModbusRTUServer.poll();
    (void)n;   // we don't need the count; we walk the writable block ourselves.

    if (!s_seen_init) {
        for (uint16_t i = 0; i < HOLDING_BLOCK_LEN; i++) {
            s_last_seen[i] = ModbusRTUServer.holdingRegisterRead(i);
        }
        s_seen_init = true;
    } else {
        for (uint16_t a = 0; a < HOLDING_BLOCK_LEN; a++) {
            uint16_t v = ModbusRTUServer.holdingRegisterRead(a);
            if (v != s_last_seen[a]) {
                s_last_seen[a] = v;
                on_holding_change(a, v);
            } else if (a == REG_WATCHDOG_CTR && g_alive_seen) {
                // Special case: watchdog counter that hasn't changed since
                // last loop is *expected* between master writes — do nothing.
                // (The freshness check lives in check_safety().)
            }
        }
    }

    check_safety();

    // Refresh INPUT registers (master reads via function 0x04).
    ModbusRTUServer.inputRegisterWrite(REG_SAFETY_STATE, g_safety_state);
    uint16_t din = (digitalRead(PIN_ESTOP_LOOP) == HIGH ? 1u : 0u)
                 | ((uint16_t)read_mode_switch() << 1);
    ModbusRTUServer.inputRegisterWrite(REG_DIGITAL_INPUTS, din);
    for (uint8_t i = 0; i < 6; i++) {
        ModbusRTUServer.inputRegisterWrite(REG_BATTERY_MV + i, read_analog(i));
    }
    ModbusRTUServer.inputRegisterWrite(REG_RELAY_FAULTS, g_relay_fault_flags);
    uint32_t up_s = millis() / 1000u;
    ModbusRTUServer.inputRegisterWrite(REG_UPTIME_LO, (uint16_t)(up_s & 0xFFFF));
    ModbusRTUServer.inputRegisterWrite(REG_UPTIME_HI, (uint16_t)(up_s >> 16));
}
