// opta_modbus_slave.ino — Arduino Opta WiFi + D1608S + A0602.
//
// DRAFT FOR REVIEW. Not compiled or tested.
//
// Modbus-RTU SLAVE on RS-485 at 115200 8N1, slave id 0x10.
// Holds the register map the tractor M7 master reads/writes.
//
// Owns:
//   - 8 directional valve coils  (4× onboard relays + 4× D1608S relays)
//   - 2× 0–10 V flow set-points  (A0602 AO1/AO2 → Burkert 8605s)
//   - 6× analog telemetry inputs (A0602 AI1..AI6)
//   - independent 200 ms watchdog on REG_ALIVE_TICK
//
// Safety:
//   - If REG_ALIVE_TICK does not change for > 200 ms → drop ALL coils + zero AOs.
//   - If REG_ESTOP_LOOP digital input goes open → same.
//   - Boot self-test: cycle each relay briefly, verify A0602 hits midpoint.

#include <Arduino.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

#define SLAVE_ID 0x10

// Register map — mirror of tractor_m7.ino enum OptaReg.
#define REG_ALIVE_TICK    0x0000
#define REG_FLOW_SP_1     0x0001
#define REG_FLOW_SP_2     0x0002
#define REG_VALVE_BASE    0x0010
#define REG_CURRENT_BASE  0x0020
#define REG_MODE_SWITCH   0x0030
#define REG_ESTOP_LOOP    0x0031
#define REG_ANALOG_BASE   0x0040
#define REG_LAST_ERROR    0x00FF

// Pin map (placeholder — verify against actual Opta + expansion pinout).
// Onboard relays R1..R4
#define PIN_R1 D0
#define PIN_R2 D1
#define PIN_R3 D2
#define PIN_R4 D3
// D1608S relays R5..R8 (addressed via OptaController library or expansion API)
// (Pseudocode here; the real library call is OptaController.setRelay(expansion, idx, val).)
// A0602 0-10 V outputs
// A0602 6 analog inputs

#define PIN_ESTOP_LOOP   D4    // digital input, HIGH = healthy
#define PIN_MODE_SW_A    D5    // 2-bit mode switch
#define PIN_MODE_SW_B    D6

static uint32_t g_last_alive_change_ms = 0;
static uint16_t g_last_alive_value     = 0xFFFF;
static uint16_t g_last_error           = 0;

#define ERR_WATCHDOG_TRIP   (1u << 0)
#define ERR_ESTOP_OPEN      (1u << 1)
#define ERR_OVERCURRENT_BASE (1u << 4)   // bits 4..11 = per-coil overcurrent

static void all_coils_off() {
    digitalWrite(PIN_R1, LOW);
    digitalWrite(PIN_R2, LOW);
    digitalWrite(PIN_R3, LOW);
    digitalWrite(PIN_R4, LOW);
    // TODO: OptaController.setRelay(D1608S, 0..3, LOW) for R5..R8
    // TODO: A0602 AO1/AO2 → 0
}

static void apply_valve_register(uint8_t idx, uint16_t value) {
    bool on = (value != 0);
    switch (idx) {
        case 0: digitalWrite(PIN_R1, on); break;
        case 1: digitalWrite(PIN_R2, on); break;
        case 2: digitalWrite(PIN_R3, on); break;
        case 3: digitalWrite(PIN_R4, on); break;
        // case 4..7: D1608S R5..R8
    }
}

static uint16_t read_analog(uint8_t i) {
    // A0602 inputs are 0–10 V scaled to 0..65535. Library call placeholder:
    // return OptaController.readAnalog(A0602, i);
    return 0;
}

static uint16_t read_current(uint8_t i) {
    // Per-coil current via INA shunt (TODO add to BOM if not on Opta natively).
    (void)i;
    return 0;
}

static uint8_t read_mode_switch() {
    return (digitalRead(PIN_MODE_SW_A) << 1) | digitalRead(PIN_MODE_SW_B);
}

static void check_safety() {
    uint32_t now = millis();
    if ((now - g_last_alive_change_ms) > 200) {
        if (!(g_last_error & ERR_WATCHDOG_TRIP)) {
            g_last_error |= ERR_WATCHDOG_TRIP;
            all_coils_off();
        }
    }
    if (digitalRead(PIN_ESTOP_LOOP) == LOW) {
        if (!(g_last_error & ERR_ESTOP_OPEN)) {
            g_last_error |= ERR_ESTOP_OPEN;
            all_coils_off();
        }
    }
}

// Modbus callbacks --------------------------------------------------------

static int on_holding_write(uint16_t address, uint16_t value) {
    // Refuse all writes while error latched (master must clear via REG_LAST_ERROR=0).
    if (g_last_error != 0 && address != REG_LAST_ERROR) return -1;

    if (address == REG_ALIVE_TICK) {
        if (value != g_last_alive_value) {
            g_last_alive_value     = value;
            g_last_alive_change_ms = millis();
        }
        return 0;
    }
    if (address == REG_FLOW_SP_1) {
        // OptaController.writeAnalog(A0602, 0, scale(value));
        return 0;
    }
    if (address == REG_FLOW_SP_2) {
        // OptaController.writeAnalog(A0602, 1, scale(value));
        return 0;
    }
    if (address >= REG_VALVE_BASE && address < REG_VALVE_BASE + 8) {
        apply_valve_register(address - REG_VALVE_BASE, value);
        return 0;
    }
    if (address == REG_LAST_ERROR && value == 0) {
        // Operator cleared latched faults from the base UI.
        g_last_error = 0;
        g_last_alive_change_ms = millis();
        return 0;
    }
    return -1;
}

// ---------- Arduino entry points ----------
void setup() {
    Serial.begin(115200);
    pinMode(PIN_R1, OUTPUT);
    pinMode(PIN_R2, OUTPUT);
    pinMode(PIN_R3, OUTPUT);
    pinMode(PIN_R4, OUTPUT);
    pinMode(PIN_ESTOP_LOOP, INPUT);
    pinMode(PIN_MODE_SW_A,  INPUT_PULLUP);
    pinMode(PIN_MODE_SW_B,  INPUT_PULLUP);

    all_coils_off();

    // RS-485 + Modbus slave (115200 8N1)
    if (!ModbusRTUServer.begin(SLAVE_ID, 115200)) {
        Serial.println("Modbus slave begin failed");
        while (1) {}
    }
    ModbusRTUServer.configureHoldingRegisters(0x0000, 0x0100);

    // Boot self-test stub: blink each relay 50 ms.
    for (uint8_t i = 0; i < 4; i++) {
        apply_valve_register(i, 1);
        delay(50);
        apply_valve_register(i, 0);
    }

    g_last_alive_change_ms = millis();
}

void loop() {
    int n = ModbusRTUServer.poll();
    if (n > 0) {
        // Walk the holding-register window we care about and apply side effects.
        // (ArduinoModbus exposes per-register holders; for brevity, scan only
        // the writable subset.)
        for (uint16_t a : {REG_ALIVE_TICK, REG_FLOW_SP_1, REG_FLOW_SP_2,
                           REG_LAST_ERROR}) {
            on_holding_write(a, ModbusRTUServer.holdingRegisterRead(a));
        }
        for (uint16_t i = 0; i < 8; i++) {
            on_holding_write(REG_VALVE_BASE + i,
                ModbusRTUServer.holdingRegisterRead(REG_VALVE_BASE + i));
        }
    }

    check_safety();

    // Refresh read-only telemetry registers for the next master poll.
    ModbusRTUServer.holdingRegisterWrite(REG_MODE_SWITCH, read_mode_switch());
    ModbusRTUServer.holdingRegisterWrite(REG_ESTOP_LOOP,
        digitalRead(PIN_ESTOP_LOOP) == HIGH ? 1 : 0);
    for (uint8_t i = 0; i < 6; i++) {
        ModbusRTUServer.holdingRegisterWrite(REG_ANALOG_BASE + i, read_analog(i));
    }
    for (uint8_t i = 0; i < 8; i++) {
        ModbusRTUServer.holdingRegisterWrite(REG_CURRENT_BASE + i, read_current(i));
    }
    ModbusRTUServer.holdingRegisterWrite(REG_LAST_ERROR, g_last_error);
}
