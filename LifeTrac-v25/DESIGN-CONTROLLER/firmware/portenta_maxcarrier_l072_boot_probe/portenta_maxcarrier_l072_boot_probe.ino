#include <Arduino.h>

#if defined(ARDUINO_PORTENTA_X8)
#include <SerialRPC.h>
#define LT_LOG_SERIAL SerialRPC
#else
#define LT_LOG_SERIAL Serial
#endif

// Default Max Carrier LoRa control pins used by Arduino's PORTENTA_CARRIER path.
#ifndef LORA_RESET
#define LORA_RESET (PD_5)
#endif

#ifndef LORA_BOOT0
#define LORA_BOOT0 (PJ_11)
#endif

// Default UART object differs by core variant; override at compile time if needed.
#ifndef LIFETRAC_L072_BOOT_SERIAL
#if defined(ARDUINO_PORTENTA_X8)
#define LIFETRAC_L072_BOOT_SERIAL Serial1
#else
#define LIFETRAC_L072_BOOT_SERIAL Serial2
#endif
#endif

#ifndef LIFETRAC_L072_BOOT_BAUD
#define LIFETRAC_L072_BOOT_BAUD 115200UL
#endif

namespace {

HardwareSerial *g_lora_serial = &LIFETRAC_L072_BOOT_SERIAL;
bool g_boot0_high = false;
bool g_probe_ok = false;
bool g_probe_ran = false;
uint32_t g_last_fail_blink_ms = 0;

static bool wait_for_byte(uint8_t *out, uint32_t timeout_ms) {
  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    if (g_lora_serial->available() > 0) {
      const int c = g_lora_serial->read();
      if (c >= 0) {
        *out = (uint8_t)c;
        return true;
      }
    }
  }
  return false;
}

static void flush_rx(void) {
  while (g_lora_serial->available() > 0) {
    (void)g_lora_serial->read();
  }
}

static void set_boot0(bool high) {
  pinMode(LORA_BOOT0, OUTPUT);
  digitalWrite(LORA_BOOT0, high ? HIGH : LOW);
  g_boot0_high = high;
}

static void pulse_reset(void) {
  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, HIGH);
  delay(40);
  digitalWrite(LORA_RESET, LOW);
  delay(80);
  digitalWrite(LORA_RESET, HIGH);
  delay(120);
}

static bool boot_sync(void) {
  const uint8_t sync = 0x7F;
  uint8_t ack = 0x00;

  flush_rx();
  g_lora_serial->write(&sync, 1);

  if (!wait_for_byte(&ack, 500)) {
    LT_LOG_SERIAL.println("[probe] sync timeout");
    return false;
  }

  if (ack != 0x79) {
    LT_LOG_SERIAL.print("[probe] sync got 0x");
    LT_LOG_SERIAL.println(ack, HEX);
    return false;
  }

  LT_LOG_SERIAL.println("[probe] sync ACK (0x79)");
  return true;
}

static bool send_cmd(uint8_t cmd) {
  uint8_t frame[2];
  uint8_t ack = 0x00;

  frame[0] = cmd;
  frame[1] = (uint8_t)(cmd ^ 0xFFU);
  g_lora_serial->write(frame, sizeof(frame));

  if (!wait_for_byte(&ack, 500)) {
    LT_LOG_SERIAL.println("[probe] cmd timeout");
    return false;
  }

  if (ack != 0x79) {
    LT_LOG_SERIAL.print("[probe] cmd NACK/other 0x");
    LT_LOG_SERIAL.println(ack, HEX);
    return false;
  }

  return true;
}

static bool read_get_version(void) {
  uint8_t ver = 0;
  uint8_t opt1 = 0;
  uint8_t opt2 = 0;
  uint8_t tail = 0;

  if (!send_cmd(0x01)) {
    LT_LOG_SERIAL.println("[probe] GET_VERSION command failed");
    return false;
  }

  if (!wait_for_byte(&ver, 500) || !wait_for_byte(&opt1, 500) || !wait_for_byte(&opt2, 500) ||
      !wait_for_byte(&tail, 500)) {
    LT_LOG_SERIAL.println("[probe] GET_VERSION response timeout");
    return false;
  }

  if (tail != 0x79) {
    LT_LOG_SERIAL.print("[probe] GET_VERSION tail 0x");
    LT_LOG_SERIAL.println(tail, HEX);
    return false;
  }

  LT_LOG_SERIAL.print("[probe] BL version 0x");
  LT_LOG_SERIAL.print(ver, HEX);
  LT_LOG_SERIAL.print(" opt1=0x");
  LT_LOG_SERIAL.print(opt1, HEX);
  LT_LOG_SERIAL.print(" opt2=0x");
  LT_LOG_SERIAL.println(opt2, HEX);
  return true;
}

static bool read_get_id(void) {
  uint8_t n = 0;
  uint8_t pid_msb = 0;
  uint8_t pid_lsb = 0;
  uint8_t tail = 0;

  if (!send_cmd(0x02)) {
    LT_LOG_SERIAL.println("[probe] GET_ID command failed");
    return false;
  }

  if (!wait_for_byte(&n, 500) || !wait_for_byte(&pid_msb, 500) || !wait_for_byte(&pid_lsb, 500) ||
      !wait_for_byte(&tail, 500)) {
    LT_LOG_SERIAL.println("[probe] GET_ID response timeout");
    return false;
  }

  if (tail != 0x79) {
    LT_LOG_SERIAL.print("[probe] GET_ID tail 0x");
    LT_LOG_SERIAL.println(tail, HEX);
    return false;
  }

  const uint16_t pid = (uint16_t)(((uint16_t)pid_msb << 8) | pid_lsb);
  LT_LOG_SERIAL.print("[probe] PID bytes=");
  LT_LOG_SERIAL.print((unsigned)n + 1U);
  LT_LOG_SERIAL.print(" chip-id=0x");
  LT_LOG_SERIAL.println(pid, HEX);
  return true;
}

static void enter_bootloader_and_probe(void) {
  LT_LOG_SERIAL.println("[probe] BOOT0=HIGH, pulsing NRST...");
  set_boot0(true);
  pulse_reset();

  g_lora_serial->end();
  g_lora_serial->begin(LIFETRAC_L072_BOOT_BAUD, SERIAL_8E1);
  delay(30);

  if (!boot_sync()) {
    LT_LOG_SERIAL.println("[probe] FAIL: no STM32 ROM bootloader ACK");
    g_probe_ok = false;
    g_probe_ran = true;
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  bool ok = true;
  ok &= read_get_version();
  ok &= read_get_id();
  g_probe_ok = ok;
  g_probe_ran = true;
  digitalWrite(LED_BUILTIN, ok ? HIGH : LOW);
  LT_LOG_SERIAL.println("[probe] probe complete");
}

static void leave_bootloader_to_app(void) {
  LT_LOG_SERIAL.println("[probe] BOOT0=LOW, pulsing NRST to app");
  set_boot0(false);
  pulse_reset();

  g_lora_serial->end();
  g_lora_serial->begin(19200UL, SERIAL_8N1);
  delay(30);
  LT_LOG_SERIAL.println("[probe] switched LoRa UART to 19200 8N1 for stock AT probing");
}

} // namespace

void setup() {
  LT_LOG_SERIAL.begin(115200);
  while (!LT_LOG_SERIAL) {
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  LT_LOG_SERIAL.println();
  LT_LOG_SERIAL.println("=== Portenta Max Carrier L072 Bootloader Probe ===");
  LT_LOG_SERIAL.println("Commands:");
  LT_LOG_SERIAL.println("  b -> enter ROM bootloader and probe ACK/version/id");
  LT_LOG_SERIAL.println("  a -> return to app boot (BOOT0 low + reset)");
  LT_LOG_SERIAL.println("  r -> pulse reset without changing BOOT0");
  LT_LOG_SERIAL.println();

  set_boot0(false);
  pulse_reset();
  g_lora_serial->begin(19200UL, SERIAL_8N1);

  // Auto-run once so the board can be tested immediately after upload.
  enter_bootloader_and_probe();
  leave_bootloader_to_app();
}

void loop() {
  // Without serial console access on X8, LED gives immediate probe status:
  // ON = pass, slow blink = fail.
  if (g_probe_ran && !g_probe_ok) {
    const uint32_t now = millis();
    if ((now - g_last_fail_blink_ms) >= 500U) {
      g_last_fail_blink_ms = now;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }

  if (LT_LOG_SERIAL.available() <= 0) {
    return;
  }

  const int c = LT_LOG_SERIAL.read();
  switch (c) {
    case 'b':
    case 'B':
      enter_bootloader_and_probe();
      break;

    case 'a':
    case 'A':
      leave_bootloader_to_app();
      break;

    case 'r':
    case 'R':
      LT_LOG_SERIAL.print("[probe] reset pulse (BOOT0 currently ");
      LT_LOG_SERIAL.print(g_boot0_high ? "HIGH" : "LOW");
      LT_LOG_SERIAL.println(")");
      pulse_reset();
      break;

    default:
      LT_LOG_SERIAL.print("[probe] unknown command: ");
      LT_LOG_SERIAL.println((char)c);
      break;
  }
}
