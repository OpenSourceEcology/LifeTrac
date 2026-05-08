#include <Arduino.h>

#include <string.h>

#ifndef LIFETRAC_PC_SERIAL
#if defined(ARDUINO_PORTENTA_X8)
#define LIFETRAC_PC_SERIAL Serial1
#ifndef LIFETRAC_PC_SERIAL_NAME
#define LIFETRAC_PC_SERIAL_NAME "Serial1"
#endif
#ifndef LIFETRAC_PC_WAIT_FOR_CONNECT
#define LIFETRAC_PC_WAIT_FOR_CONNECT 0
#endif
#else
#define LIFETRAC_PC_SERIAL Serial
#ifndef LIFETRAC_PC_SERIAL_NAME
#define LIFETRAC_PC_SERIAL_NAME "Serial"
#endif
#ifndef LIFETRAC_PC_WAIT_FOR_CONNECT
#define LIFETRAC_PC_WAIT_FOR_CONNECT 1
#endif
#endif
#endif

#ifndef LIFETRAC_PC_SERIAL_NAME
#define LIFETRAC_PC_SERIAL_NAME "custom"
#endif

#ifndef LIFETRAC_PC_WAIT_FOR_CONNECT
#define LIFETRAC_PC_WAIT_FOR_CONNECT 0
#endif

#define LT_PC_SERIAL LIFETRAC_PC_SERIAL

#ifndef LORA_RESET
#define LORA_RESET (PD_5)
#endif

#ifndef LORA_BOOT0
#define LORA_BOOT0 (PJ_11)
#endif

#ifndef LIFETRAC_L072_SERIAL
#define LIFETRAC_L072_SERIAL Serial2
#endif

#if defined(ARDUINO_PORTENTA_X8) && (SERIAL_HOWMANY < 2)
#ifndef SERIAL2_TX
#define SERIAL2_TX PA_15
#endif
#ifndef SERIAL2_RX
#define SERIAL2_RX PF_6
#endif
#ifndef SERIAL2_RTS
#define SERIAL2_RTS PF_8
#endif
#ifndef SERIAL2_CTS
#define SERIAL2_CTS PF_9
#endif
arduino::UART _UART2_(SERIAL2_TX, SERIAL2_RX, SERIAL2_RTS, SERIAL2_CTS);
#endif

#ifndef LIFETRAC_PC_BAUD
#define LIFETRAC_PC_BAUD 115200UL
#endif

#ifndef LIFETRAC_L072_AT_BAUD
#define LIFETRAC_L072_AT_BAUD 115200UL
#endif

#ifndef LIFETRAC_L072_BOOT_BAUD
#define LIFETRAC_L072_BOOT_BAUD 115200UL
#endif

#ifndef LIFETRAC_ENABLE_BOOT_PINS
#define LIFETRAC_ENABLE_BOOT_PINS 0
#endif

namespace {

HardwareSerial *g_lora = &LIFETRAC_L072_SERIAL;
uint32_t g_lora_baud = LIFETRAC_L072_AT_BAUD;
bool g_bridge_enabled = true;
bool g_local_commands_enabled = true;
bool g_at_ping_ok = false;
char g_cmd_buf[24];
uint8_t g_cmd_len = 0;
uint32_t g_last_status_ms = 0;

const uint32_t kAtBaudCandidates[] = {
  LIFETRAC_L072_AT_BAUD,
  19200UL,
  921600UL,
  9600UL,
  115200UL,
};

static void led_rgb(bool red, bool green, bool blue) {
#if defined(LEDR) && defined(LEDG) && defined(LEDB)
  digitalWrite(LEDR, red ? LOW : HIGH);
  digitalWrite(LEDG, green ? LOW : HIGH);
  digitalWrite(LEDB, blue ? LOW : HIGH);
#else
  digitalWrite(LED_BUILTIN, (red || green || blue) ? HIGH : LOW);
#endif
}

static void set_led_unknown() {
  led_rgb(false, false, true);
}

static void set_led_pass() {
  led_rgb(false, true, false);
}

static void set_led_fail() {
  led_rgb(true, false, false);
}

static void lora_begin(uint32_t baud, uint32_t config) {
  g_lora->end();
  delay(20);
  g_lora->begin(baud, config);
  g_lora_baud = baud;
  delay(40);
}

static void lora_flush_rx(uint32_t timeout_ms = 20) {
  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    while (g_lora->available() > 0) {
      (void)g_lora->read();
    }
  }
}

static bool wait_for_token(const char *token, uint32_t timeout_ms) {
  const uint32_t start = millis();
  size_t matched = 0;

  while ((millis() - start) < timeout_ms) {
    while (g_lora->available() > 0) {
      const int value = g_lora->read();
      if (value < 0) {
        continue;
      }

      LT_PC_SERIAL.write((uint8_t)value);
      if ((char)value == token[matched]) {
        matched++;
        if (token[matched] == '\0') {
          return true;
        }
      } else {
        matched = ((char)value == token[0]) ? 1U : 0U;
      }
    }
  }

  return false;
}

static bool at_ping(uint32_t baud) {
  LT_PC_SERIAL.print("[bridge] AT ping @ ");
  LT_PC_SERIAL.println(baud);
  lora_begin(baud, SERIAL_8N1);
  lora_flush_rx();
  g_lora->print("AT\r\n");
  return wait_for_token("OK", 750);
}

static bool at_ping_scan() {
  uint32_t tested[sizeof(kAtBaudCandidates) / sizeof(kAtBaudCandidates[0])];
  uint8_t tested_count = 0;

  for (uint8_t i = 0; i < (sizeof(kAtBaudCandidates) / sizeof(kAtBaudCandidates[0])); i++) {
    const uint32_t baud = kAtBaudCandidates[i];
    bool already_tested = false;
    for (uint8_t j = 0; j < tested_count; j++) {
      if (tested[j] == baud) {
        already_tested = true;
        break;
      }
    }
    if (already_tested) {
      continue;
    }

    tested[tested_count++] = baud;
    if (at_ping(baud)) {
      return true;
    }
  }

  lora_begin(LIFETRAC_L072_AT_BAUD, SERIAL_8N1);
  return false;
}

static void pulse_reset() {
#if LIFETRAC_ENABLE_BOOT_PINS
  digitalWrite(LORA_RESET, HIGH);
  delay(40);
  digitalWrite(LORA_RESET, LOW);
  delay(80);
  digitalWrite(LORA_RESET, HIGH);
  delay(180);
#else
  LT_PC_SERIAL.println("[bridge] reset pin disabled; rebuild with LIFETRAC_ENABLE_BOOT_PINS=1");
#endif
}

static void set_boot0(bool high) {
#if LIFETRAC_ENABLE_BOOT_PINS
  digitalWrite(LORA_BOOT0, high ? HIGH : LOW);
#else
  LT_PC_SERIAL.print("[bridge] BOOT0 request ignored: ");
  LT_PC_SERIAL.println(high ? "HIGH" : "LOW");
  LT_PC_SERIAL.println("[bridge] rebuild with LIFETRAC_ENABLE_BOOT_PINS=1 after confirming LORA_BOOT0/LORA_RESET pins");
#endif
}

static void enter_bootloader_mode() {
  g_bridge_enabled = false;
  LT_PC_SERIAL.println("[bridge] enter bootloader: BOOT0 HIGH + reset, UART 115200 8E1");
  set_boot0(true);
  pulse_reset();
  lora_begin(LIFETRAC_L072_BOOT_BAUD, SERIAL_8E1);
  g_local_commands_enabled = false;
  g_bridge_enabled = true;
  LT_PC_SERIAL.println("[bridge] raw bootloader bridge active; local commands disabled until M7 reset/reupload");
}

static void enter_app_mode() {
  g_bridge_enabled = false;
  g_local_commands_enabled = true;
  LT_PC_SERIAL.println("[bridge] enter app: BOOT0 LOW + reset, UART 115200 8N1");
  set_boot0(false);
  pulse_reset();
  lora_begin(LIFETRAC_L072_AT_BAUD, SERIAL_8N1);
  g_bridge_enabled = true;
}

static void print_help() {
  LT_PC_SERIAL.println();
  LT_PC_SERIAL.println("LifeTrac Portenta M7 L072 Passthrough & Ping");
  LT_PC_SERIAL.print("PC side: ");
  LT_PC_SERIAL.print(LIFETRAC_PC_SERIAL_NAME);
  LT_PC_SERIAL.print(" @ ");
  LT_PC_SERIAL.println(LIFETRAC_PC_BAUD);
  LT_PC_SERIAL.println("Transparent mode: type AT commands normally; bytes bridge PC <-> L072.");
  LT_PC_SERIAL.println("Local commands, typed at start of a line:");
  LT_PC_SERIAL.println("  !help   show this help");
  LT_PC_SERIAL.println("  !ping   send AT and set LED green/red");
  LT_PC_SERIAL.println("  !app    BOOT0 low, reset, switch L072 UART to 115200 8N1");
  LT_PC_SERIAL.println("  !boot   BOOT0 high, reset, switch L072 UART to 115200 8E1 for stm32flash");
  LT_PC_SERIAL.println("  !reset  pulse L072 reset without changing BOOT0");
  LT_PC_SERIAL.println("After !boot, local commands are disabled so stm32flash bytes stay raw.");
  LT_PC_SERIAL.println();
}

static void handle_local_command(const char *cmd) {
  if (strcmp(cmd, "!help") == 0) {
    print_help();
    return;
  }

  if (strcmp(cmd, "!ping") == 0) {
    LT_PC_SERIAL.println("[bridge] AT ping...");
    set_led_unknown();
    g_at_ping_ok = at_ping_scan();
    if (g_at_ping_ok) {
      set_led_pass();
      LT_PC_SERIAL.println("[bridge] AT ping PASS");
    } else {
      set_led_fail();
      LT_PC_SERIAL.println("[bridge] AT ping FAIL");
    }
    return;
  }

  if (strcmp(cmd, "!app") == 0) {
    enter_app_mode();
    return;
  }

  if (strcmp(cmd, "!boot") == 0) {
    enter_bootloader_mode();
    return;
  }

  if (strcmp(cmd, "!reset") == 0) {
    LT_PC_SERIAL.println("[bridge] reset pulse");
    pulse_reset();
    return;
  }

  LT_PC_SERIAL.print("[bridge] unknown local command: ");
  LT_PC_SERIAL.println(cmd);
}

static bool intercept_local_command(uint8_t byte) {
  if (!g_local_commands_enabled) {
    return false;
  }

  if (g_cmd_len == 0U && byte != '!') {
    return false;
  }

  if (byte == '\r' || byte == '\n') {
    if (g_cmd_len > 0U) {
      g_cmd_buf[g_cmd_len] = '\0';
      handle_local_command(g_cmd_buf);
      g_cmd_len = 0U;
      return true;
    }
    return false;
  }

  if (g_cmd_len < (sizeof(g_cmd_buf) - 1U)) {
    g_cmd_buf[g_cmd_len++] = (char)byte;
  } else {
    g_cmd_len = 0U;
    LT_PC_SERIAL.println("[bridge] local command too long; dropped");
  }

  return true;
}

static void pump_bridge() {
  while (LT_PC_SERIAL.available() > 0) {
    const int value = LT_PC_SERIAL.read();
    if (value < 0) {
      break;
    }

    const uint8_t byte = (uint8_t)value;
    if (intercept_local_command(byte)) {
      continue;
    }

    if (g_bridge_enabled) {
      g_lora->write(byte);
    }
  }

  while (g_lora->available() > 0) {
    const int value = g_lora->read();
    if (value < 0) {
      break;
    }
    LT_PC_SERIAL.write((uint8_t)value);
  }
}

static void print_status_periodically() {
  const uint32_t now = millis();
  if ((now - g_last_status_ms) < 5000U) {
    return;
  }
  g_last_status_ms = now;

  LT_PC_SERIAL.print("[bridge] status: lora_baud=");
  LT_PC_SERIAL.print(g_lora_baud);
  LT_PC_SERIAL.print(" bridge=");
  LT_PC_SERIAL.print(g_bridge_enabled ? "on" : "off");
  LT_PC_SERIAL.print(" boot_pins=");
  LT_PC_SERIAL.print(LIFETRAC_ENABLE_BOOT_PINS ? "enabled" : "disabled");
  LT_PC_SERIAL.print(" local_commands=");
  LT_PC_SERIAL.println(g_local_commands_enabled ? "on" : "off");
}

} // namespace

void setup() {
#if defined(LEDR) && defined(LEDG) && defined(LEDB)
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
#else
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  set_led_unknown();

#if LIFETRAC_ENABLE_BOOT_PINS
  pinMode(LORA_BOOT0, OUTPUT);
  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_BOOT0, LOW);
  digitalWrite(LORA_RESET, HIGH);
#endif

  LT_PC_SERIAL.begin(LIFETRAC_PC_BAUD);
#if LIFETRAC_PC_WAIT_FOR_CONNECT
  const uint32_t wait_start = millis();
  while (!LT_PC_SERIAL && (millis() - wait_start) < 4000U) {
  }
#endif

  print_help();

  LT_PC_SERIAL.println("[bridge] starting L072 UART and running AT ping");
  g_at_ping_ok = at_ping_scan();
  if (g_at_ping_ok) {
    set_led_pass();
    LT_PC_SERIAL.println("[bridge] AT ping PASS: received OK");
  } else {
    set_led_fail();
    LT_PC_SERIAL.println("[bridge] AT ping FAIL: no OK received");
  }

  LT_PC_SERIAL.println("[bridge] transparent bridge active");
}

void loop() {
  pump_bridge();
  print_status_periodically();
}
