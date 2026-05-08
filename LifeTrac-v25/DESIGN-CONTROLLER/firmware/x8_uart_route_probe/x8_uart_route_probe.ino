#include <Arduino.h>
#include "SEGGER_RTT.h"

#ifndef LIFETRAC_RTT_UP_BUFFER
#define LIFETRAC_RTT_UP_BUFFER 0
#endif

#ifndef LIFETRAC_ROUTE_PROBE_BAUD
#define LIFETRAC_ROUTE_PROBE_BAUD 115200UL
#endif

#ifndef LIFETRAC_UART_SNIFF_MUX_LEVEL
#define LIFETRAC_UART_SNIFF_MUX_LEVEL HIGH
#endif

#ifndef LIFETRAC_JTAG_SWD_ISO_LEVEL
#define LIFETRAC_JTAG_SWD_ISO_LEVEL LOW
#endif

static uint32_t g_seq = 0;

static void rtt_log_probe(const char *channel, uint32_t seq) {
  SEGGER_RTT_printf(
    LIFETRAC_RTT_UP_BUFFER,
    "LT_RTT_PROBE channel=%s seq=%lu ms=%lu\r\n",
    channel,
    (unsigned long)seq,
    (unsigned long)millis()
  );
}

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

#if defined(ARDUINO_PORTENTA_X8) && (SERIAL_HOWMANY < 3)
#ifndef SERIAL3_TX
#define SERIAL3_TX PJ_8
#endif
#ifndef SERIAL3_RX
#define SERIAL3_RX PJ_9
#endif
arduino::UART _UART3_(SERIAL3_TX, SERIAL3_RX);
#endif

void setup() {
  // Keep UART sniff mux deterministic while testing JTAG/SWD isolation control.
  pinMode(PA_4, OUTPUT);
  digitalWrite(PA_4, LIFETRAC_UART_SNIFF_MUX_LEVEL);

  // PC_0 controls JTAG/SWD isolation path on current schematic hypothesis.
  pinMode(PC_0, OUTPUT);
  digitalWrite(PC_0, LIFETRAC_JTAG_SWD_ISO_LEVEL);

  SEGGER_RTT_Init();
  SEGGER_RTT_ConfigUpBuffer(
    LIFETRAC_RTT_UP_BUFFER,
    "LifeTracRTT",
    NULL,
    0,
    SEGGER_RTT_MODE_NO_BLOCK_TRIM
  );

  SEGGER_RTT_printf(
    LIFETRAC_RTT_UP_BUFFER,
    "LT_RTT_PROBE boot pa4_sniff=%s pc0_jtag_swd=%s\r\n",
    (LIFETRAC_UART_SNIFF_MUX_LEVEL == HIGH) ? "HIGH" : "LOW",
    (LIFETRAC_JTAG_SWD_ISO_LEVEL == HIGH) ? "HIGH" : "LOW"
  );

  Serial1.begin(LIFETRAC_ROUTE_PROBE_BAUD);
  _UART2_.begin(LIFETRAC_ROUTE_PROBE_BAUD);
  _UART3_.begin(LIFETRAC_ROUTE_PROBE_BAUD);
}

void loop() {
  const uint32_t seq = g_seq++;

  rtt_log_probe("Serial1", seq);
  rtt_log_probe("Serial2", seq);
  rtt_log_probe("Serial3", seq);

  Serial1.println("LT_ROUTE_PROBE serial=Serial1");
  _UART2_.println("LT_ROUTE_PROBE serial=Serial2");
  _UART3_.println("LT_ROUTE_PROBE serial=Serial3");
  delay(1000);
}
