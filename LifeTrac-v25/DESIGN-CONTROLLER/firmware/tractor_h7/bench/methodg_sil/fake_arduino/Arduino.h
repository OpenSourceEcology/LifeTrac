// Host stand-in for the parts of the Arduino core tractor_h7.ino uses.
// Only for bench/methodg_sil; never on a target build.
#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef enum {
    PA_9, PA_10, PA_15, PB_3, PD_4, PD_5, PE_4, PE_5, PF_6, PF_8, PF_9
} PinName;

uint32_t millis();
uint32_t micros();
void delay(unsigned long ms);
long random(long lo, long hi);

#define __DMB() __sync_synchronize()

class HardwareSerial {
  public:
    void begin(unsigned long b) { baud = b; opened = true; }
    void flush() {}
    int available() { return (int)(rx_len - rx_pos); }
    int read() { return rx_pos < rx_len ? rx[rx_pos++] : -1; }
    size_t write(uint8_t b) { return write(&b, 1); }
    size_t write(const uint8_t* src, size_t n) {
        if (tx_len + n > sizeof(tx)) n = sizeof(tx) - tx_len;
        memcpy(tx + tx_len, src, n);
        tx_len += n;
        return n;
    }
    template <typename T> size_t print(T) { return 0; }
    template <typename T> size_t println(T) { return 0; }
    size_t println() { return 0; }

    // Harness side.
    void inject(const uint8_t* src, size_t n) {
        if (rx_pos == rx_len) rx_pos = rx_len = 0;
        if (rx_len + n > sizeof(rx)) n = sizeof(rx) - rx_len;
        memcpy(rx + rx_len, src, n);
        rx_len += n;
    }

    unsigned long baud = 0;
    bool opened = false;
    uint8_t rx[8192];
    size_t rx_len = 0, rx_pos = 0;
    uint8_t tx[65536];
    size_t tx_len = 0;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial3;
