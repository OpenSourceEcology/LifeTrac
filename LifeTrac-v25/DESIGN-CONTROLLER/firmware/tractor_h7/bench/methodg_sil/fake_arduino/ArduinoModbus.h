// Records the holding-register block the sketch writes to the Opta.
#pragma once
#include "Arduino.h"

#define INPUT_REGISTERS   3
#define HOLDING_REGISTERS 4

class FakeModbusRTUClient {
  public:
    int begin(unsigned long) { return 1; }
    int beginTransmission(int id, int type, int address, int nb) {
        (void)id; (void)type; (void)address;
        pending_n = nb; pending_i = 0;
        return 1;
    }
    int write(unsigned int v) {
        if (pending_i < 16) pending[pending_i++] = (uint16_t)v;
        return 1;
    }
    int endTransmission() {
        if (fail) return 0;
        memcpy(regs, pending, sizeof(regs));
        writes++;
        return 1;
    }
    int requestFrom(int, int, int, int nb) { reads = 0; return fail ? 0 : nb; }
    long read() { return 1000 + reads++; }

    uint16_t regs[16] = {0};
    uint32_t writes = 0;
    bool fail = false;
  private:
    uint16_t pending[16] = {0};
    int pending_n = 0, pending_i = 0, reads = 0;
};
extern FakeModbusRTUClient ModbusRTUClient;
