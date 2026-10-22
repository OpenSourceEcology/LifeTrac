#pragma once
#include "Arduino.h"

class FakeRS485 {
  public:
    void setPins(int tx, int de, int re) { (void)tx; (void)de; (void)re; }
};
extern FakeRS485 RS485;
