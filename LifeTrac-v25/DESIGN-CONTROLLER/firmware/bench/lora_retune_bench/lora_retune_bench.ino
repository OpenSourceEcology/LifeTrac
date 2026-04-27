// lora_retune_bench.ino — measure RadioLib retune cost on LifeTrac v25 radios.
//
// Flash this to MKR WAN 1310 or the tractor/base H747 during bench bring-up.
// Open Serial Monitor at 115200. The sketch prints CSV rows:
//   from,to,iterations,total_us,avg_us
//
// Choose the board pin map with one build define:
//   LIFETRAC_BOARD_MKR_WAN1310
//   LIFETRAC_BOARD_PORTENTA_MAX

#include <Arduino.h>
#include <RadioLib.h>

#if defined(LIFETRAC_BOARD_MKR_WAN1310)
SX1276 radio = new Module(6, 2, 3, 7);
#elif defined(LIFETRAC_BOARD_PORTENTA_MAX)
SX1276 radio = new Module(PD_4, PD_5, PE_4, PE_5);
#else
#error "Define LIFETRAC_BOARD_MKR_WAN1310 or LIFETRAC_BOARD_PORTENTA_MAX"
#endif

struct Profile {
  const char* name;
  float freq_mhz;
  float bw_khz;
  uint8_t sf;
  uint8_t cr_den;
};

static const Profile PROFILES[] = {
  { "control_sf7_bw125", 915.0, 125.0, 7, 5 },
  { "control_sf8_bw125", 915.0, 125.0, 8, 5 },
  { "control_sf9_bw125", 915.0, 125.0, 9, 5 },
  { "telemetry_sf9_bw250", 915.0, 250.0, 9, 8 },
  { "image_sf7_bw250", 915.0, 250.0, 7, 5 },
};

static void apply_profile(const Profile& profile) {
  radio.setFrequency(profile.freq_mhz);
  radio.setBandwidth(profile.bw_khz);
  radio.setSpreadingFactor(profile.sf);
  radio.setCodingRate(profile.cr_den);
}

static void measure_pair(const Profile& from, const Profile& to, uint16_t iterations) {
  apply_profile(from);
  delay(10);
  uint32_t start = micros();
  for (uint16_t i = 0; i < iterations; i++) {
    apply_profile(to);
    apply_profile(from);
  }
  uint32_t elapsed = micros() - start;
  uint32_t transitions = (uint32_t)iterations * 2u;
  Serial.print(from.name);
  Serial.print(',');
  Serial.print(to.name);
  Serial.print(',');
  Serial.print(transitions);
  Serial.print(',');
  Serial.print(elapsed);
  Serial.print(',');
  Serial.println(elapsed / transitions);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Serial.println("from,to,iterations,total_us,avg_us");
  int state = radio.begin(915.0, 125.0, 7, 5, 0x12, 20);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("radio.begin failed: ");
    Serial.println(state);
    while (true) delay(1000);
  }
}

void loop() {
  const uint16_t iterations = 50;
  for (uint8_t i = 0; i < sizeof(PROFILES) / sizeof(PROFILES[0]); i++) {
    for (uint8_t j = 0; j < sizeof(PROFILES) / sizeof(PROFILES[0]); j++) {
      if (i == j) continue;
      measure_pair(PROFILES[i], PROFILES[j], iterations);
    }
  }
  Serial.println("---");
  delay(5000);
}