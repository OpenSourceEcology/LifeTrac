static HardwareSerial &kProbeSerial = Serial1;
static const uint32_t kProbeBaud = 115200U;
static uint32_t g_seq = 0U;

void setup() {
  kProbeSerial.begin(kProbeBaud);
}

void loop() {
  kProbeSerial.print("LT_ROUTE_PROBE serial=Serial1 baud=");
  kProbeSerial.print(kProbeBaud);
  kProbeSerial.print(" seq=");
  kProbeSerial.print(g_seq++);
  kProbeSerial.print("\r\n");
  delay(500);
}
