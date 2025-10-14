/*
 * LifeTrac v25 Arduino Opta Controller
 * 
 * This code controls the hydraulic systems of the LifeTrac v25 via MQTT commands
 * from a remote joystick controller, or via BLE direct control from DroidPad app.
 * 
 * Hardware:
 * - Arduino Opta WiFi
 * - Arduino Pro Opta Ext D1608S (Digital I/O extension)
 * - Arduino Pro Opta Ext A0602 (Analog extension)
 * - 4x Hydraulic Directional Valves (12V DC)
 * - 1-2x Proportional Flow Control Valves (configurable)
 * - 1-2x Burkert 8605 Type 316532 Flow Valve Controllers
 * - 3-position switch for OFF/MQTT/BLE mode selection
 * 
 * Control scheme:
 * - Left track: forward/backward valve
 * - Right track: forward/backward valve  
 * - Arms: up/down valve
 * - Bucket: up/down valve
 * - Proportional flow control: speed regulation
 * 
 * Mode Selection (HONEYWELL 2NT1-1 On/Off/On switch):
 * - Position 1 (MQTT): WiFi/MQTT control via broker
 * - Position 2 (OFF): No power to Opta (hardware cutoff at center position)
 * - Position 3 (BLE): Direct Bluetooth Low Energy control from DroidPad app (default)
 * - Default: BLE mode if switch is not installed (internal pulldown resistors)
 * 
 * Flow Valve Configuration (Jumper on D11):
 * - No jumper (D11=HIGH): ONE_VALVE mode - Single valve controls all hydraulics (default)
 *   Speed limited to smallest joystick input. Limited turning capability.
 * - Jumper installed (D11=LOW): TWO_VALVES mode - Independent valve control
 *   Valve 1 (O2): Controls left track + arms
 *   Valve 2 (O3): Controls right track + bucket
 *   Allows fully adjustable turning with different speeds for left/right sides.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoBLE.h>
#include <OptaController.h>  // Required for Opta A0602 4-20mA current output

// WiFi credentials - update these for your network
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT broker settings
const char* mqtt_server = "192.168.1.100";  // Raspberry Pi IP
const int mqtt_port = 1883;
const char* mqtt_user = "lifetrac";
const char* mqtt_password = "lifetrac_pass";

// MQTT topics
const char* control_topic = "lifetrac/v25/control";
const char* status_topic = "lifetrac/v25/status";

// Pin definitions for hydraulic valve control
const int LEFT_TRACK_FORWARD_PIN = 1;   // D1
const int LEFT_TRACK_BACKWARD_PIN = 2;  // D2
const int RIGHT_TRACK_FORWARD_PIN = 3;  // D3
const int RIGHT_TRACK_BACKWARD_PIN = 4; // D4
const int ARMS_UP_PIN = 5;              // D5
const int ARMS_DOWN_PIN = 6;            // D6
const int BUCKET_UP_PIN = 7;            // D7
const int BUCKET_DOWN_PIN = 8;          // D8

// Pins for proportional flow control (4-20mA) - connects to Burkert 8605 Controller(s)
const int FLOW_CONTROL_PIN_1 = 2;       // O2 (4-20mA current loop output) - Primary flow valve
const int FLOW_CONTROL_PIN_2 = 3;       // O3 (4-20mA current loop output) - Secondary flow valve (for dual valve config)

// Mode selection switch pins (HONEYWELL 2NT1-1 On/Off/On switch)
// The switch selects between MQTT and BLE modes
// Center OFF position cuts 12V power to Opta (hardware power cutoff)
const int MODE_SWITCH_PIN_A = 9;        // D9 - Mode detection pin (used)
const int MODE_SWITCH_PIN_B = 10;       // D10 - Reserved for future expansion (currently unused)
// Switch Logic (HONEYWELL 2NT1-1):
// Position 1 (MQTT): A=HIGH -> MQTT mode
// Position 2 (OFF):  No power - hardware cutoff at center position
// Position 3 (BLE):  A=LOW  -> BLE mode (default if switch not installed - internal pulldown)
// Note: Only D9 is used for mode detection. D10 is reserved for future multi-mode expansion.

// Flow valve configuration jumper pins
// These pins detect which proportional flow valve configuration is installed
const int FLOW_CONFIG_JUMPER_PIN = 11;  // D11 - Flow valve configuration jumper
// Jumper Logic (with internal pullup resistor):
// D11=HIGH (no jumper): ONE_VALVE mode - Single valve controls all hydraulics (default)
// D11=LOW (jumper to GND): TWO_VALVES mode - Valve 1 controls left track + arms, Valve 2 controls right track + bucket

// Control mode enumeration
enum ControlMode {
  MODE_BLE,   // Bluetooth Low Energy direct control
  MODE_MQTT   // WiFi/MQTT control via broker
};

ControlMode currentMode = MODE_BLE; // Default to BLE

// Flow valve configuration enumeration
enum FlowValveConfig {
  ONE_VALVE,   // Single proportional flow valve controls all hydraulics (default)
  TWO_VALVES   // Two proportional flow valves: Valve 1 for left track + arms, Valve 2 for right track + bucket
};

FlowValveConfig flowConfig = ONE_VALVE; // Default to single valve

// BLE Service and Characteristics UUIDs
// Using custom UUIDs for LifeTrac control service
#define BLE_SERVICE_UUID        "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_JOYSTICK_LEFT_UUID  "19B10001-E8F2-537E-4F6C-D104768A1214"  // left_x, left_y
#define BLE_JOYSTICK_RIGHT_UUID "19B10002-E8F2-537E-4F6C-D104768A1214" // right_x, right_y

// BLE data size constant
constexpr int JOYSTICK_DATA_SIZE = 8;  // 2 floats = 8 bytes (4 bytes per float)

// BLE Objects
BLEService lifeTracService(BLE_SERVICE_UUID);
BLECharacteristic leftJoystickChar(BLE_JOYSTICK_LEFT_UUID, BLERead | BLEWrite, JOYSTICK_DATA_SIZE);
BLECharacteristic rightJoystickChar(BLE_JOYSTICK_RIGHT_UUID, BLERead | BLEWrite, JOYSTICK_DATA_SIZE);

// Control variables
struct JoystickData {
  float left_x = 0.0;  // Left joystick X (-1.0 to 1.0)
  float left_y = 0.0;  // Left joystick Y (-1.0 to 1.0)
  float right_x = 0.0; // Right joystick X (-1.0 to 1.0)
  float right_y = 0.0; // Right joystick Y (-1.0 to 1.0)
};

JoystickData currentInput;
WiFiClient espClient;
PubSubClient client(espClient);

// Deadzone for joystick input (0.1 = 10% of range)
const float DEADZONE = 0.1;

// Flow control current constants (4-20mA current loop)
const int BASE_CURRENT = 4;          // 4mA = no flow
const int MIN_ACTIVE_CURRENT = 6;    // 6mA = minimum active flow (~12.5%)
const int CURRENT_RANGE = 16;        // 16mA range (4-20mA span)

// Safety timeout (stop all movement if no commands received)
unsigned long lastCommandTime = 0;
const unsigned long SAFETY_TIMEOUT = 1000; // 1 second

// Deceleration feature configuration
bool ENABLE_DECELERATION = true; // Can be changed to disable deceleration

// Deceleration state tracking
struct DecelerationState {
  bool isDecelerating = false;
  unsigned long decelerationStartTime = 0;
  unsigned long decelerationDuration = 0;
  float startSpeed = 0.0;
  float targetSpeed = 0.0;
};

// Separate deceleration states for each control axis
DecelerationState leftXDecel;
DecelerationState leftYDecel;
DecelerationState rightXDecel;
DecelerationState rightYDecel;

// Store previous input values to detect zero transitions
JoystickData previousInput;

void setup() {
  Serial.begin(115200);
  Serial.println("LifeTrac v25 Controller Starting...");
  
  // Initialize mode selection switch pins with internal pulldown resistors
  // This ensures BLE mode is default when switch is not installed
  pinMode(MODE_SWITCH_PIN_A, INPUT_PULLDOWN);
  pinMode(MODE_SWITCH_PIN_B, INPUT_PULLDOWN);
  
  // Initialize flow valve configuration jumper pin with internal pullup
  // This ensures ONE_VALVE mode is default when jumper is not installed
  pinMode(FLOW_CONFIG_JUMPER_PIN, INPUT_PULLUP);
  
  // Initialize digital output pins
  pinMode(LEFT_TRACK_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_TRACK_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_TRACK_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_TRACK_BACKWARD_PIN, OUTPUT);
  pinMode(ARMS_UP_PIN, OUTPUT);
  pinMode(ARMS_DOWN_PIN, OUTPUT);
  pinMode(BUCKET_UP_PIN, OUTPUT);
  pinMode(BUCKET_DOWN_PIN, OUTPUT);
  
  // Initialize 4-20mA current output pins for flow control
  OptaController.begin();  // Initialize Opta controller library
  // Configure O2 as 4-20mA current output (primary flow valve)
  OptaController.analogWriteMode(FLOW_CONTROL_PIN_1, CURRENT_OUTPUT_4_20MA);
  // Configure O3 as 4-20mA current output (secondary flow valve for dual valve config)
  OptaController.analogWriteMode(FLOW_CONTROL_PIN_2, CURRENT_OUTPUT_4_20MA);
  
  // Ensure all outputs are off initially
  stopAllMovement();
  
  // Wait for switch pins to stabilize before reading
  delay(1000);  // 1 second delay for hardware stabilization
  
  // Read flow valve configuration jumper
  readFlowValveConfig();
  
  // Read mode switch and initialize appropriate control mode
  readModeSwitch();
  
  if (currentMode == MODE_MQTT) {
    Serial.println("Mode: MQTT");
    // Setup WiFi and MQTT
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);
  } else {
    Serial.println("Mode: BLE (Default)");
    // Setup BLE
    setupBLE();
  }
  
  Serial.println("Controller initialized successfully!");
}

void loop() {
  if (currentMode == MODE_MQTT) {
    // MQTT Mode
    // Maintain MQTT connection
    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop();
    
    // Safety timeout check
    if (millis() - lastCommandTime > SAFETY_TIMEOUT) {
      stopAllMovement();
    }
    
    // Process joystick input and control hydraulics
    processJoystickInput();
    
    // Send status updates
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime > 100) { // 10Hz status updates
      publishStatus();
      lastStatusTime = millis();
    }
  } else {
    // BLE Mode
    // Poll for BLE events
    BLE.poll();
    
    // Check if characteristics have been written
    if (leftJoystickChar.written() || rightJoystickChar.written()) {
      // Only update lastCommandTime if data was successfully processed
      if (readBLEJoystickData()) {
        lastCommandTime = millis();
      }
    }
    
    // Safety timeout check
    if (millis() - lastCommandTime > SAFETY_TIMEOUT) {
      stopAllMovement();
    }
    
    // Process joystick input and control hydraulics
    processJoystickInput();
  }
  
  delay(10); // Small delay for stability
}

void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    String clientId = "LifeTrac_v25_Controller_";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(control_topic);
      Serial.print("Subscribed to: ");
      Serial.println(control_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Parse incoming MQTT message
  Serial.print("Received message on ");
  Serial.print(topic);
  Serial.print(" (");
  Serial.print(length);
  Serial.println(" bytes)");
  
  // Parse JSON message directly from payload with length (safe - no null termination required)
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Update joystick data
  currentInput.left_x = doc["left_x"] | 0;
  currentInput.left_y = doc["left_y"] | 0;
  currentInput.right_x = doc["right_x"] | 0;
  currentInput.right_y = doc["right_y"] | 0;
  
  lastCommandTime = millis();
}

// Helper function to compute track speeds from joystick input
// Centralizes tank steering math to avoid duplication
void computeTrackSpeeds(float left_y, float left_x, float* leftSpeed, float* rightSpeed) {
  float baseSpeed = left_y;  // Forward/backward
  float turnRate = left_x;   // Left/right turning
  
  // Calculate individual track speeds using differential steering
  *leftSpeed = baseSpeed + turnRate;
  *rightSpeed = baseSpeed - turnRate;
  
  // Constrain to valid range [-1.0, 1.0]
  *leftSpeed = fmaxf(-1.0, fminf(*leftSpeed, 1.0));
  *rightSpeed = fmaxf(-1.0, fminf(*rightSpeed, 1.0));
}

// Helper function to calculate deceleration duration based on speed
// For wheels: full speed = 2s, half speed = 1s, 25% speed = 0.5s
// For arms: full speed = 1s, half speed = 0.5s, 25% speed = 0.25s
unsigned long calculateDecelerationDuration(float speed, bool isArm) {
  float absSpeed = abs(speed);
  
  if (isArm) {
    // Arms operate at half the time of wheels
    if (absSpeed >= 0.75) return 1000;      // Full speed: 1 second
    if (absSpeed >= 0.375) return 500;      // Half speed: 0.5 second
    return 250;                              // 25% speed: 0.25 second
  } else {
    // Wheels
    if (absSpeed >= 0.75) return 2000;      // Full speed: 2 seconds
    if (absSpeed >= 0.375) return 1000;     // Half speed: 1 second
    return 500;                              // 25% speed: 0.5 second
  }
}

// Helper function to initiate deceleration for a control axis
void startDeceleration(DecelerationState& decel, float currentSpeed, float targetSpeed, unsigned long duration) {
  decel.isDecelerating = true;
  decel.decelerationStartTime = millis();
  decel.decelerationDuration = duration;
  decel.startSpeed = currentSpeed;
  decel.targetSpeed = targetSpeed;
}

// Helper function to calculate current deceleration value
float getDeceleratedValue(DecelerationState& decel) {
  if (!decel.isDecelerating) {
    return decel.targetSpeed;
  }
  
  unsigned long elapsed = millis() - decel.decelerationStartTime;
  
  // Check if deceleration is complete
  if (elapsed >= decel.decelerationDuration) {
    decel.isDecelerating = false;
    return decel.targetSpeed;
  }
  
  // Linear interpolation from startSpeed to targetSpeed
  float progress = (float)elapsed / (float)decel.decelerationDuration;
  float currentValue = decel.startSpeed + (decel.targetSpeed - decel.startSpeed) * progress;
  
  return currentValue;
}

// Helper function to check if we're in mixed mode (multiple inputs active simultaneously)
bool isInMixedMode() {
  int activeInputs = 0;
  
  if (abs(currentInput.left_x) > DEADZONE) activeInputs++;
  if (abs(currentInput.left_y) > DEADZONE) activeInputs++;
  if (abs(currentInput.right_x) > DEADZONE) activeInputs++;
  if (abs(currentInput.right_y) > DEADZONE) activeInputs++;
  
  return activeInputs > 1;
}

// Helper function to check if emergency stop is active
// Emergency stop is considered active when the safety timeout has been exceeded
bool isEmergencyStopActive() {
  return (millis() - lastCommandTime > SAFETY_TIMEOUT);
}

// Helper function to handle deceleration for an axis
void handleAxisDeceleration(float currentValue, float previousValue, DecelerationState& decel, bool isArm) {
  bool wasActive = abs(previousValue) > DEADZONE;
  bool isActive = abs(currentValue) > DEADZONE;
  
  // Check if input transitioned from active to zero
  if (wasActive && !isActive) {
    // Input went to zero - check if we should start deceleration
    if (ENABLE_DECELERATION && !isEmergencyStopActive() && !isInMixedMode()) {
      // Start deceleration from previous value to zero
      unsigned long duration = calculateDecelerationDuration(previousValue, isArm);
      startDeceleration(decel, previousValue, 0.0, duration);
    } else {
      // Skip deceleration - stop immediately
      decel.isDecelerating = false;
    }
  } else if (isActive) {
    // Input is active - cancel any ongoing deceleration
    decel.isDecelerating = false;
  }
}

void processJoystickInput() {
  // Handle deceleration for each axis
  handleAxisDeceleration(currentInput.left_x, previousInput.left_x, leftXDecel, false);
  handleAxisDeceleration(currentInput.left_y, previousInput.left_y, leftYDecel, false);
  handleAxisDeceleration(currentInput.right_x, previousInput.right_x, rightXDecel, true);
  handleAxisDeceleration(currentInput.right_y, previousInput.right_y, rightYDecel, true);
  
  // Apply deceleration or use current input values
  float effectiveLeftX = leftXDecel.isDecelerating ? getDeceleratedValue(leftXDecel) : currentInput.left_x;
  float effectiveLeftY = leftYDecel.isDecelerating ? getDeceleratedValue(leftYDecel) : currentInput.left_y;
  float effectiveRightX = rightXDecel.isDecelerating ? getDeceleratedValue(rightXDecel) : currentInput.right_x;
  float effectiveRightY = rightYDecel.isDecelerating ? getDeceleratedValue(rightYDecel) : currentInput.right_y;
  
  // Calculate track movements (tank steering)
  // Left joystick Y controls forward/backward movement
  // Left joystick X controls turning (differential steering)
  
  float leftTrackSpeed, rightTrackSpeed;
  computeTrackSpeeds(effectiveLeftY, effectiveLeftX, &leftTrackSpeed, &rightTrackSpeed);
  
  // Control left track
  controlTrack(leftTrackSpeed, LEFT_TRACK_FORWARD_PIN, LEFT_TRACK_BACKWARD_PIN);
  
  // Control right track
  controlTrack(rightTrackSpeed, RIGHT_TRACK_FORWARD_PIN, RIGHT_TRACK_BACKWARD_PIN);
  
  // Control arms (right joystick Y)
  float armControl = effectiveRightY;
  controlValve(armControl, ARMS_UP_PIN, ARMS_DOWN_PIN);
  
  // Control bucket (right joystick X)
  float bucketControl = effectiveRightX;
  controlValve(bucketControl, BUCKET_UP_PIN, BUCKET_DOWN_PIN);
  
  // Set proportional flow control based on effective input values
  // We need to pass the effective values to setFlowControl, so we'll use a temporary struct
  JoystickData effectiveInput;
  effectiveInput.left_x = effectiveLeftX;
  effectiveInput.left_y = effectiveLeftY;
  effectiveInput.right_x = effectiveRightX;
  effectiveInput.right_y = effectiveRightY;
  
  setFlowControl(effectiveInput);
  
  // Store current input for next iteration
  previousInput = currentInput;
}

void controlTrack(float speed, int forwardPin, int backwardPin) {
  if (abs(speed) < DEADZONE) {
    // Within deadzone - stop movement
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
  } else if (speed > 0) {
    // Forward movement
    digitalWrite(forwardPin, HIGH);
    digitalWrite(backwardPin, LOW);
  } else {
    // Backward movement
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, HIGH);
  }
}

void controlValve(float control, int upPin, int downPin) {
  if (abs(control) < DEADZONE) {
    // Within deadzone - stop movement
    digitalWrite(upPin, LOW);
    digitalWrite(downPin, LOW);
  } else if (control > 0) {
    // Up movement
    digitalWrite(upPin, HIGH);
    digitalWrite(downPin, LOW);
  } else {
    // Down movement
    digitalWrite(upPin, LOW);
    digitalWrite(downPin, HIGH);
  }
}

// Helper function to convert input magnitude to flow control current (4-20mA)
// magnitude: Input magnitude (0.0 to 1.0)
// hasInput: Whether there is active input above deadzone
// Returns: Current value in mA (4-20 range)
int flowCurrentFromInput(float magnitude, bool hasInput) {
  if (!hasInput) {
    return BASE_CURRENT; // 4mA = no flow
  }
  
  int currentValue = BASE_CURRENT + (int)(magnitude * CURRENT_RANGE);
  currentValue = max(currentValue, MIN_ACTIVE_CURRENT); // Minimum active flow
  return currentValue;
}

void setFlowControl(const JoystickData& inputData) {
  if (flowConfig == ONE_VALVE) {
    // Single valve mode: One valve controls all hydraulics
    // Speed limited to the smallest non-zero joystick input
    // Find the minimum non-zero absolute value from all inputs
    float minInput = 1.0; // Start with maximum possible value
    bool hasInput = false;
    
    // Check each input and find minimum non-zero value
    if (abs(inputData.left_x) > DEADZONE) {
      minInput = min(minInput, abs(inputData.left_x));
      hasInput = true;
    }
    if (abs(inputData.left_y) > DEADZONE) {
      minInput = min(minInput, abs(inputData.left_y));
      hasInput = true;
    }
    if (abs(inputData.right_x) > DEADZONE) {
      minInput = min(minInput, abs(inputData.right_x));
      hasInput = true;
    }
    if (abs(inputData.right_y) > DEADZONE) {
      minInput = min(minInput, abs(inputData.right_y));
      hasInput = true;
    }
    
    // Convert to 4-20mA value using helper function
    int currentValue = flowCurrentFromInput(minInput, hasInput);
    
    // Output to primary flow valve
    OptaController.analogWriteCurrent(FLOW_CONTROL_PIN_1, currentValue);
    // Disable secondary valve in single valve mode
    OptaController.analogWriteCurrent(FLOW_CONTROL_PIN_2, BASE_CURRENT);
    
  } else {
    // Dual valve mode: Two independent valves
    // Valve 1 controls: left track + arms
    // Valve 2 controls: right track + bucket
    // This allows independent speed control for each side
    
    // Calculate track speeds using shared helper function
    float leftTrackSpeed, rightTrackSpeed;
    computeTrackSpeeds(inputData.left_y, inputData.left_x, &leftTrackSpeed, &rightTrackSpeed);
    
    // Calculate flow for Valve 1 (left track + arms)
    float maxInput1 = 0.0;
    maxInput1 = max(maxInput1, abs(leftTrackSpeed));  // Left track speed
    maxInput1 = max(maxInput1, abs(inputData.right_y)); // Arms
    bool hasInput1 = maxInput1 > DEADZONE;
    int currentValue1 = flowCurrentFromInput(maxInput1, hasInput1);
    
    // Calculate flow for Valve 2 (right track + bucket)
    float maxInput2 = 0.0;
    maxInput2 = max(maxInput2, abs(rightTrackSpeed)); // Right track speed
    maxInput2 = max(maxInput2, abs(inputData.right_x)); // Bucket
    bool hasInput2 = maxInput2 > DEADZONE;
    int currentValue2 = flowCurrentFromInput(maxInput2, hasInput2);
    
    // Output to both flow valves
    OptaController.analogWriteCurrent(FLOW_CONTROL_PIN_1, currentValue1);
    OptaController.analogWriteCurrent(FLOW_CONTROL_PIN_2, currentValue2);
  }
}

void stopAllMovement() {
  // Stop all hydraulic valves
  digitalWrite(LEFT_TRACK_FORWARD_PIN, LOW);
  digitalWrite(LEFT_TRACK_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_TRACK_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_TRACK_BACKWARD_PIN, LOW);
  digitalWrite(ARMS_UP_PIN, LOW);
  digitalWrite(ARMS_DOWN_PIN, LOW);
  digitalWrite(BUCKET_UP_PIN, LOW);
  digitalWrite(BUCKET_DOWN_PIN, LOW);
  
  // Stop flow control - set to BASE_CURRENT (no flow) for both valves
  OptaController.analogWriteCurrent(FLOW_CONTROL_PIN_1, BASE_CURRENT);
  OptaController.analogWriteCurrent(FLOW_CONTROL_PIN_2, BASE_CURRENT);
  
  // Cancel all ongoing decelerations
  leftXDecel.isDecelerating = false;
  leftYDecel.isDecelerating = false;
  rightXDecel.isDecelerating = false;
  rightYDecel.isDecelerating = false;
}

void publishStatus() {
  DynamicJsonDocument doc(512);
  
  doc["timestamp"] = millis();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["last_command_age"] = millis() - lastCommandTime;
  
  // Current valve states
  doc["left_track_forward"] = digitalRead(LEFT_TRACK_FORWARD_PIN);
  doc["left_track_backward"] = digitalRead(LEFT_TRACK_BACKWARD_PIN);
  doc["right_track_forward"] = digitalRead(RIGHT_TRACK_FORWARD_PIN);
  doc["right_track_backward"] = digitalRead(RIGHT_TRACK_BACKWARD_PIN);
  doc["arms_up"] = digitalRead(ARMS_UP_PIN);
  doc["arms_down"] = digitalRead(ARMS_DOWN_PIN);
  doc["bucket_up"] = digitalRead(BUCKET_UP_PIN);
  doc["bucket_down"] = digitalRead(BUCKET_DOWN_PIN);
  
  // Current input values
  doc["input_left_x"] = currentInput.left_x;
  doc["input_left_y"] = currentInput.left_y;
  doc["input_right_x"] = currentInput.right_x;
  doc["input_right_y"] = currentInput.right_y;
  
  String statusMessage;
  serializeJson(doc, statusMessage);
  
  client.publish(status_topic, statusMessage.c_str());
}

void readFlowValveConfig() {
  // Read flow valve configuration jumper pin
  bool pinState = digitalRead(FLOW_CONFIG_JUMPER_PIN);
  
  // Determine flow valve configuration
  // With internal pullup: HIGH (no jumper) = ONE_VALVE, LOW (jumper to GND) = TWO_VALVES
  if (pinState == LOW) {
    flowConfig = TWO_VALVES;
  } else {
    flowConfig = ONE_VALVE; // Default when no jumper (pullup keeps pin HIGH)
  }
  
  Serial.print("Flow valve configuration jumper: D11=");
  Serial.print(pinState ? "HIGH" : "LOW");
  Serial.print(" -> Config: ");
  Serial.println(flowConfig == ONE_VALVE ? "ONE_VALVE (Single valve for all)" : "TWO_VALVES (Valve 1: left+arms, Valve 2: right+bucket)");
}

void readModeSwitch() {
  // Read mode switch pins
  bool pinA = digitalRead(MODE_SWITCH_PIN_A);
  bool pinB = digitalRead(MODE_SWITCH_PIN_B);  // Read for debugging, but not used in logic
  
  // Determine mode based on switch position
  // Only D9 (pinA) is used for mode detection in current HONEYWELL 2NT1-1 configuration
  // If switch is not installed, pinA will be LOW (pulldown) -> BLE mode (default)
  if (pinA == HIGH) {
    currentMode = MODE_MQTT;
  } else {
    currentMode = MODE_BLE; // Default when pinA is LOW
  }
  
  Serial.print("Mode switch: A=");
  Serial.print(pinA);
  Serial.print(" B=");
  Serial.print(pinB);  // Display for debugging purposes
  Serial.print(" -> Mode: ");
  Serial.println(currentMode == MODE_MQTT ? "MQTT" : "BLE");
}

void setupBLE() {
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("ERROR: Starting BLE failed!");
    Serial.println("BLE hardware initialization error. Entering safe state.");
    Serial.println("All hydraulic outputs will remain disabled.");
    Serial.println("Please check BLE hardware or switch to MQTT mode using the mode switch.");
    // Stay in BLE mode but without functional BLE - safety timeout will keep outputs off
    return;
  }
  
  // Set BLE device name
  BLE.setLocalName("LifeTrac-v25");
  BLE.setDeviceName("LifeTrac-v25");
  
  // Set advertised service
  BLE.setAdvertisedService(lifeTracService);
  
  // Add characteristics to service
  lifeTracService.addCharacteristic(leftJoystickChar);
  lifeTracService.addCharacteristic(rightJoystickChar);
  
  // Add service
  BLE.addService(lifeTracService);
  
  // Initialize characteristic values to zero
  float zeros[2] = {0.0, 0.0};
  leftJoystickChar.writeValue((uint8_t*)zeros, JOYSTICK_DATA_SIZE);
  rightJoystickChar.writeValue((uint8_t*)zeros, JOYSTICK_DATA_SIZE);
  
  // Start advertising
  BLE.advertise();
  
  Serial.println("BLE LifeTrac Control Service started");
  Serial.println("Waiting for connections...");
}

// Enum and array to hold last warning times for joystick axes
enum JoystickAxis {
  LEFT_X = 0,
  LEFT_Y = 1,
  RIGHT_X = 2,
  RIGHT_Y = 3,
  JOYSTICK_AXIS_COUNT
};

const char* JoystickAxisNames[JOYSTICK_AXIS_COUNT] = {
  "Left X",
  "Left Y",
  "Right X",
  "Right Y"
};

// Helper function to validate and clamp joystick axis values
void validateAndClampJoystickValue(float& value, JoystickAxis axis) {
  static unsigned long lastWarningTimes[JOYSTICK_AXIS_COUNT] = {0, 0, 0, 0};
  unsigned long now = millis();
  unsigned long* lastWarningTime = &lastWarningTimes[axis];

  if (value < -1.0 || value > 1.0) {
    // Only print warning if at least 1000ms have passed since last warning for this axis
    if (now - *lastWarningTime >= 1000) {
      Serial.print("Warning: ");
      Serial.print(JoystickAxisNames[axis]);
      Serial.print(" out of range (");
      Serial.print(value);
      Serial.println("), clamping to [-1.0, 1.0]");
      *lastWarningTime = now;
    }
    value = constrain(value, -1.0, 1.0);
  }
}

bool readBLEJoystickData() {
  bool dataProcessed = false;
  
  // Read left joystick characteristic (left_x, left_y)
  if (leftJoystickChar.written()) {
    int dataLength = leftJoystickChar.valueLength();
    
    // Validate data length before copying (should be JOYSTICK_DATA_SIZE bytes for 2 floats)
    if (dataLength == JOYSTICK_DATA_SIZE) {
      const uint8_t* data = leftJoystickChar.value();
      
      // Validate data pointer is not null before memcpy
      if (data != NULL) {
        float values[2];
        memcpy(values, data, JOYSTICK_DATA_SIZE);
        
        // Validate and clamp values to expected range (-1.0 to 1.0)
        validateAndClampJoystickValue(values[0], LEFT_X);
        validateAndClampJoystickValue(values[1], LEFT_Y);
        
        currentInput.left_x = values[0];
        currentInput.left_y = values[1];
        
        Serial.print("BLE Left: X=");
        Serial.print(currentInput.left_x);
        Serial.print(" Y=");
        Serial.println(currentInput.left_y);
        
        dataProcessed = true;
      } else {
        Serial.println("Warning: Left joystick data pointer is null");
      }
    } else {
      Serial.print("Warning: Left joystick data length mismatch. Expected ");
      Serial.print(JOYSTICK_DATA_SIZE);
      Serial.print(", got ");
      Serial.println(dataLength);
    }
  }
  
  // Read right joystick characteristic (right_x, right_y)
  if (rightJoystickChar.written()) {
    int dataLength = rightJoystickChar.valueLength();
    
    // Validate data length before copying (should be JOYSTICK_DATA_SIZE bytes for 2 floats)
    if (dataLength == JOYSTICK_DATA_SIZE) {
      const uint8_t* data = rightJoystickChar.value();
      
      // Validate data pointer is not null before memcpy
      if (data != NULL) {
        float values[2];
        memcpy(values, data, JOYSTICK_DATA_SIZE);
        
        // Validate and clamp values to expected range (-1.0 to 1.0)
        validateAndClampJoystickValue(values[0], RIGHT_X);
        validateAndClampJoystickValue(values[1], RIGHT_Y);
        
        currentInput.right_x = values[0];
        currentInput.right_y = values[1];
        
        Serial.print("BLE Right: X=");
        Serial.print(currentInput.right_x);
        Serial.print(" Y=");
        Serial.println(currentInput.right_y);
        
        dataProcessed = true;
      } else {
        Serial.println("Warning: Right joystick data pointer is null");
      }
    } else {
      Serial.print("Warning: Right joystick data length mismatch. Expected ");
      Serial.print(JOYSTICK_DATA_SIZE);
      Serial.print(", got ");
      Serial.println(dataLength);
    }
  }
  
  return dataProcessed;
}