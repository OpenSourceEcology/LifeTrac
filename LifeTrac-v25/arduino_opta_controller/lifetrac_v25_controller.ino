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
 * - 1x Proportional Flow Control Valve
 * - 1x Burkert 8605 Type 316532 Flow Valve Controller
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

// Pin for proportional flow control (4-20mA) - connects to Burkert 8605 Controller
const int FLOW_CONTROL_PIN = 2;         // O2 (4-20mA current loop output) - interfaces with Burkert controller

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

// Control mode enumeration
enum ControlMode {
  MODE_BLE,   // Bluetooth Low Energy direct control
  MODE_MQTT   // WiFi/MQTT control via broker
};

ControlMode currentMode = MODE_BLE; // Default to BLE

// BLE Service and Characteristics UUIDs
// Using custom UUIDs for LifeTrac control service
#define BLE_SERVICE_UUID        "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_JOYSTICK_LEFT_UUID  "19B10001-E8F2-537E-4F6C-D104768A1214"  // left_x, left_y
#define BLE_JOYSTICK_RIGHT_UUID "19B10002-E8F2-537E-4F6C-D104768A1214" // right_x, right_y

// BLE Objects
BLEService lifeTracService(BLE_SERVICE_UUID);
BLECharacteristic leftJoystickChar(BLE_JOYSTICK_LEFT_UUID, BLERead | BLEWrite, 8); // 2 floats = 8 bytes
BLECharacteristic rightJoystickChar(BLE_JOYSTICK_RIGHT_UUID, BLERead | BLEWrite, 8); // 2 floats = 8 bytes

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

// Safety timeout (stop all movement if no commands received)
unsigned long lastCommandTime = 0;
const unsigned long SAFETY_TIMEOUT = 1000; // 1 second

void setup() {
  Serial.begin(115200);
  Serial.println("LifeTrac v25 Controller Starting...");
  
  // Initialize mode selection switch pins with internal pulldown resistors
  // This ensures BLE mode is default when switch is not installed
  pinMode(MODE_SWITCH_PIN_A, INPUT_PULLDOWN);
  pinMode(MODE_SWITCH_PIN_B, INPUT_PULLDOWN);
  
  // Initialize digital output pins
  pinMode(LEFT_TRACK_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_TRACK_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_TRACK_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_TRACK_BACKWARD_PIN, OUTPUT);
  pinMode(ARMS_UP_PIN, OUTPUT);
  pinMode(ARMS_DOWN_PIN, OUTPUT);
  pinMode(BUCKET_UP_PIN, OUTPUT);
  pinMode(BUCKET_DOWN_PIN, OUTPUT);
  
  // Initialize 4-20mA current output pin for flow control
  OptaController.begin();  // Initialize Opta controller library
  // Configure O2 as 4-20mA current output
  OptaController.analogWriteMode(FLOW_CONTROL_PIN, CURRENT_OUTPUT_4_20MA);
  
  // Ensure all outputs are off initially
  stopAllMovement();
  
  // Wait for switch pins to stabilize before reading
  delay(1000);  // 1 second delay for hardware stabilization
  
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
      readBLEJoystickData();
      lastCommandTime = millis();
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
  payload[length] = '\0'; // Null terminate
  String message = String((char*)payload);
  
  Serial.print("Received message on ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);
  
  // Parse JSON message
  DynamicJsonDocument doc(512);
  deserializeJson(doc, message);
  
  // Update joystick data
  currentInput.left_x = doc["left_x"] | 0;
  currentInput.left_y = doc["left_y"] | 0;
  currentInput.right_x = doc["right_x"] | 0;
  currentInput.right_y = doc["right_y"] | 0;
  
  lastCommandTime = millis();
}

void processJoystickInput() {
  // Calculate track movements (tank steering)
  // Left joystick Y controls forward/backward movement
  // Left joystick X controls turning (differential steering)
  
  float baseSpeed = currentInput.left_y; // Forward/backward
  float turnRate = currentInput.left_x;  // Left/right turning
  
  // Calculate individual track speeds
  float leftTrackSpeed = baseSpeed + turnRate;
  float rightTrackSpeed = baseSpeed - turnRate;
  
  // Constrain to valid range
  leftTrackSpeed = fmaxf(-1.0, fminf(leftTrackSpeed, 1.0));
  rightTrackSpeed = fmaxf(-1.0, fminf(rightTrackSpeed, 1.0));
  
  // Control left track
  controlTrack(leftTrackSpeed, LEFT_TRACK_FORWARD_PIN, LEFT_TRACK_BACKWARD_PIN);
  
  // Control right track
  controlTrack(rightTrackSpeed, RIGHT_TRACK_FORWARD_PIN, RIGHT_TRACK_BACKWARD_PIN);
  
  // Control arms (right joystick Y)
  float armControl = currentInput.right_y;
  controlValve(armControl, ARMS_UP_PIN, ARMS_DOWN_PIN);
  
  // Control bucket (right joystick X)
  float bucketControl = currentInput.right_x;
  controlValve(bucketControl, BUCKET_UP_PIN, BUCKET_DOWN_PIN);
  
  // Set proportional flow control based on maximum input
  setFlowControl();
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

void setFlowControl() {
  // Find the maximum absolute value from all inputs
  // This determines the overall system speed
  // 4-20mA current loop output interfaces with Burkert 8605 Controller for precise flow control
  float maxInput = 0.0;
  
  maxInput = max(maxInput, abs(currentInput.left_x));
  maxInput = max(maxInput, abs(currentInput.left_y));
  maxInput = max(maxInput, abs(currentInput.right_x));
  maxInput = max(maxInput, abs(currentInput.right_y));
  
  // Convert to 4-20mA value (4mA = minimum, 20mA = maximum)
  // Map from joystick range (0.0-1.0) to current range (4-20mA)
  // Using linear interpolation: currentValue = 4 + (maxInput * 16)
  int currentValue = 4 + (int)(maxInput * 16.0);
  
  // Apply minimum flow when any movement is commanded
  if (maxInput > DEADZONE) {
    currentValue = max(currentValue, 6); // Minimum ~12.5% flow (6mA)
  } else {
    currentValue = 4; // 4mA = no flow
  }
  
  // Output true 4-20mA current using Opta A0602 current loop hardware
  // currentValue is already in mA (4-20 range)
  OptaController.analogWriteCurrent(FLOW_CONTROL_PIN, currentValue);
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
  
  // Stop flow control - set to 4mA (no flow) using true current output
  OptaController.analogWriteCurrent(FLOW_CONTROL_PIN, 4);
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
    Serial.println("Starting BLE failed!");
    Serial.println("Falling back to MQTT mode...");
    
    // Fallback to MQTT mode instead of hanging
    currentMode = MODE_MQTT;
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);
    Serial.println("Fallback to MQTT mode complete");
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
  leftJoystickChar.writeValue((uint8_t*)zeros, 8);
  rightJoystickChar.writeValue((uint8_t*)zeros, 8);
  
  // Start advertising
  BLE.advertise();
  
  Serial.println("BLE LifeTrac Control Service started");
  Serial.println("Waiting for connections...");
}

void readBLEJoystickData() {
  // Read left joystick characteristic (left_x, left_y)
  if (leftJoystickChar.written()) {
    int dataLength = leftJoystickChar.valueLength();
    
    // Validate data length before copying (should be 8 bytes for 2 floats)
    if (dataLength == 8) {
      const uint8_t* data = leftJoystickChar.value();
      float values[2];
      if (data != nullptr) {
        memcpy(values, data, 8);
        
        // Validate and clamp values to expected range (-1.0 to 1.0)
        if (values[0] < -1.0 || values[0] > 1.0) {
          Serial.print("Warning: Left X out of range (");
          Serial.print(values[0]);
          Serial.println("), clamping to [-1.0, 1.0]");
          values[0] = constrain(values[0], -1.0, 1.0);
        }
        if (values[1] < -1.0 || values[1] > 1.0) {
          Serial.print("Warning: Left Y out of range (");
          Serial.print(values[1]);
          Serial.println("), clamping to [-1.0, 1.0]");
          values[1] = constrain(values[1], -1.0, 1.0);
        }
        
        currentInput.left_x = values[0];
        currentInput.left_y = values[1];
        
        Serial.print("BLE Left: X=");
        Serial.print(currentInput.left_x);
        Serial.print(" Y=");
        Serial.println(currentInput.left_y);
      } else {
        Serial.println("Warning: leftJoystickChar.value() returned null pointer, skipping memcpy.");
      }
      if (values[0] < -1.0 || values[0] > 1.0) {
        Serial.print("Warning: Left X out of range (");
        Serial.print(values[0]);
        Serial.println("), clamping to [-1.0, 1.0]");
        values[0] = constrain(values[0], -1.0, 1.0);
      }
      if (values[1] < -1.0 || values[1] > 1.0) {
        Serial.print("Warning: Left Y out of range (");
        Serial.print(values[1]);
        Serial.println("), clamping to [-1.0, 1.0]");
        values[1] = constrain(values[1], -1.0, 1.0);
      }
      
      currentInput.left_x = values[0];
      currentInput.left_y = values[1];
      
      Serial.print("BLE Left: X=");
      Serial.print(currentInput.left_x);
      Serial.print(" Y=");
      Serial.println(currentInput.left_y);
    } else {
      Serial.print("Warning: Left joystick data length mismatch. Expected 8, got ");
      Serial.println(dataLength);
    }
  }
  
  // Read right joystick characteristic (right_x, right_y)
  if (rightJoystickChar.written()) {
    int dataLength = rightJoystickChar.valueLength();
    
    // Validate data length before copying (should be 8 bytes for 2 floats)
    if (dataLength == 8) {
      const uint8_t* data = rightJoystickChar.value();
      float values[2];
      if (data != nullptr) {
        memcpy(values, data, 8);
        
        // Validate and clamp values to expected range (-1.0 to 1.0)
        if (values[0] < -1.0 || values[0] > 1.0) {
          Serial.print("Warning: Right X out of range (");
          Serial.print(values[0]);
          Serial.println("), clamping to [-1.0, 1.0]");
          values[0] = constrain(values[0], -1.0, 1.0);
        }
        if (values[1] < -1.0 || values[1] > 1.0) {
          Serial.print("Warning: Right Y out of range (");
          Serial.print(values[1]);
          Serial.println("), clamping to [-1.0, 1.0]");
          values[1] = constrain(values[1], -1.0, 1.0);
        }
        
        currentInput.right_x = values[0];
        currentInput.right_y = values[1];
        
        Serial.print("BLE Right: X=");
        Serial.print(currentInput.right_x);
        Serial.print(" Y=");
        Serial.println(currentInput.right_y);
      } else {
        Serial.println("Warning: Right joystick data pointer is null, skipping memcpy.");
      }
        Serial.print("Warning: Right X out of range (");
        Serial.print(values[0]);
        Serial.println("), clamping to [-1.0, 1.0]");
        values[0] = constrain(values[0], -1.0, 1.0);
      }
      if (values[1] < -1.0 || values[1] > 1.0) {
        Serial.print("Warning: Right Y out of range (");
        Serial.print(values[1]);
        Serial.println("), clamping to [-1.0, 1.0]");
        values[1] = constrain(values[1], -1.0, 1.0);
      }
      
      currentInput.right_x = values[0];
      currentInput.right_y = values[1];
      
      Serial.print("BLE Right: X=");
      Serial.print(currentInput.right_x);
      Serial.print(" Y=");
      Serial.println(currentInput.right_y);
    } else {
      Serial.print("Warning: Right joystick data length mismatch. Expected 8, got ");
      Serial.println(dataLength);
    }
  }
}