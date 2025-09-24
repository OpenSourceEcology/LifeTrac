/*
 * LifeTrac v25 Arduino Opta Controller
 * 
 * This code controls the hydraulic systems of the LifeTrac v25 via MQTT commands
 * from a remote joystick controller.
 * 
 * Hardware:
 * - Arduino Opta WiFi
 * - Arduino Pro Opta Ext D1608S (Digital I/O extension)
 * - Arduino Pro Opta Ext A0602 (Analog extension)
 * - 4x Hydraulic Directional Valves (12V DC)
 * - 1x Proportional Flow Control Valve
 * 
 * Control scheme:
 * - Left track: forward/backward valve
 * - Right track: forward/backward valve  
 * - Arms: up/down valve
 * - Bucket: up/down valve
 * - Proportional flow control: speed regulation
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

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
const int LEFT_TRACK_FORWARD_PIN = 0;   // D0
const int LEFT_TRACK_BACKWARD_PIN = 1;  // D1
const int RIGHT_TRACK_FORWARD_PIN = 2;  // D2
const int RIGHT_TRACK_BACKWARD_PIN = 3; // D3
const int ARMS_UP_PIN = 4;              // D4
const int ARMS_DOWN_PIN = 5;            // D5
const int BUCKET_UP_PIN = 6;            // D6
const int BUCKET_DOWN_PIN = 7;          // D7

// Pin for proportional flow control (PWM)
const int FLOW_CONTROL_PIN = 0;         // A0 (PWM capable)

// Control variables
struct JoystickData {
  int left_x = 0;      // Left joystick X (-100 to 100)
  int left_y = 0;      // Left joystick Y (-100 to 100)
  int right_x = 0;     // Right joystick X (-100 to 100)
  int right_y = 0;     // Right joystick Y (-100 to 100)
  bool button1 = false; // Arms control
  bool button2 = false; // Bucket control
};

JoystickData currentInput;
WiFiClient espClient;
PubSubClient client(espClient);

// Deadzone for joystick input
const int DEADZONE = 10;

// Safety timeout (stop all movement if no commands received)
unsigned long lastCommandTime = 0;
const unsigned long SAFETY_TIMEOUT = 1000; // 1 second

void setup() {
  Serial.begin(115200);
  Serial.println("LifeTrac v25 Controller Starting...");
  
  // Initialize digital output pins
  pinMode(LEFT_TRACK_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_TRACK_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_TRACK_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_TRACK_BACKWARD_PIN, OUTPUT);
  pinMode(ARMS_UP_PIN, OUTPUT);
  pinMode(ARMS_DOWN_PIN, OUTPUT);
  pinMode(BUCKET_UP_PIN, OUTPUT);
  pinMode(BUCKET_DOWN_PIN, OUTPUT);
  
  // Initialize PWM pin for flow control
  pinMode(FLOW_CONTROL_PIN, OUTPUT);
  
  // Ensure all outputs are off initially
  stopAllMovement();
  
  // Setup WiFi and MQTT
  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  
  Serial.println("Controller initialized successfully!");
}

void loop() {
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
  currentInput.button1 = doc["button1"] | false;
  currentInput.button2 = doc["button2"] | false;
  
  lastCommandTime = millis();
}

void processJoystickInput() {
  // Calculate track movements (tank steering)
  // Left joystick Y controls forward/backward movement
  // Left joystick X controls turning (differential steering)
  
  int baseSpeed = currentInput.left_y; // Forward/backward
  int turnRate = currentInput.left_x;  // Left/right turning
  
  // Calculate individual track speeds
  int leftTrackSpeed = baseSpeed + turnRate;
  int rightTrackSpeed = baseSpeed - turnRate;
  
  // Constrain to valid range
  leftTrackSpeed = constrain(leftTrackSpeed, -100, 100);
  rightTrackSpeed = constrain(rightTrackSpeed, -100, 100);
  
  // Control left track
  controlTrack(leftTrackSpeed, LEFT_TRACK_FORWARD_PIN, LEFT_TRACK_BACKWARD_PIN);
  
  // Control right track
  controlTrack(rightTrackSpeed, RIGHT_TRACK_FORWARD_PIN, RIGHT_TRACK_BACKWARD_PIN);
  
  // Control arms (right joystick Y or button1)
  int armControl = currentInput.right_y;
  if (currentInput.button1) {
    armControl = 100; // Full up when button pressed
  }
  controlValve(armControl, ARMS_UP_PIN, ARMS_DOWN_PIN);
  
  // Control bucket (button2 or right joystick X)
  int bucketControl = currentInput.right_x;
  if (currentInput.button2) {
    bucketControl = 100; // Full up when button pressed
  }
  controlValve(bucketControl, BUCKET_UP_PIN, BUCKET_DOWN_PIN);
  
  // Set proportional flow control based on maximum input
  setFlowControl();
}

void controlTrack(int speed, int forwardPin, int backwardPin) {
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

void controlValve(int control, int upPin, int downPin) {
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
  int maxInput = 0;
  
  maxInput = max(maxInput, abs(currentInput.left_x));
  maxInput = max(maxInput, abs(currentInput.left_y));
  maxInput = max(maxInput, abs(currentInput.right_x));
  maxInput = max(maxInput, abs(currentInput.right_y));
  
  // Convert to PWM value (0-255)
  int pwmValue = map(maxInput, 0, 100, 0, 255);
  
  // Apply minimum flow when any movement is commanded
  if (maxInput > DEADZONE) {
    pwmValue = max(pwmValue, 50); // Minimum 20% flow
  }
  
  analogWrite(FLOW_CONTROL_PIN, pwmValue);
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
  
  // Stop flow control
  analogWrite(FLOW_CONTROL_PIN, 0);
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
  doc["input_button1"] = currentInput.button1;
  doc["input_button2"] = currentInput.button2;
  
  String statusMessage;
  serializeJson(doc, statusMessage);
  
  client.publish(status_topic, statusMessage.c_str());
}