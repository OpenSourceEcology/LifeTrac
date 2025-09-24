/*
 * LifeTrac v25 ESP32 Remote Control
 * 
 * This code reads joystick inputs and sends control commands via MQTT
 * to the LifeTrac v25 controller.
 * 
 * Hardware:
 * - SparkFun Thing Plus - ESP32 WROOM (USB-C)
 * - SparkFun Qwiic Joystick (2x units for dual joystick control)
 * 
 * Control scheme:
 * - Left joystick: tank steering (Y=forward/back, X=turning)
 * - Right joystick: arms (Y) and bucket (X) control
 */

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "SparkFun_Qwiic_Joystick_Arduino_Library.h"

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
const char* remote_status_topic = "lifetrac/v25/remote_status";

// Joystick objects
JOYSTICK leftJoystick;   // I2C address 0x20
JOYSTICK rightJoystick;  // I2C address 0x21

// Status LED pin
const int LED_PIN = 2;       // Built-in LED for status indication

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// Control data structure
struct ControlData {
  int left_x = 0;       // Left joystick X (-100 to 100)
  int left_y = 0;       // Left joystick Y (-100 to 100)
  int right_x = 0;      // Right joystick X (-100 to 100)
  int right_y = 0;      // Right joystick Y (-100 to 100)
};

ControlData currentControl;
ControlData lastSentControl;

// Timing variables
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 50; // Send every 50ms (20Hz)

// Status LED blink pattern
unsigned long lastBlinkTime = 0;
bool ledState = false;
int blinkPattern = 0; // 0=disconnected, 1=connected, 2=transmitting

void setup() {
  Serial.begin(115200);
  Serial.println("LifeTrac v25 Remote Control Starting...");
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize joysticks
  if (leftJoystick.begin(0x20) == false) {
    Serial.println("Left joystick not detected. Check wiring and I2C address!");
  } else {
    Serial.println("Left joystick connected!");
  }
  
  if (rightJoystick.begin(0x21) == false) {
    Serial.println("Right joystick not detected. Check wiring and I2C address!");
  } else {
    Serial.println("Right joystick connected!");
  }
  
  // Initialize button pins - removed (no buttons needed)
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Setup WiFi and MQTT
  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);
  
  Serial.println("Remote control initialized successfully!");
}

void loop() {
  // Maintain MQTT connection
  if (!client.connected()) {
    blinkPattern = 0; // Disconnected
    reconnectMQTT();
  } else {
    blinkPattern = 1; // Connected
  }
  client.loop();
  
  // Read joystick and button inputs
  readInputs();
  
  // Send control data if changed or at regular intervals
  if (shouldSendUpdate()) {
    sendControlData();
    blinkPattern = 2; // Transmitting
  }
  
  // Update status LED
  updateStatusLED();
  
  // Send remote status periodically
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 500) { // 2Hz status updates
    publishRemoteStatus();
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
    // Blink LED rapidly during connection
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // LED on steady after WiFi connection
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    String clientId = "LifeTrac_v25_Remote_";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void readInputs() {
  // Read left joystick (tank steering)
  if (leftJoystick.connected()) {
    int leftX = leftJoystick.getHorizontal();  // 0-1023
    int leftY = leftJoystick.getVertical();    // 0-1023
    
    // Convert to -100 to 100 range and invert Y for intuitive control
    currentControl.left_x = map(leftX, 0, 1023, -100, 100);
    currentControl.left_y = map(leftY, 0, 1023, 100, -100); // Inverted
    
    // Apply deadzone
    if (abs(currentControl.left_x) < 10) currentControl.left_x = 0;
    if (abs(currentControl.left_y) < 10) currentControl.left_y = 0;
  }
  
  // Read right joystick (arms and bucket)
  if (rightJoystick.connected()) {
    int rightX = rightJoystick.getHorizontal(); // 0-1023
    int rightY = rightJoystick.getVertical();   // 0-1023
    
    // Convert to -100 to 100 range and invert Y for intuitive control
    currentControl.right_x = map(rightX, 0, 1023, -100, 100);
    currentControl.right_y = map(rightY, 0, 1023, 100, -100); // Inverted
    
    // Apply deadzone
    if (abs(currentControl.right_x) < 10) currentControl.right_x = 0;
    if (abs(currentControl.right_y) < 10) currentControl.right_y = 0;
  }
}

bool shouldSendUpdate() {
  // Send if any values have changed
  bool dataChanged = (currentControl.left_x != lastSentControl.left_x ||
                     currentControl.left_y != lastSentControl.left_y ||
                     currentControl.right_x != lastSentControl.right_x ||
                     currentControl.right_y != lastSentControl.right_y);
  
  // Or send at regular intervals to maintain connection
  bool timeToSend = (millis() - lastSendTime > SEND_INTERVAL);
  
  return dataChanged || timeToSend;
}

void sendControlData() {
  if (!client.connected()) {
    return;
  }
  
  // Create JSON message
  DynamicJsonDocument doc(256);
  doc["left_x"] = currentControl.left_x;
  doc["left_y"] = currentControl.left_y;
  doc["right_x"] = currentControl.right_x;
  doc["right_y"] = currentControl.right_y;
  doc["timestamp"] = millis();
  
  String message;
  serializeJson(doc, message);
  
  // Publish the message
  if (client.publish(control_topic, message.c_str())) {
    lastSentControl = currentControl;
    lastSendTime = millis();
    
    // Debug output
    if (Serial) {
      Serial.print("Sent: ");
      Serial.println(message);
    }
  } else {
    Serial.println("Failed to publish control message");
  }
}

void publishRemoteStatus() {
  if (!client.connected()) {
    return;
  }
  
  DynamicJsonDocument doc(512);
  
  doc["timestamp"] = millis();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["battery_voltage"] = readBatteryVoltage();
  
  // Joystick connection status
  doc["left_joystick_connected"] = leftJoystick.connected();
  doc["right_joystick_connected"] = rightJoystick.connected();
  
  // Current input values (for debugging)
  doc["current_left_x"] = currentControl.left_x;
  doc["current_left_y"] = currentControl.left_y;
  doc["current_right_x"] = currentControl.right_x;
  doc["current_right_y"] = currentControl.right_y;
  
  String statusMessage;
  serializeJson(doc, statusMessage);
  
  client.publish(remote_status_topic, statusMessage.c_str());
}

float readBatteryVoltage() {
  // Read battery voltage from ADC (if battery monitoring is connected)
  // This is a placeholder - adjust based on your hardware setup
  int adcValue = analogRead(A0);
  float voltage = (adcValue / 4095.0) * 3.3 * 2; // Assuming voltage divider
  return voltage;
}

void updateStatusLED() {
  unsigned long currentTime = millis();
  unsigned long interval;
  
  switch (blinkPattern) {
    case 0: // Disconnected - fast blink
      interval = 200;
      break;
    case 1: // Connected - slow blink
      interval = 1000;
      break;
    case 2: // Transmitting - very fast blink
      interval = 100;
      // Reset to connected pattern after a short time
      if (currentTime - lastSendTime > 200) {
        blinkPattern = 1;
      }
      break;
    default:
      interval = 500;
  }
  
  if (currentTime - lastBlinkTime > interval) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastBlinkTime = currentTime;
  }
}

// Emergency stop function (can be called via serial)
void emergencyStop() {
  Serial.println("EMERGENCY STOP ACTIVATED!");
  
  // Send zero values for all controls
  currentControl.left_x = 0;
  currentControl.left_y = 0;
  currentControl.right_x = 0;
  currentControl.right_y = 0;
  
  // Send stop command immediately
  sendControlData();
  
  // Flash LED rapidly to indicate emergency stop
  for (int i = 0; i < 20; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }
}

// Serial command processing for debugging
void serialEvent() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command.equals("stop") || command.equals("emergency")) {
      emergencyStop();
    } else if (command.equals("status")) {
      Serial.println("=== LifeTrac v25 Remote Status ===");
      Serial.print("WiFi Connected: ");
      Serial.println(WiFi.status() == WL_CONNECTED ? "Yes" : "No");
      Serial.print("WiFi RSSI: ");
      Serial.println(WiFi.RSSI());
      Serial.print("MQTT Connected: ");
      Serial.println(client.connected() ? "Yes" : "No");
      Serial.print("Left Joystick: ");
      Serial.println(leftJoystick.connected() ? "Connected" : "Disconnected");
      Serial.print("Right Joystick: ");
      Serial.println(rightJoystick.connected() ? "Connected" : "Disconnected");
      Serial.print("Current Controls - LX:");
      Serial.print(currentControl.left_x);
      Serial.print(" LY:");
      Serial.print(currentControl.left_y);
      Serial.print(" RX:");
      Serial.print(currentControl.right_x);
      Serial.print(" RY:");
      Serial.println(currentControl.right_y);
    }
  }
}