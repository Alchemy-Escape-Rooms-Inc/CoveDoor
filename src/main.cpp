/*
 * ============================================
 * ALCHEMY ESCAPE ROOM - COVE SLIDING DOOR CONTROLLER
 * ESP32 VERSION WITH BTS7960 MOTOR DRIVER
 * ============================================
 *
 * Version: 1.0.0
 * Date: 2026-02-10
 *
 * Automatic Sliding Door System
 * ESP32-S3 + BTS7960 Dual H-Bridge Motor Driver
 *
 * WATCHTOWER PROTOCOL COMMANDS (respond on /command topic):
 *   PING         -> responds with "PONG"
 *   STATUS       -> responds with current door state
 *   RESET        -> responds with "OK" then restarts
 *   PUZZLE_RESET -> responds with "OK" and resets door state
 *
 * DOOR COMMANDS (status updates on /status topic):
 *   OPEN   -> opens the door
 *   CLOSE  -> closes the door
 *   STOP   -> emergency stop
 *
 * MQTT TOPICS:
 *   Subscribe: MermaidsTale/CoveDoor/command
 *   Publish:   MermaidsTale/CoveDoor/status   (state changes, heartbeat)
 *   Publish:   MermaidsTale/CoveDoor/log      (mirrored serial output)
 *   Publish:   MermaidsTale/CoveDoor/limit    (limit switch events)
 *
 * LIMIT SWITCH MESSAGES (on /limit topic):
 *   LIMIT_OPEN_HIT      -> Door reached fully open position
 *   LIMIT_CLOSED_HIT    -> Door reached fully closed position
 *   LIMIT_OPEN_CLEAR    -> Door moved away from open limit
 *   LIMIT_CLOSED_CLEAR  -> Door moved away from closed limit
 *
 * BTS7960 WIRING:
 *   RPWM -> GPIO4 (PWM for opening/forward)
 *   LPWM -> GPIO5 (PWM for closing/reverse)
 *   R_EN -> 3.3V (always enabled)
 *   L_EN -> 3.3V (always enabled)
 *   VCC  -> 5V
 *   GND  -> GND
 *
 * ============================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ============================================
// DEVICE CONFIGURATION — Sourced from MANIFEST.h
// ============================================
#include "MANIFEST.h"

// Bridge: all code below still uses these names, but values come from manifest
#define DEVICE_NAME       manifest::DEVICE_NAME
#define FIRMWARE_VERSION  manifest::FIRMWARE_VERSION

const char* WIFI_SSID     = manifest::WIFI_SSID;
const char* WIFI_PASSWORD = manifest::WIFI_PASSWORD;

const char* MQTT_SERVER   = manifest::MQTT_SERVER;
const int   MQTT_PORT     = manifest::MQTT_PORT;

// ============================================
// PIN DEFINITIONS — Sourced from MANIFEST.h
// ============================================
#define RPWM_PIN        manifest::RPWM_PIN
#define LPWM_PIN        manifest::LPWM_PIN
#define LIMIT_OPEN      manifest::LIMIT_OPEN
#define LIMIT_CLOSED    manifest::LIMIT_CLOSED

// Motor Configuration — Sourced from MANIFEST.h
#define MOTOR_SPEED     manifest::MOTOR_SPEED

// PWM Configuration — Sourced from MANIFEST.h
#define PWM_CHANNEL_R   manifest::PWM_CHANNEL_R
#define PWM_CHANNEL_L   manifest::PWM_CHANNEL_L
#define PWM_FREQ        manifest::PWM_FREQ
#define PWM_RESOLUTION  manifest::PWM_RESOLUTION

// Debounce — Sourced from MANIFEST.h
#define LIMIT_DEBOUNCE_MS manifest::LIMIT_DEBOUNCE_MS

// Door timing — Sourced from MANIFEST.h
#define DOOR_RAMP_UP_MS     manifest::DOOR_RAMP_UP_MS
#define DOOR_FULL_SPEED_MS  manifest::DOOR_FULL_SPEED_MS
#define DOOR_RAMP_DOWN_MS   manifest::DOOR_RAMP_DOWN_MS
#define DOOR_TOTAL_TIME_MS  manifest::DOOR_TOTAL_TIME_MS

// ============================================
// MQTT TOPICS
// ============================================
String mqtt_topic_command;
String mqtt_topic_status;
String mqtt_topic_log;
String mqtt_topic_limit;

// ============================================
// DOOR STATES
// ============================================
enum DoorState {
  DOOR_CLOSED,
  DOOR_OPEN,
  DOOR_OPENING,
  DOOR_CLOSING,
  DOOR_STOPPED,
  EMERGENCY_STOP
};

DoorState currentState = DOOR_STOPPED;
DoorState previousState = DOOR_STOPPED;

// ============================================
// LIMIT SWITCH DEBOUNCING
// ============================================
bool rawLimitOpen = false;
bool rawLimitClosed = false;
bool debouncedLimitOpen = false;
bool debouncedLimitClosed = false;
unsigned long limitOpenStableTime = 0;
unsigned long limitClosedStableTime = 0;
bool lastRawLimitOpen = false;
bool lastRawLimitClosed = false;

// ============================================
// GLOBAL VARIABLES
// ============================================
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastHeartbeat = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastMqttReconnect = 0;
unsigned long bootTime = 0;
unsigned long motorStartTime = 0;

const unsigned long HEARTBEAT_INTERVAL = manifest::HEARTBEAT_INTERVAL;
const unsigned long WIFI_CHECK_INTERVAL = manifest::WIFI_CHECK_INTERVAL;
const unsigned long MQTT_RECONNECT_INTERVAL = manifest::MQTT_RECONNECT_INTERVAL;
bool systemReady = false;

// Buffer for MQTT log messages
char mqttLogBuffer[256];

// ============================================
// FUNCTION PROTOTYPES
// ============================================
void setup_wifi();
void setup_mqtt();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void mqtt_reconnect();
void send_heartbeat();
void send_status(const char* status);
void check_connections();
void mqttLog(const char* message);
void mqttLogf(const char* format, ...);

void startOpening();
void startClosing();
void stopMotor();
void setMotorSpeed(int openSpeed, int closeSpeed);
void checkLimitSwitches();
const char* getStateString(DoorState state);
void publishLimitEvent(const char* event);

// ============================================
// MQTT LOGGING
// ============================================
void mqttLog(const char* message) {
  Serial.println(message);
  if (mqtt.connected()) {
    mqtt.publish(mqtt_topic_log.c_str(), message, false);
  }
}

void mqttLogf(const char* format, ...) {
  va_list args;
  va_start(args, format);
  vsnprintf(mqttLogBuffer, sizeof(mqttLogBuffer), format, args);
  va_end(args);
  mqttLog(mqttLogBuffer);
}

// ============================================
// LIMIT SWITCH EVENT PUBLISHER
// ============================================
void publishLimitEvent(const char* event) {
  mqttLogf("[LIMIT] %s", event);
  if (mqtt.connected()) {
    mqtt.publish(mqtt_topic_limit.c_str(), event, false);
  }
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n");
  Serial.println("============================================");
  Serial.println("   COVE SLIDING DOOR CONTROLLER - ESP32");
  Serial.println("   BTS7960 Dual H-Bridge Motor Driver");
  Serial.println("============================================");
  Serial.print("Device Name: ");
  Serial.println(DEVICE_NAME);
  Serial.print("Firmware:    ");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("Compiled:    ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  Serial.println("============================================\n");
  Serial.println("Watchtower Protocol");

  // Build MQTT topics
  mqtt_topic_command = "MermaidsTale/" + String(DEVICE_NAME) + "/command";
  mqtt_topic_status = "MermaidsTale/" + String(DEVICE_NAME) + "/status";
  mqtt_topic_log = "MermaidsTale/" + String(DEVICE_NAME) + "/log";
  mqtt_topic_limit = "MermaidsTale/" + String(DEVICE_NAME) + "/limit";

  Serial.print("[MQTT] Command topic: ");
  Serial.println(mqtt_topic_command);
  Serial.print("[MQTT] Status topic:  ");
  Serial.println(mqtt_topic_status);
  Serial.print("[MQTT] Log topic:     ");
  Serial.println(mqtt_topic_log);
  Serial.print("[MQTT] Limit topic:   ");
  Serial.println(mqtt_topic_limit);
  Serial.println();

  // Initialize BTS7960 motor control pins with PWM
  ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RPWM_PIN, PWM_CHANNEL_R);
  ledcAttachPin(LPWM_PIN, PWM_CHANNEL_L);
  ledcWrite(PWM_CHANNEL_R, 0);
  ledcWrite(PWM_CHANNEL_L, 0);
  Serial.println("[INIT] BTS7960 motor pins configured (RPWM + LPWM with PWM)");
  Serial.print("       RPWM: GPIO");
  Serial.print(RPWM_PIN);
  Serial.print(", LPWM: GPIO");
  Serial.println(LPWM_PIN);

  // Initialize limit switch pins with internal pullups
  pinMode(LIMIT_OPEN, INPUT_PULLUP);
  pinMode(LIMIT_CLOSED, INPUT_PULLUP);
  Serial.println("[INIT] Limit switches configured with INPUT_PULLUP");
  Serial.print("       LIMIT_OPEN: GPIO");
  Serial.print(LIMIT_OPEN);
  Serial.print(", LIMIT_CLOSED: GPIO");
  Serial.println(LIMIT_CLOSED);

  // Stop motor initially
  stopMotor();

  // Read initial limit switch states (LOW = triggered with pullup)
  rawLimitOpen = (digitalRead(LIMIT_OPEN) == LOW);
  rawLimitClosed = (digitalRead(LIMIT_CLOSED) == LOW);
  debouncedLimitOpen = rawLimitOpen;
  debouncedLimitClosed = rawLimitClosed;
  lastRawLimitOpen = rawLimitOpen;
  lastRawLimitClosed = rawLimitClosed;
  limitOpenStableTime = millis();
  limitClosedStableTime = millis();

  Serial.print("[INIT] LIMIT_OPEN: ");
  Serial.println(rawLimitOpen ? "TRIGGERED" : "CLEAR");
  Serial.print("[INIT] LIMIT_CLOSED: ");
  Serial.println(rawLimitClosed ? "TRIGGERED" : "CLEAR");

  // Set initial state based on limit switches
  if (debouncedLimitOpen && debouncedLimitClosed) {
    Serial.println("[WARNING] BOTH limit switches triggered - check wiring!");
    currentState = DOOR_STOPPED;
  } else if (debouncedLimitClosed) {
    currentState = DOOR_CLOSED;
    Serial.println("[INIT] Door is CLOSED (limit switch active)");
  } else if (debouncedLimitOpen) {
    currentState = DOOR_OPEN;
    Serial.println("[INIT] Door is OPEN (limit switch active)");
  } else {
    currentState = DOOR_STOPPED;
    Serial.println("[INIT] Door position UNKNOWN (no limit active)");
  }
  previousState = currentState;

  // Connect to WiFi
  setup_wifi();

  // Setup MQTT
  setup_mqtt();

  // Record boot time
  bootTime = millis();

  // Mark system ready
  systemReady = true;

  // Send initial status via MQTT
  if (mqtt.connected()) {
    send_status("ONLINE");
    mqttLogf("[READY] System initialized - State: %s", getStateString(currentState));
  }

  Serial.println("\n============================================");
  Serial.println("   DEVICE READY");
  Serial.println("============================================\n");
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // Check WiFi and MQTT connections
  check_connections();

  // Process MQTT messages
  if (mqtt.connected()) {
    mqtt.loop();
  }

  // Send periodic heartbeat
  send_heartbeat();

  // Check limit switches with debouncing
  checkLimitSwitches();

  // Handle motor ramping for opening and closing
  if (currentState == DOOR_OPENING || currentState == DOOR_CLOSING) {
    unsigned long elapsed = millis() - motorStartTime;
    int speed = 0;

    if (elapsed < DOOR_RAMP_UP_MS) {
      // Ramping up
      speed = map(elapsed, 0, DOOR_RAMP_UP_MS, 0, MOTOR_SPEED);
    } else if (elapsed < (DOOR_TOTAL_TIME_MS - DOOR_RAMP_DOWN_MS)) {
      // Full speed
      speed = MOTOR_SPEED;
    } else if (elapsed < DOOR_TOTAL_TIME_MS) {
      // Ramping down
      unsigned long rampDownElapsed = elapsed - (DOOR_TOTAL_TIME_MS - DOOR_RAMP_DOWN_MS);
      speed = map(rampDownElapsed, 0, DOOR_RAMP_DOWN_MS, MOTOR_SPEED, 0);
    } else {
      // Done - timeout reached without hitting limit
      speed = 0;
      stopMotor();
      if (currentState == DOOR_OPENING) {
        mqttLog("[TIMEOUT] Door open timeout - no limit hit");
        currentState = DOOR_OPEN;
        send_status("OPEN");
      } else {
        mqttLog("[TIMEOUT] Door close timeout - no limit hit");
        currentState = DOOR_CLOSED;
        send_status("CLOSED");
      }
    }

    // Apply speed if still moving
    if (currentState == DOOR_OPENING || currentState == DOOR_CLOSING) {
      if (currentState == DOOR_OPENING) {
        setMotorSpeed(speed, 0);  // RPWM for opening
      } else {
        setMotorSpeed(0, speed);  // LPWM for closing
      }
    }
  }

  // Publish state changes
  if (currentState != previousState) {
    mqttLogf("[STATE] %s -> %s", getStateString(previousState), getStateString(currentState));
    previousState = currentState;

    if (mqtt.connected()) {
      send_status(getStateString(currentState));
    }
  }

  delay(10);
}

// ============================================
// WIFI FUNCTIONS
// ============================================
void setup_wifi() {
  Serial.print("[WIFI] Connecting to ");
  Serial.print(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" Connected!");
    Serial.print("[WIFI] IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WIFI] Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println(" FAILED!");
    Serial.println("[WIFI] Will retry in background...");
  }
}

void check_connections() {
  if (millis() - lastWifiCheck >= WIFI_CHECK_INTERVAL) {
    lastWifiCheck = millis();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WIFI] Disconnected - reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }

  if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) {
    mqtt_reconnect();
  }
}

// ============================================
// MQTT FUNCTIONS
// ============================================
void setup_mqtt() {
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqtt_callback);
  mqtt.setBufferSize(512);

  if (WiFi.status() == WL_CONNECTED) {
    mqtt_reconnect();
  }
}

void mqtt_reconnect() {
  if (millis() - lastMqttReconnect < MQTT_RECONNECT_INTERVAL) {
    return;
  }
  lastMqttReconnect = millis();

  Serial.print("[MQTT] Connecting to ");
  Serial.print(MQTT_SERVER);
  Serial.print(":");
  Serial.print(MQTT_PORT);
  Serial.print("...");

  if (mqtt.connect(DEVICE_NAME)) {
    Serial.println(" Connected!");

    mqtt.subscribe(mqtt_topic_command.c_str());
    Serial.print("[MQTT] Subscribed to: ");
    Serial.println(mqtt_topic_command);

    send_status("ONLINE");
    mqttLogf("[MQTT] Connected - Current state: %s", getStateString(currentState));

  } else {
    Serial.print(" Failed (rc=");
    Serial.print(mqtt.state());
    Serial.println(")");
  }
}

// ============================================
// MQTT CALLBACK - WATCHTOWER COMPLIANT
// ============================================
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // Copy topic to local buffer FIRST to prevent stack corruption
  char topicBuf[128];
  strncpy(topicBuf, topic, sizeof(topicBuf) - 1);
  topicBuf[sizeof(topicBuf) - 1] = '\0';

  // Now safe to declare other variables
  char message[128];
  if (length >= sizeof(message)) {
    length = sizeof(message) - 1;
  }
  memcpy(message, payload, length);
  message[length] = '\0';

  // Trim whitespace and convert to uppercase
  char* msg = message;
  while (*msg == ' ' || *msg == '\t' || *msg == '\r' || *msg == '\n') msg++;
  char* end = msg + strlen(msg) - 1;
  while (end > msg && (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')) {
    *end = '\0';
    end--;
  }

  // Convert to uppercase
  for (char* p = msg; *p; p++) {
    *p = toupper(*p);
  }

  mqttLogf("[MQTT] Received on %s: %s", topicBuf, msg);

  // Only process commands on our command topic
  if (strcmp(topicBuf, mqtt_topic_command.c_str()) != 0) {
    return;
  }

  // ============================================================
  // REQUIRED COMMANDS - Watchtower Protocol
  // ============================================================

  // PING - Health check for System Checker
  if (strcmp(msg, "PING") == 0) {
    mqtt.publish(mqtt_topic_command.c_str(), "PONG");
    mqttLog("[CMD] PING -> PONG");
    return;
  }

  // STATUS - Report current state
  if (strcmp(msg, "STATUS") == 0) {
    unsigned long uptime = (millis() - bootTime) / 1000;
    char statusBuf[128];
    snprintf(statusBuf, sizeof(statusBuf), "%s|UP:%lus|RSSI:%d|VER:%s|LIMIT_OPEN:%s|LIMIT_CLOSED:%s",
      getStateString(currentState),
      uptime,
      WiFi.RSSI(),
      FIRMWARE_VERSION,
      debouncedLimitOpen ? "ACTIVE" : "CLEAR",
      debouncedLimitClosed ? "ACTIVE" : "CLEAR");

    mqtt.publish(mqtt_topic_command.c_str(), statusBuf);
    mqttLogf("[CMD] STATUS -> %s", statusBuf);
    return;
  }

  // RESET - Reboot the device
  if (strcmp(msg, "RESET") == 0 || strcmp(msg, "REBOOT") == 0 || strcmp(msg, "RESTART") == 0) {
    mqtt.publish(mqtt_topic_command.c_str(), "OK");
    mqttLog("[CMD] RESET -> Rebooting...");
    stopMotor();
    delay(100);
    ESP.restart();
    return;
  }

  // PUZZLE_RESET - Reset door state without rebooting
  if (strcmp(msg, "PUZZLE_RESET") == 0) {
    stopMotor();
    // Reset to current physical state based on limit switches
    if (debouncedLimitClosed) {
      currentState = DOOR_CLOSED;
    } else if (debouncedLimitOpen) {
      currentState = DOOR_OPEN;
    } else {
      currentState = DOOR_STOPPED;
    }
    mqtt.publish(mqtt_topic_command.c_str(), "OK");
    mqttLogf("[CMD] PUZZLE_RESET -> OK (State: %s)", getStateString(currentState));
    return;
  }

  // ============================================================
  // PROP-SPECIFIC COMMANDS - Door Control
  // ============================================================

  if (strcmp(msg, "OPEN") == 0) {
    if (currentState == DOOR_OPEN) {
      mqttLog("[CMD] Door already OPEN");
      send_status("ALREADY_OPEN");
    } else if (currentState == DOOR_OPENING) {
      mqttLog("[CMD] Door already OPENING");
    } else {
      mqttLog("[CMD] OPEN command via MQTT");
      startOpening();
    }
    return;
  }

  if (strcmp(msg, "CLOSE") == 0) {
    if (currentState == DOOR_CLOSED) {
      mqttLog("[CMD] Door already CLOSED");
      send_status("ALREADY_CLOSED");
    } else if (currentState == DOOR_CLOSING) {
      mqttLog("[CMD] Door already CLOSING");
    } else {
      mqttLog("[CMD] CLOSE command via MQTT");
      startClosing();
    }
    return;
  }

  if (strcmp(msg, "STOP") == 0) {
    mqttLog("[CMD] STOP command via MQTT");
    stopMotor();
    currentState = DOOR_STOPPED;
    send_status("STOPPED");
    return;
  }

  mqttLogf("[CMD] Unknown command: %s", msg);
}

void send_status(const char* status) {
  if (!mqtt.connected()) return;
  mqtt.publish(mqtt_topic_status.c_str(), status, false);
}

void send_heartbeat() {
  if (!mqtt.connected()) return;

  if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = millis();

    unsigned long uptime = (millis() - bootTime) / 1000;
    String heartbeat = "HEARTBEAT:";
    heartbeat += getStateString(currentState);
    heartbeat += ":UP" + String(uptime) + "s";
    heartbeat += ":RSSI" + String(WiFi.RSSI());

    mqtt.publish(mqtt_topic_status.c_str(), heartbeat.c_str(), false);
    mqttLogf("[HEARTBEAT] %s", heartbeat.c_str());
  }
}

// ============================================
// BTS7960 MOTOR CONTROL FUNCTIONS
// ============================================
void setMotorSpeed(int openSpeed, int closeSpeed) {
  // BTS7960: RPWM controls forward (open), LPWM controls reverse (close)
  // Only one should be active at a time
  ledcWrite(PWM_CHANNEL_R, openSpeed);
  ledcWrite(PWM_CHANNEL_L, closeSpeed);
}

void startOpening() {
  if (debouncedLimitOpen) {
    mqttLog("[INFO] Already at OPEN limit");
    currentState = DOOR_OPEN;
    stopMotor();
    return;
  }

  mqttLog("[MOTOR] Opening door (BTS7960 RPWM active)...");
  currentState = DOOR_OPENING;
  motorStartTime = millis();
  setMotorSpeed(0, 0);  // Start at 0, ramp will handle speed

  if (mqtt.connected()) {
    send_status("OPENING");
  }
}

void startClosing() {
  if (debouncedLimitClosed) {
    mqttLog("[INFO] Already at CLOSED limit");
    currentState = DOOR_CLOSED;
    stopMotor();
    return;
  }

  mqttLog("[MOTOR] Closing door (BTS7960 LPWM active)...");
  currentState = DOOR_CLOSING;
  motorStartTime = millis();
  setMotorSpeed(0, 0);  // Start at 0, ramp will handle speed

  if (mqtt.connected()) {
    send_status("CLOSING");
  }
}

void stopMotor() {
  setMotorSpeed(0, 0);
  mqttLog("[MOTOR] Motor stopped - Both PWM outputs set to 0");
}

// ============================================
// LIMIT SWITCH HANDLING WITH DEBOUNCING
// ============================================
void checkLimitSwitches() {
  // Read raw values (LOW = triggered with pullup resistors)
  rawLimitOpen = (digitalRead(LIMIT_OPEN) == LOW);
  rawLimitClosed = (digitalRead(LIMIT_CLOSED) == LOW);

  // Debounce OPEN limit switch
  if (rawLimitOpen != lastRawLimitOpen) {
    lastRawLimitOpen = rawLimitOpen;
    limitOpenStableTime = millis();
  } else if (rawLimitOpen != debouncedLimitOpen) {
    if (millis() - limitOpenStableTime >= LIMIT_DEBOUNCE_MS) {
      debouncedLimitOpen = rawLimitOpen;

      if (debouncedLimitOpen) {
        publishLimitEvent("LIMIT_OPEN_HIT");
        if (currentState == DOOR_OPENING) {
          mqttLog("[LIMIT] Door reached OPEN position");
          stopMotor();
          currentState = DOOR_OPEN;
        }
      } else {
        publishLimitEvent("LIMIT_OPEN_CLEAR");
      }
    }
  }

  // Debounce CLOSED limit switch
  if (rawLimitClosed != lastRawLimitClosed) {
    lastRawLimitClosed = rawLimitClosed;
    limitClosedStableTime = millis();
  } else if (rawLimitClosed != debouncedLimitClosed) {
    if (millis() - limitClosedStableTime >= LIMIT_DEBOUNCE_MS) {
      debouncedLimitClosed = rawLimitClosed;

      if (debouncedLimitClosed) {
        publishLimitEvent("LIMIT_CLOSED_HIT");
        if (currentState == DOOR_CLOSING) {
          mqttLog("[LIMIT] Door reached CLOSED position");
          stopMotor();
          currentState = DOOR_CLOSED;
        }
      } else {
        publishLimitEvent("LIMIT_CLOSED_CLEAR");
      }
    }
  }

  // Safety check: stop motor if limit hit while moving
  if (currentState == DOOR_OPENING && debouncedLimitOpen) {
    stopMotor();
    currentState = DOOR_OPEN;
  }

  if (currentState == DOOR_CLOSING && debouncedLimitClosed) {
    stopMotor();
    currentState = DOOR_CLOSED;
  }
}

// ============================================
// UTILITY FUNCTIONS
// ============================================
const char* getStateString(DoorState state) {
  switch (state) {
    case DOOR_CLOSED:     return "CLOSED";
    case DOOR_OPEN:       return "OPEN";
    case DOOR_OPENING:    return "OPENING";
    case DOOR_CLOSING:    return "CLOSING";
    case DOOR_STOPPED:    return "STOPPED";
    case EMERGENCY_STOP:  return "EMERGENCY";
    default:              return "UNKNOWN";
  }
}
