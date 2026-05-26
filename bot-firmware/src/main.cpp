#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

// ==========================================
// CONFIGURATION
// ==========================================
const char* ssid = "CYXX 4879";
const char* password = "12345678";
const char* mqtt_server = "192.168.137.1";

// ==========================================
// PIN DEFINITIONS
// ==========================================
// Motor A (Left)
const int enableLeftMotor  = 4;
const int leftMotorPin1    = 16;
const int leftMotorPin2    = 17;

// Motor B (Right)
const int enableRightMotor = 5;
const int rightMotorPin1   = 18;
const int rightMotorPin2   = 19;

#define RIGHT_CHANNEL  0
#define LEFT_CHANNEL   1
#define PWM_FREQ       1000
#define PWM_RESOLUTION 8

// IR sensor pins (left to right): FL, ML, C, MR, FR
const int irPins[5] = {32, 35, 34, 39, 36};
const int weights[5] = {-2, -1, 0, 1, 2};

// MPU6050 I2C (matched to your wiring)
const int kI2CSda = 22;
const int kI2CScl = 21;

// Buzzer (passive buzzer — uses PWM tone)
const int kBuzzerPin = 13;
const int kBuzzerChannel = 2;

// Voltage sensor (0–25V module on ADC1)
const int kVoltageSensorPin = 33;

// Sensor config
const int kLineActiveLevel = LOW;
const bool kInvertLineSense = false;

// Motor direction corrections
const bool kSwapSides = false;
const bool kInvertLeft = false;
const bool kInvertRight = false;

// ==========================================
// PID TUNING (tested values from calibration)
// ==========================================
float Kp = 47.0f;
float Ki = 0.0f;
float Kd = 30.0f;

int baseSpeed = 93;
const int kMaxSpeed = 190;
int turnSpeed = 93;

// Navigation constants
const unsigned long kNodeCooldownMs = 800;
const unsigned long kCrossingTimeMs = 350;

// Battery constants (2S Li-ion: 6.0V empty, 8.4V full)
const float kBatteryFull = 8.4f;
const float kBatteryEmpty = 6.0f;
const float kBatteryLow = 6.6f;  // ~25% — trigger buzzer warning

// ==========================================
// NAVIGATION STATE MACHINE
// ==========================================
enum NavState {
  NAV_LINE_FOLLOW,
  NAV_NODE_DETECTED,
  NAV_TURNING,
  NAV_CROSSING,
  NAV_STOPPED
};

// ==========================================
// GLOBALS
// ==========================================
WiFiClient espClient;
PubSubClient client(espClient);
MPU6050 mpu6050(Wire);

bool isRunning = false;
String driveMode = "line";

// PID state
float lastError = 0.0f;
float integral = 0.0f;
unsigned long lastPidMs = 0;
unsigned long lastDebugMs = 0;

// Navigation state
NavState navState = NAV_LINE_FOLLOW;
String routeQueue = "";
int routeIndex = 0;
unsigned long nodeCooldownUntil = 0;
float turnStartAngle = 0.0f;
float turnTargetDelta = 0.0f;
unsigned long crossingStartMs = 0;

// Battery monitoring (smoothed)
float batteryVoltage = 7.4f;  // start at nominal
bool batteryLowWarned = false;

// Buzzer state (non-blocking)
unsigned long buzzerOffTime = 0;
int buzzerBeepCount = 0;
unsigned long buzzerNextBeep = 0;

// ==========================================
// FUNCTION PROTOTYPES
// ==========================================
void setup_wifi();
void reconnect();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void driveMotors(int leftSpeed, int rightSpeed, bool applySwap);
void setMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void followLine();
void readSensors(int sensorBits[5], int &activeCount, int &weightedSum);
void sendLog(String msg);
void publishNavState(String state);
String getNextRouteCommand();
void readBattery();
void buzzerTone(int freq, int durationMs);
void buzzerDestinationReached();
void buzzerUpdate();

void setup() {
  Serial.begin(115200);

  // Motor Pins
  pinMode(leftMotorPin1,  OUTPUT);
  pinMode(leftMotorPin2,  OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // IR Pins
  for (int i = 0; i < 5; i++) {
    pinMode(irPins[i], INPUT);
  }

  // PWM Setup for motors
  ledcSetup(RIGHT_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(LEFT_CHANNEL,  PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(enableRightMotor, RIGHT_CHANNEL);
  ledcAttachPin(enableLeftMotor,  LEFT_CHANNEL);

  // Buzzer PWM setup
  ledcSetup(kBuzzerChannel, 2000, 8);
  ledcAttachPin(kBuzzerPin, kBuzzerChannel);
  ledcWrite(kBuzzerChannel, 0);

  // Voltage sensor
  analogReadResolution(12);
  pinMode(kVoltageSensorPin, INPUT);

  stopMotors();

  // MPU6050 Setup
  Wire.begin(kI2CSda, kI2CScl);
  mpu6050.begin();
  Serial.println("Calibrating gyro... keep bot still!");
  mpu6050.calcGyroOffsets(true);
  Serial.println("Gyro calibration done.");

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  lastPidMs = millis();
  lastDebugMs = millis();
}

void loop() {
  // NON-BLOCKING MQTT: try once, don't block line following
  if (!client.connected()) {
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > 3000) {
      lastReconnectAttempt = millis();
      reconnect();
    }
  }
  client.loop();

  // Update gyro at 50Hz (every 20ms) — don't slow PID loop
  static unsigned long lastGyroUpdate = 0;
  if (millis() - lastGyroUpdate >= 20) {
    mpu6050.update();
    lastGyroUpdate = millis();
  }

  // Buzzer state machine (non-blocking)
  buzzerUpdate();

  // Publish alive heartbeat every 500ms (for dashboard Live/Off indicator)
  static unsigned long lastAlive = 0;
  if (millis() - lastAlive > 500) {
    if (client.connected()) {
      client.publish("ankit/bot/alive", "1");
    }
    lastAlive = millis();
  }

  // Read battery every 2 seconds
  static unsigned long lastBatteryRead = 0;
  if (millis() - lastBatteryRead > 2000) {
    readBattery();
    lastBatteryRead = millis();
  }

  if (isRunning) {
    if (driveMode == "line") {
      // ==========================================
      // NAVIGATION STATE MACHINE
      // ==========================================
      switch (navState) {

        case NAV_LINE_FOLLOW: {
          followLine();

          // Check for node (only after cooldown)
          if (millis() > nodeCooldownUntil && routeQueue.length() > 0) {
            int sensorBits[5];
            int activeCount, weightedSum;
            readSensors(sensorBits, activeCount, weightedSum);

            bool leftSide  = sensorBits[0] || sensorBits[1];
            bool rightSide = sensorBits[3] || sensorBits[4];

            // T-junction: both sides have sensors active (4 sensors)
            // + intersection: all 5 sensors active
            // L-turns are handled naturally by PID — no detection needed
            bool isNode = (leftSide && rightSide);

            if (isNode) {
              stopMotors();
              navState = NAV_NODE_DETECTED;
              sendLog("NODE DETECTED (active=" + String(activeCount) + ")");
              publishNavState("NODE_DETECTED");
            }
          }
          break;
        }

        case NAV_NODE_DETECTED: {
          String cmd = getNextRouteCommand();

          if (cmd == "") {
            // Route finished — destination reached!
            stopMotors();
            navState = NAV_STOPPED;
            buzzerDestinationReached();
            sendLog("Destination reached!");
            publishNavState("DESTINATION_REACHED");
            break;
          }

          sendLog("Executing: " + cmd);

          if (cmd == "L") {
            turnStartAngle = mpu6050.getAngleZ();
            turnTargetDelta = 90.0f;
            navState = NAV_TURNING;
            publishNavState("TURNING_LEFT");
          } else if (cmd == "R") {
            turnStartAngle = mpu6050.getAngleZ();
            turnTargetDelta = -90.0f;
            navState = NAV_TURNING;
            publishNavState("TURNING_RIGHT");
          } else if (cmd == "S") {
            crossingStartMs = millis();
            navState = NAV_CROSSING;
            publishNavState("CROSSING");
          } else if (cmd == "X") {
            stopMotors();
            navState = NAV_STOPPED;
            buzzerDestinationReached();
            sendLog("Destination reached (STOP)");
            publishNavState("DESTINATION_REACHED");
          }
          break;
        }

        case NAV_TURNING: {
          if (turnTargetDelta > 0) {
            setMotors(-turnSpeed, turnSpeed);  // Turn LEFT
          } else {
            setMotors(turnSpeed, -turnSpeed);  // Turn RIGHT
          }

          float currentAngle = mpu6050.getAngleZ();
          float delta = currentAngle - turnStartAngle;

          bool turnComplete = false;
          if (turnTargetDelta > 0 && delta >= turnTargetDelta) turnComplete = true;
          if (turnTargetDelta < 0 && delta <= turnTargetDelta) turnComplete = true;

          if (turnComplete) {
            stopMotors();
            delay(50);
            lastError = 0.0f;
            integral = 0.0f;
            lastPidMs = millis();
            nodeCooldownUntil = millis() + kNodeCooldownMs;
            navState = NAV_LINE_FOLLOW;
            sendLog("Turn complete. Delta: " + String(delta, 1));
            publishNavState("FOLLOWING");
          }

          // Debug angle during turn
          static unsigned long lastTurnDebug = 0;
          if (millis() - lastTurnDebug > 100) {
            lastTurnDebug = millis();
            Serial.printf("TURN: start=%.1f cur=%.1f delta=%.1f target=%.1f\n",
              turnStartAngle, currentAngle, delta, turnTargetDelta);
          }
          break;
        }

        case NAV_CROSSING: {
          setMotors(baseSpeed, baseSpeed);
          if (millis() - crossingStartMs >= kCrossingTimeMs) {
            lastError = 0.0f;
            integral = 0.0f;
            lastPidMs = millis();
            nodeCooldownUntil = millis() + kNodeCooldownMs;
            navState = NAV_LINE_FOLLOW;
            sendLog("Crossing complete");
            publishNavState("FOLLOWING");
          }
          break;
        }

        case NAV_STOPPED: {
          stopMotors();
          break;
        }
      }
    }
    // Manual mode is handled directly by MQTT callbacks
  } else {
    stopMotors();
  }
}

// ==========================================
// SENSOR READING
// ==========================================
void readSensors(int sensorBits[5], int &activeCount, int &weightedSum) {
  activeCount = 0;
  weightedSum = 0;
  for (int i = 0; i < 5; i++) {
    int reading = digitalRead(irPins[i]);
    bool onLine = (reading == kLineActiveLevel);
    if (kInvertLineSense) onLine = !onLine;
    sensorBits[i] = onLine ? 1 : 0;
    if (onLine) {
      activeCount++;
      weightedSum += weights[i];
    }
  }
}

// ==========================================
// MOTOR CONTROL
// ==========================================
void driveMotors(int leftSpeed, int rightSpeed, bool applySwap) {
  if (applySwap && kSwapSides) {
    int temp = leftSpeed;
    leftSpeed = rightSpeed;
    rightSpeed = temp;
  }
  if (kInvertLeft) leftSpeed = -leftSpeed;
  if (kInvertRight) rightSpeed = -rightSpeed;

  int leftOut = leftSpeed;
  int rightOut = rightSpeed;

  if (leftOut > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else if (leftOut < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }
  ledcWrite(LEFT_CHANNEL, abs(leftOut));

  if (rightOut > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else if (rightOut < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }
  ledcWrite(RIGHT_CHANNEL, abs(rightOut));

  // Publish telemetry (5 Hz)
  static unsigned long lastTelemetry = 0;
  if (millis() - lastTelemetry > 200) {
    if (client.connected()) {
      String msg = String(leftOut) + "," + String(rightOut);
      client.publish("ankit/bot/telemetry", msg.c_str());
    }
    lastTelemetry = millis();
  }
}

void setMotors(int leftSpeed, int rightSpeed) {
  driveMotors(leftSpeed, rightSpeed, true);
}

void stopMotors() {
  setMotors(0, 0);
}

// ==========================================
// PID LINE FOLLOWING
// ==========================================
void followLine() {
  unsigned long now = millis();

  int sensorBits[5];
  int activeCount, weightedSum;
  readSensors(sensorBits, activeCount, weightedSum);

  // Publish sensor readings (5 Hz)
  static unsigned long lastSensorPub = 0;
  if (now - lastSensorPub > 200) {
    if (client.connected()) {
      String msg = String(sensorBits[0]) + "," + String(sensorBits[1]) + ","
                 + String(sensorBits[2]) + "," + String(sensorBits[3]) + ","
                 + String(sensorBits[4]);
      client.publish("ankit/bot/sensors", msg.c_str());
    }
    lastSensorPub = now;
  }

  // PID Calculation
  float error = lastError;
  if (activeCount > 0) {
    error = static_cast<float>(weightedSum) / activeCount;
  }

  float dt = (now - lastPidMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.01f;
  lastPidMs = now;

  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int leftSpeed = baseSpeed + static_cast<int>(correction);
  int rightSpeed = baseSpeed - static_cast<int>(correction);
  leftSpeed = constrain(leftSpeed, 0, kMaxSpeed);
  rightSpeed = constrain(rightSpeed, 0, kMaxSpeed);

  setMotors(leftSpeed, rightSpeed);

  // Debug
  if (now - lastDebugMs >= 200) {
    lastDebugMs = now;
    Serial.printf("S:%d%d%d%d%d E:%.2f L:%d R:%d Z:%.1f\n",
      sensorBits[0], sensorBits[1], sensorBits[2], sensorBits[3], sensorBits[4],
      error, leftSpeed, rightSpeed, mpu6050.getAngleZ());
  }
}

// ==========================================
// ROUTE QUEUE
// ==========================================
String getNextRouteCommand() {
  if (routeQueue.length() == 0 || routeIndex < 0) return "";

  int startPos = 0;
  int commaPos = -1;

  // Walk to the routeIndex-th token
  for (int i = 0; i <= routeIndex; i++) {
    startPos = (i == 0) ? 0 : commaPos + 1;
    commaPos = routeQueue.indexOf(',', startPos);
    if (commaPos == -1 && i < routeIndex) return "";
  }

  String cmd;
  if (commaPos == -1) cmd = routeQueue.substring(startPos);
  else cmd = routeQueue.substring(startPos, commaPos);
  cmd.trim();
  cmd.toUpperCase();

  routeIndex++;

  // Publish remaining steps
  if (client.connected()) {
    String remaining = (commaPos == -1) ? "" : routeQueue.substring(commaPos + 1);
    String info = "STEP:" + String(routeIndex) + "|CMD:" + cmd + "|REM:" + remaining;
    client.publish("ankit/bot/nav", info.c_str());
  }

  return cmd;
}

// ==========================================
// BUZZER
// ==========================================
void buzzerTone(int freq, int durationMs) {
  ledcWriteTone(kBuzzerChannel, freq);
  buzzerOffTime = millis() + durationMs;
}

void buzzerDestinationReached() {
  // Play a happy two-tone beep
  buzzerBeepCount = 3;
  buzzerNextBeep = millis();
}

void buzzerUpdate() {
  // Turn off buzzer when duration expires
  if (buzzerOffTime > 0 && millis() >= buzzerOffTime) {
    ledcWriteTone(kBuzzerChannel, 0);
    buzzerOffTime = 0;
  }

  // Multi-beep sequence for destination reached
  if (buzzerBeepCount > 0 && millis() >= buzzerNextBeep) {
    buzzerTone(buzzerBeepCount == 2 ? 2500 : 2000, 150);
    buzzerBeepCount--;
    buzzerNextBeep = millis() + 250;
  }
}

// ==========================================
// BATTERY MONITORING
// ==========================================
void readBattery() {
  int raw = analogRead(kVoltageSensorPin);
  float voltage = (raw / 4095.0f) * 3.3f * 5.0f;  // 5:1 voltage divider

  // Exponential moving average for smoothing
  batteryVoltage = batteryVoltage * 0.8f + voltage * 0.2f;

  float percentage = constrain((batteryVoltage - kBatteryEmpty) / (kBatteryFull - kBatteryEmpty) * 100.0f, 0, 100);

  // Publish battery data
  if (client.connected()) {
    String msg = String(batteryVoltage, 2) + "," + String((int)percentage);
    client.publish("ankit/bot/battery", msg.c_str());
  }

  // Low battery warning (one-shot)
  if (batteryVoltage <= kBatteryLow && !batteryLowWarned) {
    batteryLowWarned = true;
    buzzerTone(800, 1000);  // low tone, long beep
    sendLog("WARNING: Battery low! " + String(batteryVoltage, 1) + "V");
  }
  if (batteryVoltage > kBatteryLow + 0.3f) {
    batteryLowWarned = false;  // reset if voltage recovers
  }
}

// ==========================================
// NETWORK & MQTT
// ==========================================
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.printf("MQTT [%s]: %s\n", topic, message.c_str());

  if (String(topic) == "ankit/bot/command") {
    if (message == "START") {
      isRunning = true;
      lastPidMs = millis();
      lastError = 0.0f;
      integral = 0.0f;
      navState = NAV_LINE_FOLLOW;
      nodeCooldownUntil = millis() + 500;
      sendLog("Bot Started");
      publishNavState("FOLLOWING");
    } else if (message == "STOP") {
      isRunning = false;
      navState = NAV_STOPPED;
      routeQueue = "";
      routeIndex = 0;
      stopMotors();
      sendLog("Bot Stopped");
      publishNavState("STOPPED");
    }
  } else if (String(topic) == "ankit/bot/pid") {
    int c1 = message.indexOf(',');
    int c2 = message.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > 0) {
      Kp = message.substring(0, c1).toFloat();
      Ki = message.substring(c1 + 1, c2).toFloat();
      Kd = message.substring(c2 + 1).toFloat();
      sendLog("PID: Kp=" + String(Kp) + " Ki=" + String(Ki) + " Kd=" + String(Kd));
    }
  } else if (String(topic) == "ankit/bot/speeds") {
    baseSpeed = message.toInt();
    turnSpeed = baseSpeed;
    sendLog("Speed: " + String(baseSpeed));
  } else if (String(topic) == "ankit/bot/route") {
    routeQueue = message;
    routeIndex = 0;
    navState = NAV_LINE_FOLLOW;
    nodeCooldownUntil = millis() + 500;
    sendLog("Route: " + routeQueue);
    publishNavState("ROUTE_LOADED");
  } else if (String(topic) == "ankit/bot/mode") {
    driveMode = message;
    sendLog("Mode: " + driveMode);
    if (driveMode == "line") {
      lastPidMs = millis();
      lastError = 0.0f;
      integral = 0.0f;
    }
  } else if (String(topic) == "ankit/bot/manual") {
    if (driveMode == "manual" && isRunning) {
      if (message == "FWD")        setMotors(baseSpeed, baseSpeed);
      else if (message == "BWD")   setMotors(-baseSpeed, -baseSpeed);
      else if (message == "LEFT")  setMotors(-baseSpeed, baseSpeed);
      else if (message == "RIGHT") setMotors(baseSpeed, -baseSpeed);
      else if (message == "STOP")  stopMotors();
    }
  }
}

void sendLog(String msg) {
  Serial.println(msg);
  if (client.connected()) {
    client.publish("ankit/bot/logs", msg.c_str());
  }
}

void publishNavState(String state) {
  if (client.connected()) {
    client.publish("ankit/bot/nav", state.c_str());
  }
}

void reconnect() {
  // NON-BLOCKING: try once and return immediately
  Serial.print("MQTT connecting...");
  if (client.connect("ESP32BotClient")) {
    Serial.println("connected");
    client.subscribe("ankit/bot/command");
    client.subscribe("ankit/bot/pid");
    client.subscribe("ankit/bot/speeds");
    client.subscribe("ankit/bot/mode");
    client.subscribe("ankit/bot/manual");
    client.subscribe("ankit/bot/route");
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" will retry in 3s");
    // No delay! Returns immediately so line following keeps working
  }
}
