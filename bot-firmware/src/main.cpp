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

// PWM channels — keep motors on same timer group, buzzer separate
#define LEFT_CHANNEL   0
#define RIGHT_CHANNEL  1
#define BUZZER_CHANNEL 4   // channel 4 uses timer 2, avoids motor timer 0
#define PWM_FREQ       1000
#define PWM_RESOLUTION 8

// IR sensor pins (left to right): FL, ML, C, MR, FR
const int irPins[5] = {32, 35, 34, 39, 36};
const int weights[5] = {-2, -1, 0, 1, 2};

// MPU6050 I2C
const int kI2CSda = 22;
const int kI2CScl = 21;

// Buzzer
const int kBuzzerPin = 13;

// ==========================================
// PID TUNING (your working values)
// ==========================================
float Kp = 50.0f;
float Ki = 0.0f;
float Kd = 30.0f;

int baseSpeed = 95;
const int kMaxSpeed = 190;
const int turnSpeed = 120;  // fixed — enough torque to spin in place

// Navigation constants
const unsigned long kNodeCooldownMs = 800;
const unsigned long kCrossingTimeMs = 350;
const float kTurnAngle = 80.0f;  // target turn degrees (70-110 range after inertia)

// ==========================================
// STATE
// ==========================================
enum NavState {
  NAV_LINE_FOLLOW,
  NAV_NODE_DETECTED,
  NAV_TURNING,
  NAV_CROSSING,
  NAV_STOPPED
};

WiFiClient espClient;
PubSubClient client(espClient);
MPU6050 mpu6050(Wire);

bool isRunning = false;
String driveMode = "line";

// PID
float lastError = 0.0f;
float integral = 0.0f;
unsigned long lastPidMs = 0;

// Navigation
NavState navState = NAV_LINE_FOLLOW;
String routeQueue = "";
unsigned long nodeCooldownUntil = 0;
float turnStartAngle = 0.0f;
float turnTargetDelta = 0.0f;
unsigned long crossingStartMs = 0;

// Buzzer
unsigned long buzzerOffTime = 0;
int buzzerBeepCount = 0;
unsigned long buzzerNextBeep = 0;

// ==========================================
// PROTOTYPES
// ==========================================
void setup_wifi();
void reconnect();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void setMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void followLine();
void readSensors(int bits[5], int &cnt, int &wsum);
void sendLog(String msg);
void publishNav(String s);
String popRouteCmd();
void buzzerPlay();
void buzzerUpdate();

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  // Motor direction pins
  pinMode(leftMotorPin1,  OUTPUT);
  pinMode(leftMotorPin2,  OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // Motor PWM
  ledcSetup(LEFT_CHANNEL,  PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(RIGHT_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(enableLeftMotor,  LEFT_CHANNEL);
  ledcAttachPin(enableRightMotor, RIGHT_CHANNEL);

  // IR sensors
  for (int i = 0; i < 5; i++) pinMode(irPins[i], INPUT);

  // Buzzer — separate channel from motors
  ledcSetup(BUZZER_CHANNEL, 2000, 8);
  ledcAttachPin(kBuzzerPin, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, 0);

  stopMotors();

  // ---- Motor self-test ----
  Serial.println("Motor test: LEFT forward...");
  digitalWrite(leftMotorPin1, HIGH); digitalWrite(leftMotorPin2, LOW);
  ledcWrite(LEFT_CHANNEL, 120);
  delay(300);
  ledcWrite(LEFT_CHANNEL, 0);
  digitalWrite(leftMotorPin1, LOW);

  Serial.println("Motor test: RIGHT forward...");
  digitalWrite(rightMotorPin1, HIGH); digitalWrite(rightMotorPin2, LOW);
  ledcWrite(RIGHT_CHANNEL, 120);
  delay(300);
  ledcWrite(RIGHT_CHANNEL, 0);
  digitalWrite(rightMotorPin1, LOW);

  Serial.println("Motor test done. Both wheels should have spun.");
  stopMotors();

  // MPU6050
  Wire.begin(kI2CSda, kI2CScl);
  mpu6050.begin();
  Serial.println("Calibrating gyro... keep bot still!");
  mpu6050.calcGyroOffsets(true);
  Serial.println("Gyro calibration done.");

  // WiFi + MQTT
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  lastPidMs = millis();
  Serial.println("=== READY ===");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  // Non-blocking MQTT reconnect
  if (!client.connected()) {
    static unsigned long lastTry = 0;
    if (millis() - lastTry > 3000) {
      lastTry = millis();
      reconnect();
    }
  }
  client.loop();

  // Gyro update at 50Hz
  static unsigned long lastGyro = 0;
  if (millis() - lastGyro >= 20) {
    mpu6050.update();
    lastGyro = millis();
  }

  buzzerUpdate();

  // Heartbeat + gyro publish every 500ms
  static unsigned long lastAlive = 0;
  if (millis() - lastAlive > 500) {
    if (client.connected()) {
      client.publish("ankit/bot/alive", "1");
      // Publish gyro angle for dashboard
      String gyroMsg = String(mpu6050.getAngleX(), 1) + "," +
                       String(mpu6050.getAngleY(), 1) + "," +
                       String(mpu6050.getAngleZ(), 1);
      client.publish("ankit/bot/gyro", gyroMsg.c_str());
    }
    lastAlive = millis();
  }

  // ==========================================
  // MAIN LOGIC
  // ==========================================
  if (!isRunning) {
    stopMotors();
    return;
  }

  if (driveMode == "manual") return;

  // LINE FOLLOWING + NAVIGATION
  switch (navState) {

    case NAV_LINE_FOLLOW: {
      followLine();

      // Node detection: only when we have a route
      if (millis() > nodeCooldownUntil && routeQueue.length() > 0) {
        int bits[5]; int cnt, wsum;
        readSensors(bits, cnt, wsum);
        bool leftSide  = bits[0] || bits[1];
        bool rightSide = bits[3] || bits[4];

        if (leftSide && rightSide) {
          stopMotors();
          navState = NAV_NODE_DETECTED;
          sendLog("NODE DETECTED");
          publishNav("NODE_DETECTED");
        }
      }
      break;
    }

    case NAV_NODE_DETECTED: {
      String cmd = popRouteCmd();
      if (cmd == "" || cmd == "X") {
        stopMotors();
        isRunning = false;
        navState = NAV_STOPPED;
        buzzerPlay();
        sendLog("Destination reached!");
        publishNav("DESTINATION_REACHED");
        break;
      }

      sendLog("Executing: " + cmd);

      if (cmd == "L") {
        turnStartAngle = mpu6050.getAngleZ();
        turnTargetDelta = kTurnAngle;  // +80 degrees
        navState = NAV_TURNING;
        publishNav("TURNING_LEFT");
      } else if (cmd == "R") {
        turnStartAngle = mpu6050.getAngleZ();
        turnTargetDelta = -kTurnAngle;  // -80 degrees
        navState = NAV_TURNING;
        publishNav("TURNING_RIGHT");
      } else if (cmd == "S") {
        crossingStartMs = millis();
        navState = NAV_CROSSING;
        publishNav("CROSSING");
      }
      break;
    }

    case NAV_TURNING: {
      // Spin in place: one motor forward, one motor backward
      if (turnTargetDelta > 0) {
        // LEFT turn: left motor backward, right motor forward
        digitalWrite(leftMotorPin1, LOW);  digitalWrite(leftMotorPin2, HIGH);
        ledcWrite(LEFT_CHANNEL, turnSpeed);
        digitalWrite(rightMotorPin1, HIGH); digitalWrite(rightMotorPin2, LOW);
        ledcWrite(RIGHT_CHANNEL, turnSpeed);
      } else {
        // RIGHT turn: left motor forward, right motor backward
        digitalWrite(leftMotorPin1, HIGH); digitalWrite(leftMotorPin2, LOW);
        ledcWrite(LEFT_CHANNEL, turnSpeed);
        digitalWrite(rightMotorPin1, LOW);  digitalWrite(rightMotorPin2, HIGH);
        ledcWrite(RIGHT_CHANNEL, turnSpeed);
      }

      float delta = mpu6050.getAngleZ() - turnStartAngle;
      float absDelta = abs(delta);

      // Turn is done when angle reaches target (80°)
      // Accepts anything 70°+ as "close enough" after motor inertia
      bool done = (absDelta >= (kTurnAngle - 10.0f));  // 70°+

      // Safety: force stop if we've overshot past 110°
      if (absDelta > 110.0f) done = true;

      if (done) {
        stopMotors();
        delay(50);
        lastError = 0; integral = 0; lastPidMs = millis();
        nodeCooldownUntil = millis() + kNodeCooldownMs;
        navState = NAV_LINE_FOLLOW;
        sendLog("Turn done, angle=" + String(delta, 1));
        publishNav("FOLLOWING");
      }
      break;
    }

    case NAV_CROSSING: {
      setMotors(baseSpeed, baseSpeed);
      if (millis() - crossingStartMs >= kCrossingTimeMs) {
        lastError = 0; integral = 0; lastPidMs = millis();
        nodeCooldownUntil = millis() + kNodeCooldownMs;
        navState = NAV_LINE_FOLLOW;
        sendLog("Crossing done");
        publishNav("FOLLOWING");
      }
      break;
    }

    case NAV_STOPPED: {
      stopMotors();
      break;
    }
  }
}

// ==========================================
// SENSOR READING
// ==========================================
void readSensors(int bits[5], int &cnt, int &wsum) {
  cnt = 0; wsum = 0;
  for (int i = 0; i < 5; i++) {
    bits[i] = (digitalRead(irPins[i]) == LOW) ? 1 : 0;
    if (bits[i]) { cnt++; wsum += weights[i]; }
  }
}

// ==========================================
// MOTOR CONTROL
// ==========================================
void setMotors(int leftSpd, int rightSpd) {
  // Left motor direction
  if (leftSpd > 0)      { digitalWrite(leftMotorPin1, HIGH); digitalWrite(leftMotorPin2, LOW); }
  else if (leftSpd < 0) { digitalWrite(leftMotorPin1, LOW);  digitalWrite(leftMotorPin2, HIGH); }
  else                   { digitalWrite(leftMotorPin1, LOW);  digitalWrite(leftMotorPin2, LOW); }
  ledcWrite(LEFT_CHANNEL, abs(leftSpd));

  // Right motor direction
  if (rightSpd > 0)      { digitalWrite(rightMotorPin1, HIGH); digitalWrite(rightMotorPin2, LOW); }
  else if (rightSpd < 0) { digitalWrite(rightMotorPin1, LOW);  digitalWrite(rightMotorPin2, HIGH); }
  else                    { digitalWrite(rightMotorPin1, LOW);  digitalWrite(rightMotorPin2, LOW); }
  ledcWrite(RIGHT_CHANNEL, abs(rightSpd));

  // Publish telemetry at 5Hz
  static unsigned long lastTel = 0;
  if (millis() - lastTel > 200) {
    if (client.connected()) {
      String msg = String(leftSpd) + "," + String(rightSpd);
      client.publish("ankit/bot/telemetry", msg.c_str());
    }
    lastTel = millis();
  }
}

void stopMotors() {
  digitalWrite(leftMotorPin1, LOW);  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW); digitalWrite(rightMotorPin2, LOW);
  ledcWrite(LEFT_CHANNEL, 0);
  ledcWrite(RIGHT_CHANNEL, 0);
}

// ==========================================
// PID LINE FOLLOWING
// ==========================================
void followLine() {
  unsigned long now = millis();
  int bits[5]; int cnt, wsum;
  readSensors(bits, cnt, wsum);

  // Publish sensors at 5Hz
  static unsigned long lastSens = 0;
  if (now - lastSens > 200) {
    if (client.connected()) {
      String msg = String(bits[0])+","+String(bits[1])+","+String(bits[2])+","+String(bits[3])+","+String(bits[4]);
      client.publish("ankit/bot/sensors", msg.c_str());
    }
    lastSens = now;
  }

  float error = lastError;
  if (cnt > 0) error = (float)wsum / cnt;

  float dt = (now - lastPidMs) / 1000.0f;
  if (dt <= 0) dt = 0.01f;
  lastPidMs = now;

  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int L = constrain(baseSpeed + (int)correction, 0, kMaxSpeed);
  int R = constrain(baseSpeed - (int)correction, 0, kMaxSpeed);
  setMotors(L, R);
}

// ==========================================
// ROUTE QUEUE
// ==========================================
String popRouteCmd() {
  if (routeQueue.length() == 0) return "";

  int comma = routeQueue.indexOf(',');
  String cmd;
  if (comma == -1) {
    cmd = routeQueue;
    routeQueue = "";
  } else {
    cmd = routeQueue.substring(0, comma);
    routeQueue = routeQueue.substring(comma + 1);
  }
  cmd.trim();
  cmd.toUpperCase();
  return cmd;
}

// ==========================================
// BUZZER
// ==========================================
void buzzerPlay() {
  buzzerBeepCount = 3;
  buzzerNextBeep = millis();
}

void buzzerUpdate() {
  if (buzzerOffTime > 0 && millis() >= buzzerOffTime) {
    ledcWriteTone(BUZZER_CHANNEL, 0);
    buzzerOffTime = 0;
  }
  if (buzzerBeepCount > 0 && millis() >= buzzerNextBeep) {
    ledcWriteTone(BUZZER_CHANNEL, buzzerBeepCount == 2 ? 2500 : 2000);
    buzzerOffTime = millis() + 150;
    buzzerBeepCount--;
    buzzerNextBeep = millis() + 250;
  }
}

// ==========================================
// NETWORK & MQTT
// ==========================================
void setup_wifi() {
  Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  String t = String(topic);

  if (t == "ankit/bot/command") {
    if (msg == "START") {
      isRunning = true;
      routeQueue = "";
      navState = NAV_LINE_FOLLOW;
      lastError = 0; integral = 0; lastPidMs = millis();
      nodeCooldownUntil = millis() + 500;
      sendLog("Started (line follow)");
      publishNav("FOLLOWING");
    } else if (msg == "STOP") {
      isRunning = false;
      navState = NAV_STOPPED;
      routeQueue = "";
      stopMotors();
      sendLog("Stopped");
      publishNav("STOPPED");
    }
  }
  else if (t == "ankit/bot/route") {
    routeQueue = msg;
    isRunning = true;
    navState = NAV_LINE_FOLLOW;
    lastError = 0; integral = 0; lastPidMs = millis();
    nodeCooldownUntil = millis() + 500;
    sendLog("Route loaded: " + msg + " — auto-started");
    publishNav("ROUTE_LOADED");
  }
  else if (t == "ankit/bot/pid") {
    int c1 = msg.indexOf(','); int c2 = msg.indexOf(',', c1+1);
    if (c1 > 0 && c2 > 0) {
      Kp = msg.substring(0, c1).toFloat();
      Ki = msg.substring(c1+1, c2).toFloat();
      Kd = msg.substring(c2+1).toFloat();
      sendLog("PID: " + String(Kp) + "," + String(Ki) + "," + String(Kd));
    }
  }
  else if (t == "ankit/bot/speeds") {
    baseSpeed = msg.toInt();
    sendLog("Speed: " + String(baseSpeed));
  }
  else if (t == "ankit/bot/mode") {
    driveMode = msg;
    sendLog("Mode: " + driveMode);
  }
  else if (t == "ankit/bot/manual") {
    if (driveMode == "manual" && isRunning) {
      if (msg == "FWD")        setMotors(baseSpeed, baseSpeed);
      else if (msg == "BWD")   setMotors(-baseSpeed, -baseSpeed);
      else if (msg == "LEFT")  setMotors(-baseSpeed, baseSpeed);
      else if (msg == "RIGHT") setMotors(baseSpeed, -baseSpeed);
      else if (msg == "STOP")  stopMotors();
    }
  }
}

void sendLog(String msg) {
  Serial.println(msg);
  if (client.connected()) client.publish("ankit/bot/logs", msg.c_str());
}

void publishNav(String s) {
  if (client.connected()) client.publish("ankit/bot/nav", s.c_str());
}

void reconnect() {
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
    Serial.println("failed, rc=" + String(client.state()) + " retry in 3s");
  }
}
