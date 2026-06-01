#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ==========================================
// CONFIGURATION
// ==========================================
const char* ssid = "CYXX 4879";
const char* password = "12345678";
const char* mqtt_server = "192.168.137.1";

// ==========================================
// MOTOR PINS
// ==========================================
// Left motor
const int ENA = 4;   // Enable (PWM speed)
const int IN1 = 16;  // Direction pin 1
const int IN2 = 17;  // Direction pin 2

// Right motor — pins SWAPPED to fix direction
const int ENB = 5;   // Enable (PWM speed)
const int IN3 = 19;  // Direction pin 1 (swapped)
const int IN4 = 18;  // Direction pin 2 (swapped)

// ==========================================
// ULTRASONIC + SERVO PINS
// ==========================================
const int TRIG_PIN  = 25;
const int ECHO_PIN  = 26;
const int SERVO_PIN = 13;

// ==========================================
// PWM CHANNELS (each on a separate timer)
// ==========================================
#define LEFT_PWM_CH   0   // timer 0, 1kHz
#define RIGHT_PWM_CH  2   // timer 1, 1kHz
#define SERVO_CH      4   // timer 2, 50Hz

// IR sensors (left to right): FL, ML, C, MR, FR
const int irPins[5] = {32, 35, 34, 39, 36};
const int weights[5] = {-2, -1, 0, 1, 2};

// ==========================================
// TUNING
// ==========================================
float Kp = 50.0f;
float Ki = 0.0f;
float Kd = 30.0f;
int baseSpeed = 95;
const int turnSpeed = 160;

// Navigation timing
const unsigned long kTurnTimeMs     = 700;
const unsigned long kCrossingTimeMs = 350;
const unsigned long kNodeCooldownMs = 800;

// Obstacle detection
const int kObstacleThreshold = 20;   // cm — stop if closer
const int kClearThreshold    = 25;   // cm — resume when farther

// ==========================================
// STATE
// ==========================================
enum NavState { NAV_FOLLOW, NAV_NODE, NAV_TURN, NAV_CROSS, NAV_STOP };

WiFiClient espClient;
PubSubClient client(espClient);

bool isRunning = false;
String driveMode = "line";

// PID state
float lastError = 0.0f;
float integral = 0.0f;
unsigned long lastPidMs = 0;

// Navigation state
NavState navState = NAV_FOLLOW;
String routeQueue = "";
unsigned long nodeCooldownUntil = 0;
int turnDir = 0;
unsigned long turnStartMs = 0;
unsigned long crossStartMs = 0;

// Obstacle state
bool obstacleDetected = false;

// ==========================================
// SERVO CONTROL (raw LEDC — no extra library)
// ==========================================
void servoWrite(int angle) {
  // SG90 at 50Hz, 10-bit (1024 steps)
  // 0° ≈ 26 duty,  180° ≈ 128 duty
  int duty = map(constrain(angle, 0, 180), 0, 180, 26, 128);
  ledcWrite(SERVO_CH, duty);
}

// ==========================================
// ULTRASONIC DISTANCE MEASUREMENT
// ==========================================
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000); // ~4m max range
  if (duration == 0) return 999;                   // no echo = far away
  return duration * 0.034 / 2;                     // µs → cm
}

// ==========================================
// MOTOR CONTROL
// ==========================================
void motorLeft(int spd) {
  if (spd > 0)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (spd < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else               { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW); }
  ledcWrite(LEFT_PWM_CH, abs(spd));
}

void motorRight(int spd) {
  // Direction inverted to match physical wheel direction
  if (spd > 0)      { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else if (spd < 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else               { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW); }
  ledcWrite(RIGHT_PWM_CH, abs(spd));
}

void setMotors(int left, int right) {
  motorLeft(left);
  motorRight(right);
  static unsigned long lastTel = 0;
  if (millis() - lastTel > 200 && client.connected()) {
    client.publish("ankit/bot/telemetry", (String(left) + "," + String(right)).c_str());
    lastTel = millis();
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(LEFT_PWM_CH, 0);
  ledcWrite(RIGHT_PWM_CH, 0);
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
// PID LINE FOLLOWING
// ==========================================
void followLine() {
  unsigned long now = millis();
  int bits[5]; int cnt, wsum;
  readSensors(bits, cnt, wsum);

  float error = lastError;
  if (cnt > 0) error = (float)wsum / cnt;

  float dt = (now - lastPidMs) / 1000.0f;
  if (dt <= 0) dt = 0.01f;
  lastPidMs = now;

  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int L = constrain(baseSpeed + (int)correction, -baseSpeed, 190);
  int R = constrain(baseSpeed - (int)correction, -baseSpeed, 190);
  setMotors(L, R);
}

// ==========================================
// ROUTE HELPERS
// ==========================================
String popRoute() {
  if (routeQueue.length() == 0) return "";
  int c = routeQueue.indexOf(',');
  String cmd;
  if (c == -1) { cmd = routeQueue; routeQueue = ""; }
  else { cmd = routeQueue.substring(0, c); routeQueue = routeQueue.substring(c + 1); }
  cmd.trim(); cmd.toUpperCase();
  return cmd;
}

void sendLog(String msg) {
  Serial.println(msg);
  if (client.connected()) client.publish("ankit/bot/logs", msg.c_str());
}

void publishNav(String s) {
  if (client.connected()) client.publish("ankit/bot/nav", s.c_str());
}

// ==========================================
// OBSTACLE DETECTION & SCANNING
// ==========================================
void servoScan() {
  // Quick 3-position scan (bot is stopped, so blocking is fine)
  servoWrite(30);   // look left
  delay(300);
  long leftDist = measureDistance();

  servoWrite(90);   // look center
  delay(300);
  long centerDist = measureDistance();

  servoWrite(150);  // look right
  delay(300);
  long rightDist = measureDistance();

  servoWrite(90);   // return to center

  // Publish scan: "left,center,right"
  if (client.connected()) {
    String msg = String(leftDist) + "," + String(centerDist) + "," + String(rightDist);
    client.publish("ankit/bot/scan", msg.c_str());
  }
  sendLog("Scan L:" + String(leftDist) + " C:" + String(centerDist) + " R:" + String(rightDist));
}

void checkObstacle() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 100) return;  // check every 100ms
  lastCheck = millis();

  long dist = measureDistance();

  // Publish distance
  if (client.connected()) {
    client.publish("ankit/bot/obstacle", String(dist).c_str());
  }

  if (!obstacleDetected && dist > 0 && dist < kObstacleThreshold) {
    // Obstacle appeared!
    obstacleDetected = true;
    stopMotors();
    sendLog("OBSTACLE at " + String(dist) + "cm!");
    publishNav("OBSTACLE");
    servoScan();  // scan the area once
  }
  else if (obstacleDetected && dist >= kClearThreshold) {
    // Path is clear again
    obstacleDetected = false;
    lastPidMs = millis();  // reset PID timing to avoid spike
    sendLog("Path clear — resuming");
    publishNav("FOLLOWING");
  }
}

// ==========================================
// MQTT CALLBACK
// ==========================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  String t = String(topic);

  if (t == "ankit/bot/command") {
    if (msg == "START") {
      isRunning = true; routeQueue = ""; navState = NAV_FOLLOW;
      obstacleDetected = false;
      lastError = 0; integral = 0; lastPidMs = millis();
      nodeCooldownUntil = millis() + 500;
      sendLog("Started"); publishNav("FOLLOWING");
    } else if (msg == "STOP") {
      isRunning = false; navState = NAV_STOP; routeQueue = "";
      obstacleDetected = false;
      stopMotors(); sendLog("Stopped"); publishNav("STOPPED");
    }
  }
  else if (t == "ankit/bot/route") {
    routeQueue = msg; isRunning = true; navState = NAV_FOLLOW;
    obstacleDetected = false;
    lastError = 0; integral = 0; lastPidMs = millis();
    nodeCooldownUntil = millis() + 500;
    sendLog("Route: " + msg); publishNav("ROUTE_LOADED");
  }
  else if (t == "ankit/bot/pid") {
    int c1 = msg.indexOf(','); int c2 = msg.indexOf(',', c1+1);
    if (c1 > 0 && c2 > 0) {
      Kp = msg.substring(0, c1).toFloat();
      Ki = msg.substring(c1+1, c2).toFloat();
      Kd = msg.substring(c2+1).toFloat();
    }
  }
  else if (t == "ankit/bot/speeds") { baseSpeed = msg.toInt(); }
  else if (t == "ankit/bot/mode") { driveMode = msg; }
  else if (t == "ankit/bot/manual") {
    if (driveMode == "manual" && isRunning) {
      if (msg == "FWD")        setMotors(baseSpeed, baseSpeed);
      else if (msg == "BWD")   setMotors(-baseSpeed, -baseSpeed);
      else if (msg == "LEFT")  setMotors(-turnSpeed, turnSpeed);
      else if (msg == "RIGHT") setMotors(turnSpeed, -turnSpeed);
      else                     stopMotors();
    }
  }
}

// ==========================================
// WIFI & MQTT
// ==========================================
void setup_wifi() {
  Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
}

void reconnect() {
  Serial.print("MQTT connecting...");
  if (client.connect("ESP32BotClient")) {
    Serial.println("connected");
    const char* topics[] = {"ankit/bot/command", "ankit/bot/pid", "ankit/bot/speeds",
                            "ankit/bot/mode", "ankit/bot/manual", "ankit/bot/route"};
    for (int i = 0; i < 6; i++) client.subscribe(topics[i]);
  } else {
    Serial.println("failed, retry in 3s");
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== BOT STARTING ===");

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Motor PWM
  ledcSetup(LEFT_PWM_CH,  1000, 8);  // ch0, timer0
  ledcSetup(RIGHT_PWM_CH, 1000, 8);  // ch2, timer1
  ledcAttachPin(ENA, LEFT_PWM_CH);
  ledcAttachPin(ENB, RIGHT_PWM_CH);

  // IR sensors
  for (int i = 0; i < 5; i++) pinMode(irPins[i], INPUT);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo PWM (50Hz, 10-bit resolution for precise angle)
  ledcSetup(SERVO_CH, 50, 10);
  ledcAttachPin(SERVO_PIN, SERVO_CH);
  servoWrite(90);  // center position

  stopMotors();

  // === MOTOR SELF-TEST ===
  Serial.println("[TEST] Left forward...");
  motorLeft(120); delay(400); motorLeft(0); delay(200);

  Serial.println("[TEST] Left backward...");
  motorLeft(-120); delay(400); motorLeft(0); delay(200);

  Serial.println("[TEST] Right forward...");
  motorRight(120); delay(400); motorRight(0); delay(200);

  Serial.println("[TEST] Right backward...");
  motorRight(-120); delay(400); motorRight(0); delay(200);

  Serial.println("[TEST] Both forward...");
  setMotors(120, 120); delay(400); stopMotors(); delay(200);

  // === ULTRASONIC TEST ===
  long testDist = measureDistance();
  Serial.println("[TEST] Ultrasonic: " + String(testDist) + "cm");

  // === SERVO TEST ===
  Serial.println("[TEST] Servo sweep...");
  servoWrite(30);  delay(400);
  servoWrite(150); delay(400);
  servoWrite(90);  delay(200);
  Serial.println("[TEST] Done!");

  stopMotors();

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
    if (millis() - lastTry > 3000) { lastTry = millis(); reconnect(); }
  }
  client.loop();

  // Heartbeat
  static unsigned long lastAlive = 0;
  if (millis() - lastAlive > 500) {
    if (client.connected()) client.publish("ankit/bot/alive", "1");
    lastAlive = millis();
  }

  // === OBSTACLE CHECK (runs continuously) ===
  checkObstacle();

  // === READ SENSORS (runs continuously for dashboard) ===
  int bits[5]; int cnt, ws;
  readSensors(bits, cnt, ws);
  static unsigned long lastSens = 0;
  if (millis() - lastSens > 200 && client.connected()) {
    String s = String(bits[0])+","+String(bits[1])+","+String(bits[2])+","+String(bits[3])+","+String(bits[4]);
    client.publish("ankit/bot/sensors", s.c_str());
    lastSens = millis();
  }

  if (!isRunning) { stopMotors(); return; }
  if (driveMode == "manual") return;

  if (obstacleDetected) {
    stopMotors();
    return;  // freeze everything while obstacle is in the way
  }

  // === NAVIGATION STATE MACHINE ===
  switch (navState) {

    case NAV_FOLLOW: {
      followLine();
      // Node detection (only with active route)
      if (millis() > nodeCooldownUntil && routeQueue.length() > 0) {
        int b[5]; int cnt, ws;
        readSensors(b, cnt, ws);
        if ((b[0] || b[1]) && (b[3] || b[4])) {
          stopMotors();
          navState = NAV_NODE;
          sendLog("NODE"); publishNav("NODE_DETECTED");
        }
      }
      break;
    }

    case NAV_NODE: {
      String cmd = popRoute();
      if (cmd == "" || cmd == "X") {
        stopMotors(); isRunning = false; navState = NAV_STOP;
        sendLog("ARRIVED!"); publishNav("DESTINATION_REACHED");
      } else if (cmd == "L") {
        turnDir = 1; turnStartMs = millis(); navState = NAV_TURN;
        sendLog("TURN LEFT"); publishNav("TURNING_LEFT");
      } else if (cmd == "R") {
        turnDir = -1; turnStartMs = millis(); navState = NAV_TURN;
        sendLog("TURN RIGHT"); publishNav("TURNING_RIGHT");
      } else if (cmd == "S") {
        crossStartMs = millis(); navState = NAV_CROSS;
        sendLog("GO STRAIGHT"); publishNav("CROSSING");
      }
      break;
    }

    case NAV_TURN: {
      // Pivot turn: one wheel forward, one stopped
      if (turnDir > 0) setMotors(0, turnSpeed);
      else              setMotors(turnSpeed, 0);

      if (millis() - turnStartMs >= kTurnTimeMs) {
        stopMotors(); delay(30);
        lastError = (turnDir > 0) ? -2.0f : 2.0f;
        integral = 0; lastPidMs = millis();
        nodeCooldownUntil = millis() + kNodeCooldownMs;
        navState = NAV_FOLLOW;
        sendLog("Turn done, PID following"); publishNav("FOLLOWING");
      }
      break;
    }

    case NAV_CROSS: {
      setMotors(baseSpeed, baseSpeed);
      if (millis() - crossStartMs >= kCrossingTimeMs) {
        lastError = 0; integral = 0; lastPidMs = millis();
        nodeCooldownUntil = millis() + kNodeCooldownMs;
        navState = NAV_FOLLOW;
        sendLog("Crossed"); publishNav("FOLLOWING");
      }
      break;
    }

    case NAV_STOP: { stopMotors(); break; }
  }
}
