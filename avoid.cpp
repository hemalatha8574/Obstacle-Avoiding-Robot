/*
  Project: Obstacle Avoidance Robot
  Features:
  - Ultrasonic distance (HC-SR04) with timeout and median filtering
  - L298N dual motor driver with PWM speed control
  - Servo-mounted sensor scans left/center/right to pick best path
  - FSM: FORWARD -> AVOID -> TURN -> RECOVER
  - Stuck detection (no progress -> back up and retry)
*/

#include <Servo.h>

// ------------------ USER CONFIG ------------------
// Motor pins (L298N)
const uint8_t ENA = 5;  // PWM (left)
const uint8_t IN1 = 8;
const uint8_t IN2 = 9;

const uint8_t ENB = 6;  // PWM (right)
const uint8_t IN3 = 10;
const uint8_t IN4 = 11;

// Ultrasonic
const uint8_t TRIG = 2;
const uint8_t ECHO = 3;

// Servo (mounting HC-SR04)
const uint8_t SERVO_PIN = 12;

// Behavior
const uint8_t SPEED_FWD = 180;   // 0..255
const uint8_t SPEED_TURN = 170;
const uint8_t SPEED_BACK = 160;

const uint16_t SAFE_DIST_CM = 22;
const uint16_t PAN_LEFT = 140;
const uint16_t PAN_CENTER = 90;
const uint16_t PAN_RIGHT = 40;

const uint16_t TURN_TIME_MS = 450;
const uint16_t BACK_TIME_MS = 350;

const uint32_t SCAN_INTERVAL_MS = 400;
const uint8_t MEDIAN_N = 5;
const unsigned long ECHO_TIMEOUT_US = 25000; // ~4.3m

// -------------------------------------------------

Servo servoHead;

void motorStop() { analogWrite(ENA,0); analogWrite(ENB,0); }
void motorFwd(uint8_t sp) {
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  analogWrite(ENA, sp); analogWrite(ENB, sp);
}
void motorBack(uint8_t sp) {
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
  analogWrite(ENA, sp); analogWrite(ENB, sp);
}
void motorTurnLeft(uint8_t sp) {
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);   // left backward
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);   // right forward
  analogWrite(ENA, sp); analogWrite(ENB, sp);
}
void motorTurnRight(uint8_t sp) {
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
  analogWrite(ENA, sp); analogWrite(ENB, sp);
}

unsigned long measureEcho() {
  digitalWrite(TRIG, LOW); delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, ECHO_TIMEOUT_US); // microseconds
}

uint16_t distCM() {
  // median of N samples
  unsigned long arr[MEDIAN_N];
  for (uint8_t i=0;i<MEDIAN_N;i++) {
    arr[i] = measureEcho();
    delay(5);
  }
  // sort small N
  for (uint8_t i=0;i<MEDIAN_N;i++)
    for (uint8_t j=i+1;j<MEDIAN_N;j++)
      if (arr[j]<arr[i]) { auto t=arr[i]; arr[i]=arr[j]; arr[j]=t; }

  unsigned long us = arr[MEDIAN_N/2];
  if (us==0) return 999;
  // speed of sound: ~0.0343 cm/us, divide by 2 for round trip
  return (uint16_t)(us * 0.0343 / 2.0);
}

uint16_t scanAt(uint8_t angle) {
  servoHead.write(angle);
  delay(180); // settle
  return distCM();
}

enum Mode { MD_FORWARD, MD_AVOID, MD_TURN, MD_RECOVER };
Mode mode = MD_FORWARD;

uint32_t tScan=0, tModeStart=0;
int turnDir = 0; // -1 left, +1 right

void setup() {
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);

  servoHead.attach(SERVO_PIN);
  servoHead.write(PAN_CENTER);

  Serial.begin(9600);
  Serial.println(F("Robot boot"));
  tModeStart = millis();
}

void loop() {
  uint32_t now = millis();

  switch (mode) {
    case MD_FORWARD: {
      motorFwd(SPEED_FWD);
      // periodic center check
      if (now - tScan >= SCAN_INTERVAL_MS) {
        tScan = now;
        servoHead.write(PAN_CENTER);
        uint16_t d = distCM();
        Serial.print(F("FWD dist=")); Serial.println(d);
        if (d < SAFE_DIST_CM) {
          mode = MD_AVOID; tModeStart = now; motorStop();
        }
      }
    } break;

    case MD_AVOID: {
      // look left/right and choose
      uint16_t dl = scanAt(PAN_LEFT);
      uint16_t dr = scanAt(PAN_RIGHT);
      Serial.print(F("Scan L/R=")); Serial.print(dl); Serial.print('/'); Serial.println(dr);
      if (dl >= dr) { turnDir = -1; } else { turnDir = +1; }
      mode = MD_TURN; tModeStart = now;
    } break;

    case MD_TURN: {
      if (turnDir < 0) motorTurnLeft(SPEED_TURN); else motorTurnRight(SPEED_TURN);
      if (now - tModeStart >= TURN_TIME_MS) {
        mode = MD_FORWARD; tModeStart = now;
        servoHead.write(PAN_CENTER);
      }
    } break;

    case MD_RECOVER: {
      motorBack(SPEED_BACK);
      if (now - tModeStart >= BACK_TIME_MS) {
        mode = MD_FORWARD; tModeStart = now;
      }
    } break;
  }

  // Stuck detection: if in FORWARD but repeatedly see too-close, back off
  static uint8_t blockedCount=0;
  if (mode == MD_FORWARD && now - tScan >= SCAN_INTERVAL_MS) {
    // already scanned above, but for safety:
    tScan = now;
    servoHead.write(PAN_CENTER);
    uint16_t d = distCM();
    if (d < SAFE_DIST_CM) {
      blockedCount++;
      if (blockedCount >= 3) {
        mode = MD_RECOVER; tModeStart = now; blockedCount = 0; Serial.println(F("Recovering"));
      }
    } else {
      if (blockedCount>0) blockedCount--;
    }
  }
}
