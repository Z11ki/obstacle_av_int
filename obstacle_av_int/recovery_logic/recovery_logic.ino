/*
 * Obstacle Avoiding Robot with Stuck Detection
 * Architecture: Finite State Machine + Sliding Time Window
 * Author: Zekariyas Amberbir
  */

// --
// INCLUDES
// --
#include <Arduino.h>

// --
// PIN CONFIG
// --
// Ultrasonic Sensor (HC-SR04)
const uint8_t PIN_TRIG = 9;
const uint8_t PIN_ECHO = 10;

// Motor Driver (L298N example)
const uint8_t PIN_IN1 = 2;
const uint8_t PIN_IN2 = 3;
const uint8_t PIN_IN3 = 4;
const uint8_t PIN_ENB = 5;
const uint8_t PIN_ENA = 6;  // PWM
const uint8_t PIN_IN4 = 7;  // PWM

// Optional LEDs
const uint8_t PIN_LED_GREEN = 11;
const uint8_t PIN_LED_RED   = 12;

// --
// TUNABLE CONSTANTS
// --
const float DIST_THRESHOLD_CM = 20.0;

const unsigned long REVERSE_TIME_MS = 1000;
const unsigned long TURN_TIME_MS    = 700;
const unsigned long LOOP_DELAY_MS   = 50;

// Stuck detection
const uint8_t MAX_TURNS_WINDOW = 5;
const unsigned long WINDOW_TIME_MS = 10000;

// Motor speed (0–255)
const uint8_t MOTOR_SPEED = 180;

// --
// STATE MACHINE
// --
enum RobotState {
  STATE_FORWARD,
  STATE_AVOID,
  STATE_RECOVERY
};

RobotState currentState = STATE_FORWARD;

// --
// TURN TRACKING (Sliding Window)
// --
const uint8_t MAX_EVENTS = 20; // buffer size (safe upper bound)
unsigned long turnTimestamps[MAX_EVENTS];
uint8_t turnCount = 0;

// --
// FUNCTION DECLARATIONS
// --
float readDistanceCM();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
void setMotorSpeed(uint8_t speed);

void recordTurn(unsigned long now);
void pruneOldTurns(unsigned long now);
bool isStuck();

void recoveryBehavior();

// --
// SETUP
// --
void setup() {
  Serial.begin(9600);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);

  setMotorSpeed(MOTOR_SPEED);
}

//--
// MAIN LOOP
//--
void loop() {
  unsigned long now = millis();
  float distance = readDistanceCM();

  Serial.print("Distance: ");
  Serial.println(distance);

  switch (currentState) {

    case STATE_FORWARD:
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_RED, LOW);

      if (distance >= DIST_THRESHOLD_CM) {
        moveForward();
      } else {
        currentState = STATE_AVOID;
      }
      break;

    case STATE_AVOID:
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, HIGH);

      stopMotors();

      moveBackward();
      delay(REVERSE_TIME_MS);

      // Random turn direction
      if (random(0, 2) == 0) {
        turnLeft();
      } else {
        turnRight();
      }
      delay(TURN_TIME_MS);

      stopMotors();

      // ---- Record turn event ----
      recordTurn(now);
      pruneOldTurns(now);

      if (isStuck()) {
        currentState = STATE_RECOVERY;
      } else {
        currentState = STATE_FORWARD;
      }
      break;

    case STATE_RECOVERY:
      Serial.println("RECOVERY MODE");
      recoveryBehavior();

      // Reset system
      turnCount = 0;

      currentState = STATE_FORWARD;
      break;
  }

  delay(LOOP_DELAY_MS);
}

//--
// SENSOR FUNCTION
//--
float readDistanceCM() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 30000); // timeout 30ms

  if (duration == 0) {
    return 999.0; // no echo → treat as far
  }

  float distance = duration * 0.0343 / 2.0;
  return distance;
}

//--
// MOTOR CONTROL
//--
void setMotorSpeed(uint8_t speed) {
  analogWrite(PIN_ENA, speed);
  analogWrite(PIN_ENB, speed);
}

void moveForward() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
}

void moveBackward() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
}

void turnLeft() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
}

void turnRight() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
}

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

//--
// TURN TRACKING LOGIC
//--
void recordTurn(unsigned long now) {
  if (turnCount < MAX_EVENTS) {
    turnTimestamps[turnCount++] = now;
  }
}

void pruneOldTurns(unsigned long now) {
  uint8_t i = 0;

  while (i < turnCount) {
    if ((now - turnTimestamps[i]) > WINDOW_TIME_MS) {
      // Shift left
      for (uint8_t j = i; j < turnCount - 1; j++) {
        turnTimestamps[j] = turnTimestamps[j + 1];
      }
      turnCount--;
    } else {
      i++;
    }
  }
}

bool isStuck() {
  return (turnCount > MAX_TURNS_WINDOW);
}

//--
// RECOVERY BEHAVIOR
//--
void recoveryBehavior() {
  stopMotors();

  moveBackward();
  delay(2 * REVERSE_TIME_MS);

  turnLeft();
  delay(2 * TURN_TIME_MS);

  moveForward();
  delay(1000);

  stopMotors();
}