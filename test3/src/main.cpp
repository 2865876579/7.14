#include <Arduino.h>

// Same motor pins as the main Arduino project.
const uint8_t LEFT_FORWARD_PIN = 5;
const uint8_t LEFT_BACKWARD_PIN = 6;
const uint8_t RIGHT_FORWARD_PIN = 9;
const uint8_t RIGHT_BACKWARD_PIN = 10;

const uint8_t TEST_PWM = 150;
const unsigned long START_DELAY_MS = 2000;

void stopAllMotors() {
  analogWrite(LEFT_FORWARD_PIN, 0);
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_FORWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
}

void runAllMotorsForward() {
  analogWrite(LEFT_BACKWARD_PIN, 0);
  analogWrite(RIGHT_BACKWARD_PIN, 0);
  analogWrite(LEFT_FORWARD_PIN, TEST_PWM);
  analogWrite(RIGHT_FORWARD_PIN, TEST_PWM);
}

void setup() {
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);

  stopAllMotors();
  Serial.begin(9600);
  Serial.println(F("Motor forward test starts in 2 seconds."));
  Serial.println(F("Send s to stop, f to run forward."));
  delay(START_DELAY_MS);

  runAllMotorsForward();
  Serial.println(F("All motors: FORWARD, PWM=150"));
}

void loop() {
  while (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 's' || command == 'S') {
      stopAllMotors();
      Serial.println(F("All motors: STOP"));
    } else if (command == 'f' || command == 'F') {
      runAllMotorsForward();
      Serial.println(F("All motors: FORWARD, PWM=150"));
    }
  }
}
