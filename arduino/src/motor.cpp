#include <Arduino.h>
#include "config.h"
#include "motor.h"

int currentLeftSpeed  = 0;
int currentRightSpeed = 0;

static unsigned long leftReverseKickEndTime  = 0;
static unsigned long rightReverseKickEndTime = 0;

static void setOneMotor(int fwdPin, int bwdPin, int spd, int prev, unsigned long* kickEnd) {
  spd = constrain(spd, -255, 255);
  int pwm = abs(spd);
  if (spd > 0) {
    *kickEnd = 0;
    analogWrite(fwdPin, pwm);
    analogWrite(bwdPin, 0);
  } else if (spd < 0) {
    if (prev >= 0) *kickEnd = millis() + MOTOR_REVERSE_KICK_MS;
    if ((long)(*kickEnd - millis()) > 0) pwm = MOTOR_REVERSE_KICK_PWM;
    else if (pwm < MOTOR_REVERSE_MIN_PWM) pwm = MOTOR_REVERSE_MIN_PWM;
    pwm = constrain(pwm, 0, 255);
    analogWrite(fwdPin, 0);
    analogWrite(bwdPin, pwm);
  } else {
    *kickEnd = 0;
    analogWrite(fwdPin, 0);
    analogWrite(bwdPin, 0);
  }
}

void motorInit() {
  pinMode(LEFT_RED_PIN,    OUTPUT);
  pinMode(LEFT_BLACK_PIN,  OUTPUT);
  pinMode(RIGHT_RED_PIN,   OUTPUT);
  pinMode(RIGHT_BLACK_PIN, OUTPUT);
  stopCar();
}

void forward()  { setMotor(FORWARD_SPEED, FORWARD_SPEED); }
void turnLeft()  { setMotor(TURN_INNER_SPEED, TURN_OUTER_SPEED); }
void turnRight() { setMotor(TURN_OUTER_SPEED, TURN_INNER_SPEED); }
void stopCar()   { setMotor(0, 0); }

void setMotor(int l, int r) {
  int pl = currentLeftSpeed, pr = currentRightSpeed;
  currentLeftSpeed  = constrain(l, -255, 255);
  currentRightSpeed = constrain(r, -255, 255);
  setOneMotor(LEFT_RED_PIN,  LEFT_BLACK_PIN,  currentLeftSpeed,  pl, &leftReverseKickEndTime);
  setOneMotor(RIGHT_RED_PIN, RIGHT_BLACK_PIN, currentRightSpeed, pr, &rightReverseKickEndTime);
}
