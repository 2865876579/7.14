#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <ServoTimer2.h>

#define SERVO_1 1
#define SERVO_2 2
#define SERVO_3 3

#define ARM_MODE_NONE   -1
#define ARM_MODE_DETECT  0
#define ARM_MODE_GRAB    1
#define ARM_MODE_PLACE   2
#define ARM_MODE_HOLD    3

void servoInit();
void servoSetAngle(int num, int angle);
void servoCenter(int num);
void servo1Detach();
void servo1Restore();
void servo2Detach();
void servo2Restore();
void servo3Detach();
void servo3Restore();

void armUpdate();
void armRequestMode(int mode);
bool armIsBusy();
int armGetMode();

#endif
