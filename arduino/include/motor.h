#ifndef MOTOR_H
#define MOTOR_H

extern int currentLeftSpeed;
extern int currentRightSpeed;

void motorInit();
void forward();
void turnLeft();
void turnRight();
void stopCar();
void setMotor(int leftSpeed, int rightSpeed);

#endif
