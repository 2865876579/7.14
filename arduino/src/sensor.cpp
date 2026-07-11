#include <Arduino.h>
#include "config.h"
#include "sensor.h"

int leftRaw    = 0;
int middleRaw  = 0;
int rightRaw   = 0;
int leftBlack  = 0;
int middleBlack = 0;
int rightBlack = 0;

void sensorInit() {
  pinMode(LEFT_SENSOR_PIN,   INPUT);
  pinMode(MIDDLE_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN,  INPUT);
}

void readSensors() {
  leftRaw   = analogRead(LEFT_SENSOR_PIN);
  middleRaw = analogRead(MIDDLE_SENSOR_PIN);
  rightRaw  = analogRead(RIGHT_SENSOR_PIN);
  leftBlack   = (leftRaw   < LEFT_THRESHOLD)   ? 1 : 0;
  middleBlack = (middleRaw < MIDDLE_THRESHOLD) ? 1 : 0;
  rightBlack  = (rightRaw  < RIGHT_THRESHOLD)  ? 1 : 0;
}
