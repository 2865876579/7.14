#ifndef SENSOR_H
#define SENSOR_H

extern int leftRaw;
extern int middleRaw;
extern int rightRaw;
extern int leftBlack;
extern int middleBlack;
extern int rightBlack;

void sensorInit();
void readSensors();

#endif
