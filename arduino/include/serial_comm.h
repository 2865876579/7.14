#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#define SHAPE_NONE          0
#define SHAPE_TRIANGLE      1
#define SHAPE_QUADRILATERAL 2
#define SHAPE_ELLIPSE       3

void serialCommInit();
void serialCommUpdate();
void serialCommClearInput();

int getDetectedShape();
bool hasDetectedShape();
void clearDetectedShape();

#endif
