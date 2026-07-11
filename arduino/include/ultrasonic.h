#ifndef ULTRASONIC_H
#define ULTRASONIC_H

extern float ultrasonicDistanceCm;
extern bool  ultrasonicObjectInStopRange;

void ultrasonicInit();
void ultrasonicUpdate();
bool ultrasonicShouldStop();

#endif
