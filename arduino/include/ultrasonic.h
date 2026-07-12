#ifndef ULTRASONIC_H
#define ULTRASONIC_H

extern float ultrasonicDistanceCm;
extern bool  ultrasonicObjectInStopRange;
extern bool  ultrasonicSampleValid;

void ultrasonicInit();
bool ultrasonicUpdate();
bool ultrasonicShouldStop();

#endif
