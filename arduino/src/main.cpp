#include <Arduino.h>
#include "config.h"
#include "control.h"
#include "motor.h"
#include "sensor.h"
#include "servo_control.h"
#include "ultrasonic.h"
#include "serial_comm.h"

void setup() {
  serialCommInit();
  if (SERVO_ENABLED) servoInit();
  motorInit();
  sensorInit();
  ultrasonicInit();
}

void loop() {
  readSensors();
  runLineFollow();
  delay(CONTROL_DELAY_MS);
}
