#include <Arduino.h>
#include "config.h"
#include "ultrasonic.h"

float ultrasonicDistanceCm      = 0.0f;
bool  ultrasonicObjectInStopRange = false;

static unsigned long lastSampleTime = 0;
static unsigned long lastPrintTime  = 0;

static void debugPrint() {
  if (!ULTRASONIC_SERIAL_DEBUG_ENABLED) return;
  unsigned long now = millis();
  if (now - lastPrintTime < ULTRASONIC_SERIAL_PRINT_INTERVAL_MS) return;
  lastPrintTime = now;
  Serial.print("ultrasonic_cm=");
  Serial.print(ultrasonicDistanceCm, 1);
  Serial.print(", stop_range=");
  Serial.println(ultrasonicObjectInStopRange ? 1 : 0);
}

void ultrasonicInit() {
  if (!ULTRASONIC_ENABLED) return;
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

void ultrasonicUpdate() {
  if (!ULTRASONIC_ENABLED) {
    ultrasonicDistanceCm      = 0.0f;
    ultrasonicObjectInStopRange = false;
    debugPrint();
    return;
  }
  unsigned long now = millis();
  if (now - lastSampleTime < ULTRASONIC_SAMPLE_INTERVAL_MS) {
    debugPrint();
    return;
  }
  lastSampleTime = now;

  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long echo = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_ECHO_TIMEOUT_US);
  if (echo == 0) {
    ultrasonicDistanceCm      = 0.0f;
    ultrasonicObjectInStopRange = false;
    debugPrint();
    return;
  }
  ultrasonicDistanceCm = echo / 58.0f;
  ultrasonicObjectInStopRange =
      ultrasonicDistanceCm >= ULTRASONIC_STOP_MIN_CM &&
      ultrasonicDistanceCm <= ULTRASONIC_STOP_MAX_CM;
  debugPrint();
}

bool ultrasonicShouldStop() {
  return ULTRASONIC_ENABLED && ultrasonicObjectInStopRange;
}
