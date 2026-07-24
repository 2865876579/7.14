#include <Arduino.h>

const uint8_t LEFT_SENSOR_PIN = A0;
const uint8_t MIDDLE_SENSOR_PIN = A3;
const uint8_t RIGHT_SENSOR_PIN = A2;

const unsigned long SERIAL_BAUD = 9600;
const unsigned long LIVE_PRINT_INTERVAL_MS = 250;
const uint16_t CALIBRATION_SAMPLES = 200;
const uint8_t SAMPLE_INTERVAL_MS = 5;

struct SensorValues {
  int left;
  int middle;
  int right;
};

struct CalibrationResult {
  SensorValues average;
  SensorValues minimum;
  SensorValues maximum;
  bool valid;
};

CalibrationResult whiteResult = {};
CalibrationResult blackResult = {};
unsigned long lastLivePrintMs = 0;

SensorValues readSensors() {
  SensorValues values;
  values.left = analogRead(LEFT_SENSOR_PIN);
  values.middle = analogRead(MIDDLE_SENSOR_PIN);
  values.right = analogRead(RIGHT_SENSOR_PIN);
  return values;
}

void printValues(const SensorValues &values) {
  Serial.print(F("L="));
  Serial.print(values.left);
  Serial.print(F("  M="));
  Serial.print(values.middle);
  Serial.print(F("  R="));
  Serial.print(values.right);
}

void printHelp() {
  Serial.println();
  Serial.println(F("=== Grayscale sensor calibration ==="));
  Serial.println(F("Pins: LEFT=A0, MIDDLE=A3, RIGHT=A2"));
  Serial.println(F("Place all sensors on WHITE, then send: w"));
  Serial.println(F("Place all sensors on BLACK, then send: b"));
  Serial.println(F("Other commands: p=print result, r=reset, h=help"));
  Serial.println(F("The program prints live readings every 250 ms."));
  Serial.println();
}

CalibrationResult captureCalibration(const __FlashStringHelper *surfaceName) {
  CalibrationResult result = {};
  long leftSum = 0;
  long middleSum = 0;
  long rightSum = 0;

  result.minimum = {1023, 1023, 1023};
  result.maximum = {0, 0, 0};

  Serial.print(F("Sampling "));
  Serial.print(surfaceName);
  Serial.print(F(" ("));
  Serial.print(CALIBRATION_SAMPLES);
  Serial.println(F(" samples)... Keep the car still."));

  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    SensorValues values = readSensors();
    leftSum += values.left;
    middleSum += values.middle;
    rightSum += values.right;

    result.minimum.left = min(result.minimum.left, values.left);
    result.minimum.middle = min(result.minimum.middle, values.middle);
    result.minimum.right = min(result.minimum.right, values.right);
    result.maximum.left = max(result.maximum.left, values.left);
    result.maximum.middle = max(result.maximum.middle, values.middle);
    result.maximum.right = max(result.maximum.right, values.right);
    delay(SAMPLE_INTERVAL_MS);
  }

  result.average.left = leftSum / CALIBRATION_SAMPLES;
  result.average.middle = middleSum / CALIBRATION_SAMPLES;
  result.average.right = rightSum / CALIBRATION_SAMPLES;
  result.valid = true;

  Serial.print(surfaceName);
  Serial.print(F(" average: "));
  printValues(result.average);
  Serial.println();
  Serial.print(surfaceName);
  Serial.print(F(" range:   L="));
  Serial.print(result.minimum.left);
  Serial.print('-');
  Serial.print(result.maximum.left);
  Serial.print(F("  M="));
  Serial.print(result.minimum.middle);
  Serial.print('-');
  Serial.print(result.maximum.middle);
  Serial.print(F("  R="));
  Serial.print(result.minimum.right);
  Serial.print('-');
  Serial.println(result.maximum.right);
  return result;
}

void printCalibrationResult() {
  Serial.println();
  Serial.println(F("=== Calibration result ==="));

  if (!whiteResult.valid) {
    Serial.println(F("WHITE: not sampled (send w)"));
  } else {
    Serial.print(F("WHITE average: "));
    printValues(whiteResult.average);
    Serial.println();
  }

  if (!blackResult.valid) {
    Serial.println(F("BLACK: not sampled (send b)"));
  } else {
    Serial.print(F("BLACK average: "));
    printValues(blackResult.average);
    Serial.println();
  }

  if (!whiteResult.valid || !blackResult.valid) {
    Serial.println(F("Sample both surfaces before calculating thresholds."));
    Serial.println();
    return;
  }

  SensorValues threshold;
  threshold.left = (whiteResult.average.left + blackResult.average.left) / 2;
  threshold.middle = (whiteResult.average.middle + blackResult.average.middle) / 2;
  threshold.right = (whiteResult.average.right + blackResult.average.right) / 2;

  Serial.print(F("Suggested threshold: "));
  printValues(threshold);
  Serial.println();
  Serial.println(F("Copy to arduino/include/config.h:"));
  Serial.print(F("const int LEFT_THRESHOLD   = "));
  Serial.print(threshold.left);
  Serial.println(';');
  Serial.print(F("const int MIDDLE_THRESHOLD = "));
  Serial.print(threshold.middle);
  Serial.println(';');
  Serial.print(F("const int RIGHT_THRESHOLD  = "));
  Serial.print(threshold.right);
  Serial.println(';');

  if (blackResult.average.left >= whiteResult.average.left ||
      blackResult.average.middle >= whiteResult.average.middle ||
      blackResult.average.right >= whiteResult.average.right) {
    Serial.println(F("NOTE: At least one BLACK value is not lower than WHITE."));
    Serial.println(F("Check sensor position or whether your modules use reverse output."));
  }
  Serial.println();
}

void handleCommand(char command) {
  if (command >= 'A' && command <= 'Z') command += 'a' - 'A';

  switch (command) {
    case 'w':
      whiteResult = captureCalibration(F("WHITE"));
      printCalibrationResult();
      break;
    case 'b':
      blackResult = captureCalibration(F("BLACK"));
      printCalibrationResult();
      break;
    case 'p':
      printCalibrationResult();
      break;
    case 'r':
      whiteResult.valid = false;
      blackResult.valid = false;
      Serial.println(F("Calibration data reset."));
      break;
    case 'h':
    case '?':
      printHelp();
      break;
    case '\r':
    case '\n':
    case ' ':
    case '\t':
      break;
    default:
      Serial.print(F("Unknown command: "));
      Serial.println(command);
      printHelp();
      break;
  }
}

void setup() {
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(MIDDLE_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);
  Serial.begin(SERIAL_BAUD);
  delay(500);
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    handleCommand(Serial.read());
  }

  unsigned long now = millis();
  if (now - lastLivePrintMs >= LIVE_PRINT_INTERVAL_MS) {
    lastLivePrintMs = now;
    Serial.print(F("LIVE  "));
    printValues(readSensors());
    Serial.println();
  }
}
