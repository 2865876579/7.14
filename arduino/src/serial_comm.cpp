#include <Arduino.h>
#include "serial_comm.h"

static int     detectedShape = SHAPE_NONE;
static bool    shapeReceived = false;
static char    recvBuf[32];
static uint8_t recvLen = 0;

void serialCommInit() {
  Serial.begin(9600);
}

void serialCommUpdate() {
  if (shapeReceived) return;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (recvLen > 0) {
        recvBuf[recvLen] = '\0';
        recvLen = 0;
        if (strcmp(recvBuf, "triangle") == 0) {
          detectedShape = SHAPE_TRIANGLE;
          shapeReceived = true;
        } else if (strcmp(recvBuf, "quadrilateral") == 0) {
          detectedShape = SHAPE_QUADRILATERAL;
          shapeReceived = true;
        } else if (strcmp(recvBuf, "ellipse") == 0) {
          detectedShape = SHAPE_ELLIPSE;
          shapeReceived = true;
        }
      }
    } else {
      if (recvLen < (uint8_t)(sizeof(recvBuf) - 1)) {
        recvBuf[recvLen++] = c;
      } else {
        recvLen = 0;
      }
    }
  }
}

void serialCommClearInput() {
  while (Serial.available() > 0) {
    Serial.read();
  }
  recvLen = 0;
}

int  getDetectedShape() { return detectedShape; }
bool hasDetectedShape() { return shapeReceived; }

void clearDetectedShape() {
  detectedShape = SHAPE_NONE;
  shapeReceived = false;
  recvLen       = 0;
  serialCommClearInput();
}
