#include "servo_control.h"
#include "config.h"

static ServoTimer2 servo1;
static ServoTimer2 servo2;
static ServoTimer2 servo3;
static bool servo1Attached = false;
static bool servo2Attached = false;
static bool servo3Attached = false;
static int servo1LastAngle = ARM_DETECT_S1;

static int armCurrentMode = ARM_MODE_DETECT;
static int armNextMode = ARM_MODE_DETECT;
static int armStep = 0;
static int armTargetN[3];
static int armTargetS[3];
static unsigned long armStepTime = 0;
static unsigned long armDoneTime = 0;

static int angleToUs(int angle) {
  return map(constrain(angle, 0, 270), 0, 270, 750, 2250);
}

static void writeAngle(int num, int angle) {
  if (num == SERVO_1 && servo1Attached) {
    servo1LastAngle = angle;
    servo1.write(angleToUs(angle));
  }
  if (num == SERVO_2 && servo2Attached) {
    servo2.write(angleToUs(angle));
  }
  if (num == SERVO_3 && servo3Attached) {
    servo3.write(angleToUs(angle));
  }
}

void servoInit() {
  servo1.attach(SERVO1_PIN);
  servo1Attached = true;
  servo2.attach(SERVO2_PIN);
  servo2Attached = true;
  servo3.attach(SERVO3_PIN);
  servo3Attached = true;

  writeAngle(SERVO_1, ARM_DETECT_S1);
  writeAngle(SERVO_2, ARM_DETECT_S2);
  writeAngle(SERVO_3, ARM_DETECT_S3);

  armCurrentMode = ARM_MODE_DETECT;
  armNextMode = ARM_MODE_DETECT;
  armStep = 0;
}

void servoSetAngle(int num, int angle) {
  writeAngle(num, angle);
}

void servoCenter(int num) {
  writeAngle(num, 135);
}

void servo1Detach() {
  if (servo1Attached) {
    servo1.detach();
    servo1Attached = false;
  }
}

void servo1Restore() {
  if (!servo1Attached) {
    servo1.attach(SERVO1_PIN);
    servo1Attached = true;
    servo1.write(angleToUs(servo1LastAngle));
  }
}

void servo2Detach() {
  if (servo2Attached) {
    servo2.detach();
    servo2Attached = false;
  }
}

void servo2Restore() {
  if (!servo2Attached) {
    servo2.attach(SERVO2_PIN);
    servo2Attached = true;
  }
}

void servo3Detach() {
  if (servo3Attached) {
    servo3.detach();
    servo3Attached = false;
  }
}

void servo3Restore() {
  if (!servo3Attached) {
    servo3.attach(SERVO3_PIN);
    servo3Attached = true;
  }
}

void armRequestMode(int mode) {
  if (mode == armCurrentMode && armStep == 0) return;

  armNextMode = mode;

  if (mode == ARM_MODE_GRAB) {
    armTargetN[0] = SERVO_1; armTargetS[0] = ARM_GRAB_S1;
    armTargetN[1] = SERVO_2; armTargetS[1] = ARM_GRAB_S2;
    armTargetN[2] = SERVO_3; armTargetS[2] = ARM_GRAB_S3;
  } else if (mode == ARM_MODE_PLACE) {
    armTargetN[0] = SERVO_2; armTargetS[0] = ARM_PLACE_S2;
    armTargetN[1] = SERVO_1; armTargetS[1] = ARM_PLACE_S1;
    armTargetN[2] = SERVO_3; armTargetS[2] = ARM_PLACE_S3;
  } else if (mode == ARM_MODE_HOLD) {
    armTargetN[0] = SERVO_1; armTargetS[0] = ARM_HOLD_S1;
    armTargetN[1] = SERVO_2; armTargetS[1] = ARM_HOLD_S2;
    armTargetN[2] = SERVO_3; armTargetS[2] = ARM_HOLD_S3;
  } else {
    armTargetN[0] = SERVO_1; armTargetS[0] = ARM_DETECT_S1;
    armTargetN[1] = SERVO_2; armTargetS[1] = ARM_DETECT_S2;
    armTargetN[2] = SERVO_3; armTargetS[2] = ARM_DETECT_S3;
  }

  writeAngle(armTargetN[0], armTargetS[0]);
  armStep = 1;
  armStepTime = millis();
}

bool armIsBusy() {
  return armStep != 0;
}

int armGetMode() {
  return armCurrentMode;
}

void armUpdate() {
  if (armStep == 0) return;

  if (armStep == 4) {
    if (millis() - armDoneTime >= ARM_DONE_HOLD_MS) {
      armCurrentMode = armNextMode;
      armStep = 0;
    }
    return;
  }

  bool isGrab = (armNextMode == ARM_MODE_GRAB);
  bool isPlace = (armNextMode == ARM_MODE_PLACE);
  bool isHold = (armNextMode == ARM_MODE_HOLD);
  unsigned long waitMs = ARM_STEP_INTERVAL_MS;
  if (isGrab && armStep == 1) waitMs = ARM_GRAB_S2_DELAY_MS;
  if (isGrab && armStep == 2) waitMs = ARM_GRAB_S3_DELAY_MS;

  if (millis() - armStepTime < waitMs) return;
  armStepTime = millis();

  if (armStep == 1) {
    writeAngle(armTargetN[1], armTargetS[1]);
    if (isHold) {
      // PLACE 和 HOLD 的舵机3目标相同，跳过无实际动作的重复写入。
      armStep = 4;
      armDoneTime = millis();
    } else {
      armStep = 2;
    }
  } else if (armStep == 2) {
    writeAngle(armTargetN[2], armTargetS[2]);
    if (isPlace) {
      // 放置到位后直接进入完成保持，去掉纯状态切换的额外等待。
      armStep = 4;
      armDoneTime = millis();
    } else {
      armStep = 3;
    }
  } else if (armStep == 3) {
    armStep = 4;
    armDoneTime = millis();
  }
}
