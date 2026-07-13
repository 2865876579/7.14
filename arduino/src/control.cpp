#include <Arduino.h>
#include "config.h"
#include "control.h"
#include "motor.h"
#include "sensor.h"
#include "servo_control.h"
#include "ultrasonic.h"
#include "serial_comm.h"

int currentAction = ACTION_FORWARD_NO_LINE;
int lineState = LINE_STATE_NONE;
int currentMode = MODE_FOLLOW;
unsigned long gridModeStartTime = 0;

static bool cycleInitialized = false;
static bool gridUsedThisCycle = false;
static bool finalLeftSearchCompleted = false;
static bool startupLineFound = false;
static int lastTurnAction = ACTION_FORWARD;
static unsigned long cycleStartTime = 0;
static unsigned long lastGridExitTime = 0;
static unsigned long backwardClearLineTime = 0;
static bool gridJustExited = false;

enum UltrasonicCountState : uint8_t {
  ULTRASONIC_COUNT_ARMED,
  ULTRASONIC_COUNT_LOCKED
};

static UltrasonicCountState ultrasonicCountState = ULTRASONIC_COUNT_ARMED;
static uint8_t ultrasonicEnterConfirmCount = 0;
static uint8_t ultrasonicClearConfirmCount = 0;
static uint8_t ultrasonicNoEchoCount = 0;
static uint8_t ultrasonicDetectionCount = 0;
static bool ultrasonicOnceStopDone = false;
static bool ultrasonicOnceStopActive = false;
static unsigned long ultrasonicOnceStopStart = 0;
static const unsigned long ULTRASONIC_ONCE_STOP_MS = 1500;

static bool armGrabRequested = false;
static bool armGrabDone = false;
static bool armPlaceRequested = false;
static bool armPlaceDone = false;
static bool armRestoreRequested = false;

// 抓取完成后摆头找线
static unsigned long grabDoneTime = 0;     // 抓取完成时刻（0=未完成）
static bool grabWiggleRight = true;        // 当前摆头方向
static unsigned long grabWiggleSwitchTime = 0; // 上次切换方向的时刻
static bool grabWiggleStarted = false;
static bool grabWiggleFirstLeg = true;

static unsigned long serialBlockUntilMs = 0;

// 慢速模式丢线计时：用于区分”终点全黑倒车”与”普通直角弯”
static unsigned long slowLineLostStartTime = 0;
static bool slowLongLossPending = false;
static bool slowRightRecoveryActive = false;
static unsigned long slowRightRecoveryStartTime = 0;

static void enterFollowMode(bool slowMode);
static void runFollowLikeMode();

static float clampF(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static float absF(float v) {
  return v < 0.0f ? -v : v;
}

static float lineStrength(int raw, int thresh) {
  return clampF((thresh + LINE_RAW_SIGNAL_MARGIN - raw) / LINE_RAW_SIGNAL_MARGIN, 0.0f, 1.0f);
}

static bool shouldStopThisDetection() {
  int shape = getDetectedShape();
  if (shape == SHAPE_TRIANGLE && ultrasonicDetectionCount == 1) return true;
  if (shape == SHAPE_ELLIPSE && ultrasonicDetectionCount == 2) return true;
  if (shape == SHAPE_QUADRILATERAL && ultrasonicDetectionCount == 3) return true;
  return false;
}

static bool stopForUltrasonicObstacle() {
  bool hasNewSample = ultrasonicUpdate();

  if (ultrasonicOnceStopActive) {
    if (millis() - ultrasonicOnceStopStart < ULTRASONIC_ONCE_STOP_MS) {
      currentAction = ACTION_STOP_AT_TARGET;
      stopCar();
      return true;
    }

    ultrasonicOnceStopActive = false;
    ultrasonicOnceStopDone = true;
    if (!armPlaceRequested) {
      armPlaceRequested = true;
      armRequestMode(ARM_MODE_PLACE);
    }
    return false;
  }

  if (!hasNewSample) return false;

  if (ultrasonicCountState == ULTRASONIC_COUNT_LOCKED) {
    if (!ultrasonicSampleValid) {
      ultrasonicClearConfirmCount = 0;
      if (ultrasonicNoEchoCount < ULTRASONIC_NO_ECHO_CLEAR_SAMPLES) {
        ultrasonicNoEchoCount++;
      }
      if (ultrasonicNoEchoCount >= ULTRASONIC_NO_ECHO_CLEAR_SAMPLES) {
        ultrasonicCountState = ULTRASONIC_COUNT_ARMED;
        ultrasonicNoEchoCount = 0;
      }
      return false;
    }

    ultrasonicNoEchoCount = 0;
    if (ultrasonicDistanceCm > ULTRASONIC_CLEAR_CM) {
      if (ultrasonicClearConfirmCount < ULTRASONIC_CLEAR_CONFIRM_SAMPLES) {
        ultrasonicClearConfirmCount++;
      }
      if (ultrasonicClearConfirmCount >= ULTRASONIC_CLEAR_CONFIRM_SAMPLES) {
        ultrasonicCountState = ULTRASONIC_COUNT_ARMED;
        ultrasonicClearConfirmCount = 0;
      }
    } else {
      // 小于 12cm、仍在触发范围内或处于迟滞区时都保持锁定。
      ultrasonicClearConfirmCount = 0;
    }
    return false;
  }

  if (!ultrasonicSampleValid || !ultrasonicShouldStop()) {
    ultrasonicEnterConfirmCount = 0;
    return false;
  }

  if (ultrasonicEnterConfirmCount < ULTRASONIC_ENTER_CONFIRM_SAMPLES) {
    ultrasonicEnterConfirmCount++;
  }
  if (ultrasonicEnterConfirmCount < ULTRASONIC_ENTER_CONFIRM_SAMPLES) return false;

  ultrasonicEnterConfirmCount = 0;
  ultrasonicClearConfirmCount = 0;
  ultrasonicNoEchoCount = 0;
  ultrasonicCountState = ULTRASONIC_COUNT_LOCKED;
  if (ultrasonicDetectionCount < 3) ultrasonicDetectionCount++;

  if (!ultrasonicOnceStopDone && shouldStopThisDetection()) {
    ultrasonicOnceStopActive = true;
    ultrasonicOnceStopStart = millis();
    currentAction = ACTION_STOP_AT_TARGET;
    stopCar();
    return true;
  }
  return false;
}

int getLineState() {
  return leftBlack * 100 + middleBlack * 10 + rightBlack;
}

float calculateLineError() {
  float ls = lineStrength(leftRaw, LEFT_THRESHOLD);
  float ms = lineStrength(middleRaw, MIDDLE_THRESHOLD);
  float rs = lineStrength(rightRaw, RIGHT_THRESHOLD);
  float total = ls + rs + ms * LINE_CENTER_STRENGTH_WEIGHT;
  if (total <= 0.001f) return 0.0f;
  float bias = gridJustExited ? LINE_ERROR_BIAS : 0.0f;
  return ((-ls + rs) / total * LINE_ERROR_MAX) + bias;
}

static void driveStraight() {
  currentAction = ACTION_FORWARD;
  int speed = currentMode == MODE_SLOW ? SLOW_FORWARD_SPEED : NORMAL_FORWARD_SPEED;
  setMotor(speed, speed);
}

static void driveNoLine() {
  currentAction = ACTION_FORWARD_NO_LINE;
  setMotor(NORMAL_FORWARD_SPEED, NORMAL_FORWARD_SPEED);
}

static void driveSoftL() {
  currentAction = ACTION_TURN_LEFT;
  lastTurnAction = ACTION_TURN_LEFT;
  if (currentMode == MODE_SLOW) {
    setMotor(SLOW_SOFT_TURN_INNER_SPEED, SLOW_SOFT_TURN_OUTER_SPEED);
  } else {
    setMotor(NORMAL_SOFT_TURN_INNER_SPEED, NORMAL_SOFT_TURN_OUTER_SPEED);
  }
}

static void driveSoftR() {
  currentAction = ACTION_TURN_RIGHT;
  lastTurnAction = ACTION_TURN_RIGHT;
  if (currentMode == MODE_SLOW) {
    setMotor(SLOW_SOFT_TURN_OUTER_SPEED, SLOW_SOFT_TURN_INNER_SPEED);
  } else {
    setMotor(NORMAL_SOFT_TURN_OUTER_SPEED, NORMAL_SOFT_TURN_INNER_SPEED);
  }
}

static void driveMiddleL() {
  currentAction = ACTION_TURN_LEFT;
  lastTurnAction = ACTION_TURN_LEFT;
  if (currentMode == MODE_SLOW) {
    setMotor(SLOW_MIDDLE_TURN_INNER_SPEED, SLOW_MIDDLE_TURN_OUTER_SPEED);
  } else {
    setMotor(NORMAL_MIDDLE_TURN_INNER_SPEED, NORMAL_MIDDLE_TURN_OUTER_SPEED);
  }
}

static void driveMiddleR() {
  currentAction = ACTION_TURN_RIGHT;
  lastTurnAction = ACTION_TURN_RIGHT;
  if (currentMode == MODE_SLOW) {
    setMotor(SLOW_MIDDLE_TURN_OUTER_SPEED, SLOW_MIDDLE_TURN_INNER_SPEED);
  } else {
    setMotor(NORMAL_MIDDLE_TURN_OUTER_SPEED, NORMAL_MIDDLE_TURN_INNER_SPEED);
  }
}

static void driveLostL() {
  currentAction = ACTION_SEARCH_LEFT;
  lastTurnAction = ACTION_TURN_LEFT;
  setMotor(-LOST_TURN_REVERSE_SPEED, LOST_TURN_FORWARD_SPEED);
}

static void driveLostR() {
  currentAction = ACTION_SEARCH_RIGHT;
  lastTurnAction = ACTION_TURN_RIGHT;
  setMotor(LOST_TURN_FORWARD_SPEED, -LOST_TURN_REVERSE_SPEED);
}

static void driveSlowLostL() {
  currentAction = ACTION_SEARCH_LEFT;
  lastTurnAction = ACTION_TURN_LEFT;
  setMotor(-SLOW_LOST_LEFT_INNER_SPEED, SLOW_LOST_LEFT_OUTER_SPEED);
}

static void driveSlowRightRecovery() {
  currentAction = ACTION_TURN_RIGHT;
  lastTurnAction = ACTION_TURN_RIGHT;
  setMotor(SLOW_RIGHT_RECOVERY_FORWARD_SPEED, -SLOW_RIGHT_RECOVERY_REVERSE_SPEED);
}

static void driveSlowLostR() {
  currentAction = ACTION_SEARCH_LEFT;
  lastTurnAction = ACTION_TURN_LEFT;
  setMotor(-GRID_EXIT_LOST_OUTER_SPEED, GRID_EXIT_LOST_INNER_SPEED);
}

// 首次从中心摆到一侧，后续使用双倍时间在左右两侧之间穿过中心。
static void driveGrabWiggle() {
  currentAction = ACTION_FORWARD_NO_LINE;
  unsigned long now = millis();
  if (!grabWiggleStarted) {
    grabWiggleStarted = true;
    grabWiggleFirstLeg = true;
    grabWiggleRight = true;
    grabWiggleSwitchTime = now;
  }

  unsigned long legDuration = grabWiggleFirstLeg
      ? GRAB_WIGGLE_HALF_CYCLE_MS
      : GRAB_WIGGLE_HALF_CYCLE_MS * 2UL;
  if (now - grabWiggleSwitchTime >= legDuration) {
    grabWiggleRight = !grabWiggleRight;
    grabWiggleSwitchTime = now;
    grabWiggleFirstLeg = false;
  }
  if (grabWiggleRight) {
    setMotor(GRAB_WIGGLE_OUTER_SPEED, -GRAB_WIGGLE_INNER_SPEED);
  } else {
    setMotor(-GRAB_WIGGLE_INNER_SPEED, GRAB_WIGGLE_OUTER_SPEED);
  }
}

static void driveSharpL() {
  currentAction = ACTION_TURN_LEFT;
  lastTurnAction = ACTION_TURN_LEFT;
  if (currentMode == MODE_SLOW) {
    setMotor(-SLOW_SHARP_TURN_REVERSE_SPEED, SLOW_SHARP_TURN_FORWARD_SPEED);
  } else {
    setMotor(-NORMAL_SHARP_TURN_REVERSE_SPEED, NORMAL_SHARP_TURN_FORWARD_SPEED);
  }
}

static void driveSharpR() {
  currentAction = ACTION_TURN_RIGHT;
  lastTurnAction = ACTION_TURN_RIGHT;
  if (currentMode == MODE_SLOW) {
    setMotor(SLOW_SHARP_TURN_FORWARD_SPEED, -SLOW_SHARP_TURN_REVERSE_SPEED);
  } else {
    setMotor(NORMAL_SHARP_TURN_FORWARD_SPEED, -NORMAL_SHARP_TURN_REVERSE_SPEED);
  }
}

static void driveCornerL() {
  currentAction = ACTION_TURN_LEFT;
  lastTurnAction = ACTION_TURN_LEFT;
  setMotor(-CORNER_TURN_REVERSE_SPEED, CORNER_TURN_FORWARD_SPEED);
}

static void driveBackward() {
  currentAction = ACTION_BACKWARD;
  setMotor(-FINAL_BACKWARD_LEFT_SPEED, -FINAL_BACKWARD_RIGHT_SPEED);
}

static void driveLostCorrection() {
  if (lastTurnAction == ACTION_TURN_LEFT) {
    driveLostL();
  } else if (lastTurnAction == ACTION_TURN_RIGHT) {
    driveLostR();
  } else {
    driveLostL();
  }
}

static void driveByError(float err) {
  float a = absF(err);
  if (a >= LINE_SHARP_TURN_ERROR) {
    err < 0 ? driveSharpL() : driveSharpR();
  } else if (a >= LINE_MIDDLE_TURN_ERROR) {
    err < 0 ? driveMiddleL() : driveMiddleR();
  } else if (a >= LINE_SOFT_TURN_ERROR) {
    err < 0 ? driveSoftL() : driveSoftR();
  } else {
    driveStraight();
  }
}

static void applyLineFollowDrive() {
  float err = calculateLineError();

  if (currentMode != MODE_SLOW) {
    slowRightRecoveryActive = false;
    slowRightRecoveryStartTime = 0;
  } else if (slowRightRecoveryActive) {
    if (middleBlack == 1) {
      slowRightRecoveryActive = false;
      slowRightRecoveryStartTime = 0;
      if (lineState == LINE_STATE_MIDDLE_RIGHT) {
        driveSoftR();
        return;
      }
      if (lineState == LINE_STATE_MIDDLE) {
        driveStraight();
        return;
      }
      // 左中或全黑交回原流程处理。
    } else if (lineState == LINE_STATE_RIGHT || lineState == LINE_STATE_NONE) {
      if (millis() - slowRightRecoveryStartTime < SLOW_RIGHT_RECOVERY_TIMEOUT_MS) {
        driveSlowRightRecovery();
        return;
      }
      slowRightRecoveryActive = false;
      slowRightRecoveryStartTime = 0;
      if (lineState == LINE_STATE_NONE) {
        driveSlowLostL();
        return;
      }
    } else {
      slowRightRecoveryActive = false;
      slowRightRecoveryStartTime = 0;
    }
  }

  if (lineState == LINE_STATE_NONE) {
    if (currentMode == MODE_SLOW) {
      driveSlowLostL();
    } else if (gridJustExited) {
      driveSlowLostR();
    } else {
      driveLostCorrection();
    }
    return;
  }
  if (gridJustExited) gridJustExited = false;

  if (lineState == LINE_STATE_LEFT_MIDDLE) {
    currentMode == MODE_SLOW ? driveCornerL() : driveByError(err);
    return;
  }
  if (lineState == LINE_STATE_MIDDLE_RIGHT) {
    currentMode == MODE_SLOW ? driveSoftR() : driveByError(err);
    return;
  }
  if (lineState == LINE_STATE_LEFT) {
    currentMode == MODE_SLOW ? driveCornerL() : driveSharpL();
    return;
  }
  if (lineState == LINE_STATE_RIGHT) {
    if (currentMode == MODE_SLOW) {
      slowRightRecoveryActive = true;
      slowRightRecoveryStartTime = millis();
      driveSlowRightRecovery();
    } else {
      driveSharpR();
    }
    return;
  }
  if (lineState == LINE_STATE_MIDDLE) {
    driveByError(err);
    return;
  }
  if (lineState == LINE_STATE_LEFT_RIGHT || lineState == LINE_STATE_ALL) {
    if (currentMode == MODE_SLOW && lineState == LINE_STATE_ALL) {
      if (slowLongLossPending && finalLeftSearchCompleted && armPlaceDone) {
        // 长丢线后压到全黑 → 终点倒车（与 runFollowLikeMode 路径统一）
        slowLongLossPending = false;
        currentMode = MODE_BACKWARD_TO_CLEAR;
        backwardClearLineTime = millis();
        driveBackward();
      } else {
        driveCornerL();
      }
    } else {
      driveStraight();
    }
    return;
  }
  driveByError(err);
}

static void enterFollowMode(bool slowMode) {
  currentMode = slowMode ? MODE_SLOW : MODE_FOLLOW;
  currentAction = ACTION_FORWARD;
}

static void resetCycleState() {
  gridUsedThisCycle = false;
  finalLeftSearchCompleted = false;
  startupLineFound = false;
  lastGridExitTime = 0;
  backwardClearLineTime = 0;
  gridJustExited = false;
  ultrasonicCountState = ULTRASONIC_COUNT_ARMED;
  ultrasonicEnterConfirmCount = 0;
  ultrasonicClearConfirmCount = 0;
  ultrasonicNoEchoCount = 0;
  ultrasonicDetectionCount = 0;
  ultrasonicOnceStopDone = false;
  ultrasonicOnceStopActive = false;
  ultrasonicOnceStopStart = 0;
  armGrabRequested = false;
  armGrabDone = false;
  armPlaceRequested = false;
  armPlaceDone = false;
  armRestoreRequested = false;
  grabDoneTime = 0;
  grabWiggleRight = true;
  grabWiggleSwitchTime = 0;
  grabWiggleStarted = false;
  grabWiggleFirstLeg = true;
  slowLineLostStartTime = 0;
  slowLongLossPending = false;
  slowRightRecoveryActive = false;
  slowRightRecoveryStartTime = 0;
  clearDetectedShape();
  serialBlockUntilMs = millis() + 1500;
  cycleStartTime = millis();
  lastTurnAction = ACTION_FORWARD;
  enterFollowMode(false);
}

static void forceResetAllState() {
  resetCycleState();
  cycleInitialized = false;
}

static void beginCycle() {
  resetCycleState();
  servo2Restore();
  armRequestMode(ARM_MODE_DETECT);
  cycleInitialized = true;
}

static void enterGridMode() {
  if (currentMode != MODE_FOLLOW) return;
  gridUsedThisCycle = true;
  currentMode = MODE_GRID;
  currentAction = ACTION_FORWARD;
  gridModeStartTime = millis();
  servo1Detach();
  servo3Detach();
  setMotor(GRID_LEFT_SPEED, GRID_RIGHT_SPEED);
}

static void runGridState() {
  currentAction = ACTION_FORWARD;
  if (gridModeStartTime == 0) gridModeStartTime = millis();
  if (millis() - gridModeStartTime < GRID_MODE_TIME_MS) {
    servo1Detach();
    servo3Detach();
    setMotor(GRID_LEFT_SPEED, GRID_RIGHT_SPEED);
    return;
  }
  lastGridExitTime = millis();
  gridModeStartTime = 0;
  gridJustExited = true;
  servo1Restore();
  servo3Restore();
  enterFollowMode(false);
  applyLineFollowDrive();
}

static void runFinalLeftSearchMode() {
  if (lineState == LINE_STATE_NONE) {
    driveSlowLostL();
    return;
  }
  finalLeftSearchCompleted = true;
  enterFollowMode(true);
  // 找到线后统一走慢速循迹流程，确保全黑时经过倒车判定（而非直接当直角弯）
  runFollowLikeMode();
}

static void runBackwardToClearMode() {
  if (backwardClearLineTime != 0 &&
      millis() - backwardClearLineTime >= BACKWARD_STOP_DELAY_MS) {
    backwardClearLineTime = 0;
    currentMode = MODE_CYCLE_END;
    currentAction = ACTION_STOP_AT_TARGET;
    stopCar();
    armRequestMode(ARM_MODE_DETECT);
    return;
  }
  driveBackward();
}

static void runCycleEndMode() {
  currentAction = ACTION_STOP_AT_TARGET;
  stopCar();
  if (armGetMode() == ARM_MODE_DETECT && !armIsBusy() && hasDetectedShape()) {
    forceResetAllState();
  }
}

static void handleArmPlaceFlow() {
  if (!armGrabRequested) return;

  if (!armGrabDone) {
    if (!armIsBusy() && armGetMode() == ARM_MODE_GRAB) {
      armGrabDone = true;
      grabDoneTime = millis();
      grabWiggleRight = true;
      grabWiggleSwitchTime = 0;
      grabWiggleStarted = false;
      grabWiggleFirstLeg = true;
      servoSetAngle(SERVO_2, ARM_GRID_S2);
    }
    return;
  }

  if (!armPlaceDone) {
    if (!armIsBusy() && armGetMode() == ARM_MODE_PLACE) {
      armPlaceDone = true;
      armRestoreRequested = true;
      armRequestMode(ARM_MODE_HOLD);
    }
    return;
  }

  if (armRestoreRequested) {
    if (!armIsBusy() && armGetMode() == ARM_MODE_HOLD) {
      armRestoreRequested = false;
      enterFollowMode(true);
    }
  }
}

// 慢速模式下持续累计丢线时长。连续丢线≥阈值后置位 slowLongLossPending，
// 用于在随后检测到三灰全黑时区分”终点倒车”与”普通直角弯”。
// 标志一旦置位就保持，直到全黑触发倒车时才清除。
static void updateSlowLineLossTracking() {
  bool slowContext = (currentMode == MODE_SLOW || currentMode == MODE_FINAL_LEFT_SEARCH);
  if (!slowContext) {
    slowLineLostStartTime = 0;
    slowLongLossPending = false;
    return;
  }

  if (slowRightRecoveryActive && lineState == LINE_STATE_NONE) {
    slowLineLostStartTime = 0;
    return;
  }

  if (lineState == LINE_STATE_NONE) {
    if (slowLineLostStartTime == 0) {
      slowLineLostStartTime = millis();
    } else if (millis() - slowLineLostStartTime >= SLOW_BACKWARD_MIN_LINE_LOSS_MS) {
      slowLongLossPending = true;
    }
    return;
  }

  // 压到线：结束本次丢线计时，但不清除挂起标志
  slowLineLostStartTime = 0;
}

static void runFollowLikeMode() {
  bool normalMode = (currentMode == MODE_FOLLOW);
  bool slowMode = (currentMode == MODE_SLOW);
  if (normalMode &&
      GRID_MODE_ENABLED &&
      lineState == LINE_STATE_ALL &&
      millis() - lastGridExitTime >= GRID_REENTER_BLOCK_MS) {
    enterGridMode();
    return;
  }

  if (slowMode && !finalLeftSearchCompleted &&
      lineState == LINE_STATE_NONE && !slowRightRecoveryActive) {
    currentMode = MODE_FINAL_LEFT_SEARCH;
    driveSlowLostL();
    return;
  }

  if (slowMode && finalLeftSearchCompleted &&
      armPlaceDone && lineState == LINE_STATE_ALL) {
    if (slowLongLossPending) {
      // 之前发生过≥300ms的丢线，再压到全黑 → 终点倒车
      slowLongLossPending = false;
      currentMode = MODE_BACKWARD_TO_CLEAR;
      backwardClearLineTime = millis();
      runBackwardToClearMode();
    } else {
      // 没有长时间丢线就遇到全黑 → 当作直角弯处理
      driveCornerL();
    }
    return;
  }

  applyLineFollowDrive();
}

void applyRawLineFollow(int forwardSpeed) {
  (void)forwardSpeed;
  applyLineFollowDrive();
}

void runLineFollow() {
  if (!cycleInitialized) beginCycle();

  lineState = getLineState();

  if (millis() >= serialBlockUntilMs) {
    serialCommUpdate();
  }

  if (currentMode != MODE_GRID) {
    armUpdate();
    handleArmPlaceFlow();

    if (hasDetectedShape() && !armGrabRequested &&
        armGetMode() == ARM_MODE_DETECT && !armIsBusy()) {
      armGrabRequested = true;
      armRequestMode(ARM_MODE_GRAB);
    }

    if (armIsBusy()) {
      currentAction = ACTION_STOP_AT_TARGET;
      stopCar();
      return;
    }

    if (armGetMode() == ARM_MODE_DETECT && !hasDetectedShape()) {
      currentAction = ACTION_STOP_AT_TARGET;
      stopCar();
      return;
    }
  }

  if (gridUsedThisCycle &&
      currentMode != MODE_GRID &&
      armGetMode() == ARM_MODE_GRAB &&
      !armIsBusy() &&
      !armPlaceRequested) {
    if (stopForUltrasonicObstacle()) return;
  }

  if (millis() - cycleStartTime < CYCLE_STARTUP_HOLD_MS) {
    currentAction = ACTION_STOP_AT_TARGET;
    stopCar();
    return;
  }

  if (!startupLineFound) {
    if (lineState == LINE_STATE_NONE) {
      // 抓取完成后直行找线，超过设定时间还没找到就摆头
      if (armGrabDone && millis() - grabDoneTime >= GRAB_WIGGLE_DELAY_MS) {
        driveGrabWiggle();
      } else {
        driveNoLine();
      }
      return;
    }
    startupLineFound = true;
  }

  updateSlowLineLossTracking();

  switch (currentMode) {
    case MODE_FOLLOW:
    case MODE_SLOW:
      runFollowLikeMode();
      break;
    case MODE_GRID:
      runGridState();
      break;
    case MODE_FINAL_LEFT_SEARCH:
      runFinalLeftSearchMode();
      break;
    case MODE_BACKWARD_TO_CLEAR:
      runBackwardToClearMode();
      break;
    case MODE_CYCLE_END:
      runCycleEndMode();
      break;
    default:
      beginCycle();
      break;
  }
}

void runNormalLineFollow() {
  if (currentMode == MODE_FOLLOW || currentMode == MODE_SLOW) {
    applyLineFollowDrive();
  }
}

void runLineFollowWithSpeed(int forwardSpeed) {
  (void)forwardSpeed;
  runNormalLineFollow();
}

void startGridMode() {
  enterGridMode();
}
