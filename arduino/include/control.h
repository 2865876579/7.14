#ifndef CONTROL_H
#define CONTROL_H

// 传感器组合状态码
#define LINE_STATE_NONE         0
#define LINE_STATE_LEFT       100
#define LINE_STATE_MIDDLE      10
#define LINE_STATE_RIGHT        1
#define LINE_STATE_LEFT_RIGHT 101
#define LINE_STATE_LEFT_MIDDLE 110
#define LINE_STATE_MIDDLE_RIGHT 11
#define LINE_STATE_ALL        111

// 小车动作标识
#define ACTION_FORWARD         1
#define ACTION_TURN_LEFT       2
#define ACTION_TURN_RIGHT      3
#define ACTION_FORWARD_NO_LINE 4
#define ACTION_SEARCH_LEFT     5
#define ACTION_SEARCH_RIGHT    6
#define ACTION_STOP_AT_TARGET  7
#define ACTION_BACKWARD        8

// 运行模式
#define MODE_FOLLOW            1
#define MODE_GRID              2
#define MODE_STOP              3
#define MODE_SLOW              4
#define MODE_FINAL_LEFT_SEARCH 5
#define MODE_BACKWARD_TO_CLEAR 6
#define MODE_CYCLE_END         7

extern int currentAction;
extern int lineState;
extern int currentMode;
extern unsigned long gridModeStartTime;

int   getLineState();
float calculateLineError();
void  applyRawLineFollow(int forwardSpeed);
void  runLineFollow();
void  runNormalLineFollow();
void  runLineFollowWithSpeed(int forwardSpeed);
void  startGridMode();

#endif
