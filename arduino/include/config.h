#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===== 电机引脚 =====
const int LEFT_RED_PIN    = 5;
const int LEFT_BLACK_PIN  = 6;
const int RIGHT_RED_PIN   = 9;
const int RIGHT_BLACK_PIN = 10;

// ===== 灰度传感器引脚（从左到右：A0、A3、A2） =====
const int LEFT_SENSOR_PIN   = A0;
const int MIDDLE_SENSOR_PIN = A3;
const int RIGHT_SENSOR_PIN  = A2;

// ===== 超声波传感器 =====
const int ULTRASONIC_TRIG_PIN = 12;
const int ULTRASONIC_ECHO_PIN = 8;
const bool ULTRASONIC_ENABLED = true;
const float ULTRASONIC_STOP_MIN_CM = 12.0f;   // 触发停车的最近距离（cm）
const float ULTRASONIC_STOP_MAX_CM = 20.0f;   // 触发停车的最远距离（cm）
const float ULTRASONIC_CLEAR_CM = 25.0f;      // 超过此距离才开始确认当前障碍物已离开
const unsigned long ULTRASONIC_SAMPLE_INTERVAL_MS = 60;
const unsigned long ULTRASONIC_ECHO_TIMEOUT_US = 4000;
const uint8_t ULTRASONIC_ENTER_CONFIRM_SAMPLES = 2;
const uint8_t ULTRASONIC_CLEAR_CONFIRM_SAMPLES = 3;
const uint8_t ULTRASONIC_NO_ECHO_CLEAR_SAMPLES = 4;
const bool ULTRASONIC_SERIAL_DEBUG_ENABLED = false;
const long ULTRASONIC_SERIAL_BAUD = 9600;
const unsigned long ULTRASONIC_SERIAL_PRINT_INTERVAL_MS = 200;

// ===== 栅格模式 =====
const bool GRID_MODE_ENABLED = true;
const int GRID_LEFT_SPEED  = 210;               // 栅格冲刺左轮速度（0~255）
const int GRID_RIGHT_SPEED = 215;                 // 栅格冲刺右轮速度（0~255）
const unsigned long GRID_MODE_TIME_MS     = 3600;  // 栅格冲刺持续时间（ms）
const unsigned long GRID_REENTER_BLOCK_MS = 5000;   // 退出栅格后多久才能再次进入（ms）

// ===== 灰度传感器阈值（低于阈值判为黑线） =====
const int LEFT_THRESHOLD   = 123;
const int MIDDLE_THRESHOLD = 145;
const int RIGHT_THRESHOLD  = 123;

// ===== 基础直行速度 =====
const int FORWARD_SPEED = 150; // forward() 直接调用时的速度

// ===== 电机反转参数 =====
const int MOTOR_REVERSE_MIN_PWM  = 0;
const int MOTOR_REVERSE_KICK_PWM = 255;
const unsigned long MOTOR_REVERSE_KICK_MS = 0;

// ===== 循迹误差参数 =====
const float LINE_ERROR_MAX              = 100.0f;
const float LINE_RAW_SIGNAL_MARGIN      = 100.0f;
const float LINE_CENTER_STRENGTH_WEIGHT = 1.00f;
const float LINE_ERROR_BIAS             = -20.0f; // 误差零点偏置：负值让车整体偏右循迹（左中传感器压线）

// ===== 转向误差阈值 =====
const float LINE_SOFT_TURN_ERROR   = 12.0f; // 轻微偏线阈值
const float LINE_MIDDLE_TURN_ERROR = 38.0f; // 明显偏线阈值
const float LINE_SHARP_TURN_ERROR  = 62.0f; // 急转阈值

// ===== 普通循迹速度（最终 PWM） =====
const int NORMAL_FORWARD_SPEED             = 90;
const int NORMAL_SOFT_TURN_INNER_SPEED     = 62;
const int NORMAL_SOFT_TURN_OUTER_SPEED     = 83;
const int NORMAL_MIDDLE_TURN_INNER_SPEED   = 53;
const int NORMAL_MIDDLE_TURN_OUTER_SPEED   = 93;
const int NORMAL_SHARP_TURN_FORWARD_SPEED  = 168;
const int NORMAL_SHARP_TURN_REVERSE_SPEED  = 128;

// ===== 慢速循迹速度（最终 PWM） =====
const int SLOW_FORWARD_SPEED           = 105;
const int SLOW_SOFT_TURN_INNER_SPEED   = 73;
const int SLOW_SOFT_TURN_OUTER_SPEED   = 98;
const int SLOW_MIDDLE_TURN_INNER_SPEED = 62;
const int SLOW_MIDDLE_TURN_OUTER_SPEED = 109;
const int SLOW_SHARP_TURN_FORWARD_SPEED  = 210;
const int SLOW_SHARP_TURN_REVERSE_SPEED  = 160;
const int SLOW_RIGHT_RECOVERY_FORWARD_SPEED = 180;
const int SLOW_RIGHT_RECOVERY_REVERSE_SPEED = 160;
const unsigned long SLOW_RIGHT_RECOVERY_TIMEOUT_MS = 300;

// ===== 固定动作速度（最终 PWM） =====
const int SHARP_TURN_FORWARD_SPEED = 210; // 急转外侧速度
const int SHARP_TURN_REVERSE_SPEED = 160; // 急转内侧反转速度
const int LOST_TURN_FORWARD_SPEED  = 168; // 普通丢线修正外侧速度
const int LOST_TURN_REVERSE_SPEED  = 112; // 普通丢线修正内侧反转速度
const int SLOW_LOST_LEFT_INNER_SPEED = 25;  // 慢速丢线左找线内侧速度
const int SLOW_LOST_LEFT_OUTER_SPEED = 195; // 慢速丢线左找线外侧速度
const int GRID_EXIT_LOST_INNER_SPEED = 190;  // 栅格退出后找线内侧速度
const int GRID_EXIT_LOST_OUTER_SPEED =100; // 栅格退出后找线外侧速度

// ===== 抓取完成后摆头找线 =====
const unsigned long GRAB_WIGGLE_DELAY_MS      = 700; // 抓取后多久没找到线就开始摆头（ms）
const unsigned long GRAB_WIGGLE_HALF_CYCLE_MS = 800;//头固定单次时间（ms）
const int GRAB_WIGGLE_MAX_STEPS = 5;     // 保留（固定幅度模式下不再使用）
const int GRAB_WIGGLE_INNER_SPEED = 170; // 摆头内侧速度（实际反转）
const int GRAB_WIGGLE_OUTER_SPEED = 170; // 摆头外侧速度
const int CORNER_TURN_FORWARD_SPEED  = 240; // 直角弯外2速度
const int CORNER_TURN_REVERSE_SPEED  = 240;// 直角弯内侧反转速度
const int FINAL_BACKWARD_LEFT_SPEED  = 105; // 终点后退左轮速度
const int FINAL_BACKWARD_RIGHT_SPEED = 125; // 终点后退右轮速度

const int TURN_OUTER_SPEED = SHARP_TURN_FORWARD_SPEED;
const int TURN_INNER_SPEED = -SHARP_TURN_REVERSE_SPEED;

// ===== 流程时间参数 =====
const unsigned long CYCLE_STARTUP_HOLD_MS      = 800;  // 启动后等待时间（ms）
const unsigned long BACKWARD_CLEAR_DURATION_MS = 500;  // 倒车清线时间（ms，未使用）
const unsigned long BACKWARD_STOP_DELAY_MS     = 3700; // 终点倒车持续时间（ms）
const unsigned long SLOW_BACKWARD_MIN_LINE_LOSS_MS = 300; // 慢速模式：丢线需超过此时长，全黑才触发倒车，否则走直角弯
const int CONTROL_DELAY_MS = 10; // 主循环间隔（ms）

// ===== 舵机引脚 =====
const bool SERVO_ENABLED = true;
const int SERVO1_PIN = 3;
const int SERVO2_PIN = 4;
const int SERVO3_PIN = 7;

// ===== 机械臂角度 =====
// 识别模式
const int ARM_DETECT_S1 = 20;
const int ARM_DETECT_S2 = 35;
const int ARM_DETECT_S3 = 235;//235
// 抓取模式
const int ARM_GRAB_S1 = 20;
const int ARM_GRAB_S2 = 145; // 上下范围：5~145
const int ARM_GRAB_S3 = 175;
// 放置模式
const int ARM_PLACE_S1 = 130;
const int ARM_PLACE_S2 = 100;
const int ARM_PLACE_S3 = 235;
// 栅格模式
const int ARM_GRID_S1 = 20;
const int ARM_GRID_S2 = 145;
// 保持模式（放置完成后慢速循迹期间）
const int ARM_HOLD_S1 = 20;
const int ARM_HOLD_S2 = 120;
const int ARM_HOLD_S3 = 235;

// ===== 机械臂动作时序 =====
const unsigned long ARM_STEP_INTERVAL_MS  = 1200; // 每步动作间隔（ms）
const unsigned long ARM_GRAB_S2_DELAY_MS  = 5000; // 抓取模式 servo2 动作前等待（ms）
const unsigned long ARM_GRAB_S3_DELAY_MS  = 1000; // 抓取模式 servo3 动作前等待（ms）
const unsigned long ARM_DONE_HOLD_MS      = 500; // 动作完成后额外等待（ms）

#endif
