#ifndef POSITION_PROTECT_H
#define POSITION_PROTECT_H

#include <FU68xx_6.h>
#include "PositionCtrl.h"

/* 三个判断结果 */
#define POS_PROTECT_NONE     0      /* 正常，无限位/堵转 */
#define POS_PROTECT_LIMIT    1      /* 机械限位 */
#define POS_PROTECT_STALL    2      /* 堵转 */

/* ==================== 软件限位值 ==================== */
/*
 * 这是软件限位，不是机械限位，可以比机械限位小。
 * 位置 fb 是经过 POS_FB_DIR/POS_FB_SCALE 后的机械角度/百分比，
 * 这里填的就是“到这个位置就不许再往外走”的值。
 */
#if (MOTOR_POSITION_CLOSE_LOOP == 2)
#define POS_LIMIT_MIN        5.0f    /* %，软件下限，按机构改 */
#define POS_LIMIT_MAX        95.0f   /* %，软件上限，按机构改 */
#else
#define POS_LIMIT_MIN        5.0f    /* °，软件下限，按机构改 */
#define POS_LIMIT_MAX        355.0f  /* °，软件上限，按机构改 */
#endif

/* 按设备设堵转电流；升降/折叠按实际电机额定电流再调 */
#if (MOTOR_DEV_NUM == 1)
  #define POS_STALL_CURRENT_A  15.0f
#elif (MOTOR_DEV_NUM == 2)
  #define POS_STALL_CURRENT_A  15.0f
#elif (MOTOR_DEV_NUM == 3)
  #define POS_STALL_CURRENT_A  15.0f
#else
  #define POS_STALL_CURRENT_A  15.0f
#endif

/* 堵转确认时间：30ms，滤掉 PWM 开关尖峰 */
#ifndef POS_STALL_CURRENT_MS
#define POS_STALL_CURRENT_MS 30
#endif

/* 堵转速度阈值：|SpeedFlt| 低于该值才认为“转不动”，单位 rpm */
#ifndef POS_STALL_SPEED_RPM
#define POS_STALL_SPEED_RPM 100.0f
#endif

/* 堵转电流一阶低通系数：0~1，越小越平滑 */
#ifndef POS_STALL_CURRENT_LPF
#define POS_STALL_CURRENT_LPF 0.5f
#endif

/* 限位判断：posActive=1 才判断，只有位置闭环时传 1 */
uint8 PositionProtect_LimitCheck(float target, float fb, uint8 posActive);

/* 堵转判断：全局调用，无状态门控 */
uint8 PositionProtect_StallCheck(void);

#endif
