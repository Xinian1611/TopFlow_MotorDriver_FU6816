/************************************************************************************
 * @Author       : FengYu
 * @Date         : 2026-07-08 13:40:27
 * @LastEditors  : zhangbao zhb@wujiang.com
 * @LastEditTime : 2026-07-08 13:44:16
 * @Description  : 电机参数
 * @
 * @Copyright (c) 2026 by ${git_name_email}, All Rights Reserved.
*************************************************************************************/

#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

/******************************************************************************
 * 头文件
 ******************************************************************************/

#include "CanProtocol.h"


/******************************************************************************
 * 宏定义
 ******************************************************************************/

#define MOTOR_TYPE_NONE                         0
#define MOTOR_12V_600W                          1
#define MOTOR_24V_1200W                         2
#define MOTOR_36V_2400W                         3
#define MOTOR_TYPE_MAX                          4

#define MOTOR_SELECT                            MOTOR_36V_2400W
#define SPEED_PRECISION                         (1)
#define MAX_SPEED                               (2300)
#define IDLE_SPEED                              (250)

// 12V电机参数
#define MOTOR_12V_POLE_PAIRS                     (7.0)
#define MOTOR_12V_RS                             (0.0163 / 2)
#define MOTOR_12V_LD                             (0.0639/1000.0 / 2)
#define MOTOR_12V_LQ                             (0.0655/1000.0 / 2)
#define MOTOR_12V_SPEED_BASE                     (1669.0 * 2)
#define MOTOR_12V_KE                             (7.19)                              

// 24V电机参数
#define MOTOR_24V_POLE_PAIRS                     (7.0)
#define MOTOR_24V_RS                             (0.0163 / 2)
#define MOTOR_24V_LD                             (0.0573/1000.0 / 2)
#define MOTOR_24V_LQ                             (0.0562/1000.0 / 2)
#define MOTOR_24V_SPEED_BASE                     (1831.0 * 2)
#define MOTOR_24V_KE                             (13.1)                               

// 36V电机参数
#define MOTOR_36V_POLE_PAIRS                     (7.0)
#define MOTOR_36V_RS                             (0.019)
#define MOTOR_36V_LD                             (0.138 / 1000.0 * 0.99)
#define MOTOR_36V_LQ                             (MOTOR_36V_LD)
#define MOTOR_36V_SPEED_BASE                     (4096.0)
#define MOTOR_36V_KeVpp                          (7.6832)
#define MOTOR_36V_KeT                            (100.4)
#define MOTOR_36V_KE                             (MOTOR_36V_POLE_PAIRS * MOTOR_36V_KeVpp * MOTOR_36V_KeT / 207.84)                                 


/******************************************************************************
 * 类型定义
 ******************************************************************************/

/// @brief  电机运行状态结构体
typedef struct {
    uint8_t work_mode;       // 当前工作模式
    uint8_t is_started;      // 启停状态: 0=停机, 1=起机
    uint8_t is_estop;        // 急停状态: 0=非急停, 1=急停
    uint8_t reversal_braking; // 反向刹车状态: 0=未刹车, 1=刹车中
    float ref;               // 目标值(绝对值)
    uint8_t direction;       // 旋转方向: 0=CW正转, 1=CCW反转
    uint8_t fault_code;      // 当前故障码
    uint32_t run_time;       // 单次运行时间(s)
    uint32_t total_time;     // 总运行时间(s)
} MotorRunState_t;


/******************************************************************************
 * 变量声明
 ******************************************************************************/

extern float xdata Pole_Pairs;
extern float xdata RS;
extern float xdata LD;
extern float xdata LQ;
extern float xdata MOTOR_SPEED_BASE;
extern uint8_t xdata MOTOR_CTRL_MODE;

extern MotorRunState_t xdata g_run_state;


/******************************************************************************
 * 函数声明
 ******************************************************************************/
extern void MotorCtrl_Init(void);
extern void MotorCtrl_SetRun(uint8_t state);
extern void MotorCtrl_SetEstop(uint8_t state);
extern void MotorCtrl_SetMode(uint8_t mode);
extern void MotorCtrl_SetTarget(int16_t current, int16_t speed);
extern void MotorCtrl_Update(void);


#endif // MOTOR_CTRL_H
