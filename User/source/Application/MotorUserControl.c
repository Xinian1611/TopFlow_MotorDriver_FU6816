/************************************************************************************
 * @Author       : FengYu
 * @Date         : 2026-07-08 15:03:49
 * @LastEditors  : zhangbao zhb@wujiang.com
 * @LastEditTime : 2026-07-08 15:03:59
 * @Description  : 电机控制模块
 * @
 * @Copyright (c) 2026 by ${git_name_email}, All Rights Reserved.
*************************************************************************************/

/******************************************************************************
 * 头文件
 ******************************************************************************/
#include <MyProject.h>
#include "CanSlave.h"
#include "DevConfig.h"


/******************************************************************************
 * 全局变量
 ******************************************************************************/

//  电机参数
float xdata Pole_Pairs = 7.0;                               ///<      电机 极对数
float xdata RS = 0.0163;                                    ///< (Ω)  电机 相电阻
float xdata LD = 0.143/1000.0;					            ///< (H)  电机 相电感
float xdata LQ = 0.151/1000.0;                              ///< (H)  电机 相电感
float xdata MOTOR_SPEED_BASE = 2350.0 * 2.0;                ///< (RPM) 速度基准 = 额定速度 * 2
float xdata Ke = 17.9;                                      ///< (V/KRPM) 反电动势常数


//  电机控制模式
uint8_t xdata MOTOR_CTRL_MODE = SPEED_LOOP_CONTROL; ///< 闭环方式选择

//  电机运行状态
MotorRunState_t xdata g_run_state;


/******************************************************************************
 * 函数定义
 ******************************************************************************/

/// @brief 初始化电机参数
/// @param None
/// @retval None
static void MotorCtrl_InitParams(void)
{
    // 载入电机参数
    DevConfig_LoadParam();

    // 初始化电机参数
    switch (g_motor_cfg->motor_type)
    {
    case MOTOR_12V_600W:
        Pole_Pairs = MOTOR_12V_POLE_PAIRS;
        RS = MOTOR_12V_RS;
        LD = MOTOR_12V_LD;
        LQ = MOTOR_12V_LQ;
        Ke = MOTOR_12V_KE;
        MOTOR_SPEED_BASE = MOTOR_12V_SPEED_BASE;
        break;

    case MOTOR_24V_1200W:
        Pole_Pairs = MOTOR_24V_POLE_PAIRS;
        RS = MOTOR_24V_RS;
        LD = MOTOR_24V_LD;
        LQ = MOTOR_24V_LQ;
        Ke = MOTOR_24V_KE;
        MOTOR_SPEED_BASE = MOTOR_24V_SPEED_BASE;
        break;

    case MOTOR_36V_2400W:
        Pole_Pairs = MOTOR_36V_POLE_PAIRS;
        RS = MOTOR_36V_RS;
        LD = MOTOR_36V_LD;
        LQ = MOTOR_36V_LQ;
        Ke = MOTOR_36V_KE;
        MOTOR_SPEED_BASE = MOTOR_36V_SPEED_BASE;
        break;

    default:
        break;
    }

    // 默认速度闭环
    MOTOR_CTRL_MODE = SPEED_LOOP_CONTROL;
}

/// @brief 初始化电机控制模块
/// @param None
/// @retval None
void MotorCtrl_Init(void)
{
    // 初始化参数
    MotorCtrl_InitParams();

    // 初始化运行状态
    // memset(&g_run_state, 0, sizeof(g_run_state));
    // g_run_state.work_mode = EXT_WORK_MODE_SPEED;
    // g_run_state.is_started = 1;
    // g_run_state.is_estop = 0;
    // g_run_state.ref = 100;
    // g_run_state.direction = CCW;

    memset(&g_run_state, 0, sizeof(g_run_state));
    g_run_state.work_mode = EXT_WORK_MODE_SPEED;
    g_run_state.is_started = 0;          /* 上电不自动起机，等 CAN 的"起机" */
    g_run_state.is_estop   = 0;
    g_run_state.ref        = 0.0f;
    g_run_state.direction  = CCW;
}

/// @brief 设置电机急停状态
/// @param state 0: 未急停 1: 已急停
/// @retval None
void MotorCtrl_SetEstop(uint8_t state)
{
    g_run_state.is_estop = state;
    if (state)
    {
        g_run_state.ref = 0;
        g_run_state.run_time = 0;
    }
}

/// @brief 设置电机运行状态
/// @param state 0: 停止 1: 运行
/// @retval None
void MotorCtrl_SetRun(uint8_t state)
{
    if (state == g_run_state.is_started)
        return;

    g_run_state.is_started = state;

    if (g_run_state.is_started == 0)
    {
        g_run_state.run_time = 0;

        g_run_state.ref              = 0.0f;   /* 停机撤给定，再起机必须重新下发速度 */
        g_run_state.reversal_braking = 0;
    }
}

/// @brief 设置电机控制模式
/// @param mode 闭环方式选择
/// @retval None
void MotorCtrl_SetMode(uint8_t mode)
{
    // 检查模式是否有效
    if (mode < EXT_WORK_MODE_CURRENT || mode > EXT_WORK_MODE_SPEED)
        return;

    if (mode == g_run_state.work_mode)
        return;

    g_run_state.work_mode = mode;
    if (mode == EXT_WORK_MODE_SPEED)
    {
        MOTOR_CTRL_MODE = SPEED_LOOP_CONTROL;
    }
    else if (mode == EXT_WORK_MODE_CURRENT)
    {
        MOTOR_CTRL_MODE = CURRENT_LOOP_CONTROL;
    }
}

/// @brief 设置电机目标值(正数正转, 负数反转)
/// @param current 段字节13-14：0.01A，32767 = 0A
/// @param speed   段字节11-12：0.01%，32767 = 0%，±10000 = ±100%
/// @note  ① 必须按无符号收：正数 raw 一定 >32767，用 int16 会变成大负数；
///        ② 0% 只表示"目标 0"，停机由启停字段负责 —— 这里不再自动置 is_started=0，
///           否则"起机+0% → 再给 20%"这种时序会让后面的速度全部不生效。
/// @retval None
void MotorCtrl_SetTarget(uint16_t current, uint16_t speed)
{
    int32 v;
    float mag;
    float base;
    float idle;

    if (g_run_state.is_estop)
    {
        return;                                 /* 急停中不接受目标 */
    }

    if (g_run_state.work_mode == EXT_WORK_MODE_CURRENT)
    {
        v   = (int32)current - 32767;           /* 0.01A */
        mag = (v >= 0) ? (float)v : (float)(-v);
        g_run_state.direction = (v >= 0) ? CW : CCW;
        g_run_state.ref       = mag / 100.0f;   /* 0.01A → A */

    }
    else                                        /* 转速模式 */
    {
        v   = (int32)speed - 32767;             /* 0.01% */
        mag = (v >= 0) ? (float)v : (float)(-v);
        if (mag > 10000.0f) mag = 10000.0f;     /* 钳到 ±100% */

        g_run_state.direction = (v >= 0) ? CW : CCW;

        if (mag <= 0.0f)
        {
            g_run_state.ref = 0.0f;             /* 0% = 停机 */
        }
        else
        {
            /* 100% = 配置里的 max_speed */
            base = (g_motor_cfg->max_speed != 0) ? (float)g_motor_cfg->max_speed
                                                 : MOTOR_SPEED_BASE;
            /* 非零给定时的下限 = 配置里的 idle_speed(rpm) */
            idle = (float)g_motor_cfg->idle_speed;
            if (idle > base) idle = base;

            /* 1%~100% → idle_speed ~ max_speed */
            g_run_state.ref = idle + (base - idle) * (mag / 10000.0f);
        }
    }
}
// void MotorCtrl_SetTarget(uint16_t current, uint16_t speed)
// {
//     int32 v;
//     float mag;
//     float base;

//     if (g_run_state.is_estop)
//     {
//         return;                                 /* 急停中不接受目标 */
//     }

//     if (g_run_state.work_mode == EXT_WORK_MODE_CURRENT)
//     {
//         v   = (int32)current - 32767;           /* 0.01A */
//         mag = (v >= 0) ? (float)v : (float)(-v);
//         g_run_state.direction = (v >= 0) ? CW : CCW;
//         g_run_state.ref       = mag / 100.0f;   /* 0.01A → A */
//     }
//     else                                        /* 转速模式 */
//     {
//         v   = (int32)speed - 32767;             /* 0.01% */
//         mag = (v >= 0) ? (float)v : (float)(-v);
//         g_run_state.direction = (v >= 0) ? CW : CCW;

//         /* 协议 100% = 主机约定的最大转速(配置里 max_speed)，不是 FOC 基准 4096 */
//         base = (g_motor_cfg->max_speed != 0) ? (float)g_motor_cfg->max_speed
//                                              : MOTOR_SPEED_BASE;
//         g_run_state.ref = mag / 10000.0f * base;
//     }
// }
// /// @brief 设置电机目标值(正数正转, 负数反转)
// /// @param current 目标电流(0.01A)
// /// @param speed 目标转速(0.01%)
// /// @retval None
// void MotorCtrl_SetTarget(uint16_t current, uint16_t speed)
// {
//     // 根据工作模式和控制状态设置电机参数
//     if (g_run_state.is_started && !g_run_state.is_estop)
//     {
//         if (g_run_state.work_mode == EXT_WORK_MODE_SPEED)
//         {
//             // float ref_val = (speed - 32767) * 1.0f;
//             float ref_val = (float)(speed-32767)/10000.0f * MOTOR_SPEED_BASE;
//             if (ref_val >= 0)
//             {
//                 g_run_state.direction = CW;       // 正数正转
//                 g_run_state.ref = ref_val;
//             }
//             else
//             {
//                 g_run_state.direction = CCW;      // 负数反转
//                 g_run_state.ref = -ref_val;       // 存储绝对值
//             }
//             if (speed == 32767)
//             {
//                 g_run_state.is_started = 0;
//             }
//         }
//         else if (g_run_state.work_mode == EXT_WORK_MODE_CURRENT)
//         {
//             float ref_val = (current - 32767) / 100.0f;
//             if (ref_val >= 0)
//             {
//                 g_run_state.direction = CW;       // 正数正转
//                 g_run_state.ref = ref_val;
//             }
//             else
//             {
//                 g_run_state.direction = CCW;      // 负数反转
//                 g_run_state.ref = -ref_val;       // 存储绝对值
//             }
//             if (current == 32767)
//             {
//                 g_run_state.is_started = 0;
//             }
//         }
//     }
// }

/// @brief 更新电机状态
/// @param None
/// @retval None
void MotorCtrl_Update(void)
{

}

