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
//已经定义在Customer.h中
// //  电机参数
// float xdata Pole_Pairs = 7.0;                               ///<      电机 极对数
// float xdata RS = 0.0163;                                    ///< (Ω)  电机 相电阻
// float xdata LD = 0.143/1000.0;					            ///< (H)  电机 相电感
// float xdata LQ = 0.151/1000.0;                              ///< (H)  电机 相电感
// float xdata MOTOR_SPEED_BASE = 2350.0 * 2.0;                ///< (RPM) 速度基准 = 额定速度 * 2
// float xdata Ke = 17.9;                                      ///< (V/KRPM) 反电动势常数


// //  电机控制模式
// uint8_t xdata MOTOR_CTRL_MODE = SPEED_LOOP_CONTROL; ///< 闭环方式选择

//  电机运行状态
MotorRunState_t xdata g_run_state;


/******************************************************************************
 * 函数定义
 ******************************************************************************/

// /// @brief 初始化电机参数
// /// @param None
// /// @retval None
// static void MotorCtrl_InitParams(void)
// {
//     // 载入电机参数
//     DevConfig_LoadParam();

//     // 初始化电机参数
//     switch (g_motor_cfg->motor_type)
//     {
//     case MOTOR_12V_600W:
//         Pole_Pairs = MOTOR_12V_POLE_PAIRS;
//         RS = MOTOR_12V_RS;
//         LD = MOTOR_12V_LD;
//         LQ = MOTOR_12V_LQ;
//         Ke = MOTOR_12V_KE;
//         MOTOR_SPEED_BASE = MOTOR_12V_SPEED_BASE;
//         break;

//     case MOTOR_24V_1200W:
//         Pole_Pairs = MOTOR_24V_POLE_PAIRS;
//         RS = MOTOR_24V_RS;
//         LD = MOTOR_24V_LD;
//         LQ = MOTOR_24V_LQ;
//         Ke = MOTOR_24V_KE;
//         MOTOR_SPEED_BASE = MOTOR_24V_SPEED_BASE;
//         break;

//     case MOTOR_36V_2400W:
//         Pole_Pairs = MOTOR_36V_POLE_PAIRS;
//         RS = MOTOR_36V_RS;
//         LD = MOTOR_36V_LD;
//         LQ = MOTOR_36V_LQ;
//         Ke = MOTOR_36V_KE;
//         MOTOR_SPEED_BASE = MOTOR_36V_SPEED_BASE;
//         break;

//     default:
//         break;
//     }

//     // 默认速度闭环
//     MOTOR_CTRL_MODE = SPEED_LOOP_CONTROL;
// }

/// @brief 初始化电机控制模块
/// @param None
/// @retval None
void MotorCtrl_Init(void)
{
    // 初始化参数
    // MotorCtrl_InitParams();
    /* 改动：原 MotorCtrl_InitParams() 已删除 —— 它会把初版电机的
     *   Pole_Pairs/RS/LD/LQ/Ke/MOTOR_SPEED_BASE 覆盖到头板的编译期宏上。
     *   这里只保留“读 EEPROM 配置”+“初始化运行状态”两件事。*/
    DevConfig_LoadParam();        // 读 88 字节配置 → g_motor_cfg（告警阈值、speed_kp/ki）
    DevConfig_ApplySpeedPI();     // PI1_KP/PI1_KI ← g_motor_cfg->speed_kp/ki

    // 初始化运行状态
    memset(&g_run_state, 0, sizeof(g_run_state));
#if (MOTOR_POSITION_CLOSE_LOOP != 0)
    g_run_state.work_mode = EXT_WORK_MODE_POSITION;
#else
    g_run_state.work_mode = EXT_WORK_MODE_SPEED;
#endif
    g_run_state.fault_code = FAULT_NONE;   // 状态帧 Byte7 上电应为 0xFF，不是 0x00
    g_run_state.direction  = FR_MODE;      // 可选：与 mcFocCtrl.FR 对齐

    PositionCtrl_Init();                   // 清位置环状态/命令/输出
}

/// @brief 设置电机急停状态
/// @param state 0: 未急停 1: 已急停
/// @retval None
void MotorCtrl_SetEstop(uint8_t state)
{
    g_run_state.is_estop = state;
    if (state)
    {
        g_pos_cmd_start = 0;               // 位置环停止
        g_pos_cmd_raw   = 0xFFFF;          // 清目标
        g_pos_state     = POS_IDLE;        // 状态回空闲

        g_run_state.ref = 0;
        g_run_state.run_time = 0;
    }
}

/// @brief 设置电机运行状态
/// @param state 0: 停止 1: 运行
/// @retval None
void MotorCtrl_SetRun(uint8_t state)
{
    if (state == 0)
    {
        g_pos_cmd_start = 0;               // 位置环停止
        g_pos_cmd_raw   = 0xFFFF;          // 清目标
        g_pos_state     = POS_IDLE;        // 状态回空闲
        g_run_state.ref = 0;
        g_run_state.run_time = 0;
    }

    if (state == g_run_state.is_started)
        return;

    g_run_state.is_started = state;

    if (state == 1)
    {
        g_pos_cmd_start = 1;               // 允许位置环启动
    }
}

/// @brief 设置电机控制模式
/// @param mode 闭环方式选择
/// @retval None
// void MotorCtrl_SetMode(uint8_t mode)
// {
//     // 检查模式是否有效
//     if (mode < EXT_WORK_MODE_CURRENT || mode > EXT_WORK_MODE_SPEED)
//         return;

//     if (mode == g_run_state.work_mode)
//         return;

//     g_run_state.work_mode = mode;
//     if (mode == EXT_WORK_MODE_SPEED)
//     {
//         MOTOR_CTRL_MODE = SPEED_LOOP_CONTROL;
//     }
//     else if (mode == EXT_WORK_MODE_CURRENT)
//     {
//         MOTOR_CTRL_MODE = CURRENT_LOOP_CONTROL;
//     }
// }
void MotorCtrl_SetMode(uint8_t mode)
{
#if (MOTOR_POSITION_CLOSE_LOOP != 0)
    if (mode == EXT_WORK_MODE_POSITION)
    {
        if (mode == g_run_state.work_mode)
            return;

        g_run_state.work_mode = mode;
        g_run_state.ref       = 0;
        g_pos_cmd_raw         = 0xFFFF;    // 等新目标
        g_pos_state           = POS_IDLE;
        return;
    }
#endif

    if (mode != EXT_WORK_MODE_SPEED)
        return;

    if (mode == g_run_state.work_mode)
        return;

#if (MOTOR_POSITION_CLOSE_LOOP != 0)
    if (g_run_state.work_mode == EXT_WORK_MODE_POSITION)
    {
        g_pos_cmd_raw = 0xFFFF;            // 离开位置模式，清目标
        g_pos_state   = POS_IDLE;
    }
#endif

    g_run_state.work_mode = mode;
}

/// @brief 设置电机目标值(正数正转, 负数反转)
/// @param current 目标电流原始值：段字节13-14，0.01A 单位，零点 32767（无符号）
/// @param speed   目标转速原始值：段字节11-12，0.01% 单位，零点 32767（无符号）
/// @note  ★收的是协议"原始值"，两个要点：
///        ① 必须无符号读(EXT_GET_U16)。零点 32767、范围 ±100%(±10000 counts)，
///           正数 raw 一定 >32767（最高位=1）；有符号读会把正转变成大负数。
///        ② 换算要在这里做完：值 = raw - 32767（带符号）；
///           转速模式 → 值/10000 = 比例 → ×MOTOR_SPEED_BASE = rpm；
///           电流模式 → 值/100 = A。
///           因为 g_run_state.ref 的单位是 rpm / A
///           （TargetRef_Process 里用 S_Value()/I_Value()，它们期望的正是 rpm/A），
///           若把 0.01% 的原始值直接塞进 ref，S_Value() 会溢出（int16）。
/// @retval None
void MotorCtrl_SetTarget(uint16_t current, uint16_t speed, uint16_t closed_loop)
{
    int32 v;
    float ref_val;
#if (MOTOR_POSITION_CLOSE_LOOP == 0)
    (void)closed_loop;                       /* 模式 0 不用位置目标 */
#endif

#if (MOTOR_POSITION_CLOSE_LOOP != 0)
    /*
     * 位置目标不依赖 is_started 锁存：
     * 位置环可能因为“暂时没有目标”把 is_started 清 0，
     * 目标单独到达时仍要能存进去，由 g_pos_cmd_start 决定是否运动。
     */
    if ((g_run_state.work_mode == EXT_WORK_MODE_POSITION) &&
        (!g_run_state.is_estop))
    {
        g_pos_cmd_raw = closed_loop;           // 段字节15-16 才是位置目标

        /* 先按“目标 - 当前反馈”选启动方向，避免开环强拖往反方向顶限位 */
        if ((g_pos_fb_raw != 0xFFFF) && (closed_loop != g_pos_fb_raw))
        {
            if (POS_FB_DIR > 0.0f)
            {
                g_run_state.direction = (closed_loop > g_pos_fb_raw) ? CW : CCW;
            }
            else
            {
                g_run_state.direction = (closed_loop > g_pos_fb_raw) ? CCW : CW;
            }
        }
        return;
    }
#endif

    // 根据工作模式和控制状态设置电机参数
    if (g_run_state.is_started && !g_run_state.is_estop)
    {
        if (g_run_state.work_mode == EXT_WORK_MODE_SPEED)
        {
            v = (int32)speed - 32767;                            /* 0.01% 单位，带符号 */

            if (v >= 0)
            {
                g_run_state.direction = CW;                        // 正数正转
                ref_val = (float)v;
            }
            else
            {
                g_run_state.direction = CCW;                       // 负数反转
                ref_val = (float)(-v);                             // 存储绝对值
            }

            g_run_state.ref = ref_val / 10000.0f * MOTOR_SPEED_BASE;   /* 0.01% → 比例 → rpm */

            if (v == 0)
            {
                g_run_state.is_started = 0;                        /* 目标 0% → 自动停机 */
            }
        }
        else if (g_run_state.work_mode == EXT_WORK_MODE_CURRENT)
        {
            v = (int32)current - 32767;                          /* 0.01A 单位，带符号 */

            if (v >= 0)
            {
                g_run_state.direction = CW;                        // 正数正转
                ref_val = (float)v;
            }
            else
            {
                g_run_state.direction = CCW;                       // 负数反转
                ref_val = (float)(-v);                             // 存储绝对值
            }

            g_run_state.ref = ref_val / 100.0f;                    /* 0.01A → A */

            if (v == 0)
            {
                g_run_state.is_started = 0;
            }
        }
    }
}

/// @brief 更新电机状态
/// @param None
/// @retval None
void MotorCtrl_Update(void)
{

}

