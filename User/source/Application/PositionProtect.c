/**
 * @file    PositionProtect.c
 * @brief   位置闭环的限位/堵转判断
 * @note    限位看位置，堵转看电流；两者分开，限位优先。
 */
#include <MyProject.h>
#include "PositionProtect.h"
#include <math.h>

#define POS_PROT_ABS_F(x)       (((x) < 0.0f) ? -(x) : (x))          /* 浮点绝对值 */
#define POS_PROT_ABS_S16(x)     ((int16)(((x) < 0) ? -(x) : (x)))    /* int16 绝对值 */
#define POS_PROT_STALL_CNT      (POS_STALL_CURRENT_MS / POS_CTRL_PERIOD_MS) /* 堵转确认需要的周期数 */

static uint16 xdata s_stallCnt;                 /* 堵转确认计数 */
static float  xdata s_iqFilt;                   /* Iq 低通滤波值，单位 A */

//
uint8 PositionProtect_LimitCheck(float target, float fb, uint8 posActive)
{
    if (!posActive)                              /* 不在位置模式，不判 */
    {
        return POS_PROTECT_NONE;
    }

    /* MAX：到软件上限，目标还要继续增大 → 立即限位 */
    if ((fb >= POS_LIMIT_MAX) && (target > fb))
    {
        return POS_PROTECT_LIMIT;
    }

    /* MIN：到软件下限，目标还要继续减小 → 立即限位 */
    if ((fb <= POS_LIMIT_MIN) && (target < fb))
    {
        return POS_PROTECT_LIMIT;
    }

    return POS_PROTECT_NONE;                     /* 没到限位，正常跑 */
}

/* ==================== 堵转判断 ==================== */
/**
 * @brief  电流幅值堵转判断
 * @retval POS_PROTECT_STALL / POS_PROTECT_NONE
 * @note   任何 MOE=1 的状态都判，不看位置/速度/闭环状态。
 *         用 sqrt(Id^2 + Iq^2)，不能只看 Iq：Id 大时会低估实际电流。
 */
uint8 PositionProtect_StallCheck(void)
{
    float id_a;                                  /* Id 实际电流，单位 A */
    float iq_a;                                  /* Iq 实际电流，单位 A */
    float i_mag;                                 /* 电流幅值，单位 A */

    if (MOE == 0)                                /* 输出没开，清状态即可 */
    {
        s_stallCnt = 0;
        s_iqFilt   = 0.0f;
        return POS_PROTECT_NONE;
    }

    /* Q15 -> A */
    id_a = (float)FOC__ID / 32767.0f * (float)HW_ADC_REF / ((float)HW_RSHUNT * (float)HW_AMPGAIN);
    iq_a = (float)FOC__IQ / 32767.0f * (float)HW_ADC_REF / ((float)HW_RSHUNT * (float)HW_AMPGAIN);

    i_mag = (float)sqrt((double)(id_a * id_a + iq_a * iq_a));

    /* 一阶低通，滤掉 PWM 开关尖峰 */
    s_iqFilt += POS_STALL_CURRENT_LPF * (i_mag - s_iqFilt);

    if (s_iqFilt > POS_STALL_CURRENT_A)          /* 15A */
    {
        if (s_stallCnt < POS_PROT_STALL_CNT)
        {
            s_stallCnt++;
        }

        if (s_stallCnt >= POS_PROT_STALL_CNT)    /* 30ms */
        {
            return POS_PROTECT_STALL;
        }
    }
    else
    {
        s_stallCnt = 0;
    }

    return POS_PROTECT_NONE;
}
