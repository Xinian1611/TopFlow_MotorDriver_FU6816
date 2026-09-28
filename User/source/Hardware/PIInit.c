/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      xxx.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2022-08-05
 * @date      Last modify:2022-08-05
 * @note      Last modify author is Leo.li
 * @brief
 */
 
/* Includes -------------------------------------------------------------------------------------*/
#include <MyProject.h>
#include "DevConfig.h"


/** 
 * @brief        PI初始化,PI1用于外环
 * @date         2022-09-13
 */
void PI_Init(void)
{
    if (g_motor_cfg != NULL)
    {
        PI1_KP          = g_motor_cfg->speed_kp;
        PI1_KI          = g_motor_cfg->speed_ki;
    }
    else
    {
        PI1_KP          = SKP;
        PI1_KI          = SKI;
    }
    PI1_EK1         =0;
    PI1_EK          =0;
    PI1_UKH         =0;
    PI1_UKL         =0;
    PI1_UKMAX       = SOUTMAX;
    PI1_UKMIN       = SOUTMIN;
}

/**
    @brief        PI2用于母线限流
    @date         2022-07-14
*/
void PI2_Init(void)
{
    PI2_KP          = LimitCurrent_KP;
    PI2_KI          = LimitCurrent_KI;
    PI2_EK1         =0;
    PI2_EK          =0;
    PI2_UKH         =0;
    PI2_UKL         =0;
    PI2_UKMAX       = QOUTMAX;
    PI2_UKMIN       = 0;
    PI2_KD          = 0;             
    PI2_EK2         = 0;                    
}
/**
    @brief        PI3用于弱磁增速
    @date         2022-07-14
*/
void PI3_Init(void)
{
    PI3_KP          = AKP;
    PI3_EK1         = 0;
    PI3_EK          = 0;
    PI3_KI          = AKI;
    PI3_UKH         = 0;
    PI3_UKL         = 0;
    PI3_UKMAX       = A_Value(AMAX);
    PI3_UKMIN       = A_Value(AMIN);
}
