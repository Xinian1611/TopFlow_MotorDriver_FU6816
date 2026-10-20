/**************************** (C) COPYRIGHT 2018 Fortiortech shenzhen *****************************
* File Name          : RSDDetect.h
* Author             : Bruce, Fortiortech Hardware
* Version            : V1.0
* Date               : 2017-12-27
* Description        : This file contains all the common data types used for
*                      Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/
#ifndef __RSDDETECT_H_
#define __RSDDETECT_H_

#include "FU68xx_6.h"
#include "Customer.h"


/** RSD参数变量 */
typedef struct
{
    uint16 StepTime[4];              ///< 一个脉冲的周期值
    uint8  Times;                    ///< 进入RSD中断次数

    uint16 Period;                   ///< 电周期值,除数不能超过16位
    int16  Count;                    ///< 当前脉冲数
    int16  CountPre;                 ///< 上一个状态脉冲数
    int16  State;                    ///< RSD的状态
    int16  Speed;                    ///< RSD的速度
    
    uint8  SpeedUpdate;              ///< 速度更新标志
    uint8  HighSpdStart;
    

    uint32 RSDSpeedBase;                ///< RSD的速度基准
    uint16 RSDCCWSBRCnt;                ///< RSD反转后启动前刹车计数
    uint8  RSDStep;                     ///< 速度捕获计数
    
    uint8 SetFR;
    uint8 Status;
    uint8 ArrCnt;
    
}MotorRSDTypeDef;

#define     RSDSpeedCalBase             (uint32)(32767*TIM2_Fre*15/Pole_Pairs/MOTOR_SPEED_BASE)
#define     RSDSpeedCalMaxSpeed         (uint16)(TIM2_Fre*15/Pole_Pairs/MOTOR_SPEED_BASE)+1



#define     FORWARD                     (1)
#define     REVERSE                     (2)
#define     STATIC                      (3)
#define     DETECTING                   (0)


extern MotorRSDTypeDef   mcRsd;
extern void RsdProcess(void);
extern void RSDDetectInit(void);


/****************************end RSD参数变量*******************************/

#endif