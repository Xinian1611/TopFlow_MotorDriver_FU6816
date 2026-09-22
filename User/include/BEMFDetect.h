/**************************** (C) COPYRIGHT 2018 Fortiortech shenzhen *****************************
* File Name          : BEMFDetect.h
* Author             : Bruce, Fortiortech Hardware
* Version            : V1.0
* Date               : 2017-12-27
* Description        : This file contains all the common data types used for
*                      Motor Control.
***************************************************************************************************
* All Rights Reserved
**************************************************************************************************/
#ifndef __BEMFDETECT_H_
#define __BEMFDETECT_H_

#include <FU68xx_6.h>
#include <Customer.h>

/****************************BEMF参数变量**************************/

#define BEMF_FR_CCW                           (0xA0) // 
#define BEMF_FR_CW                            (0xB0) // 
#define BEMF_FR_ERR                           (0xC0) // 


#define  BEMF_FORWARD                           (1)
#define  BEMF_REVERSE                           (2)
#define  BEMF_STATIC                            (3)
#define  BEMF_DETECTING                         (0)

#define BEMFSpeedCalBase                        (int32)(32767.0 / 8.0 * (TIM2_Fre * 60 / Pole_Pairs / MOTOR_SPEED_BASE))



#define BEMFSpeedCalMinPeriod                   (int32)(1.0/8.0 * (TIM2_Fre * 60 / Pole_Pairs / MOTOR_SPEED_BASE)) + 1




//定义使用BEMF启动时ATO_BW值
#define ATO_BW_BEMF_START               (400.0)

#define OBSW_KP_GAIN_BEMF_START         _Q12(2 * _2PI * ATT_COEF * ATO_BW_BEMF_START / BASE_FREQ)
#define OBSW_KI_GAIN_BEMF_START         _Q12(_2PI * ATO_BW_BEMF_START * ATO_BW_BEMF_START * TPWM_VALUE / BASE_FREQ)

//定义使用BEMF启动时DKI QKI值
#define DKI_BEMF_START                  _Q12(1.0)
#define QKI_BEMF_START                  _Q12(1.0)

//定义使用BEMF启动最低转速,ROM
#define BEMFMotorStartSpeed             _Q15(2500.0 / MOTOR_SPEED_BASE)
#define BEMFMotorStartSpeedHigh         _Q15(12000.0 / MOTOR_SPEED_BASE)


typedef struct
{
    int16  BEMFSpeed;                   //反电动势检测的速度
    uint32 BEMFSpeedBase;               //反电动势检测的速度基准
    uint8  Status;                      // bemf检测状态
    
    
    uint8  FR_SET;
    uint8  FR ;                         // 当前转向
    uint8  FRPre;                       // 上一次转向
    int8   FRCount;                       // 正确转向计数，正数为CW，负数为CCW
    
    uint8  SpeedUpdate;              ///< 速度更新标志
    uint8  HighSpdStart;
    
   
    uint16 PeriodTime;                  //转一圈的周期计数值/8,因除数只能是16位的
    uint16 SectorTime[6];               // 60度扇区时间
}BEMFDetect_TypeDef;



extern BEMFDetect_TypeDef xdata mcBemf;

extern void BEMFDetectInit(void);

extern void BemfProcess(void);

extern void BEMFFOCCloseLoopStart(void);

#endif