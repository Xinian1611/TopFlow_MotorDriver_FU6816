/* --------------------------- (C) COPYRIGHT 2021 Fortiortech ShenZhen -----------------------------
    File Name      : MotorControlFunction.h
    Author         : Fortiortech  Appliction Team
    Version        : V1.0
    Date           : 2021-04-11
    Description    : This file contains motor contorl parameter used for Motor Control.
----------------------------------------------------------------------------------------------------
                                       All Rights Reserved
------------------------------------------------------------------------------------------------- */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTORCONTROLFUNCTION_H_
#define __MOTORCONTROLFUNCTION_H_
#include "PosCheck.h"
#include "FU68xx_6.h"
#include "Customer.h"
#include "Parameter.h"
#include "AddFunction.h"

///****************************BEMF参数变量**************************/
//#define TIM2_Fre                        (187500.0) // TIM2计数频率187.5KHz
////定义使用BEMF启动时ATO_BW值
//#define ATO_BW_BEMF_START               (400.0)

//#define OBSW_KP_GAIN_BEMF_START         _Q12(2 * _2PI * ATT_COEF * ATO_BW_BEMF_START / BASE_FREQ)
//#define OBSW_KI_GAIN_BEMF_START         _Q12(_2PI * ATO_BW_BEMF_START * ATO_BW_BEMF_START * TPWM_VALUE / BASE_FREQ)

////定义使用BEMF启动时DKI QKI值
//#define DKI_BEMF_START                  _Q12(1.0)
//#define QKI_BEMF_START                  _Q12(1.0)

////定义使用BEMF启动最低转速,ROM
//#define BEMFMotorStartSpeed             _Q15(150.0 / MOTOR_SPEED_BASE)
//#define BEMFMotorStartSpeedHigh         _Q15(4000.0 / MOTOR_SPEED_BASE)

////定义使用BEMF检测时间,ms
//#define BEMF_START_DETECT_TIME          (300)//速度快的电机，时间可以缩短到50

////定义使用BEMF启动检测延时,ms
//#define BEMF_START_DELAY_TIME           (150)//速度快的电机，时间可以缩短到15

//#define TempBEMFSpeedBase               (int32)(32767.0 / 8.0 * (TIM2_Fre * 60 / Pole_Pairs / MOTOR_SPEED_BASE))
//#define TempBEMFSpeedBase1              (int32)(32767.0 / 6.0 / 8.0 * (TIM2_Fre * 60 / Pole_Pairs / MOTOR_SPEED_BASE))


//typedef struct
//{
//    uint16 BEMFSpeed;                   //反电动势检测的速度
//    uint32 BEMFSpeedBase;               //反电动势检测的速度基准
//    uint8  BEMFStatus;                  //反电动势的状态，6拍的状态
//    uint8  FRStatus;                    //正反转
//    uint16 PeriodTime;                  //转一圈的周期计数值/8,因除数只能是16位的
//    uint16 MC_StepTime[6];              //转一拍的计数值数组
//    uint16 StepTime;                    //单拍的计数值
//    uint16 BEMFTimeCount;               //反电动势检测时间
//    uint8  FirstCycle;                  //反电动势检测第一个周期
//    uint8  BEMFStep;                    //拍的计数
//    uint8  BEMFSpeedInitStatus;         //速度初始化状态
//    uint8  FlagSpeedCal;                //速度计算标志
//    uint8  BEMFStartStatus;             //强制启动标志位
//    uint8  BEMFStartDelayStatus;        //强制启动标志位  
//    uint8  BEMFCCWFlag;                 //反转的强弱标志
//    uint8  BEMFBrakeFlag;
//}BEMFDetect_TypeDef;

//typedef struct   
//{
//  int8  BemfSetFR;
//  int8  BemfValue;                     
//  uint8 BemfNum; 
//  int8  BemfTabA,BemfTabB,RefNumZ,RefNumY,RefNumX;    
//  uint8 BemfFR;                        
//  uint8  Calcnum;                       
//  uint16  Calcnms;                       
//} Stk_TypeDef;

/* Define to prevent recursive inclusion -------------------------------------*/
typedef struct
{
  int16   IuOffset;       //Iu的偏置电压
  int32   IuOffsetSum;    //Iu的偏置电压总和
  int16   IvOffset;       //Iv的偏置电压
  int32   IvOffsetSum;    //Iv的偏置电压总和
  int16   Iw_busOffset;   //Iw或Ibus的偏置电压
  int32   Iw_busOffsetSum;//Iw或Ibus的偏置电压总和
  int16   OffsetCount;    //偏置电压采集计数
  int8    OffsetFlag;     //偏置电压结束标志位
}CurrentOffset;

//extern Stk_TypeDef        xdata   Stk;
//extern BEMFDetect_TypeDef xdata BEMFDetect;

extern CurrentOffset xdata mcCurOffset;


extern void VariablesPreInit(void);
extern void GetCurrentOffset(void);
extern void Motor_Ready (void);
extern void Motor_Init (void);

extern void FOC_Init(void);
extern void Motor_Charge(void);
extern void MC_Stop(void);
extern void MC_Break(void);
extern void Motor_Static_Open(void);
extern void Motor_FocTailWind_Open(void);
extern void SquToFOCInit(void);
extern void SquStartInit(void);

extern void Motor_Align(void);
extern void MotorcontrolInit(void);
extern void BEMFTailWindDealwith(void);
extern void Motor_TailWind(void);

#endif