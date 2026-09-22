/* --------------------------- (C) COPYRIGHT 2020 Fortiortech ShenZhen -----------------------------
    File Name      : AddFunction.h
    Author         : Fortiortech  Appliction Team
    Version        : V1.0
    Date           : 2021-11-07
    Description    : This file contains all the common data types used for Motor Control.
----------------------------------------------------------------------------------------------------
                                       All Rights Reserved
------------------------------------------------------------------------------------------------- */
/* Define to prevent recursive inclusion -------------------------------------*/


#ifndef __AddFunction_H_
#define __AddFunction_H_

/******************************************************************************/
#include <FU68xx_6.h>
/******************************************************************************/

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    uint32 Times;                                                         // 启停次数
    uint16 TimeCnt;                                                       // 运行时间计数
    uint8  State;                                                         // 启停状态
} debugONOFFTypeDef;

typedef struct
{
    int16  mcDcbusFlt;                                                          // 母线电压
    uint16 mcADCCurrentbus;                                                     // 母线电流  
    int16  Angle_Com;
    int16  NTCTempFlt;                                                            // NTC电压值
    int16  Power;                                                               // 当前功率
    int16  PowerFlt;                                                            // 功率滤波后的值
    int16  SpeedFlt;                                                            // 当前速度滤波后的值
    int16  SpeedFlt_Pre;
    int16  UqFlt;                                                               // Q轴电压 
    int16  UdFlt;                                                               // D轴电压
    
    int16  ud_align_ref;
    int16  ud_align;
    int16  ud_align_flag;
    int16  id_align_sample;
    int16  id_align_sample_pre;
    int16  id_align_sample_error;
    int16  align_theta_acc;
    int16  AlignAngle;                                                          // 预定位角度
    int16  AlignAngle_Temp;
	int16  Align_Count;
    
    uint8  ChargeStep;                                                          // 预充电的步骤
    uint8  Start_Mode;                                                          // 启动方式        
    uint8  CtrlMode;                                                            // 控制模式
    
    int16  Max_ia;
    int16  Max_ib;
    int16  Max_ic;
    
    uint8  FR;
    uint8  Flg_ATORampEnd;
    
    int16  Ref;                                                                 // 控制目标给定
    int16  IqRef;                                                               // Q轴给定电流
    int16  IdRef;                                                               // D轴给定电流
    int16  IqSpeedRef;                                                            // Q轴速度给定电流
    int16  IqADCCurrentRef;                                                      // Q轴母线电流给定电流
    int16  WeakRef;                                                              //弱磁给定电流
    int16  Angle;                                                             //弱磁角度
    uint16 sqrtUdq;                                  
    
    uint8  Mode0HoldCnt;                                                        // MODE0切MODE1延迟时间计数   
   
    uint16 EsValue;                                                             // 当前FOC_ESQU滤波后的值

    uint16 RunStateCnt;                                                         // 运行状态计数
   
    uint16 LoopTime;                                                            // 外环控制周期计数
   
    uint16 State_Count;                                                         // 电机各个状态的时间计数

    int16  mcPosCheckAngle;                                                   // 位置检测的角度

    uint8  MCU_TEMP;                                                            //MCU超过70°后，读取当前温度值
    int16  MDU_TEMP;
    
}FOCCTRL;



typedef struct
{
    int16  RefValue;
    float  OutValue_float;
    float  IncValue;
    float  DecValue;   
}MCRAMP;


typedef struct
{
    uint16 Compare;             // 比较值
    uint16 Period;              // 周期值
    
    uint16 TimerDR;
    uint16 TimerDROld;
    uint16 TimerARR;            // 周期值
    uint16 TimerARROld;         // 周期值
    
    uint16 Duty;                // PWM占空比，Q15格式
    uint16 Freq;                // PWM频率
    
    uint8  isUpdate;            // PWM新的duty更新
    
    uint8  MotorOnFilter;       //开机滤波
    uint8  MotorOffFilter;      //关机滤波
    
}PWMINPUTCAL;

typedef struct
{
    uint32 SleepDelayCout;          //休眠计数
    int8  SleepFlag;
}SLEEPMODE;

/* Exported variables ---------------------------------------------------------------------------*/


extern bool                 data  isCtrlPowOn;
extern debugONOFFTypeDef    xdata debug_ONOFFTest;
extern FOCCTRL              xdata mcFocCtrl;
extern PWMINPUTCAL          xdata mcPwmInput;
extern MCRAMP               xdata mcRefRamp;
extern SLEEPMODE   xdata   SleepSet;
extern uint16 Power_Currt;
extern uint8 Flag_Single_Mode;

/* Exported functions ---------------------------------------------------------------------------*/

extern void   VariablesPreInit(void);

extern void   TargetRef_Process(void);
extern void   Speed_response(void);
extern int16   Motor_Ramp(int16 ref);

extern void   VSPSample(void);
extern void   ONOFF_Starttest(void);
extern void   ATORamp(void);

extern void   TickCycle_1ms(void);

extern void   PWMInputCapture(void);

extern uint32 Abs_F32(int32 value);

/******* Sleep控制**************/
extern void SleepMode_Init(void);
extern void SleepMode(void);
extern void sc_goToSleep(void);
/******* **********************/
extern void SinCal(int16 Ref, int16 Theta, int16 * Sin, int16 * Cos);
extern uint16 SqrtUDQ(int16 sqrtUd, int16 sqrtUq);

#endif