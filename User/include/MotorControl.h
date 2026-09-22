/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      MotorControl.h
 * @author    Fortiortech  Appliction Team
 * @since     Create:2021-04-11
 * @date      Last modify:2022-07-14
 * @note      Last modify author is Kris.huang
 * @brief       
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTORCONTROL_H_
#define __MOTORCONTROL_H_


/* Exported types -------------------------------------------------------------------------------*/


typedef enum
{
    mcReady     = 0,    ///< 准备状态,该状态电机空闲，等待控制命令 
    mcInit      = 1,    ///< 初始化,该状态进行启动前的变量初始化
    mcCharge    = 2,    ///< 预充电,电机启动前给自举电容充电，一般用于高压驱动，低压驱动一般不需要
    mcTailWind  = 3,    ///< 顺逆风检测,该状态下电机进行顺逆风检测 
    mcPosiCheck = 4,    ///< 初始位置检测
    mcAlign     = 5,    ///< 预定位
    mcStart     = 6,    ///< 启动，用于配置启动代码
	  mcSquStart  = 7,    ///< 免调试启动
    mcRun       = 8,    ///< 运行，
    mcStop      = 9,    ///< 停止
    mcFault     = 10,    ///< 故障状态
    mcBrake     = 11,   ///< 刹车
} MotStateType;



typedef union
{
    uint8 SetMode;                                                              ///< 整个配置模式使能位
    struct
    {
        uint8 CalibFlag        :1;                                              ///< 电流校准的标志位
        uint8 ChargeSetFlag    :1;                                              ///< 预充电配置标志位
        uint8 AlignSetFlag     :1;                                              ///< 预定位配置标志位
        uint8 TailWindSetFlag  :1;                                              ///< 顺逆风配置标志位
        uint8 StartSetFlag     :1;                                              ///< 启动配置标志位
        uint8 PosiCheckSetFlag :1;                                              ///< 位置检测配置标志位
        uint8 Break            :1;                                              ////</ 刹车配置标志位
    } SetFlag;
}MotStaM;



/* Exported variables ---------------------------------------------------------------------------*/
extern MotStateType data mcState;

extern MotStaM    McStaSet;

/* Exported functions ---------------------------------------------------------------------------*/
extern void MC_Control(void);
extern void MotorcontrolInit(void);
extern void McTailWindDealwith(void);

extern void TailWindDealwith(void);

#endif
