/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      Protect.h
 * @author    Kris.huang
 * @note      Last modify author is Kris.huang
 * @since     2022-07-01
 * @date      2022-07-14
 * @brief     This file contains protection parameter used for Motor Control. 
 */
 

/* Define to prevent recursive inclusion -------------------------------------------------------- */
#ifndef __PROTECT_H_
#define __PROTECT_H_


/*保护参数设置--------------------------------------------------------------------------------------------*/
/* 硬件过流保护比较值来源 */
#define COMPARE_DAC_MODE                    (0)                                     ///< DAC设置硬件过流值
#define COMPARE_HW_MODE                     (1)                                     ///< 硬件设置硬件过流值
#define COMPARE_MODE                        (COMPARE_DAC_MODE)                      ///< 硬件过流值的来源
#define HWOCValue                           (150.0)                                  ///< (A) DAC模式下的硬件过流值

/* Faults processing Enable */
#define SWCurrentProtectEn                  (0)                                     ///< 相电流软件过流保护使能： 0,不使能；1，使能
#define VoltageProtectEn                    (0)                                     ///< 过欠压保护使能：   0,不使能；1，使能
#define PhaseLossProtectEn                  (0)                                     ///< 缺相保护使能：     0,不使能；1，使能
#define OverTPProtectEn                     (0)                                     ///< 过温保护使能：     0,不使能；1，使能
#define StallProtectEn                      (0)                                     ///< 堵转失速保护使能： 0,不使能；1，使能
#define OffsetProtectEn                     (0)                                     ///< 偏置电压保护：     0,不使能；1，使能
#define LVWProtectEn                        (0)                                     ///< 硬件低压预警(LVW)保护使能：0,不使能；1，使能

/*  保护重启参数设置  */
#define OC_RecoveryTimes                    (5)                                 // 重启次数,设定值<255 达到重启次数后不再重启，设定值>=255，一直重启
#define OC_RecoveryDelayTime                (5000)                              // (ms)重启延迟时间

#define OV_RecoveryTimes                    (255)                                // 重启次数,设定值<255 达到重启次数后不再重启，设定值>=255，一直重启
#define OV_RecoveryDelayTime                (500)                                // (ms)重启延迟时间

#define LP_RecoveryTimes                    (5)                                  // 重启次数,设定值<255 达到重启次数后不再重启，设定值>=255，一直重启
#define LP_RecoveryDelayTime                (5000)                               // (ms)重启延迟时间

#define OT_RecoveryTimes                    (5)                                   // 重启次数,设定值<255 达到重启次数后不再重启，设定值>=255，一直重启
#define OT_RecoveryDelayTime                (5000)                                // (ms)重启延迟时间

#define Stall_RecoveryTimes                 (5)                                   // 重启次数,设定值<255 达到重启次数后不再重启，设定值>=255，一直重启
#define Stall_RecoveryDelayTime             (5000)                                // (ms)堵转重启延迟时间

#define Offset_RecoveryTimes                 (5)                                   // 重启次数,设定值<255 达到重启次数后不再重启，设定值>=255，一直重启
#define Offset_RecoveryDelayTime             (5000)                                // (ms)偏置电压重启延迟时间
/* 软件保护恢复使能 */
#define ProtectRecoveryEn                   (0)

/* 软件过流保护参数设置 */
#define SW_OC_CurrentVal                    I_Value(80.0)                            ///< 相电流软件过流值

/* 直流母线电压保护参数设置值 */
#define OVER_VOLTAGE_PROTECT                UDC_Value(42)                          ///< (V) 直流母线电压过压保护值
#define UNDER_VOLTAGE_PROTECT               UDC_Value(30.0)                          ///< (V) 直流母线电压欠压保护值

#define OVER_VOLTAGE_RECOVER                UDC_Value(36)                          ///< (V) 直流母线电压过压保护恢复值            
#define UNDER_VOLTAGE_RECOVER               UDC_Value(32)                          ///< (V) 直流母线电压欠压保护恢复值

/* 堵转保护参数设置值 */
#define STALL_MAX_SPEED                     S_Value(4500)
#define STALL_MIN_SPEED                     S_Value(100)

#define EsThresholdValue                   (500.0)                                  
#define EsThresholdSpeed                    S_Value(300.0)                          ///< (RPM) 电机转速
#define SPEED_BEMF_RATIO                    (EsThresholdSpeed/EsThresholdValue)     ///< () 转速与反电动势的比值

/* 缺相保护参数设置值 */
#define PhaseLossCurrentValue               I_Value(0.6)                            ///< (A) 缺相电流值

/* -----TSD过温保护----- */
#define TempProtValue                       150                                    ///< （℃）保护温度
#define TempRecoverValue                    125                                    ///< （℃）恢复温度

/* -----NTC过温保护----- */
#define TemperatureProtectTime 			(1000)									     // (ms)温度保护检测时间
#define OVER_Temperature 		        Tempera_Value(1.05)						     // 过温保护阈值，根据NTC曲线设定，10K上拉电阻，80℃
#define UNDER_Temperature          		Tempera_Value(1.19)						     // 过温保护恢复阈值，根据NTC曲线设定，10K上拉电阻，70℃

/* -----偏置电压保护----- */
#define GetCurrentOffsetValue             _Q14(0.3/(HW_ADC_REF/2.0))               // (%)偏置电压保护误差范围，超过该范围保护,0.2V+10mV*10倍=0.3V   




/* Exported types ------------------------------------------------------------ */

typedef enum
{
    FaultNoSource           = 0,  ///< 无故障                                                  
    FaultHardOVCurrent      = 1,  ///< 硬件过流
    FaultSoftOVCurrent      = 2,  ///< 软件过流    
    FaultOverVoltageDC      = 3,  ///< 过压                                                 
    FaultUnderVoltageDC     = 4,  ///< 欠压
    FaultPhaseLost          = 5,  ///< 缺相
    FaultStall              = 6,  ///< 堵转                                                  
    FaultNtcOTErr           = 7,  ///< NTC过温
    FaultTSD                = 8,  ///< MCU内部过温
    FaultOffset             = 9, ///< 偏置电压保护   
    
} FaultStateType;


typedef struct
{
    uint8 OverCurACnt;                                                           
    uint8 OverCurBCnt; 
    uint8 OverCurCCnt;                                                        
	            											
	
    uint8 HDOC_Times;
    uint8 HDOC_DectTimeCnt;
    
}FaultCurrentVarible;


typedef struct
{
   uint16 TSDDetecCnt;    
   uint16 NTCDetecCnt;    
	
}FaultTemperatureVarible;


typedef struct
{  
    uint16 Lphasecnt;                                                          
    uint16 AOpencnt ;                                                           
    uint16 BOpencnt ;                                                           
    uint16 COpencnt ;                                                           
    uint16 ABCOpenCnt;															
    uint16 mcLossPHRecCount;                                                 
    uint16 DectDealyCnt;
    
    
    
}FaultPhaseLossVarible;


typedef struct
{
    uint16 OverVoltDetecCnt;                                                     
    uint16 UnderVoltDetecCnt; 
  
    uint16 VoltRecoverCnt;                                                    
		
	uint16 BusVoltDetecCnt;													     
	  
    uint16 DectDealyCnt;
    
    uint16 OverVoltageVal;
    uint16 UnderVoltageVal;
  
}FaultVoltageVarible;




typedef struct
{        
    uint16 DectDealyCnt;    
    uint16 EsDectCnt;  
    uint16 SpeedAndEsDectCnt;            
    uint16 SpeedMinCnt;  
    uint16 Mode0DectCnt;       
    
    int16 OverSpeedVal;
    int16 UnderSpeedVal;
    uint8  Type;
		uint16 StartDealyCnt;
	  uint8 StallModeFlag;
  
}FaultStallTypedef;

typedef struct
{
    uint16 DC_DelayTcnt;
    uint16 LP_DelayTcnt;
    uint16 OT_DelayTcnt;
    uint16 SWOC_DelayTcnt;
    uint16 Stall_DealyTcnt;
    uint16 Offset_DealyTcnt;
    
    uint8  OV_Times; 
    uint8  OT_Times;
    uint8  LP_Times;
    uint8  SWOC_Times;
    uint8  Stall_Times;
    uint8  Offset_Times;
    
}FaultRecoverTypedef;



typedef struct
{
    FaultPhaseLossVarible       PhaseLoss;
    FaultVoltageVarible         Voltage;
    FaultStallTypedef           Stall;
    FaultTemperatureVarible     Temperature;

}FaultVarible;




/* Exported variables ---------------------------------------------------------------------------*/

extern FaultStateType               data mcFaultSource;
extern FaultVarible                 xdata     Fault;
extern FaultCurrentVarible          idata	  mcCurVarible;
extern FaultRecoverTypedef          xdata   Restart;

/* Exported functions ---------------------------------------------------------------------------*/
extern void   Fault_Stall(void);
extern void   Fault_Temperature(void);
extern void   Fault_Voltage(void);
extern void   Fault_PhaseLoss(void);
extern void   Fault_Detection(void);
extern void   Fault_GetCurrentOffset(void);
extern void   Fault_Recovery(void);
#endif

