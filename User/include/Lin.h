/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      MyProject.h
 * @author    Fortiortech Appliction Team
 * @note      Last modify author is Kris.huang
 * @since     2017-12-27
 * @date      2022-07-14
 * @brief     This file contains all the common data types used for Motor Control.
 *
 */

#ifndef LIN_H
#define LIN_H

#include "FU68xx_6_Type.h"
#include <FU68xx_6_MCU.h>


#define ID_TX 1             /* LIN发送帧 */
#define ID_RX 2             /* LIN接收帧 */

#define LINRW_R ClrBit(LIN_CR, LINRW) /* 设置为接收 */
#define LINRW_W SetBit(LIN_CR, LINRW) /* 设置为发送 */

#define LIN_ACK SetBit(LIN_CSR, LINACK);      /* 应答帧头，该位置 1 前必须将校验模式, 读写模式,数据, 数据长度准备好 */
#define CheckEnhanced ClrBit(LIN_CR, CHKMOD); /* 增强校验模式 */
#define CheckClass SetBit(LIN_CR, CHKMOD);    /* CLASS校验模式 */

#define LIN_Message_OverTime								(10)			// 单帧超时时间			
#define LIN_Bus_OverTime										(10000)			// 单帧超时时间			


/* set the size of data, SIZE: 0~8*/
#define LIN_SETSIZE(SIZE) LIN_SIZE = SIZE;

typedef struct
{
    uint8 Counter;
    uint8 Error;
		uint8 State;				// 0: Idle	1:Busy
    uint8 RData[10];
    uint8 SData[10];
    uint8 SData3D[10]; // 诊断回复数据
    uint8 Pack;
    uint8 NAD;
		uint16 OverTimeCnt;
		uint16 BusLostCnt;
} LINState;
extern LINState xdata LS;

typedef struct
{
  /* 接收到的数据0x2A */
  uint8  EnableReq_ECF;  // 使能信号，无使能，即使有转速请求也不工作
  uint16 SpeedReq_ECF;    // ECF发送目标转速
    
  /* 回复的数据0x2E */
  //Byte[0]
  uint8 ECF_EnableSt;   // 返回实际转速
  uint8 ECF_WorkingSt; //
  uint8 ECF_WorkingMode; //
  //Byte[1]
   uint8 ECF_Speed ;            //反馈转速
  //Byte[2]
   uint8 ECF_Voltage;           //反馈电压
  //Byte[3,4]
   uint16 ECF_Current;           //反馈电流
  //Byte[5,6]
   uint16 ECF_Power;             //反馈功率
   //Byte[7]
   uint8 ECF_FaultSt;             //控制器有无故障标志：  
   uint8 ECF_CommunicationError;  //通信状态标志  
   uint8 ECF_VoltageError;      // 电压故障
   uint8 ECF_UnderVoltageError;      // 欠压故障
   uint8 ECF_OverVoltageError;      // 过压故障
   uint8 ECF_OverTempError;
   uint8 ECF_HardwareError;
   uint8 ECF_StallError;
   uint8 ECF_StallErrorCheck;   // 堵转故障确认
   uint8 ECF_OverCurrentError;
   uint8 ECF_LinLostError;  // 丢帧，LIN故障
     
   uint8  TxCnt;            
     
} LINdata;


extern void LIN_Init(void);        /* LIN初始化 */
extern void LIN_list(void);        /* 接收调度表 */
extern void LIN_Error_Handing(void); /* 错误查询 */
extern void ReceiveSYS(uint8 PID); /* 接收的数据处理 */
extern uint8 Erase_Flage;

/* 用户LIN的接收发送协议数据处理  */
extern LINdata xdata EOP;
extern void EOP_TXLIN(void);
extern void EOP_RXLIN(void);
extern uint8 loc_linBusTimeoutFlag;
extern uint16 loc_linBusTimeoutCnt;
#endif