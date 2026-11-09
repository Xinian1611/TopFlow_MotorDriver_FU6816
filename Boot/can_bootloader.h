/**
  ******************************************************************************
  * @file    can_bootloader.h
  * $Author: wdluo $
  * $Revision: 17 $
  * $Date:: 2012-07-06 11:16:48 +0800 #$
  * @brief   基于CAN总线的Bootloader程序.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_BOOTLOADER_H
#define __CAN_BOOTLOADER_H
/* Includes ------------------------------------------------------------------*/
#include <FU68xx_6.h>       
#include "boot_flash.h" 
#include "can_compat.h"
#include "crc16.h"

#define APP_EXE_FLAG_START_ADDR    ((uint32_t)0x1700)   //0x08000000
#define APP_START_ADDR             ((uint32_t)0x1800)   //0x08009000
#define APP_EXE_FLAG_ADDR          ((uint32_t)0x1700)   //0x08008000
#define APP_END_ADDR               ((uint32_t)0x7F00)   //0x08040000

#define CMD_WIDTH 4

#define EXE_WAIT_TIME   60*1000*3   // 60s

extern uint16_t Station_ID;
extern uint8_t CAN1_CanRxMsgFlag;

//#define IPADataType_EEPROM
#define IPADataType_FLASH
//IAP数据在EEPROM中存储的位置定义--------------------------------------------
#define IAPFlag_EEPROM_Page                0        //IAP更新标志在EEPROM中存储页
#define IAPID_EEPROM_Page                  1        //IAPID在EEPROM中存储页

typedef struct _CBL_CMD_LIST{
  //Bootloader相关命令
  unsigned char Erase;        //擦出APP储存扇区数据
  unsigned char WriteInfo;    //设置多字节写数据相关参数（写起始地址，数据量）
  unsigned char Write;        //以多字节形式写数据
  unsigned char Check;        //检测节点是否在线，同时返回固件信息
  unsigned char SetBaudRate;  //设置节点波特率
  unsigned char Excute;       //执行固件
  //节点返回状态
  unsigned char CmdSuccess;   //命令执行成功
  unsigned char CmdFaild;     //命令执行失败
} CBL_CMD_LIST,*PCBL_CMD_LIST; 

#define ExcuteFlg_Offset   0
#define ExcuteFlgSize      5
#define StationID_Offset   8
#define StationIDSize      7

void CAN_BOOT_JumpToApplication(uint32_t Addr);
void CAN_BOOT_Command(void);
void CMD_Excute(void);
uint16_t CAN_BOOT_GetAddrData(void);
FLASH_Status CAN_BOOT_ProgramDatatoFlash(uint16_t StartAddress, uint8_t *pData, uint16_t DataNum);
FLASH_Status CAN_BOOT_ErasePage(uint16_t StartPageAddr, uint16_t EndPageAddr);

/* Robert_EEPROM.c（参数页）*/
uint8_t  CheckXor(uint8_t *strData,int len);
uint8_t  ReadExcuteFlg(void);
uint32_t ReadCANID(void);
void     WriteExcuteFlg(void);

#endif




