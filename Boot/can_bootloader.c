/**
  ******************************************************************************
  * @file    can_bootloader.c
  * $Author: wdluo $
  * $Revision: 17 $
  * $Date:: 2012-07-06 11:16:48 +0800 #$
  * @brief   基于CAN总线的Bootloader程序.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
#include "can_bootloader.h"

/* Private typedef -----------------------------------------------------------*/
typedef  void (*pFunction)(void);
uint8_t CAN1_CanRxMsgFlag=0;//接收到CAN数据后的标志

uint16_t Station_ID = 0;
static uint8_t xdata Store_Temp[1026];

/* 顺序必须与 struct _CBL_CMD_LIST 的成员顺序一致：
   Erase / WriteInfo / Write / Check / SetBaudRate / Excute / CmdSuccess / CmdFaild */
CBL_CMD_LIST CMD_List =
{
    0x03,   /* Erase       擦除APP区域数据 */
    0x05,   /* WriteInfo   写起始地址、数据量 */
    0x06,   /* Write       多字节写数据 */
    0x01,   /* Check       在线检测，回固件信息 */
    0x04,   /* SetBaudRate 设置节点波特率（本工程未实现）*/
    0x09,   /* Excute      执行固件 */
    0x0A,   /* CmdSuccess  命令执行成功 */
    0x00    /* CmdFaild    命令执行失败 */
};

extern CBL_CMD_LIST CMD_List;
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static uint16_t start_addr = APP_START_ADDR;
static uint16_t data_size  = 0;
static uint16_t data_index = 0;
static uint8_t  writeInfoDone = 0;   /* 0：还没收到合法的 WriteInfo */

/**
  * @brief  将数据烧写到指定地址的Flash中 。
  * @param  Address Flash起始地址。
  * @param  Data 数据存储区起始地址。
  * @param  DataNum 数据字节数。
  * @retval 数据烧写状态。
  */
FLASH_Status CAN_BOOT_ProgramDatatoFlash(uint16_t StartAddress,uint8_t *pData,uint16_t DataNum)
{
	FLASH_Status FLASHStatus = FLASH_COMPLETE;
	uint16_t i=0,len=0;

	/* 地址校验以APP地址参考 */
	if(StartAddress < APP_EXE_FLAG_START_ADDR)
		return FLASH_ERROR_PG;

	/* 块固定 1024 字节（4 的倍数），按 4 字节写即可，与公司方案一致 */
	len = (uint16_t)(DataNum >> 2);
	FLASH_Unlock();
	SetBit(WDT_CR, WDTRF);
	for(i=0;i<len;i++)
	{
		FLASHStatus = FLASH_ProgramWord(StartAddress, *((uint32_t*)pData));
		SetBit(WDT_CR, WDTRF);
		if (FLASHStatus == FLASH_COMPLETE)
		{
			StartAddress += 4;
			pData += 4;
		}
		else
		{
			FLASH_Lock();
			return FLASHStatus;
		}
	}
	FLASH_Lock();
	return FLASHStatus;
}

/**
  * @brief  擦出指定扇区区间的Flash数据 。
  * @param  StartPage 起始扇区地址
  * @param  EndPage 结束扇区地址
  * @retval 扇区擦出状态  
  */
FLASH_Status CAN_BOOT_ErasePage(uint16_t StartPageAddr,uint16_t EndPageAddr)
{
	uint16_t NbPages;
	uint16_t i=0,j=0;
	FLASH_Status FLASHStatus=FLASH_COMPLETE;

	/* ---- ① 参数扇区 0x0F00（不满 1 页，领导原文的页循环覆盖不到）---- */
	for(j = StartPageAddr; (j < EndPageAddr) && (j < APP_START_ADDR); j += 0x0100)
	{
		SetBit(WDT_CR, WDTRF);
		FLASHStatus = FLASH_EraseSector(j);
	}

	/* ---- ② APP 主体按 2KB 页擦（0x1000 ~ 0x77FF）---- */
	{
		uint16_t pageEnd = (EndPageAddr > 0x7800) ? 0x7800 : EndPageAddr;
		NbPages = (pageEnd - APP_START_ADDR) / 0x800;

		FLASH_Unlock();
		for(i = 0;i < NbPages;i++)
		{
			SetBit(WDT_CR, WDTRF);                       /* 原 delayus(100) */
			j = (APP_START_ADDR + (0x800 * i));
			FLASHStatus = FLASH_ErasePage(j);
		}
		FLASH_Lock();
	}

	/* ---- ③ 尾部 0x7800~0x7EFF（不满 1 页）---- */
	if(EndPageAddr > 0x7800)
	{
		for(j = 0x7800; j < EndPageAddr; j += 0x0100)
		{
			SetBit(WDT_CR, WDTRF);
			FLASHStatus = FLASH_EraseSector(j);
		}
	}

	return FLASHStatus;
}

/**
  * @brief  获取节点地址信息
  * @param  None
  * @retval 节点地址。
  */
uint16_t CAN_BOOT_GetAddrData(void)
{
	return Station_ID;
}
/**
  * @brief  控制程序跳转到指定位置开始执行 。
  * @param  Addr 程序执行地址。
  * @retval 程序跳转状态。
  */
void CAN_BOOT_JumpToApplication(uint32_t Addr)
{
	if(*(uint8_t code *)Addr != 0x02)      /* 0x02 = LJMP 操作码：APP 起点必须是 LJMP */
		return;

	EA = 0;
	(*(void (*)())Addr)();                 /* 与厂商 PADDR 跳法一致 */
}

void CMD_Check(void)
{
	struct CANTxStc Cantest;
	Cantest.IdentifyID.all = (CAN_BOOT_GetAddrData()<<CMD_WIDTH)|CMD_List.Check;
	Cantest.MsgFlag.bitf.SendReadyFlag = 0;
	Cantest.MsgFlag.bitf.SendFinishFlag = 0;
	Cantest.MsgFlag.bitf.SendEnableFlag = CAN_Enable;
	Cantest.MsgBoxNum = CPDD_CANTxStatus_NoMailBox;
	Cantest.MsgConfig.bitf.DataType = CPDD_CANDataType_FrameData;
	Cantest.MsgConfig.bitf.IdentifyType = CPDD_CANType_Extended;
	Cantest.Buff[0] = 0;
	Cantest.Buff[1] = 1;
	Cantest.Buff[2] = 0;
	Cantest.Buff[3] = 0;
	Cantest.Buff[4] = 0;
	Cantest.Buff[5] = 0;
	Cantest.Buff[6] = 0;
	Cantest.Buff[7] = 0;
	
	CANSendMsgCheck(0,&Cantest,1);
}

void CMD_Excute(void)
{
	if((*(uint8_t code *)APP_START_ADDR) == 0x02)   /* 必须是 LJMP 才算有效 APP */
    {
        if(ReadExcuteFlg() == 1) WriteExcuteFlg();
        CAN_BOOT_JumpToApplication(APP_START_ADDR);
    }
}

void CMD_Erase(void)
{
	struct CANTxStc TxMessage;
	FLASH_Status ret;
	uint32_t startAddr,EndAddr;
	
	TxMessage.MsgFlag.bitf.SendReadyFlag = 0;
	TxMessage.MsgFlag.bitf.SendFinishFlag = 0;
	TxMessage.MsgFlag.bitf.SendEnableFlag = CAN_Enable;
	TxMessage.MsgBoxNum = CPDD_CANTxStatus_NoMailBox;
	TxMessage.MsgConfig.bitf.DataType = CPDD_CANDataType_FrameData;
	TxMessage.MsgConfig.bitf.IdentifyType = CPDD_CANType_Extended;
	
	startAddr = APP_START_ADDR;
	EndAddr = APP_END_ADDR;
	ret = CAN_BOOT_ErasePage(startAddr,EndAddr);
	
	start_addr = APP_START_ADDR;
	if(ret==FLASH_COMPLETE)
	{
		TxMessage.IdentifyID.all = (CAN_BOOT_GetAddrData()<<CMD_WIDTH)|CMD_List.Erase;
	}
	else
	{
		TxMessage.IdentifyID.all = (CAN_BOOT_GetAddrData()<<CMD_WIDTH)|CMD_List.CmdFaild;
	}
	CANSendMsgCheck(0,&TxMessage,1);
}

void CMD_WriteInfo(void)
{
	uint16_t i=0;
	uint8_t Data[8]={0,0,0,0,0,0,0,0};
	struct CANTxStc TxMessage;
	
	TxMessage.MsgFlag.bitf.SendReadyFlag = 0;
	TxMessage.MsgFlag.bitf.SendFinishFlag = 0;
	TxMessage.MsgFlag.bitf.SendEnableFlag = CAN_Enable;
	TxMessage.MsgBoxNum = CPDD_CANTxStatus_NoMailBox;
	TxMessage.MsgConfig.bitf.DataType = CPDD_CANDataType_FrameData;
	TxMessage.MsgConfig.bitf.IdentifyType = CPDD_CANType_Extended;
	
	TxMessage.Buff[0] = 0;
	TxMessage.Buff[1] = 0;
	TxMessage.Buff[2] = 0;
	TxMessage.Buff[3] = 0;
	TxMessage.Buff[4] = 0;
	TxMessage.Buff[5] = 0;
	TxMessage.Buff[6] = 0;
	TxMessage.Buff[7] = 0;
	
	for(i=0;i<8;i++)
	Data[i] = RxMessage[0].Data[i];

	/* 只取低 2 字节（16 位地址 + 32KB flash）*/
	start_addr = ((uint16_t)Data[2] << 8) | (uint16_t)Data[3];
	data_size  = ((uint16_t)Data[6] << 8) | (uint16_t)Data[7];

	/* 地址/长度合法性：非法直接回 CmdFaild，绝不置 writeInfoDone */
	if((data_size < 2) || (data_size > 1026) ||
	   (start_addr < APP_START_ADDR) ||
	   ((uint32_t)start_addr + (uint32_t)(data_size - 2) > APP_END_ADDR))   /* 用实际长度 */
	{
		writeInfoDone = 0;
		data_index    = 0;
		TxMessage.IdentifyID.all = (CAN_BOOT_GetAddrData()<<CMD_WIDTH)|CMD_List.CmdFaild;
		CANSendMsgCheck(0,&TxMessage,1);
		return;
	}

	writeInfoDone = 1;
	data_index = 0;
	for(i=0;i<1026;i++)
		Store_Temp[i] = 255;

	TxMessage.IdentifyID.all = (CAN_BOOT_GetAddrData()<<CMD_WIDTH)|CMD_List.WriteInfo;
	CANSendMsgCheck(0,&TxMessage,1);
}

void CMD_Write(void)
{
	uint16_t i=0;
	FLASH_Status ret = FLASH_COMPLETE;
	uint16_t crc_data,crcexp=0;
	struct CANTxStc TxMessage;
	
	TxMessage.MsgFlag.bitf.SendReadyFlag = 0;
	TxMessage.MsgFlag.bitf.SendFinishFlag = 0;
	TxMessage.MsgFlag.bitf.SendEnableFlag = CAN_Enable;
	TxMessage.MsgBoxNum = CPDD_CANTxStatus_NoMailBox;
	TxMessage.MsgConfig.bitf.DataType = CPDD_CANDataType_FrameData;
	TxMessage.MsgConfig.bitf.IdentifyType = CPDD_CANType_Extended;
	
	if(RxMessage[0].DLC > 8)
		RxMessage[0].DLC = 8;
	for(i=0;i<RxMessage[0].DLC ;i++)
	{
		if(data_index >= 1026)
			break ;
		
		Store_Temp[data_index++] = RxMessage[0].Data[i];
	}
		
	if(!writeInfoDone)                  /* 没收到合法 WriteInfo，直接拒绝，避免 data_size=0 越界 */
	{
		TxMessage.IdentifyID.all = (CAN_BOOT_GetAddrData()<<CMD_WIDTH)|CMD_List.CmdFaild;
		CANSendMsgCheck(0,&TxMessage,1);
		return;
	}

	if(data_index >= data_size)         /* 一块收齐（data_size 含 2 字节 CRC）*/
	{
		data_index    = 0;
		writeInfoDone = 0;              /* 下一块必须重新 WriteInfo（本方案每块都带地址）*/

		crcexp   = ((uint16_t)Store_Temp[data_size-2] << 8) | Store_Temp[data_size-1];
		crc_data = crc16_ccitt(Store_Temp, (uint16_t)(data_size-2));

		if(crc_data == crcexp)
			ret = CAN_BOOT_ProgramDatatoFlash(start_addr, Store_Temp, (uint16_t)(data_size - 2));
		else
			ret = FLASH_ERROR_PG;       /* CRC 错 → 走失败应答，别让上位机干等超时 */

		TxMessage.IdentifyID.all = (CAN_BOOT_GetAddrData()<<CMD_WIDTH) |
		                           (ret == FLASH_COMPLETE ? CMD_List.Write : CMD_List.CmdFaild);
		CANSendMsgCheck(0,&TxMessage,1);
	}
}

/**
  * @brief  执行主机下发的命令
  * @param  pRxMessage CAN总线消息
  * @retval 无
  */
void CAN_BOOT_ExecutiveCommand(uint8_t can_cmd)
{
	uint32_t jumpAddr = 0;

  //CMD_List.Erase，擦除Flash中的数据，需要擦除的Flash大小存储在Data[0]到Data[3]中
  //该命令必须在Bootloader程序中实现，在APP程序中可以不用实现
  if(can_cmd == CMD_List.Erase)
	{
		CMD_Erase();
    return;
  }

  //CMD_List.WriteInfo，设置写Flash数据的相关信息，比如数据起始地址，数据大小
  //数据偏移地址存储在Data[0]到Data[3]中，数据大小存储在Data[4]到Data[7]中，该函数必须在Bootloader程序中实现，APP程序可以不用实现
  if(can_cmd == CMD_List.WriteInfo)
	{
		CMD_WriteInfo();
  }
	
  //CMD_List.Write，先将数据存储在本地缓冲区中，然后计算数据的CRC，若校验正确则写数据到Flash中
  //每次执行该数据，数据缓冲区的数据字节数会增加pRxMessage->DLC字节，当数据量达到data_size（包含2字节CRC校验码）字节后
  //对数据进行CRC校验，若数据校验无误，则将数据写入Flash中
  //该函数在Bootloader程序中必须实现，APP程序可以不用实现
  if(can_cmd == CMD_List.Write)
	{
		CMD_Write();
    return;
  }
	
  //CMD_List.Check，节点在线检测
  //节点收到该命令后返回固件版本信息和固件类型，该命令在Bootloader程序和APP程序都必须实现
  if(can_cmd == CMD_List.Check)
	{
    CMD_Check();
    return;
  }
	
  //CMD_List.Excute，控制程序跳转到指定地址执行
  //该命令在Bootloader和APP程序中都必须实现
  	if(can_cmd == CMD_List.Excute)
	{
		jumpAddr = ((uint32_t)RxMessage[0].Data[0]<<24)|((uint32_t)RxMessage[0].Data[1]<<16)
				| ((uint32_t)RxMessage[0].Data[2]<<8)| (uint32_t)RxMessage[0].Data[3];
		if(jumpAddr == APP_START_ADDR)
			CMD_Excute();
		return;
	}
}

void CAN_BOOT_Command(void)
{
	uint16_t can_addr=0;
	uint8_t can_cmd=0;
	
	if(CAN1_CanRxMsgFlag && RxMessage[0].valid)
	{
		// EA = 0;
		
		can_addr = ((RxMessage[0].ExtId) >> CMD_WIDTH);  //ID的bit4~bit15位为节点地址
		can_cmd = ((RxMessage[0].ExtId)&0x0f);//ID的bit0~bit3位为命令码
		if((can_addr == CAN_BOOT_GetAddrData()) && (can_addr != 0))
		{
			CAN_BOOT_ExecutiveCommand(can_cmd);
		}
		CAN1_CanRxMsgFlag = 0;
		
		// EA = 1;
	}
}
/*********************************END OF FILE**********************************/

