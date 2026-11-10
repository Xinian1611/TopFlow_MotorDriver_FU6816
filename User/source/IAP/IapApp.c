#include "IapApp.h"

union IAP_IDENTIFY IAPIdentifyTx,IAPIdentifyRx;

uint16_t IAPCheckOverTime = 0;          /* 原来是 float 秒；现在当 1ms 倒计时用 */
uint16_t Station_ID       = 1;          /* 空片默认地址（与上位机约定）；IAPData_Init 会用参数页的值覆盖 */
uint8_t  IAPRxFinishFlag  = 0;

static uint8_t  IAPUpgradePending = 0;
static uint16_t IAPStopTimeout    = 0;

// uint8_t FlashData_Page[STM_FLASH_PAGE_SIZE];

//------------------------------------------------------------------------------------
//函数功能：数据段数据初始化
//输入参数：无
//返回参数：无
//特别说明：无
//------------------------------------------------------------------------------------
void IAPData_Init(void)
{
	Station_ID = ReadCANID();                       /* 读 0x1708，读不到返回 0 */
	if((0 == Station_ID) || (Station_ID > 0x7F))
		Station_ID = 1;          /* 参数页无效（空片/校验错）→ 用默认地址 */

	IAPIdentifyTx.all  = IAP_ID_BUILD(Station_ID, 0);   /* 本机基址，命令位=0 */
	IAPIdentifyRx.all  = IAPIdentifyTx.all;

	IAPCheckOverTime  = 0;
	IAPRxFinishFlag   = 0;
	IAPUpgradePending = 0;
	IAPStopTimeout    = 0;
}

#ifdef IPADataType_FLASH
//------------------------------------------------------------------------------------
//函数功能：从FLASH中读取数据
//输入参数：faddr：FLASH地址
//返回参数：无
//特别说明：无
void STMFLASH_ReadBuff(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)
{
	uint32_t i=0,len;
	uint8_t *pByte = (uint8_t *)pBuffer;

	len = NumToRead;
	for(i=0;i<len;i++)
		pByte[i] = *(uint8_t code *)(ReadAddr + i);     /* 8051 必须走 code 空间 */
}

//------------------------------------------------------------------------------------
//函数功能：从FLASH中读取1页数据
//输入参数：faddr：FLASH地址
//返回参数：无
//特别说明：无
void STMFLASH_ReadPage(uint32_t ReadAddr,uint32_t *pBuffer)
{
	uint32_t i=0,PageStartAddr;

	i = (ReadAddr - STM_FLASH_BASE)/STM_FLASH_PAGE_SIZE;
	PageStartAddr = (i*STM_FLASH_PAGE_SIZE + STM_FLASH_BASE);

	for(i=0;i<STM_FLASH_PAGE_SIZE;i++)
		((uint8_t *)pBuffer)[i] = *(uint8_t code *)(PageStartAddr + i);
}

//------------------------------------------------------------------------------------
//函数功能：擦除FLASH指定页数据
//输入参数：faddr：FLASH地址
//返回参数：0：擦除失败，1：擦除成功
//特别说明：无
//------------------------------------------------------------------------------------
uint8_t STMFLASH_ErashPage(uint32_t faddr)
{
	if(FLASH_SUCCESS == Flash_SectorErase((uint16_t)faddr))
		return 1;
	if(FLASH_SUCCESS == Flash_SectorErase((uint16_t)faddr))   /* 原逻辑：失败再擦一次 */
		return 1;
	return 0;
}

//------------------------------------------------------------------------------------
//函数功能：写FLASH指定数据
//输入参数：faddr：FLASH地址
//返回参数：0：写失败，1：写成功
//特别说明：无
//------------------------------------------------------------------------------------
uint8_t STMFLASH_WriteBuff(uint32_t StartAddress,uint8_t *pData,uint32_t DataNum)
{
	uint32_t i;

	if(0 == STMFLASH_ErashPage(StartAddress))
		return 0;

	for(i=0;i<DataNum;i++)
	{
		SetBit(WDT_CR, WDTRF);                              /* 原 delayus(100) */
		Flash_byte_Write((uint16_t)(StartAddress + i), pData[i]);
	}
	return 1;
}

//------------------------------------------------------------------------------------
//函数功能：写FLASH指定页数据
//输入参数：faddr：FLASH地址
//返回参数：0：写失败，1：写成功
//特别说明：无
//------------------------------------------------------------------------------------
uint8_t STMFLASH_WritePage(uint32_t StartAddress,uint8_t *pData)
{
	uint32_t i,PageStartAddr;

	if(0 == STMFLASH_ErashPage(StartAddress))
		return 0;

	i = (StartAddress - STM_FLASH_BASE)/STM_FLASH_PAGE_SIZE;
	PageStartAddr = (i*STM_FLASH_PAGE_SIZE + STM_FLASH_BASE);

	for(i=0;i<STM_FLASH_PAGE_SIZE;i++)
	{
		SetBit(WDT_CR, WDTRF);
		Flash_byte_Write((uint16_t)(PageStartAddr + i), pData[i]);
	}
	return 1;
}

//------------------------------------------------------------------------------------
//函数功能：IAPFlag更新
//输入参数：无
//返回参数：0：更新失败，1：更新成功
//特别说明：无
//------------------------------------------------------------------------------------
int16_t IAPFlagErase(void)
{
	uint8_t  crcdata=0, TempData[8], WriteBuff[16];
	uint16_t temp16=0;
	uint32_t i;

	/* 1) 先看是不是已经"无效"了，是就不用再擦写 */
	STMFLASH_ReadBuff(IAPFlag_FLASH_OFFSET,(uint32_t*)TempData,8);
	*((uint8_t *)(&temp16)+1) = TempData[2];
	*(uint8_t *)(&temp16)     = TempData[3];
	for(i=0;i<4;i++)
		crcdata ^= TempData[i];
	if((0x55 == TempData[0]) && (0xaa == TempData[1]) &&
	   (0xffff == temp16) &&
	   (crcdata == TempData[4]))
	{
		return CPDD_TRUE;
	}

	/* 2) 只读前 16 字节（标志 5B + ID 7B + 填充），改前 5 字节，ID 原样保留 */
	STMFLASH_ReadBuff(IAPFlag_FLASH_OFFSET,(uint32_t*)WriteBuff,16);

	crcdata = 0;
	WriteBuff[0] = 0x55;
	WriteBuff[1] = 0xaa;
	WriteBuff[2] = 0xff;
	WriteBuff[3] = 0xff;
	for(i=0;i<4;i++)
		crcdata ^= WriteBuff[i];
	WriteBuff[4] = crcdata;

	/* 3) 擦掉 0x0F00 那个 256B 扇区，再写回这 16 字节 */
	if(0 == STMFLASH_WriteBuff(IAPFlag_FLASH_OFFSET, WriteBuff, 16))
		return CPDD_FALSE;

	return CPDD_TRUE;
}

//------------------------------------------------------------------------------------
//函数功能：IAPID更新
//输入参数：无
//返回参数：0：更新失败，1：更新成功
//特别说明：无
//------------------------------------------------------------------------------------
int16_t IAPIDUpdate(void)
{
	uint8_t  crcdata=0, TempData[8], WriteBuff[16];
	uint32_t i, temp32=0;

	/* 1) 先看 ID 是不是已经一致 */
	STMFLASH_ReadBuff(IAPID_FLASH_OFFSET,(uint32_t*)TempData,8);
	*((uint8_t *)(&temp32)+3) = TempData[2];
	*((uint8_t *)(&temp32)+2) = TempData[3];
	*((uint8_t *)(&temp32)+1) = TempData[4];
	*(uint8_t *)(&temp32)     = TempData[5];
	for(i=0;i<6;i++)
		crcdata ^= TempData[i];
	if((0x55 == TempData[0]) && (0xaa == TempData[1]) &&
	   ((temp32 & 0xfffffff0) == (IAPIdentifyTx.all & 0xfffffff0)) &&
	   (crcdata == TempData[6]))
	{
		return CPDD_TRUE;
	}

	/* 2) 读前 16 字节（标志 + ID），把标志原样带上，只改 ID */
	STMFLASH_ReadBuff(IAPFlag_FLASH_OFFSET,(uint32_t*)WriteBuff,16);

	crcdata = 0;
	WriteBuff[8]  = 0x55;
	WriteBuff[9]  = 0xaa;
	WriteBuff[10] = *((uint8_t *)(&IAPIdentifyTx.all)+3);
	WriteBuff[11] = *((uint8_t *)(&IAPIdentifyTx.all)+2);
	WriteBuff[12] = *((uint8_t *)(&IAPIdentifyTx.all)+1);
	WriteBuff[13] = *(uint8_t *)(&IAPIdentifyTx.all);
	for(i=8;i<14;i++)
		crcdata ^= WriteBuff[i];
	WriteBuff[14] = crcdata;

	/* 3) 擦扇区 + 写回 16 字节 */
	if(0 == STMFLASH_WriteBuff(IAPFlag_FLASH_OFFSET, WriteBuff, 16))
		return CPDD_FALSE;

	return CPDD_TRUE;
}
#endif

/*------------------------------------------------------------------------------------
函数功能：应答上位机的"在线检查"（原来由 CAN.c 置发送使能 + CANTxMsgIAP 缓冲完成）
输入参数：无
返回参数：无
特别说明：8 字节 = 版本 4B + 1,0,0,0；ID = (Station<<4)|OnlineCheck
          这里的 1 是"我是 APP"的标志位，boot 应答时是 0,1,0...
------------------------------------------------------------------------------------*/
static void IAPSendCheck(void)
{
	uint8_t buf[8];

	buf[0] = IAP_APP_VER0;
	buf[1] = IAP_APP_VER1;
	buf[2] = IAP_APP_VER2;
	buf[3] = IAP_APP_VER3;
	buf[4] = 1;
	buf[5] = 0;
	buf[6] = 0;
	buf[7] = 0;

	(void)CanMin_IapSend(IAP_ID_BUILD(Station_ID, OnlineCheck), buf, 8);  /* IAP 帧必须扩展帧 */
}

//------------------------------------------------------------------------------------
//函数功能：IAP升级请求检查
//输入参数：无
//返回参数：无
//特别说明：无
//------------------------------------------------------------------------------------
void IAPRequestCheck(void)
{
	static uint8_t UpdateFlag = 0;
	uint8_t cmd;                    /* 命令码，必须从 .all 里取 */

	/* 已收到 cmd9：等电机停稳后再擦标志 */
	if(IAPUpgradePending)
	{
		if((mcState == mcStop) || (mcState == mcReady))
		{
			IAPUpgradePending = 0;
			EA = 0;
			if(CPDD_TRUE == IAPFlagErase())
			{
				SetBit(RST_SR, SOFTR);
				while(1);
			}
			EA = 1;
		}
		else if(0 == IAPStopTimeout)     /* 3s 没停稳：放弃本次升级 */
		{
			IAPUpgradePending = 0;
			UpdateFlag = 0;
		}
		return;
	}

	if(0 == IAPRxFinishFlag)
	{
		if((1 == UpdateFlag) && (0 == IAPCheckOverTime))
			UpdateFlag = 0;
		return ;
	}

	IAPRxFinishFlag = 0;
	IAPCheckOverTime = 30 * 100;

	/* 关键：C51 是大端，IAPIdentifyRx.all 的低 4 位才是命令码；
	   用 bitf.IAPCmd 会读到高 16 位（恒为 0），判断永远不成立 */
	cmd = (uint8_t)(IAPIdentifyRx.all & 0x0F);

	if(OnlineCheck == cmd)
	{
		IAPSendCheck();             /* 只要握手就应答，每次都回 */
	}

	if(0 == UpdateFlag)
	{
		if(OnlineCheck == cmd)
		{
			#ifdef IPADataType_FLASH
			EA = 0;
			#endif

			if(CPDD_TRUE == IAPIDUpdate())
				UpdateFlag = 1;

			#ifdef IPADataType_FLASH
			EA = 1;
			#endif
		}
	}
	else
	{
		if((ExcuteApp == cmd) || (0x0F == cmd))
		{
			CanMin_SetEstop(1);
			CanMin_SetRun(0);
			g_run_state.is_started = 0;

			IAPUpgradePending = 1;
			IAPStopTimeout    = 3000;    /* 3s 停机窗口 */
		}
	}
}

/*------------------------------------------------------------------------------------
函数功能：异或校验（f105 Robert_EEPROM.c 原函数）
输入参数：strData：数据首地址；len：参与校验的字节数
返回参数：各字节异或结果
特别说明：只用于参数页里的 5 字节标志 / 7 字节 ID，不做 CRC16
------------------------------------------------------------------------------------*/
uint8_t CheckXor(uint8_t *strData,int len)
{
	uint8_t checksum = 0;
	int i;
	for(i = 0;i < len;i++)
		checksum = checksum ^ strData[i];
	return checksum;
}

/*------------------------------------------------------------------------------------
函数功能：从参数页读本机地址（对应 f105 Robert_EEPROM.c 的 ReadCANID）
输入参数：无
返回参数：Station_ID（0~127）；参数页全 0xFF 或校验错时返回 0
特别说明：读的是 Flash 0x0F08 起的 7 字节：55 AA + 4B(Station<<4，大端) + XOR
          必须用 code 指针读，否则读到的是 xdata
------------------------------------------------------------------------------------*/
uint16_t ReadCANID(void)
{
	uint8_t readout[StationIDSize] = {0};
	uint8_t CRC_Result = 0;
	uint32_t result = 0;
	uint8_t i;

	for(i = 0;i < StationIDSize;i++)
		readout[i] = *(uint8_t code *)(STM_FLASH_BASE + IAPID_FLASH_OFFSET + i);

	CRC_Result = CheckXor(&readout[0],StationIDSize-1);
	if((CRC_Result == readout[StationIDSize-1]) && (readout[0] == 0x55) && (readout[1] == 0xAA))
	{
		result = ((uint32_t)readout[2] << 24) | ((uint32_t)readout[3] << 16) |
		         ((uint32_t)readout[4] << 8)  | (uint32_t)readout[5];
		return (uint16_t)(result >> 4);
	}
	return 0;
}

/*------------------------------------------------------------------------------------
函数功能：CAN 层把一帧 11 位标准帧交给 IAP（替代原来的 CANRxMsgIAP 报文对象）
输入参数：id：11 位标准 ID；data：8 字节数据；dlc：数据长度
返回参数：无
特别说明：APP 只认 cmd1（握手）和 cmd9（进 boot），其它命令在 APP 里不响应
          这里只记 ID 和置完成标志，真正的 flash 操作放在 1ms 的 IAPRequestCheck 里，
          避免在 CAN 中断/解析上下文里做 20ms 级别的擦写
------------------------------------------------------------------------------------*/
void IAPRxStdFrame(uint16_t id, uint8_t *dat, uint8_t dlc)
{
	uint8_t cmd;

	if(IAP_ID_ADDR(id) != Station_ID)   return;     /* 不是本机 */
	cmd = IAP_ID_CMD(id);
		/* 0x0F 与 cmd9 同义（F105 用 0x0F、L431 用 9，两条都收） */
	if((cmd != OnlineCheck) && (cmd != ExcuteApp) && (cmd != 0x0F))
		return;

	IAPIdentifyRx.all = id;
	IAPRxFinishFlag   = 1;
}

/* 1ms 调用：3 秒握手窗口倒计时 */
void IAPTick1ms(void)
{
	if(IAPCheckOverTime) IAPCheckOverTime--;
	if(IAPStopTimeout)   IAPStopTimeout--;
}
