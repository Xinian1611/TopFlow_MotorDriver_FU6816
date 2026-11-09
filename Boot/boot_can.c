#include "boot_can.h"

/*------------------------------------------------------------------------------------
函数功能：boot 的 CAN 初始化（250k，标准帧，全通滤波，不开中断）
输入参数：无
返回参数：无
特别说明：boot 阶段 EA 必须为 0，只能轮询；参数照搬 APP 的 CAN.c
------------------------------------------------------------------------------------*/
void CAN_Init(void)
{
	uint16 BRP  = 11;               /* 24MHz / (12*8) = 250k */
	uint8  SEG1 = 5;
	uint8  SEG2 = 0;

	CAN_BTR0 = 0;
	CAN_BTR1 = 0;
	CAN_BTR2 = 0;
	CAN_BTR0 |= (uint8)(BRP >> 8);
	CAN_BTR1  = (uint8)BRP;
	CAN_BTR2 |= (uint8)((SEG2 << 4) | SEG1);

	SetBit(CAN_CR0, FILMOD);        /* 单滤波 */
	CAN_FIR0 = 0; CAN_FIR1 = 0; CAN_FIR2 = 0; CAN_FIR3 = 0;
	CAN_FMR0 = 0xFF;                /* 屏蔽位全 1 = 全不关心，标准/扩展都收 */
	CAN_FMR1 = 0xFF;
	CAN_FMR2 = 0xFF;
	CAN_FMR3 = 0xFF;

	CAN_IER = 0;                    /* 不开任何中断 */
	ClrBit(CAN_CR0, RSTMOD);
	ClrBit(CAN_CR0, CAN_CH);        /* 锁死 P0.0/P0.1 */
	SetBit(CAN_CR0, CANEN);
}

/*  ----------------------------------------------------------------------------------------------*/
/*  Function Name  : CAN_Read can数据接收
	/*  Date           : 2022-08-26
/*  Parameter      : 从邮箱中获取接收的数据值 ，boot不会考虑标准扩展混合的情况
	/*  ----------------------------------------------------------------------------------------------*/
void CAN_Read(void)                 /* 填 RxMessage[0] */
{
	uint8 rdptr;

    RxMessage[0].ExtId = 0;
    RxMessage[0].DLC   = 0;
    RxMessage[0].valid = 0;

    rdptr = (uint8)(CAN_CR2 & (RDPTR1 | RDPTR0));
	switch (rdptr)
	{
	case (0 | 0):                                   /* 邮箱 0 */
		if((CAN_RX0CR & 0x80) == 0) break;                          /* 只收扩展帧 */
		if(CAN_RX0ID0 || CAN_RX0ID1 || (CAN_RX0ID2 & 0xC0)) break;  /* 只认 ID<=0x7FF(addr<=0x7F) */
		RxMessage[0].DLC   = (uint8)(CAN_RX0CR & 0x0F);
		RxMessage[0].ExtId = ((uint16_t)CAN_RX0ID2 << 5) | ((uint16_t)CAN_RX0ID3 >> 3);
		RxMessage[0].Data[0]=CAN_RX0DR0; RxMessage[0].Data[1]=CAN_RX0DR1;
		RxMessage[0].Data[2]=CAN_RX0DR2; RxMessage[0].Data[3]=CAN_RX0DR3;
		RxMessage[0].Data[4]=CAN_RX0DR4; RxMessage[0].Data[5]=CAN_RX0DR5;
		RxMessage[0].Data[6]=CAN_RX0DR6; RxMessage[0].Data[7]=CAN_RX0DR7;
		RxMessage[0].valid = 1;
		break;

	case (0 | RDPTR0):                              /* 邮箱 1 */
		if((CAN_RX1CR & 0x80) == 0) break;
		if(CAN_RX1ID0 || CAN_RX1ID1 || (CAN_RX1ID2 & 0xC0)) break;
		RxMessage[0].DLC   = (uint8)(CAN_RX1CR & 0x0F);
		RxMessage[0].ExtId = ((uint16_t)CAN_RX1ID2 << 5) | ((uint16_t)CAN_RX1ID3 >> 3);
		RxMessage[0].Data[0]=CAN_RX1DR0; RxMessage[0].Data[1]=CAN_RX1DR1;
		RxMessage[0].Data[2]=CAN_RX1DR2; RxMessage[0].Data[3]=CAN_RX1DR3;
		RxMessage[0].Data[4]=CAN_RX1DR4; RxMessage[0].Data[5]=CAN_RX1DR5;
		RxMessage[0].Data[6]=CAN_RX1DR6; RxMessage[0].Data[7]=CAN_RX1DR7;
		RxMessage[0].valid = 1;
		break;

	case (RDPTR1 | 0):                              /* 邮箱 2 */
		if((CAN_RX2CR & 0x80) == 0) break;
		if(CAN_RX2ID0 || CAN_RX2ID1 || (CAN_RX2ID2 & 0xC0)) break;
		RxMessage[0].DLC   = (uint8)(CAN_RX2CR & 0x0F);
		RxMessage[0].ExtId = ((uint16_t)CAN_RX2ID2 << 5) | ((uint16_t)CAN_RX2ID3 >> 3);
		RxMessage[0].Data[0]=CAN_RX2DR0; RxMessage[0].Data[1]=CAN_RX2DR1;
		RxMessage[0].Data[2]=CAN_RX2DR2; RxMessage[0].Data[3]=CAN_RX2DR3;
		RxMessage[0].Data[4]=CAN_RX2DR4; RxMessage[0].Data[5]=CAN_RX2DR5;
		RxMessage[0].Data[6]=CAN_RX2DR6; RxMessage[0].Data[7]=CAN_RX2DR7;
		RxMessage[0].valid = 1;
		break;
	case (RDPTR1 | RDPTR0):                          /* 邮箱 3：FU68x6 共 4 个邮箱 */
		break;

	default: break;
	}
	if(RxMessage[0].valid)                           /* 只有真正收下的帧才通知上层 */
		CAN1_CanRxMsgFlag = 1;
}

/*------------------------------------------------------------------------------------
函数功能：轮询收一帧（boot 不能用中断：0x0003~0x007B 已 LJMP 到 APP 区）
输入参数：无
返回参数：无
特别说明：收到即填 RxMessage[0] 并置 CAN1_CanRxMsgFlag，由 CAN_BOOT_Command 解析
------------------------------------------------------------------------------------*/
void CAN_detect(void)
{
	if(ReadBit(CAN_IFR, RXIF))
	{
		CAN_Read();
		SetBit(CAN_CR1, BUFRLS);     /* 轮询模式必须手动释放接收缓冲 */
	}
	if(ReadBit(CAN_IFR, TXIF))  ClrBit(CAN_IFR, TXIF);
	if(ReadBit(CAN_IFR, ERRIF)) ClrBit(CAN_IFR, ERRIF);
}

/*------------------------------------------------------------------------------------
函数功能：发一帧 11 位标准帧（替代领导 CPDD CAN.c 的同名函数）
输入参数：index/Length 忽略；Data 只用 IdentifyID.all 和 Buff[0..7]
返回参数：无
特别说明：MsgFlag/MsgConfig/MsgBoxNum 是 STM32 驱动参数，这里直接忽略
------------------------------------------------------------------------------------*/
void CANSendMsgCheck(uint8_t index,struct CANTxStc *Data,uint16_t Length)
{
	uint16 id;
	uint16 to = 3000;

	id = (uint16)(Data->IdentifyID.all & 0x07FF);

	while(ReadBit(CAN_SR, TXING))    /* 上一帧没发完，最多等 3000 次，不死等 */
	{
		if(--to == 0) break;
	}

	/* 29 位扩展帧：与公司 F105/L431 方案一致 */
	CAN_TXCR  = 0x88;                /* IDE=1(扩展) | DLC=8 */
	CAN_TXID0 = (uint8)(id >> 21);
	CAN_TXID1 = (uint8)(id >> 13);
	CAN_TXID2 = (uint8)(id >> 5);
	CAN_TXID3 = (uint8)(id << 3);

	CAN_TXDR0 = Data->Buff[0];
	CAN_TXDR1 = Data->Buff[1];
	CAN_TXDR2 = Data->Buff[2];
	CAN_TXDR3 = Data->Buff[3];
	CAN_TXDR4 = Data->Buff[4];
	CAN_TXDR5 = Data->Buff[5];
	CAN_TXDR6 = Data->Buff[6];
	CAN_TXDR7 = Data->Buff[7];

	SetBit(CAN_CR1, TXREQ);
}
