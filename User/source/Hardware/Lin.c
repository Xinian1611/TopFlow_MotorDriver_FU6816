/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      main.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2021-08-13
 * @date      Last modify:2022-08-13
 * @note      Last modify author is Leo.li
 * @brief     包含Lin初始化，数据处理
 */

#include <Myproject.h>

/* 用户设置接收帧的ID，和接收帧的数据长度  */
#define SEND01_ID 0X01 //  ID:0x09长度==有效字节数
#define SEND02_ID 0X02 // ID:0x3C长度==有效字节数

#define RID01_LEN 8 /* 设置接收ID1的数据长度 */
#define RID02_LEN 8 /* 设置接收ID2的数据长度 */
#define RID3C_LEN 8 /* 设置接收诊断3C，3D的数据长度 */
/* 用户设置接收帧的ID，和接收帧的数据长度  end */

/* 诊断使用的用户可以删除  */
#define WILDCARD_SUPPLIER_ID 0x7FFF
#define WILDCARD_FUNCTION_ID 0xFFFF
#define L_SUPPLIER_ID 0x0041 // supplier ID (for Micronas: 0x0041)
#define L_FUNCTION_ID 0x1234 // function ID (for Micronas: 0x0041)
#define L_VARIANT_ID 0xAB	 //
#define L_SERIAL_NUMBER 0x12345678
#define L_ID_TABLE_SIZE 10 // number of supported IDs (including 0x3C and 0x3D)
#define INITIAL_NAD 0x01
#define WILDCARD_NAD 0x7F
#define OUT_CNT 1020
uint16 out_time = 0; // 诊断超时时间
/* 诊断使用的用户可以删除 end  */

uint8 Drlen = 0;            //Lin数据长度
uint8 Lin_Response_ER = 0; /* lin总线发生错误标志位 */
uint8 ID_Rflag = 0;		   // 有效ID标志位
uint8 Erase_Flage = 0;
LINState LS;

uint8 xdata ucaLinBuff[10] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; /* dma通道数组 */
uint8 l_PID_Table[10] = {0x2A, 0x2E, 0xe2, 0xa3, 0x64};						   // list of supported PIDs 
void Receive3C(void);

/* 用户LIN的接收发送协议数据处理  */
LINdata xdata EOP;	  ///< EOP数据
uint8 loc_linBusTimeoutFlag = 0;
uint16 loc_linBusTimeoutCnt = 0;
/**
 * @brief     Lin初始化
 * @date      2022-08-13
 */
void LIN_Init(void)
{
	/*-------------------------------------------------------------------------------------------------
		DMA通道配置
		DMASEL=1:配置为DMA1 DMASEL=0:配置为DMA0
	-------------------------------------------------------------------------------------------------*/
	/* 初始化寄存器,关闭LIN模块 */
	DMA0_CR0 = 0;
	LIN_CR = 0;
	LIN_SR = 0;
	LIN_CSR = 0;
	/* 配置dma通道 */
	DMA0_BA = (uint16)ucaLinBuff;
	SetBit(DMA0_CR0, DMAEN | DMACFG2 | DMACFG1);
	ClrBit(LIN_CR, DMASEL);
	/* 配置LIN接口 */
	
	#if (LIN_TRANSCEIVER == EXTERNAL_LIN_DOUBLELINE)//双线模式 
        #if (LIN_PORTSELECTION == TXDGP10_RXDGP11)
        SetBit(LIN_CR, CHSEL1);	  // 00:配置为TXD：P00 RXD：p01;
        ClrBit(LIN_CR, CHSEL0);	  // 10:配置为TXD：P10 RXD：p11    //11:单线lin P11
        ClrBit(P1_PU, P11HV_EN);  // P11 高压输入模式，关闭   P11与P10口为开漏输出，需要加硬件上拉
        ClrBit(P1_PU, P10HV_EN);  // P10 高压输入模式，关闭
        #elif (LIN_PORTSELECTION == TXDGP00_RXDGP01)
        ClrBit(LIN_CR, CHSEL1);	  // 00:配置为TXD：P00 RXD：p01;
        ClrBit(LIN_CR, CHSEL0);	  // 10:配置为TXD：P10 RXD：p11    //11:单线lin P11    
        SetBit(P0_PU , P01);      // RXD注意是否有硬件上拉，可以配置软件上拉
        #endif
        SetBit(P4_OE, P43);
        GP43 = 1;               /* 外置收发器休眠管脚使能配置 */
	#elif (LIN_TRANSCEIVER == INNER_LIN_SINGLELINE)//单线模式P11
        SetBit(LIN_CR, CHSEL1);	  // 00:配置为TXD：P00 RXD：p01;
        SetBit(LIN_CR, CHSEL0);	  // 10:配置为TXD：P10 RXD：p11    //11:单线lin P11
        SetBit(P1_PU, P11HV_EN);  // P11 高压输入模式，关闭   P11与P10口为开漏输出，需要加硬件上拉
	#endif
	
	ClrBit(LIN_CR, CHKMOD);	  // 0:增强校验    1:标准校验
	ClrBit(LIN_CR, AUTOSIZE); // 自动长度设定（ID[4][5]决定）
	SetBit(LIN_CR, LINIE);	  // 中断设置
	SetBit(LIN_CSR, LINEN);	  // LIN模块使能

	SetBit(LIN_CR, MBAUD);    // 手动波特率
	LIN_BAUD = 1249;          // 波特率为19200
}

/* -------------------------------------------------------------------------------------------------
	Function Name  : Data_Error_Check
	Description    : 检验是否发生数据检验错误
	Date           : 2020-07-30
	Parameter      : None
------------------------------------------------------------------------------------------------- */
ebool Data_Error_Check(void)
{
	return ReadBit(LIN_SR, ERRCHK);
}

/**
 * @brief        检验是否发生ID校验错误
 * @return       错误源信号
 * @date         2022-08-14
 */
ebool ID_Error_BIT(void)
{
	return ReadBit(LIN_CSR, ERRBIT);
}
/* -------------------------------------------------------------------------------------------------
	Function Name  : LIN_Error_LenthHanding
	Description    : 处理长度错误的函数
	Date           : 2020-07-30
	Parameter      : LIN一致性的Response err 要求在接收到完整正确的帧头之后发生错误，错误标志位才能置1
------------------------------------------------------------------------------------------------- */
void LIN_Error_LenthHanding(void)
{
    if (ID_Rflag == ID_RX) /* 判断上一次的接收数据有没有完成 */
    {
        if ((Drlen != 0) && (Drlen < 8)) /* 前一帧数据长度不满足要求 */
        {
            Lin_Response_ER = 1; /* 故障标志位置1 */
        }
    }
    if (Lin_Response_ER)	  /* 发送故障标志位 */
    {
    //			ucaLinBuff[0] = 0x01; /* 发送故障标志位  */
        EOP.ECF_LinLostError = 1; /* 发送故障标志位  */
        EOP.ECF_FaultSt      = 1;
    }
    else
    {
    //			ucaLinBuff[0] = 0x00;
        EOP.ECF_LinLostError = 0; /* 发送故障标志位  */
        EOP.ECF_FaultSt      = 0;
        
    }
    Lin_Response_ER = 0; /* 发送完成之后清除故障标志位 */
}
/* -------------------------------------------------------------------------------------------------
	Function Name  : LIN_Error_Handing
	Description    : 处理错误的函数
	Date           : 2020-07-30
	Parameter      : LIN一致性的Response err 要求在接收到完整正确的帧头之后发生错误，错误标志位才能置1
------------------------------------------------------------------------------------------------- */
void LIN_Error_Handing(void)
{
	ClrBit(LIN_SR, ERRPRTY);
	ClrBit(LIN_SR, ERRSYNC);
	//前面两个错误不需要报故障
	
	if (Data_Error_Check()) // 数据校验错误
	{
		ClrBit(LIN_SR, ERRCHK);
		if (ID_Rflag) // 总线接收发送时候出现校验错误
		{
			Lin_Response_ER = 1; // lin总线错误
		}
	}
	else if (ID_Error_BIT()) // bit数据错误
	{
		ClrBit(LIN_CSR, ERRBIT);
//	  ClrBit(LIN_CSR, LINSTOP);  //自动波特率时候要做清除操作
		if (ID_Rflag)			 // 总线的数据是发送或者接收时候
			Lin_Response_ER = 1; // lin总线错误
	}

}

/*-------------------------------------------------------------------------------------------------
	Function Name :	void LIN_list(uint8 PID)
	Description   :	总线收发列表 ，对接收的数据设置长度和检验类型 。对发送的数据设置
	Input         :	要发送的字符串的地址
	Output		  :	无
-------------------------------------------------------------------------------------------------*/
uint8 LIN_ID1 = 0;
void LIN_list(void)
{
	uint8 i = 0;
	LIN_ID1 = LIN_ID;
	Drlen = DMA0_LEN; // 读取接收数据的长度，

	/* 用户接收的ID */
	if ((LIN_ID1 == (l_PID_Table[0] & 0x3f)) && (l_PID_Table[0] != 0)) // 接收数据
	{
		ID_Rflag = ID_RX; /* 设置接收标志 */
		LIN_SETSIZE(8);	  /* 接收数据长度 */
		CheckEnhanced;	  /* 设置校验模式 */
		LINRW_R;		  /* 接收数据模式 */
		LIN_ACK;		  /* 响应帧头 */
	}

	/* 用户发送的ID */
	else if ((LIN_ID1 == (l_PID_Table[1] & 0x3f)) || (LIN_ID1 == (l_PID_Table[2] & 0x3f))) // 发送数据
	{
        LIN_Error_LenthHanding();   /* 长度错误处理 */  
		ID_Rflag = ID_TX;
		LIN_SETSIZE(8);			  /* 发送数据长度 */
		// 用户发送数据处理
		// 用户发送数据处理
		// 用户发送数据处理
        ucaLinBuff[0] = (EOP.ECF_EnableSt | (EOP.ECF_WorkingSt << 1) | (EOP.ECF_WorkingMode << 2) | (0x0f << 4));
        ucaLinBuff[1] = EOP.ECF_Speed;
        ucaLinBuff[2] = EOP.ECF_Voltage;
        ucaLinBuff[3] = (((EOP.ECF_Voltage & 0x1ff) >> 8) | ((EOP.ECF_Current & 0x03) << 6) | (0x1f << 1));
        ucaLinBuff[4] = ((EOP.ECF_Current & 0x3ff) >> 2);
        ucaLinBuff[5] = EOP.ECF_Power & 0xff;
        ucaLinBuff[6] = (((EOP.ECF_Power & 0x3ff) >> 8) | (0x3f << 2));
        ucaLinBuff[7] = (((EOP.ECF_OverCurrentError & 0x01) << 7) |
                         ((EOP.ECF_StallError & 0x01) << 6) |
                         ((EOP.ECF_HardwareError & 0x01) << 5) |
                         ((EOP.ECF_OverTempError & 0x01) << 4) |
                         ((EOP.ECF_OverVoltageError & 0x01) << 3) |
                         ((EOP.ECF_UnderVoltageError & 0x01) << 2) |
                         ((EOP.ECF_LinLostError & 0x01) << 1) |
                         ((EOP.ECF_FaultSt & 0x01) << 0));
		CheckEnhanced; /* 设置校验模式 */
		LINRW_W;	   /* 发送数据 */
		LIN_ACK;	   /* 响应帧头 */
	}

	/* 诊断信号0X3c 和0x3d的处理根据客户自己的协议处理就行 */
	/* 这里只是一个示例代码 */
	/* 诊断信号0X3c处理 */
	else if (LIN_ID1 == 0x3c)
	{
		ID_Rflag = ID_RX;		/* 设置为接收模式 */
		LIN_SETSIZE(RID3C_LEN); /* 接收数据长度 */
		CheckClass;				/* 设置为标准校验模式 */
		LINRW_R;				/* 接收数据模式 */
		LIN_ACK;				/* 响应帧头 */
	}
	/*  诊断信号0X3d处理 */
	else if (LIN_ID1 == 0x3d)
	{
		if (out_time == 0) // 诊断读取时间超时 ，不回复数据
		{
			LS.Pack = 0;
		}
		else
		{
			out_time = OUT_CNT;
		}

		if (LS.Pack == 1)
		{
			ID_Rflag = ID_TX;
			for (i = 0; i < 8; i++)
			{
				ucaLinBuff[i] = LS.SData3D[i];
			}
			LIN_SETSIZE(RID3C_LEN); /* 设置发送长度 */
			CheckClass;				/* 设置发送模式 */
			LINRW_W;				/* 设置发送模式 */
			LIN_ACK;				/* 响应帧头 */
			LS.Pack = 0;			/* 清除发送包标志位 */
		}
		else
		{
			ClrBit(LIN_SR, LINDONE); /* 清除传输完成产生中断的标志位 */
			LINRW_R;				 /* 接收数据模式 */
			ID_Rflag = 0;
		}
	}

	else /* 无效ID处理 */
	{
		ClrBit(LIN_SR, LINDONE); /* 清除传输完成产生中断的标志位 */
		LINRW_R;				 /* 接收数据模式 */
		ID_Rflag = 0;
        LIN_ACK;                 /* 无效ID也应答 */              
	}
}

/*  -------------------------------------------------------------------------------------------------
	Function Name  : ReceiveSYS
	Description    : 总线接收数据处理
	Date           :
				   : [输入]
	------------------------------------------------------------------------------------------------- */
uint16 err = 0;
void ReceiveSYS(uint8 PID)
{
	uint8 i = 0;
	Drlen = DMA0_LEN; // 读取接收数据的长度，

	if (PID == l_PID_Table[0])
	{
		if (Drlen == RID01_LEN) /* 接收完成的数据要进行长度判断 */
		{
			for (i = 0; i < 8; i++)
			{
				LS.RData[i] = ucaLinBuff[i];        /* 验证接收数据 */
			}
			// 用户代码数据处理
			// 用户代码数据处理
			// 用户代码数据处理
			// 用户代码数据处理
			// 用户代码数据处理
            EOP.EnableReq_ECF = ucaLinBuff[0]; // 使能信号，无使能，即使有转速请求也不工作
            EOP.SpeedReq_ECF = ucaLinBuff[1];  // 请求转速
		}
		else
		{
			Lin_Response_ER = 1; /* lin总线错误标志位置1 */
		}
	}
	else if (PID == 0X3C) /* 诊断数据处理 */
	{

		if (Drlen == RID3C_LEN) /* 接收完成的数据要进行长度判断 */
		{
			for (i = 0; i < 8; i++)
			{
				LS.RData[i] = ucaLinBuff[i]; /* 数据从dma数组中取出，注意不要直接更改DMA地址。数据最好转存到另一个数组再去处理，不要直接处理dma数组 */
			}
			// 用户代码数据处理
			// 用户代码数据处理
			/* 客户有自己的诊断可以用自己的，这个只是参考 */
			Receive3C(); // 诊断数据处理，可以更改为用户自己的程序

			if (ucaLinBuff[0] == 0x00) // 进入睡眠模式
			{
				// 执行休眠函数
				_nop_();
			}
		}
		else
		{
			Lin_Response_ER = 1; /* lin总线错误标志位置1 */
		}
	}

	ID_Rflag = 0;
}

/*-------------------------------------------------------------------------------------------------
	Function Name :	void Receive3C(void)
	Description   :	诊断使用的用户可以删除
	Input         :	诊断使用的用户可以删除
	Output		  :	诊断使用的用户可以删除
-------------------------------------------------------------------------------------------------*/

/******************************************************************************/
/** \fn		_Function_ID_match ()
 *
 *	\brief	Checks if the given Function ID matches the LIN	Product Identification
 *
 *	input:	pointer to Function ID low	byte
 *
 *	return:	0 = no match
 *				otherwise =	match	or	wildcard
 *
 *******************************************************************************/
static uint8 _Supplier_ID_match(uint8 *pSupplier_ID)
{
	return ((pSupplier_ID[0] == (WILDCARD_SUPPLIER_ID & 0xFF) && pSupplier_ID[1] == (WILDCARD_SUPPLIER_ID >> 8)) || (pSupplier_ID[0] == (L_SUPPLIER_ID & 0xFF) && pSupplier_ID[1] == (L_SUPPLIER_ID >> 8)));
}
static uint8 _Function_ID_match(uint8 *pFunction_ID)
{
	return ((pFunction_ID[0] == (WILDCARD_FUNCTION_ID & 0xFF) && pFunction_ID[1] == (WILDCARD_FUNCTION_ID >> 8)) || (pFunction_ID[0] == (L_FUNCTION_ID & 0xFF) && pFunction_ID[1] == (L_FUNCTION_ID >> 8)));
}

const uint16 messageID[L_ID_TABLE_SIZE - 2] = // -2 because there is no message ID for ID_60 and ID_61
	{
		0x2021, // supported message IDs
		0x2223};

const uint8 l_serial_number[4] =
	{
		L_SERIAL_NUMBER & 0xFF, // low byte first
		(L_SERIAL_NUMBER >> 8) & 0xFF,
		(L_SERIAL_NUMBER >> 16) & 0xFF,
		(L_SERIAL_NUMBER >> 24) & 0xFF};

void Receive3C(void)
{
	uint8 i = 0;
	uint8 tmp = 0;
	out_time = 1020; // 多包超时接收时间
	switch (LS.RData[2])
	{
	case 0xB0: // B0服务肯定相应
		if ((LS.RData[0] == INITIAL_NAD || LS.RData[0] == WILDCARD_NAD) &&
			_Supplier_ID_match(&LS.RData[3]) &&
			_Function_ID_match(&LS.RData[5]))
		{
			LS.NAD = LS.RData[7]; //	设置新的 NAD
			LS.SData3D[0] = INITIAL_NAD;
			LS.SData3D[1] = 1;	  //	PCI
			LS.SData3D[2] = 0xF0; //	RSID = SID+0x40
			LS.SData3D[3] = 0xFF; //	unused
			LS.SData3D[4] = 0xFF; //	unused
			LS.SData3D[5] = 0xFF; //	unused
			LS.SData3D[6] = 0xFF; //	unused
			LS.SData3D[7] = 0xFF; //	unused
			LS.Pack = 1;
		}
		break;

	case 0xB1: //	SID=Assign_Frame
		if ((LS.RData[0] == LS.NAD || LS.RData[0] == WILDCARD_NAD) &&
			_Supplier_ID_match(&LS.RData[3]))
		{
			if (messageID[i] == (LS.RData[5] + ((uint16)LS.RData[6] << 8)))
			{
				// l_PID_Table[i]	= lin_ID_60[7];
				//	prepare slave response
				LS.SData3D[0] = LS.NAD;
				LS.SData3D[1] = 1;	  //	PCI
				LS.SData3D[2] = 0xF1; //	RSID = SID+0x40
				LS.SData3D[3] = 0xFF; //	unused
				LS.SData3D[4] = 0xFF; //	unused
				LS.SData3D[5] = 0xFF; //	unused
				LS.SData3D[6] = 0xFF; //	unused
				LS.SData3D[7] = 0xFF; //	unused
				LS.Pack = 1;
				break;
			}
		}
		break;
	case 0xB2: //	SID=Read_by_Identifier
			   //	check	NAD, Supplier ID and	Function	ID
		if ((LS.RData[0] == LS.NAD || LS.RData[0] == WILDCARD_NAD) && (LS.RData[1] == 6) &&
			_Supplier_ID_match(&LS.RData[4]) &&
			_Function_ID_match(&LS.RData[6]))
		{
			//	Check	Identifier
			if (LS.RData[3] == 0) //	read LIN	Product Identification
			{
				LS.Pack = 1;
				//	prepare slave response
				LS.SData3D[0] = LS.NAD;
				LS.SData3D[1] = 0x06;				  //	PCI
				LS.SData3D[2] = 0xF2;				  //	RSID = SID+0x40
				LS.SData3D[3] = L_SUPPLIER_ID & 0xFF; //	supplier	ID
				LS.SData3D[4] = L_SUPPLIER_ID >> 8;
				LS.SData3D[5] = L_FUNCTION_ID & 0xFF; //	function	ID
				LS.SData3D[6] = L_FUNCTION_ID >> 8;
				LS.SData3D[7] = L_VARIANT_ID; //	variant
			}
			else if (LS.RData[3] == 1) //	read serial	number
			{
				//	prepare slave response
				LS.SData3D[0] = LS.NAD;
				LS.SData3D[1] = 0x05; //	PCI
				LS.SData3D[2] = 0xF2; //	RSID = SID+0x40
				LS.SData3D[3] = L_SERIAL_NUMBER & 0xFF;
				LS.SData3D[4] = (L_SERIAL_NUMBER >> 8) & 0xFF;
				LS.SData3D[5] = (L_SERIAL_NUMBER >> 16) & 0xFF;
				LS.SData3D[6] = (L_SERIAL_NUMBER >> 24) & 0xFF;
				LS.SData3D[7] = 0xFF;
			}
			else
			{
				LS.SData3D[0] = LS.NAD;
				LS.SData3D[1] = 0x03; //	PCI
				LS.SData3D[2] = 0x7F; //	RSID
				LS.SData3D[3] = 0xB2; //	requested SID
				LS.SData3D[4] = 0x12; //	error	code
				LS.SData3D[5] = 0xFF;
				LS.SData3D[6] = 0xFF;
				LS.SData3D[7] = 0xFF;
			}
			LS.Pack = 1;
		}

		if ((LS.RData[0] != LS.NAD) && (LS.RData[0] != WILDCARD_NAD) || (LS.RData[1] != 6))
		{
			LS.Pack = 0;
		}
		break;
	case 0xB6:
		if ((LS.RData[0] == LS.NAD || LS.RData[0] == WILDCARD_NAD))
		{
			//	prepare slave response  == B6服务肯定相应
			LS.SData3D[0] = LS.NAD;
			LS.SData3D[1] = 1;	  //	PCI
			LS.SData3D[2] = 0xF6; //	RSID = SID+0x40
			LS.SData3D[3] = 0xFF; //	unused
			LS.SData3D[4] = 0xFF; //	unused
			LS.SData3D[5] = 0xFF; //	unused
			LS.SData3D[6] = 0xFF; //	unused
			LS.SData3D[7] = 0xFF; //	unused
			LS.Pack = 1;
		}
		break;

	case 0xB7: //	SID=assign FID range
			   //	check	initial NAD, Supplier ID and Function ID
		tmp = LS.RData[3];
		if ((LS.RData[0] == LS.NAD || LS.RData[0] == WILDCARD_NAD))
		{
			if (tmp == 0X01)
				tmp = 0X01;
			//	prepare slave response
			LS.SData3D[0] = LS.NAD;
			LS.SData3D[1] = 1;	  //	PCI
			LS.SData3D[2] = 0xF7; //	RSID = SID+0x40
			LS.SData3D[3] = 0xFF; //	unused
			LS.SData3D[4] = 0xFF; //	unused
			LS.SData3D[5] = 0xFF; //	unused
			LS.SData3D[6] = 0xFF; //	unused
			LS.SData3D[7] = 0xFF; //	unused
			LS.Pack = 1;
			/* 				ID:0x20				*/
			if (LS.RData[4] != 0XFF)
				l_PID_Table[tmp] = LS.RData[4];
			if (LS.RData[5] != 0XFF)
				l_PID_Table[tmp + 1] = LS.RData[5];
			if (LS.RData[6] != 0XFF)
				l_PID_Table[tmp + 2] = LS.RData[6];
			if (LS.RData[7] != 0XFF)
				l_PID_Table[tmp + 3] = LS.RData[7];
		}
		break;
	}
}

/* 诊断使用的用户可以删除  end */


/**
	@brief        EOP_回复的lin数据准备 10ms更新一次
	@date         2022-07-14
*/
void EOP_TXLIN(void)
{
	uint8 i = 0;
	if (EOP.TxCnt == 0)
	{
		EOP.TxCnt = 10; // 10ms更新一次数据

		if (EOP.EnableReq_ECF == 0) // 使能位
		{
			EOP.ECF_EnableSt = 0;
		}
		else
		{
			EOP.ECF_EnableSt = 1;
		}

		/* 工作模式设置 */
		if (mcState != mcRun)
		{
			EOP.ECF_WorkingMode = 0; // 空闲
		}
		else if (mcFocCtrl.CtrlMode == 0)
		{
			EOP.ECF_WorkingMode = 1; // 开环
		}
		else if (mcFocCtrl.CtrlMode == 1)
		{
			EOP.ECF_WorkingMode = 2; // 闭环
		}
		else
		{
			EOP.ECF_WorkingMode = 0; // 空闲
		}

		if (mcState == mcReady)
		{
			EOP.ECF_WorkingSt = 0;
		}
		else if (mcState == mcRun)
		{
			EOP.ECF_WorkingSt = 1;
		}

		EOP.ECF_Speed = (uint8)(mcFocCtrl.SpeedFlt / S_Value(15.0));		  // 反馈转速 精度：15rpm*Inc
		EOP.ECF_Voltage = (uint8)((mcFocCtrl.mcDcbusFlt / UDC_Value(0.1)));	  // 母线电压 分辨率0.1V
		EOP.ECF_Current = (uint16)(mcFocCtrl.mcADCCurrentbus / I_Value(0.1)); // 母线电流 分辨率0.1A
		EOP.ECF_Power = (EOP.ECF_Voltage * EOP.ECF_Current) / 100;            // 功率，需标定功率系数

		// 缺相 相短故障
		if (mcFaultSource == FaultHardOVCurrent || mcFaultSource == FaultPhaseLost)
		{
            EOP.ECF_HardwareError = 1;
		}

		// 软件过流故障
		if (mcFaultSource == FaultSoftOVCurrent)
		{
            EOP.ECF_OverCurrentError = 1;
		}

		// 欠压故障
		if (mcFaultSource == FaultUnderVoltageDC)
		{
            EOP.ECF_UnderVoltageError = 1;
		}

		// 过压故障延时
		if (mcFaultSource == FaultOverVoltageDC)
		{
            EOP.ECF_OverVoltageError = 1;
		}
        // 堵转故障延时
		if (mcFaultSource == FaultStall)
		{
			EOP.ECF_StallError = 1;
		}

		// 过温故障延时
		if (mcFaultSource == FaultNtcOTErr || mcFaultSource == FaultTSD)
		{
            EOP.ECF_OverTempError = 1;
		}

		// 故障总使能位
		if (mcFaultSource ==FaultNoSource )
		{
           EOP.ECF_HardwareError = 0;
           EOP.ECF_OverCurrentError = 0;
           EOP.ECF_UnderVoltageError = 0;
           EOP.ECF_OverVoltageError = 0;
           EOP.ECF_StallError = 0;
           EOP.ECF_OverTempError = 0;
           EOP.ECF_FaultSt = 0;
		}
        else
        {
            EOP.ECF_FaultSt = 1;
            EOP.ECF_WorkingSt = 0;
            EOP.ECF_WorkingMode = 0;
        }
            
	}
}
/**
	@brief        EOP_接收数据
	@date         2022-07-14
*/
void EOP_RXLIN(void)
{
	// 有使能信号并且有目标转速值
	if (EOP.EnableReq_ECF == 1)
	{
		if (EOP.SpeedReq_ECF <= 20) // 0-20 停止
		{
			isCtrlPowOn = false;
			mcFocCtrl.Ref = 0;
		}
		else if (EOP.SpeedReq_ECF <= 44) // 21-44 660rpm
		{
			isCtrlPowOn = true;
			mcFocCtrl.Ref = MOTOR_SPEED_MIN_RPM;
		}
		else if (EOP.SpeedReq_ECF <= 180) // 精度：15rpm*Inc  转速换算
		{
			isCtrlPowOn = true;
			mcFocCtrl.Ref = S_Value(EOP.SpeedReq_ECF * 15); // 精度：15rpm*Inc  转速换算
		}
		else
		{
			isCtrlPowOn = true; // 开机
			mcFocCtrl.Ref = MOTOR_SPEED_MAX_RPM;
		}
	}
	else
	{
		isCtrlPowOn = false; // 关机
		mcFocCtrl.Ref = 0;	 //  停止
	}
}
