/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      main.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2022-12-29
 * @date      Last modify:2022-12-29
 * @note      Last modify author is Leo.li
 * @brief     Can Init/ Send /Receive/deal with
 */


#include <MyProject.h>
#include "RingBuffer.h"
#include "CanSlave.h"

#define     CAN_RCV_BUF_SIZE  256
// 数据长度
#define CAN_FRAME_DATA_LEN      8       // 标准CAN帧数据长度

CAN_Data xdata Can;
CAN_Data xdata CanMsg;
CAN_Data xdata CanSend;
RingBuffer_t xdata Can_RcvRb;
uint8_t xdata Can_RcvBuf[CAN_RCV_BUF_SIZE];

/**********************************过滤模式*********************************/
/*****变量说明：FIL_SIN：0：双过滤器模式            1：单过滤器模式*****************/
/*****变量说明：ID_MOD ：0：过滤标准帧              1：过滤扩展帧*******************/
#define     FIL_SIN    1
#define     ID_MOD     1

/**
* @brief     Can初始化
* @date      2022-12-29
*/
void CAN_Init(void)
{
    uint8   IDE_FIL;
    uint16  BRP;
    uint8   SEG2;
    uint8   SEG1;
    uint8   CAN_IDE;
    uint8   CAN_RTR;
	  uint32  EXTID1;
//    uint8   Data0;
//    uint8   Data1;
//    uint32  EXTID2;
//    uint32  STDID1;
//    uint32  STDID2;


	/*****配置Can接收芯片使能*****/
	// SetBit(P0_OE , P07);
    // GP07 = 0;

    /***************************清0避免写入bug**********************************/
    CAN_BTR0 = 0;
    CAN_BTR1 = 0;
    CAN_BTR2 = 0;
    /*  该位域定义了为了重新同步，在每位中可以延长或缩短多少个时间单元的上限tRESYN
        tRESYN=tq*(RESYN[1:0]+1)                                                                                            */
    //  SetBit(CAN_BTR0,SJW1);
    //  SetBit(CAN_BTR0,SJW0);
    /*****************  时间触发采样模式        0：禁止     1：使能     *****************/
    //  SetBit(CAN_BTR2,TTCM);
    /* 功能：CAN模块波特率设置
	   1M ：BRP  = 3; SEG2 = 0; SEG1 = 3;
	   500K ：BRP  = 5; SEG2 = 0; SEG1 = 5;
       250K ：BRP  = 11; SEG2 = 0; SEG1 = 5;
	   */
    BRP  = 11;                                                               //范围0-0x3FF           时间单元tq=(BRP+1)/ 24MHz
    SEG2 = 0;                                                               //范围0-0x07            时间段2时间长度tSEG2=tq*(SEG2+1)
    SEG1 = 5;                                                               //范围0-0x0F            时间段1时间长度tSEG1=tq*(SEG1+1)
    CAN_BTR0 |= (BRP >> 8);
    CAN_BTR1  =  BRP;
    CAN_BTR2 |= (SEG2 << 4) | SEG1;

    IDE_FIL = 0;                                                            //选择过滤模式：1：过滤扩展帧 0：过滤标准帧
    #if FIL_SIN                                                             //单过滤模式
    {
        SetBit(CAN_CR0, FILMOD);
        #if ID_MOD                                                          //过滤扩展帧
        {
            EXTID1   = 0x1FFFFFFF;                                            //0-0x1FFF
            CAN_IDE  = 1;                                                    //过滤扩展帧
            CAN_RTR  = 1;                                                    //过滤远程帧
            CAN_FIR0 = EXTID1 >> 21;
            CAN_FIR1 = ((EXTID1 >> 18) & 0xE0) | (CAN_IDE << 3) | (CAN_RTR << 4) | ((EXTID1 >> 15) & 0x07);
            CAN_FIR2 = EXTID1 >> 7 ;
            CAN_FIR3 = EXTID1 << 1 ;
            CAN_FMR0 = 0xFF;
            CAN_FMR1 = 0xFF;
            CAN_FMR2 = 0xFF;
            CAN_FMR3 = 0xFF;
        }
        #else                                                                //过滤标准帧
        {
            STDID1 = 0x7FF;                                                  //0-0x7FF
            CAN_IDE = 1;                                                     //过滤扩展帧
            CAN_RTR = 1;                                                     //过滤远程帧
            Data0   = 0xFF;                                                  //过滤第一字节
            Data1   = 0xFF;                                                  //过滤第二字节
            CAN_FIR0 = STDID1 >> 3;
            CAN_FIR1 = ((STDID1 << 5) & 0xE0) | (CAN_IDE << 3) | (CAN_RTR << 4) ;
            CAN_FIR2 = Data0;
            CAN_FIR3 = Data1;
            CAN_FMR0 = 0xFF;
            CAN_FMR1 = 0xFF;
            CAN_FMR2 = 0xFF;
            CAN_FMR3 = 0xFF;
        }
        #endif
    }
    #else                                                                    //双过滤模式
    {
        ClrBit(CAN_CR0, FILMOD);
        #if ID_MOD                                                           //过滤扩展帧
        {
            EXTID1 = 0x1FFFFFFF;                                             //0-0x1FFF8000，只可更改EXTID1[15]-[28]
            EXTID2 = 0x1FFFFFFF;                                             //0-0x1FFF8000，只可更改EXTID1[15]-[28]
            CAN_IDE = 1;                                                     //过滤扩展帧
            CAN_RTR = 1;                                                     //过滤远程帧
            CAN_FIR0 = EXTID1 >> 21;
            CAN_FIR1 = ((EXTID1 >> 18) & 0xE0) | (CAN_IDE << 3) | (CAN_RTR << 4) | ((EXTID2 >> 15) & 0x07);
            CAN_FIR0 = EXTID2 >> 21;
            CAN_FIR1 = ((EXTID2 >> 18) & 0xE0) | (CAN_IDE << 3) | (CAN_RTR << 4) | ((EXTID2 >> 15) & 0x07);
            CAN_FMR0 = 0xFF;
            CAN_FMR1 = 0xFF;
            CAN_FMR2 = 0xFF;
            CAN_FMR3 = 0xFF;
        }
        #else                                                                                                       //过滤标准帧
        {
            STDID1   = 0x7FF;                                                  //0-0x7FF
            STDID2   = 0x7FF;                                                  //0-0x7FF
            CAN_IDE  = 1;                                                     //过滤扩展帧
            CAN_RTR  = 1;                                                     //过滤远程帧
            CAN_FIR0 = STDID1 >> 3;
            CAN_FIR1 = ((STDID1 << 5) & 0xE0) | (CAN_IDE << 3) | (CAN_RTR << 4) ;
            CAN_FIR2 = STDID2 >> 3;
            CAN_FIR3 = ((STDID2 << 5) & 0xE0) | (CAN_IDE << 3) | (CAN_RTR << 4) ;
            CAN_FMR0 = 0xFF;
            CAN_FMR1 = 0xFF;
            CAN_FMR2 = 0xFF;
            CAN_FMR3 = 0xFF;
        }
        #endif
    }
    #endif

    /*---------------------CAN协议版本-----------------------------------*/
    /*-------------------0：CAN2.0A/B，支持标准帧、扩展帧-----------------/
    /*------------------------1：CAN1.0，仅支持标准帧--------------------*/
    ClrBit(CAN_CR0, CANREV);
    /*当检测到总线活动后，硬件自动对CAN_CR1[SLEEP]清0来唤醒CAN*/
    ClrBit(CAN_CR0, AWAKMOD);

    /*----------------LISTEN置1 监听模式使能--------------*/
		/*----------------SELFTST置1 自测模式使能--------------*/
    ClrBit(CAN_CR0, SELFTST);
    ClrBit(CAN_CR0, LISTEN );
    /*在复位模式下，CAN控制器处于复位状态，不可以接收或发送报文。对CAN_CR0[RSTMOD]清‘0’以进入复位模式*/
    ClrBit(CAN_CR0, RSTMOD);
    SetBit(CAN_IER, ETYIE | BOFIE | PERIE | ERWIE | OVIE | ARBIE | TXIE | RXIE);    //中断开启选择
    // SetBit(CAN_CR0, CAN_CH);                    //端口转移：0:txd→P00,rxd→P01 1:txd→P05,rxd→P06
    SetBit(CAN_CR0, CANEN);                     //CAN模块使能

    rb_init(&Can_RcvRb, Can_RcvBuf, CAN_RCV_BUF_SIZE);
}

bool CAN_MsgIsEmpty(void)
{
    return rb_is_empty(&Can_RcvRb);
}

bool CAN_MsgIsFull(void)
{
    return rb_count(&Can_RcvRb) == (CAN_RCV_BUF_SIZE - CAN_RCV_BUF_SIZE % sizeof(CAN_Data));
}

bool CAN_MsgPush(CAN_Data *pMsg)
{
    uint8 *pData = (uint8 *)pMsg;
    int i = 0;
    if (CAN_MsgIsFull())
    {
        return false;
    }
    for (i; i < sizeof(CAN_Data); i++)
    {
        rb_write(&Can_RcvRb, pData[i]);
    }
    return true;
}

bool CAN_MsgPop(CAN_Data *pMsg)
{
    uint8 *pData = (uint8 *)pMsg;
    int i = 0;
    if (CAN_MsgIsEmpty())
    {
        return false;
    }
    for (i; i < sizeof(CAN_Data); i++)
    {
        if (!rb_read(&Can_RcvRb, &pData[i]))
        {
            return false;
        }
    }
    return true;
}


/* 函数名称：CAN_Read
 * 函数功能：读取CAN总线上对应函数
 * 参数说明：
 * 注意事项：若开启中断，需要注释掉SetBit(CAN_CR1, BUFRLS);
 */
void CAN_Read(void)
{
    uint8 rdptr;
    rdptr = CAN_CR2 & (RDPTR1 | RDPTR0);

    switch (rdptr)
    {
        case (0 | 0):
            Can.CAN_IDE = ( CAN_RX0CR & 0x80 );
            Can.CAN_RTR = ( CAN_RX0CR & 0x40 );
            Can.CAN_DLC = ( CAN_RX0CR & 0x0F );

            if (Can.CAN_IDE)
            {
                Can.EXTID = 0;
                Can.EXTID |= (uint32)CAN_RX0ID0 << 21;
                Can.EXTID |= (uint32)CAN_RX0ID1 << 13;
                Can.EXTID |= (uint32)CAN_RX0ID2 << 5 ;
                Can.EXTID |= (uint32)CAN_RX0ID3 >> 3 ;//
            }
            else
            {
                Can.STDID = 0;
                Can.STDID |= CAN_RX0ID0 << 3 ;
                Can.STDID |= CAN_RX0ID1 >> 5;
            }

            Can.Data0 = CAN_RX0DR0;
            Can.Data1 = CAN_RX0DR1;
            Can.Data2 = CAN_RX0DR2;
            Can.Data3 = CAN_RX0DR3;
            Can.Data4 = CAN_RX0DR4;
            Can.Data5 = CAN_RX0DR5;
            Can.Data6 = CAN_RX0DR6;
            Can.Data7 = CAN_RX0DR7;
            CAN_MsgPush(&Can);
            break;

        case (0 | RDPTR0):
            Can.CAN_IDE = ( CAN_RX1CR & 0x80 );
            Can.CAN_RTR = ( CAN_RX1CR & 0x40 );
            Can.CAN_DLC = ( CAN_RX1CR & 0x0F );

            if (Can.CAN_IDE)
            {
                Can.EXTID = 0;
                Can.EXTID |= (uint32)CAN_RX1ID0 << 21;
                Can.EXTID |= (uint32)CAN_RX1ID1 << 13;
                Can.EXTID |= (uint32)CAN_RX1ID2 << 5 ;
                Can.EXTID |= (uint32)CAN_RX1ID3 >> 3 ;
            }
            else
            {
                Can.STDID = 0;
                Can.STDID |= CAN_RX1ID0 << 3 ;
                Can.STDID |= CAN_RX1ID1 >> 5 ;
            }

            Can.Data0 = CAN_RX1DR0;
            Can.Data1 = CAN_RX1DR1;
            Can.Data2 = CAN_RX1DR2;
            Can.Data3 = CAN_RX1DR3;
            Can.Data4 = CAN_RX1DR4;
            Can.Data5 = CAN_RX1DR5;
            Can.Data6 = CAN_RX1DR6;
            Can.Data7 = CAN_RX1DR7;
            CAN_MsgPush(&Can);
            break;

        case (RDPTR1 | 0):
            Can.CAN_IDE = ( CAN_RX2CR & 0x80 );
            Can.CAN_RTR = ( CAN_RX2CR & 0x40 );
            Can.CAN_DLC = ( CAN_RX2CR & 0x0F );

            if (Can.CAN_IDE)
            {
                Can.EXTID = 0;
                Can.EXTID |= (uint32)CAN_RX2ID0 << 21;
                Can.EXTID |= (uint32)CAN_RX2ID1 << 13;
                Can.EXTID |= (uint32)CAN_RX2ID2 << 5 ;
                Can.EXTID |= (uint32)CAN_RX2ID3 >> 3 ;
            }
            else
            {
                Can.STDID = 0;
                Can.STDID |= CAN_RX2ID0 << 3 ;
                Can.STDID |= CAN_RX2ID1 >> 5;
            }

            Can.Data0 = CAN_RX2DR0;
            Can.Data1 = CAN_RX2DR1;
            Can.Data2 = CAN_RX2DR2;
            Can.Data3 = CAN_RX2DR3;
            Can.Data4 = CAN_RX2DR4;
            Can.Data5 = CAN_RX2DR5;
            Can.Data6 = CAN_RX2DR6;
            Can.Data7 = CAN_RX2DR7;
            CAN_MsgPush(&Can);
            break;

        default:
            break;
    }

    SetBit(CAN_CR1, BUFRLS);                                    //如果开了中断请注释这句
}

uint8 can_hw_send(uint32_t can_id, const uint8_t *dat, uint8_t len)
{
    if (len != CAN_FRAME_DATA_LEN || dat == NULL) {
        return 0;
    }
    // 根据ID值自动判断帧类型: >0x7FF使用扩展帧
    if (can_id > 0x7FF) {
        CanSend.CAN_IDE = IDE_EXT;
    } else {
        CanSend.CAN_IDE = IDE_STD;
    }
    CanSend.CAN_RTR = RTR_DAT;
    CanSend.CAN_DLC = len;
    CanSend.EXTID = CanSend.STDID = can_id;

    memcpy(&CanSend.Data0, dat, len);

    return CAN_Send();
}


/* 函数名称：CAN_Send
 * 函数功能：发送CAN数据
 * 参数说明：
 * 注意事项：无
 */
uint8 CAN_Send(void)
{
    uint16 timeout = 10000;
    while (!ReadBit(CAN_SR, TXSUC))
    {
        if (--timeout == 0) return 0;
    }

    CAN_TXCR  = CanSend.CAN_IDE  | CanSend.CAN_RTR | CanSend.CAN_DLC;

    if (CanSend.CAN_IDE)
    {
        CAN_TXID0 = CanSend.EXTID >> 21;
        CAN_TXID1 = CanSend.EXTID >> 13;
        CAN_TXID2 = CanSend.EXTID >> 5;
        CAN_TXID3 = CanSend.EXTID << 3;     //4
    }
    else
    {
        CAN_TXID0 = CanSend.STDID >> 3;
        CAN_TXID1 = CanSend.STDID << 5;
        CAN_TXID2 = 0;
        CAN_TXID3 = 0;
    }

    CAN_TXDR0 = CanSend.Data0;
    CAN_TXDR1 = CanSend.Data1;
    CAN_TXDR2 = CanSend.Data2;
    CAN_TXDR3 = CanSend.Data3;
    CAN_TXDR4 = CanSend.Data4;
    CAN_TXDR5 = CanSend.Data5;
    CAN_TXDR6 = CanSend.Data6;
    CAN_TXDR7 = CanSend.Data7;
    SetBit(CAN_CR1, TXREQ);
    return 1;
}

void CAN_MsgParse(void)
{
    if (CAN_MsgIsEmpty())
    {
        return;
    }
    memset(&CanMsg, 0, sizeof(CAN_Data));
    CAN_MsgPop(&CanMsg);

    if (CanMsg.CAN_IDE == IDE_EXT)
    {
        ext_slave_process_rx(CanMsg.EXTID, &CanMsg.Data0, CAN_FRAME_DATA_LEN);
    }
}

