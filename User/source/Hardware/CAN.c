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
#define     CAN_FRAME_DATA_LEN      8       // 标准CAN帧数据长度

bool CAN_MsgIsEmpty(void);
bool CAN_MsgIsFull(void);
bool CAN_MsgPush(CAN_Data *pMsg);
bool CAN_MsgPop(CAN_Data *pMsg);

CAN_Data xdata Can;
CAN_Data xdata CanMsg;
CAN_Data xdata CanSend;
RingBuffer_t xdata Can_RcvRb;
uint8_t xdata Can_RcvBuf[CAN_RCV_BUF_SIZE];

/* ==================== 【调试用】开关 + 计数器 ==================== */
/* ★ RSTMOD 极性 A/B 开关（本次排查核心）
 *   1 = 执行 ClrBit(CAN_CR0, RSTMOD)（20261013 移植时新加的写法）
 *   0 = 注释掉这一句（= 初版 / Software_24v_20261008 / 20261010 / 20261012 的写法，
 *       RSTMOD 保持复位值 1）
 *
 * 【实测记录 15:56，用 1 时】
 *   CR0 读回 0x88（CANEN=1 FILMOD=1 CAN_CH=0 RSTMOD=0）→ 初始化写进去了、引脚也对；
 *   但 SR=0x0C 永远空闲、TEC=REC=0、rx=0、err/ovf/arb=0、协议层 in=0
 *   → 控制器"已使能但完全不收发，连发送错误计数都不涨"，即停在复位模式的特征
 *     （正常发出去没人 ACK 时 TEC 会一路涨到 bus-off）。
 *   初版能跑通，而初版这一句是注释状态 => 按 0 编译复测。*/
#define CAN_DEBUG_RSTMOD_CLEAR   0

/* ★ 自发自收实验（用来区分"控制器坏了"和"总线/主机问题"）
 *   1 = 打开 CAN_CR1.TXSELF(0x10 自发自收) + NOART(0x02 禁自动重传)
 *   打开后如果 rx 计数涨了 → 控制器/波特率/代码通路都好，问题在总线或主机；
 *   如果 rx 还是 0 → CAN 控制器压根没在工作 */
#define CAN_DEBUG_LOOPBACK       0

volatile uint16 xdata g_can_rxIntCnt  = 0;
volatile uint16 xdata g_can_popCnt    = 0;
volatile uint16 xdata g_can_txCnt     = 0;
volatile uint16 xdata g_can_txToCnt   = 0;
volatile uint16 xdata g_can_errCnt    = 0;
volatile uint16 xdata g_can_ovifCnt   = 0;
volatile uint16 xdata g_can_arbCnt    = 0;
volatile uint8  xdata g_can_cr0       = 0;
volatile uint8  xdata g_can_srAfterTx = 0;
volatile uint8  xdata g_can_ifrLast   = 0;
volatile uint16 xdata g_can_busLowCnt = 0;   /* 【调试】1ms 任务里采样 CANRX(P0.1) 出现显性的次数 */

volatile uint8 xdata g_dbgCmdLast    = 0;

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
	// SetBit(P0_OE , P07);  //当前P0.7 = EEPROM_WP，与 CAN 无关。
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
#if CAN_DEBUG_RSTMOD_CLEAR
    ClrBit(CAN_CR0, RSTMOD);
#endif
    // ClrBit(CAN_CR0, RSTMOD);
    SetBit(CAN_IER, ETYIE | BOFIE | PERIE | ERWIE | OVIE | ARBIE | TXIE | RXIE);                 //中断开启选择
    // SetBit(CAN_CR0, CAN_CH);                                                                     //端口转移：0:txd→P00,rxd→P01 1:txd→P05,rxd→P06
    ClrBit(CAN_CR0, CAN_CH);        // 当前硬件版本：锁死 CAN 在 P0.0 / P0.1，禁止改成 SetBit！
    SetBit(CAN_CR0, CANEN);                                                                      //CAN模块使能
    
#if CAN_DEBUG_LOOPBACK
    SetBit(CAN_CR1, TXSELF);        /* 自发自收，不依赖总线 ACK */
    SetBit(CAN_CR1, NOART);         /* 禁止自动重传，避免无 ACK 时无限重发 */
#endif
    /* 【调试】回读关键寄存器，确认写入真的生效 */
    g_can_cr0     = CAN_CR0;
    g_can_ifrLast = CAN_IFR;

    /* 接收环形缓冲初始化。size 必须是 2 的整数次幂（256 满足）*/
    rb_init(&Can_RcvRb, Can_RcvBuf, CAN_RCV_BUF_SIZE);
}

/* 函数名称：CAN_Read
 * 函数功能：读取CAN总线上对应函数
 * 参数说明：
 * 注意事项：若开启中断，需要注释掉SetBit(CAN_CR1, BUFRLS);
 */
void CAN_Read(void)
{
    uint8 rdptr;
    g_can_rxIntCnt++;                       /* 【调试】收到一帧（RXIF） */
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
			
			// CAN_Send();
            CAN_MsgPush(&Can);          //  改为入接收环形缓冲，交给 CAN_MsgParse() 解析
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

            CAN_MsgPush(&Can);          // 改为入接收环形缓冲，交给 CAN_MsgParse() 解析
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

            CAN_MsgPush(&Can);          // 改为入接收环形缓冲，交给 CAN_MsgParse() 解析
            break;
            
        default:
            break;
    }
		
    // SetBit(CAN_CR1, BUFRLS);                                    //如果开了中断请注释这句
}

/* 新增：接收缓冲接口（中断里 Push，1ms 任务里 Pop）*/
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
        return false;                       // 满则丢新帧
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

/* 函数名称：CAN_Send
 * 函数功能：发送CAN数据（使用全局 CanSend，非阻塞化：带超时）
 * 注意事项： 原厂是 while(!TXSUC) 死等，总线上没接收方时会永久卡死；
 *           改成超时返回，配合上层发送队列(每1ms只发1帧)做限流。
 */
uint8 CAN_Send(void)
{
    // while (!ReadBit(CAN_SR, TXSUC));
    uint16 timeout = 10000;
    g_can_txCnt++;                          /* 【调试】发送调用计数 */
    while (ReadBit(CAN_SR, TXING))
    {
        if (--timeout == 0)
        {
            g_can_txToCnt++;                /* 【调试】等 TXING 超时 */
            break;                          // 超时也继续把新帧写下去
        }
    }
    
    CAN_TXCR  = CanSend.CAN_IDE  | CanSend.CAN_RTR | CanSend.CAN_DLC;
    
    if (CanSend.CAN_IDE)
    {
        CAN_TXID0 = CanSend.EXTID >> 21;
        CAN_TXID1 = CanSend.EXTID >> 13;
        CAN_TXID2 = CanSend.EXTID >> 5;
        CAN_TXID3 = CanSend.EXTID << 3;
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
    g_can_srAfterTx = CAN_SR;               /* 【调试】发送后的状态快照 */
    return 1;
}

/* 统一发送入口。len 0~8；can_id > 0x7FF 自动用扩展帧 */
uint8 can_hw_send(uint32_t can_id, const uint8_t *dat, uint8_t len)
{
    if (len > CAN_FRAME_DATA_LEN || dat == NULL) {
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

/* 新增：1ms 任务里调用。环非空则弹出一帧，扩展帧交给协议层解析 */
void CAN_MsgParse(void)
{
    /* 【调试】总线活动采样：CANRX(P0.1) 为显性(低)就计数。
     * 本机自己发帧也会被算进去（约 11 帧/s => 约 11 次/s），
     * 所以这个计数"明显跑得比 11/s 快"就说明主机真的在往总线上发东西。*/
    {
        uint8 k;

        for (k = 0; k < 4; k++)
        {
            if ((P0 & 0x02) == 0) g_can_busLowCnt++;
        }
    }

    if (CAN_MsgIsEmpty())
    {
        return;
    }
    memset(&CanMsg, 0, sizeof(CAN_Data));
    CAN_MsgPop(&CanMsg);
    g_can_popCnt++;                         /* 【调试】出队计数 */

    if (CanMsg.CAN_IDE == IDE_EXT)
    {
        ext_slave_process_rx(CanMsg.EXTID, &CanMsg.Data0, CAN_FRAME_DATA_LEN);
    }
}

/* ==================== 【调试用】CAN 状态打印 ====================
 * 由 main.c 大循环每 500ms 调一次，绝不要放进中断或 1ms 任务。
 * Can / CanMsg 会被中断随时改写，这里读到的是"最近一次"快照。
 * 协议层那几个计数（in/bad/filt/cmd…）在 CanSlave.c 的 ext_slave_debug_print()
 */
void CAN_DebugPrint(void)
{
    /* 全部实参统一 (uint16) + %u —— 踩过的坑：C51 的 printf 是可变参数、实参走栈，
     * 只要有一处实参宽度和格式符不匹配（例如传 (uint16) 却用 %bx 只吃 1 字节），
     * 后面所有参数会整体错位，打出来的数全是"看起来像真的"的假值。*/

    /* --- 寄存器（直接读硬件，不是软件计数）---------------------------------------------------
     * CR0 = CAN_CR0 控制寄存器0 : bit0=RSTMOD(复位模式) bit6=CAN_CH(引脚转移) bit7=CANEN(模块使能)
     *                             正常 = 136 (0x88 = CANEN|FILMOD；CAN_CH=0、RSTMOD=0)
     *                             137(0x89) = RSTMOD 读回是 1
     *                             200/201(0xC8/0xC9) = CAN_CH=1，CAN 跑到 P0.5/P0.6 去了
     *                             8(0x08) = CANEN 没使能
     * SR  = CAN_SR  状态寄存器   : bit5=TXING(正在发送) bit4=RXING(正在接收) bit3=TXSUC(发送成功)
     *                             正常 = 12 (0x0C) 空闲；偶尔 32(0x20) 是正好抓在发送瞬间
     * CR1 = CAN_CR1 控制寄存器1  : bit0=TXREQ(发送请求) bit4=TXSELF(自发自收) bit5=SLPREQ(睡眠请求)
     *                             正常 = 0（TXREQ 被硬件自动清掉）；一直=1 说明发送请求没被受理
     * IFR = CAN_IFR 中断标志     : 1=RXIF 2=TXIF 4=ARBIF 8=OVIF 16=ERRIF
     *                             正常 = 0（ISR 会逐个清掉，这里只是中断入口的快照）
     * ---------------------------------------------------------------------------------------- */
    printf("[CAN] CR0=%u SR=%u CR1=%u IFR=%u\r\n",
           (uint16)CAN_CR0, (uint16)CAN_SR, (uint16)CAN_CR1, (uint16)CAN_IFR);

    /* --- 配置回读 + 发送错误计数 --------------------------------------------------------------
     * IER  = CAN_IER 中断使能      : 正常 = 255 (0xFF，8 个中断全开)
     * BTR1 = 波特率低位 = BRP      : 正常 = 11  (tq=(11+1)/24MHz = 0.5us)
     * BTR2 = 段配置=(SEG2<<4)|SEG1 : 正常 = 5 → 1+6+1 = 8tq/位 → 250kbps
     * TEC  = 发送错误计数          : 正常 = 0；一直往上涨 = 发出去没人 ACK（对端没接/没解出来）
     * ---------------------------------------------------------------------------------------- */
    printf("[CAN] IER=%u BTR1=%u BTR2=%u TEC=%u\r\n",
           (uint16)CAN_IER, (uint16)CAN_BTR1, (uint16)CAN_BTR2, (uint16)CAN_TEC);

    /* --- 接收侧错误 + 引脚状态 ----------------------------------------------------------------
     * REC  = 接收错误计数  : 正常 = 0；涨 = 收到波形但解不出来（波特率不对/接线问题）
     * ESR  = 错误状态寄存器: bit7=bus-off  bit6=错误警告(计数≥96)；正常 = 0
     * P0_OE= P0 输出使能   : 正常 = 128 (0x80，只有 bit7=P0.7 EEPROM_WP 是输出)
     *                        bit0/bit1 若为 1 => P0.0/P0.1 被当成 GPIO 占用了
     * rx   = g_can_rxIntCnt: RXIF 中断次数 = 硬件真正收到几帧（主机发多少就涨多少）
     * ---------------------------------------------------------------------------------------- */
    printf("[CAN] REC=%u ESR=%u P0_OE=%u rx=%u\r\n",
           (uint16)CAN_REC, (uint16)CAN_ESR, (uint16)P0_OE, g_can_rxIntCnt);

    /* --- 收发软件计数（在 CAN 中断 / 1ms 任务里累加）-------------------------------------------
     * pop = g_can_popCnt : CAN_MsgParse 出队帧数，正常与 rx 同步涨；
     *                      不涨而 rx 涨 => 1ms 任务没在跑（主循环卡住 / 看门狗复位）
     * tx  = g_can_txCnt  : CAN_Send 调用次数，正常约 11~15/s（心跳+状态+日志）
     * to  = g_can_txToCnt: 等 TXING 超时次数，正常 0；涨 => 总线上没人 ACK / 总线卡住
     * err = g_can_errCnt : ERRIF 错误中断次数，正常 0
     * ---------------------------------------------------------------------------------------- */
    printf("[CAN] pop=%u tx=%u to=%u err=%u\r\n",
           g_can_popCnt, g_can_txCnt, g_can_txToCnt, g_can_errCnt);

    /* --- 溢出 / 仲裁丢失 + 发送后状态 ---------------------------------------------------------
     * ovf  = g_can_ovifCnt   : OVIF 接收 FIFO 溢出，正常 0（涨 = 主循环被打印拖慢来不及取）
     * arb  = g_can_arbCnt    : ARBIF 仲裁丢失，正常 0；狂涨 = 波特率不对
     * srTx = g_can_srAfterTx : 每次 SetBit(TXREQ) 之后"立刻"读的 CAN_SR 快照
     *                          0 = 发送已启动（TXSUC/TXDONE 被清）；bit3=1 说明上一帧发成功
     * ---------------------------------------------------------------------------------------- */
    printf("[CAN] ovf=%u arb=%u srTx=%u\r\n",
           g_can_ovifCnt, g_can_arbCnt, (uint16)g_can_srAfterTx);

    /* busLow = g_can_busLowCnt: 1ms 任务里采样 CANRX(P0.1) 出现显性(低)的次数。
     *                          只算本机自己发的帧约 11/s（即每 500ms 那一行 +5~6）；
     *                          明显更快 => 主机真的在往总线上发东西 */
    printf("[CAN] busLow=%u\r\n", g_can_busLowCnt);   /* 明显大于 11/s 就说明主机在发 */

    /* --- 最近收到的一帧（快照，会被中断随时改写）----------------------------------------------
     * rxId  = Can.EXTID    最近收到的 29 位扩展 ID（十六进制输出）；0 = 还没收到过
     * dlc   = Can.CAN_DLC  数据长度，正常 = 8
     * d0/d1 = Can.Data0/1  载荷前两个字节【★这里是十进制！】
     *                      指令帧 d0 就是 ctrl 字节：34=0x22(急停) 18=0x12(停机) 10=0x0A(起机)
     *                      d1 是目标转速高字节（0x7F 表示零值那一段）
     * 【协议层】in/bad/filt/cmd、发送队列 txQ、心跳/状态/日志节拍 hb/st/lg
     *          在 CanSlave.c 的 ext_slave_debug_print() 里
     * ---------------------------------------------------------------------------------------- */
    printf("[CAN] rxId=%lx dlc=%u d0=%u d1=%u\r\n",
           Can.EXTID, (uint16)Can.CAN_DLC, (uint16)Can.Data0, (uint16)Can.Data1);
}

/* ==================== 【调试用】CAN 上电自检 ====================
 * 说明见 CAN.h。这里只做两件事：读引脚电平 + 自发自收。
 * 实参全部 (uint16) 配 %u（除了 long 用 %lx），避免可变参数错位。
 */

/* 发一帧 0x12345678 并等最多约 20ms，返回期间收到的帧数 */
static uint8 CAN_LoopbackOnce(void)
{
    uint16 i;
    uint8  dat[8];

    memset(dat, 0x11, 8);

    g_can_rxIntCnt = 0;                 /* 只看这一次的结果 */

    CanSend.CAN_IDE = IDE_EXT;
    CanSend.CAN_RTR = RTR_DAT;
    CanSend.CAN_DLC = CAN_FRAME_DATA_LEN;
    CanSend.EXTID   = 0x12345678UL;
    memcpy(&CanSend.Data0, dat, CAN_FRAME_DATA_LEN);
    CAN_Send();

    for (i = 0; i < 30000; i++)
    {
        if (g_can_rxIntCnt != 0) break;
    }

    return (uint8)g_can_rxIntCnt;
}

void CAN_SelfTest(void)
{
    uint8 r0, r1;
    uint8 cr0_save;

    /* ① 引脚电平 + 寄存器现状 */
    printf("[ST] P0=%u CR0=%u SR=%u CR1=%u\r\n",
           (uint16)P0, (uint16)CAN_CR0, (uint16)CAN_SR, (uint16)CAN_CR1);
    printf("[ST] IFR=%u IER=%u TEC=%u REC=%u\r\n",
           (uint16)CAN_IFR, (uint16)CAN_IER, (uint16)CAN_TEC, (uint16)CAN_REC);

    /* 开自发自收 + 禁止自动重传：这样不需要总线上有第二个节点来 ACK */
    SetBit(CAN_CR1, TXSELF);
    SetBit(CAN_CR1, NOART);

    /* ② A：保持当前配置（RSTMOD=0）试一次
     *    注意 r0/r1 必须转成 (uint16) —— 直接传 uint8 配 %u 会让后面所有参数错位 */
    r0 = CAN_LoopbackOnce();
    printf("[ST] A RSTMOD0 rx=%u ide=%u SR=%u IFR=%u\r\n",
           (uint16)r0, (uint16)Can.CAN_IDE, (uint16)CAN_SR, (uint16)CAN_IFR);
    printf("[ST] A ext=%lx std=%lx\r\n", Can.EXTID, Can.STDID);

    /* ③ B：把 RSTMOD 置 1 再试一次 */
    cr0_save = CAN_CR0;
    SetBit(CAN_CR0, RSTMOD);
    r1 = CAN_LoopbackOnce();
    printf("[ST] B RSTMOD1 rx=%u id=%lx SR=%u IFR=%u\r\n",
           (uint16)r1, Can.EXTID, (uint16)CAN_SR, (uint16)CAN_IFR);

    /* ④ 恢复原状，后面按正常流程跑 */
    CAN_CR0 = cr0_save;
    ClrBit(CAN_CR1, TXSELF);
    ClrBit(CAN_CR1, NOART);
    ClrBit(CAN_CR1, TXREQ);             /* 清掉自检残留的发送请求 */
    printf("[ST] end CR0=%u CR1=%u P0=%u\r\n",
           (uint16)CAN_CR0, (uint16)CAN_CR1, (uint16)P0);
}

/* ==================== 【调试用】CAN 引脚 / 收发器 连通性探针 ====================
 * 原理：如果收发器确实接在 CANTX/CANRX 上，那么我们发帧时它会把总线状态回送到 RXD，
 *       于是采样 CANRX 能看到"显性=低"电平；若发帧期间 CANRX 一直是高，
 *       说明这两根脚上没有收发器（或收发器没供电/没使能）。
 *
 * 判读：
 *   txLow>0 且 rxLow>0  => 两脚 + 收发器都通，问题在总线/主机侧（接线、120Ω 终端、主机没发）
 *   txLow>0 且 rxLow=0  => 收发器不在这一对脚上（或没供电）
 *   txLow=0             => CANTX 引脚压根没被驱动
 *
 * 下面两个开关按探针结果翻：
 */
/* ★ txLow=0 时改成 1：给 P0.0 加输出使能再探一次 */
#define CAN_DEBUG_SET_P00_OE     0
/* ★ txLow>0 且 rxLow=0 时改成 1：换到 CAN_CH=1（P0.5/P0.6）再探一次，
 *   验证收发器是不是接在那一对脚上。注意 P0.5/P0.6 同时是调试串口，
 *   探针期间会临时摘掉 UART1 复用，探完恢复。*/
#define CAN_DEBUG_PROBE_CH1      0

/* 发一帧，同时高频采样：CANTX/CANRX 引脚上出现的"显性(低)"次数 + TXING 持续时间 */
static void CAN_SendAndSample(uint8 txMask, uint8 rxMask, uint16 *txLow, uint16 *rxLow, uint16 *txing)
{
    uint16 i;
    uint8  dat[8];

    memset(dat, 0x55, 8);               /* 0x55 = 显性/隐性交替，便于观察 */

    CanSend.CAN_IDE = IDE_EXT;
    CanSend.CAN_RTR = RTR_DAT;
    CanSend.CAN_DLC = CAN_FRAME_DATA_LEN;
    CanSend.EXTID   = 0x12345678UL;
    memcpy(&CanSend.Data0, dat, CAN_FRAME_DATA_LEN);

    *txLow = 0;
    *rxLow = 0;
    *txing = 0;

    CAN_Send();

    /* 采样约 10ms。一帧 8 字节 @250kbps 约 0.5ms，所以：
     *   正常"发一帧就完"          => txing 约占 5%~15%（几千分之几百）
     *   没人 ACK + 自动重传       => TXING 一直不清 => txing 接近 10000
     * 这个数直接说明"总线对端到底有没有在应答我们"。
     * （若分析仪是只听/静默模式，它不 ACK，这里就会是接近 100%。）*/
    for (i = 0; i < 10000; i++)
    {
        if ((P0 & txMask) == 0) (*txLow)++;
        if ((P0 & rxMask) == 0) (*rxLow)++;
        if (ReadBit(CAN_SR, TXING)) (*txing)++;
    }
}

void CAN_PinProbe(void)
{
    uint16 txLow, rxLow, txing;

#if CAN_DEBUG_SET_P00_OE
    SetBit(P0_OE, P00);                 /* 给 CANTX 加输出使能 */
#endif

    CAN_SendAndSample(0x01, 0x02, &txLow, &rxLow, &txing);
    printf("[PB] P0.0/P0.1 txLow=%u rxLow=%u txing=%u P0=%u\r\n",
           txLow, rxLow, txing, (uint16)P0);
    printf("[PB] SR=%u TEC=%u REC=%u IFR=%u\r\n",
           (uint16)CAN_SR, (uint16)CAN_TEC, (uint16)CAN_REC, (uint16)CAN_IFR);

#if CAN_DEBUG_PROBE_CH1
    /* 换到 P0.5(TXD)/P0.6(RXD) 再探一次：先把调试串口从这两脚上摘下来 */
    ClrBit(PH_SEL, UART1EN);
    ClrBit(CAN_CR0, CANEN);
    SetBit(CAN_CR0, CAN_CH);
    SetBit(P0_PU, P06);                 /* RX 上拉，避免悬空 */
    SetBit(CAN_CR0, CANEN);

    CAN_SendAndSample(0x20, 0x40, &txLow, &rxLow, &txing);

    /* 恢复：CAN 回 P0.0/P0.1，串口复用还回去 */
    ClrBit(CAN_CR0, CANEN);
    ClrBit(CAN_CR0, CAN_CH);
    SetBit(CAN_CR0, CANEN);
    SetBit(PH_SEL, UART1EN);

    printf("[PB] P0.5/P0.6 txLow=%u rxLow=%u txing=%u P0=%u\r\n",
           txLow, rxLow, txing, (uint16)P0);
#endif
}

/* ==================== 【调试】CAN 变量观测帧 ====================
 * 帧格式：标准帧，ID = 0x100 + 页号，DLC = 8
 *         Byte0-1 = 第1个量   Byte2-3 = 第2个量   Byte4-5   Byte6-7
 *         每个量 16 位，高字节在前（与工程里 EXT_SET_U16 一致）
 *         上位机按 ID 区分页，按字节位置对号入座即可（要直观就配个 DBC/信号表）
 *
 *  要观测什么量，就改 can_dbg_build() 的 case 
 *    换一个量 → 把某个 EXT_SET_U16 的参数换掉
 *    加一页    → 复制一个 case，页号 +1，同时把 CAN_DBG_PAGES 加 1
 *    32 位的量 → 拆成 (uint16)(v >> 16) 和 (uint16)v 占两个量位
 */
#define CAN_DBG_PAGES   3

static uint8 xdata g_dbgDat[CAN_FRAME_DATA_LEN];
static uint8 xdata g_dbgPage = 0;      /* 0=空闲，1..CAN_DBG_PAGES=当前待发页 */
static uint16 xdata g_dbgTick = 0;

static void can_dbg_build(uint8 page)
{
    memset(g_dbgDat, 0, CAN_FRAME_DATA_LEN);

    switch (page)
    {
        case 1:     /* ID 0x101：状态机 + 角度 */
            EXT_SET_U16(g_dbgDat[0], g_dbgDat[1], (uint16)mcState);
            EXT_SET_U16(g_dbgDat[2], g_dbgDat[3], (uint16)mcFocCtrl.CtrlMode);
            EXT_SET_U16(g_dbgDat[4], g_dbgDat[5], (uint16)FOC__THETA);
            EXT_SET_U16(g_dbgDat[6], g_dbgDat[7], (uint16)FOC__ETHETA);
            break;

        case 2:     /* ID 0x102：速度环 */
            EXT_SET_U16(g_dbgDat[0], g_dbgDat[1], (uint16)mcFocCtrl.Ref);
            EXT_SET_U16(g_dbgDat[2], g_dbgDat[3], (uint16)mcFocCtrl.SpeedFlt);
            EXT_SET_U16(g_dbgDat[4], g_dbgDat[5], (uint16)FOC__EOME);
            EXT_SET_U16(g_dbgDat[6], g_dbgDat[7], (uint16)FOC__IQ);
            break;

        case 3:     /* ID 0x103：母线 / NTC / 故障 */
            EXT_SET_U16(g_dbgDat[0], g_dbgDat[1], (uint16)mcFocCtrl.mcDcbusFlt);
            EXT_SET_U16(g_dbgDat[2], g_dbgDat[3], (uint16)mcFocCtrl.NTCTempNum);
            EXT_SET_U16(g_dbgDat[4], g_dbgDat[5], (uint16)mcFaultSource);
            EXT_SET_U16(g_dbgDat[6], g_dbgDat[7], (uint16)Can.Data0);
            break;

        default:
            break;
    }
}

void can_dbg_task(void)                 /* 1ms */
{
    if (g_dbgPage != 0) return;         /* 上一轮还没发完，不重开 */
    if (++g_dbgTick >= CAN_DEBUG_PERIOD_MS)
    {
        g_dbgTick = 0;
        g_dbgPage = 1;
    }
}

void can_dbg_poll(void)                 /* 1ms：每毫秒最多 1 帧，TXING 忙就跳过，绝不阻塞 */
{
    if (g_dbgPage == 0) return;
    if (ReadBit(CAN_SR, TXING)) return;

    can_dbg_build(g_dbgPage);
    can_hw_send((uint32)(0x100 + g_dbgPage), g_dbgDat, CAN_FRAME_DATA_LEN);

    g_dbgPage++;
    if (g_dbgPage > CAN_DBG_PAGES) g_dbgPage = 0;
}
