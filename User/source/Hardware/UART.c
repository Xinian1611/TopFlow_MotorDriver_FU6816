/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      xxx.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2022-08-05
 * @date      Last modify:2022-08-05
 * @note      Last modify author is Leo.li
 * @brief
 */
 
#include <definition.h>            /* 新增 */
#include <Customer.h>              /* 新增：DEBUG_UART_OR_CAN */
#include <CAN.h>                   /* 新增：模式2 需要 can_hw_send / CAN_SR / TXING */
#include <UART.h>

#define             USART1_INIT    0
#define             USART2_INIT    1
#define             USART_INIT     USART1_INIT

/* ==================== CAN 调试文本通道（DEBUG_UART_OR_CAN == 2 时启用）====================
 * printf 的字符先进 8 字节小缓冲，满 8 字节或遇到 '\n' 就发一帧标准帧 ID=CAN_DEBUG_TEXT_ID。
 * 发送是“忙就丢”的非阻塞写法：CAN_SR.TXING 还高就整帧丢掉并计数，绝不进 CAN_Send 里的
 * 超时等待 —— 否则总线对端不 ACK 时每帧都要空转上万次循环，主循环和 1ms 节拍会被拖垮。
 * 纪律：printf 只能在主循环里调，任何中断里都不能进这里。
 */
#if (DEBUG_UART_OR_CAN == 2)
static uint8    xdata g_dbgTxBuf[8];
static uint8    xdata g_dbgTxLen = 0;
volatile uint16 xdata g_dbgTxDrop = 0;

static void DebugCanFlush(void)
{
    if (g_dbgTxLen == 0)
    {
        return;
    }

    while (g_dbgTxLen < 8)                       /* 补 0 对齐 8 字节 */
    {
        g_dbgTxBuf[g_dbgTxLen++] = 0x00;
    }

    if (ReadBit(CAN_SR, TXING))                  /* 上一帧还没发完 → 丢这一帧，不阻塞 */
    {
        g_dbgTxDrop++;
    }
    else
    {
        (void)can_hw_send(CAN_DEBUG_TEXT_ID, g_dbgTxBuf, 8);
    }

    g_dbgTxLen = 0;
}
#endif


/** 
 * @brief        Uart2初始化函数
 * @param        None
 * @return       none
 * @author       Marcel
 * @date         2022-07-13
 * @version      1.0   
 * @property     Public
*/
void UART2_Init(void)
{
    SetBit(PH_SEL , UART2EN);   /* 端口复用为 UART2 */

    /*  新增：UART2 功能转移，TXD2/RXD2 由 P3.7/P3.6 挪到 P0.0/P0.1
       PH_SEL1.UART2CH1 = 1 → P37-->P00(TXD2S)   P36-->P01(RXD2S)
       见 FU68xx_6_MCU.h:1251  */
    SetBit(PH_SEL1, UART2CH1);

    /*  新增：RXD2(P0.1) 配成输入并软件上拉，避免悬空误触发起始位；
       TXD2(P0.0) 由 UART2 外设驱动，不用配输出使能 */
    ClrBit(P0_OE , P01);
    SetBit(P0_PU , P01);
    SetBit(P0_OE , P00);        /* ★ TXD2 = P0.0 输出使能（对照 dir_usart.c:74） */

#if 0   /* P3.6/P3.7 已不用（板上接外部晶振），保留作参考 */
    ClrBit(P3_OE , P36);        //输入使能
    SetBit(P3_PU , P36);        //上拉电阻
    SetBit(P3_OE , P37);        //输出使能            
    SetBit(P3_PU , P37);        //上拉电阻 	
#endif

    ClrBit(UT2_CR , UT2MOD1);	//00-->单线制8bit		01-->8bit uart(波特率可设置)
    SetBit(UT2_CR , UT2MOD0);	//10-->单线制9bit	    11-->9bit uart(波特率可设置)

    ClrBit(UT2_CR , UT2SM2);    //0-->单机通讯			1-->多机通讯；
    SetBit(UT2_CR , UT2REN);    //0-->不允许串行输入	1-->允许串行输入，软件清0;
    ClrBit(UT2_CR , UT2TB8);	//模式2/3下数据发送第9位，在多机通信中，可用于判断当前数据帧的数据是地址还是数据，TB8=0为数据，TB8=1为地址
    ClrBit(UT2_CR , UT2RB8);	//模式2/3下数据接收第9位，若SM2=0,作为停止位

    ClrBit(IP3 , PSPI_UT21);	//中断优先级时最低
    ClrBit(IP3 , PSPI_UT20);	//中断优先级时最低
//    PSPI_UT21 = 0;              //中断优先级时最低
//    PSPI_UT20 = 0;

    ClrBit(UT2_BAUD , BAUD2_SEL);//倍频使能0-->Disable  1-->Enable
//    ClrBit(UT2_BAUD , UART2CH);  //UART2端口功能转移使能0：P36->RXD P37->TXD 1:P01->RXD P00->TXD
    ClrBit(UT2_BAUD , UART2IEN); //UART2中断使能0-->Disable  1-->Enable
    UT2_BAUD = 0x000c;           //波特率可设置 = 24000000/(16/(1+ UT_BAUD[BAUD_SEL]))/(UT_BAUD+1)
                                 //9B-->9600 0x000c-->115200 0x0005-->256000
}

void UART1_Init(void)
{
    /* ---- ① 端口复用：UART1 从 P0.5(TXD)/P0.6(RXD) 转移到 P1.0(TXDS)/P1.1(RXDS) ---- */
    SetBit(PH_SEL , UART1EN);   /* UART1 端口复用总开关，转移后仍然要置 1 */

    /* ---- ② P1.0/P1.1 按普通数字口配置（这两脚默认是 LIN/高压脚）---- */
    ClrBit(P1_PU, P10HV_EN);    /* 关 P1.0 高压 PAD → 普通 TTL 电平 */
    ClrBit(P1_PU, P11HV_EN);    /* 关 P1.1 高压 PAD */
    SetBit(P1_OE, P10);         /* P1.0 = TXD → 输出使能（同 UART2_Init 对 P0.0 的写法）*/
    ClrBit(P1_OE, P11);         /* P1.1 = RXD → 输入 */

    /* ---- ③ UART1 本体：8bit UART，允许接收，不开中断（printf 轮询 TI）---- */
    UT_MOD1 = 0;                /* 00 单线8bit  01 8bit UART */
    UT_MOD0 = 1;
    SM2 = 0;
    REN = 1;
    TB8 = 0;
    RB8 = 0;

    ClrBit(IP3 , PI2C_UT11);    /* 中断优先级最低 */
    ClrBit(IP3 , PI2C_UT10);

    ClrBit(UT_BAUD , UART_2xBAUD);
    ES0 = 0;                    /* UART1 中断关：printf 只做查询发送 */
    /*  关键一句：bit12 UART_CH=1 → 端口转移；低 12 位 0x00C → 115200 8N1
       必须整字一次写！不能先 SetBit(UT_BAUD, UART_CH) 再写 0x000c，会被冲掉 */
    UT_BAUD = 0x100c;
}

// void put_char(unsigned char c)
// {
//     UT2_DR = c;
//     while(!ReadBit(UT2_CR , UT2TI));
//     ClrBit(UT2_CR , UT2TI);
// }
void put_char(unsigned char c)
{
#if(USART_INIT == 0)
    UT_DR = c;                              /* UART1 数据寄存器（原来写的是 UART2 的 UT2_DR）*/
    while(!ReadBit(UT_CR , TI));            /* UART1 发送完成标志：TI = UT_CR^1 */
    ClrBit(UT_CR , TI);
#elif(USART_INIT == 1)
    UT2TI = 0;                    /* 清 UART2 发送完成标志 */
    UT2_DR = c;                   /* UART2 数据寄存器 */
    while (UT2TI == 0);           /* 等这一字节发完：UT2TI = UT2_CR^1 */
    UT2TI = 0;
#endif
}

void put_string(unsigned char *str)
{
    while(*str != 0)
    {
        put_char(*str);
        str++;
    }
}

char putchar(char c)
{
#if (DEBUG_UART_OR_CAN == 1)
    /* 【模式1：UART 调试】UART1 = P0.5(TXD) / P0.6(RXD)，115200 8N1。
     * 每字符阻塞等 TI（约 87us/字符 @115200），调试期够用。
     * P0.5/P0.6 与 MT6701 的硬件 I2C 是同一对脚，用串口期间不要调 MT6701_Init()。*/
    #if(USART_INIT == 0)
        TI = 0;                             /* 清掉上一次的完成标志，作为本次发送起点 */
        UT_DR = (unsigned char)c;
        while (TI == 0)                     /* 等这一字节发完 */
        {
            ;
        }
        TI = 0;
    #elif(USART_INIT == 1)
        UT2TI = 0;
        UT2_DR = (unsigned char)c;
        while (UT2TI == 0)
        {
            ;
        }
        UT2TI = 0;
    #endif
#elif (DEBUG_UART_OR_CAN == 2)
    /* 【模式2：CAN 调试】printf → CAN 标准帧文本 */
    g_dbgTxBuf[g_dbgTxLen++] = (uint8)c;
    if ((g_dbgTxLen >= 8) || (c == '\n'))
    {
        DebugCanFlush();
    }
#else
    /* 【模式0：调试关闭】*/
    (void)c;
#endif

    return (c);
}
