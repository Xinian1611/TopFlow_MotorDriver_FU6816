/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      xxx.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2022-08-05
 * @date      Last modify:2022-08-05
 * @note      Last modify author is Leo.li
 * @brief
 */
 
#include <UART.h>


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
    SetBit(PH_SEL , UART2EN);   //P3[6]as UART2_RXD; P3[7]as UART2_TXD

#if 0
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
    SetBit(PH_SEL , UART1EN);   //P0[6]as UART2_RXD; P0[5]as UART2_TXD

#if 0
    ClrBit(P0_OE , P06);        //输入使能
    SetBit(P0_PU , P06);        //上拉电阻
    SetBit(P0_OE , P05);        //输出使能            
    SetBit(P0_PU , P05);        //上拉电阻 	
#endif

    UT_MOD1 = 0;	//00-->单线制8bit		01-->8bit uart(波特率可设置)
    UT_MOD0 = 1;	//10-->单线制9bit	    11-->9bit uart(波特率可设置)
    SM2 = 0;        //0-->单机通讯			1-->多机通讯；
    REN = 1;        //0-->不允许串行输入	1-->允许串行输入，软件清0;
    TB8 = 0;	    //模式2/3下数据发送第9位，在多机通信中，可用于判断当前数据帧的数据是地址还是数据，TB8=0为数据，TB8=1为地址
    RB8 = 0;	    //模式2/3下数据接收第9位，若SM2=0,作为停止位

    ClrBit(IP3 , PI2C_UT11);	//中断优先级时最低
    ClrBit(IP3 , PI2C_UT10);	//中断优先级时最低    

    ClrBit(UT_BAUD , UART_2xBAUD);  //倍频使能0-->Disable  1-->Enable
    ES0 = 0;                        //UART1中断使能0-->Disable  1-->Enable
    UT_BAUD = 0x000c;//波特率可设置 = 24000000/(16/(1+ UT_BAUD[BAUD_SEL]))/(UT_BAUD+1)
                     //9B-->9600 0x000c-->115200
}

void put_char(unsigned char c)
{
    UT2_DR = c;
    while(!ReadBit(UT2_CR , UT2TI));
    ClrBit(UT2_CR , UT2TI);
}

void put_string(unsigned char *str)
{
    while(*str != 0)
    {
        put_char(*str);
        str++;
    }
}
