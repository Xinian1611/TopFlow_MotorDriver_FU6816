/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      xxx.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2022-08-05
 * @date      Last modify:2022-08-05
 * @note      Last modify author is Leo.li
 * @brief
 */
 
#include <Customer_Debug.h>
#include <SPI.h>




/** 
 * @brief        SPI初始化
 * @date         2022-09-13
 */
void SPI_Init(void)
{
    ClrBit(SPI_CR1, SPIEN);                                            // 0,disable SPI;1 enable
    /*  -------------------------------------------------------------------------------------------------
        SPI管脚配置
        1、禁止UART复用，P06配置为MISO，P05配置为MOSI
        2、禁止比较器输出复用，P07配置为SCLK
        -------------------------------------------------------------------------------------------------*/
    ClrBit(PH_SEL, UART1EN);                                              // 0,P06 as GPIO or SPI_MISO,P05 as GPIO or SPI_MOSI;1,P06 and p07 as USART
    /*  -------------------------------------------------------------------------------------------------
        SPI时钟相位/极性配置
        CPHA = 0, CPOL = 0:上升沿接收，下降沿发送，空闲电平为低
        CPHA = 0, CPOL = 1:上升沿发送，下降沿接收，空闲电平为高
        CPHA = 1, CPOL = 0:上升沿发送，下降沿接收，空闲电平为低
        CPHA = 1, CPOL = 1:上升沿接收，下降沿发送，空闲电平为高
        -------------------------------------------------------------------------------------------------*/
    SetReg(SPI_CR0, CPHA | CPOL, CPOL);
 
// 三线模式，MCU的 SCLK NSS MOSI 接SPI小板的 SCLK NSS MOSI，    SPI拨码开关：SW2 = 0;SW3 = 0;    
// 双线模式，MCU的 SCLK MOSI     接SPI小板的 SCLK  MOSI，       SPI拨码开关：SW2 = 1;SW2 = 0;   
// 单线模式，MCU的 SCLK          接SPI小板的 MOSI，             SPI拨码开关：SW2 = 1;SW3 = 1;   
    
    #if (SPI_DBG_MODE == DBG_SPI_1)
    {
        ClrBit(SPI_CR1,NSSMOD1);
        SetBit(SPI_CR1,NSSMOD0);
		
		//SetBit(PH_SEL1 , SPICT0);                         //单线制SPI功能转移(P05-->P00)
    }
    #elif (SPI_DBG_MODE == DBG_SPI_2)
    {
        ClrBit(SPI_CR1,NSSMOD1);
        ClrBit(SPI_CR1,NSSMOD0);        
    }
    #else
    {
        SetBit(SPI_CR1,NSSMOD1);
        SetBit(SPI_CR1,NSSMOD0);
    }
    #endif
        
    
  
    SetBit(SPI_CR0, SPIMS);                          // 0:Slave, 1:Master
    /*  -------------------------------------------------------------------------------------------------
        SPI中断配置
        SPIF：SPI字节传输完成中断标志，硬件置位，软件清零
        WCOL：SPI写冲突中断（发送缓冲器非空时写操作），硬件置位，软件清零
        MODF：SPI方式错误中断（多主方式NSS被拉低，MSTEN和SPIEN被清零）硬件置位，软件清零
        RXOVRN：SPI接收溢出中断（SPI接收缓冲器接收溢出），硬件置位，软件清零
        -------------------------------------------------------------------------------------------------*/
    SetReg(SPI_CR1, SPIIF | WCOL | MODF | RXOVRN, 0x00);           // SPI所有中断清除
    SPIIE = 0;                                                     // SPI中断使能
    SPI_CLK = 1;                                                   // Fspi = Fcpu / (2*(SPI_CLK + 1)) = 6MHz
    SetBit(SPI_CR1, SPIEN);                                        // enable SPI
}


