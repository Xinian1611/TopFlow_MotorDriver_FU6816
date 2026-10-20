/* *************************** (C) COPYRIGHT 2015 Fortiortech shenzhen *****************************
    File Name          : BuzzerScan.c
    Author             : Billy Long Fortiortech  Market Dept
    Version            : V1.0
    Date               : 01/07/2015
    Description        : This file contains Buzzer Scan function used for Motor Control.
***************************************************************************************************
    All Rights Reserved
**************************************************************************************************/


/* Includes -------------------------------------------------------------------------------------*/

#include <FU68xx_4.h>
#include <MyProject.h>

/* Private variables ----------------------------------------------------------------------------*/
BZScan_TypeDef xdata BZScan;

/*  -------------------------------------------------------------------------------------------------
    Function Name : void BuzzerInit(void)
    Description   : 按键参数初始化
    Input         : 无
    Output        : 无
    -------------------------------------------------------------------------------------------------*/
void BuzzerInit(void)
{
    BZScan.FlagBZ = 0;
    BZScan.BZRunTime = 0;
    BZScan.BZCnt = 0;
    BZScan.BZMultitimes = 0;
    BZScan.BZMultitimesCount = 0;
}

/*  -------------------------------------------------------------------------------------------------
    Function Name : void SetBuzzer(uint8 Length)
    Description   : 置位蜂鸣器鸣叫标志
    Input         : Length：响铃长度  单位：100ms
    Output        : 无
    -------------------------------------------------------------------------------------------------*/
void SetBuzzer(uint8 BZRunTime, uint8 BZMultitimes)
{
    BZScan.FlagBZ = 1;
    BZScan.BZRunTime = BZRunTime;
    BZScan.BZMultitimes = BZMultitimes;
}


/*  ----------------------------------------------------------------------------------------------*/
/*  Function Name  : BuzzerScan
    /*  Description    : 蜂鸣器，BZScan.BZRunTime控制时间长度
    /*  Date           : 2020-03-25
    /*  Parameter      : None
    /*  ----------------------------------------------------------------------------------------------*/
void BuzzerScan(void)
{
    if (BZScan.BZMultitimes > 0)
    {
        BZScan.BZMultitimesCount++;
        
        if (BZScan.BZMultitimesCount % 200 == 0)
        {
            BZScan.BZMultitimesCount = 0;
            BZScan.FlagBZ = 1;
        }
    }
    else
    {
        BZScan.FlagBZ = 0;
        BZScan.BZMultitimesCount = 0;
    }
    
    if (BZScan.FlagBZ && BZScan.BZMultitimes)
    {
        SetBit(TIM3_CR1, T3EN);
        //        SetBit(TIM4_CR1, T4EN);
        //        SetBit(PH_SEL, T4SEL);
        BZScan.BZCnt ++;
        
        if (BZScan.BZCnt >= (100 * BZScan.BZRunTime))
        {
            BZScan.BZCnt = 0;
            BZScan.FlagBZ = 0;
            ClrBit(TIM3_CR1, T3EN);
            ResetBUZZERPin;
            //            ClrBit(TIM4_CR1, T4EN);
            //            ClrBit(PH_SEL, T4SEL);
            //            ResetBUZZERPin;
            //            BZScan.BZMultitimes = 0;
            BZScan.BZMultitimes--;
        }
    }
}
