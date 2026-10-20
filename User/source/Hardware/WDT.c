/**
 * @copyright Copyright(C) 2023, Fortior Technology Co., Ltd. All rights reserved.
 * @file      WDT.c
 * @author    Fortiortech Application Team
 * @date      2023-06-27
 * @brief     This file contains WDT parameter used for WDT Control.
 */
 
 #include "FU68xx_6_MCU.h"
 #include "WDT_INIT.h"
 
 /**
 * @function     WatchDogConfig
 * @brief        看门狗初始化
 * @param[in]    Value:[输入]----定时时间，单位ms，最小定时时间8ms，最大定时时间1800ms
 * @return       None
 * @date         2023-06-27
*/
void WatchDogConfig(unsigned int Value)
{
	SetBit(CCFG1, WDT_EN);					//看门狗使能
	WDT_ARR = ((unsigned int)(65532 - (unsigned long int)Value * 32768 / 1000) >> 8);
	RST_SR = 0x80;
//	ClrBit(RST_SR, RSTWDT);					//看门狗复位标志清零
	SetBit(WDT_CR, WDTRF);					//看门狗初始化
}

 /**
 * @function     WatchDogRefresh
 * @brief        看门狗喂狗
 * @param[in]    None
 * @return       None
 * @date         2023-06-27
*/
void WatchDogRefresh(void)
{
	SetBit(WDT_CR, WDTRF);				//看门狗复位标志位
}