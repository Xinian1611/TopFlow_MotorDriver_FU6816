/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      TSD.c
 * @author    Fortiortech  Appliction Team
 * @date      2022-07-13
 * @brief     This file contains TSD function used for Motor Control.
 */

#include <TSD.h>

uint8 Micro_TJ[] = { 70, 71, 72, 74, 75, 76, 77, 79, 80, 81,
	                 82, 83, 84, 86, 87, 88, 89, 91, 92, 93,
	                 95, 96, 97, 99, 100,101,102,104,105,107,
	                 108,110,111,112,114,115,117,118,120,121,
	                 123,124,126,128,129,131,132,134,136,137,
	                 139,141,142,144,146,148,149,151,152,154,
	                 156,158,160
                   };

/**
 * @brief        TSD初始化函数
 * @param        None
 * @return       none
 * @date         2022-09-13
 * @version      1.0
 * @property     Public
*/
void TSD_Init(void)
{
    SetBit(TSD_CR, TSDEN);        //温度保护使能
    SetBit(TSD_CR, TSEN_EN);        //温度传感器使能
}

/**
 * @brief        TSD芯片内部温度
 * @param        None
 * @return       none
 * @date         2023-
}3-24
 * @version      1.0
 * @property     Public
*/
uint8 TSD_Gain(void)
{
    uint8 SetTJ_Value = 0;
	uint8 TJ_Value = TSEN_DR;
	
    SetTJ_Value = Micro_TJ[TJ_Value];
	
    return SetTJ_Value;
}