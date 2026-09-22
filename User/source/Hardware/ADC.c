/**
 * @copyright (C) COPYRIGHT 2022 Fortiortech Shenzhen
 * @file      xxx.c
 * @author    Fortiortech  Appliction Team
 * @since     Create:2022-08-05
 * @date      Last modify:2022-08-05
 * @note      Last modify author is Leo.li
 * @brief
 */
#include <MyProject.h>

#include <ADC.h>

/**
 * @brief        ADC初始化
 * @date         2022-09-13
 */
void ADC_Init(void)
{
    /********************ADC 端口模拟功能设置************************/
    SetBit(P2_AN, PIN0);    //AD0 P20 CH open--IU--固定
    SetBit(P2_AN, PIN3);    //AD1 P23 CH open--IV--固定
    SetBit(P2_AN, PIN4);    //AD2 P24 CH open--IV--固定
    SetBit(P2_AN, PIN5);    //AD3 P25 CH open
    SetBit(P2_AN, PIN7);    //AD4 P27 CH open--IBUS/IW--固定
    //  SetBit(P3_AN , PIN2);   //AD5 P32 CH open
    //  SetBit(P3_AN , PIN3);   //AD6 P33 CH open
    SetBit(P3_AN, PIN4);    //AD7 P34 CH open
    //  SetBit(P2_AN , PIN1);   //AD8 P21 CH open
    //  SetBit(P1_AN , PIN6);   //AD9 P16 CH open
    //  SetBit(P1_AN , PIN4);   //AD10 P14 CH open
    SetBit(P2_AN , PIN6);   //AD11 P26 CH open
    //  SetBit(P1_AN , PIN3);   //AD12 P13 CH open
    //  SetBit(P1_AN , PIN5);   //AD13 P15 CH open
    /****************************************************************/
    SetBit(ADC_MASK, CH0EN | CH1EN | CH2EN | CH3EN | CH4EN | CH7EN  | CH11EN| CH15EN);   //通道使能
    #if(VOLTAGE_MODE == INTERNAL)
    {
        #if(VOLTAGE_SCALE == Ratio_6_5)
        SetBit(ADC_CR, ADCRATIO);           //
        #else
        ClrBit(ADC_CR, ADCRATIO);
        #endif
    }
    #endif
    SetBit(ADC_CR, ADCALIGN);       //ADC数据次高位对齐使能0-->Disable 1-->Enable
    ClrBit(ADC_CR, ADCIE);          //ADC中断使能
    SetBit(ADC_CR, ADCEN);          //Enable ADC0
}

