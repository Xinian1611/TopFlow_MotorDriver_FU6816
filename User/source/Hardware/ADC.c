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
     /* AD0 = P2.0：新板该脚与 P4.6 相连，本工程不使用 → 不配模拟、不使能通道 */
    SetBit(P2_AN, PIN3);    //AD1  P2.3  抱闸电磁铁通/断电检测
    SetBit(P2_AN, PIN4);    //AD2  P2.4  NTC 温度分压
    SetBit(P2_AN, PIN5);    //AD3  P2.5  霍尔信号采集（配模拟输入，通道在下面使能）
    ClrBit(P2_OE, PIN5);    //霍尔脚按输入用
    SetBit(P2_AN, PIN7);    //AD4  P2.7  单电阻母线电流（AMP0O）
    /* AD7 = P3.4：新板未使用 → 不配 */
    SetBit(P2_AN , PIN6);   //AD11 P2.6  I_BUS（AMP0O 经 10k + 1uF 滤波）
    /* AD6 = P3.3：OVP 输入电源（PWR+）分压检测，网络标名 OVP */
    SetBit(P3_AN, PIN3);
    ClrBit(P3_OE, PIN3);    //输入脚
    /* AD15 是内部 VCC 通道（VOLTAGE_MODE = INTERNAL 时 FOC 用它测母线），不需要配引脚 */
    /* P2.3 是三重复用脚（AD1 / AMP2O / DA1）：必须禁止 DA1 输出到 PAD，
         否则 DA1 会顶掉抱闸电磁铁的状态检测 */
    ClrBit(P2_OE , PIN3);
    /****************************************************************/
    //通道使能：AD1 抱闸 / AD2 NTC / AD3 霍尔 / AD4 母线电流 / AD6 OVP / AD15 内部VCC
    SetBit(ADC_MASK, CH1EN | CH2EN | CH3EN | CH4EN | CH6EN | CH11EN | CH15EN);   

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

