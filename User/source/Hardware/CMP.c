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

void CMP0_Init(void)
{

}

/*****************************************************************************
 * Function      : CMP3_Init
 * Description   : CMP3_INIT
 * Input         : void  
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20190320
 *   Author      : Bruce HW&RD
 *   Modification: Created function

*****************************************************************************/


void CMP3_Init(void)
{
   
    /******CMP3 端口模拟功能设置*******/    
    #if (Shunt_Resistor_Mode == Single_Resistor)
    {
        SetBit(P2_AN , P27);	        //CMP3 Pin设置为模拟模式  + 
        ClrBit(CMP_CR1 , CMP3MOD1);     //00-->P27-单比较器模式    01-->P20/P23-双比较器模式
        ClrBit(CMP_CR1 , CMP3MOD0);     //1X-->P20/P23/P27-三比较器模式
    }
    #elif (Shunt_Resistor_Mode == Double_Resistor)
    {
        SetBit(P2_AN , P27);	        //CMP3 Pin设置为模拟模式  + 
        ClrBit(CMP_CR1 , CMP3MOD1);     //00-->P27-单比较器模式    01-->P20/P23-双比较器模式
        ClrBit(CMP_CR1 , CMP3MOD0);     //1X-->P20/P23/P27-三比较器模式
    }
    #elif (Shunt_Resistor_Mode == Three_Resistor)
    {
        SetBit(P2_AN , P27 | P23 | P20);// CMP3 Pin设置为模拟模式  + 
        SetBit(CMP_CR1 , CMP3MOD1);     // 00-->P27-单比较器模式    01-->P20/P23-双比较器模式
        SetBit(CMP_CR1 , CMP3MOD0);     // 1X-->P20/P23/P27-三比较器模式
    }
    #endif	//end Shunt_Resistor_Mode
    
    #if (COMPARE_MODE == COMPARE_HW_MODE)
    {
        /**P2.6使能其模拟功能，使能数字输出**/
        SetBit(P2_AN , P26);
        ClrBit(P2_OE , P26); 
        ClrBit(DAC_CR , DAC0_1EN);
    }
    #else
    {
//        /**P2.6使能其模拟功能，使能数字输出**/
//        SetBit(P2_AN , P26);
//        SetBit(P2_OE , P26); 
        /* DAC0 Enable */
        SetBit(DAC_CR , DAC0_1EN);        
        /******************************
        0: 正常模式，DAC输出电压范围为0到VREF
        1: 半电压转换模式，DAC输出电压范围为VHALF到VREF
        ****************************/
        ClrBit(DAC_CR , DACMOD);
        /**********设置DAC过流值,舍弃低位精度*****************/
        
        //  DAC0_DR 是DAC0的高8位,DAC0 的最低位忽略
        DAC0_DR     =   DAC_OverCurrentValue;
    }
    #endif	//end Compare_Mode
    
    SetBit(CMP_CR1 , CMP3HYS);  // CMP3 Hysteresis voltage Enable
    /*---------------------------------------------------------------------------------
	选择母线电流保护触发信号源，外部中断0或者比较器3中断。
	0-CMP3,1-INT0
    ---------------------------------------------------------------------------------*/
    ClrBit(EVT_FILT, INT0_MOE_EN);
    
    
    /*---------------------------------------------------------------------------------
	触发硬件保护后硬件关闭驱动输出MOE配置， 常规应用需要使能MOE自动清零
	00--MOE不自动清零
	01--MOE自动清零
    ----------------------------------------------------------------------------------*/
    ClrBit(EVT_FILT , MOEMD1);
    SetBit(EVT_FILT , MOEMD0);
    /*----------------------------------------------------------------------------------
    
    
    母线电流保护时间滤波宽度
    00-不滤波    
    01-4cpu clock    
    10-8cpu clock    
    11-16cpu clock
    -----------------------------------------------------------------------------------*/
    SetBit(EVT_FILT , EFDIV1);
    SetBit(EVT_FILT , EFDIV0);

    SetBit(CMP_CR1 , CMP3EN);   //CMP3 Enable
}

/*****************************************************************************
 * Function      : CMP3_Interrupt_Init
 * Description   : CMP3中断配置
 * Input         : void  
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20190320
 *   Author      : Bruce HW&RD
 *   Modification: Created function

*****************************************************************************/
void CMP3_Interrupt_Init(void)
{
    ClrBit(CMP_SR , CMP3IF);
    /*------------------------------------------------------------------------
    比较器中断模式配置
    00: 不产生中断  
    01: 上升沿产生中断  
    10: 下降沿产生中断  
    11: 上升/下降沿产生中断
    ------------------------------------------------------------------------*/
    ClrBit(CMP_CR0 , CMP3IM1);
    SetBit(CMP_CR0 , CMP3IM0);
    
    SetBit(IP3 , PCMP31);    // 中断优先级别3
    SetBit(IP3 , PCMP30);
 				
}



