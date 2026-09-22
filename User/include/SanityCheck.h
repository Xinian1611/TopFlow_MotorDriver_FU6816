/* --------------------------- (C) COPYRIGHT 2021 Fortiortech -------- -----------------------------
    File Name      : SanityCheck.h
    Author         : Fortiortech  Appliction Team
    Version        : V1.1
    Date           : 2021-12-02
    Description    : This file contains .H file function used for check Customer.h set.
----------------------------------------------------------------------------------------------------
                                       All Rights Reserved
------------------------------------------------------------------------------------------------- */
#ifndef __SANITYCHECK_H_
#define __SANITYCHECK_H_


    #if (PosCheckEnable==1)
        #error "data 20220616,RPD Program has not been added"
    #endif

//#if (DBG_MODE == DBG_UART)
//    #error "data 20220622, UART debug Program has not been added"
//#endif





    /* 驱动电平配置检查 */
    #if (PWM_LEVEL_MODE != HIGH_LEVEL && PWM_LEVEL_MODE != LOW_LEVEL && PWM_LEVEL_MODE != UP_H_DOWN_L && PWM_LEVEL_MODE != UP_L_DOWN_H)
        #error " PWM_LEVEL_MODE err.[Set: HIGH_LEVEL,LOW_LEVEL,UP_H_DOWN_L,UP_L_DOWN_H]"
    #endif

    /* VREF参考电压配置检查 */
    #if (HW_ADC_VREF == VREF3_0 || HW_ADC_VREF == VREF4_0 || HW_ADC_VREF == VREF4_5)
        #if (VREF_OUT_EN == 0)
          #error " VREF err.[Enable P3.5 AN function]"
        #endif
    #elif (HW_ADC_VREF == VREF5_0)

    #else
        #error " VREF error.[VREF can only be VREF3_0  VREF4_0  VREF4_5  VREF5_0] "  
    #endif

    /*  运放配置检查 */
    #if (HW_AMP_MODE == AMP_PGA_DUAL)
        #if (HW_AMPGAIN != AMP2x && HW_AMPGAIN != AMP4x && HW_AMPGAIN != AMP8x && HW_AMPGAIN != AMP16x)
            #error " Amp gain error.[Internal PGA gain can only be AMP2x  AMP4x  AMP8x  AMP16x] "  
        #endif
    #elif (HW_AMP_MODE == AMP_NOMAL)
    #else
        #error "Amp mode error.[Set: AMP_NOMAL or AMP_PGA_DUAL]"
    #endif
    
    /*  采样模式配置检查 */
    #if (Shunt_Resistor_Mode != Single_Resistor && Shunt_Resistor_Mode != Double_Resistor && Shunt_Resistor_Mode != Three_Resistor )
        #error " Shunt_Resistor_Mode error.[Set: Single_Resistor / Double_Resistor / Three_Resistor] "  
    #endif
 
    /*  偏置电压设置检查 */
    #if (HW_VHALF_SEL != VHALF1_8 && HW_VHALF_SEL != VHALF1_4 && HW_VHALF_SEL != VHALF25_64 && HW_VHALF_SEL != VHALF1_2)
        #error " HW_VHALF_SEL error.[Set: VHALF1_8 / VHALF1_4 / VHALF25_64 / VHALF1_2] "  
    #endif
 
    /*  预定位测试模式设置检查 */
    #if (ALIGN_MOME != ALIGN_DSIABLE && ALIGN_MOME != ALIGN_NOMAL && ALIGN_MOME !=ALIGN_TEST)
        #error " AlignTestMode error.[Set: Disable / Enable ] "  
    #endif
    
    
  
    
    /*  顺逆风设置 */
    #if (TAILWIND_MODE != NoTailWind && TAILWIND_MODE != RSDMethod && TAILWIND_MODE != BEMFMethod)
        #error " TailWind_Mode error.[Set: NoTailWind / RSDMethod / BEMFMethod] "  
    #endif
    

    /*  硬件过流值检测 */
//    #if (DAC_OverCurrentValue  > 0xff)
//        #error " DAC_OverCurrentValue error.[data overflow] "  
//    #endif



 
#endif